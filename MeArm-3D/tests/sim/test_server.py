# -*- coding: utf-8 -*-
"""Phase 6 验收：无头设备服务（`server.py`）的协议一致性。

判据全部对着 **Go 侧的协议实现** `backend/internal/protocol/protocol.go`：

  * `EncodeJR` 只保留 **1 位小数** ⇒ 文本协议的固有量化，往返偏差 ≤0.05°，
    这是**预期行为**（不是 bug），验收断言按此设容差；
  * `EncodeOKJR` 按**通道降序**输出 `S<n>=%.2f`；
  * `EncodeState` 按 `JointOrder` 位次、保留 2 位小数；
  * `EncodeServoReport` 输出 `# SERVO S9=… S8=… S7=… S6=…`（通道降序，2 位小数）。

⚠️ 三条链路的**上报语义**（Go `ParseReply` 的分类）必须记准，别凭印象：

  * `OK JR`                  → `ReplyOKJR`，携带**目标角**（核对标定用）
  * `STATE <四个数值>`        → `ReplyState`，携带**实际关节角** ⇒ 命令引起
  * `# … S<n>=…` / `STATUS …` → `ReplyServo`，携带**实际舵机角** ⇒ 设备侧自主变化
  * `OK SET …` / `OK JOY …`   → `ReplyOther`（**刻意**不入白名单：它们含 `S<n>=`，
                               但那是**钳位后的目标角**。收下就会"状态恒等于命令、误差恒为 0"）

★ 这里有一条**曾经写反、现已修正**的登记项：本文件早期写着「`STATUS` 回执 Go 侧
根本不解析（落到 `ReplyOther`）」。那是引入 `ReplyServo` 白名单之前的旧事实。
现在的白名单是 `#` 与 `STATUS` 两个前缀 —— 因此 `STATUS` **会被**识别成
`ReplyServo`（= 实际舵机角 ⇒ 界面跟随）。真机链路上这条不会生效，因为
`serial.go` 的 `execStatus` 会先把它翻译成 `STATE`；那个**不对称**是已知的，
登记在 `docs/serial-v1.md`。
"""
from __future__ import annotations

import io
import re
import subprocess
import sys
import time

import pytest

from conftest import SIM_DIR
from server import Emitter, MujocoDevice

# ⚠️ 这里**刻意不再**写 `ORDER = ("base", "shoulder", "elbow", "gripper")`。
#
# 曾经有那一行，注释还写着"与 robot.yaml 的 JointOrder 一致" —— 那正是
# 本项目最忌讳的**第二份真值**：线序的真值只有 `robot.yaml` 一份，
# 抄到测试里之后，改关节顺序会让这条测试以"解析错位"的面目失败，
# 而真正的错因是测试自己过期了。现在统一由 `dev.robot.joint_order()` 派生。


# ---------------------------------------------------------------------------
# 辅助
# ---------------------------------------------------------------------------


class Capture:
    """把设备的输出收进一个 StringIO，并提供"取最后一行"的便利。"""

    def __init__(self) -> None:
        self.buf = io.StringIO()
        self.emitter = Emitter(self.buf)

    def lines(self) -> list[str]:
        return [ln for ln in self.buf.getvalue().splitlines() if ln]

    def last(self) -> str:
        ln = self.lines()
        assert ln, "设备没有任何输出"
        return ln[-1]

    def clear(self) -> None:
        self.buf.seek(0)
        self.buf.truncate(0)


@pytest.fixture
def dev():
    return make_dev()


def make_dev() -> MujocoDevice:
    """造一台设备 + 它的输出捕获器（需要"两台设备对照"的测试直接调用）。"""
    cap = Capture()
    d = MujocoDevice(out=cap.emitter)
    d.capture = cap            # type: ignore[attr-defined]
    return d


def last_line(dev) -> str:
    return dev.capture.last()


# ---------------------------------------------------------------------------
# 基本命令
# ---------------------------------------------------------------------------


def test_ping(dev):
    dev.handle("PING")
    assert last_line(dev) == "OK PING"


def test_status_at_home_is_four_servos_at_90(dev):
    """HOME 位 = 四个舵机恰好 90°（这是 robot.yaml 标定自洽性的直接体现）。"""
    dev.handle("STATUS")
    assert last_line(dev) == "STATUS S9=90 S7=90 S8=90 S6=90"


def test_unknown_command(dev):
    dev.handle("FOO bar")
    assert last_line(dev) == "ERR UNKNOWN FOO bar"


def test_blank_line_is_ignored(dev):
    dev.handle("   ")
    assert dev.capture.lines() == []


# ---------------------------------------------------------------------------
# JR 受理
# ---------------------------------------------------------------------------


def test_jr_ok_matches_go_encoding(dev):
    """`OK JR` 必须与 Go 的 `EncodeOKJR` 逐字节一致（通道降序 + 2 位小数）。"""
    joints = {"base": 0.0, "shoulder": 20.0, "elbow": 130.0, "gripper": 50.0}
    dev.handle("JR 0 20 130 50")

    expect = {a.channel: a.joint_to_servo(joints[a.joint_id]) for a in dev.robot.actuators}
    want = " ".join(["OK", "JR"] + [f"S{c}={expect[c]:.2f}"
                                    for c in sorted(expect, reverse=True)])
    assert last_line(dev) == want
    # 顺带把"降序"这条独立钉住（否则上面那条在只有 1 个通道时也会过）
    assert last_line(dev).split()[2:] == ["S9=90.00", "S8=48.39", "S7=117.58", "S6=90.00"]


def test_jr_rejects_joint_limit_with_go_error_text(dev):
    """越限必须回 `ERR JOINT <id> <v> (limit <min>..<max>)` —— 与 Go/固件同文案。

    ⚠️ 期望值**从 robot.yaml 派生**（`dev.robot.joint("elbow")`），不写死字面量。
    此前这里硬编码 `108.44..141.86`，等于把限位真值抄了第二份 —— 重新标定之后
    这条会以"文案不对"的面目失败，而真正的错因是**测试自己过期了**。

    用 `fullmatch` 而不是 `startswith`：这样"两位小数"「`..`」这些**格式契约**
    仍然被钉住（只比数值的话，把格式从 `%.2f` 改成 `%g` 就没人拦得住了）。
    """
    el = dev.robot.joint("elbow")
    dev.handle("JR 0 20 90 50")          # elbow=90 < 下限
    line = last_line(dev)
    assert re.fullmatch(
        rf"ERR JOINT elbow 90\.00 \(limit {el.limit_min:.2f}\.\.{el.limit_max:.2f}\)", line
    ), line


def test_jr_rejects_wrong_arity(dev):
    dev.handle("JR 0 20 130")
    assert last_line(dev).startswith("ERR ARG")


def test_jr_rejects_non_numeric(dev):
    dev.handle("JR a b c d")
    assert last_line(dev).startswith("ERR ARG")


def test_joint_and_servo_limits_are_equivalent(dev):
    """★ 两层限位**完全等价** —— 关节限位换算到舵机后恰好落在舵机限位边界上。

    这是标定自洽性的硬证据（spec §14 要求四方限位一致）：
    把每个关节的 limit_min/max 用 `joint_to_servo` 送过去，得到的值必须**正好等于**
    该舵机的 servo_min/max。若有人改了其中一边，这条立刻变红。

    副作用（值得知道）：正因为等价，**真配置下构造不出"关节入限但舵机出限"的位形** ——
    所以下面那条 `ERR SERVO` 测试只能靠临时收紧 servo_min 来触发。
    """
    for a in dev.robot.actuators:
        j = dev.robot.joint(a.joint_id)
        lo = a.joint_to_servo(j.limit_min)
        hi = a.joint_to_servo(j.limit_max)
        got = sorted((lo, hi))
        want = sorted((a.servo_min, a.servo_max))
        assert got == pytest.approx(want, abs=1e-6), (
            f"S{a.channel}({a.joint_id}) 关节限位 {j.limit_min}..{j.limit_max} "
            f"换算到舵机是 {got}，但舵机限位是 {want}")


def test_jr_rejects_servo_hard_limit(dev):
    """舵机层限位必须独立生效（关节层过了也要挡）。

    真配置下两层等价 ⇒ 只能**临时收紧** servo_min 来构造这个场景。
    这里改的是测试自己那份 RobotCfg（`load_robot()` 每次返回新对象），
    不会污染其他测试，也不会写回配置文件。
    """
    import dataclasses

    a = dev.robot.actuators[0]                    # base 的 S9，中位 90、限位 [30,150]
    dev.robot.actuators[0] = dataclasses.replace(
        a, servo_min=a.servo_min + 70.0)          # 30 → 100，把中位 90 也排除掉
    dev.handle("JR 0 20 130 50")
    line = last_line(dev)
    assert line.startswith(f"ERR SERVO S{a.channel}"), line
    assert "limit" in line


def test_reset_returns_servos_to_90(dev):
    dev.handle("JR 0 20 130 50")
    dev.capture.clear()
    dev.handle("RESET")
    lines = dev.capture.lines()
    assert lines[0] == "OK RESET"
    assert lines[1].startswith("OK JR ")
    for ch in (6, 7, 8, 9):
        assert f"S{ch}=90.00" in lines[1]


# ---------------------------------------------------------------------------
# ★ 核心语义：OK JR 是"意图"，STATE 才是"实际"
# ---------------------------------------------------------------------------


def test_ok_jr_is_intent_state_is_actual(dev):
    """受理回执只说明"我打算去哪"，**实际位置一步都没动**。

    这是本项目的一条铁律（MG90S 无位置回读）：`OK` / `STATUS` / 后端 `joint_state`
    全都在说意图，唯一的外部地面真值是相机。把它写成断言，防止有人日后
    图省事把 `OK JR` 的舵机角当作"实际位置"回推给前端。
    """
    home_shoulder = dev.current_joints()["shoulder"]
    dev.handle("JR 0 40 130 50")
    assert last_line(dev).startswith("OK JR")
    # 受理瞬间实际位置分毫未动
    assert dev.current_joints()["shoulder"] == pytest.approx(home_shoulder, abs=1e-9)

    # 推进 20ms：动了，但远未到目标
    dev.sim.step(20)
    mid = dev.current_joints()["shoulder"]
    assert home_shoulder < mid < 40.0 - 1.0, f"20ms 后 shoulder={mid}"

    # 收敛后贴住目标（留出重力的稳态误差 τ/kp）
    dev.sim.settle(4.0)
    assert dev.current_joints()["shoulder"] == pytest.approx(40.0, abs=0.6)


def test_state_frame_converges_then_stops(dev):
    """`STATE` 只在位置真的变化时发；收敛后必须停下来（否则永远 30Hz 刷帧）。

    ⚠️ 这里的 `settle()` 必须比别处**严得多**（1e-4 → 1e-8 rad/s）—— 这不是调参凑绿，
    而是两个阈值的量纲本来就不同，必须联立检查：
      * `settle(tolerance_rad=T)` 约束的是**速度** ⇒ 1s 内最多漂 `T` 弧度
      * `report_if_moved` 比的是**相对上次发射的累计位移**，阈值 1e-5 rad
    ⇒ 只要 `T × 1s > 1e-5`，一条"仍在缓慢爬行"的臂就**必然**发出状态帧。
      那是**正确**行为（它确实在动），不是 bug —— 是这条测试的前提没成立。

    被动腕（软等式约束）把收敛尾巴拉长了：旧的 1e-4 退出时残留速度实测 3.7e-5 rad/s，
    1s 累计 3.7e-5 > 1e-5，于是恰好在第 7 块左右发出唯一一帧。
    改成 1e-8 之后臂是**真静止**（实测 1s 累计位移 ~1e-14），断言才真的落在"已收敛"上。
    """
    dev.handle("JR 0 40 130 50")
    dev.sim.settle(20.0, tolerance_rad=1e-8, hold_s=0.5)
    dev.capture.clear()

    last = dev.sim.data.qpos.copy()
    dev.report_if_moved(last)                    # 静止后首帧是允许发的
    before = len(dev.capture.lines())
    for _ in range(20):
        dev.sim.step(50)                         # 再推 1s（20 × 50ms）
        last = dev.report_if_moved(last)
    assert len(dev.capture.lines()) == before, "已静止却仍在发 STATE"


def test_state_uses_absolute_elbow_angle(dev):
    """`STATE` 的 elbow 是**绝对角**（与 UI / 协议一致），不是 MuJoCo 的局部角。"""
    dev.handle("JR 0 40 125 50")
    dev.sim.settle(4.0)
    dev.report_if_moved(None)
    parts = last_line(dev).split()
    assert parts[0] == "STATE"
    vals = [float(x) for x in parts[1:]]
    assert len(vals) == 4
    joints = dict(zip(dev.robot.joint_order(), vals))
    # 若错把局部角报出去，elbow 会变成 125-40=85 左右
    assert joints["elbow"] > 100.0, f"elbow 报成了局部角？{joints}"
    assert joints["elbow"] == pytest.approx(125.0, abs=1.0)


# ---------------------------------------------------------------------------
# 与 Go `ParseReply` 的兼容性
# ---------------------------------------------------------------------------


def classify_go_reply(line: str) -> str:
    """复刻 Go `ParseReply` 的分类（不认得的一律 OTHER）。

    ⚠️ 判定顺序与 Go **必须**一致：`ERR` → `STATE` → `ERR JOINT` → `OK JR`
       → `#`/`STATUS` 舵机快照。顺序不是风格问题：含 `S<n>=` 的行有好几种，
       把 `OK JR` 放到快照后面就会把"目标角"读成"实际角"。
    """
    up = line.upper()
    if up.startswith("ERR"):
        return "ERR"
    if up.startswith("STATE ") or up == "STATE":
        try:
            for x in line.split()[1:]:
                float(x)
            return "STATE"
        except ValueError:
            return "OTHER"
    if up.startswith("OK JR"):
        return "OK_JR" if re.search(r"S\d+=", line) else "OTHER"
    # 白名单：`#`（异步上报）与 `STATUS`（查询应答）**才**算实际角
    if re.search(r"S\d+=-?\d", line) and (line.startswith("#") or up.startswith("STATUS")):
        return "SERVO"
    return "OTHER"


def test_go_parseable_replies_are_exactly_the_three_shapes(dev):
    """正常路径的三类回执必须能被 Go 认出来：`OK JR` / `STATE` / `ERR`。"""
    cases = {
        "JR 0 20 130 50": "OK_JR",     # 受理
        "JR 0 20 90 50": "ERR",        # 关节越限
        "JR 1 2 3": "ERR",             # 参数个数错
        "ZZZ": "ERR",                  # 未知命令
    }
    for cmd, want in cases.items():
        dev.capture.clear()
        dev.handle(cmd)
        first = dev.capture.lines()[0]
        assert classify_go_reply(first) == want, f"{cmd!r} → {first!r}"


def test_reply_classification_registry(dev):
    """★ 登记在案的回执 → Go 分类映射（含两条最容易搞错的）。

      * `# SERVO …` → **SERVO**：实际角 ⇒ 上层发布 `origin=device` ⇒ 界面**跟随**
      * `STATUS …`  → **SERVO**：同上（白名单第二个前缀）。真机链路上这条不会生效，
                      `serial.go` 的 `execStatus` 会先把它翻译成 `STATE` ——
                      那个**链路间不对称**是已知的，见 docs/serial-v1.md。
      * `OK SET …` / `OK JOY …` → **OTHER**：它们**也**含 `S<n>=`，但携带的是
                      **钳位后的目标角**。一旦收进白名单，状态就恒等于命令、
                      误差恒为 0，整条"滞后 → 收敛"语义被一个乐观 ACK 抹掉。
      * `OK PING` / `OK RESET` / `OK RESET -> 90` → **OTHER**（心跳只关心有没有回音）
    """
    cases = {
        "# SERVO S9=90.00 S8=48.39 S7=117.58 S6=90.00": "SERVO",
        "STATUS S9=90 S7=90 S8=90 S6=90": "SERVO",
        # ⚠️ 光一个 `STATUS` 词、不带任何 `S<n>=` ⇒ 仍是 OTHER：白名单管的是**前缀**，
        #    但 `reServoKV` 必须先匹配到键值对，两者是"与"的关系。
        "STATUS": "OTHER",
        "OK SET S9=90": "OTHER",
        "OK JOY S9=98 S8=57 S7=26 S6=98": "OTHER",
        "OK PING": "OTHER",
        "OK RESET": "OTHER",
    }
    for line, want in cases.items():
        assert classify_go_reply(line) == want, f"{line!r} 的分类变了？请复核 protocol.go"


def test_ping_and_reset_are_reply_other(dev):
    """`OK PING` / `OK RESET` 是旁路回执（Go 只关心"有没有回音"）。"""
    for cmd, first_line in (("PING", "OK PING"), ("RESET", "OK RESET")):
        dev.capture.clear()
        dev.handle(cmd)
        assert dev.capture.lines()[0] == first_line
        assert classify_go_reply(first_line) == "OTHER", (
            f"{first_line!r} 竟然被 Go 识别了？请复核 protocol.go 的 ParseReply")


# ---------------------------------------------------------------------------
# ★ 核心语义（二）：设备侧自主变化（SET / JOY / S<n>=）→ `# SERVO`
# ---------------------------------------------------------------------------
#
# 这一节钉住的是「下位机被命令之外的手段改了，上位机界面要跟着走」这条链路。
# 真机上对应硬件摇杆 / 红外遥控 / 面板手拧；这里用固件级命令 SET / JOY 制造
# 同样的**外部**变化，从而让三种链路（sim / serial / mujoco）用同一份验收脚本。
#
# ⚠️ 两条上报路径**不能混**：
#     JR 受理       → `STATE`（命令引起，origin=command）
#     SET/JOY       → `# SERVO`（设备自主变化，origin=device）
#   反过来会立刻出两个真实缺陷：用 `STATE` 跑外部变化 ⇒ 拖滑杆时滑杆被"回拉"；
#   外部变化走 `STATE` 之外的路 ⇒ 命令路径失去状态回推、误差面板恒为 0。

#: 摇杆公式的期望值表。
#:
#: **期望值来源**：固件 `core/joystick.c::joystick_delta()` 的源码逐行推导
#: （死区 200..800 不动 / `id==8` 方向取反 / 步长 2..10 与推杆深度成比例）。
#: 其中两条**另有独立于源码的实测支撑**（这是它值得当锚点的原因）：
#:
#:   (9, 100) → +5   真机 `JOY 9 100` 让 base 关节 +5.000°；base 的 scale = 1 ⇒ 直读
#:   (7, 0)   → +8   真机 `JOY 7 0` 让 shoulder 关节 +5.555°；shoulder 的
#:                   scale = (100-20)/(49.455-(-6.094)) = 1.4401 ⇒ 8 / 1.4401 = 5.555 ✓
#:
#: 也就是说这张表不是"抄自己"，而是固件实测 ↔ 源码公式的**跨链路交叉验证**。
JOYSTICK_CASES: tuple[tuple[int, int, int], ...] = (
    (9, 500, 0),        # 死区内不动
    (9, 200, 0),        # 边界：raw=200 不算 past_lo（严格小于）
    (9, 199, 2),        # beyond=1 ⇒ 2 + 1//30 = 2
    (9, 100, 5),        # beyond=100 ⇒ 2 + 3 = 5
    (9, 0, 8),          # beyond=200 ⇒ 2 + 6 = 8
    (9, 900, -5),       # past_hi：普通轴"推高"= 负
    (9, 1023, -9),      # beyond=223 ⇒ 2 + 7 = 9，封顶 10 未触及
    (8, 100, -5),       # id==8 方向取反
    (8, 900, 5),
    (6, 0, 8),
    (7, 1023, -9),
)


def test_joystick_formula_matches_firmware():
    """公式表的**逐值**锚定（固件 / Go / Python 三处必须同值）。"""
    from server import joystick_delta

    for servo_id, raw, want in JOYSTICK_CASES:
        assert joystick_delta(servo_id, raw) == want, (
            f"joystick_delta({servo_id}, {raw}) = {joystick_delta(servo_id, raw)}，期望 {want}")
    # 越界输入必须先夹进 0..1023（固件第一件事就是夹）
    assert joystick_delta(9, -100) == 8
    assert joystick_delta(9, 99999) == -9
    assert joystick_delta(9, 99999) == joystick_delta(9, 1023)


def test_joy_moves_target_by_formula_step(dev):
    """`JOY <id> <raw>` 在**目标**上增量，增量恰为公式步长（经舵机硬限位钳位）。"""
    for servo_id, raw, want in JOYSTICK_CASES:
        if want == 0:
            continue
        d = make_dev()
        a = d.actuator_by_channel(servo_id)
        before = d.target_servo()[servo_id]
        d.handle(f"JOY {servo_id} {raw}")
        after = d.target_servo()[servo_id]
        expect = min(max(before + want, a.servo_min), a.servo_max)
        assert after == pytest.approx(expect), (
            f"JOY {servo_id} {raw}：目标 {before} → {after}，期望 {expect}")


def test_joy_reply_carries_actual_servo_angles(dev):
    """`OK JOY` 回执是**实际**舵机角（对齐固件 `OK JOY S6=.. S7=.. S8=.. S9=..`）。"""
    dev.handle("JOY 7 0")
    line = last_line(dev)
    assert line.startswith("OK JOY "), line
    chans = [p.split("=")[0] for p in line.split()[2:]]
    assert chans == sorted(chans, reverse=True), f"通道未按降序：{chans}"
    assert all(re.fullmatch(r"S[6-9]=\d+", p) for p in line.split()[2:]), line
    # 此刻实际位置还停在 HOME（受理瞬间物理没动）
    assert f"S7={int(round(dev.current_servo()[7]))}" in line


def test_joy_full_frame_uses_firmware_channel_order(dev):
    """`JOY <r9> <r8> <r6> <r7>` 的位次与固件一致（A0..A3 → 9/8/6/7）。"""
    before = dev.target_servo()
    dev.handle("JOY 0 1023 1023 0")
    after = dev.target_servo()
    # 期望增量写死（来自公式表），**钳位边界**从配置派生
    for ch, want in ((9, 8), (8, 9), (6, -9), (7, 8)):
        a = dev.actuator_by_channel(ch)
        expect = min(max(before[ch] + want, a.servo_min), a.servo_max)
        assert after[ch] == pytest.approx(expect), f"S{ch}: {before[ch]} → {after[ch]}，期望 {expect}"


def test_joy_rejects_unknown_servo_and_bad_syntax(dev):
    dev.handle("JOY 5 100")
    assert last_line(dev).startswith("ERR ARG"), last_line(dev)
    dev.capture.clear()
    dev.handle("JOY 9")
    assert last_line(dev) == "ERR SYNTAX JOY"


def test_set_clamps_to_servo_hard_limit(dev):
    """`SET` 越界**不报错**，钳到舵机硬限位并回生效值（复刻固件 `arm_set_angle`）。

    ★ 同时证另一件事：`SET` **只改目标**，物理位置一步都没动
      —— 这是"绝不 `qpos[...] = target`"（spec §12）的可观测判据。
    """
    a = dev.actuator_by_channel(9)
    before_home = dev.current_servo()[9]
    dev.handle(f"SET 9 {int(a.servo_max) + 50}")
    line = last_line(dev)
    assert line.startswith("OK SET"), line
    assert f"S9={int(a.servo_max)}" in line
    assert dev.target_servo()[9] == pytest.approx(a.servo_max)
    assert dev.current_servo()[9] == pytest.approx(before_home)
    assert abs(dev.current_servo()[9] - a.servo_max) > 5.0, "SET 直接把物理位置摆过去了？"


def test_set_shorthand_is_equivalent_to_set_command():
    """`S<id>=<ang>` 与 `SET <id> <ang>` 必须完全等价（含回执形状）。"""
    d1, d2 = make_dev(), make_dev()
    d1.handle("SET 6 88")
    d2.handle("S6=88")
    assert d1.capture.last() == d2.capture.last()
    assert d1.target_servo()[6] == pytest.approx(d2.target_servo()[6])


def test_set_accepts_up_to_three_pairs_like_firmware(dev):
    """固件 `MAX_PAIRS=3`：sim 不该比真机宽容。"""
    dev.handle("SET 9 90 8 90 7 90")
    assert last_line(dev).startswith("OK SET"), last_line(dev)
    dev.capture.clear()
    dev.handle("SET 9 90 8 90 7 90 6 90")
    assert last_line(dev).startswith("ERR ARG"), last_line(dev)


def test_set_pairs_must_be_well_formed(dev):
    dev.handle("SET 9")
    assert last_line(dev).startswith("ERR ARG"), last_line(dev)
    dev.capture.clear()
    dev.handle("SET 9 abc")
    assert last_line(dev).startswith("ERR ARG"), last_line(dev)
    dev.capture.clear()
    dev.handle("SET 5 90")
    assert last_line(dev).startswith("ERR ARG"), last_line(dev)


def test_external_change_reports_servo_not_state(dev):
    """★ 外部变化 ⇒ `# SERVO`（不是 `STATE`）。

    这一帧到后端就是 `origin=device`，前端据此让**命令侧**跟随；
    若错发成 `STATE`，前端会把命令侧一路拖向实际位置（拖滑杆时滑杆被回拉）。
    """
    dev.handle("JOY 7 0")
    dev.sim.step(20)
    dev.capture.clear()
    dev.report_if_moved(None)
    line = last_line(dev)
    assert line.startswith("# SERVO "), f"外部变化必须走 `# SERVO`，实得 {line!r}"
    assert not line.startswith("STATE")
    # 编码形状：通道降序 + 2 位小数（与 Go EncodeServoReport 逐字节一致）
    chans = [p.split("=")[0] for p in line.split()[2:]]
    assert chans == sorted(chans, reverse=True), f"通道未按降序：{chans}"
    assert all(re.fullmatch(r"S[6-9]=\d+\.\d\d", p) for p in line.split()[2:]), line


def test_command_change_still_reports_state(dev):
    """★ 命令引起的运动**仍然**走 `STATE` —— 别把分流写成"一律 `# SERVO`"。

    若写成一律 `# SERVO`，命令路径就失去状态回推，误差面板恒为 0，
    整条"滞后 → 收敛"语义被抹掉（`controller` 里 `verifyCalibrationEcho` 同理）。
    """
    dev.handle("JR 0 20 130 50")
    dev.sim.step(20)
    dev.capture.clear()
    dev.report_if_moved(None)
    line = last_line(dev)
    assert line.startswith("STATE "), f"命令引起的运动必须走 `STATE`，实得 {line!r}"
    assert not line.startswith("#")


def test_reset_clears_external_motion(dev):
    """`RESET` 是**命令**路径 ⇒ 之后的位置推进回到 `STATE` 上报。"""
    dev.handle("JOY 7 0")
    dev.handle("RESET")
    assert dev.external_motion is False
    dev.sim.step(20)
    dev.capture.clear()
    dev.report_if_moved(None)
    assert last_line(dev).startswith("STATE "), last_line(dev)


def test_jr_clears_external_motion(dev):
    """`JR` 同样是命令路径 ⇒ 之后回到 `STATE`（否则拖滑杆时滑杆会被回拉）。"""
    dev.handle("JOY 7 0")
    assert dev.external_motion is True
    dev.handle("JR 0 20 130 50")
    assert dev.external_motion is False


def test_external_report_yields_to_command_in_flight(dev):
    """★ 闸门：命令在途时外部上报**让位**（只记脏、不发），且**让位 ≠ 丢弃**。

    ⚠️ 为什么要**直接构造在途态**：本链路走 stdio，命令处理是**同步**的，
       `jr_busy` 窗口是亚毫秒级 —— e2e 脚本根本撞不进去。
       这与 Go 侧同一条纪律：**闸门由单测证、e2e 只证可观测后果**
       （playbook §14.10；那边 e2e 也摸不到 15ms 窗口）。
    """
    dev.handle("JOY 7 0")
    dev.sim.step(20)
    dev.capture.clear()

    dev.jr_busy = True                       # 直接构造"命令在途"
    snapshot = dev.report_if_moved(None)
    assert dev.capture.lines() == [], "命令在途时不该发出任何上报"
    assert dev.report_dirty is True, "让位必须留下脏标记（否则就是把上报丢了）"
    assert snapshot is None, "让位时不该推进快照 —— 否则下一轮就看不到位移、真的丢了"

    dev.jr_busy = False
    dev.flush_report()                       # 命令了结 ⇒ 补报
    line = last_line(dev)
    assert line.startswith("# SERVO "), line
    assert dev.report_dirty is False


def test_flush_report_reports_latest_value_not_stale(dev):
    """补报取的是**当时**的 actual ⇒ 中间帧被合并（latest-wins），不积压也不补旧姿态。"""
    dev.handle("JOY 7 1023")                 # 肩向下拨满
    dev.jr_busy = True
    dev.sim.step(30)
    dev.report_if_moved(None)                # 让位：只记脏
    assert dev.report_dirty is True

    dev.sim.step(300)                        # 让它多走一段
    current = dev.current_servo()
    dev.jr_busy = False
    dev.capture.clear()
    dev.flush_report()
    line = last_line(dev)
    assert line.startswith("# SERVO "), line
    got = {int(p[1]): float(p.split("=")[1]) for p in line.split()[2:]}
    for ch, v in got.items():
        assert v == pytest.approx(current[ch], abs=0.01), (
            f"S{ch} 补报的是旧值（{v} ≠ 当前 {current[ch]}）—— 违背 latest-wins")


# ---------------------------------------------------------------------------
# 真进程冒烟（Phase 7 的 Go 侧依赖的正是这条路径）
# ---------------------------------------------------------------------------


def test_server_process_smoke():
    """像 Go 那样用管道起真进程：喂命令、读回执、靠 EOF 优雅退出。

    ⚠️ 命令必须**分两批**喂，中间留出主循环推进的时间 —— 这不是凑绿，而是
       `pump_once()` 的既有语义决定的：它**先 drain 完队列再判 stop**，
       所以「把所有命令（含 QUIT）一次喂完再关管道」会让循环在第一轮就退出，
       **一步物理都不推**，于是根本不会产生任何上报帧。那样断言 `# SERVO`
       失败的原因会是"测试没给主循环机会"，而不是"上报机制坏了"。
    """
    proc = subprocess.Popen(
        [sys.executable, str(SIM_DIR / "server.py"), "--no-realtime", "--report-hz", "1000"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace", bufsize=1,
    )
    try:
        proc.stdin.write("PING\nSTATUS\nJR 0 20 130 50\nSET 6 88\nS6=42\nJOY 7 0\n")
        proc.stdin.flush()
        time.sleep(1.0)          # 让主循环 drain → step → 上报
        proc.stdin.write("QUIT\n")
        proc.stdin.flush()
        out, err = proc.communicate(timeout=180)
    finally:
        if proc.poll() is None:
            proc.kill()
    assert proc.returncode == 0, f"退出码 {proc.returncode}\nstderr:\n{err}"
    assert "OK PING" in out, out
    assert "STATUS S9=90 S7=90 S8=90 S6=90" in out, out
    assert "OK JR S9=90.00 S8=48.39 S7=117.58 S6=90.00" in out, out
    assert "OK SET S6=88" in out, out
    assert "OK SET S6=42" in out, out
    assert "OK JOY " in out, out
    # ★ 端到端最关键的一条：真进程也必须产出 `# SERVO`（外部变化的上报通道）
    assert "# SERVO " in out, f"外部变化没有产生 # SERVO 上报：\n{out}"
    servo_line = next(ln for ln in out.splitlines() if ln.startswith("# SERVO "))
    assert [p.split("=")[0] for p in servo_line.split()[2:]] == ["S9", "S8", "S7", "S6"], servo_line

