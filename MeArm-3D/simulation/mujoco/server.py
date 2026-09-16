# -*- coding: utf-8 -*-
"""无头 MuJoCo 设备服务：在 stdio 上跑 **arm-device 文本协议**。

它对接 `device.Device` 的第三个实现（`backend/internal/device/mujoco.go`）。
协议与 `backend/internal/device/sim.go` 以及真实固件**逐字节一致**：

    ── 下行（Go → 本进程）────────────────────────────────────────────
      JR <j1> <j2> <j3> <grip>     关节角整帧命令（位次 = JointOrder）
      SET <id> <ang> [<id> <ang>…] 直接设定舵机角（≤3 组，固件 `SET` 语义）
      S<id>=<ang>                  `SET` 的简写（id 只能 6..9）
      JOY <id> <raw>               单轴摇杆拨动（raw 0..1023）
      JOY <r9> <r8> <r6> <r7>      整帧摇杆（位次与固件一致）
      STATUS | STATE?              查询舵机角（整数）
      RESET                        全部舵机回 90°（= HOME，不是"关节全 0"）
      PING                         心跳
      QUIT | EXIT                  优雅退出

    ── 上行（本进程 → Go）────────────────────────────────────────────
      OK JR S9=.. S8=.. S7=.. S6=..   受理回执（**目标**舵机角，2 位小数）
      STATE <j1> <j2> <j3> <grip>     **实际**关节角主动上报（2 位小数）
      # SERVO S9=.. S8=.. S7=.. S6=.. **实际**舵机角异步上报（设备侧自主变化）
      OK SET … / OK JOY …             舵机级指令受理回执（**钳位后目标角**，整数）
      STATUS S9=.. S7=.. S8=.. S6=..  舵机角查询应答（整数）
      OK RESET / OK PING
      ERR JOINT <id> <v> (limit <min>..<max>)
      ERR SERVO S<n> <v> (limit <min>..<max>)
      ERR ARG <说明> / ERR UNKNOWN <原文>

于是 WebSocket / controller / protocol / 前端**全部零改动**，接入点只是
`backend/main.go` 的 device 工厂多一个 case（spec §2 / §25 / §40）。

# 两条上报路径（这是"上位机状态跟随"的关键，别把它们混成一条）

    JR 受理         → 位置推进 → `STATE …`      关节角，**命令引起**（origin=command）
    SET/JOY/S<n>=   → 位置推进 → `# SERVO …`    舵机角，**设备侧自主变化**（origin=device）

为什么要分成两条：`JR` 是上位机命令，设备只是在执行 —— 状态跟随命令即可；
而 `SET` / `JOY` 在本项目里模拟的是**命令之外的改动**（真机上对应硬件摇杆、红外遥控、
面板手拧、调试直控）。这类改动必须让界面**跟着走**，否则 UI 会停在一个与现场不符的
姿态上，而且看不出任何异常。

⚠️ 因此不要把 `# SERVO` 也用在 JR 路径上：那会让界面把命令侧一路拖向实际位置，
   拖滑杆时会看到滑杆被"回拉"（命令语义被跟随语义吃掉）。

⚠️ 三条链路（sim / serial / mujoco）必须**行为一致** —— 判据就是
   `core/tools/verify_device_follow.mjs` 同一份脚本在三种 `--config` 下全绿。

# 优先级：自主上报让位于命令

`SET`/`JOY` 触发的上报是**低优先级**的：命令在途（`jr_busy`）时只记脏、不发，
等命令了结后**重新取值**补报（latest-wins，中间帧被合并而不是积压）。
与固件侧「TX 环为空才发」、Go sim 侧「`jrBusy` 抑制 + `flushReport()`」是同一条规则。

三个时间尺度（spec §23 / §24）—— 三者互相解耦，**渲染帧率不决定物理步长**：

    physics  1000 Hz   固定 1ms 步长；唯一推进 `qpos` 的地方
    control   100 Hz   `MeArmSim.step()` 内部按 `_ctrl_accum` 节流（不是每个物理步都改 ctrl）
    report     30 Hz   只有位置**真的变了**才发 STATE / `# SERVO`

⚠️ `OK JR` / `OK SET` 回的是"我打算去哪"（目标或钳位后目标），`STATE` / `# SERVO`
   才是"它现在在哪"（实际）。这是本项目的一条铁律：MG90S 没有位置回读，任何
   `OK`/`STATUS` 都只是意图，唯一的外部地面真值是相机（见 docs/decisions.md D34）。

⚠️ ★★ `SET` / `JOY` **只改 target**，绝不出现 `qpos[...] = target` 这种直接摆位
   （spec §12）。这里的 `actual` 永远来自 MuJoCo 物理 —— 这正是它比 Go sim 更值钱的
   地方：Go sim 要自己写斜坡，这里由物理引擎给出真实响应。
"""

from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
from collections.abc import Mapping
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))

import numpy as np  # noqa: E402

from limits import validate_joints  # noqa: E402
from model import RobotSim  # noqa: E402
from robotcfg import RobotCfg, load_robot_by_id  # noqa: E402
from units import ensure_utf8_stdout  # noqa: E402

DEFAULT_REPORT_HZ = 30.0
DEFAULT_PHYS_HZ = 1000.0
DEFAULT_BATCH_MS = 10.0

#: 固件 `SET` 的多组上限（core/cmd.c `MAX_PAIRS`）。sim 不该比真机宽容。
MAX_SET_PAIRS = 3

#: `S<id>=<angle>` 简写（固件 cmd.c 的 shorthand，id 只能 6..9）。
RE_SERVO_SHORTHAND = re.compile(r"^\s*S([6-9])=(-?\d+)\s*$", re.IGNORECASE)


def joystick_delta(servo_id: int, raw: int) -> int:
    """摇杆读数 → 单次拨动步长（度）。与固件 `core/joystick.c::joystick_delta()` **同一条公式**。

    两处必须一致，否则 sim 上"拨一下走多远"与真机会对不上，而验收数字看起来仍然
    自洽 —— 这是最容易蒙混过去的一类偏差。Go 侧 `sim.go::joystickDelta` 是同一式的
    第二个实现，三处同值由测试锚定（tests/sim/test_server.py 的公式表）。

        raw < 200 → 一个方向；raw > 800 → 另一方向；中间死区不动（返回 0）
        id == 8（左舵）方向取反 —— 与原始 Arduino 草图一致
        步长与推杆深度成比例：min 2、max 10（度）

    ⚠️ 用 `//` 而不是 `/`：`beyond` 恒为非负（见两个分支），此时 Python 的向下取整
       与 C/Go 的向零取整**同值**。若哪天引入负数，两者会分叉 —— 别改这条前提。
    """
    if raw < 0:
        raw = 0
    if raw > 1023:
        raw = 1023

    past_hi = raw > 800
    past_lo = raw < 200
    if not past_hi and not past_lo:
        return 0

    beyond = raw - 800 if past_hi else 200 - raw
    step = 2 + beyond // 30
    if step > 10:
        step = 10

    positive = past_hi if servo_id == 8 else past_lo
    return step if positive else -step


class Emitter:
    """线程安全地往 stdout 写整行（**立即 flush**，否则 Go 侧会卡在缓冲里）。"""

    def __init__(self, stream=None) -> None:
        self._s = stream if stream is not None else sys.stdout
        self._lock = threading.Lock()

    def line(self, text: str) -> None:
        with self._lock:
            try:
                self._s.write(text + "\n")
            except ValueError:          # 管道已关闭
                return
            self._s.flush()


class MujocoDevice:
    """把 MuJoCo 物理包成一台"能收 JR、回 OK/ERR/STATE"的机械臂。"""

    def __init__(
        self,
        *,
        robot: RobotCfg | None = None,
        robot_id: str | None = None,
        sim: RobotSim | None = None,
        report_hz: float = DEFAULT_REPORT_HZ,
        out: Emitter | None = None,
    ) -> None:
        # 选择器（`config/robots.yaml`）是"加载哪台机器人"的唯一声明处。
        # Go 侧通过 `--robot <id>` 把它传进来 —— 两边各按自己的 default 猜，
        # 就会出现"限位在 Go 侧用 A 的、在 Python 侧用 B 的"这种双方都自认正确的错位。
        self.robot = robot if robot is not None else load_robot_by_id(robot_id)
        self.sim = sim if sim is not None else RobotSim(robot=self.robot, robot_id=robot_id)
        self.order = self.robot.joint_order()      # base, shoulder, elbow, gripper
        self.out = out if out is not None else Emitter()
        self.report_period = 1.0 / float(report_hz)

        # 通道 → 关节（`STATUS` 的字段顺序按 sim.go：跟随 joint_order）
        self.chan_of_joint: dict[str, int] = {}
        for a in self.robot.actuators:
            self.chan_of_joint.setdefault(a.joint_id, a.channel)

        self._cmds: queue.Queue[str] = queue.Queue()
        self._stop = threading.Event()

        # external_motion：当前这轮位置推进是否由**设备侧外部手段**引起
        # （SET / JOY / S<n>=）。决定上报走哪条路（见文件头"两条上报路径"）。
        # 命令路径（JR / RESET）把它置回 False。
        self.external_motion = False
        # jr_busy：是否有命令在途。
        # ★ 这是本链路的"低优先级"落点：在途期间外部上报**让位**（只记脏、不发）。
        # ⚠️ 与 Go sim 的差别要说清楚：那边有 `LatencyMs`（15ms）的送达延迟，
        #    所以窗口是毫秒级、e2e 能撞上；本链路走 stdio，处理是**同步**的，
        #    窗口是亚毫秒级。因此"让位"这条规则由**单测直接构造在途态**证明，
        #    e2e 只证可观测后果（与 Go 侧同一纪律，见 playbook §14.10）。
        self.jr_busy = False
        # report_dirty：有尚未报出的外部位置变化。
        # ⚠️ 只记"有没有"，**不缓存数值** —— 补报那一刻才取 current_servo()，
        #    所以被合并掉的永远是过期帧（latest-wins），既不积压也不补发旧姿态。
        self.report_dirty = False

        # 开机位 = 模型 HOME（四个舵机恰好 90°）。协议 §4 明确 RESET 不许
        # 改成"关节全 0"：关节全 0 对肘（绝对角 108..142）是不可达位姿。
        self.boot_pose: dict[str, float] = dict(self.robot.home_pose)
        self.sim.reset(self.boot_pose)

    # ------------------------------------------------------------------
    # 编码（必须与 protocol.go 的 Encode* 逐字节一致）
    # ------------------------------------------------------------------

    def encode_ok_jr(self, servo: Mapping[int, float]) -> str:
        """`OK JR S9=.. S8=.. S7=.. S6=..` —— 通道**降序**（对齐 EncodeOKJR）。"""
        chans = sorted(servo, reverse=True)
        return "OK JR " + " ".join(f"S{c}={servo[c]:.2f}" for c in chans)

    def encode_status(self, servo: Mapping[int, float]) -> str:
        """`STATUS S9=.. S7=.. S8=.. S6=..` —— 顺序按 joint_order，整数（对齐 sim.go）。"""
        chans = [self.chan_of_joint[j] for j in self.order if j in self.chan_of_joint]
        return "STATUS " + " ".join(f"S{c}={int(round(servo[c]))}" for c in chans
                                    if c in servo)

    def encode_state(self, joints: Mapping[str, float]) -> str:
        """`STATE <j1> <j2> <j3> <grip>`（对齐 EncodeState，2 位小数）。"""
        return "STATE " + " ".join(f"{joints[j]:.2f}" for j in self.order)

    def encode_servo_report(self, servo: Mapping[int, float]) -> str:
        """`# SERVO S9=.. S8=.. S7=.. S6=..`（对齐 `protocol.EncodeServoReport`）。

        语义三条，缺一条就会被误用：

          * `#` 前缀 = **异步事件**。协议 §3 已把 `#` 定为"非应答"，
            因此它天然不参与命令-应答门控 —— 上层不需要特判，也不该拿它
            去放行某条在途命令。
          * 它携带的是设备侧**实际**角度（由 `qpos` 反解），与 `OK JR` 的
            目标角是两个不同的量。**这是它存在的全部意义**。
          * 它表达的是**设备自主变化**（摇杆 / 红外 / 手拧 / 调试直控），
            因此上层据此发布的状态必须标 `origin=device`。

        ⚠️ 通道**降序**（S9 S8 S7 S6）与 Go 侧一致：map 迭代序在语言之间不一样，
           不固定顺序就没法做逐字节对照，测试也会变成随机失败。
        """
        chans = sorted(servo, reverse=True)
        return "# SERVO " + " ".join(f"S{c}={servo[c]:.2f}" for c in chans)

    def encode_servo_kv(self, prefix: str, servo: Mapping[int, float]) -> str:
        """编 `OK SET S7=90 S6=88` 这类**应答**行（对齐 `encodeServoKV`，整数）。

        ⚠️ 只用于"我收到了"这种应答，**不要**拿它当状态上报 —— 应答里的角度是
           目标/钳位值，不是实际位置；状态上报走 `encode_servo_report`。
        """
        chans = sorted(servo, reverse=True)
        if not chans:
            return prefix
        return prefix + " " + " ".join(f"S{c}={servo[c]:.0f}" for c in chans)

    # ------------------------------------------------------------------
    # 换算
    # ------------------------------------------------------------------

    def joints_to_servo(self, joints: Mapping[str, float]) -> dict[int, float]:
        out: dict[int, float] = {}
        for a in self.robot.actuators:
            theta = joints.get(a.joint_id)
            if theta is None:
                continue
            out[a.channel] = a.joint_to_servo(float(theta))
        return out

    def current_joints(self) -> dict[str, float]:
        """当前**实际**关节角（度，绝对语义）—— 由 `qpos` 反解而来。"""
        return self.sim.joint_angles_deg()

    def current_servo(self) -> dict[int, float]:
        return self.joints_to_servo(self.current_joints())

    # ------------------------------------------------------------------
    # 命令
    # ------------------------------------------------------------------

    def handle(self, raw: str) -> None:
        line = raw.strip()
        if not line:
            return
        upper = line.upper()

        if upper.startswith("JR"):
            self._handle_jr(line)
        # ---- 真机舵机级入口（SET / S<n>= / JOY）--------------------------
        #
        # 为什么 MuJoCo 链路也要认这几条**固件级**命令：它们代表"命令之外的改动"。
        # 真机上对应的是硬件摇杆（`joystick_scan()`）、红外遥控、面板手拧 ——
        # 都不会经过本机的 JR 通路。把它们做成可下发的命令，是为了让"下位机被
        # 外部手段改动 → 上位机状态跟随"这条链路在**三种链路**上都能端到端验证，
        # 而不必等到接上真机才发现界面不动。
        #
        # ⚠️ 语义边界：`SET` 在这里**不是** JR 的翻译结果（真机链路里 JR→SET 的翻译
        #    在 `serial.go`，本进程收到 JR 直接处理，不需要拆 SET）。收到的 SET
        #    只可能来自调试 / 验收脚本，所以按"外部改动"对待是对的。
        elif upper.startswith("SET") or RE_SERVO_SHORTHAND.match(line):
            self._handle_set(line)
        elif upper.startswith("JOY"):
            self._handle_joy(line)
        elif upper in ("STATUS", "STATE?"):
            self.out.line(self.encode_status(self.current_servo()))
        elif upper == "RESET":
            self.sim.set_target_joints(self.boot_pose)
            # RESET 是**命令**路径（上位机要求回中位）⇒ 后续推进走 STATE 上报
            self.external_motion = False
            self.out.line("OK RESET")
            self.out.line(self.encode_ok_jr(self.joints_to_servo(self.boot_pose)))
        elif upper == "PING":
            self.out.line("OK PING")
        elif upper in ("QUIT", "EXIT"):
            self.stop()
        else:
            self.out.line(f"ERR UNKNOWN {line}")

    def _handle_jr(self, line: str) -> None:
        fields = line.split()[1:]
        if len(fields) != len(self.order):
            self.out.line(
                f"ERR ARG JR 需要 {len(self.order)} 个关节角，收到 {len(fields)} 个")
            return
        try:
            joints = {j: float(v) for j, v in zip(self.order, fields)}
        except ValueError as exc:
            self.out.line(f"ERR ARG {exc}")
            return

        # ① 关节限位 —— 与 Go controller / 真机固件读的是**同一份** robot.yaml
        v = validate_joints(self.robot, joints)
        if v is not None:
            self.out.line(v.message())
            return

        # ② 舵机角换算 + 舵机硬限位
        servo = self.joints_to_servo(joints)
        for a in self.robot.actuators:
            s = servo.get(a.channel)
            if s is None:
                continue
            if s < a.servo_min - 1e-6 or s > a.servo_max + 1e-6:
                self.out.line(f"ERR SERVO S{a.channel} {s:.2f} "
                              f"(limit {a.servo_min:.2f}..{a.servo_max:.2f})")
                return

        # ③ 受理：只写**目标**。实际位置永远由物理决定 ——
        #    绝不出现 `qpos[...] = target` 这种"直接摆位"（spec §12）。
        #
        # ⚠️ 下面两行是"优先级"在本进程里的落点：
        #   external_motion=False → 本次位置推进用 `STATE` 上报（命令引起）
        #   jr_busy=True          → 在途期间**外部**上报让位（只记脏、不发）
        # 与服务端口径一致：交互指令优先，状态上报不许插队。
        self.external_motion = False
        self.jr_busy = True
        self.sim.set_target_joints(joints)
        self.out.line(self.encode_ok_jr(servo))
        self.jr_busy = False
        # 在途期间被让位掉的外部变化，在这里补报最新值（让位 ≠ 丢弃）
        self.flush_report()

    # ------------------------------------------------------------------
    # 舵机级入口（SET / S<n>= / JOY）—— 单位一律**舵机度**，与固件同一坐标系
    # ------------------------------------------------------------------

    def _handle_set(self, line: str) -> None:
        """`SET <id> <ang> [<id> <ang>…]` / `S<id>=<ang>`：直接设定舵机目标角。

        与固件 `arm_set_angle()` 一致：**钳位到该舵机的硬限位并返回生效值**，
        越界不报错（真机就是在舵机允许的行程内截断）。
        """
        pairs, err = self.parse_set_pairs(line)
        if err is not None:
            self.out.line(f"ERR ARG {err}")
            return
        applied = {ch: self.set_servo_target(ch, ang) for ch, ang in pairs.items()}
        self.mark_external()
        self.out.line(self.encode_servo_kv("OK SET", applied))

    def _handle_joy(self, line: str) -> None:
        """`JOY <id> <raw>` / `JOY <r9> <r8> <r6> <r7>`：模拟硬件摇杆的一次拨动。"""
        fields = line.split()
        deltas: dict[int, int] = {}

        if len(fields) == 3:                       # 单轴
            try:
                ch = int(fields[1])
                raw = int(fields[2])
            except ValueError:
                self.out.line(f"ERR ARG JOY {' '.join(fields[1:])}")
                return
            if not self.has_servo(ch):
                self.out.line(f"ERR ARG JOY {' '.join(fields[1:])}")
                return
            delta = joystick_delta(ch, raw)
            if delta:
                deltas[ch] = delta
        elif len(fields) == 5:                     # 整帧（位次与固件 JOY 一致）
            ids = (9, 8, 6, 7)
            for i in range(4):
                try:
                    raw = int(fields[1 + i])
                except ValueError:
                    self.out.line(f"ERR ARG JOY {fields[1 + i]}")
                    return
                if not self.has_servo(ids[i]):
                    continue
                delta = joystick_delta(ids[i], raw)
                if delta:
                    deltas[ids[i]] = delta
        else:
            self.out.line("ERR SYNTAX JOY")
            return

        for ch, delta in deltas.items():
            self.nudge_servo(ch, delta)
        if deltas:
            # ★ 摇杆驱动的运动 = **设备侧自主变化** ⇒ 位置变化要主动上报，
            #   否则上位机界面停在旧值上（"摇杆动了但界面不动"）。
            self.mark_external()
        # 回执携带**实际**舵机角（对齐 sim.go / 固件 `OK JOY S9=.. S8=..`）
        self.out.line(self.encode_servo_kv("OK JOY", self.current_servo()))

    # ------------------------------------------------------------------
    # 舵机空间小工具
    # ------------------------------------------------------------------

    def actuator_by_channel(self, ch: int):
        """按**通道号**找执行器（返回 None 表示不存在）。

        ⚠️ 通道号（6/7/8/9）**不等于**关节顺序里的位次 —— 别用 `self.order` 去索引它。
        """
        for a in self.robot.actuators:
            if a.channel == ch:
                return a
        return None

    def has_servo(self, ch: int) -> bool:
        return self.actuator_by_channel(ch) is not None

    def target_servo(self) -> dict[int, float]:
        """当前**目标**舵机角（由 `sim.target_angles_deg()` 换算）。"""
        targets = self.sim.target_angles_deg()
        return self.joints_to_servo(targets)

    def set_servo_target(self, ch: int, angle: float) -> float:
        """直接设定某舵机的目标角（`SET` / `S<n>=` 语义）。返回**钳位后**生效值。

        ⚠️ 只改 `target`，绝不改 `actual`（= 物理 `qpos`）。真机也是这个语义：
           `arm_set_angle()` 只写目标，实际位置由舵机自己慢慢爬过去。
        """
        a = self.actuator_by_channel(ch)
        if a is None:
            return angle
        if angle < a.servo_min:
            angle = a.servo_min
        if angle > a.servo_max:
            angle = a.servo_max
        self.sim.set_target_joints({a.joint_id: a.servo_to_joint(angle)})
        return angle

    def nudge_servo(self, ch: int, delta: int) -> None:
        """在当前**目标角**上增量（`JOY` 语义），并钳位到舵机硬限位。

        ⚠️ 基于 target 而不是 actual 增量，且不改写 actual —— 与固件
           `arm_nudge()` 同一取向：斜坡中途的拨动只微调终点，不中断正在进行的运动。
           （固件注释里记着：旧写法 `target = current` 会把正在斜坡中的目标就地取消，
           表现正是"gripper 有概率不执行"。）
        """
        a = self.actuator_by_channel(ch)
        if a is None or delta == 0:
            return
        v = self.target_servo().get(ch, 0.0) + float(delta)
        if v < a.servo_min:
            v = a.servo_min
        if v > a.servo_max:
            v = a.servo_max
        self.sim.set_target_joints({a.joint_id: a.servo_to_joint(v)})

    def parse_set_pairs(self, line: str) -> tuple[dict[int, float], str | None]:
        """解析 `SET <id> <ang> [<id> <ang>…]` 与简写 `S<id>=<ang>`。

        返回 `(pairs, None)` 或 `({}, 错误说明)`。
        """
        m = RE_SERVO_SHORTHAND.match(line)
        if m is not None:
            ch = int(m.group(1))
            if not self.has_servo(ch):
                return {}, f"未知舵机 S{ch}"
            return {ch: float(m.group(2))}, None

        fields = line.split()
        if len(fields) < 3 or len(fields) % 2 != 1:
            return {}, "SET 需要成对的 <id> <angle>"
        if (len(fields) - 1) // 2 > MAX_SET_PAIRS:
            # 与固件 MAX_PAIRS 对齐 —— sim 不该比真机宽容
            return {}, f"SET 最多 {MAX_SET_PAIRS} 组"
        out: dict[int, float] = {}
        for i in range(1, len(fields) - 1, 2):
            try:
                ch = int(fields[i])
            except ValueError:
                return {}, f"未知舵机 {fields[i]!r}"
            if not self.has_servo(ch):
                return {}, f"未知舵机 {fields[i]!r}"
            try:
                out[ch] = float(fields[i + 1])
            except ValueError:
                return {}, f"角度 {fields[i + 1]!r} 不是数字"
        return out, None

    def mark_external(self) -> None:
        """把后续的位置推进标记为"设备侧外部变化"（上报走 `# SERVO`）。"""
        self.external_motion = True

    def flush_report(self) -> None:
        """补报被"命令在途"让位掉的外部变化（值**当场取**，因此是 latest-wins）。"""
        if not self.report_dirty:
            return
        self.out.line(self.encode_servo_report(self.current_servo()))
        self.report_dirty = False

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------

    def _read_stdin(self) -> None:
        try:
            for line in sys.stdin:
                self._cmds.put(line)
        except (OSError, ValueError):
            pass
        self._stop.set()          # stdin 关闭 ⇒ 优雅退出

    def stop(self) -> None:
        self._stop.set()

    def pump_once(self, batch: int) -> bool:
        """处理待办命令 + 推进 `batch` 个物理步。返回是否应继续。

        ⚠️ 命令必须先 drain **再**判 stop。写成"先判 stop"会丢命令：
        stdin 被管道喂完立刻 EOF 时，`_stop` 会在队列被消费前就置位。
        """
        while True:
            try:
                raw = self._cmds.get_nowait()
            except queue.Empty:
                break
            self.handle(raw)

        if self._stop.is_set():
            return False
        self.sim.step(batch)
        return True

    def report_if_moved(self, last_qpos):
        """位置变化超过阈值才发一帧 STATE；返回最新 qpos 快照。

        ⚠️ 不能用严格相等：MuJoCo 在"接近静止"时 qpos 的末位仍会抖，
        严格相等会让状态帧以 30Hz 无限期地发下去。sim.go 的等价判据是
        "target 与 actual 是否还有 delta"，这里用位置阈值对齐同一语义。

        阈值取 1e-5 rad（≈0.00057°）—— 远小于串口协议 0.01° 的量化步长，
        因此"真有位移"一定会被报出来，不会漏帧。

        ⚠️ **判据是「相对上次发射的累计位移」，不是「单次调用的增量」**：
        快照只在**发射**时更新（未发射时原样返回入参），所以一条以 1e-3 rad/s
        缓慢爬行的臂会每 ~10ms 发一帧，而一条以 1e-7 rad/s 爬行的臂每 ~100s 发一帧。
        这是有意为之 —— 若改成"单次增量 > 1e-5 才发"，慢速运动就会**彻底静默**。
        推论（写测试时必须知道）：`settle(tolerance_rad=T)` 管的是**速度**，
        要求 1s 内不发帧就得 `T × 1s < 1e-5`；见
        tests/sim/test_server.py::test_state_frame_converges_then_stops。
        """
        q = np.array(self.sim.data.qpos, copy=True)
        if last_qpos is None or float(np.max(np.abs(q - last_qpos))) > 1e-5:
            # ★ 两条上报路径在这里分流（见文件头）：
            #     external_motion=True  → 设备侧自主变化 ⇒ `# SERVO`（origin=device）
            #     external_motion=False → 命令引起       ⇒ `STATE`  （origin=command）
            #   ⚠️ 不能反过来、也不能只留一条：用 `STATE` 跑外部变化会让界面把命令侧
            #      拖向实际位置（拖滑杆时看到滑杆被回拉）；只发 `# SERVO` 则命令路径
            #      失去状态回推，**误差面板恒为 0**，整条"滞后→收敛"语义被抹掉。
            if self.external_motion:
                # ★ 低优先级：命令在途时**让位** —— 只记脏、不发。
                #   注意此刻**不更新快照**（返回 last_qpos），因此下一轮仍会看到
                #   位移并再次尝试上报 ⇒ 让位 ≠ 丢弃。
                if self.jr_busy:
                    self.report_dirty = True
                    return last_qpos
                self.out.line(self.encode_servo_report(self.current_servo()))
                self.report_dirty = False
            else:
                self.out.line(self.encode_state(self.current_joints()))
            return q
        # 已静止：若还有被"命令在途"让位掉的外部变化，在此补报**最新值**
        # （值不缓存 ⇒ 天然 latest-wins，不会积压中间帧）。
        self.flush_report()
        return last_qpos

    def run(
        self,
        *,
        phys_hz: float = DEFAULT_PHYS_HZ,
        batch_ms: float = DEFAULT_BATCH_MS,
        realtime: bool = True,
    ) -> None:
        # ⚠️ 物理步长以 **MJCF 自己的** `timestep` 为准，不是 `1/phys_hz`。
        #    两者不一致时（MeArm 0.001 / 官方 SO-101 0.002），按 phys_hz 算 chunk
        #    会让仿真以错误速率前进，而且**不报任何错** —— 只是"看起来有点快"。
        #    RobotSim 构造时已经做过一次硬自检（timestep 必须等于驱动配置），
        #    这里再把 --phys-hz 与实际生效频率对一下，不一致就明说。
        dt = float(self.sim.model.opt.timestep)
        eff_hz = 1.0 / dt
        if phys_hz > 0 and abs(eff_hz - float(phys_hz)) > 1e-6:
            print(f"[warn] --phys-hz={phys_hz:g} 与 MJCF 的 timestep={dt:g}s "
                  f"（= {eff_hz:g} Hz）不一致，以 MJCF 为准", file=sys.stderr)
        batch = max(1, int(round((batch_ms / 1000.0) / dt)))
        chunk_s = batch * dt
        chunk_ms = chunk_s * 1000.0

        # 每 chunk 发一帧 STATE 就够 30Hz：chunk 默认 10ms ⇒ 最多 100Hz，
        # 再用 report_period 节流到 report_hz。
        chunks_per_report = max(1, int(round(self.report_period / chunk_s)))

        reader = threading.Thread(target=self._read_stdin, daemon=True,
                                  name="mujoco-stdin")
        reader.start()

        next_t = time.perf_counter()
        since_report = 0
        last_qpos = None
        while True:
            if not self.pump_once(batch):
                break
            since_report += 1
            if since_report >= chunks_per_report:
                since_report = 0
                last_qpos = self.report_if_moved(last_qpos)
            if realtime:
                next_t += chunk_s
                slack = next_t - time.perf_counter()
                if slack > 0:
                    # ⚠️ 按 chunk（默认 10ms）睡一次，而不是每个物理步睡 1ms。
                    #    Windows 的 sleep 粒度约 1~2ms，逐步 sleep 会让仿真
                    #    比真实时间慢一个数量级 —— 那样"实时"就名存实亡了。
                    time.sleep(slack)
                else:
                    next_t = time.perf_counter()      # 落后了就重新对齐，不追债


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="MuJoCo 设备服务（stdio 文本协议）")
    p.add_argument("--robot", default=None,
                   help="机器人 id（config/robots.yaml 的 key）；缺省 = 选择器的 default。"
                        "由 backend 的 -robot / robot.model_id 透传。")
    p.add_argument("--xml", default=None,
                   help="MJCF 路径；缺省由选择器决定"
                        "（MeArm 生成产物 / SO-ARM101 官方 MJCF 原样）")
    p.add_argument("--report-hz", type=float, default=DEFAULT_REPORT_HZ,
                   help="STATE 上报频率（默认 30）")
    p.add_argument("--phys-hz", type=float, default=DEFAULT_PHYS_HZ,
                   help="期望的物理步频；与实际 timestep 不符时以 MJCF 为准并告警")
    p.add_argument("--batch-ms", type=float, default=DEFAULT_BATCH_MS,
                   help="每次 sleep 前连续推进的物理时长（默认 10ms）")
    p.add_argument("--no-realtime", action="store_true",
                   help="不做实时对齐（尽快跑，供自动化测试使用）")
    args = p.parse_args(argv)

    ensure_utf8_stdout()
    try:
        sys.stdout.reconfigure(newline="\n", line_buffering=True)   # type: ignore[attr-defined]
    except (AttributeError, ValueError):
        pass

    dev = MujocoDevice(robot_id=args.robot, report_hz=args.report_hz)
    if args.xml:
        # 显式覆盖：只在明确知道自己在做什么时用（否则走选择器）
        dev.sim = dev.sim.__class__(xml_path=args.xml, robot=dev.robot)
    dev.run(phys_hz=args.phys_hz, batch_ms=args.batch_ms,
            realtime=not args.no_realtime)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
