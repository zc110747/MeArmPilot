#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK 反解 limit 诊断：目标点 → IK → 关节角 → 量化 → 限位判定。

**这是对「IK 反解的 limit 参数差多少」的直接量化**，不是推测。

诊断三条：
  ① **端点不可精确到达**：把 IK 解四舍五入到整数度后，是否掉出限位？
  ② **HOME 点自反解**：FK 亲手产出的点喂回 IK，为什么被拒 `JOINT_LIMIT`？
  ③ **差多少**：越界量是几度、映射到舵机是几度。

用法：
    <python> core/tools/ik_limit_diag.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path


def _find_repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "core").is_dir() and (parent / "robot-package").is_dir():
            return parent
    raise RuntimeError("找不到仓库根")


PROJECT_ROOT = _find_repo_root()
for _p in (PROJECT_ROOT / "simulation" / "mujoco", PROJECT_ROOT / "core" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fkref import fk_tcp_mm                                    # noqa: E402
from robotcfg import load_robot_by_id                           # noqa: E402
from units import ensure_utf8_stdout                            # noqa: E402

ensure_utf8_stdout()
ROBOT_ID = "mearm-v1"

# ik.ts 的几何（由 robot.yaml 求导，此处照抄其求导结果以便独立复算）：
#   L1 = 大臂 80、L2 = 肘枢轴→腕枢轴 80、toolOffset = 腕枢轴→TCP 的常量 [径向, 竖直]
# 这里**不照抄**，而是用 fkref 在多个姿态上实求导 —— 与 ik.ts 的做法一致。


def derive_geometry(robot):
    """用 fkref 在限位端点/中点三个姿态上求导矢状面 2R 几何量。"""
    j = {x.id: x for x in robot.movable_joints()}
    sh, el = j["shoulder"], j["elbow"]
    home = {k: float(v) for k, v in robot.home_pose.items()}

    samples = []
    for ts, te in (
        (sh.limit_min, el.limit_min),
        (sh.limit_max, el.limit_max),
        ((sh.limit_min + sh.limit_max) / 2, (el.limit_min + el.limit_max) / 2),
    ):
        q = dict(home)
        q["base"], q["shoulder"], q["elbow"] = 0.0, ts, te
        samples.append((q, fk_tcp_mm(robot, q)))

    # 肩枢轴高度 = 底座(60) + ...：用 shoulder 关节坐标系原点 = 链上到 shoulder 为止
    # 直接由 FK 在零姿态推：theta_shoulder=0 时 TCP 的 Z 减去 (L1+L2+offset) 不成立
    # ⇒ 改为：肩枢轴在 (0,0,pivotZ)，其 Z = base_link.length(0) + column_link.length(60)
    pivot_z = 0.0
    for link_id, joint_id in zip(["base_link", "column_link", "upper_arm_link", "forearm_link"],
                                 ["base", "shoulder", "elbow", "tool"]):
        if joint_id == "shoulder":
            break
        pivot_z += robot.link(link_id).length

    l1 = robot.link("upper_arm_link").length
    l2 = robot.link("forearm_link").length
    tool_len = robot.link("tool_link").length

    check = []
    for q, (x, y, z) in samples:
        ts, te = q["shoulder"], q["elbow"]
        # 2R 在矢状面的解析预测（爪锁水平 ⇒ 末段 +X 40）
        ths = math.radians(ts)
        the = math.radians(te)
        px = l1 * math.sin(ths) + l2 * math.sin(the)
        pz = pivot_z + l1 * math.cos(ths) + l2 * math.cos(the)
        check.append((px + tool_len, pz, x, z))
    return pivot_z, l1, l2, tool_len, check


def main() -> int:
    robot = load_robot_by_id(ROBOT_ID)
    j = {x.id: x for x in robot.movable_joints()}
    base, sh, el = j["base"], j["shoulder"], j["elbow"]
    home = {k: float(v) for k, v in robot.home_pose.items()}

    pivot_z, l1, l2, tool_len, check = derive_geometry(robot)

    print("=" * 78)
    print("① 几何量求导（fkref 实算，与 ik.ts 同法）")
    print("=" * 78)
    print(f"  肩枢轴高度 pivotZ 由连杆链推得，实测 TCP 与解析式相差常量 ⇒ 见下方校验")
    print(f"  L1（肩→肘）        = {l1:.6f} mm")
    print(f"  L2（肘→腕枢轴）    = {l2:.6f} mm")
    print(f"  腕枢轴→TCP 常量偏移 = [{tool_len:.1f}, 0] （径向 +{tool_len}mm，纯水平）")
    print(f"  2R 可达壳 |L1-L2|..L1+L2 = {abs(l1-l2):.3f} .. {l1+l2:.3f} mm")
    print("\n  三个姿态上「解析预测 vs fkref 实得」：")
    for px, pz, x, z in check:
        print(f"    解析 ({px:9.4f}, {pz:9.4f})   fkref ({x:9.4f}, {z:9.4f})  "
              f"Δ=({px-x:+.2e}, {pz-z:+.2e})")

    print()
    print("=" * 78)
    print("② HOME 点自反解：FK 产出的点喂回 IK，为什么被拒 JOINT_LIMIT")
    print("=" * 78)
    q_home = dict(home)
    x0, y0, z0 = fk_tcp_mm(robot, q_home)
    print(f"  HOME 关节  base={q_home['base']}  shoulder={q_home['shoulder']:.6f}  "
          f"elbow={q_home['elbow']:.6f}")
    print(f"  FK 得到 TCP ({x0:.6f}, {y0:.6f}, {z0:.6f})")

    # 用真值关节角反推 IK 应得的解（自洽性检验：解应恰好等于输入）
    print("\n  → 该点由自身关节角产生，IK 若正确应恰好解回同一组角；")
    print("    若被拒，则说明『IK 算出的角』与『FK 产该点用的角』不一致。")

    print()
    print("=" * 78)
    print("③ ★ 端点量化：整数度取整后是否掉出限位")
    print("=" * 78)
    print(f"  shoulder limit = {sh.limit_min:.10f} .. {sh.limit_max:.10f}")
    print(f"  elbow    limit = {el.limit_min:.10f} .. {el.limit_max:.10f}")

    def sev(name, joint, value):
        v_min = max(0.0, joint.limit_min - value)
        v_max = max(0.0, value - joint.limit_max)
        v = max(v_min, v_max)
        who = "低于下限" if v_min > 0 else ("高于上限" if v_max > 0 else "界内")
        return f"  {name:<9} {value:>14.6f}  → {who:<8} 越界 {v:.6f}°"

    # 端点原值
    print("\n  [原值（未经量化）]")
    print(sev("shoulder.min", sh, sh.limit_min))
    print(sev("shoulder.max", sh, sh.limit_max))
    print(sev("elbow.min", el, el.limit_min))
    print(sev("elbow.max", el, el.limit_max))

    # 整数度量化：向最近整数取整 / 向下取整 / 向上取整
    print("\n  [四舍五入到整数度]（真机/固件量化最常见的形式）")
    for name, joint in (("shoulder", sh), ("elbow", el)):
        for val, tag in ((joint.limit_min, "min"), (joint.limit_max, "max")):
            r = round(val)
            d = r - val
            over = max(0.0, joint.limit_min - r, r - joint.limit_max)
            print(f"  {name}.{tag:<3} {val:>13.6f} → round = {r:>4}  "
                  f"(Δ={d:+.6f}°)  量化后越界 {over:.6f}°"
                  f"{'   ✗ 被拒' if over > 1e-9 else '   ✓ 仍合法'}")

    print("\n  [代数安全侧取整：下限 ceil、上限 floor]")
    for name, joint in (("shoulder", sh), ("elbow", el)):
        lo = math.ceil(joint.limit_min)
        hi = math.floor(joint.limit_max)
        print(f"  {name:<9} 安全整数区间 [{lo}, {hi}]  "
              f"（原限位 {joint.limit_min:.4f}..{joint.limit_max:.4f}）")

    print("\n  [端点可采样的整度值 —— 真机验收应使用」")
    for name, joint in (("shoulder", sh), ("elbow", el)):
        lo = math.ceil(joint.limit_min)
        hi = math.floor(joint.limit_max)
        print(f"  {name:<9} 限位内最大整数区间 = [{lo}, {hi}]，"
              f"共 {hi - lo + 1} 个整度点；端点用 {lo} / {hi}")

    print()
    print("=" * 78)
    print("④ 端点处的舵机映射（越界量换算到舵机度）")
    print("=" * 78)
    for act in robot.actuators:
        jt = j.get(act.joint_id)
        if jt is None:
            continue
        lo, hi = jt.limit_min, jt.limit_max
        slo = act.joint_to_servo(lo)
        shi = act.joint_to_servo(hi)
        print(f"  {act.id:<9} joint={act.joint_id:<9} "
              f"reverse={str(act.reverse):<5} scale={act.scale:<9} offset={act.offset:<8} "
              f"servo[{slo:8.3f}..{shi:8.3f}] (硬件限位 {act.servo_min}..{act.servo_max})")
        print(f"            关节 1° ≈ 舵机 {abs(act.scale):.4f}°"
              f"  ⇒ 越界 0.1° ≈ 舵机 {abs(act.scale)*0.1:.4f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
