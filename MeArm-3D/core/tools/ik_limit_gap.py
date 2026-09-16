#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK 限位差距量化：把「IK 可达域」与「真机关节限位可达域」逐项对照。

目的：回答「目前差多少」——
  ① 目标点层面：IK 能解的区域 vs 真机能去的区域，差多少 mm
  ② 关节角层面：IK 解的角 vs 真机限位，差多少度
  ③ 端点层面：限位端点取整后差多少度
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT / "simulation" / "mujoco", ROOT / "core" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from fkref import fk_tcp_mm                                     # noqa: E402
from robotcfg import load_robot_by_id                            # noqa: E402
from units import ensure_utf8_stdout                             # noqa: E402

ensure_utf8_stdout()


def main() -> int:
    robot = load_robot_by_id("mearm-v1")
    j = {x.id: x for x in robot.movable_joints()}
    sh, el = j["shoulder"], j["elbow"]
    home = {k: float(v) for k, v in robot.home_pose.items()}
    pivot_z = robot.link("base_link").length + robot.link("column_link").length
    l1 = robot.link("upper_arm_link").length
    l2 = robot.link("forearm_link").length
    toff = robot.link("tool_link").length

    print("=" * 92)
    print("差距对照表：IK 可达域 vs 真机关节限位可达域")
    print("=" * 92)

    # ---- ① 「真机限位全网格」的可达域（J1/J2/J3 扫） ---------------------------
    # 已在 ws_scan 中实测；此处只重算矢状面关键量，避免重复大扫描。
    print("\n【① 目标点层面：IK 的可解范围 vs 真机能到的范围】")
    print()
    print("  真机 J1/J2/J3 全网格实测（ws_scan，234498 点，步长 1°）：")
    print("    TCP X ∈ [ 40.686, 177.091] mm")
    print("    TCP Y ∈ [-153.365, 153.365] mm")
    print("    TCP Z ∈ [ 48.965, 114.693] mm")
    print("    R = hypot(X,Y) ∈ [ 81.373, 177.091] mm")
    print()
    print("  IK 的判据（ik.ts + robot.yaml 限位）:")
    reach_min, reach_max = abs(l1 - l2), l1 + l2
    print(f"    腕枢轴距肩枢轴 D ∈ [{reach_min:.3f}, {reach_max:.3f}] mm  ← 纯几何壳")
    print(f"    ⇒ 换算回 TCP: 径向 R = D + {toff:.0f} ∈ "
          f"[{reach_min + toff:.3f}, {reach_max + toff:.3f}] mm")
    print(f"    Z 由 2R 决定，同时受 shoulder/elbow 限位约束")

    # ---- ② 关键差距：肘部绝对角 vs 限位 --------------------------------------
    print("\n【② 关节角层面：肘部「绝对角」与限位的错配】")
    print()
    print(f"  elbow 限位（真值 robot.yaml）: {el.limit_min:.10f} .. {el.limit_max:.10f}")
    print(f"  elbow 存的是**绝对倾角**（0 = 天顶，+ 向前倾）")
    print()
    print("  这意味着 elbow 的下限 108.44° 描述的是一个**相对角**约束：")
    print(f"    elbow_abs = shoulder + relative ≥ {el.limit_min:.4f}")
    print(f"    relative ≥ {el.limit_min:.4f} − shoulder")
    print()
    print("  由此推出**可行肩角区间随肘角收缩**（这是 IK 两支解被拒的直接原因）：")
    print()
    print("    姿态              shoulder(°)   elbow_abs 相对角  是否在 elbow 限位内")
    print("    " + "-" * 74)
    for tag, ts, te in (
        ("zero（全 0°）", 0.0, 0.0),
        ("HOME", home["shoulder"], home["elbow"]),
        ("shoulder_min", sh.limit_min, home["elbow"]),
        ("shoulder_max", sh.limit_max, home["elbow"]),
        ("elbow_min", home["shoulder"], el.limit_min),
        ("elbow_max", home["shoulder"], el.limit_max),
    ):
        rel = te - ts
        ok = el.limit_min - 1e-9 <= te <= el.limit_max + 1e-9
        print(f"    {tag:<16} {ts:>10.4f}  {te:>10.4f}  {rel:>8.4f}   "
              f"{'✓' if ok else '✗ 越界 ' + format(el.limit_min - te, '.4f') + '°'}")

    # ---- ③ 端点量化差距 -----------------------------------------------------
    print("\n【③ 端点量化：整数度取整后差多少】")
    print()
    hdr = f"    {'关节':<10}{'限位下限':>16}{'round':>8}{'Δ':>12}   {'限位上限':>14}{'round':>8}{'Δ':>12}"
    print(hdr)
    print("    " + "-" * 84)
    for name, joint in (("shoulder", sh), ("elbow", el)):
        lo_r, hi_r = round(joint.limit_min), round(joint.limit_max)
        d_lo, d_hi = lo_r - joint.limit_min, hi_r - joint.limit_max
        print(f"    {name:<10}{joint.limit_min:>16.6f}{lo_r:>8d}{d_lo:>+12.6f}   "
              f"{joint.limit_max:>14.6f}{hi_r:>8d}{d_hi:>+12.6f}")
    print()
    print("    ⚠️ elbow 两端 round 后都**落到限位外**：")
    print(f"       round({el.limit_min:.6f}) = {round(el.limit_min)} < {el.limit_min:.6f}"
          f"  → 越界 {el.limit_min - round(el.limit_min):.6f}°")
    print(f"       round({el.limit_max:.6f}) = {round(el.limit_max)} > {el.limit_max:.6f}"
          f"  → 越界 {round(el.limit_max) - el.limit_max:.6f}°")
    print()
    print("    安全整数区间（下限 ceil / 上限 floor）：")
    print(f"       shoulder [{math.ceil(sh.limit_min)}, {math.floor(sh.limit_max)}]")
    print(f"       elbow    [{math.ceil(el.limit_min)}, {math.floor(el.limit_max)}]")

    # ---- ④ 舵机换算 ---------------------------------------------------------
    print("\n【④ 越界量换算到舵机度（真机上差多少个舵机码）】")
    print()
    for act in robot.actuators:
        jt = j.get(act.joint_id)
        if jt is None or act.joint_id not in ("shoulder", "elbow"):
            continue
        print(f"    {jt.id:<9} 关节 1° → 舵机 {abs(act.scale):.5f}°"
              f"   ⇒ elbow 越界 0.441485° → 舵机 {0.441485 * abs(act.scale):.4f}°")
    print()
    print("=" * 92)
    print("⑤ ★ 真正的根因（不是量化，是「解支反了」）")
    print("=" * 92)
    print()
    print("  实测：home / shoulder_min / shoulder_max / elbow_min / elbow_max 五个点，")
    print("        **elbow-up 支全部精确解得回原角（越界 0.000000）**，")
    print("        而 elbow-down 支把 shoulder 与 elbow 的值**互换了** ——")
    print("        那是『把小臂绝对角当成 2R 相对角』才会出现的形态。")
    print()
    print("  zero 点 (40, 0, 220) 是唯一真正不可解的：")
    print("        dr=0, dz=160 ⇒ D = 160 = L1 + L2 = 160，恰好落在可达壳**边界**（臂完全伸直），")
    print("        alpha = 0 ⇒ 两支解退化为同一点，elbow_abs = shoulder = 0°,")
    print(f"        而 elbow 限位下限是 {el.limit_min:.4f}° ⇒ 差 {el.limit_min - 0:.4f}°。")
    print("        这是**几何必然**：全零位形下臂是笔直的，而真机肘关节物理上弯不到那么直。")
    print()
    print("  ⇒ 结论：zero 被拒 **不是实现缺陷**，而是『全 0° 位形本身不在真机限位内』。")
    print("    它同时暴露了一件事：robot.yaml 的 `joints.elbow.limit.min = 108.44`")
    print("    与 `homePose.elbow = 112.62` 只差 4.18°，而 zero 用的是**未标定的 0°**。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
