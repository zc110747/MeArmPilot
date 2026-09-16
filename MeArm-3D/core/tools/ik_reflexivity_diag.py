#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""诊断：为什么由 FK 亲手产出的点（如 zero / home）会被 IK 判 JOINT_LIMIT。

复算 ik.ts 的解析式，把两支解算出来，与本人真值关节角对照。
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


def probe(robot, label: str, q: dict, *, j, pivot_z, l1, l2, toff) -> None:
    sh, el = j["shoulder"], j["elbow"]
    x, y, z = fk_tcp_mm(robot, q)
    r = math.hypot(x, y)
    dr = r - toff
    dz = z - pivot_z
    d = math.hypot(dr, dz)
    print(f"\n--- {label} ---")
    print(f"  输入关节  shoulder={q['shoulder']:>14.8f}  elbow={q['elbow']:>14.8f}")
    print(f"  FK  TCP  ({x:.6f}, {y:.6f}, {z:.6f})")
    print(f"  扣偏移后 腕枢轴目标  dr={dr:.6f}  dz={dz:.6f}  d={d:.6f}"
          f"   (reach {abs(l1 - l2):.3f}..{l1 + l2:.3f})")

    reach_max = l1 + l2
    reach_min = abs(l1 - l2)
    if d > reach_max + 1e-9 or d < reach_min - 1e-9:
        print("  ⇒ OUT_OF_WORKSPACE（几何不可达）")
        return

    cd = min(max(d, reach_min), reach_max)
    cos_a = (cd * cd - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    cos_a = max(-1.0, min(1.0, cos_a))
    alpha = math.degrees(math.acos(cos_a))
    phi = math.degrees(math.atan2(dr, dz))
    print(f"  alpha={alpha:.10f}  phi={phi:.10f}")

    for sign, tag in ((1, "elbow-up"), (-1, "elbow-down")):
        ra = sign * alpha
        ths = phi - math.degrees(
            math.atan2(l2 * math.sin(math.radians(ra)), l1 + l2 * math.cos(math.radians(ra)))
        )
        the = ths + ra
        v_s = max(0.0, sh.limit_min - ths, ths - sh.limit_max)
        v_e = max(0.0, el.limit_min - the, the - el.limit_max)
        ok = max(v_s, v_e) <= 1e-9
        mark = "✓" if ok else "✗"
        print(f"    {mark} {tag:<11} shoulder={ths:>14.8f}  elbow={the:>14.8f}  "
              f"越界 vS={v_s:.6f} vE={v_e:.6f}")
        if not ok:
            if the < el.limit_min:
                print(f"        ↳ elbow 低于下限 {el.limit_min:.8f} 差 "
                      f"{el.limit_min - the:.6f}°")
            if the > el.limit_max:
                print(f"        ↳ elbow 高于上限 {el.limit_max:.8f} 差 "
                      f"{the - el.limit_max:.6f}°")
            if ths < sh.limit_min:
                print(f"        ↳ shoulder 低于下限 {sh.limit_min:.8f} 差 "
                      f"{sh.limit_min - ths:.6f}°")
            if ths > sh.limit_max:
                print(f"        ↳ shoulder 高于上限 {sh.limit_max:.8f} 差 "
                      f"{ths - sh.limit_max:.6f}°")


def main() -> int:
    robot = load_robot_by_id("mearm-v1")
    j = {x.id: x for x in robot.movable_joints()}
    pivot_z = robot.link("base_link").length + robot.link("column_link").length
    l1 = robot.link("upper_arm_link").length
    l2 = robot.link("forearm_link").length
    toff = robot.link("tool_link").length
    home = {k: float(v) for k, v in robot.home_pose.items()}

    print("=" * 78)
    print("IK 自反性诊断：FK 产出的点喂回 IK，是否解得回原角")
    print("=" * 78)
    print(f"pivotZ={pivot_z}  L1={l1}  L2={l2}  toolOffset={toff}")
    print(f"shoulder limit {j['shoulder'].limit_min:.8f} .. {j['shoulder'].limit_max:.8f}")
    print(f"elbow    limit {j['elbow'].limit_min:.8f} .. {j['elbow'].limit_max:.8f}")

    zero = {k: 0.0 for k in home}
    probe(robot, "zero（全 0°）", zero, j=j, pivot_z=pivot_z, l1=l1, l2=l2, toff=toff)
    probe(robot, "home", home, j=j, pivot_z=pivot_z, l1=l1, l2=l2, toff=toff)

    # 限位端点姿态
    for tag, ts, te in (
        ("shoulder_min", j["shoulder"].limit_min, home["elbow"]),
        ("shoulder_max", j["shoulder"].limit_max, home["elbow"]),
        ("elbow_min", home["shoulder"], j["elbow"].limit_min),
        ("elbow_max", home["shoulder"], j["elbow"].limit_max),
    ):
        q = dict(home)
        q["shoulder"], q["elbow"] = ts, te
        probe(robot, f"限位端点 {tag}", q, j=j, pivot_z=pivot_z, l1=l1, l2=l2, toff=toff)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
