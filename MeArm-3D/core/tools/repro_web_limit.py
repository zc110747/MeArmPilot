#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复现用户报告的现象：J1/J2/J3 到极限 → 记 XYZ → 回 HOME → 输入同样 XYZ → JOINT_LIMIT

用户给的实测数据：
    J1 = ?      （用户写 "j1-29.8"，疑为 -29.8 或 29.8）
    J2 = 49.4   （= shoulder 上限 49.4549 的下取整）
    J3 = 141.5  （= elbow   上限 141.8582 的下取整）
    得到 XYZ ≈ (97, 157, 49)

本脚本**只读、只算**，不改任何文件。

做法：
  ① 用 fkref（独立参考实现）对候选 (J1,J2,J3) 求 TCP，与 (97,157,49) 对照
  ② 把该 XYZ 喂给「复刻的 ik.ts 解析式」，看它判成什么
  ③ 逐项检查：是 OUT_OF_WORKSPACE 还是 JOINT_LIMIT？越界差多少度？
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

ROBOT_ID = "mearm-v1"


def solve_like_ik_ts(robot, target, *, l1, l2, pivot_z, pivot_r, toff):
    """复刻 ik.ts 的 solveIk 判据（含 base 判限位），返回完整诊断。"""
    j = {x.id: x for x in robot.movable_joints()}
    base, sh, el = j["base"], j["shoulder"], j["elbow"]

    x, y, z = target
    r = math.hypot(x, y)
    azimuth_indeterminate = r < 1e-9
    azimuth_raw = 0.0 if azimuth_indeterminate else math.degrees(math.atan2(y, x))

    dr = r - pivot_r - toff
    dz = z - pivot_z
    d = math.hypot(dr, dz)
    reach_min, reach_max = abs(l1 - l2), l1 + l2

    out = {
        "target": target, "r": r, "dr": dr, "dz": dz, "d": d,
        "reach": (reach_min, reach_max), "azimuth": azimuth_raw,
        "azimuthIndeterminate": azimuth_indeterminate,
        "candidates": [], "reason": None, "joint": None, "message": None,
    }

    if d > reach_max + 1e-9 or d < reach_min - 1e-9:
        out["reason"] = "OUT_OF_WORKSPACE"
        out["message"] = (
            f"腕枢轴到肩枢轴距离 {d:.3f}mm，可达范围 "
            f"{reach_min:.3f}..{reach_max:.3f}mm"
        )
        return out

    cd = min(max(d, reach_min), reach_max)
    cos_a = max(-1.0, min(1.0, (cd * cd - l1 * l1 - l2 * l2) / (2 * l1 * l2)))
    alpha = math.degrees(math.acos(cos_a))
    phi = math.degrees(math.atan2(dr, dz))

    for sign, tag in ((1, "elbow-up"), (-1, "elbow-down")):
        ra = sign * alpha
        ths = phi - math.degrees(
            math.atan2(l2 * math.sin(math.radians(ra)), l1 + l2 * math.cos(math.radians(ra)))
        )
        the = ths + ra
        v_s = max(0.0, sh.limit_min - ths, ths - sh.limit_max)
        v_e = max(0.0, el.limit_min - the, the - el.limit_max)
        out["candidates"].append({
            "branch": tag, "shoulder": ths, "elbow": the,
            "vS": v_s, "vE": v_e, "feasible": max(v_s, v_e) <= 1e-9,
        })

    # base 限位
    if azimuth_indeterminate:
        azimuth_used = 0.0
    else:
        azimuth_used = azimuth_raw
    v_b = 0.0 if azimuth_indeterminate else max(
        0.0, base.limit_min - azimuth_used, azimuth_used - base.limit_max
    )
    out["baseViolation"] = v_b
    out["azimuthUsed"] = azimuth_used

    feasible = [c for c in out["candidates"] if c["feasible"] and v_b <= 1e-9]
    if not feasible:
        if v_b > 1e-9:
            out["reason"] = "JOINT_LIMIT"
            out["joint"] = base.id
        else:
            best = min(out["candidates"], key=lambda c: max(c["vS"], c["vE"]))
            out["reason"] = "JOINT_LIMIT"
            out["joint"] = el.id if best["vE"] >= best["vS"] else sh.id
            out["best"] = best
    else:
        best = min(feasible, key=lambda c: max(c["vS"], c["vE"]))
        out["reason"] = None
        out["chosen"] = best
    return out


def report(robot, label, q, *, l1, l2, pivot_z, pivot_r, toff, targets):
    print("=" * 96)
    print(f"{label}")
    print("=" * 96)
    x, y, z = fk_tcp_mm(robot, q)
    print(f"  关节  base={q['base']:>10.4f}  shoulder={q['shoulder']:>10.4f}  "
          f"elbow={q['elbow']:>10.4f}")
    print(f"  → FK TCP = ({x:.4f}, {y:.4f}, {z:.4f})")
    print()
    for tgt in targets:
        res = solve_like_ik_ts(robot, tgt, l1=l1, l2=l2,
                               pivot_z=pivot_z, pivot_r=pivot_r, toff=toff)
        print(f"  ── 目标 ({tgt[0]}, {tgt[1]}, {tgt[2]}) ──")
        print(f"     r={res['r']:.4f}  dr={res['dr']:.4f}  dz={res['dz']:.4f}  d={res['d']:.4f}"
              f"   reach=[{res['reach'][0]:.3f}, {res['reach'][1]:.3f}]")
        print(f"     azimuth={res['azimuth']:.4f}  base越界={res.get('baseViolation', 0):.6f}")
        for c in res["candidates"]:
            m = "✓" if c["feasible"] else "✗"
            print(f"       {m} {c['branch']:<11} shoulder={c['shoulder']:>12.6f} "
                  f"elbow={c['elbow']:>12.6f}  vS={c['vS']:.6f} vE={c['vE']:.6f}")
        if res["reason"]:
            print(f"     ⇒ 判定 {res['reason']}  关节={res['joint']}")
            if res.get("best"):
                b = res["best"]
                print(f"        最接近的一支 {b['branch']}：差 {max(b['vS'], b['vE']):.6f}°")
        else:
            c = res["chosen"]
            print(f"     ⇒ 成功，选支 {c['branch']}  shoulder={c['shoulder']:.6f} "
                  f"elbow={c['elbow']:.6f}")
        print()


def main() -> int:
    robot = load_robot_by_id(ROBOT_ID)
    j = {x.id: x for x in robot.movable_joints()}
    sh, el = j["shoulder"], j["elbow"]
    home = {k: float(v) for k, v in robot.home_pose.items()}
    pivot_z = robot.link("base_link").length + robot.link("column_link").length
    pivot_r = 0.0
    l1 = robot.link("upper_arm_link").length
    l2 = robot.link("forearm_link").length
    toff = robot.link("tool_link").length

    print("=" * 96)
    print("复现：网页界面 J1/J2/J3 到极限 → 记 XYZ → 回 HOME → 输入同 XYZ")
    print("=" * 96)
    print(f"  真值限位  shoulder {sh.limit_min:.6f}..{sh.limit_max:.6f}   "
          f"elbow {el.limit_min:.6f}..{el.limit_max:.6f}")
    print(f"  几何  pivotZ={pivot_z} pivotR={pivot_r} L1={l1} L2={l2} toolOffset={toff}")
    print(f"  HOME  {home}")
    print()

    print("【先验证：用户报的 J2=49.4 / J3=141.5 与限位上限的关系】")
    print(f"  shoulder 上限 = {sh.limit_max:.6f}  ← 用户说 49.4（下取整 49）")
    print(f"  elbow    上限 = {el.limit_max:.6f}  ← 用户说 141.5（下取整 141）")
    print()

    # 用户报告的点：J2=49.4, J3=141.5，J1 待定
    print("=" * 96)
    print("扫描 J1 取值，找哪个能给出 (97, 157, 49)")
    print("=" * 96)
    best_hit = None
    for b10 in range(-600, 601, 1):
        b = b10 / 10.0
        q = dict(home)
        q["base"], q["shoulder"], q["elbow"] = b, 49.4, 141.5
        x, y, z = fk_tcp_mm(robot, q)
        err = math.dist((x, y, z), (97.0, 157.0, 49.0))
        if best_hit is None or err < best_hit[0]:
            best_hit = (err, b, x, y, z)
    err, b, x, y, z = best_hit
    print(f"  最佳匹配 J1 = {b:+.1f}°  →  TCP = ({x:.4f}, {y:.4f}, {z:.4f})")
    print(f"  与用户报的 (97, 157, 49) 偏差 = {err:.4f} mm")
    print()

    # 用 49.4 / 141.5 精确复算
    q = dict(home)
    q["base"], q["shoulder"], q["elbow"] = b, 49.4, 141.5
    report(robot, f"★ 场景复现：J1={b:+.1f} J2=49.4 J3=141.5", q,
           l1=l1, l2=l2, pivot_z=pivot_z, pivot_r=pivot_r, toff=toff,
           targets=[(97.0, 157.0, 49.0)])

    # 再试：用 J2/J3 取限位上限（网页滑杆的真实端点）
    q2 = dict(home)
    q2["base"], q2["shoulder"], q2["elbow"] = b, sh.limit_max, el.limit_max
    report(robot, f"对照：J1={b:+.1f} J2={sh.limit_max:.4f} J3={el.limit_max:.4f}", q2,
           l1=l1, l2=l2, pivot_z=pivot_z, pivot_r=pivot_r, toff=toff,
           targets=[(97.0, 157.0, 49.0)])

    # 关键：把 FK 产出的**精确** TCP 喂回去
    xt, yt, zt = fk_tcp_mm(robot, q2)
    print("=" * 96)
    print("关键对照：FK 产出的**精确** TCP 喂回 IK")
    print("=" * 96)
    report(robot, f"J1={b:+.1f} J2={sh.limit_max:.4f} J3={el.limit_max:.4f} 的精确 TCP",
           q2, l1=l1, l2=l2, pivot_z=pivot_z, pivot_r=pivot_r, toff=toff,
           targets=[(xt, yt, zt), (97.0, 157.0, 49.0)])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
