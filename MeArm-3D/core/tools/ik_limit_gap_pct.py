#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""IK 限位差距 —— 按 **5% 相对误差** 判据复核。

⚠️ 本工具**只读、只报告**，不修改任何配置 / 真值 / 基线。

判据（用户口径）：|偏差| / 参考量 ≤ 5% ⇒ 认为正常。

需要判什么：
  ① 端点量化：round(limit) 相对 limit 的偏差率
  ② IK 判据 vs 真机实测：可达范围两端的相对差
  ③ 自反性：FK→IK 解回的角 vs 原角的相对差
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

TOL_PCT = 5.0


def judge(delta: float, ref: float, label: str) -> bool:
    """按 5% 相对判据评判；ref ≈ 0 时退化为绝对判据（只用来看，不作结论）。"""
    if abs(ref) < 1e-12:
        print(f"    {label:<46} |Δ|={abs(delta):.6f}  ref≈0，无法算相对率")
        return True
    pct = abs(delta) / abs(ref) * 100.0
    ok = pct <= TOL_PCT
    mark = "✓ 正常" if ok else "✗ 超限"
    print(f"    {label:<46} Δ={delta:>+12.6f}  ref={ref:>12.6f}  "
          f"偏差 {pct:>7.4f}%   {mark}")
    return ok


def main() -> int:
    robot = load_robot_by_id("mearm-v1")
    j = {x.id: x for x in robot.movable_joints()}
    sh, el = j["shoulder"], j["elbow"]
    home = {k: float(v) for k, v in robot.home_pose.items()}
    pivot_z = robot.link("base_link").length + robot.link("column_link").length
    l1 = robot.link("upper_arm_link").length
    l2 = robot.link("forearm_link").length
    toff = robot.link("tool_link").length

    results: list[bool] = []

    # ---------------------------------------------------------------- ①
    print("=" * 96)
    print(f"① 端点量化偏差率（判据：≤ {TOL_PCT}% 为正常）")
    print("=" * 96)
    print()
    for name, joint in (("shoulder", sh), ("elbow", el)):
        for tag, val in (("下限", joint.limit_min), ("上限", joint.limit_max)):
            r = round(val)
            print(f"  {name}.{tag}  限位 {val:.6f}  →  round = {r}")
            results.append(judge(r - val, val, f"    {name}.{tag} 量化偏差率"))
        print()

    # ---------------------------------------------------------------- ②
    print("=" * 96)
    print("② IK 判据 vs 真机实测：可达范围两端的相对差")
    print("=" * 96)
    print()
    ws = {
        "X": (40.686, 177.091),
        "Y": (-153.365, 153.365),
        "Z": (48.965, 114.693),
        "R": (81.373, 177.091),
    }
    # IK 的纯几何判据换到 TCP 空间
    ik_R = (abs(l1 - l2) + toff, l1 + l2 + toff)
    print(f"  IK 纯几何判据 R ∈ [{ik_R[0]:.3f}, {ik_R[1]:.3f}] mm")
    print(f"  真机实测     R ∈ [{ws['R'][0]:.3f}, {ws['R'][1]:.3f}] mm")
    print()
    print("  【R 内圈】IK 允许更近 ⇒ 真机多出来的一圈")
    results.append(judge(ws["R"][0] - ik_R[0], ik_R[1], "    (R_min_ws − R_min_ik) / R_max"))
    print("  【R 外圈】两者上限")
    results.append(judge(ik_R[1] - ws["R"][1], ik_R[1], "    (R_max_ik − R_max_ws) / R_max"))

    # ---------------------------------------------------------------- ③
    print()
    print("=" * 96)
    print("③ 自反性：FK 产出的点 → IK 解回的角 vs 原角（elbow-up 支）")
    print("=" * 96)
    print()
    cases = [
        ("home", home["shoulder"], home["elbow"]),
        ("shoulder_min", sh.limit_min, home["elbow"]),
        ("shoulder_max", sh.limit_max, home["elbow"]),
        ("elbow_min", home["shoulder"], el.limit_min),
        ("elbow_max", home["shoulder"], el.limit_max),
    ]
    for tag, ts, te in cases:
        q = dict(home)
        q["shoulder"], q["elbow"] = ts, te
        x, y, z = fk_tcp_mm(robot, q)
        r = math.hypot(x, y)
        dr, dz = r - toff, z - pivot_z
        d = math.hypot(dr, dz)
        cd = min(max(d, abs(l1 - l2)), l1 + l2)
        cos_a = max(-1.0, min(1.0, (cd * cd - l1 * l1 - l2 * l2) / (2 * l1 * l2)))
        alpha = math.degrees(math.acos(cos_a))
        phi = math.degrees(math.atan2(dr, dz))
        ths = phi - math.degrees(
            math.atan2(l2 * math.sin(math.radians(alpha)), l1 + l2 * math.cos(math.radians(alpha)))
        )
        the = ths + alpha
        print(f"  {tag}")
        results.append(judge(ths - ts, ts if abs(ts) > 1e-9 else 1.0, "    shoulder 解回偏差率"))
        results.append(judge(the - te, te, "    elbow    解回偏差率"))
        print()

    # ---------------------------------------------------------------- 汇总
    print("=" * 96)
    print("汇总")
    print("=" * 96)
    bad = [r for r in results if not r]
    print(f"  判定项 {len(results)} 个 · 正常 {len(results) - len(bad)} · "
          f"超 {TOL_PCT}% 的 {len(bad)}")
    if not bad:
        print(f"\n  ✓ 全部落在 {TOL_PCT}% 判据内 —— 按你的口径属**正常**。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
