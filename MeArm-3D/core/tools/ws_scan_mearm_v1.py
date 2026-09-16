#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mearm-v1 工作空间扫描：J1/J2/J3 网格 → XYZ 实际范围。

**目的**：把「IK 反解给出的限位」与「真机姿态能到的限位」之间的差距量化出来。

数据来源要求（本项目铁律）：
* 几何量一律从 `robot-package/mearm-v1/model/robot.yaml` 读，**不硬编码**；
* 走 `simulation/mujoco/fkref.py`（**独立参考实现**），不用自研解析器；
* 输出只做**统计**，不下判断。

用法：
    <python> core/tools/ws_scan_mearm_v1.py               # 默认 1° 网格
    <python> core/tools/ws_scan_mearm_v1.py --step 2      # 粗扫
    <python> core/tools/ws_scan_mearm_v1.py --json out.json
"""
from __future__ import annotations

import argparse
import json
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

from fkref import fk_tcp_mm                                   # noqa: E402
from robotcfg import load_robot_by_id                          # noqa: E402
from units import ensure_utf8_stdout                           # noqa: E402

ensure_utf8_stdout()

ROBOT_ID = "mearm-v1"


def frange(a: float, b: float, step: float):
    n = int(round((b - a) / step))
    for i in range(n + 1):
        yield a + i * step


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="mearm-v1 工作空间扫描")
    ap.add_argument("--step", type=float, default=1.0, help="关节网格步长（度）")
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args(argv)

    robot = load_robot_by_id(ROBOT_ID)
    j = {x.id: x for x in robot.movable_joints()}
    base, sh, el = j["base"], j["shoulder"], j["elbow"]
    print(f"[ws] {ROBOT_ID} · 关节限位（真值 robot.yaml）")
    for k in ("base", "shoulder", "elbow", "gripper"):
        jj = j[k]
        print(f"     {k:<9} {jj.limit_min:>10.4f} .. {jj.limit_max:>10.4f}")
    print(f"[ws] 网格步长 {args.step}°，HOME={robot.home_pose}")

    home = {k: float(v) for k, v in robot.home_pose.items()}
    step = args.step
    n_base = int(round((base.limit_max - base.limit_min) / step))
    n_sh = int(round((sh.limit_max - sh.limit_min) / step))
    n_el = int(round((el.limit_max - el.limit_min) / step))
    total = (n_base + 1) * (n_sh + 1) * (n_el + 1)
    print(f"[ws] 采样点 {total} = {n_base + 1}×{n_sh + 1}×{n_el + 1}")

    xs, ys, zs, rs = [], [], [], []
    # 同时记录"以 HOME 姿态为参照的矢状面可达壳"（J1=0 平面内）
    sag = []  # (dr, dz)  腕枢轴相对肩枢轴
    for b in frange(base.limit_min, base.limit_max, step):
        for s in frange(sh.limit_min, sh.limit_max, step):
            for e in frange(el.limit_min, el.limit_max, step):
                q = dict(home)
                q["base"], q["shoulder"], q["elbow"] = b, s, e
                x, y, z = fk_tcp_mm(robot, q)
                xs.append(x); ys.append(y); zs.append(z)
                rs.append((x * x + y * y) ** 0.5)
                if abs(b) < 1e-9:
                    sag.append((x, z))

    def rng(v, tol: float = 1.0):
        """[min, max]；`tol` 用于「接近某条件」的子集（空集返回 (None, None)）。"""
        v = list(v)
        return (min(v), max(v)) if v else (None, None)

    res = {
        "robotId": ROBOT_ID,
        "stepDeg": step,
        "samples": total,
        "limits": {k: {"min": j[k].limit_min, "max": j[k].limit_max}
                   for k in ("base", "shoulder", "elbow", "gripper")},
        "tcp": {
            "x": rng(xs), "y": rng(ys), "z": rng(zs),
            "r": rng(rs),
            "zAtR0": rng([z for r, z in zip(rs, zs) if r < 1.0]),
            "zAtR1": rng([z for r, z in zip(rs, zs) if r < 2.0]),
        },
        "sagittalXZ_j1_zero": {"x": rng([p[0] for p in sag]), "z": rng([p[1] for p in sag])},
    }

    def fmt_rng(t, unit="mm"):
        if t[0] is None:
            return "（该网格下无采样点）"
        return f"{t[0]:>10.3f} .. {t[1]:>10.3f}  {unit}"

    print("\n=== TCP 实际可达范围（J1/J2/J3 全网格）===")
    print(f"  X: {fmt_rng(res['tcp']['x'])}")
    print(f"  Y: {fmt_rng(res['tcp']['y'])}")
    print(f"  Z: {fmt_rng(res['tcp']['z'])}")
    print(f"  R=hypot(X,Y): {fmt_rng(res['tcp']['r'])}")
    print(f"  R<2 段 Z: {fmt_rng(res['tcp']['zAtR1'])}")
    print("\n=== J1=0 矢状面（X-Z，爪 40mm 已含）===")
    print(f"  X: {fmt_rng(res['sagittalXZ_j1_zero']['x'])}")
    print(f"  Z: {fmt_rng(res['sagittalXZ_j1_zero']['z'])}")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"\n[ws] → {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
