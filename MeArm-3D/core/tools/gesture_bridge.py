#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArmPilot TCP 控制协议的**最小参考客户端**（只依赖标准库）。

给外部项目（visionflow 的手势控制）当接入样板用 —— 直接抄这个类即可，
不必引入本工程的任何东西。协议本身见 `docs/protocol/`。

```bash
<python> core/tools/gesture_bridge.py --demo     # 跑一遍自检（需要后端在 9100）
<python> core/tools/gesture_bridge.py --caps     # 只打印能力描述
```

## 它示范的四件事

| 事 | 做法 |
|---|---|
| ① 建立基准 | 连上先 `caps` + `home`，**不要**上来就下发 |
| ② 坐标映射 | 手势的归一化坐标 → mm，边界**全部取自 caps**，不抄常量 |
| ③ 丢帧策略 | 零增量**跳过**不下发；命令被拒时保留上一个好点，不硬冲 |
| ④ 闭环 | 每次下发后读回 `state.tcp`，用它而不是本地推算值做下一步基准 |

## 两种映射路线

- **绝对（`goto`）**：手的位置 → 臂的位置。适合"指哪打哪"，但要处理不可达。
- **增量（`nudge`）**：手的位移 → 臂的位移。适合"手往左，臂往左"，天然跟手。

⚠️ `state.tcp` 是**命令值**（最近一次被受理的目标），不是设备过程值。
做闭环时拿它当基准是对的（这正是相对命令所用的基准），
但不要拿它当"设备已经到位了"。
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import sys
from typing import Sequence

HOST = "127.0.0.1"
PORT = 9100

# 相对命令的最小位移（mm）。低于它就别发 —— 服务端会把全零矢量当参数错误拒绝，
# 而抖动噪声会让每帧都产生一个没意义的微小增量。
MIN_DELTA_MM = 0.5


class ArmLink:
    """一个 TCP 连接，一行一条 JSON。"""

    def __init__(self, host: str = HOST, port: int = PORT, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = b""

    def send(self, obj: object | str) -> dict:
        line = obj if isinstance(obj, str) else json.dumps(obj)
        self.sock.sendall((line + "\n").encode("utf-8"))
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("服务端关闭了连接")
            self.buf += chunk
        raw, _, self.buf = self.buf.partition(b"\n")
        return json.loads(raw.decode("utf-8"))

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class GestureBridge:
    """手势 → 机械臂。用法：

    ```python
    b = GestureBridge()
    b.open()                       # caps + home
    b.goto_norm(0.5, 0.5, 0.5)     # 归一化坐标 → 臂
    b.nudge_mm(5, 0, -2)           # 或者直接给 mm 增量
    b.close()
    ```
    """

    def __init__(self, host: str = HOST, port: int = PORT):
        self.link = ArmLink(host, port)
        self.caps: dict = {}
        self.last_good: list[float] | None = None  # 最近一次**成功**的目标点

    # ---- 生命周期 -----------------------------------------------------
    def open(self) -> dict:
        """连上以后的第一件事：拿能力 + 归位。"""
        r = self.link.send({"cmd": "caps"})
        if not r.get("ok") or not r.get("caps"):
            raise RuntimeError(f"caps 失败: {r.get('error')}")
        self.caps = r["caps"]
        r = self.link.send({"cmd": "home"})
        if not r.get("ok"):
            raise RuntimeError(f"home 失败: {r.get('error')}")
        self.last_good = (r.get("state") or {}).get("tcp")
        return self.caps

    def close(self) -> None:
        self.link.close()

    # ---- 坐标映射 -----------------------------------------------------
    def norm_to_mm(self, nx: float, ny: float, nz: float) -> list[float]:
        """归一化 [0,1]³ → 机械臂 mm。边界取自 `caps.workspace`，**不抄常量**。

        ⚠️ 这只是**粗映射**：workspace 是可达域的包围盒，盒内仍可能不可达
        （可达域是被限位切过的甜甜圈壳层）。所以映射完必须走 `goto()`，
        由它用命令的 `ok` 做最终判据。
        """
        ws = self.caps.get("workspace") or {}
        lo, hi = ws.get("min") or [0, 0, 0], ws.get("max") or [0, 0, 0]
        return [
            lo[i] + (hi[i] - lo[i]) * min(1.0, max(0.0, n)) for i, n in enumerate((nx, ny, nz))
        ]

    # ---- 下发 ---------------------------------------------------------
    def goto(self, xyz: Sequence[float]) -> bool:
        """绝对定位到 [x,y,z]（mm）。失败时**保留上一个好点**并返回 False。

        为什么不硬冲：手势识别本身有抖动，偶尔吐出一个不可达点是常态，
        把它当致命错误会让整条交互断掉。返回 False 让调用方决定要不要告警。
        """
        r = self.link.send({"cmd": "moveto", "xyz": list(xyz)})
        if not r.get("ok"):
            return False
        self.last_good = (r.get("state") or {}).get("tcp") or list(xyz)
        return True

    def goto_norm(self, nx: float, ny: float, nz: float) -> bool:
        return self.goto(self.norm_to_mm(nx, ny, nz))

    def nudge(self, delta: Sequence[float]) -> bool:
        """三轴同时相对位移（mm）。零增量自动跳过（返回 True，表示"无需动作"）。"""
        d = [float(x) for x in delta]
        if math.hypot(*d) < MIN_DELTA_MM:
            return True  # 手势静止 —— 不下发，也不算失败
        r = self.link.send({"cmd": "movexyz", "delta": d})
        if not r.get("ok"):
            return False
        self.last_good = (r.get("state") or {}).get("tcp")
        return True

    def nudge_mm(self, dx: float, dy: float, dz: float) -> bool:
        return self.nudge((dx, dy, dz))

    @property
    def tcp(self) -> list[float] | None:
        """读当前 TCP（发一条只读 `state`，不产生任何运动）。"""
        r = self.link.send({"cmd": "state"})
        return (r.get("state") or {}).get("tcp") if r.get("ok") else None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="ArmPilot 手势控制参考客户端")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--caps", action="store_true", help="只打印能力描述")
    ap.add_argument("--demo", action="store_true", help="跑一遍自检")
    args = ap.parse_args(argv)
    fail = 0

    def check(name: str, ok: bool, detail: str = "") -> None:
        nonlocal fail
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  — {detail}" if detail else ""))
        if not ok:
            fail += 1

    b = GestureBridge(args.host, args.port)
    try:
        caps = b.open()
        print("── caps ──")
        print(json.dumps(caps, ensure_ascii=False, indent=2))
        if args.caps:
            return 0
        if not args.demo:
            return 0

        print()
        print("── 自检 ──")
        check("home 之后有 TCP", b.tcp is not None, f"tcp={b.tcp}")

        # ① 归一化映射 → 绝对定位（中心点，必然在包围盒正中）
        ok = b.goto_norm(0.5, 0.5, 0.5)
        want = b.norm_to_mm(0.5, 0.5, 0.5)
        got = b.tcp
        check("goto_norm(中心)", ok and got is not None
              and all(abs(got[i] - want[i]) <= 0.5 for i in range(3)),
              f"tcp={got} 期望 {[round(v, 2) for v in want]}")

        # ② 绝对语义幂等：再发一次同一点必须停在原地
        before = b.tcp
        b.goto_norm(0.5, 0.5, 0.5)
        after = b.tcp
        check("goto 幂等", before is not None and after is not None
              and all(abs(before[i] - after[i]) <= 1e-6 for i in range(3)),
              f"{before} → {after}")

        # ③ 增量：三轴同时
        before = b.tcp
        d = [3.0, -2.0, 1.0]
        ok = b.nudge_mm(*d)
        after = b.tcp
        check("nudge 三轴同时", ok and before is not None and after is not None
              and all(abs((after[i] - before[i]) - d[i]) <= 0.5 for i in range(3)),
              f"Δ={[round(after[i] - before[i], 3) for i in range(3)] if after and before else None} "
              f"期望 {d}")

        # ④ 零增量必须被**跳过**（不是报错）
        before = b.tcp
        ok = b.nudge_mm(0, 0, 0)
        after = b.tcp
        check("零增量被跳过（不下发、不报错）", ok and before == after, f"{before} → {after}")

        # ⑤ 不可达点：返回 False 且**保留上一个好点**，不硬冲
        good_before = b.last_good
        ok = b.goto([0.0, 0.0, 999.0])
        check("不可达点返回 False 且保留旧点",
              ok is False and b.last_good == good_before, f"last_good={b.last_good}")

        # ⑥ 归位
        r = b.link.send({"cmd": "home"})
        home_tcp = (r.get("state") or {}).get("tcp")
        check("home 回到 HOME 的 TCP", r.get("ok") is True and home_tcp is not None,
              f"tcp={[round(v, 2) for v in home_tcp]}" if home_tcp else r.get("error", ""))
    finally:
        b.close()

    print()
    print(f"==== 手势桥自检 {'PASS' if fail == 0 else f'FAIL {fail}'} ====")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
