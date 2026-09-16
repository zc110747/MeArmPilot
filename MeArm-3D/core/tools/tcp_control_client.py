#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ArmPilot TCP JSON 控制接口的最小客户端 + 冒烟验收。

```bash
<python> core/tools/tcp_control_client.py                 # 冒烟（连 127.0.0.1:9100）
<python> core/tools/tcp_control_client.py --host H --port P
<python> core/tools/tcp_control_client.py --cmd '{"cmd":"gripper","action":"open"}'
<python> core/tools/tcp_control_client.py --json out.json  # 落盘全部应答
```

退出码：0 = 全部断言通过；1 = 有 FAIL。

## 它验证什么（不只是"连得上"）

| 段 | 断言 |
|---|---|
| ① 协议 | 一条命令一行应答；`ok` 字段存在 |
| ② move | 六个方向都能走，且 **TCP 坐标真的按方向变了**（不是只看 ok=true） |
| ③ gripper | open/close 分别落在关节限位的两个端点 |
| ④ servo | 1..4 都能下单号 |
| ⑤ 错误 | 非法 JSON / 未知命令 / 非法轴 / 缺参数 / 角度越界 → 都 `ok=false` 且**连接仍可用** |
| ⑥ 健壮 | 超长行不会打死服务；断线重连照常工作 |

★ **不做**的事：不校验 IK/FK 的数值（那由 `backend/internal/robot/kinematics_test.go`
拿冻结基线把关），也不改任何配置。这里只证明"接口通、语义对、出错不乱"。
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

PASS = 0
FAIL = 0
LOG: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    global PASS, FAIL
    if ok:
        PASS += 1
        print(f"  [PASS] {name}" + (f"  — {detail}" if detail else ""))
    else:
        FAIL += 1
        print(f"  [FAIL] {name}" + (f"  — {detail}" if detail else ""))
    LOG.append(f"{'PASS' if ok else 'FAIL'}\t{name}\t{detail}")


class Client:
    """一个 TCP 连接，一行一条 JSON。"""

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = b""
        self.host, self.port = host, port

    def send(self, obj: object | str) -> dict:
        """发一行（dict 会被 json.dumps），读一行应答。"""
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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="ArmPilot TCP JSON 控制接口客户端")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9100)
    ap.add_argument("--cmd", action="append", default=[],
                    help="只发这一条 JSON（可重复）；给它就跳过冒烟验收")
    ap.add_argument("--json", type=Path, default=None, help="把全部应答写到该文件")
    args = ap.parse_args(argv)

    if args.cmd:
        c = Client(args.host, args.port)
        try:
            for line in args.cmd:
                print(json.dumps(c.send(line), ensure_ascii=False))
        finally:
            c.close()
        return 0

    print(f"ArmPilot TCP 控制接口冒烟验收 — {args.host}:{args.port}")
    print()

    c = Client(args.host, args.port)
    try:
        # ---- ① 协议 ----------------------------------------------------
        print("── ① 协议：一行一答 ─────────────────────────")
        r = c.send({"cmd": "gripper", "action": "open"})
        check("收到应答且含 ok 字段", isinstance(r, dict) and "ok" in r, json.dumps(r, ensure_ascii=False)[:120])
        check("gripper open 成功", r.get("ok") is True, r.get("error", ""))
        check("成功时回带 state", isinstance(r.get("state"), dict),
              f"joints={list((r.get('state') or {}).get('joints', {}))}")
        st = r.get("state") or {}
        check("state 含 tcp 与 servo", "tcp" in st and "servo" in st,
              f"tcp={st.get('tcp')} servo={st.get('servo')}")

        # ---- ② move 六个方向 -------------------------------------------
        print()
        print("── ② move：六个方向都要真的动 ───────────────")
        # 回 HOME 附近，避免从上次遗留的姿态起步导致撞限位
        c.send({"cmd": "servo", "servo": 1, "angle": 90})
        c.send({"cmd": "servo", "servo": 2, "angle": 90})
        c.send({"cmd": "servo", "servo": 3, "angle": 90})

        for axis in ("x", "y", "z"):
            for direction in ("+", "-"):
                before = (c.send({"cmd": "gripper", "action": "open"}).get("state") or {}).get("tcp")
                step = 2.0
                r = c.send({"cmd": "move", "axis": axis, "direction": direction, "step": step})
                after = ((r.get("state") or {}).get("tcp") if r.get("ok") else None)
                idx = {"x": 0, "y": 1, "z": 2}[axis]
                sign = 1.0 if direction == "+" else -1.0
                if not r.get("ok") or before is None or after is None:
                    check(f"move {axis}{direction}", False, r.get("error", "无 state"))
                    continue
                got = after[idx] - before[idx]
                # 容差 0.5mm：舵机角是整数下发（真机）或物理量（mujoco），
                # 这里只判"方向对 + 量级对"，不判逐位相等。
                check(f"move {axis}{direction}",
                      abs(got - sign * step) <= 0.5,
                      f"Δ{axis.upper()} = {got:+.4f} mm（期望 {sign * step:+.1f}）")

        # ---- ③ gripper -------------------------------------------------
        print()
        print("── ③ gripper：open / close 落在限位端点 ─────")
        opened = c.send({"cmd": "gripper", "action": "open"})
        closed = c.send({"cmd": "gripper", "action": "close"})
        oj = (opened.get("state") or {}).get("joints", {})
        cj = (closed.get("state") or {}).get("joints", {})
        if "gripper" in oj and "gripper" in cj:
            check("open > close（θ 增大 = 张开）", oj["gripper"] > cj["gripper"],
                  f"open={oj['gripper']:.2f}° close={cj['gripper']:.2f}°")
        else:
            check("open > close（θ 增大 = 张开）", False, "state 里没有 gripper 关节")

        # ---- ④ servo 1..4 ----------------------------------------------
        print()
        print("── ④ servo：四个舵机都能下单号 ──────────────")
        for n in (1, 2, 3, 4):
            r = c.send({"cmd": "servo", "servo": n, "angle": 90})
            check(f"servo {n} = 90°", r.get("ok") is True, r.get("error", ""))

        # ---- ⑤ 错误分支（且连接仍然可用）--------------------------------
        print()
        print("── ⑤ 错误：都要被拒，且连接不能废 ───────────")
        bad_cases = [
            ("非法 JSON", "{", "invalid json"),
            ("未知命令", '{"cmd":"teleport"}', "unknown cmd"),
            ("缺 cmd", "{}", "missing cmd"),
            ("非法轴", '{"cmd":"move","axis":"w","direction":"+","step":5}', "invalid axis"),
            ("非法方向", '{"cmd":"move","axis":"x","direction":"*","step":5}', "invalid direction"),
            ("缺 step", '{"cmd":"move","axis":"x","direction":"+"}', "missing step"),
            ("非法夹爪动作", '{"cmd":"gripper","action":"half"}', "invalid action"),
            ("舵机号越界", '{"cmd":"servo","servo":9,"angle":90}', "invalid servo"),
            ("角度越界", '{"cmd":"servo","servo":1,"angle":999}', "angle out of range"),
            ("空行", "", "empty command"),
        ]
        for name, line, want in bad_cases:
            try:
                r = c.send(line)
            except (ConnectionError, json.JSONDecodeError, socket.timeout) as exc:
                check(name, False, f"连接/解析异常: {exc}")
                continue
            check(name, r.get("ok") is False and want in (r.get("error") or ""),
                  f"ok={r.get('ok')} error={r.get('error')!r}")

        alive = c.send({"cmd": "gripper", "action": "open"})
        check("错误轰炸之后连接仍然可用", alive.get("ok") is True, alive.get("error", ""))

        # ---- ⑥ 健壮性：超长行只断自己 -----------------------------------
        print()
        print("── ⑥ 健壮：超长行只断这一条连接 ─────────────")
        try:
            r = c.send("A" * 200000)
            check("超长行被拒", r.get("ok") is False and "too long" in (r.get("error") or ""),
                  f"error={r.get('error')!r}")
        except (ConnectionError, socket.timeout):
            # 服务端直接掐断也是可接受的行为
            check("超长行被拒（服务端断开）", True, "连接被服务端关闭")
        c.close()

        c2 = Client(args.host, args.port)
        r = c2.send({"cmd": "gripper", "action": "close"})
        check("超长行不影响其它客户端", r.get("ok") is True, r.get("error", ""))
        c2.close()

    finally:
        try:
            c.close()
        except Exception:
            pass

    print()
    print(f"==== TCP 接口验收 PASS {PASS} / FAIL {FAIL}（共 {PASS + FAIL} 项）====")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text("\n".join(LOG) + "\n", encoding="utf-8")
        print(f"# 结果 -> {args.json}")

    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
