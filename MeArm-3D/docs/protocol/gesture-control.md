# 手势控制接入指南（visionflow）

> 面向 **visionflow** 项目：把摄像头识别出的手势，映射到 ArmPilot 机械臂的
> `(x, y, z)` 运动。
>
> 协议本身见 [`README.md`](./README.md) 与 [`tcp-xyz-v2.md`](./tcp-xyz-v2.md)；
> 本文只讲**接入套路**和必须避开的坑。
>
> 可直接运行的参考客户端：`core/tools/gesture_bridge.py`（只依赖标准库，
> `python core/tools/gesture_bridge.py --demo` 自检 7 项全绿）。

---

## 1. 会话流程

```
连上 9100
  │
  ├─① {"cmd":"caps"}   → 拿几何 / 限位 / 可达包围盒（坐标映射要用）
  ├─② {"cmd":"home"}   → 归位，建立已知基准
  │
  └─③ 循环（10~20 Hz）
        ├─ 手势 → 归一化坐标 或 位移增量
        ├─ 映射到 mm（边界取自 caps，不抄常量）
        └─ {"cmd":"movexyz","delta":[dx,dy,dz]}   增量式
           或 {"cmd":"moveto","xyz":[x,y,z]}      绝对式
           └─ 读回 state.tcp 当下一帧基准
```

**别跳过 ① ②。** 舵机控制是绝对角语义，没有基准的第一帧就是一次跳变；
坐标映射若抄死边界，`robot.yaml` 一改就静默错位。

---

## 2. 两条映射路线

### 2.1 增量式（`movexyz`）—— 推荐先做这个

"手往左，臂往左"。天然跟手，且**不需要知道手在画面里的绝对位置**。

```python
delta_mm = (hand_now - hand_prev) * gain     # 手势位移 → mm 位移
if hypot(*delta_mm) < 0.5:  skip            # 静止帧不下发
send({"cmd":"movexyz","delta":list(delta_mm)})
```

- `gain`（mm/归一化单位）建议从 `caps.workspace` 的跨度推：
  `gain = (ws.max[i] - ws.min[i]) / 手势在画面里的活动跨度`。
- 优点：不会被"包围盒内其实不可达"坑到 —— 增量小，越界时命令被拒，
  下一帧从**当前**位置继续，自然沿着可达域边界滑动。

### 2.2 绝对式（`moveto`）—— "指哪打哪"

"手指向哪里，臂就去哪里"。需要把手的归一化坐标映射到 mm：

```python
def norm_to_mm(caps, nx, ny, nz):
    lo, hi = caps["workspace"]["min"], caps["workspace"]["max"]
    return [lo[i] + (hi[i] - lo[i]) * clamp01(n[i]) for i in range(3)]
```

> ⚠️ `workspace` 是可达域的**包围盒**，可达域是被关节限位切过的**甜甜圈壳层**
> —— 盒**内**仍可能不可达（中间是空的）。所以它只能做**粗钳位**
> （盒外一定不可达），真正的判据是命令的 `ok`。

**不可达时不要硬冲**：保留上一个成功的点，等手势移回可达区。

```python
ok = bridge.goto(target)
if not ok:
    ...  # 保留 last_good，不更新基准；可给个视觉反馈
```

---

## 3. 必须避开的坑

| # | 坑 | 表现 | 做法 |
|---|----|------|------|
| 1 | **全零 delta** | 服务端按参数错误拒绝（`invalid delta: zero displacement`） | 客户端跳过静止帧，**别**把它当故障 |
| 2 | `state.tcp` 是**命令值** | 命令刚下发就读，读到的是目标不是实际位置 | 拿它当**基准**是对的（相对命令用的就是这个基准）；但别当"已到位" |
| 3 | 服务端**默认不节流** | `min_send_interval_ms: 0`，发太快会让命令互相 latest-wins | 客户端自己限到 **10~20 Hz** |
| 4 | 包围盒内可能不可达 | `moveto` 报 `OUT_OF_WORKSPACE` / `JOINT_LIMIT` | 增量式天然规避；绝对式要保留上一个好点 |
| 5 | 目标 == 当前位置 | 零位移 ⇒ 零状态帧 ⇒ "等收敛"拿到空/NaN | 先判位移量再发 |
| 6 | 边界抄进客户端 | `robot.yaml` 一改就静默错位 | 一律从 `caps` 取 |
| 7 | 真机无位置反馈 | `joint_state` 是开环目标值，链路全绿**不证明**物理到位 | 真机验收要相机证据 |

---

## 4. 最小客户端（可直接抄）

完整版见 `core/tools/gesture_bridge.py`。骨架：

```python
import json, math, socket

class ArmLink:
    def __init__(self, host="127.0.0.1", port=9100, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = b""
    def send(self, obj):
        line = obj if isinstance(obj, str) else json.dumps(obj)
        self.sock.sendall((line + "\n").encode("utf-8"))
        while b"\n" not in self.buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("服务端关闭了连接")
            self.buf += chunk
        raw, _, self.buf = self.buf.partition(b"\n")
        return json.loads(raw.decode("utf-8"))

class GestureBridge:
    MIN_DELTA_MM = 0.5
    def __init__(self):
        self.link = ArmLink()
        self.caps = self.link.send({"cmd": "caps"})["caps"]
        self.last_good = self.link.send({"cmd": "home"})["state"]["tcp"]

    def norm_to_mm(self, nx, ny, nz):
        lo, hi = self.caps["workspace"]["min"], self.caps["workspace"]["max"]
        return [lo[i] + (hi[i] - lo[i]) * min(1.0, max(0.0, n))
                for i, n in enumerate((nx, ny, nz))]

    def goto(self, xyz) -> bool:
        r = self.link.send({"cmd": "moveto", "xyz": list(xyz)})
        if not r.get("ok"):
            return False                      # 保留上一个好点，不硬冲
        self.last_good = r["state"]["tcp"]
        return True

    def nudge(self, delta) -> bool:
        d = [float(x) for x in delta]
        if math.hypot(*d) < self.MIN_DELTA_MM:
            return True                       # 静止帧：跳过，不算失败
        r = self.link.send({"cmd": "movexyz", "delta": d})
        if not r.get("ok"):
            return False
        self.last_good = r["state"]["tcp"]
        return True
```

---

## 5. 命令选择速查

| 手势语义 | 命令 |
|---------|------|
| 手往某方向移动 | `movexyz`（增量） |
| 手停在某个位置 | `moveto`（绝对） |
| 握拳 / 张开 | `gripper` `close` / `open` |
| 张开手掌（复位） | `home` |
| 需要边界做坐标映射 | `caps`（连上时取一次即可） |

---

## 6. 自测

```bash
# ① 起后端（sim 链路，不会动真机）
cd backend && ./bin/armpilot-backend.exe -c config.yaml

# ② 协议冒烟（42 项）
python core/tools/tcp_control_client.py

# ③ 手势桥自检（7 项：映射 / 幂等 / 三轴增量 / 零增量跳过 / 不可达保留 / 归位）
python core/tools/gesture_bridge.py --demo
```

> ★ 上面的计数现跑现取。改完任何一端先重跑，别信"上次是绿的"。
