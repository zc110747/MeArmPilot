# ArmPilot Serial Protocol v1

> **状态：§4 的固件侧 `JR`/`STATE` 仍待实现（Phase 9）；§5 的 Browser ↔ Go 侧已随
> Phase 8 落地并验收。** 本文是 Phase 9–12 的协议设计基线，供 `MeArm-Device`（AVR 固件）与
> `MeArm-RemoteControl`（Go 服务）同步扩展时对齐。Phase 1–7 完全不依赖本协议
> （Phase 7 走 `MockTransport`，不碰串口）。

## 1. 分层原则（不可破坏）

```
RobotModel  ──  唯一几何/限位来源（config/robot.yaml）
JointState  ──  唯一状态语言（关节角，degree）
Calibration ──  关节角 → 舵机角（offset / scale / reverse）
Protocol    ──  只负责把舵机角编成字节，不认识关节、不认识 IK
Transport   ──  只负责搬字节，不认识协议语义
```

**铁律**
- IK 永远只输出 `JointState`，绝不输出舵机角 / PWM / 通道号。
- 协议层不认识"关节"，只认识"舵机 ID + 角度"。
- 标定表只有一份（`config/robot.yaml`），固件与前端不得各存一份。

## 2. 关节 ↔ 舵机映射（v1 固定，来自 robot.yaml）

> ⚠️ **2026-09-12 实测修正**：舵机号与运动学角色**不是**一一对应的直觉关系。
> 早期版本按固件 `SERVO_LEFT/RIGHT` 的命名推定 `shoulder→S8 / elbow→S7`，**是错的**。
> 真机逐度扫描 + 白底暗件分割 + FK 骨架拟合（`docs/hardware-measurement.md`）确证：
> **S7 = 大臂/肩、S8 = 小臂/肘**。固件里的 left/right 只是**安装位**，不代表角色。
> 照旧映射写固件会得到一台反着动的机械臂。

| 关节 | 舵机 ID | 关节角范围 | 舵机角范围（固件硬限位） | 标定（关节 → 舵机） |
|------|---------|------------|--------------------------|---------------------|
| base | S9 | −60 … +60 | 30 … 150 | `θ + 90` |
| shoulder | **S7** | −6.09 … +49.45 | 80 … 160 | `1.44018·θ + 88.776` |
| elbow | **S8** | 108.44 … 141.86 | 20 … 100 | `−2.39401·θ + 359.61`（reverse） |
| gripper | S6 | 10 … 100 | 40 … 130 | `−θ + 140`（reverse） |

**elbow 的范围不是笔误**：小臂存的是**离开天顶的绝对倾角**（平行四连杆解耦），
真机可达区间就是 108°–142°，不存在"0° 小臂"。固件侧的限位校验必须照这张表，
不能套用"每个关节都 0–90"的通例。

> ⚠️ **表里只有 4 行，是刻意的**（2026-09-13，ADR **D70**）：`robot.yaml` 里还有第 5 个关节
> `tool`，它是 `type: passive` —— 会转、**没有独立输入**、角度由 `coupling` 派生，
> 绝对倾角被连杆锁在 90°（爪恒水平）。它**不进** `JR` / `STATE` / 标定表，
> 因此 `JR` 恒为**四元组** `base shoulder elbow gripper`。
>
> 谁若从 `robot.yaml` **生成**这张表/固件解析，必须按 `type == "revolute"` 过滤：
> 一旦把 `tool` 也算进去，`JR` 会变成五元组，而**固件、`EncodeJR()`、前端解析全部按四元组写**
> —— 表现为整条链路静默错位（不是报错）。Go 侧对应 `robot.JointOrder()`。

标定表只有这一份真值，位于 `config/robot.yaml`。固件侧应由该文件**生成**，
而不是手抄 —— 抄两份必然漂移（见 §6 待办第一条）。

## 3. 现有固件协议（`MeArm-Device`，v0）

现状（`core/cmd.c`）是**舵机级文本协议**，作为 v1 的传输底座继续保留：

```
SET 9 120            # 单舵机
SET 9 120 8 90       # 组合（≤3 个舵机）
S7=90                # 简写
JOY 900 200 512 800  # 4 路摇杆 raw
STATUS               # 回 S6=.. S7=.. S8=.. S9=..
RESET                # 全部回 90°
```

回执契约：**每条指令恰好回一行**，合法 `OK ...`、非法 `ERR ...`；异步事件以 `# ` 开头
（`# IR RAW=...`）。上位机门控以"非 `# ` 开头的行 = 应答"判定。

### 3.1 设备侧自主变化的上报（`# SERVO`）—— **已实现**

固件在**舵机被命令之外的手段**改动时主动上报一行：

```
# SERVO S6=88 S7=120 S8=92 S9=90
```

三个要点（缺一条就会用错）：

1. **`# ` 前缀 = 异步事件**，因此它天然不参与"每条指令恰好回一行"的应答契约，
   也不该拿它去放行某条在途命令的 ACK 门控。
2. **它报的是舵机角，不是关节角** —— 固件不认识关节（标定真值只有
   `robot-package/<id>/model/robot.yaml` 那一份）。换算由后端唯一那一处
   （`protocol.ServoAnglesToJoints`）完成，与 `OK JR` 的标定核对共用同一张表。
3. **触发条件只有"外部驱动"**：硬件摇杆（`joystick_scan()`）、红外遥控、
   串口 `JOY` / `AUTO`。**命令驱动的运动（`SET` / `JR` / `RESET`）一律不上报** ——
   那一路已有 ACK 与 `STATUS`，多报一份会让上位机把"命令侧"跟着实际位置拖走
   （拖动滑杆时会看到滑杆被回拉）。

**优先级低于一切命令应答**（这是硬要求，不是优化）：固件在主循环里
**`cmd_poll()` 之后**才调用 `arm_report_tick()`，且**只在 TX 环完全为空时**才发；
被挡下时保留脏标记、下一轮重新取值（latest-wins —— 不积压，也不补发旧姿态）。
理由见 `bsp/uart.c` 文件头：应答被挤掉就等于"上位机认为机械臂没收到命令"，
而那正是历史上最难查的一类故障。

### 3.2 三条设备链路对等落地（`sim` / `mujoco` / `serial`）—— **已实现**

`device.Device` 有三个实现，**同一条 `# SERVO` 契约、同一个 `origin` 语义**：

| 链路 | 实现 | 上报编码 | 让位闸门 |
|------|------|----------|----------|
| `sim` | `backend/internal/device/sim.go`（内置假固件） | `# SERVO` / `STATE` | `jrBusy` + `flushReport()` |
| `mujoco` | `backend/internal/device/mujoco.go` + `simulation/mujoco/server.py` | 同上（stdio 上跑同一套文本协议，逐字节一致） | `jr_busy` + `flush_report()` |
| `serial` | `backend/internal/device/serial.go` → ATmega328P 固件 | 同上（固件 `arm_report_tick()`） | `uart_tx_used() == 0` |

三处闸门的语义**完全一致：让位 ≠ 丢弃**。被在途命令挡下时只置脏标记，
补报时**重新取值**（latest-wins）—— 既不积压，也不补发一个已经过时的旧姿态。

同一个验收脚本跑三条链路（`--config` 决定用哪份运行配置）：

```bash
node core/tools/verify_device_follow.mjs --config sim      # 23/23（无需硬件）
node core/tools/verify_device_follow.mjs --config mujoco   # 22/22（容差见下）
node core/tools/verify_device_follow.mjs --config serial   # 23/23（需真机）
```

> ⚠️ **MuJoCo 的命令收敛容差比另外两条宽**（`MUJOCO_ACK_TOL = 1.0` vs 默认 `0.15`）。
> 原因是**物理**而非实现差异：该链路有重力 + 有限 PD 增益，臂停在**力矩平衡处**而非命令角，
> 实测 elbow 静态偏差 `0.551°`、shoulder `0.274°`（base / gripper `0.000°`），
> 且 **3s 与 6s 读数完全一致** ⇒ 是**稳态误差**，不是"还没收敛"。见 ADR **D81**。

**摇杆步长只有一份真值**：固件 `core/joystick.c::joystick_delta()`。
Go `sim.go::joystickDelta` 与 Python `server.py::joystick_delta` 与它**逐值同式**
（`raw` 钳位 `0..1023` → 死区 `200..800` 归零 → `step = 2 + beyond/30` 上限 `10`，
正负号按通道号定）。
⚠️ Python 侧 `beyond // 30` 与 C / Go 的向零取整**同值仅当 `beyond >= 0`**，
该前提由 `raw` 先被钳位保证 —— 注释里钉住了这条，别照抄成通用式。
单测表里含两条**真机实测交叉验证**过的用例（`(9,100)→+5`、`(7,0)→+8`）：
只测自己写的表等于自证，必须有外部读数。
`tests/sim/test_server.py` 另含 `joystick_delta(9, -100) == 8`、`(9, 99999) == (9, 1023)` 两条边界。

#### ⚠️ 两处**刻意保留**的链路间不对称（不是遗漏 —— 改动前先读这一节）

1. **`SET` 的 `origin` 随链路不同。**

   | 链路 | `SET` 归属 | 原因 |
   |------|-----------|------|
   | `serial` | **命令路径**（固件 `s->external = false`，不主动上报） | 真机链路上 `SET` 只可能来自 `serial.go` 对 `JR` 的翻译 |
   | `sim` / `mujoco` | **设备侧外部改动**（走 `# SERVO`，`origin=device`） | 这两条链路**直接处理 `JR`**、从不拆 `SET`（翻译只发生在 `serial.go`）⇒ 收到的 `SET` 只可能来自调试 / 验收脚本 |

   同一串字节在两条链路上归属不同的 `origin`，**这是设计**：
   判据是"**它由谁发出**"，不是"它长什么样"。
   （`serial.go::handleLine` 的 `default` 分支确实会把调试期的裸 `SET` 透传给固件，
   但回执**原样转发、不合成任何关节级回执** —— 与上表自洽。）

2. **`STATUS` 的翻译层不同。**

   | 链路 | 路径 |
   |------|------|
   | `sim` / `mujoco` | `protocol.ParseReply` 直接把 `STATUS S6=.. S7=..` 判为 `ReplyServo`（**实际角**） |
   | `serial` | `serial.go::execStatus` 先 `awaitLineWithServo` 取值，再 `emitStateFromServo` **合成一行 `STATE`** |

   ⇒ 上层（`controller.mergeState`）看到的结果相同，但**回执类型不同**。
   在此处加断言必须**按链路写**，不能假定三条链路回执类型一致。

## 4. v1 目标：关节级协议（新增，向后兼容）

ArmsPilot 的虚拟机械臂天然工作**关节空间**。v1 让固件也理解关节空间，
避免"上位机做标定、固件再做一次标定"的双份真值。

```
JR <j1> <j2> <j3> <grip>        # 关节角整帧（degree，浮点，保留 1 位小数）
JR 0 0.85 112.62 50             # HOME 位（= RESET，四舵机恰好全 90°）
JR 0 30 120 50                  # 肩前倾 30°、小臂绝对角 120°
```

回执（同时回出换算后的舵机角，便于上位机核对标定）：

```
OK JR S9=90.00 S7=90.00 S8=90.00 S6=90.00      # ← JR 0 0.85 112.62 50
OK JR S9=90.00 S7=131.98 S8=72.33 S6=90.00     # ← JR 0 30 120 50
ERR JOINT elbow 95.00 (limit 108.44..141.86)
```

注意第二条回执里 **S7 与 S8 都在动**：肩转 30° 时小臂的**绝对角**保持 120°，
但因为小臂相对大臂的夹角变了，两个舵机都要重新算。这正是"绝对角语义"在协议层的体现 ——
固件若按"相对角"实现，`S8` 会算出完全不同的值。

- 固件侧**内置同一张标定表**（由 `robot.yaml` 生成，或编译期常量 + 上电可写入）
- `JR` 与既有 `SET` / `JOY` 并存：`SET` 仍是舵机级直控（调试/回归用），`JR` 是关节级
- **`RESET` 语义保持"全部舵机 90°"**，即 HOME 位，不要改成"关节全 0"

### 状态回读

```
STATE <j1> <j2> <j3> <grip>          # 关节空间状态（由固件从舵机角反算）
# 或（有真实位置反馈时）
FB <j1> <j2> <j3> <grip>             # 实际关节角反馈，用于 Actual / Error 显示
```

无位置反馈的开环舵机下，`FB` 的值等于命令值；一旦换用带反馈的方案
（电位器 / 磁编码器 / 舵机回读），**上位机侧 UI 与误差计算无需改动**（见 D9）。

## 5. 上位机侧协议（Phase 8：Browser ↔ Go Server）—— **已实现**

实现落点：`MeArm-3D/backend/`（自包含 Go module `armpilot/backend`），
前端 `frontend/src/robot/transport/WebSocketTransport.ts` + `wsProtocol.ts`。

| 项 | 值 |
|----|-----|
| 默认监听 | `0.0.0.0:8090`，端点 `/ws/joint`，健康检查 `/healthz` |
| 为什么不是 8080 | `MeArm-RemoteControl` 已占用 8080（舵机级摇杆通道），两者**可同时运行** |
| 链路末端 | `device.mode = sim`（内置假固件，Phase 8）\| `mujoco`（物理仿真，MuJoCo 轨）\| `serial`（真串口，Phase 9）—— **三者对等**，见 §3.2 |
| 应用层心跳 | 客户端发 `ping` / 服务端回 `pong`；间隔 10s（前端）/ 15s（服务端传输层 ping） |
| 传输层心跳 | 服务端发 RFC6455 `ping`，浏览器自动回 `pong`，40s 静默判死 |

### 5.1 消息全集（`version: 1`）

```jsonc
// ── 下行（浏览器 → 服务端）─────────────────────────────────────────────
{ "version": 1, "type": "joint_command", "timestamp": 1757654321000, "seq": 42,
  "joints": { "base": 0, "shoulder": 20, "elbow": 120, "gripper": 50 } }
{ "version": 1, "type": "ping",            "timestamp": 1757654321000 }
{ "version": 1, "type": "status_request",  "timestamp": 1757654321000 }

// ── 上行（服务端 → 浏览器）─────────────────────────────────────────────
// 接入即推 + status_request 响应：模型真值（"标定表只有一份"的在线校验源）
{ "version": 1, "type": "hello", "timestamp": 1757654321000,
  "model": { "id": "marm", "name": "mARM", "source": "…/config/robot.yaml",
             "jointOrder": ["base","shoulder","elbow","gripper"],
             "limits": [ { "id": "elbow", "role": "elbow", "min": 108.4414852068, "max": 141.8582211436 }, … ],
             "calibration": [ { "jointId": "shoulder", "servoId": "S7", "channel": 7,
                                "offset": 88.776, "scale": 1.44018, "reverse": false,
                                "servoLo": 80, "servoHi": 160 }, … ],
             "homePose": { "base": 0, "shoulder": 0.8498937633, "elbow": 112.6185771989, "gripper": 50 } },
  "device": "sim", "connected": true }

// 状态：关节角由链路末端从**舵机实际角反算**（不是命令回显）
//
// `origin` 说明这帧**由谁引起**（可选字段，缺省 = "command"：
// 老前端不认识它也照常工作 —— 只更新 Actual、不跟随）：
//   "command" — 本机命令的应答 / 状态快照   ⇒ 界面**不**跟随
//   "device"  — 设备侧自主变化：摇杆 / 红外 / 面板手拧 / 调试直控
//                                            ⇒ 界面**跟随**，把命令侧对齐过去
{ "version": 1, "type": "joint_state", "timestamp": 1757654321100, "origin": "device",
  "joints": { "base": 0, "shoulder": 19.8, "elbow": 120.1, "gripper": 50 } }

{ "version": 1, "type": "pong",          "timestamp": 1757654321010, "seq": 42 }
{ "version": 1, "type": "device_status", "timestamp": 1757654321000,
  "device": "sim", "connected": true }

// 事件 / 错误（**不是**连接终态：回一行 ERR，链路仍然活着）
{ "version": 1, "type": "error", "timestamp": 1757654321200,
  "code": "JOINT_LIMIT", "message": "elbow 95.00 超出 108.44..141.86" }
```

错误码（`protocol.Code*` / `wsProtocol.CODE_*`，两侧必须一致）：

| code | 含义 |
|------|------|
| `BAD_MESSAGE` | JSON 非法 / 缺 `type` / 关节值非数字 |
| `VERSION_MISMATCH` | `version` 非 0 且 ≠ 1（字段语义会静默错解，必须拒） |
| `JOINT_LIMIT` | 关节或舵机越界（前端计入 `rejected`，**不**断开连接） |
| `DEVICE_UNAVAILABLE` | 链路末端不可用（串口打开失败等） |
| `ACK_TIMEOUT` | 单条指令在 `ack_timeout_ms` 内没等到回执 |
| `INTERNAL` | 其它内部错误 |

### 5.2 Go 侧职责分层（不许 WebSocket 层直接操作串口）

```
WebSocket Client → Protocol(编解码) → Robot Controller → Device(sim | serial) → AVR
```

| 层 | 包 | 只负责 | **绝不**负责 |
|----|----|--------|--------------|
| 编解码 | `internal/protocol` | JSON / `JR` / `OK JR` / `STATE` / `ERR` 的字符串↔结构体 | 不认识关节、不认识标定、不认识机械结构 |
| 标定与限位 | `internal/robot` | 读 `config/robot.yaml`；关节↔舵机换算；限位校验 | 不认识 socket、不认识串口 |
| 控制器 | `internal/controller` | **唯一"懂机械臂"处**：ACK 门控、latest-wins、标定回执核对、状态发布 | 不认识 HTTP / WebSocket 帧 |
| 链路末端 | `internal/device` | `sim`（假固件）/ `mujoco`（物理仿真）/ `serial`（Phase 9）—— 协议与固件逐字节一致，见 §3.2 | 不认识关节语义（只收发字节行） |
| 对外 | `internal/wsserver` | RFC6455 帧、路由、广播、心跳 | 不认识关节语义（只转发） |

### 5.3 五条**必须做对**的语义（都有测试锁死）

1. **`OK JR` 携带的是目标舵机角，不是实际位置。**
   它只用于**核对标定**（与本地标定算出的值比对，偏差 > 0.1° 报警告）。
   ❌ 拿它当 Actual 回推 ⇒ 误差恒为 0，整条误差链路形同虚设。
   ✅ 状态只能来自 `STATE` 行（由舵机**实际角**反算）。

2. **ACK 门控 + latest-wins。**
   同一时刻**只有 1 条 `JR` 在途**（Phase 9 串口 115200 + ACK 门控下必须如此）。
   在途期间新的 `joint_command` **只覆盖"待发槽"，不排队** —— 排队会让拖动时命令堆积，
   回执永远在追历史的某个中间位置。

3. **`min_send_interval_ms > 0` 时不许用 `time.Sleep`。**
   睡眠会卡住读循环，而回执正需要读循环来消费 ⇒ 自锁。正确做法是撤下在途、放回待发槽、
   用定时器补发。

4. **心跳是两层的。**
   浏览器 `WebSocket` API 不暴露传输层 `ping`，所以应用层必须自己发 `{"type":"ping"}` 等 `pong`；
   同时服务端发 RFC6455 `ping`（浏览器自动回 `pong`）由服务端看门狗判死。
   ⚠️ 上一轮 ping 仍在途时**不得**重复发，否则超时判定会被自己不断推后。

5. **要不要"跟随"必须由 `origin` 判定，不能由前端猜。**
   设备侧的自主变化（硬件摇杆 / 红外遥控）必须让界面跟着走 —— 否则滑杆、主臂、
   目标点全部停在旧值上，而画面**看不出任何异常**。
   但两种"猜"的写法都不成立：
   - 猜"Actual ≠ Command 就跟随" ⇒ 拖动时设备还在斜坡上，Actual 必然落后于 Command，
     命令侧会被一路拉回半路位置（正是 `命令 → 状态 → 命令` 回环）；
   - 猜"一律跟随" ⇒ 命令侧被实际值拖走，命令语义消失。
   ⇒ 所以在协议里**标注来源**：只有 `origin === "device"` 才跟随。

   ⚠️ 跟随的那条路径必须**抑制下发**（前端 `transportBridge.suppressCommandSend`），
      否则每次摇杆上报都会触发一条命令下发 —— 变成"设备自己动 → 上位机把它推回去"
      的对抗（验收里有专门一条断言钉住这一点）。

### 5.4 ⚠️ 链路精度 = 跟踪误差的可分辨下限

`JR` 只保留 **1 位小数**（0.1°），因此：

- 前端内部的**全精度**命令（如 HOME 位 `elbow = 112.6185771989`）与线上值（`112.6`）
  天然相差 `0.0186°`。**跟踪误差必须在线上精度上定义**，否则会得到一个
  **永远收敛不到 0 的量化残差**（实测表现为面板恒显 `0.02°`、`moving` 永不归零）。
  前端的处理：`wsProtocol.quantizeForWire()` —— 比对前把命令归整到 0.1°（`WIRE_JOINT_STEP_DEG`）。
- 0.1° 关节 ≙ 舵机 `0.1 × scale` 度：肩 `0.14°`、肘 `0.24°`。
  舵机自身（8 位定时器 / 1µs tick）分辨率约 `0.18°`，**同量级**，
  故 1 位小数没有浪费带宽也没有损失真实精度。


## 6. 待办（Phase 9 起）

- [ ] 由 `robot-package/<id>/model/robot.yaml` **生成**固件标定表（避免手抄导致双份真值）
- [ ] 固件 `core/cmd.c` 增加 `JR` / `STATE <关节>` 解析与回执
- [x] 固件 `arm_report_tick()`：舵机被**外部手段**改动时主动上报 `# SERVO`（§3.1），
      优先级低于命令应答（TX 环为空才发）
- [x] 后端 `protocol.ParseReply` 识别 `# SERVO` / `STATUS` 为 `ReplyServo`（**实际角**），
      与 `ReplyOKJR`（**目标角**）严格分流；`joint_state` 增加 `origin` 字段
- [x] Go `internal/protocol` 增加关节级编解码 + 单测（Phase 8：`EncodeJR` / `EncodeOKJR` /
      `EncodeState` / `EncodeServoReport` / `ParseReply` / `ServoAnglesToJoints`，
      含"`ERR` 判定必须先于 `OK JR`"与"`OK SET` 不得当实际角"）
- [x] `internal/device/serial.go` 落地真实串口（Phase 9：Windows 非重叠 I/O，**不用 `bufio`**；
      Uno DTR 复位静默窗口 `connect_settle_ms=2600` + 暖机包）
- [x] `internal/device/sim.go` 补固件级外部入口（`SET` / `S<n>=` / `JOY`），
      按"设备侧自主变化"路径上报 —— **不接真机也能端到端验证"摇杆 → 界面跟随"**
- [ ] `MeArm-RemoteControl` 现有摇杆通道与关节通道并存，注意 ACK 门控饥饿问题
      （见 skill `arm-robot-serial` 关键坑 6：周期查询会饿死遥控流）
- [ ] 真机验证（硬件到位后）：`node core/tools/verify_device_follow.mjs --config serial`
      —— 拨动摇杆/红外 → 串口应出现 `# SERVO …`；网页滑杆应跟随；且串口敲 `STATS`
      其 `tx_drop` 保持 0（这是"遥测没有挤占应答"的**唯一独立证据**，收发对账不算）
- [x] 仿真侧对等实现 + 离线验收：`core/tools/verify_device_follow.mjs --config sim`
      **PASS 23 / FAIL 0**（连跑 3 次稳定；仿真与真机跑同一份脚本，只差 `--config`）。
      覆盖：单次拨动的方向/幅度、事件流（多帧）、命令在途插入拨动（命令优先 + 让位不丢）、
      设备在坡上时命令夺回报通道（无"回拉"）、收尾复位

### 6.2 Phase 9 真机实测：**回执证明不了物理到位**

> 这是本节最重要的一条，也是 Phase 9 验收方法论的基石。

固件**没有位置反馈**（无编码器、无电位器回读）。`arm_get_angle()` 返回的是固件变量里
记着的**目标值**，因此：

| 回执 / 字段 | 它在说什么 | 能不能证明"到位" |
|-------------|-----------|-----------------|
| `OK SET S7=118` | 我收到了，把目标设成 118 | ❌ 不能 |
| `OK JR S9=.. S8=..` | 我按标定算出的**目标**舵机角 | ❌ 不能 |
| `STATE <sh> <el> <ba> <grip>` | 固件当前记着的关节角 | ❌ 不能 |
| 后端回推 `joint_state` | 上面这条的转发 | ❌ 不能 |
| **相机反解照片** | 它**实际上**在哪 | ✅ 唯一能 |

**哪怕机械臂卡死在桌面上，前四条依然一字不差。** 所以：

1. 回推的 `joint_state` **只作链路自洽性参考**（证明命令确实穿过了整条链路），
   验收里**绝不作为「到位」证据**。
2. 真机验收必须走 `tools/verify_serial_e2e.mjs`（串起 WebSocket → 串口 → 相机抓帧 →
   `tools/verify_pose.py` 反解比对），见 `docs/decisions.md` D34。
3. 期望值取**量化后**的关节角：固件只吃整数舵机度，物理落点是
   `servoToJoint(round(jointToServo(θ)))`，与意图角天然差 `0.347°`(肩)/`0.209°`(肘)。
   拿意图角当期望 = 白送一份假误差。
4. **base 必须留 0°**：相机只能测矢状面，base 离面即判 SKIP。

**Phase 9 首次真机闭环实测结果**（PASS 18 / FAIL 1）：

| 项 | 值 |
|----|-----|
| 链路末端 | `device=serial`（真机） |
| 标定单一真值 | `homePose` 与 `config/robot.yaml` **逐位一致**（容差 `1e-6`） |
| 开机就绪门 | Uno DTR 复位静默窗口 2.7s；不等待会报 `DEVICE_UNAVAILABLE` |
| 链路回推 | 7 步 JR 命令 `max\|Δ\| ≤ 0.004°`（**纯链路自洽，不含物理**） |
| 相机重复性 | 同位姿两帧反解差 肩 `0.26°` / 肘 `0.01°` |
| 增益复核 · 肘 | 反解 `−0.4235` vs yaml `−0.4177` ⇒ **+1.4%，肘标定被独立证实 ✅** |
| 增益复核 · 肩 | 反解 `+0.6033` vs yaml `+0.6944` ⇒ **−13.1%，肩标定需重测 ❌** |
| ⚠️ 误差源 | 同台面同取景相隔 2 分钟两批，锚点绝对角偏置 `+2.69° → +7.75°`（**漂 5°**），而两批 Otsu 阈值同为 164 ⇒ **自动曝光是绝对角主导误差源** |

⇒ **高精度复测的前置条件**：按 `docs/hardware-measurement.md` 的 Phase 4.5 标准重布台面
（白分割板铺满视场 + 画面内放尺 + 正交侧视 + **锁死相机曝光**）。**锁死曝光是硬要求。**

### 6.1 Phase 9 接串口时的实测坑（已写入 `device/serial.go` 注释）

1. **ATmega328P 开机静默窗口**：打开串口拉低 DTR 会复位 MCU，bootloader 交权期内
   （约 1–2s）指令被吞且无回执。`SimTuning.BootMs` 就是为复现这条而存在的参数。
2. **Windows 非重叠 I/O 的读会立即返回 0 字节**（不是阻塞），照 `bufio.Scanner` 写会
   得到"读循环空转 100% CPU"或"半个包就当一行"。
3. **不要用 `bufio`**：它会把"还没收到换行的一行"留在内部缓冲，超时判定的时间基准就错了。
4. **状态应由固件主动上报**，上位机轮询 `STATUS` 会与 ACK 门控互相饿死
   （每次查询都要等回执，而回执要读循环消费，两边互锁）。
   ✅ 已按**事件驱动**落地（比周期上报更省带宽）：固件只在舵机被
   **命令之外的手段**改动时才发 `# SERVO`，且只在 TX 环完全空闲时发（§3.1）。
   静止时零遥测流量。
