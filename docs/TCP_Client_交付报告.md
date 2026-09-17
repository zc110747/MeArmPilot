# MeArm-RemoteControl · TCP Client 接入 MeArm-3D —— 最终交付报告

> 对应 `docs/MeArm_Prompter_10.md` §20（20.1–20.8）。
> **本报告的每个数字都来自本机实跑**；所有真机相关结论一律标 `NOT TESTED`，不做推断。
> 交付主体：`MeArm-RemoteControl` 默认以 **TCP Client** 驱动 `MeArm-3D`，同时**保留** `--real` 串口真机模式。

---

## 20.1 实现摘要

### 已完成

| 能力 | 说明 |
|------|------|
| **TCP Client 传输** | 新增 `internal/netlink/`：长连接 + 一行一条 JSON（JSON Lines）、ACK 门控 + latest-wins、断线指数退避重连**并重取基准**、单一写泵 |
| **默认走 network 模式** | `start.bat` 默认 network（TCP Client → MeArm-3D `9100`）；`start.bat --real` 走原串口模式，**既有串口链路一行未删** |
| **舵机级摇杆** | 双 3D 摇杆映射为 `servo` 命令（本机 4 轴 ⇒ `servo 1..4`）；跨链路方向换算经 `internal/protocol.NetInvertFor` |
| **XYZ 面板** | 六向步进 → `move`（`axis` + `direction` + `step`） |
| **爪控制** | `gripper`（`open` / `close`），端点真值取自 A 端配置 |
| **四舵机面板** | 逐轴直接给角 → `servo`；越界由服务端拒绝并在网页显示原因 |
| **只读基准** | 连接建立即发 `{"cmd":"state"}` 取当前角基准（重连后**重取**，不沿用旧基准） |
| **状态反馈** | 服务端**结构化** `state.servo` / `state.tcp` / `state.device` → 广播给网页 `{t:"state"}` |
| **来源标注（D84）** | MeArm-3D 侧新增 `origin=external`，使对端页面在外部驱动时**主臂与幽灵统一**（详见 §20.6） |

### 未完成 / 未做

| 项 | 状态 | 说明 |
|----|------|------|
| 真机端到端（network 模式 + 真机） | `NOT TESTED` | 全程仅在 Sim 上验证 |
| 串口模式真机复测 | `NOT TESTED` | 串口代码路径未改动，但本轮未接真机 |
| 鉴权 / TLS | `NOT IMPLEMENTED` | 接口假定可信局域网 |
| 绝对位姿直输 | `NOT IMPLEMENTED` | 属 v2，不在本次范围 |

---

## 20.2 最终架构

```text
RemoteControl Web (9001, 内嵌静态页 + WS)
    ↓  {t:"joy"} / {t:"xyz"} / {t:"grip"} / {t:"servo"}
RemoteControl Go Backend
    ↓
internal/netlink  (TCP Client：长连接 + JSON Lines + ACK 门控 + latest-wins)
    ↓  127.0.0.1:9100
MeArm-3D TCP Server (internal/tcpserver)
    ↓  controller.ApplyFrom(protocol.OriginExternal, …)
MeArm-3D Controller / Device
    ↓
Sim (内置假固件) / Real (串口)
```

**双传输共存**（端口刻意错开，互不干扰）：

| 模式 | 入口 | 出口 | 说明 |
|------|------|------|------|
| network（默认） | 9001 Web/WS | **TCP Client → 9100** | 不经 Uno，直接驱动 MeArm-3D 的 sim/real |
| serial（`--real`） | 9001 Web/WS | **9002 TCP 透传** + 串口 | 原有链路，保留 |
| 状态旁路 | — | — | MeArm-3D 广播 `ws://…:8090/ws/joint`（供 MeArm-3D 自己的页面） |

**判"当前是哪个模式"看 9002**：串口模式会 LISTENING，network 模式不监听（比翻日志可靠）。

---

## 20.3 文件修改清单

### MeArm-RemoteControl（交付主体）

**新增**

| 文件 | 原因 |
|------|------|
| `internal/netlink/client.go` | TCP Client 主体：连接生命周期、读写泵、ACK 门控、latest-wins、重连补基准 |
| `internal/netlink/axis.go` | 摇杆轴 → 舵机角**积分**（死区 + 速度区间 12..90 °/s 线性 + 按 dt 积分 + 方向位）；含与 MeArm-3D 舵机编号的**接线表** |
| `internal/netlink/client_test.go` · `axis_test.go` | 上述两者的单测（含"降级到 FallbackAngle 且显式上报"） |
| `main_test.go` | 进程级单测（模式组装 / 启动路径） |
| `internal/link/link.go` · `network.go` · `serial.go` | 新增"**最小适配层**"，唯一目的是让同一套网页摇杆能换传输。⚠️ 它**不是重构**：`serialLink` 的代码原本就写在 `internal/web/server.go` 的 `handleWS` 里，本次**原样搬家**（语义 / 顺序 / 日志一字未改）；`netLink` 只做转发。两个实现的能力差异经 `Caps` **显式**暴露给网页，不让网页猜 |
| `internal/config/config.go` | `mode`（network/serial）、主机端口、轴映射、方向、超时等配置 |
| `start.bat` | 默认 network；`--real` 切串口（纯 ASCII、双退出路径 PAUSE） |
| `tools/sim2sim-check.mjs` | 端到端验收：`arm-web(network) ⇄ MeArm-3D(sim)`，期望值从 `config.yaml` + `robot.yaml` **派生** |
| `web/static/js/netpanel.js` | 网络模式面板（XYZ / 爪 / 四舵机 / 状态） |

**修改**

| 文件 | 原因 |
|------|------|
| `main.go` | 按 `mode` 组装 link + netlink；把网络模式的状态接进 web 广播 |
| `internal/web/server.go` | 新增 `state` 消息（角度 / TCP / 链路末端）；`RefreshState` 主动补缺口；错误上报 |
| `internal/protocol/protocol.go` | 跨链路**方向换算** `NetInvertFor`（串口链路的 `invert_*` 含一层固件补偿，直接复用会让两种模式推杆方向相反） |
| `internal/protocol/protocol_test.go` | 上述换算的单测（8 轴保持、其余取反） |
| `web/static/index.html` · `js/main.js` · `js/wsclient.js` · `css/style.css` | 面板与消息接线 |
| `config.yaml` · `config.yaml.example` | 模式与网络参数 |
| `README.md` | 启动方式、模式判据、验收数据、`start.bat` 告警 |

### MeArm-3D（最小侵入，**不改动任何既有行为**）

| 文件 | 原因 |
|------|------|
| `internal/tcpserver/protocol.go` | 新增**只读** `state` 命令（D83）；三条写命令改走 `ApplyFrom(OriginExternal, …)`（D84） |
| `internal/protocol/protocol.go` | 新增 `OriginExternal` 常量（D84） |
| `internal/controller/controller.go` | `Apply` → 薄包装 `ApplyFrom(origin, joints)`；来源随命令同行（D84） |
| `frontend/src/robot/model/RobotState.ts` · `transport/wsProtocol.ts` · `transport/WebSocketTransport.ts` · `store/transportBridge.ts` | 放行并跟随 `origin=external`（D84） |
| 两侧测试 3 文件 | 新增 6 条断言（3 后端 + 3 前端） |
| `core/tools/verify_external_origin.mjs`（新增） | 后端侧来源分段验收 3 段 |
| `frontend/tests/e2e/external-origin-follow.mjs`（新增） | 浏览器侧"主臂与幽灵是否统一"判据（**9 PASS / 0 FAIL**） |
| `README.md` · `docs/tcp-control-v1.md` · `docs/decisions.md` | 文档与 ADR **D83 / D84** |

### 未修改的关键文件（刻意不动）

| 文件 / 模块 | 为什么不动 |
|-------------|-----------|
| RemoteControl `internal/serial/*`（字节级串口实现） | 原有真机链路，**一行未改**；回归由 39 项单测守着 |
| RemoteControl `internal/tcp/*`（9002 TCP 透传） | 原有透传链路，**一行未改** |
| RemoteControl `internal/hub/*`（网页广播） | 未改；网络模式**复用**它的广播能力 |
| MeArm-3D `internal/robot/kinematics.go`（FK/IK） | 几何真值只有一份（`robot.yaml`）；TCP 层不得自造第二份 |
| MeArm-3D `internal/device/*`（sim / serial / mujoco） | 命令与 WS **同一条路**，不需要新路径 |
| MeArm-3D `internal/wsserver`（8090） | 协议字段只**新增**可选值，既有解析不变 |
| MeArm-3D `simulation/mujoco/*.xml`（MJCF） | 未参与本次改动 |
| MeArm-3D 前端 IK / 拖动 / 示教 / 纹理 | 未参与本次改动 |
| RemoteControl 串口协议编码 | 保留原样，回归靠单测 |

---

## 20.4 启动方式

```text
start.bat
    → Network Mode      （默认：TCP Client → MeArm-3D 9100，链路末端 sim/real 由 MeArm-3D 决定）

start.bat --real
    → Serial Mode       （原串口链路：本机串口 → arm-device 固件；9002 透传同时开启）
```

**前置**：network 模式需要 MeArm-3D 后端在跑（`MeArm-3D/backend/bin/armpilot-backend.exe -c config.yaml`，监听 `8090` + `9100`）。

**`start.bat` 注意事项**（已写进 `MeArm-RemoteControl/README.md`）：
- **不杀已有实例**：若 `arm-web.exe` 已在跑，会 listen 失败但**不退出**；重建时旧二进制被改名 `arm-web.exe~`。
  启动前先确认只有一个 `arm-web.exe`。
- 每条退出路径（成功与失败）都 `PAUSE`；纯 ASCII，避免 GBK 控制台解析问题。

---

## 20.5 协议映射

### 消息边界

**JSON Lines**：一条命令一行，`\n` 结尾。响应同理，一行一条。

### 下行命令（RemoteControl → MeArm-3D）

| 能力 | 报文 | 备注 |
|------|------|------|
| **XYZ** | `{"cmd":"move","axis":"x\|y\|z","direction":"+\|−","step":<mm>}` | `step > 0`，六向 |
| **爪** | `{"cmd":"gripper","action":"open"\|"close"}` | 只认这两个值 |
| **四舵机** | `{"cmd":"servo","servo":<1..N>,"angle":<deg>}` | `servo` 是**舵机编号**，不是关节名 |
| **只读基准 / 刷新** | `{"cmd":"state"}` | 无参数、无副作用 |

> 摇杆走的就是 `servo`（舵机级），不是关节级：这是本工程"舵机命名 ≠ 运动学角色"的直接后果 ——
> 见 memory 铁律 3。摇杆增量在客户端按真实 dt 积分成**绝对角**再下发。

**摇杆接线表**（`internal/netlink/axis.go::DefaultAxisMap`，来源 = `robots.yaml` → `robot.yaml`
的 `actuators[].channel` 与 `Model.JointOrder()` 的顺序）：

| 网页摇杆轴 | arm-device 舵机 | MeArm-3D 舵机编号 | 关节 |
|-----------|----------------|------------------|------|
| 左摇杆 X | S9 | `servo 1` | base（底座） |
| 左摇杆 Y | S8 | `servo 3` | elbow（左舵） |
| 右摇杆 X | S6 | `servo 4` | gripper（夹取） |
| 右摇杆 Y | S7 | `servo 2` | shoulder（右舵） |

> ⚠️ 这张表存在的唯一理由：网页摇杆沿用的是 **arm-device 固件的舵机 id**（`9/8/6/7`），
> 而 MeArm-3D 的 `servo` 用的是**按 `JointOrder()` 的顺序编号**（`9→1, 7→2, 8→3, 6→4`）。
> 两者编号不一致，必须显式对齐 —— 直接按数字照搬会让"左摇杆 Y"去驱动肩而不是肘。
> 这里的 clamp（`ServoMin/Max`）**不是软限位**，只是"别把 NaN / 离谱数字发上线"的数值保护；
> 真正的硬件行程在 `robot.yaml` 的 `actuators[].limits`，由 MeArm-3D 的 `servo` 命令把关。

### 上行响应

**成功**（每条成功应答**都附带** `state` 快照）：

```json
{"ok":true,"cmd":"move","state":{
  "joints":{"base":0,"shoulder":0.85,"elbow":112.62,"gripper":50},
  "servo":[90,90.0,90.0,40],
  "tcp":[115.033,0,109.224],
  "device":"sim"}}
```

**失败**（错误文本，v1 不引入数字码）：

```json
{"ok":false,"cmd":"move","error":"JOINT_LIMIT: ..."}
```

常见 `error`：`invalid json` · `unknown cmd "<x>"` · `OUT_OF_WORKSPACE: …` · `JOINT_LIMIT: …` ·
`angle out of range (<id> min..max)` · `no state available yet`。
**任何错误只影响这一条命令，连接保持可用。**

### 状态响应

见 §20.6。**关键：角度取服务端结构化 `state.servo`，不解析文本回显。**

---

## 20.6 状态反馈

### 状态从哪里获取

**唯一来源 = MeArm-3D 的服务端真值**，有两条获取途径，都不新增协议：

1. **主动查询**：连接建立时 `AutoSync` 发一条 `{"cmd":"state"}` 取基准；
   **重连后重取**（不沿用断线前基准）；网页"刷新"也走这条（`RefreshState`）。
2. **随应答带回**：摇杆持续下发 `servo`，每条成功应答都带 `state` —— 高频操作下天然有连续状态流，**无需轮询**。

### 是否复用 TCP 状态 / WebSocket / 增加适配

| 项 | 结论 |
|----|------|
| 复用 TCP 状态 | ✅ 完全复用。只**新增一个只读命令** `state`（D83），并把每条成功应答本就带的 `state` 暴露成可单独索取 |
| 复用 WebSocket | ⚠️ 要分清**两个** WS：**RemoteControl 自己的 9001 WS 完全复用**（走既有 `hub` 广播，未改一行）；**MeArm-3D 的 8090 WS 不复用** —— 网络模式不经过它，RemoteControl 自己就是上游，直接读 TCP 应答里的 `state` |
| 是否增加适配 | ✅ 增加**一层很薄的**适配：把 `state.servo`（舵机角）逐值按本机轴映射转成网页面板角度。**不做**第二套几何运算 |

### 网页如何更新

```text
MeArm-3D TCP 应答 state.servo / tcp / device
    ↓  netlink.Client 解析
web.Server.BroadcastNetState(servo, tcp, device)
    ↓  hub 广播
{ "t":"state", "angles":{…}, "tcp":[x,y,z], "device":"sim" }   （9001 上的 WS）
    ↓  wsclient.js
netpanel.js 刷新四舵机角 / XYZ / 链路末端
```

⚠️ 角度**一律来自结构化 `state.servo`** —— 不去解析设备回显文本（那是串口模式的做法）。

### 当前限制

| 限制 | 说明 |
|------|------|
| 状态是**命令值**，不是过程值 | `state.joints` 是"最近一次被受理的目标"。要看设备实际到位情况，需读 MeArm-3D 的 WS `joint_state{origin:"device"}` |
| 无主动推送（TCP 侧） | TCP v1 是"一问一答"。持续状态流是靠**高频摇杆命令的应答**带来的，静止时不会自己刷新（可按刷新键补） |
| 老服务端降级 | 服务端不认识 `state` ⇒ 用配置里的 `fallback_angle` 起步，并在网页**显式上报**，不静默猜 |
| 真机未验证 | 见 §20.7 |

---

## 20.7 测试结果

| # | 项目 | 结论 | 证据 |
|---|------|------|------|
| 1 | **TCP Client**（连接 / 门控 / latest-wins / 重连重取基准 / 降级） | `PASS` | `go vet ./...` 干净；`go test -race -count=1 ./...` ⇒ **39 顶级用例 / 0 FAIL**（`internal/netlink` 为其主要覆盖，含"服务端不认识 `state` 时降级并显式上报"） |
| 2 | **Network Mode**（端到端 `arm-web ⇄ MeArm-3D(sim)`） | `PASS` | `tools/sim2sim-check.mjs` ⇒ **30 PASS / 0 FAIL** |
| 3 | **XYZ**（六向步进 + 几何校验） | `PASS` | 同上（含 `goHome()` 前置断言，消除位姿残留导致的偶发 FAIL） |
| 4 | **Gripper**（open/close 端点） | `PASS` | 同上：期望值**从 `robot.yaml` 派生**（open 40° / close 130°） |
| 5 | **Servo**（四轴直控 + 越界拒绝 + 被拒后回滚） | `PASS` | 同上：`servo 1..4=91` 逐一回读一致；越界被服务端拒绝且未执行；被拒后仍能继续驱动（未卡死） |
| 6 | **MeArm-3D Sim** | `PASS` | 链路末端 `device=sim`；并发冲刷 120 帧后 JSON 未交叉、后端未崩（`jsonErrors=0`） |
| 7 | **状态反馈** | `PASS` | 逐段验收读到 `state.servo` / `tcp` / `device`；网页 `{t:"state"}` 更新四舵机面板与 XYZ |
| 8 | **Serial Mode 代码回归** | `PASS` | 串口链路的实现（`internal/serial/*` 字节传输 + `internal/link/serial.go` 协议搬家）**本轮一行未改**；`internal/serial` 与 `arm-web` 包的单测在上述 39 项内全绿 |
| 9 | **MeArm-3D 原有流程** | `PASS` | 后端 `go vet` 干净 + `go test` **114 顶级用例 / 0 FAIL**（6 包）；前端 `tsc -b --force` 0 error + `vitest` **442 PASS（33 文件）**；WS 端到端、Mock、MuJoCo 轨、示教/拖动/纹理等模块本轮**均未触碰**（`run_sim2sim.py --all` 与 `pytest` 本轮未重跑） |
| 10 | **来源标注 `origin=external`**（D84） | `PASS` | `core/tools/verify_external_origin.mjs` ⇒ **3/3**；`frontend/tests/e2e/external-origin-follow.mjs` ⇒ **9/9** |
| — | **Serial Mode 真机复测** | `NOT TESTED` | 未接真机 |
| — | **network 模式 + 真机 MeArm-3D** | `NOT TESTED` | 未接真机 |
| — | 鉴权 / TLS | `NOT IMPLEMENTED` | 属 v2 |

### 关键读数（实跑）

```
Sim2Sim        30 PASS / 0 FAIL   （共收到 60 条 WS 消息，JSON 解析失败 0 条）
来源分段 A/B/C  3 PASS / 0 FAIL   （A=command 2 帧 · B=external 6 帧 · C=command 8 帧）
浏览器 e2e      9 PASS / 0 FAIL   （外部驱动后 commandJoints.base 30→−30；
                                   主臂末端位移 114.97 mm；|tcp − actualTcp| = 0.000 mm；
                                   渲染侧两棵树 TCP 世界坐标重合 0.000 mm；
                                   日志出现"外部入口在驱动…"；新增下行命令 0 条）
后端           go vet 干净；go test 114 顶级用例 / 0 FAIL（6 包）
前端           tsc -b --force 0 error；vitest 442 PASS / Test Files 33 passed
```

---

## 20.8 兼容性报告

| 问题 | 结论 |
|------|------|
| **是否修改 MeArm-3D** | ✅ 修改，但**最小侵入**：新增 1 个只读命令 + 1 个来源常量；`Apply()` 保留为 `ApplyFrom(OriginCommand, …)` 的**薄包装**，既有调用点行为逐字节不变 |
| **MeArm-3D 原有流程是否受影响** | ❌ 不受影响。后端 `go test` 114 项 / 前端 442 项全绿；原有 WS 单客户端流程、Mock 流程、MuJoCo 轨、串口轨均未触碰 |
| **RemoteControl 串口代码是否保留** | ✅ **完整保留**。`--real` 走原串口链路；新增的 network 是**并列**分支，不是替换 |
| **原有启动方式是否兼容** | ✅ `start.bat --real` 行为不变；默认分支由串口改为 network（**这是本次的需求**） |
| **是否修改 WebSocket** | ⚠️ 只**新增**一个可选枚举值（`origin` 可以是 `external`）。字段缺省语义与引入前一致（按 `command` 处理），**老客户端零影响** |
| **是否修改 Serial Protocol** | ❌ 未修改。`docs/serial-v1.md` 定义的设备文本协议（`JOY` / `SET` / `JR` / `OK` / `STATE` / `# SERVO`）**一字未动** |
| **是否存在状态同步限制** | ⚠️ 有，且已显式声明：TCP 侧"一问一答"，静止时无主动推送；`state` 是命令值而非过程值；老服务端需降级到 `fallback_angle`（网页会显式提示，不静默猜） |
| **是否存在 TCP 并发风险** | ⚠️ 已处理：单一写泵 + 每路 latest-wins + 有界队列；服务端侧 `Handler` 互斥量把"读当前 → 算目标 → 下发"做成原子操作，避免相对位移互相踩踏；单连接 panic 被 `recover()` 隔离。实测并发冲刷 120 帧后 JSON 未交叉 |
| **是否存在未测试项目** | ⚠️ 有，且**全部标注**：真机（两种模式）、鉴权/TLS、绝对位姿直输 |

### 一句话总结

> 本次把 `MeArm-RemoteControl` 的**默认控制路径**从"串口驱动 Uno"换成"TCP Client 驱动 MeArm-3D"，
> **原有串口能力与 MeArm-3D 既有流程均未破坏**；
> 过程中发现的"外部入口驱动时对端页面只动幽灵臂"是**协议层面的来源标注缺失**（ADR **D84**），
> 已修复并留下两侧可复跑的判据脚本。**所有真机结论均为 `NOT TESTED`。**
