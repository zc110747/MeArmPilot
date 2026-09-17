# TCP JSON 控制接口 v1

> 本文描述后端第四个**入口**。它不是第四个控制核心。

## 0. 定位：端口扩展 + 协议转换

```
HTTP  /healthz ─┐
WebSocket      ─┼─▶ controller.Apply（关节角整帧）─▶ device ─▶ sim / serial / mujoco
Serial（上位机）─┤        ↑ 限位 · ACK 门控 · latest-wins 全在这里
TCP JSON  ←─────┘
```

TCP 层**只做两件事**：把一行 JSON 翻译成一次 `controller.Apply`，
再把执行后的状态读回来。它不持有 IK 算法、不持有软限位、不判断
`device.mode` —— 换链路（sim / serial / mujoco）本接口一个字都不用改。

因此：**真机上能做的，TCP 上都能做；真机上被挡下的，TCP 上也一样被挡下。**

---

## 1. 开启

`backend/config*.yaml`：

```yaml
tcp:
  enabled: true              # 默认 false —— 不显式开启就不监听
  host: "0.0.0.0"
  port: 9100                 # 刻意避开 8090(WS) / 5273(vite) / 8080 / 9001
  max_line_bytes: 65536      # 单行上限，超限断开该连接
  read_timeout_ms: 0         # 0 = 不设读超时（长连接）
```

启动后日志会打印实际监听地址。三份配置（`config.yaml` / `config.mujoco.yaml` /
`config.serial.yaml`）都已开启，端口一致。

> ⚠️ 与 WebSocket 共用 8090 的逻辑不同：TCP 是**独立端口**，可以和后端的
> HTTP/WS **同时**提供；但同一个 9100 端口仍只能有一个后端实例占用。

---

## 2. 传输格式：JSON Lines

- 一条命令 = **一行 JSON + `\n`**；一行一条应答（严格一问一答，可流水线发送）。
- UTF-8；不带 BOM；行尾 `\n`（也接受 `\r\n`）。
- 单行超过 `max_line_bytes` ⇒ 服务端**断开该连接**（不影响其它连接）。
- 多余字段忽略，未知 `cmd` 返回 `ok:false`，连接保持可用。

---

## 3. 命令

### 3.1 `move` —— XYZ 相对位移

```json
{"cmd":"move","axis":"x","direction":"+","step":5}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `axis` | `x` / `y` / `z` | 项目既有坐标系：**+X 前 / +Y 左 / +Z 上**（`docs/coordinate-system.md` §1） |
| `direction` | `+` / `-` | 沿轴正/负向 |
| `step` | > 0，单位 **mm** | 相对当前位姿的位移量 |

语义：**相对**运动。基准取 `Snapshot()` 的**命令值**（目标位），不是设备回读的
过程值 —— 否则连续 `+X` 会每一步都从"还在斜坡上"的读数再加，永远追不上。

执行链：`当前关节角 → FK → 目标点 ± step → IK → 关节角 → controller.Apply`。

### 3.2 `gripper` —— 夹爪开合

```json
{"cmd":"gripper","action":"open"}
{"cmd":"gripper","action":"close"}
```

端点取自 `robot.yaml` 里 `role: gripper` 关节的**限位**（本机型 10°..100°）：
θ 增大 = 张开（`docs/coordinate-system.md` §3），故 `open=max`、`close=min`。
**不写死角度。**

### 3.3 `servo` —— 直接给舵机角度

```json
{"cmd":"servo","servo":1,"angle":90}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `servo` | `1..N` | 编号按 `Model.JointOrder()`，本机型 **1=base(S9) 2=shoulder(S7) 3=elbow(S8) 4=gripper(S6)** |
| `angle` | 舵机角（degree） | 必须落在 `robot.yaml` 的 `actuators[].limits` 内 |

校验**复用**舵机硬件行程 `Actuator.Limits`，再经 `ServoToJoint` 换算成关节角
交给 `Apply` ⇒ 关节限位仍由 controller 兜底。两套限位同源。

### 3.4 `state` —— 读当前状态（**只读**，v1.1 补）

```json
{"cmd":"state"}
```

无参数、**无副作用**：不调用 `Apply`、不碰 `device`、不占 ACK 门控。
应答与写命令的成功应答同构（`ok:true` + `state`）。

**为什么必须有它。** v1 的前三条命令全是**写**命令 —— 任何外部控制器在"首次下发
之前"都拿不到当前舵机角。而舵机级控制是**绝对角**语义（`servo` 要的是目标角），
没有基准就只能猜，猜错的第一帧就是一次可见的跳变。
`MeArm-RemoteControl` 的网络模式正是这个场景：它连上后先发一条 `state` 拿基准，
再开始增量下发。

它同时兑现了「状态反馈优先复用服务端真值」的要求：**没有新增状态系统**，
只是把已有的 `h.state()` 暴露成一个入口。

> ⚠️ 返回的 `servo` / `joints` 仍是**命令值**（= 最近一次被受理的目标），
> 不是设备过程值 —— 与 §3 其它命令、以及 WS 侧 `joint_states` 的语义一致（见 §8）。

---

## 4. 应答

成功：

```json
{"ok":true,"cmd":"move","state":{
  "joints":{"base":0,"shoulder":0.85,"elbow":112.62,"gripper":50},
  "servo":[90,90.0,90.0,40],
  "tcp":[115.033,0,109.224],
  "device":"sim"}}
```

失败：

```json
{"ok":false,"cmd":"move","error":"OUT_OF_WORKSPACE: ..."}
```

`state` 只在成功时出现；`servo` 数组按 §3.3 的编号排列；`tcp` 是末端位置
`[x,y,z]`（mm）。

### 错误码（全部为 `error` 文本，v1 不引入数字码）

| `error` | 触发 |
|---------|------|
| `empty command` | 空行 |
| `invalid json` | 解析失败 |
| `missing cmd` | 无 `cmd` 字段 |
| `unknown cmd "<x>"` | 未知命令 |
| `invalid axis` / `invalid direction` / `missing step` / `invalid step` | `move` 参数问题（`step` 必须 > 0 且非 NaN/Inf） |
| `OUT_OF_WORKSPACE: …` | 目标点不在可达壳层内 |
| `JOINT_LIMIT: …` | IK 有解但解落在关节限位外 |
| `missing action` / `invalid action` | `gripper` 参数问题（只认 `open`/`close`） |
| `missing servo` / `invalid servo (expected 1..N)` / `missing angle` / `invalid angle` | `servo` 参数问题 |
| `angle out of range (<id> min..max)` | 舵机角超出该舵机硬件行程 |
| `no state available yet` | `state` 收不到快照（正常路径不可达：`controller.New` 已把命令值初始化为 HOME） |

任何错误**只影响这一条命令**，连接保持可用（验收中"错误轰炸 10 条后仍可正常
下发"是硬指标）。

---

## 5. 安全与并发

| 项 | 做法 |
|----|------|
| 限位 | 关节限位走 `Model.Validate`（`controller.Apply` 内部）；舵机行程走 `Actuator.Limits`。TCP 层**不持有任何角度常数** |
| 不绕过设备 | 命令一律经 `controller.ApplyFrom`，与 WebSocket 同一条路（ACK 门控 / latest-wins 一致） |
| 来源标注 | 三条写命令一律声明 `origin=external`（见 §5.1），不使用 `Apply` 的默认来源 |
| 多客户端 | 允许并发连接；`Handler` 内的互斥量把「读当前 → 算目标 → 下发」做成原子操作，避免相对位移互相踩踏 |
| 崩溃隔离 | 每连接独立 goroutine + `recover()`；单连接 panic 不会拖垮后端进程 |
| 非法输入 | `Execute` 对任何输入都返回 `Result`，不返回 error、不 panic |

### 5.1 为什么三条写命令都带 `origin=external`

WS 侧的状态帧带一个 `origin` 字段（字面量见 `internal/protocol`），**它决定对端页面
要不要让"命令侧跟随"**：

| `origin` | 含义 | 对端页面的行为 |
|----------|------|----------------|
| `command` | 由对端**自己的**命令引起（`JR` 应答 / 回读） | 只更新"实际臂"，命令侧不动 |
| `device` | 设备**自主**变化（硬件摇杆 / 红外 / 手拧 / 调试直控） | 命令侧跟随 + 抑制回发 |
| `external` | **另一台上位机**经本接口下发的命令 | 命令侧跟随 + 抑制回发（同 `device`） |

TCP v1 的写命令若借用 `command`，对端会表现为：

1. **画面上只有半透明的"幻影臂"在动，不透明的主臂与滑杆停在旧值** ——
   看起来像渲染故障，实际是来源标注错了。
2. **更危险**：对端的 `commandJoints` 停在旧值。用户下一次动页面上**任何一个**
   控件，就会把**整组旧指令**一次性下发 —— 真机上表现为机械臂突然跳回旧位姿。

因此 `move` / `gripper` / `servo` 一律经 `Arm.ApplyFrom(protocol.OriginExternal, …)`
下发。这**只新增一个来源字面量**，不改任何既有命令的语义、几何、限位与应答格式；
不带 `origin` 的旧客户端行为与引入前完全一致。

> 判据落点：`internal/tcpserver/protocol_test.go::TestWriteCommands_CarryExternalOrigin`
> （三条写命令逐一断言来源）、`internal/controller/controller_test.go` 的两条
> （外部命令的状态帧标 `external`、在途 latest-wins 场景来源不丢）。

---

## 6. 几何真值从哪来

后端原先**没有** FK/IK（WebSocket 是纯关节级协议）。`move` 需要它，因此新增
`internal/robot/kinematics.go` —— 但：

- 几何 `Geom{PivotZ, L1, L2, ToolR}` **全部由 `robot.yaml` 的 `links[].length` 按
  关节角色推导**（本机型 = `60 / 80 / 80 / 40` mm），不写死；缺链即报错，不静默兜底。
- 正确性以**冻结基线**为判据：`robot-package/mearm-v1/tests/cases/fk_cases.json`
  （116 例）与 `ik_cases.json`（121 例，含 6 例已知失败分类）。
  实测：FK 最差 **7.1e-13 mm**、IK 往返最差 **7.4e-14 mm**、失败分类 6/6 一致。
- 腕部是**被动**关节（夹爪锁定水平）⇒ "腕支点→TCP" 是恒定水平偏置 `ToolR`，
  **不是**会转的连杆；肘角是**绝对角**（`θe = θs + α`，不要再叠加肩角，见
  `docs/coordinate-system.md` D18）。

---

## 7. 手动冒烟

```bash
# 一次性验收（28 项，含六向位移几何校验、错误轰炸、超长行隔离）
python MeArm-3D/core/tools/tcp_control_client.py

# 单条命令
python MeArm-3D/core/tools/tcp_control_client.py --cmd '{"cmd":"gripper","action":"open"}'

printf '{"cmd":"move","axis":"z","direction":"+","step":2}\n' | nc 127.0.0.1 9100
```

---

## 8. 未覆盖 / 已知限制

- **不提供**绝对位姿直输（WebSocket 侧也没有；需要的话属于 v2，不在本次范围）。
- `move` 只解平面 2R（base 旋转 + 肩肘），与前端 IK 同一套语义；腕部被动。
- `state.joints` 返回的是**命令值**（刚下发的目标），不是设备过程值 ——
  要看设备实际到位情况，读 WebSocket 的 `joint_state{origin:"device"}`。
- 无鉴权：接口假定运行在可信局域网内。放到公网前必须加认证（v2）。
