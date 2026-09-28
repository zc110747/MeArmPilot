# TCP JSON 控制接口 v1

> v1 的四条命令：`move` / `gripper` / `servo` / `state`。
> v2 追加的 XYZ 矢量命令见 [`tcp-xyz-v2.md`](./tcp-xyz-v2.md)。
> 传输格式、坐标系、通用应答字段见 [`README.md`](./README.md)。

**v2 没有改动这里任何一条命令** —— 语义、参数、错误文案、错误优先级全部照旧。
判据：`backend/internal/tcpserver/protocol_xyz_test.go`
的 `TestMove_SingleAxisEqualsMoveXYZ`（单轴 `move` 与 `movexyz` 逐位一致）
与 `TestV1Commands_UnchangedAfterV2`。

---

## 1. `move` —— 单轴相对位移

```json
{"cmd":"move","axis":"x","direction":"+","step":5}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `axis` | `x` / `y` / `z` | 项目既有坐标系：**+X 前 / +Y 左 / +Z 上** |
| `direction` | `+` / `-` | 沿轴正/负向 |
| `step` | **> 0**，单位 mm | 相对当前位姿的位移量 |

语义：**相对**运动。基准取 `Snapshot()` 的**命令值**（目标位），不是设备回读的
过程值 —— 否则连续 `+X` 会每一步都从"还在斜坡上"的读数再加，永远追不上。

执行链：`当前关节角 → FK → 目标点 ± step → IK → 关节角 → controller.Apply`。

> 一次只想动一个轴时用这条；**三轴同时动请用 v2 的 `movexyz`** ——
> 连发三条 `move` 是三次「读当前 → IK → Apply」，在限位边界附近会逐步走到
> 一个单次直达不会选的分支。

---

## 2. `gripper` —— 夹爪开合

```json
{"cmd":"gripper","action":"open"}
{"cmd":"gripper","action":"close"}
```

端点取自 `robot.yaml` 里 `role: gripper` 关节的**限位**（本机型 10°..100°）：
θ 增大 = 张开（`docs/coordinate-system.md` §3），故 `open=max`、`close=min`。
**不写死角度。**

---

## 3. `servo` —— 直接给舵机角度

```json
{"cmd":"servo","servo":1,"angle":90}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `servo` | `1..N` | 编号按 `Model.JointOrder()`，本机型 **1=base(S9) 2=shoulder(S7) 3=elbow(S8) 4=gripper(S6)** |
| `angle` | 舵机角（degree） | 必须落在 `robot.yaml` 的 `actuators[].limits` 内 |

校验**复用**舵机硬件行程 `Actuator.Limits`，再经 `ServoToJoint` 换算成关节角
交给 `Apply` ⇒ 关节限位仍由 controller 兜底。两套限位同源。

> ⚠️ `servo` 是**绝对角**语义：下发前必须先知道当前角度，否则第一帧就是一次跳变。
> 这就是为什么要有下一条 `state`。

---

## 4. `state` —— 读当前状态（**只读**）

```json
{"cmd":"state"}
```

无参数、**无副作用**：不调用 `Apply`、不碰 `device`、不占 ACK 门控。
应答与写命令的成功应答同构（`ok:true` + `state`）。

**为什么必须有它。** 前三条全是**写**命令 —— 任何外部控制器在"首次下发之前"
都拿不到当前舵机角。而舵机级控制是**绝对角**语义，没有基准就只能猜，
猜错的第一帧就是一次可见的跳变。`MeArm-RemoteControl` 的网络模式正是这个场景：
连上后先发一条 `state` 拿基准，再开始增量下发。

它同时兑现了「状态反馈优先复用服务端真值」的要求：**没有新增状态系统**，
只是把已有的 `h.state()` 暴露成一个入口。

> ⚠️ 返回的 `servo` / `joints` 仍是**命令值**（= 最近一次被受理的目标），
> 不是设备过程值 —— 与 §1–§3、以及 WS 侧 `joint_state` 的语义一致。

---

## 5. 错误码

**全部为 `error` 文本，v1 不引入数字码。**

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

任何错误**只影响这一条命令**，连接保持可用（验收里"错误轰炸 10 条后仍可正常
下发"是硬指标）。

`move` 的错误**优先级**是契约的一部分：
`axis` → `direction` → `step` 缺失 → `step` 非法 → 无位姿 → IK。
改顺序会让"给了非法 axis 却报 OUT_OF_WORKSPACE"这类回归溜过去。

---

## 5.1 为什么写命令都带 `origin=external`

WS 侧的状态帧带一个 `origin` 字段（字面量见 `backend/internal/protocol`），
**它决定对端页面要不要让"命令侧跟随"**：

| `origin` | 含义 | 对端页面的行为 |
|----------|------|----------------|
| `command` | 由对端**自己的**命令引起（`JR` 应答 / 回读） | 只更新"实际臂"，命令侧不动 |
| `device` | 设备**自主**变化（硬件摇杆 / 红外 / 手拧 / 调试直控） | 命令侧跟随 + 抑制回发 |
| `external` | **另一台上位机**经本接口下发的命令 | 命令侧跟随 + 抑制回发（同 `device`） |

TCP 的写命令若借用 `command`，对端会表现为：

1. **画面上只有半透明的"幻影臂"在动，不透明的主臂与滑杆停在旧值** ——
   看起来像渲染故障，实际是来源标注错了。
2. **更危险**：对端的 `commandJoints` 停在旧值。用户下一次动页面上**任何一个**
   控件，就会把**整组旧指令**一次性下发 —— 真机上表现为机械臂突然跳回旧位姿。

因此 `move` / `gripper` / `servo`（以及 v2 的 `movexyz` / `moveto` / `home`）
一律经 `Arm.ApplyFrom(protocol.OriginExternal, …)` 下发。

> 判据落点：`internal/tcpserver/protocol_test.go::TestWriteCommands_CarryExternalOrigin`、
> `protocol_xyz_test.go::TestXYZWriteCommands_CarryExternalOrigin`、
> `internal/controller/controller_test.go` 的两条（外部命令的状态帧标 `external`、
> 在途 latest-wins 场景来源不丢）。

---

## 6. 几何真值从哪来

后端原先**没有** FK/IK（WebSocket 是纯关节级协议）。`move` 需要它，因此新增
`backend/internal/robot/kinematics.go` —— 但：

- 几何 `Geom{PivotZ, L1, L2, ToolR}` **全部由 `robot.yaml` 的 `links[].length` 按
  关节角色推导**（本机型 = `60 / 80 / 80 / 40` mm），不写死；缺链即报错，不静默兜底。
- 正确性以**冻结基线**为判据：`robot-package/mearm-v1/tests/cases/fk_cases.json`
  （116 例）与 `ik_cases.json`（121 例，含 6 例已知失败分类）。
- 腕部是**被动**关节（夹爪锁定水平）⇒ "腕支点→TCP" 是恒定水平偏置 `ToolR`，
  **不是**会转的连杆；肘角是**绝对角**，不要再叠加肩角（ADR **D18**）。

外部项目要自己算 FK/IK 的话，公式就在 `kinematics.go` 文件头；几何常量用
`caps` 拿（[`tcp-xyz-v2.md` §4](./tcp-xyz-v2.md)），**不要抄进自己的代码**。

---

## 7. 已知限制

- v1 **不提供**绝对位姿直输 —— v2 的 `moveto` 补上了。
- `move` 只解平面 2R（base 旋转 + 肩肘），与前端 IK 同一套语义；腕部被动。
- `state.joints` 返回命令值，不是设备过程值。
- 无鉴权。
