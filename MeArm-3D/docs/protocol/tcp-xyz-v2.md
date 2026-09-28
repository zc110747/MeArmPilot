# TCP JSON 控制接口 v2 —— XYZ 矢量命令

> 面向**三轴同时变化**的操作接口。v1 的 `move` 一次只能动一个轴，
> v2 补上矢量形式与绝对定位。
>
> 传输、坐标系、通用应答字段见 [`README.md`](./README.md)；
> v1 四命令见 [`tcp-v1.md`](./tcp-v1.md)。

新增四条：

| `cmd` | 语义 | 写/只读 |
|-------|------|---------|
| `movexyz` | 三轴**同时**相对位移 | 写 |
| `moveto` | **绝对**目标点 | 写 |
| `home` | 回 HOME 位姿 | 写 |
| `caps` | 读几何 / 限位 / 行程 / 可达包围盒 | **只读** |

---

## 1. `movexyz` —— 三轴同时相对位移

```json
{"cmd":"movexyz","delta":[5,-3,2]}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `delta` | `[dx, dy, dz]`，**长度必须恰好 3**，单位 mm | 相对当前**命令位姿**的位移矢量 |

执行链：`当前关节角 → FK → 目标点 + delta → 一次 IK → 一次 Apply`。

### 为什么不是连发三条 `move`

不只是"少两趟往返"：

1. 三次 `move` 是三次「读当前 → IK → Apply」，第二次的基准已是第一次下发后的
   命令值 —— 串行叠加在**限位边界附近**会逐步走到一个单次直达不会选的分支
   （2R 有 elbow-up / elbow-down 两支，`nearest` 选支依赖当前姿态）。
2. 手势控制这类"每帧一个增量"的场景，一次到位才谈得上实时。

### 与 `move` 的关系

**同一套语义**：基准是命令值而非设备过程值，失败原因同样透传
（`OUT_OF_WORKSPACE` / `JOINT_LIMIT`）。

代码上二者共用同一条 `applyTarget` 尾巴，且有一条测试**逐位比对**
单轴 `move` 与 `movexyz` 的结果
（`protocol_xyz_test.go::TestMove_SingleAxisEqualsMoveXYZ`）——
`move axis=x direction=+ step=5` 与 `movexyz delta=[5,0,0]` 必须完全一致。

### 参数错误

| `error` | 触发 |
|---------|------|
| `missing delta` | 没给 `delta` |
| `invalid delta (expected [delta] with 3 numbers, got N)` | 长度不是 3 |
| `invalid delta` | 含 NaN / Inf |
| `invalid delta: zero displacement` | **全零矢量** |

> ⚠️ **全零位移按参数错误拒绝**（与 v1 `move` 的 `step > 0` 同一条规则）。
> 手势控制每帧都可能算出零增量 —— 客户端应当**跳过**这一帧，不要下发，
> 也不要把它当故障。判据是错误文本里那句 `zero displacement`。

---

## 2. `moveto` —— 绝对目标点

```json
{"cmd":"moveto","xyz":[120,10,100]}
```

| 字段 | 取值 | 说明 |
|------|------|------|
| `xyz` | `[x, y, z]`，**长度必须恰好 3**，单位 mm | 绝对目标点，项目既有坐标系 |

v1 刻意没有它（曾列为 v2）。手势控制需要它：**"手指向某个位置"是绝对语义**，
用一串相对增量去逼近会累积漂移，而且每帧都要先读一次 `state` 才知道基准。

### 幂等性（关键判据）

连续下发**同一个**目标点必须**停在原地**。若实现把它当成相对量，
第一次到位、第二次就开始漂 —— 而"命令被受理了"的用例完全看不见这种错误。

### 参数错误

| `error` | 触发 |
|---------|------|
| `missing xyz` | 没给 `xyz` |
| `invalid xyz (expected [xyz] with 3 numbers, got N)` | 长度不是 3 |
| `invalid xyz` | 含 NaN / Inf |
| `OUT_OF_WORKSPACE: …` | 目标点不在可达壳层内 |
| `JOINT_LIMIT: …` | 几何可达，但两支解都撞关节限位 |

> ⚠️ `[0,0,0]` 是**合法的目标点**（不是参数错误）。它通常不可达，
> 但那该由 `OUT_OF_WORKSPACE` / `JOINT_LIMIT` 说出来，而不是被参数校验
> 当成"坐标格式不对"—— 与 `movexyz` 的全零规则**故意不同**，别混。

---

## 3. `home` —— 回 HOME 位姿

```json
{"cmd":"home"}
```

目标取自 `robot.yaml` 的 `homePose`，**不写死角度**。

它不是"XYZ 命令"，但外部控制器都要一个"归位"动作 —— 手势交互里那通常就是
"张开手掌 / 握拳"的复位，也是坐标系映射的基准点。

| `error` | 触发 |
|---------|------|
| `model has no home pose` | 模型没有 `homePose`（正常路径不可达） |

---

## 4. `caps` —— 能力描述（**只读**）

```json
{"cmd":"caps"}
```

把外部控制器需要的常量一次交代清楚。**存在的理由**：外部项目（visionflow）
要把"手的归一化坐标"映射到"机械臂的 mm"，就必须知道可达范围与限位 ——
不给它这些，它只能把边界抄进自己的代码，那立刻就是**第二份真值**。

应答里每个数字都由 `robot.yaml` 派生。除 `caps` 外还附带一份常规 `state`。

### 真机实测应答

```json
{"ok":true,"cmd":"caps","caps":{
  "protocol":2,
  "device":"sim",
  "units":{"length":"mm","angle":"deg"},
  "frame":{"x":"front","y":"left","z":"up","handed":"right",
           "origin":"shoulder pivot axis, on the base plate plane"},
  "geom":{"pivotZ":60,"l1":80,"l2":80,"toolR":40,"reachMin":0,"reachMax":160},
  "joints":[
    {"id":"base","role":"base","min":-60,"max":60},
    {"id":"shoulder","role":"shoulder","min":-6.0936827341,"max":49.454929245},
    {"id":"elbow","role":"elbow","min":108.4414852068,"max":141.8582211436},
    {"id":"gripper","role":"gripper","min":10,"max":100}],
  "servos":[
    {"n":1,"id":"servo_9","joint":"base","min":30,"max":150,"unit":"deg"},
    {"n":2,"id":"servo_7","joint":"shoulder","min":80,"max":160,"unit":"deg"},
    {"n":3,"id":"servo_8","joint":"elbow","min":20,"max":100,"unit":"deg"},
    {"n":4,"id":"servo_6","joint":"gripper","min":40,"max":130,"unit":"deg"}],
  "home":{"base":0,"shoulder":0.8498937633,"elbow":112.6185771989,"gripper":50},
  "workspace":{"min":[40.46,-153.01,49.08],"max":[175.88,153.01,114.68],"samples":12},
  "commands":["move","gripper","servo","state","movexyz","moveto","home","caps"]},
 "state":{...}}
```

### 字段

| 字段 | 说明 |
|------|------|
| `protocol` | 协议版本（当前 2）。**只在新增加命令时递增**，改语义不算 |
| `device` | 链路末端 `sim` / `serial` / `mujoco` |
| `units` | 单位，写死在应答里而不是靠文档约定 |
| `frame` | 坐标系朝向 + 原点说明 |
| `geom.pivotZ/l1/l2/toolR` | 机构几何（mm），由 `robot.yaml` 的 `links[].length` 按角色求导 |
| `geom.reachMin/reachMax` | **腕枢轴**的可达壳半径 = `\|L1−L2\|` .. `L1+L2` |
| `joints[]` | 关节软限位，**顺序 = `servo` 命令的编号** = JR 的位次 |
| `servos[]` | 舵机硬件行程，`n` 与 `servo` 命令的编号一致（1..N） |
| `home` | HOME 位姿（关节角） |
| `workspace.min/max` | TCP 可达**包围盒**（mm），由关节限位采样 FK 求出 |
| `workspace.samples` | 每轴采样份数（复现同一组边界要用同一份数） |
| `commands` | 本服务支持的命令名 —— **能力探测用这个，不要靠版本号猜** |

> ⚠️ `reachMin/reachMax` 是**腕枢轴**的可达壳，**不是 TCP 的**：TCP 还要再水平
> 前伸 `toolR`（腕被动、爪锁水平）。做坐标钳位用 `workspace` 那一组，
> 别自己从 `reach*` 推。
>
> ⚠️ `workspace` 是**外边界**：盒内仍可能不可达（可达域是被限位切过的甜甜圈壳层，
> 中间是空的）。它的用途是**粗钳位**（盒外一定不可达），
> 真正的判据永远是命令的 `ok` 字段。

---

## 5. 安全与并发（与 v1 一致）

- 三条写命令（`movexyz` / `moveto` / `home`）一律
  `ApplyFrom(protocol.OriginExternal, …)` —— 理由见 [`tcp-v1.md` §5.1](./tcp-v1.md)。
- 关节限位走 `Model.Validate`，舵机行程走 `Actuator.Limits`，
  TCP 层不持有任何角度/尺寸常数。
- 多客户端并发时，`Handler` 内互斥量把「读当前 → 算目标 → 下发」做成原子操作
  （判据：`TestHandler_SerializesXYZCommands`，20 次并发 +1mm 必须精确累加 20mm）。
- `caps` / `state` 是只读查询，不产生任何下发。

---

## 6. 兼容性：v1 客户端无需改动

v2 是**纯新增**：

| 项 | 变化 |
|----|------|
| v1 的命令 / 参数 / 错误文案 / 错误优先级 | **零变化** |
| v1 的应答字段 | **零变化**（`caps` 是可选字段，只有 `caps` 命令才带） |
| `Command` 解析结构 | 只加了 `delta` / `xyz` 两个可选字段，老命令用不到 |
| 未知命令 | 照旧 `ok:false` + `unknown cmd "x"`，连接保持可用 |
| 端口号 / 传输格式 | 不变 |

判据落在 `backend/internal/tcpserver/protocol_xyz_test.go`：

- `TestMove_SingleAxisEqualsMoveXYZ` —— 六个单轴方向，`move` 与 `movexyz`
  的下发关节角与 `state.tcp` **逐位相同**。
- `TestV1Commands_UnchangedAfterV2` —— v1 四条仍可用，且 `move` 的错误优先级
  （非法 axis 先于缺 step）没变。

> 换句话说：v2 的红线不是"新命令对不对"，而是"**旧命令有没有被顺手改坏**"。
> 上面两条就是那个警报器。

---

## 7. 已知限制

- `movexyz` / `moveto` 只解平面 2R（base 旋转 + 肩肘），腕部被动、爪恒水平。
  **不给姿态（orientation），只给位置** —— 本协议定位的是 TCP 点，不是完整位姿。
- 无轨迹规划：一条命令 = 一次直达。中间路径不做避障，也不保证不扫到东西。
- 无速度/加速度控制：走多快由设备侧的角速度决定。
- 无鉴权。
