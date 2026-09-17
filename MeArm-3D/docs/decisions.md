# 设计决策记录（ADR）

记录"为什么这么做"，尤其是**与原 spec 示例不一致**的地方，方便后续复盘与修改。
每条都有编号，代码注释会引用编号（如 `D2`）。

> 本文**最新条目在前**（D84 在最上，D1 在最下）。

## D84 · 新增 `origin=external`：让「另一台上位机在驱动」被对端页面正确跟随

**背景**：`MeArm-RemoteControl`（`arm-web`）在 network 模式下作为 TCP Client 经 9100
驱动本后端的 Sim。功能上链路是通的（摇杆 → Sim → 状态回推），但用户在 MeArm-3D 页面上
看到的现象是：**只有半透明的"实际臂幽灵"在动，不透明的主臂不动**。

**根因**（代码 + 实测双证）：前端有两条渲染路径 —— 主臂画 `commandJoints`（"要去哪"），
幽灵画 `actualJoints`（"现在在哪"）。`transportBridge.handleState` 只对后端标注
`origin === 'device'` 的帧做「命令侧跟随」（写 `commandJoints` + 抑制回发），
其余来源一律只写 `actualJoints`。而 TCP 入口**借用了 `command`** —— 因为 `Apply` 的
默认来源就是它。探针实测：经 9100 发一条 `servo`，抓 `/ws/joint` 得 **8/8 帧
`origin=command`、零帧 `device`**，与推断一致。

**危害不止观感**：对端的 `commandJoints` 停在旧值。用户下一次动页面上**任何一个**控件，
会把**整组旧指令**一次性下发 —— 真机上表现为机械臂突然跳回旧位姿。

**决策**：新增第三个来源常量 `OriginExternal`。`move` / `gripper` / `servo` 一律经
`Arm.ApplyFrom(protocol.OriginExternal, …)` 下发；前端把它与 `device` **同等对待**
（跟随 + 抑制回发）。

**为什么不让对端"凡不是自己引起的一律跟随"**（即把 `command` 也纳入跟随）：
`command` 帧由对端**自己**刚下发的命令引起，命令侧**本来就在那个值上**，跟随是 no-op；
但在快速拖动时会有多条命令在途，**旧命令**的状态帧到达时去写命令侧，会把用户的**新**
拖动位置覆盖掉 —— 表现为拖动时指针回弹/抖动。所以 `command` 必须保持不跟随。

**为什么不复用 `device`**：语义不同，而且**来源必须可观测区分**。`device` 的含义是
"设备侧自主变化（摇杆 / 红外 / 手拧）"，其日志文案就是照此写的；外部入口若借 `device`，
排障面板会对着现场**误报**一次"手动拨动了机械臂"，掩盖真正的原因（另一台上位机在下命令）。
日志文案分开是本次特意保留的判据之一。

**实现上的两个易错点**（都已固化在注释里）：

1. **来源必须随命令同行**，不能等状态帧到了再从 `inflight` 取：`OK JR` 一到就
   `finishInflight()` 清空 inflight，而后续 `STATE` 推进帧仍属于刚发出的命令。
   因此来源存进 `curOrigin` / `pendingOrigin`，与 joints 一起走 `dispatch → send`。
   在途 latest-wins 时进待发槽的 `pendingOrigin` 也必须一起覆盖，否则尾沿补发会用错来源。
2. **前端有两层 origin 闸门**，只改一层不够：后端常量层 + `WebSocketTransport` 把
   `env.origin` 映射成 `'device' | undefined` 的**白名单**。漏改白名单的症状极具迷惑性 ——
   后端明明标了 `external`，页面却不跟随；`tsc` 也会顺带报
   `TS2367: ... '"command" | undefined' 和 '"external"' have no overlap`。

**刻意不做的事**：不改任何既有命令的语义 / 几何 / 限位 / 应答格式；不新增角度常数；
不引入订阅或推送；`Apply()` 保留为 `ApplyFrom(OriginCommand, …)` 的薄包装，
既有调用点行为逐字节不变。

**验收**：
- 后端 `go build` / `go vet` 干净，`go test ./...` 全绿（含新增 3 条来源断言）。
  并以**变异测试**反证测试有效性：把 `commandOrigin()` 临时改成恒返 `OriginCommand`，
  两条 controller 测试如实变红且报错直指根因，随后恢复。
- 前端 `tsc -b --force` 干净；vitest **33 文件 / 442 用例 PASS**（原 439 + 新增 3 条）。
- 端到端探针（`core/tools/verify_external_origin.mjs`，零依赖、无需浏览器）三段严格分离：
  本页命令 → `command`、外部 TCP → `external`、再回本页 → **回到** `command`（3/3）。
- 浏览器 e2e（`frontend/tests/e2e/external-origin-follow.mjs`，headless Edge + CDP +
  dev 探针）：外部驱动后 `commandJoints` 变化、主臂末端位移 114.97 mm、
  `|tcp − actualTcp| = 0.000 mm`、渲染侧两棵树 TCP 世界坐标重合 0.000 mm、
  日志出现"外部入口在驱动"且不误报"设备侧自主变化"、**未触发本页下发**
  （9 PASS / 0 FAIL）。
- Sim2Sim 端到端仍 **30 PASS / 0 FAIL**（D83 当时记的 29/29 是补 XYZ 段前置条件之前的数字）。

**真机链路未测**（`NOT TESTED`）：本次全部验证在 Sim 上完成。

---

## D83 · TCP v1 补一个**只读** `state`：让外部客户端能建立"当前角"基准，且不碰 controller / device / 几何

**背景**：`MeArm-RemoteControl`（`arm-web`）新增 network 模式，作为 TCP Client 接入本后端的
TCP 控制接口。它的网页摇杆是**增量**语义（推杆 = 角度持续变化），必须先知道"现在在哪"
才能积分出要下发的绝对角；但 TCP v1 原有命令（`move` / `gripper` / `servo`）**全是写命令**，
没有任何只读查询 —— 客户端只能自己猜一个起步角，首次动作就与实际位姿脱节。

**决策**：给 TCP v1 加 `state` 命令，**只读**。实现只是 `dispatch` 里一个 case + `readState()`，
复用已有的 `h.state()`（它原本就用于给每条成功应答附带状态快照）：

```go
case "state":
    // 只读：不碰 Apply、不碰 device，因此不需要 h.mu（见文件头注释）。
    return h.readState()
```

**为什么这是最小改动**：不动 controller / device / IK / FK / MJCF / WS / HTTP / 前端；
不新增任何角度常数（状态里的角度来自模型与设备，不是新造的第二份真值）；
不改变任何既有命令的行为与应答格式 —— `move` / `gripper` / `servo` 的应答本就带 `state`
（经 `ok(cmd)` 统一附上），只是以前**没有办法单独索取**它。

**刻意不做的事**：不做订阅 / 主动推送（保持"一问一答"的窄接口）、不加时间戳与历史
（那会变成第二个真值源）、不接受任何参数（避免长成通用查询接口）。

**降级路径放在消费端**：客户端连不上或不认识 `state`（老版本后端）时，退回其配置里的
`fallback_angle` 并在 UI 显式上报，**不静默猜** —— 保证"新客户端 + 老后端"不会假装
自己知道当前位姿。

**验收**：`internal/tcpserver/protocol_test.go` 新增用例覆盖——只读性（`state` 不产生位移）、
基准值来自 HOME 派生而非硬编码、能反映紧随其后的写命令、`cmd` 大小写不敏感；
后端 `go test ./...` **111 PASS** + `go vet` 干净。消费端端到端（Sim2Sim）**29 / 29**。
协议文档已补 §3.4 与错误码 `no state available yet`。

**已知限制**：`state` 返回的仍是**命令值 / 目标值**（与 D82 的 `state.joints` 同源），
**不是舵机实际位置** —— 真机没有位置回读（见根 README §9）。

## D82 · TCP JSON 控制接口是**第四个入口**，不是第四个控制核心：后端运动学只在新文件里补，几何从 `robot.yaml` 推导，判据用冻结基线

**背景**：需求是给 Go 后端加一个 TCP JSON Lines 控制接口（`move` / `gripper` / `servo`），
且**最高优先级约束**是"增量、非重构"——既有 HTTP / WebSocket / Serial / Sim / Real /
IK / FK / 前端行为一律不得改变。

**核心取舍**：新入口唯一的"控制动作"是调用 `controller.Apply(关节角整帧)`。
限位校验（`Model.Validate`）、ACK 门控、latest-wins 全在 `Apply` 内部，
绕过它 = 绕过安全门。因此 TCP 层不持有任何角度/尺寸常数：

- 关节限位 → `Model.Validate`（由 `Apply` 调用）
- 舵机硬件行程 → `Actuator.Limits`（`robot.yaml` 的 `actuators[].limits`）
- 几何 → `robot.yaml` 的 `links[].length` 按**关节角色**推导（本机型 `60/80/80/40` mm）

**不得不补的东西**：审计确认后端**原本没有 FK/IK**（WebSocket 是纯关节级协议），
而 `move` 需要 XYZ→关节角。取舍是把它关进**新文件** `internal/robot/kinematics.go`，
不动既有 `robot.Load` / `yamlFile`；正确性不以自测自证，而以**冻结基线**
`robot-package/mearm-v1/tests/cases/{fk,ik}_cases.json` 当独立第三方判据
（FK 最差 `7.1e-13 mm`、IK 往返 `7.4e-14 mm`、失败分类 6/6、分支 115/115）。
这与 Sim2Sim「判据只有一份」是同一个思路：Go 是继前端 / Python 之后的第 4 个独立实现。

**相对运动的基准取 command 不取 state**：`move` 若以设备回读的过程值做基准，
连续 `+X` 会每一步都从"还在斜坡上"的读数再加 ⇒ 永远追不上。与前端
"能不能跟随 Actual"是同源的坑。

**并发在 adapter 层串行化**（`Handler.mu` 把「读当前→算目标→下发」做成原子），
不去改 controller 的 latest-wins——这是"不重构"的代价最小化解。

**默认关闭**：`tcp.enabled` 默认 `false`，端口 `9100`（避开 8090 / 5273 / 8080 / 9001）。

**验收**（真实机器未安装 ⇒ 只能 sim / mujoco）：TCP 冒烟 sim **28/28** · mujoco **28/28**；
设备跟随 sim **23/23** · mujoco **22/22**；前端 e2e **88/88**（含 Phase 8 真 Go 后端）；
前端单测 **439/439**；后端 `go test ./...` 全 ok。**前端零改动。**

**已知限制**：无鉴权，假定运行在可信局域网；`move` 只解平面 2R（腕部被动）；
`state.joints` 返回命令值而非设备过程值（要看到位情况读 WS 的 `origin:"device"` 帧）。
协议见 `docs/tcp-control-v1.md`。

## D81 · 三条设备链路对等：`# SERVO` / `origin` 在 sim·mujoco·serial 上同构，并登记两处刻意不对称

**背景**：真机接入后用户报「真机摇杆动作后机械臂会动，但**前端页面不同步状态**」。

**根因不是代码缺陷，是构建产物陈旧。** `backend/bin/armpilot-backend.exe` 是
`2026-09-15 19:03` 构建的，而 09-16 17:15 的提交 `7cec472` 改了 Go 源码却**没有重建**。
二进制符号级证据（`strings` 探测）：`device_command` **0** 次、`ORIGIN_DEVICE` **0** 次、
`ReplyServo` **0** 次；WS 探针收到 `{"code":"BAD_MESSAGE","message":"未知消息类型: device_command"}`，
且 `joint_state` 无 `origin` 字段。重建后真机链路立即 23/23 + 前端 DOM 9/9。

> ⚠️ 这条值得单独记住：`start.bat` 只在**启动时**检查"源码比二进制新"，
> **运行中的陈旧进程它发现不了** —— 那正是本次的形式。

**决策一（对等改造）**：把"设备侧自主变化 → 上位机跟随"这条语义在 MuJoCo 链路上做**对等**实现。
`simulation/mujoco/server.py` 新增 `SET` / `S<n>=` / `JOY`（单轴 + 整帧）三入口、
`# SERVO` 编码、两条上报路径分流，以及 `jr_busy` + `report_dirty` 让位闸门。

**决策二（公式只有一份真值）**：摇杆步长以固件 `core/joystick.c::joystick_delta()` 为真值，
Go `sim.go::joystickDelta` 与新增的 Python `server.py::joystick_delta` **逐值同式**。
⚠️ Python 侧 `beyond // 30` 与 C/Go 的向零取整**同值仅当 `beyond >= 0`** —— 该前提由
`raw` 先被钳到 0..1023 保证，注释里钉住了这一点。
单测表含两条**真机实测交叉验证**的用例（`(9,100)→+5`、`(7,0)→+8`）：
只测自己写的表等于自证，必须有外部读数。

**决策三（容差按链路登记，不追求统一小数）**：MuJoCo 有重力 + 有限 PD 增益，
臂停在**力矩平衡处**而非命令角 ⇒ 存在**物理静态偏差**：
实测 elbow `Δ=0.551°`、shoulder `Δ=0.274°`、base/gripper `Δ=0.000°`，
且 **3s 与 6s 读数完全一致**（是稳态误差，不是"没收敛"）。
⇒ `verify_device_follow.mjs` 的 `MUJOCO_ACK_TOL = 1.0`（sim / serial 仍用默认 0.15），
容差来源写进断言文案 `ackTolNote`，随 `--ack-tol` 可覆盖。

**复盘：废弃"空载开机位自校准"方案（自我推翻）**。
首跑 3 条 FAIL（Δ≈0.6°）时，第一反应是"用开机静态偏差自校准容差"：
`ackTol = 开机偏差 + 0.25`。复跑得出 0.451° 仍失败（实差 0.550），
且**两次开机测量给出 0.201° 与 0.59°** —— 不可复现。原因是开机快照取在 `t≈0.5s`，
**臂还在从初始 qpos 往平衡态松弛**。
⇒ 拿一个自身不可复现的量当真值，等于把噪声焊进判据。改为显式常量并写明理由。

**两处刻意保留的链路间不对称**（判据是"由谁发出"，不是"长什么样"，详见 `serial-v1.md` §3.2）：

| | `SET` 的 `origin` | `STATUS` 的翻译层 |
|---|---|---|
| `serial` | `command`（固件 `external=false`，不上报） | `serial.go::execStatus` 先翻译成一行 `STATE` |
| `sim` / `mujoco` | `device`（走 `# SERVO`） | `ParseReply` 直接判为 `ReplyServo`（实际角） |

`SET` 归属不同的理由：真机链路上 `SET` 只可能是 `JR` 的翻译产物；
而 `sim` / `mujoco` 直接处理 `JR`、从不拆 `SET`，故收到的 `SET` 必来自调试 / 验收脚本。

**只改一个文件就解决可移植性缺陷**：`config.yaml` 里写死的 `C:/Users/lx176/...`（那台机器
已不存在）改为经 `cfg.ResolveMujocoPython` 解析（显式配置 → `ARMPILOT_MUJOCO_PYTHON` →
用户目录下的隔离环境 → PATH 的 `python`）。新增三链路各一份配置：`config.yaml`(sim) /
`config.mujoco.yaml`(mujoco) / `config.serial.yaml`(serial)，共用 8090 ⇒ 不能同时启动。

**验收（现跑现取）**：`verify_device_follow.mjs` sim **23/23** · mujoco **22/22**（连跑 3 次稳定）·
serial **23/23**（真机）· 前端 DOM 读表 **9/9** · pytest `tests/sim/test_server.py` **33 passed** ·
pytest 全量 **214 passed** · `go vet` 干净 · `go test ./...` 5 包全 ok · `tsc -b --force` 0 error ·
vitest **439 passed**。

**遗留**：MuJoCo 的静态偏差是**参数化物理**（Level 3→4，增益/质量非本台标定）的必然结果，
不是模型缺陷。若要把它压到 0.1° 量级，正确做法是给物理参数做标定（或加积分项），
而不是放宽判据 —— 本次只是**把容差按链路登记清楚**，没有掩盖它。

## D80 · 夹爪（S6）标定方向修正：真机 **S6=40 张开 / S6=130 闭合**，与旧标定正好相反

**背景**：用户实测反馈「grip 的控制和真实设备相反，打开和关闭方向相反」。

旧标定（`robot.yaml` → `servo_6`）是 `offset: 40 / scale: 1 / reverse: false`，即
`servo = θ + 40`，并配 `gripper.limit = 0..90`、文档写明「θ=0 完全闭合，θ=90 完全张开」。

真机实测口径（本次确认）：**S6=40° 时爪张开，S6=130° 时爪闭合** —— 与上式恰好反。

⇒ 网页拖到「张开」（θ 大）时真机在闭合。**网页 3D 显示是对的，错的是标定表**，
所以用户看到的症状正是"虚拟臂正常、真机反向"。

**为什么不能简单把 `reverse` 翻过来**：`homePose.gripper = 50` 是 2026-09-12 实拍反解值，
必须反算出 **S6 = 90°**（固件开机位）。反向映射下两条约束互相打架：

| offset | θ=0（闭合） | θ=50（HOME） | θ=90（张开） | 问题 |
|---|---|---|---|---|
| 130 | 130 ✅ | **80 ❌** | 40 ✅ | HOME 不是 90 |
| 140 | **140 ❌** 超舵机上限 | 90 ✅ | 50 | 闭合端超限 |

**根因是 `gripper.limit = 0..90` 本身是照着旧（错）方向定的**，不是 offset 的问题。

**决策**：改标定为 `offset: 140 / scale: 1 / reverse: true`（`servo = -θ + 140`），
关节限位由舵机硬限位 40..130 **反算**得到 **`10..100`**：

| | 关节 θ | 舵机 S6 | 真机物理 |
|---|---|---|---|
| 闭合端 | **10** | 130 | 闭合 |
| HOME | 50 | **90** | 固件开机位（自洽保持） |
| 张开端 | **100** | 40 | 张开 |

三条端点全部自洽，且 HOME 仍严格等于固件 RESET 位。

**只改 `robot.yaml` 一个文件** —— 前后端、固件、UI 一律照读，无一处代码改动。

**后果（本次连锁，全部已处理）**
- 生成物过期 ⇒ 重跑 `gen_model.py`（MJCF）/ `gen_urdf.py`（URDF）
- 冻结基线 ⇒ `freeze_baseline.py --update`（`actuators` 在语义核心白名单内；
  ⚠️ 路径键**不变**，符合规程）
- 黄金数据 ⇒ 重跑 `gen_mearm_v1_baseline.py` + `run_sim2sim.py --freeze`
- 写死旧限位/旧标定的测试断言 ⇒ 逐条更新（前端 3 处、Go 3 处）；
  其中 `robotStore.test.ts` 的两处改为**引用模型限位**而非写死端点，
  避免下次修订再制造假红
- ⚠️ `TestSerialClampedEchoSurvivesInOKJR` 的语义随方向翻转而变：
  旧写法（θ=200）在新映射下钳到**下限**，为保持"钳到上限"的原意改用 θ=−200

**验收**：pytest 198 passed · vitest 401 passed · `go test ./...` 全绿 ·
`--check` 逐位一致 · `validate --all` 通过 ·
Go 侧端到端核对（闭合端→130 / HOME→90 / 张开端→40）三项全中。

**遗留**：真机方向来自用户实测口述，尚未用相机记录（`verify_pose.py` 的
爪开合反解没有现成判据）。若后续要把这条也变成"可独立复核的读数"，
需要为夹爪设计一个近景拍摄 + 爪间距量化的标准流程。

---

## D79 · 活动机器人是 **store 状态**（不是模块常量）：初值来自配置、切换有**守卫**、`hello` **只互检不静默切换**

**背景**：引入第二台机器人后，"当前是哪一台"这件事原先写在 `robotStore.ts` 的**模块级常量**
`const model = loadRobotModel('mearm-v1')` 里 —— 它有 50 处消费点（`clipJointState` /
`deriveVirtual` / `moveTo` / 六个 action / 四个派生选择器）。于是"能加载第二台"
与"能**切到**第二台"是两件事：前者靠 `loadRobotModel(id)` 就够了，后者必须让整份状态跟着换。

**决策**：

1. **`robotId` + `model` 进 store 状态**，模块常量降级为"仅用于构造初始 state"（改名 `initialModel`
   并加注释：运行期一律 `get().model`，把常量当"当前模型"用等于限位/标定整体错位且**不报错**）。
2. 所有纯函数（`clipJointState` / `deriveVirtual`）**显式收 `model` 参数**；四个派生选择器
   （`jointLabel` / `positioningJointIds` / `gripperJointId` / `targetGapMm`）的 `model` 是
   **可选参数、缺省取当前活动模型** —— 既保住既有调用点与既有测试，又让多机器人语境能显式传入。
3. **初值来自 `config/robots.yaml → default`**（`defaultRobotId()`）。这与 spec「只通过后端
   配置文件选择模型」一致：网页端没有模型选择器，它只**跟随**配置；后端 `robot.model_id` 默认留空
   ⇒ 三端在默认情况下天然一致。
4. `setRobot(id)` 的**守卫**：`transportDriven === true`（已接入传输）时**拒绝**，并给出一条说明
   "为什么 + 怎么修"（先断开）的日志。同 `setMode` 的哲学：**校验通过才改状态**。
5. 切换时**整体复位**模型相关派生状态（joints / target / ikStatus / alignmentError / teachTrack）。
   示教轨迹是**用户数据** ⇒ 清空时必须单独记一条 `warn`，说明原有几帧、以及为什么
   （它记录的是旧模型的关节空间，换模型后回放会驱动语义不同的关节）。
6. `hello` 带 `robotId` 做**在线互检**，但**不做静默切换**：渲染一台、驱动另一台是真实危险，
   静默换掉会把"配置不一致"藏起来。

**后果**
- ✅ `tests/acceptance/robot-switch.test.ts` 13 项：往返 **200 轮**后回到 MeArm 的状态与初始
  **逐位相同**（无漂移）；切换后关节键集合恰好是新模型的；已连接时拒绝且状态一位不变；
  未知 id 拒绝且**不回退**（回退会把"id 写错一个字"变成"静默加载了另一台"）。
- ⚠️ 发现并记录了一条**真隐患**：`gripper` 是两台**同名但不同义**的关节（MeArm `0..90` vs
  SO-101 `-10..100`）⇒ **"键集合相等"不能当模型一致的判据**，只有限位能兜住。已写成显式断言。
- ✅ MeArm 行为**零变化**：既有 384 条前端断言逐条不变（改造后 409 条，新增 25 条）。

---

## D78 · 能力门必须落在 `moveTo` 上：`NO_SOLVER` 是**第三种**结果，不是"不可达"

**背景**：`store.moveTo()` 直接调 `solveIk(model, xyz, …)` —— 那是 MeArm 的**平面 2R 解析解**。
把它对着 6 铰链的 SO-101 跑，会发生本项目最危险的一类失败：**它会成功返回**。

```text
  solveIk(soArmModel, [200, 0, 150])
    └─ 取 base/shoulder/elbow 三个"角色"，按 2R 几何解出一组角
    └─ success: true, joints: {...}, residual: 一个很小的数（因为残差是它自己 FK 算的）
```

调用方若只看 `success`，就会把机械臂派到一个**语义完全错误**的位姿，而界面上一切正常。

**决策**：

1. `moveTo` 先读**能力声明**：`loadRobot(current.robotId).kinematics.capability.solverKind`。
   `!== 'analytic'` ⇒ **一次都不调用求解器**，直接返回
   `{ success:false, reason:'NO_SOLVER', message, candidates:[] }`。
2. 在 `IkReason` 里新增 `'NO_SOLVER'`，**刻意与 `'OUT_OF_WORKSPACE'` / `'JOINT_LIMIT'` 并列而不合并**：
   后两者是"试过、算出来了、不行"，前者是"**没算**"。合并会让用户以为要调目标点，而真相是
   这台机器人压根没有逆解器。
3. 拒绝时**仍然写 `target` 与 `ikStatus`**（与"目标越界也保留 target"同一取向）：界面必须显示
   "我想去哪"和"为什么没动"，不能什么都不做。
4. 日志**每个机器人只记一次**（`warnedNoSolver`）—— 拖动时 `moveTo` 是高频调用，
   否则日志面板会被同一句话刷满（与 `SoArm101Kinematics.warnNotImplementedOnce` 同一考量）。
5. **判据取自数据（`capability`）而不是 `if (robotId === 'so-arm101')`（逻辑）**：
   将来 SO-101 加了数值 IK，只需要改它自己的 `capability`，`moveTo` 一个字都不用动。

**后果**
- ✅ 前端断言：SO-101 上 `moveTo` 返回 `NO_SOLVER` 且 `commandJoints` **一位都不动**；
  MeArm 上 `moveTo` 仍走解析解（**能力门没有误伤 Golden Baseline**）。
- ⚠️ 这条门**只挡住了 store**。任何绕过 store 直接调 `solveIk(model, …)` 的新代码都不受保护
  ⇒ 新代码请走 `RobotRegistry` 的 `kinematics.inverse()`（它自己带能力声明）。

---

## D77 · 统一 Sim2Sim：判据只有**一份**；容差**按机器人登记且必须写明理由**

**背景**：Sim2Sim（同一组关节角 / 同一个目标点，前端与 MuJoCo 两侧是否落在同一处）原本是
**为 MeArm 写的**：判据在 `tests/sim2sim/`，采集入口在 `tools/gen_mearm_v1_baseline.py`。
引入第二台机器人后有两条路：

| | 做法 | 后果 |
|---|---|---|
| ✗ | 照抄一份 SO-101 的 | 两台机器人各有一套验收标准，**两套各自都能自己绿** ⇒ "用同一套判据"就只是说法 |
| ✅ | 抽出 `runSim2Sim(robot)` | 判据只有一份，两台跑同一份代码；新增机器人自动进入矩阵 |

**决策**：

1. 唯一入口 `simulation/mujoco/sim2sim.py:run_sim2sim(robot_id)` + CLI `tools/run_sim2sim.py`。
   **三侧面**（前端桥 / Python 参考实现 / MuJoCo `MjModel`）一个不少 —— 把参考实现换成
   "再调一次前端"会把三侧面退化成**一条自证链**。
2. **IK 段由 `capability.solverKind` 决定，不是 `if robot == …`**。没有逆解器时：
   **不做任何求解**，但**仍然发几个探测目标过去**，把"前端确实拒绝了、理由是 `NOT_IMPLEMENTED`"
   记录下来。一条拒绝记录是**证据**，"我们没测"是空白 —— 两者完全不同。
3. 没有的东西**不做基线、不写字段**：报告里没有 `workspace` / `geometry`，
   快照里也没有（`test_no_robot_fabricates_ik` 与前端基线测试都盯着这一条）。
4. **容差按机器人登记，且必须写明理由；未登记一律报错**（`FK_TOL_MM`，**不设缺省值**）：

   | robotId | 容差 | 理由 |
   |---|---|---|
   | `mearm-v1` | `1e-6 mm` | `robot.yaml` 与 `fkref.py` 同源（都是本项目写的），实测 `~1e-13` |
   | `so-arm101` | `5e-2 mm` | 官方 URDF 的 `<origin rpy>` 截断到 6 位有效数字（`1.5708 ≠ π/2`），官方 MJCF 用四元数归一化后恰好 90° ⇒ **两份官方文件自身**差 µm 级，实测 `~3.3e-3` |

   悄悄给个"够大"的缺省，等于把"新增机器人时没人想过它的精度来源"这件事藏起来；
   而那正是"为了让它通过而放宽阈值"的开端。
5. **两批独立采集的产物必须互相对照**：MeArm 的 `sim2sim.json`（本工具采集）与 4 份
   `tests/baseline/mearm-v1/*.json`（`gen_mearm_v1_baseline.py` 采集）在**同名用例**上
   FK 必须逐位相同 ⇒ 抽出统一框架时**顺手改掉 MeArm 期望值**这条风险被钉住
   （`test_mearm_sim2sim_agrees_with_the_four_file_golden_baseline`）。

**后果**
- ✅ `tools/run_sim2sim.py --all` 一条命令输出两行矩阵；`pytest tests/sim` **161 passed**
  （新增 14 项矩阵用例）；前端新增 12 项读**同一批**冻结文件。
- ✅ MeArm 侧数值与抽框架前一致（`1e-13` 量级），4 份黄金数据**逐位未变**。
- ⚠️ 唯一新增的"两个标准"是**故意的**：`tests/sim2sim/`（MeArm 黄金基线回归）与
  `sim2sim.json`（统一框架快照）是**两种用途**——前者锚"与冻结时一致"，后者锚"两台机器人在
  同一套判据下的表现"。前者只 MeArm 有，后者每台都有；交叉一致性判据把两者连起来。

---

## D76 · SO-101 的三条真值取舍：**能读到"引擎实际用的值"就绝不读文本**；官方没声明的必须写出来

**背景**：引入官方 SO-ARM101 时，同一批事实在官方**两份文件**（URDF / MJCF）里各有一份，
且**数值不等**。选错会得到"看起来跑通了、但和仿真不是同一个机器人"的模型。

**实测到的三处不一致**（全部逐位核对过，见 `assets/models/so-arm101/official/SOURCE.md §4.2`）：

| 事实 | URDF | MJCF | 取谁 / 为什么 |
|---|---|---|---|
| TCP 帧朝向 | `gripper_frame_joint` 的 `rpy=[0,π,0]` | `gripperframe` site 的 `quat=Ry(π/2)` | **MJCF** —— 位置逐位相同、姿态**差 90°**。取 MJCF 后 FK↔MuJoCo 姿态差 **0.0007°**；取 URDF 则差 **90.0004°**。ArmPilot 的 MuJoCo 侧读的就是它 |
| 关节限位 | `1.91986`（**截断到 6 位小数**） | `1.9198621771937616`（满精度，**正好 110°**） | **MJCF** —— MuJoCo **执行的是** `range`。取 MJCF 才能保证「配置里声明的区间 ≡ 仿真里生效的区间」 |
| 物理量（kp/kv/forcerange/damping/质量） | 无 | 全都有 | **MJCF**，且 `physics.yaml` **一个数值都不复制** |

**决策**：

1. **限位 / TCP 帧 / 物理量一律取 MJCF**；`robot.yaml` 的 link/joint **几何**（origin / axis / mesh）
   取 URDF（它是唯一把它们写全的来源），欧拉角用 `rotationConvention: rpy` **原样承载**
   —— 换算成 intrinsic XYZ 会让 yaml 里出现一批在官方文件里**查不到的数**。
2. `physics.yaml` **不复制任何物理量**：只放 ① 真值声明（含固定 commit 与 sha256）
   ② ArmPilot 自己的驱动参数 ③ **审计快照** ④ **官方未声明的东西**。
3. 审计快照由 `tools/inspect_so101_physics.py --check` 逐值复核，且**从 `MjModel` 读而不是读 XML 文本**
   —— 这条不是洁癖：XML 的 `class` 里写 `forcerange="-2.94 2.94"`，而 6 个 `<position>`
   **逐个覆盖**成 ±3.35；**文本会骗你，MjModel 不会**。
4. **官方没声明的必须显式写出来**（`not_declared_by_official`）：无地面/工作台、**无 `<contact><exclude>` 对**
   （⇒ 相邻连杆默认会互相碰撞）、无关节速度上限、无独立标定段、质量来自 CAD 而非称重。
   把"缺失"写下来，否则缺失会被默认成"应该有、大概没问题"。

**后果**
- ✅ `inspect_so101_physics.py --check` 46 项吻合；**已做反向验证**（改 kp / damping / timestep /
  mass / 删段五种变异全部被抓出）⇒ 它不是一条永远为真的断言。
- ✅ FK↔MuJoCo 实测残差 **位置 2.0~2.3 µm、旋转矩阵 ≤ 1.0e-5**。
- ⚠️ **该残差有根因、不是换算错误**：官方 URDF 把 `<origin rpy>` 截断到 **6 位有效数字**
  （`1.5708` ≠ π/2、`3.14159` ≠ π），而 MJCF 里形如 `[0.707107,0,0.707107,0]` 的四元数
  **归一化后恰好是 90°** ⇒ 两份文件自身就有 ~2 µm 的系统差。robot.yaml 的 origin 取自 URDF 原文
  （可逐个复核），故保留该微差；测试容差取 10 µm / 1e-4（离实测 4~10 倍余量，仍足以拦住
  "欧拉角约定用反"的 90° 与"origin 抄错"的 mm 级）。
- ⬜ 官方 MJCF 没有 floor/table 也没有 exclude 对 ⇒ **Phase 6 要单独裁决**是否在**运行期**
  （不改官方文件）补碰撞体与排除对，并实测是否出现自穿模伪接触。

## D75 · 第二个机器人靠**配置选择**加载：选择器只做选择，分派收敛到**一张表**

**背景**：spec 要求「只通过后端配置文件选择机器人，业务代码不得堆积 `if robot == ...`」，
且 **MeArm-V1 是 Golden Baseline，冲突时优先停止抽象而不是改 MeArm**。

**决策**：

1. **三段链，各管一件事**：
   ```text
   config/robots.yaml              ← 只放 id / name / config（"有哪些、默认谁"）
   model/robotConfigRegistry.ts    ← id → yaml 原文（import.meta.glob 构建期登记）
   model/loadRobotModel(id?)       ← 按 id 缓存解析结果
   registry/RobotRegistry.ts       ← ★ 唯一分派表：工厂表(definition → engine)
   ```
   选择器里放参数的诱惑很大（"顺手把限位也写这儿"），但那就制造了第二份真值 ⇒ **禁止**。
2. **业务代码只问 `loadRobot(id)`**，拿到同一形状的 `definition + kinematics`；
   「谁有 IK」由 `kinematics.capability` **声明**（数据），不由调用方去猜（逻辑）。
   `assertRegistryCoverage()` 把"配置里加了一台、代码里没写引擎"变成**明确的错误**（测试钉住）。
3. `loadRobotModel()` 无参 = 读选择器的 `default`（当前 `mearm-v1` ⇒ **行为与改前逐位相同**）；
   同时把既有 **22 处**调用点**显式**写成 `loadRobotModel('mearm-v1')` ——
   这是**参数化**而不是"改测试"：它们本来就只测 MeArm，写出来才不会被"将来改了 default"
   静默换掉被测对象。**未知 id 抛错、不回退**：回退会让"id 写错一个字"表现成
   "静默加载了另一台机器人"，而模型不对时所有 FK/限位判据都会失真却都能跑。
4. **新增维度一律带缺省**，缺省值 = 既有语义：`RotationConvention`（缺省 `'xyz'`）、
   `ActuatorUnit`（缺省 `'deg'`）、`MeshGeometry`（新增分支，不动既有 6 类）。
   ⇒ 333→351 条既有断言**逐条不变**，`gen_mearm_v1_baseline.py --check` 4 份黄金数据**逐位一致**。
5. **两个叶子模块**专为打断循环导入而抽（不是可有可无的重构）：
   `model/configError.ts`（`RobotConfigError`）、`model/robotIds.ts`（机器人 id 常量）。
   留在原处会形成 `loadRobotModel ↔ robotConfigRegistry` 与 `RobotRegistry ↔ 实现` 两个环
   —— ESM 能容忍，但那是"靠使用时机的运气"，模块顶层用一次就是 TDZ 崩溃。
6. **IK 诚实留白**：`SoArm101Kinematics.capability = {positioningDof: 5, supportsOrientation: false,
   solverKind: 'none'}`，`inverse()` → `ikFailure('NOT_IMPLEMENTED')`
   （`success:false` / `joints:{}` / **`positionError:null`**）。
   不抄 MeArm 的平面 2R 解析解（SO-101 不是那种机构，解出来的角必然错，而 `positionError`
   还会因为**用自己的 FK 自证**而显示成一个"很小的残差"）；也不塞数值解（无收敛性/工作空间依据）。
   ⇒ **能诚实说"没实现"比伪造一个"看起来能用"的求解器重要。**

**后果**
- ✅ 新增 33 条断言（选择器 / 注册表一致性 / SO-101 定义 / FK↔MuJoCo / IK 诚实性 / MeArm 不变）
  ⇒ 前端 **384/384**；`tsc` 0 error；`pytest tests/sim` 147 + `tests/sim2sim` 9 全绿。
- ✅ 核心判据是**跨实现互证**而非自证：FK 的黄金值取自 `mj_forward()` 读 `gripperframe` site
  （MuJoCo 给出的值），姿态用**旋转矩阵**比而非欧拉角（规避万向锁假失败）。
- ⚠️ Go 侧 `internal/robot` 与 Python 侧 `robotcfg.py` 的选择器支持是 **Phase 3 剩余部分**：
  三端必须读**同一份** `config/robots.yaml`，禁止各自抄一份映射表。
- ⚠️ `loadRobot` 目前**尚未被 app 调用** ⇒ `SoArm101Kinematics` 与 `NOT_IMPLEMENTED`
  在产物里被 tree-shake（实测命中 0）。这是预期的死代码消除，Phase 4 接入渲染层后即消失。

## D74 · 首次**接管握手**：命令起点必须来自"机器现状"，否则第一帧就会多画一棵"幽灵"

**背景（用户报告）**：网页**首次加载**时机械臂显示有"重影 / 阴影"，按下 HOME（或任意命令）后消失。

**复现（纯 sim2sim，无需真机）**：把后端 sim 停在**非 HOME** 位姿再冷启动页面 ——

```bash
node tools/park_sim_pose.mjs ws://127.0.0.1:8091/ws/joint \
     --joints '{"base":0,"shoulder":-5,"elbow":110,"gripper":0}'
```

`tools/park_sim_pose.mjs` 只做"把机器停在指定位姿然后断开"这一件事，**文件内不含任何限位常量**
（真值只有 `config/robot.yaml` 一份，位姿一律由调用方传入）。逐帧探针
（`tools/first_load_probe.mjs` + `TestProbe.tsx` 的 `SceneFrameSample`）读到：

```
主臂 tcp=[115.033, 0, 109.224]   ← FK(commandJoints) = HOME（页面"假设"的位姿）
幽灵 tcp=[108.203, 0, 112.334]   ← FK(actualJoints) = 机器现状；31 帧持续存在、不收敛
```

两张用户截图做**平移对齐后的像素残差**（残差集中在臂部、`>30` 的 8727 像素），
证明那只是"半透明的第二条臂"，而不是 Z-fighting、材质或深度问题。
对照组：后端停在 HOME 时残差为 0、画面干净 —— 幽灵**重合时本来就不该被看见**。

**根因**：`robotStore` 把 `commandJoints` 初始化成 `robot.yaml` 的 `homePose`，
这是**对机器现状的假设**；连上链路之前，页面无从知道机器在哪。
`ActualGhostArm` 忠实把 `actualJoints` 画成第二棵半透明树 ⇒ 两端不一致时，
画面上就多出一棵错开的臂。三个后果按严重性排序：

1. ★ **首次下发是一记跳变**：命令侧仍是假设的 HOME，用户只动 1° 滑杆，
   下发的却是 `HOME + 1°` 整组角 ⇒ 真机从"它现在的位置"直接扑向 HOME。
2. 关节面板显示的是假设位姿而非机器现状 —— UI 在说假话。
3. 视觉上像渲染重影（用户实际看到的那一条）。

**决策**：**首次接管做一次显式握手** —— 新增 `robotStore.attachToActual(joints)`。

- `transportBridge` 在**首次** `connected` 时置 `attachPending`，由**第一帧回推**消费；
  该帧走 `attachToActual`（command 与 actual 一起写、**逐位相同**），此后回推照旧只写 `actualJoints`。
- 握手**不下发任何命令**：`suppressCommandSend` 精确覆盖那一次 `set`。接管不是"意图"，
  绝不能因此驱动机械臂。
- 握手**让位于用户意图**：若第一帧回推之前用户已经动过手（`commandAuthoredSinceConnect`），
  直接放弃握手 —— 否则会把刚下达的命令悄悄擦掉。
  **这条护栏是被 mock 闭环的 4 项失败逼出来的**：Mock 的 tick 比 connect 慢一拍，
  `setJoint` 先到、回推后到，于是握手把用户命令覆盖成了"半路位置"。
- **重连**保持 D32 的"补发当前命令"不变：那时用户已经有意图，不该被机器现状覆盖。
- 首次可交付的一次性工具：`tools/park_sim_pose.mjs`（造前提）+ `tools/first_load_probe.mjs`（逐帧取证）。

**为什么不是别的方案**

- **首次连接就补发命令** ⇒ 真机一跳即到 HOME（危险），且违反 Phase 7 时序契约（D32 已用测试钉住）。
- **只把幽灵藏起来**（默认关 / 首次不画）⇒ 面板仍在说假话、跳变危险仍在，还额外**毁掉一个真信号**。
- **每次回推都写命令** ⇒ 回到 `命令 → 状态 → 命令` 回环（spec §十九）。

**后果**
- ✅ 占位复现（后端停 `{0,-5,110,0}`）冷启动：主臂与幽灵 TCP **逐位相同** = `[108.203, 0, 112.334]`，
  滑杆如实显示 `-5.0° / 110.0° / 0.0°`，日志多一条
  `接管：以机器现状为命令起点 —— 与页面初始位姿最大相差 17.619°（gripper）`
- ✅ 新增 4 项接管握手验收（对齐 / 不下发 / 只发生一次 / 让位于用户意图）
  + 2 项 `attachToActual` 单测（含限位钳位）；vitest `345 → 351`
- ✅ `ws-transport-loop` 的"回推只写 actual 且引用不变"拆成**两段**（握手一次性 + 稳态引用不变）——
  这条钉子比以前**更强**：它现在同时钉住"握手只发生一次"
- ⚠️ 保留一条纪律：**读渲染必须用渲染后的 `matrixWorld`**。本轮 frame 0 读到幽灵 TCP = 原点，
  是"`useFrame` 先于 `gl.render`、矩阵尚未更新"的**仪器伪影**（主臂那帧正确是因为
  `RobotArm` 每 0.25s 显式 `updateMatrixWorld`）。差点被它带偏成"幽灵没被应用变换"。

---

## D73 · Sim2Sim 回归的三条纪律：容差写在数据里 · Python 不解释语义 · 锚点也要对

**背景**：D71/D72 建起了黄金数据与抽象层，但如果回归判据本身不独立，
"抽象前后一致"就只是**自证**（实现改了、判据跟着改，照样全绿）。

**决策**：

1. **容差从基线文件里读，不在测试文件里另立一套。**
   `fk_cases.json → tolerances.{frontend_vs_recorded_mm, ref_vs_mujoco_mm, threejs_vs_frontend_matrix}` /
   `ik_cases.json → tolerances.closed_loop_mm`。测试用 `tolerance(doc, key, default)` 读取，
   缺键时才用保守缺省 —— 方向**只能是收紧**。
   ⇒ 容差是冻结契约的一部分；散落在测试里迟早与数据漂移，而漂移的方向通常是**变松**。

2. **Python 侧只递 JSON，不解释任何运动学语义。**
   IK 由**前端真实的 `ik.ts`** 解（经 `tests/sim/ikbridge.py` → `kinematics-bridge.mjs`，
   Vite SSR 加载同一份源码）。一旦验收程序自己算几何，"独立判据"就没了 ——
   它证明的只是"我又写了一遍、而且自洽"。

3. **不只对末端，还要对每个关节锚点。**
   MuJoCo 侧新增 `harness.mujoco_joint_origin_mm()`，覆盖 `robot.yaml` 的**全部关节**：
   - 被动腕 `tool` 在 MJCF 里**没有 `<joint>` 元素** ⇒ 走"子连杆 body 的 `xpos`"这条**等价路径**
     （生成器把 body 放在 `parent.length + origin.position` 处，两种取法得到同一个量）
   - 叶关节 `gripper` 不参与末端定位，但仍是坐标系列的一部分

   理由：TCP 偏差只是"结果"，锚点逐级对比才能指出**错在哪一节**；
   而且这两类关节最容易在"只有 4 个关节"的心智模型里被整个漏掉。

4. **扫描用一次批量调用。** 桥是"一次性 CLI"（每次调用起一个 node 进程），
   `sweep1000` 逐点调用会起 1000 个进程。批量接口是唯一可行的形态。

5. **探针泄漏判据要写清扫描范围。** 既有契约是 `dist/assets/*.js` **0 命中**；
   `.js.map` 内联了源码文本（含 `import.meta.env.DEV` 守卫那一行），**必然含该字样**。
   不写范围，下一个人会对着 `grep -r dist/` 的 1 命中重新排查一遍。

**后果**

- ✅ 前端 13 项 + MuJoCo 侧 9 项回归，全部对着**同一批** JSON
- ✅ 实测（全部远优于容差）：`Joint→FK` 7.2e-13 mm · `Joint→Three.js` 7.1e-13 mm ·
  `XYZ→IK→FK` 4.0e-13 mm · `Joint→MuJoCo` 5.0e-13 mm · 关节锚点 5.3e-13 mm ·
  `XYZ→IK→MuJoCo` 4.0e-13 mm · `sweep1000` 1.2e-13 mm（失败 0，全 `elbow-up`）
- ⚠️ 关节锚点辅助函数目前**有两份**（`test_fk.py` 局部版 + `harness.py` 共享版）：
  为遵守"不得修改原有测试"，本阶段只新增未合并，已登记为 followups F3

---

## D72 · 最小抽象 = **包一层**，不是**重做一层**（`RobotDefinition` / `KinematicsEngine` / `IKResult`）

**背景**：spec 要求"最小程度的架构抽象"，并明确禁止若干诱人的做法。
动手前的审查（`docs/architecture/mearm-v1-baseline-analysis.md` §0）先给出一条结论：

> **MeArm-3D 的现有实现已经是"一个 Robot Model + 一套运动学"的干净结构。**

所以缺的不是架构。真要重做一层，只会把"抽象前后行为一致"变成一件需要大量解释的事。

**决策**：只新增四个文件，全部**纯委托 / 纯类型**，一个算法行都不写。

```text
definition/RobotDefinition.ts        RobotModel 的**分节视图** + defineRobot() + isModel()
kinematics/IKResult.ts               统一结果形状 + 适配器 fromMeArmIkResult()
kinematics/KinematicsEngine.ts       能力声明 KinematicsCapability + 调用面接口
kinematics/mearm/MeArmKinematics.ts  实现：forward/inverse 全部 forward 到 fk.ts / ik.ts
```

四条**刻意的**约束（每一条都是被"回归要有意义"逼出来的）：

| 约束 | 为什么 |
|---|---|
| `links`/`joints`/`actuators`/`tcp`/`homePose`/`robotModel` 全是**原对象引用** | 一旦某天有人做深拷贝，"抽象前后一致"就不再是同一对象的两次读取。由 `expect(def.robotModel).toBe(model)` 钉住 |
| **绝不复制 `effectiveJointAngle()`** | 渲染层 `buildRobotObject3D.ts` 与 `fk.ts` **共用**该函数（前者直接 import 后者）—— 这是 Phase 3 能到 8.7e-14 mm 的根本原因。抽象层只准 re-export / 委托（风险 R1） |
| `orientationError` 恒为 **`null`**（不是 `0`） | 本机没有姿态自由度。填 `0` 会**同时骗过调用方和测试**（看起来"姿态误差为零"）。失败时 `positionError` 同样是 `null` |
| 未知 `prefer` **显式抛错** | `ik.ts` 对无法识别的 `prefer` 会**静默退化**为 `elbow-up`。透传会让"抽象层传错参数"变成一条安静的、结果仍然"正确"的路径 |

**不做的事（spec 明令）**：不改 `ik.ts` 成 `GenericIK`；不给求解器加 5/6DOF 位姿参数；
不把 `solveIk` 的返回改成 spec 示例字段名（会破坏既有 2000 组闭环断言与 e2e 探针，
正解是**适配**不是替换）；不为贴合建议目录树而大规模重命名。

**判据**：抽象层调用与直接调用**逐位一致**（`Object.is` 全 true，237 例差异 0），
不是"接近"。见 `frontend/tests/unit/kinematicsEngine.test.ts`（14 项）与
`frontend/tests/sim2sim/`、`tests/sim2sim/`。

---

## D71 · MeArm-V1 黄金数据：把**行为**落盘，判据从"两套实现互相比对"改成"与冻结时一致"

**背景**：本项目既有测试大多是「**当场算两遍、互相比对**」（FK 参考实现 vs MuJoCo、
前端 FK vs Three.js）。那能证明"两套实现对得上"，**证明不了"今天的行为和上周一样"** ——
重构之后两套实现可能一起变，判据照样全绿。

**决策**：为 MeArm-V1 落一份**行为快照**（characterization baseline），
作为第一个正式 Robot Model 的伴生数据。

```text
tests/baseline/mearm-v1/
  joint_cases.json      116 例   公共输入：零位/HOME · 各关节 min/mid/max · 全最小/全最大 · 100 组随机
  fk_cases.json         116 例   tcpFrontend（前端 fk.ts）/ tcpRef + framesRef（fkref.py）/ tcpMujoco
  ik_cases.json         121 例   可达 115 + 越界 3 + 方位角越限 3；含分支/残差/错误码
  workspace_cases.json  121 例   可达性 / 错误码 / 2R 几何量 / 矢状面距离
```

采集器 `tools/gen_mearm_v1_baseline.py` 的四条原则：

1. **期望值一律实跑采集，绝不手算。** 前端侧经 `kinematics-bridge.mjs`（Vite SSR 加载
   **同一份** `fk.ts` / `ik.ts`）；MuJoCo 侧走 `reset()`（纯 `mj_forward`，不引入重力/接触噪声）。
2. **固定 seed `20260914`**，全流程可复现；唯一允许变动的是 `generated_at`。
   `--check` 逐位复现已提交文件（重构后自证用）。
3. **随机只作补充**：主数据集是显式枚举的定点用例（spec §6）。
4. **落盘 12 位小数**（`-0.0 → 0.0`）：远严于最小容差 `1e-9`，同时 JSON 稳定、diff 友好。

模型标识（`robot.model: MeArm-V1` / `version: 1.0.0`）是**只读元数据**，
不在 `freeze_baseline.py` 的语义核心白名单内 ⇒ 语义核心哈希不变，
仅 L1 整文件记录漂移（属"放行"情形，`--update` 刷新即可）。

**后果**

- ✅ 「抽象前后行为一致」从一句口号变成一条可执行断言
- ✅ 随机/环境相关的不确定性被隔离：黄金数据负责"与昨天一致"，扫描负责"入参空间更宽"
- ⚠️ 有意改变行为时**必须重新生成并说明原因**（`--check` 会逐字段列出差异）
- ⚠️ 用例边界刻意包含**超限位位形**（`elbow = 0°`）：它在真机限位外，但 FK 是纯几何求值，
  这条同时充当"竖直段 = 60+80+80、水平段 = 40"的**独立几何常量核对**

---

## D70 · 爪被连杆锁成**水平** ⇒ `tool` 改**被动关节**；MuJoCo 必须用 **tendon + equality** 锁绝对角

用户报障（原话）：

> 目前对于爪部分模拟处理还有问题，这个部分角度会随着前后移动同时有变化

即虚拟臂的爪会随前后移动而倾斜，而真机不是。这是**模型错了**，不是渲染错了。

### 一、先立判据：旧模型被实测否决

受控实验（定机位扫 S8、量爪指轴线并画回原图复核，全文见 `hardware-measurement.md` §5.3）：

| 量 | 变化量 |
|---|---|
| 小臂**绝对倾角** | 29.24° |
| 爪指**画面倾角** | **7.31°** |

- 旧模型 `tool = fixed`（爪与小臂刚性固连）预期 1:1 跟随 → **≈29.24°** ⇒ ❌ **否决**
- "爪完全不随动" 预期 ≈0° ⇒ 也否决（确实动了 7.31°）
- **反向折角补偿** ⇒ 爪的**绝对倾角近似恒定（≈水平）** ✅

> 这正是 D47 的同一套方法：**先证测量可复现，再谈模型对不对**。
> 本轮没有再动 `coupling.gain`（D47 已判定 −1 不改）—— 改的是**腕的性质**，不是参数。

### 二、配置层：新增 `passive` 关节类型

```yaml
- id: tool
  type: passive                            # 会转，但没有独立输入
  coupling: { joint: elbow, gain: -1 }     # 局部角 = 锁定值 − θ_elbow
  limit: { min: 90, max: 90 }              # 绝对倾角恒 90°（水平）
```

三条硬约束：

1. **`min === max` 强制成立** —— `passive` 关节的值恒取 `limits.min`（没有输入可扫），
   写成区间只会制造"看起来能动"的假象。锁定值就是"水平"这个物理事实的编码。
2. **不进 `JointState` / UI 滑杆 / JR 协议 / `homePose` / `actuators`** —— 它没有输入，
   出现在这些地方只会让"命令"与"事实"混为一谈。
3. **`coupling` 仍要写** —— 前端 FK 与 MuJoCo 生成器都靠它推局部角，
   写成"不写 coupling"会让两端各自猜一个值。

### 三、⚠️ 这一轮真正的坑：`qpos` 坐标数 ≠ 自由度

`passive` 关节**在串联网里必须是 hinge**（要有 `qpos`、要参与 FK），但它**不是自由度**。
只要求"是不是一条会转的轴"就会把它算成自由度，于是：

| 口径 | 判据 | 结果 |
|---|---|---|
| 前端 | `type === 'revolute' && max > min` | 4 ✅ |
| Go | `Type == "revolute"` | 4 ✅ |
| Python | `is_dof = (type == "revolute")`；`has_qpos = (type != "fixed")` | 4 / 5 ✅ |

`nq = 5`、`njnt = 5`、`nu = 4`、**自由度 = 4** —— 这四个数同时存在且都对，
把它们混成一个"关节数"就会协商出**五元组的 JR**，而固件与 `docs/serial-v1.md`
只认四元组（`base/shoulder/elbow/gripper`）。

> 实测踩法：`SimState.joint_velocities` / `joint_torques` 一度按 `joint_ids`（5 个）顺序取，
> 于是 `as_dict()` 的 zip 把 `gripper` 读成了 `tool` 的值 —— **不报错、只是数错**。
> 修法是引入 `dof_vector()`（按 `robot.joint_order()` 的 4 个自由度重排），
> 并让 `ctrl_vector()` / `constraint_torque()` 都走它。

### 四、MuJoCo：锁**绝对角**只能靠 tendon，`equality/joint` 是错的

串联网里某关节的**绝对转角 = 从根到它、且转轴与之共线的全部关节局部角之和**。
MuJoCo 的 `qpos` 存的恰恰是**局部角**，所以"把绝对角钉死"的约束是「这串 qpos 的**加权和** = 锁定角」。

| 写法 | 效果 |
|---|---|
| ❌ `<equality><joint joint1="tool" joint2="elbow"/>` | `q_elbow` 是**局部角**，**少一项 shoulder** ⇒ 残差恰为 `homePose.shoulder`（0.8499°）。看着"差不多对"，其实整个姿态偏了 |
| ✅ `<tendon><fixed name="tool_lock_tendon">` 把 `q_shoulder + q_elbow + q_tool` 线性组合 + `<equality><tendon polycoef="1.57079632679 0 0 0 0">` | 组合和**就是**绝对角，钉在 `deg2rad(90)` |

生成器里不写死这三项，而是用 `_axes_parallel()` + `absolute_lock_terms()` 从配置**推**：
遍历从根到该关节的路径，凡转轴与之共线者系数记 1，其余记 0。
这样将来换机构（多一个共线关节 / 换锁定角）不需要改 MJCF 逻辑。

**验证判据**（已进测试）：`q_shoulder + q_elbow + q_tool` 与 `deg2rad(90)` 的残差应为 0，
且 HOME 位的 FK 与 `fkref.py` 的差 < 1e-6mm（实测 1.421e-14 mm）。

### 五、IK：换掉的不是"一个数"，而是 **2R 的作用对象**

`elbow → TCP` 不再是定长直线（随 θe 在 **109~120mm** 间变），所以：

| | 旧（`fixed`） | 新（被动腕） |
|---|---|---|
| `L2` | 120（= 小臂 80 + `tcp.offset.z` 40） | **80**（= `forearm_link.length`） |
| 「腕枢轴 → TCP」 | 含在 L2 里，**方向随 θe 转** | **常量 `[40, 0]`**（纯水平），解算前从目标点减掉 |
| `reach` | `[40, 200]` | **`[0, 160]`** —— `L1 == L2` ⇒ **内半径退化为 0** |

两个直接后果，都反直觉：

1. **"离枢轴太近"不再构成越界**。`L1 == L2` 时肩肘可完全对折。
   测试里那些"上方过近"的反例全部作废，真实约束改由**关节限位 + 腕的 hinge 区间**给出。
2. **"偏移是常量"必须被数值证明，不能靠推导**（D37/D47 的同一纪律）。
   `ikGeometry()` 在 **≥3 个合法姿态**上采样并要求逐位一致（`PROBE_TOL_MM = 1e-6` mm）。
   谁把腕改回刚性固连，这里**当场抛 `IkModelError`** —— 而不是让所有解静默偏 40mm
   （**残差自查同样是偏的**，这种错不会自己暴露）。

### 六、连带后果：台面高度必须**跟着重推**（不能沿用旧值）

爪锁平后机构包络的**最低点由 15.8mm 抬到 32.71mm** —— 原来 30mm 的台面**够不着了**。
6 条碰撞测试同时红，根因是"机构能力真的变了"，不是测试写错。

重推窗口（两台面探针 + 编译后自检 `|got_top − target| < 1e-6`）：

| 台面顶面高度 | 结果 |
|---|---|
| < 32mm | 最深位形碰不到 ⇒ 接触通道测不出来 |
| **32 ~ 50mm** | **可用窗口** |
| ≥ 56mm | 会把 `JR(40,130)` 这类**中段常规位形**也挡住 ⇒ 污染非碰撞用例 |

取 **46mm**（`pos.z 0.031 + size.z 0.015`）。这一改动了 `physics_core_sha256`，
按 D55 走 `freeze_baseline.py --update` 重新冻结 —— 让"我确实动了物理"留在 diff 里。

> 探针本身也踩了三个坑（把 body 局部坐标当世界坐标、重复减 `BODY_Z`、脚本硬编码旧台面高度），
> 全靠"编译后自检"抓出来；顺带证明了 `MjSpec.to_xml()` round-trip 是保真的
> （tendon / equality / ngeom / nq / nv 全一致），排除了一条错误怀疑。

### 七、验收

六套全绿（`tsc -b` · `vitest 318/318` · `vite build` + 探针零泄漏 · `e2e 88/88` ·
`go test ./...` · `pytest tests/sim 147/147`），外加 IK↔FK 往返 2000 组随机位姿最大误差
**1.180e-13 mm**。e2e 里两条写死旧闭式（`111.96 / 93.84`）的断言同步改为
`115.03 / 109.22` —— 旧式是把爪当直线延伸，正是本轮要修的那个模型。

## D69 · 「空洞」= 小臂侧板到腕点缺 12.5mm 料：用**沿臂推导覆盖区间**定位 + 用**无贴图补料**填充（而不是拉伸主板）

用户诉求（原话）：

> 目前机械臂上有一段空洞(连接前爪部分)，直接延伸填充，这部分是电机贴纸反光，导致之前未捕捉到

纯**外观层**改动（D55：只动 `links[].geometry` / `details`，`length` / `joints` / `actuators` 未触碰）。

### 一、定位：不从截图猜，从配置推「沿臂方向的覆盖区间」

盯截图只能看到"那里有个缝"，说不出它有多宽、是哪个件短了。改从 `forearm_link` 的局部坐标系
（z 从肘指向腕，`length: 80` ⇒ **腕点 = z 80**）把每个件的覆盖区间列出来：

| 件 | 局部 z 覆盖 |
|---|---|
| 侧板 ×2（`plate [18,5,68]` @ z=34） | 0 .. 68 |
| 肘轴端盖 ×2（`cylinder r5 h4` @ z=0） | −2 .. 2 |
| 腕部横撑（`plate [18,22,5]` @ z=64） | 61.5 .. 66.5 |
| **空缺** | **68 .. 80** |

腕板在 **tool 系** z=0.5 起（`tool_link.geometry.position [0,0,6]`，`size [22,26,11]`）
⇒ 全局 80.5。所以 **68 → 80.5 共 12.5mm 完全没有几何** —— 侧视能直接看穿到背景。
位置正是用户说的"连接前爪部分"。

放大截图印证：小臂板下端与腕板之间是一条**连续背景色带**，不是反光面。

### 二、修法：加一段**无贴图补料**，而**不延长主板**

先按最直觉的做法试过"把主板 68 → 80"，`plateTexture.test.ts` 当场报：

```
forearm_link: 纹理长宽比 3.778 vs 板大面 4.444: expected 0.15 to be less than 0.05
```

**这条契约是对的**（它是"把图配错板"的防线），不该为了让它过而放宽容差。
而那段**本来就没有照片数据** —— 用户已说明它被夹爪舵机 S6 的贴纸反光糊住、反解时未捕捉到
⇒ 硬套贴图本身就是错的。

故：**主板保持 `[18,5,68]`（贴图契约逐字不动）**，另加两片补料 `size: [18,5,12] @ z=74`
（覆盖 68 .. 80，与腕点齐平），**不带 texture**。

### 三、两条只在「插入位置」上才暴露的耦合（本轮真踩到）

`tools/make_texture.py::load_plate_sizes()` 遍历的是 `geometry`（plate）**加上 `details` 里的
所有 plate**，命名规则用 **`enumerate(details)` 的下标**做别名键（`forearm_brace` 就是
`forearm_link` 的 index 3 别名）。于是：

| 做法 | 后果 |
|---|---|
| 补料用 `plate` 插在 details **中间** | 清单多出 `forearm_link_detail1/2`，且 `forearm_brace` → `forearm_link_detail5` ⇒ `test_texture_pipeline` 报「采集指南提到脚本不认识的文件名：`['forearm_brace']`」 |
| 补料用 `plate` 追加到**末尾** | `detail1/2` 仍在清单里 ⇒ 反向断言报「脚本能处理但指南里没提到的板件」 |
| **`box` + 追加到末尾** | 清单**逐字恢复原样** ✓ |

⇒ 定案 `type: box` + 放在 `details` **末尾**。两条理由都是硬的：
① 纹理管线只枚举 plate，`box` 不在范围内；② 它本来就不是外露装饰板，是**补料** ——
没有照片数据，也就没有纹理长宽比契约要守。

⚠️ 教训：**"给 details 加个件"不是无副作用的操作**。下标即别名键，插入位置会静默改变
"待拍板"的命名集合 —— 这个耦合只在跨语言（Py 读 YAML ↔ 文档 ↔ 测试）的断言里才暴露。

### 四、验收

| 判据 | 结果 |
|---|---|
| `tsc -b --force` | **0 error** |
| `vitest run` | **313/313** |
| `vite build` + 探针字面量扫描 | 通过 / **`__armPilot` 0 命中** |
| `pytest tests/sim` | **147 passed** |
| e2e `ui-smoke.mjs` | **88/88 PASS, 0 FAIL** |
| `load_plate_sizes()` | 8 项，与原清单逐字一致（无 `_detail` 残留） |

**像素 diff（受控：同机位、同关节状态、同显示开关，唯一变量 = 这段补料）**：
差异 23657 px，bbox `x[0,180] y[94,426]` —— **全部落在缺口处，画面其余部分 0 变化**，
即"改的就是那一处、没有波及别处"。并排图 `cmp_gap.png` 可见改前那条三角缺口被填满。

### 五、附带：收掉 D66 遗留的两个 pytest 回归

上一轮的验收四件套（tsc/vitest/build/e2e）**不含 pytest**，于是 D66 引入的两处漂移漏了两天，
本轮开跑才暴露：

1. `simulation/mujoco/gen_model.py` 不认识 `geometry.type: 'jaw'` ⇒ MJCF 生成时该件静默产出
   空 geom，`test_generated_mjcf_is_in_sync_with_config` 失败。修：与前端 `case 'none': case 'jaw'`
   对齐，`jaw` 与 `none` 同样不产出碰撞体（爪的形状由渲染层轮廓挤出，不参与碰撞）。
2. `docs/texture-capture-guide.md` 里"别按 `jaw_link.jpg` 命名"这句**字面**被
   `test_guide_filenames_are_known_to_the_script` 的 `` `([a-z_]*)\.jpg` `` 正则抓到，
   而脚本已不认 `jaw_link`（D66 起改为 `type: jaw`）。修：改写成
   "别用「`jaw_link` + `.jpg`」命名" —— **保住信息量**，只去掉会被误读成文件名的字面。

⇒ 教训：**四件套漏 pytest**。MJCF / 纹理管线 / 文档↔脚本一致性这三类契约全在 pytest 里，
前端那套覆盖不到。跨端改动（config 或 geometry 类型）必须**五个都跑**。

## D68 · 视口改**产品渲染式灰底** + 板件统一**近黑** + 世界轴**默认关**（那根"不动的蓝线"）

用户诉求（原话）：

> 将虚拟空间背景切换为灰色，模型中蓝色部分也替换成黑色，另外移动后，中轴的蓝色不移动的线优化掉

三条互相独立，各自一处落点。全部属**外观层**（D55：`links[].geometry` / `details` 外观可自由迭代，
`length` / `joints` / `actuators` 运动学层未触碰）。

### 落点与证据

| 诉求 | 落点 | 证据 |
|---|---|---|
| 背景 → 灰 | `RobotScene.tsx`：`<color attach="background">` `#14171c` → **`#8b8e93`** | 截图采样 `(139,142,147)` = `#8b8e93` ✓ |
| 蓝件 → 黑 | `config/robot.yaml` 6 个色值共 **11 处** → **`#232830`** | 见二（**先验证 `color` 语义再动手**） |
| "不动的蓝线" | `robotStore.ts`：`showWorldAxes: true` → **`false`** | 见三（受控实验 + 像素 diff） |

### 一、背景改灰 ⇒ **Z=0 参考圆盘必须跟着改**

背景从 `#14171c`（近黑）换到 `#8b8e93`（中灰）后，原来那个近黑的底座参考圆盘 `#1d222a`
会在灰底上变成一个**突兀的黑洞** —— 那是"背景换了、地面没换"。故一并改为 `#7a7e84`
（只比背景**暗一档**，仍能读出 Z=0 平面在哪）。

**网格没动**，因为灰度关系自动反相且仍可读：采样得网格线 `(111,116,123)` **暗于**背景
`(139,142,147)` —— 深底上是亮线、灰底上是暗线，两种都是"线比底亮/暗"，辨识度都在。

> `#8b8e93` 是**审美选择，不是量测结论**：选它的理由是中性（不引入色偏）、与 UI 现有冷灰
> 调性一致、明度 ≈ 55%（近黑机件在它上面轮廓清楚，也不至于刺眼）。
> 若要更亮/更暗，改这一个值即可，不牵动任何其它逻辑。

### 二、蓝件 → 近黑：**先验证「贴图件的 `color` 只染色窄边」再动手**

`robot.yaml` 里偏蓝的板件色有 6 个，共 11 处：

| 原值 | 处数 | 位置 |
|---|---|---|
| `#4d8fe8` | 5 | 立柱侧板 ×2、大臂 ×2（**含贴图件**）、腕部横撑 |
| `#6aa0ff` | 2 | 小臂 ×2（**含贴图件**） |
| `#3d6fc0` | 1 | 大臂端部横撑 |
| `#2e3849` | 1 | 底盘主板 |
| `#44567a` | 1 | 立柱主板 |
| `#39455a` | 1 | 腕座主板 |

**关键前置确认（否则会误伤照片纹理）**：`plateTexture` 存在时 `createPlateMaterial` 返回的是
**6 元材质数组** —— 两个大面用 `createTextureMaterial`，其基色**硬编码 `0xffffff`**
（`buildRobotObject3D.ts`："照片本身已含颜色；材质基色保持白，避免二次染色"），
其余四面才用 `createMaterial(geometry.color, …)`。

⇒ **`geometry.color` 在这块板上只染色那 4 个窄边**，改它**不会**把照片乘黑。
这一条是本轮能放心批量替换 `color` 的前提；若反过来（大面也用 `color` 做 tint），
把这 11 处改黑就等于**把两块照片纹理一起乘黑**，那是完全不同的结果。

统一取 **`#232830`**（近黑中性，明度 ≈ 0.16）：与贴图大面（黑件照片）协调，
比纯黑亮一档以保留板面能被光照塑形；且保持 UI 的"无装饰色"约定。
未改 `#2b3446`（端盖）/ `#2f3644`（舵机壳）/ `#161a21`（螺栓）—— 它们是**非板的深色机加工件**
（实拍本就近黑），不属于用户说的"蓝色部分"。

### 三、"移动后中轴那根蓝色的、不移动的线" = **世界轴（World Axis）**

**现象解释**：`WorldAxes` 是挂在**世界原点**的 `axesHelper`（`args=[130]`），
三根线为 R/G/B = X/Y/Z。其中 **Z 蓝轴竖直向上**，恰与 **J1 旋转轴（底座转轴）重合**
⇒ 看上去就是"机械臂的**中轴**上有一根蓝线"；而它固定在世界系，
J1 一转（立柱让开）它就露出来且**不跟着动** —— 用户的描述逐点吻合。

**受控实验**（同机位、同关节状态，**只翻转 "World Axis" 一个复选框**）：

| 组 | 自变量 | 结果 |
|---|---|---|
| A | World Axis 勾选（原默认） | 画面里可见 2 条从原点射向画面外的长斜线（X 红 / Y 绿） |
| B | World Axis 取消（`TOGGLE_OFF="World Axis"`） | **两条长线全部消失**，其余逐像素不变 |

**像素 diff 量化**（`.workbuddy/captures/ui_before.png` vs `ui_noworldaxes.png`，`>12` 阈值）：

- 视口内差异 **582 px**，集中在 `y[560,800)`（两条斜线）与 `y[80,120)`；
- `y[80,120)` 那 157 px 经可视化确认是 **UI 里 "World Axis" 复选框自身的勾选标记**，
  不是 3D 里的东西（diff 可视化图 `diff_axes.png` 中标红处正是复选框）；
- ⚠️ **立柱上那道亮蓝竖条在 diff 里没有任何标记** —— 它当时还是 `#4d8fe8` 的**立柱侧板**
  （本轮已改黑），**不是**世界轴。这一条把"看着像轴的蓝色竖条"排除掉了，避免改错对象。

**修法**：`showWorldAxes` 默认改 **`false`**（**保留开关本身**，调试坐标系时仍可打开）。
不删组件：删掉等于把一个调试能力永久拿掉，而用户要的是"别再默认挡在画面里"。

### 未改的部分（明确记账）

改完后视口里剩下的蓝色只有两处，均**不属于机械臂模型**，故本轮不动：

- **DragHandle 把手球** `#4c8dff`（半透明）—— 它是"目标点"交互件，且颜色带语义
  （可达 / 超程 = 蓝 / 红，见 `DragHandle.tsx`）；
- **TCP 标记的 Z 轴短柱** `#6699ff`（26mm）—— 调试标记，随 TCP 移动（会动）。

两者要一起改黑随时可做，但它们不是"模型中"的件。

### 验收

`tsc -b --force` **0 error** · `vitest run` **313/313** · `vite build` 通过且产物
`__armPilot` **0 命中** · e2e `ui-smoke.mjs` **88/88 PASS, 0 FAIL**（后端由脚本自拉，
`device=sim`；8090/5273 跑前均为空闲，无残留进程污染）。

## D67 · 探针的两类**静默失真**：读错表（选择器未限定作用域）+ 滞后读数拆成多次往返；以及 `start.bat` 的**残留进程对**

### 问题

本轮跑 e2e 出现 **6 FAIL**，其中 **5** 条与爪子改动**毫无关系**，是环境与探针自身的问题：

| FAIL 读数 | 真因 |
|---|---|
| 状态表 Command 列跟随滑杆 — `0 / 0` | **探针读错了表**（见一） |
| 滞后瞬间幽灵与主臂分离 — `0.2 mm` | 滞后读数**拆成多次 CDP 往返**，慢环境里读到已收敛值（见二） |
| 滞后期间健康结论不是"已到位" — 已到位（0.18°） | 同上 |
| 松手后 Actual 收敛 / TCP 误差归零 | 页面被残留后端接管，Actual 由 WS 回推驱动（见三） |
| Phase 10.6 点 Real Robot 未被拒绝 | 同上 + **mode 跨批次残留**（见三、四） |

### 一、`readCommandCell` 读错表：全局选择器命中了**两张** `table.grid`

探针原文：

```js
document.querySelectorAll('.sidebar table.grid tbody tr')[rowIndex]
```

`.sidebar` 里 `table.grid` **不止一张**（`ConnectionControl` 的"指标 / 值"表 + `StatusPanel` 的关节表
+ `RobotModelPanel` 两张），而 `ConnectionControl` 在 `App.tsx` 里**排在 `StatusPanel` 之前**。
⇒ 一旦"指标 / 值"表被渲染，`rows[1]` 就从 J2 关节行**静默漂移**到"丢帧 / 拒绝"行。

**受控实验**（`.workbuddy/captures/probe_table_scope.mjs`：同一页面，只翻转"是否已发过命令"）：

| 组 | 自变量 | `table.grid` 数 | 旧选择器 `rows[1]` | 新选择器 `rows[1]` |
|---|---|---|---|---|
| A | 未连接 | 3 | `0.8°`（J2 行） | `0.8°` |
| B | 已连接、**未**发过命令 | 3 | `0.8°` | `0.8°` |
| C | 已连接、**已**发过命令 | **4** | **`0 / 0`** ← 丢帧 / 拒绝 | **`39.9°`** ← J2 |

B 组是关键的对照：`transportStats` 只在**帧流过之后**才非 null ⇒ "指标 / 值"表此时才渲染。
所以自变量是"发过命令"，不是"连上了"。**修法**：与 `readJointGap` 一致，先 `cardByTitle('状态 · Status')`
再取 `table.grid tbody tr`。

⚠️ 该写法必须放在**函数**里：写成模块级模板字面量会在 `import` 时求值，
而那时 `cardByTitle` 还在 TDZ ⇒ 直接 `ReferenceError: Cannot access 'cardByTitle' before initialization`。

### 二、滞后相位的四条断言必须**同一次求值**取全

`READ_LAG_WITH_GHOST` 的注释早就写了这条纪律，但当时**只合并了两条**：
`READ_ERR_VALUES` / `READ_LINK_HEALTH` / 后续收敛轮询仍是**分开的往返**。
Mock 的 34° 跳合约 150ms 收敛完 —— 负载一高，后取的读数就落到"已收敛"态 ⇒ 间歇性假 FAIL。

**修法**：合并成单个 `readLagSnapshot()`，一次返回
`{command, actual, gap, errDeg, errVals, ghostGap, health}`，让四条断言判**同一时刻**。

| | (b) 滞后量 | (b1) 幽灵分离 | 自洽？ |
|---|---|---|---|
| 修前 | 39.1° | **0.2 mm** | ❌ 自相矛盾（39° 滞后不可能只分离 0.2mm） |
| 修后 | 39.1° | **53.5 mm**（误差 −39.06°） | ✅ 三读数互相印证 |

### 三、`start.bat` 会同时拉起**两个**必须一起清掉的进程

残留的不是"一个 dev server"，而是**一对**：

| 进程 | 端口 | 危害 |
|---|---|---|
| `armpilot-backend.exe -c config.serial.yaml` | 8090 | 末端 = **serial** ⇒ e2e 复用后"前提不成立"（D65 已为 Phase 8 加 skip） |
| `vite`（**带 `VITE_AUTO_CONNECT=ws`**） | 5273 | 页面**加载即自动连上真机链路** ⇒ 比 D65 更重：Actual 由 WS 回推驱动，**Mock 闭环类断言整体失真** |

⇒ 清场必须**两个端口一起清**，只清一个还会踩。

⚠️ 另一条工具事实：`(cmd &)` 起的后台进程**只活到本次工具调用结束** ⇒
"先起服务、下一条命令再跑 e2e"必然 `ERR_CONNECTION_REFUSED`；起服务与跑 e2e 必须在**同一次调用**里。
（同理 `/tmp/xxx.log` 重定向在沙箱内会被拦，日志要落到工作区内。）

### 四、D65 漏掉的第三处：Phase 10.6

D65 末尾自己写了"后续新增依赖特定链路末端的断言时应默认走 `skip()`"，
但当时**只改了 Phase 8 的两条**，Phase 10.6 的同前提断言仍在用 `check()`。修法两处：

1. `device === 'serial'` 时这两条报 `skip()`（措辞与 Phase 8 一致）；
2. **入口显式归位 Simulation** —— 污染环境里出现过「提示条说已拒绝、按钮却仍高亮 Real」的**自相矛盾**：
   真因是 Phase 8 走 serial 分支时其末尾的"归位到 Real Robot"**不执行**，mode 被留在 real，
   Phase 10.6 读到的是**上一段的结果**（§9.10 第 7 例的同一类错误）。

### 后果

- ✅ 读错表与滞后读数竞态**双双消除**；`skip()` 覆盖补齐到 Phase 10.6
- ✅ 干净环境 **88/88 PASS, 0 FAIL, 0 SKIP**；污染环境的 6 FAIL 全部归因为环境/探针，**非爪子回归**
- ⚠️ 教训：**"读到一个值"不等于"读到了想读的那个对象"**。位置型读取（`rows[i].children[j]`）
  必须在**同一个容器内**限定作用域，否则改任何一处无关 UI 都会让它静默指向别的表

---

## D66 · 夹爪按**实拍照片反解平面轮廓**：`type: jaw` 一条连续闭合折线 + 三条由几何自洽给出的派生关系

### 问题

爪（`jaw_link`）在渲染层是**两片倒角方块**（`plate` 占位 + 硬编码 `#d29922`），
与实物差得最远：照片里它是**根部整圈方齿齿轮盘 + 内侧缘一排锯齿 + 末端斜切**的机加工件。
用户给 5 张近景照片，要求"爪子也按照这些图片进行重构"。

### 一、能读出来的只有**轮廓**，所以就读轮廓

黑件在照片里近纯黑（与 D64 同一批照片：`≤3` 码值占 72%~77%），**没有可提取的贴图信息**；
但**剪影可用** —— 激光切割的亮切口恰好把轮廓勾出来。⇒ 本轮目标是**形状**，不是纹理。
（提亮路线一并作废：把数组归一化到 0–1 后再 `np.clip(a, 0, 255)` 会把整幅图压成黑，
`np.clip(np.power(a, gamma) * bright * 255.0, 0, 255)` 才对 —— 该 bug 一度让我误判"裁切位置错"。）

读出的形体特征（5 张照片一致）：

| 特征 | 实拍依据 |
|---|---|
| 根部**整圈方齿齿轮盘**，中心有装饰镂空 + 铰轴螺栓 | 齿轮直径 / 爪长 = 246/359 = **0.69**（模型 24/34 = 0.71） |
| **两片爪的齿轮互相啮合**（不是各自独立转） | 齿廓在两者之间连续咬合 |
| 爪指**内侧缘一排锯齿**（浅、密） | 约 9 个 |
| 外侧缘光滑，出盘后**先收窄一次**（"脖子"） | 收窄点约在 1/3 长度处 |
| 末端**斜切收尖** | 尖角内低外高 |

### 二、为什么必须画成**一条连续闭合折线**

`ExtrudeGeometry` **不做布尔并集**：把"齿轮盘"与"爪指"画成两个 Shape 再各自挤出，
重叠区会 z-fighting、分离处会露接缝。⇒ 齿轮齿廓 + 脖子 + 外侧缘 + 内侧锯齿 + 斜切尖
必须**首尾相接成一条闭合折线**（`buildJawOutline`）。

坐标映射只有一个：`Shape(x, y)` → 机构 `(Y, Z)`、挤出方向 → `X`，
即循环置换 `(x→y, y→z, z→x)`（`JAW_PLANE_MATRIX`，行列式 **+1** ⇒ 不翻手性、法线无需修正）。
⚠️ **镜像必须反转点序**：轮廓 x 取反后绕向变顺 ⇒ 挤出体法线朝里；`shape.holes` 也必须与外轮廓**反向**。

### 三、三条派生关系**由几何自洽给出**，不写进 yaml

yaml 只放**形状标量**（`length / width / gearRadius / gearTeeth / toothDepth / serrations / …`），
下面三条在 `jawRenderSpec()` 里推 —— 否则就是把"配平"这件事交给配置文件：

| # | 关系 | 为什么 |
|---|---|---|
| ① | 两齿轮中心距 = `2 × pitchRadius`（`pitchRadius = gearRadius − toothDepth/2`） | 标准啮合：一方齿顶恰好落到对方齿根 |
| ② | 爪指中线相对齿轮中心内偏 `δ = pitchRadius − width/2` | 让齿轮中心落在**爪指内侧缘**上 |
| ③ | 由 ① ② 得 `θ=0` 时两爪**内侧缘正好贴合** | 闭合位置由几何定，不靠调参 |

解析层加了**前置校验**：`pitchRadius < width/2` ⇒ 抛 `RobotConfigError`，并说明
"爪指中线会落到齿轮外侧且 θ=0 两爪无法贴合"（拒绝不是目的，说清为什么才是）。

数值：`pitchRadius = 10.5` · `gearRootRadius = 9` · `fingerInset = 6` · `hubOffsetY = 10.5` ·
齿弧内齿数 = **11**（`gearTeeth=16` 是整圈值，按 `2π − 开口角` 折算）。

材质改 `metalness 0.4 / roughness 0.4`（原 0.3/0.55）：黑件的形状可读性来自**镜面项**（D64），
镜面项不乘那个 ≈0 的暗反照率 ⇒ 激光切割的亮切口是照片里唯一勾出轮廓的东西。

### 四、验收

| 判据 | 结果 |
|---|---|
| `tsc -b --force` | **0 error** |
| `vitest run` | **313/313**（24 文件全绿） |
| `vite build` + `grep -o "__armPilot" dist/assets/*.js \| wc -l` | 构建通过；**0 命中**（探针未进生产包） |
| e2e `ui-smoke.mjs`（干净环境） | **88/88 PASS, 0 FAIL, 0 SKIP** |
| 开合判据 | 单测**读实际几何顶点**（爪尖 = Z 最大处的平均 Y），**不在测试里重算符号** |

⚠️ `serrations: 9` / `serrationDepth: 0.75` / `color: "#2b323c"` 三项是**观感目视调参**，
**未做受控量测** —— 不要与"从照片读轮廓"（有剪影证据）混为一谈。
⚠️ 板边 0.3mm 倒角会让爪尖实际顶点偏离理想点 **0.176mm**（几何事实，不是逻辑错）⇒
"θ=0 内侧缘贴合"用 `toBeLessThan(0.5)` 容差，另加**精确**断言 `closedLeft.y ≈ −closedRight.y`（镜像对称）。
⚠️ `jaw_link.length`（**运动学**量）恒为 0，`git diff` 未触碰运动学层（D55）。

**下一步**：爪的形状已由照片定型 ⇒ 它不再需要"补拍纹理"（薄片、面积极小，收益最低）。

---

## D64 · 近黑照片纹理**不可见**：根因是 8bit 量化不是曝光；解法是**逐材质 `envMap`**

### 问题

D63 把照片贴上去后，大臂/小臂板从亮蓝变成**纯黑**。ROI 实测 `L mean=0.08 / p50=0.07`，
`hp_std=0.086`（局部细节能量），`L>=20` 占比 `0%` ——
**不是"偏暗"，是信息被抹掉了。**

### 一、先定性：是量化，不是曝光

`assets/textures/mearm/tiles/` 实测（全图口径）：

| tile | 尺寸 | mean | p50 | p95 | ≤3 占比 | ≤8 占比 |
|---|---|---|---|---|---|---|
| `upper_arm_link` | 220×740 | 5.25 | 2.37 | 15.93 | **72.2%** | 87.7% |
| `forearm_link` | 180×680 | 4.10 | 1.44 | 15.09 | **77.0%** | 87.8% |

照片是**按白色桌面曝光**的 ⇒ 黑色亚克力落在 8bit 的 **1~3 码值**；经 three 的
sRGB→线性（`sRGB(2) → 线性 0.0006`）再乘 <1 的辐照度 ⇒ 渲染输出 `L≈0.08`。
**七成以上像素在量化台阶上，提亮只是把台阶一起放大。**

`tile_lift_preview.png` 四种曲线对照（受控实验）：

| 曲线 | 结果 |
|---|---|
| 线性 ×8 | 只放大 JPEG 块与量化台阶 |
| gamma 1/2.2 | 前臂被压成近平色 |
| p1..p99.5 拉伸 | 板面出现条带 |
| 推 `L≈60` 所需 **+6.2 EV（×74）** | 板面摊成 **41 / 60 / 74 三级平台** ⇒ 假色斑，**比纯黑更假** |

⇒ `exposureEv` 留在 **0**（旋钮保留在 yaml 里并注明为什么不拧）。

### 二、黑件能不能"被看见形状"，取决于它**反射了什么**

非金属 `F0 ≈ 0.04`，**镜面项不乘**那个 ≈0 的暗反照率 ⇒
形状信息走镜面通道、走 IBL，而不是走漫反射。这是"环境反射能救、等比例提亮不能救"的物理原因。

### 三、★ 踩到的坑：`scene.environment` 下 `material.envMapIntensity` 被 three **覆盖**

V5 与 V6 的统计**逐位相同**（`L=59.17 / p05=57.36 / p50=59.29 / max=78.2`）——
yaml 改了却毫无影响。根因在 three `0.186.0` `WebGLRenderer.js:2736`：

```js
if ( ( material.isMeshStandardMaterial || ... ) && material.envMap === null && scene.environment !== null ) {
    m_uniforms.envMapIntensity.value = scene.environmentIntensity;
}
```

⇒ **只有材质自带 `envMap` 时，它的 `envMapIntensity` 才被尊重。**
逐材质"关掉环境反射"的设计在 `scene.environment` 下**根本不生效**。

> 这个坑最初是被一个 TS 错误顺带暴露的：`environmentIntensity` 写错了层级 ⇒ 读到 `undefined`
> ⇒ 等效 `1.0` ⇒ 恰好解释了 `59.17` 这个"好看得过分"的数。**类型检查抓住了一个物理量级的错误。**

另外 `scene.environment` 是**全局**的，实测把整机非贴图件一并点亮（底座蓝板 ×3.38）——
那是另一个层面的美术改动，不该搭这趟车。

### 四、选定方案：逐材质 `envMap` + 程序化 `RoomEnvironment`

`RobotScene` 里的 `SceneEnvironment` 组件**整体撤除**，改为按材质下发：
`plateEnvironment.ts`（PMREM 生成与缓存）+ `applyAppearance(root, appearance, envMap)`。

- IBL 由 `RoomEnvironment` + `PMREMGenerator.fromScene(room, 0.04)` **程序化生成**
  ⇒ 不引入任何外部 HDR 资产（离线可复现、不加产物体积）
- 只有 `material.name === 'plateTexture'` 的材质被触碰，**其余材质一个字节不动**
- `envMap` 由调用方（`RobotArm`）在 `useEffect` 里取后传入 ⇒
  `buildRobotObject3D` 保持**不依赖 DOM / WebGL**（单测跑在 node 环境）

### 五、实测（同机位、同 ROI `x[402,520) y[380,640)`，n=30680 px）

| 量 | 改前 | 改后 V7 | 变化 |
|---|---|---|---|
| `L` mean | 0.08 | **12.21** | ×152.6 |
| `p95−p05`（动态范围） | 0.07 | **1.86** | ×26.6 |
| **`hp_std`（细节能量·主判据）** | 0.086 | **0.293** | **×3.41** |
| `\|dL/dy\|` / `\|dL/dx\|` | 0.010 / 0.027 | 0.055 / 0.140 | ×5.5 / ×5.2 |
| `L≥20` 占比 | 0.0% | 0.0% | 仍暗于背景 `22.7` ⇒ **剪影保留** |
| 泄漏检查（ROI 是否越界） | 0.00% | 0.08% | ROI 仍全在板内 |

**零外溢**（判据：按**基线图的底座蓝板像素集合**逐像素比对，46928 px）：

| 方案 | 底座蓝板 `L` mean | 倍数 | `max\|Δ\|` |
|---|---|---|---|
| 改前 | 50.637 | — | — |
| **V7 逐材质 `envMap`** | **50.637** | **×1.0000** | **0.00** |
| V6 `scene.environment` + 逐材质 `envMapIntensity=0`（无效） | 170.937 | ×3.3757 | 156.99 |
| V2 `scene.environment` 强度 0.25 | 100.964 | ×1.9939 | 84.34 |
| V1 `scene.environment` | 133.160 | ×2.6297 | 122.75 |

⇒ **逐像素完全相同。** "零外溢"是实测事实，不是"看着差不多"。

**旋钮性质被厘清**：`environmentIntensity` 与 `roughness` **只缩放整体亮度、不改变细节能量** ——
`L` 在 `7.54 ~ 30.40` 之间动了 4 倍时，`hp_std` 只在 `0.364~0.405` 之间动 11%。
所以"再亮一点"换不来更多细节；调这两个值只是为了落在**暗于背景但仍可读**的窗口里。

### 六、参数落点（`config/robot.yaml → appearance.texturedPlate`）

```yaml
environmentIntensity: 0.25   # 0 = 逐值回到引入本特性前的行为
exposureEv: 0                # 见 §一：提亮救不回来
roughness: 0.6
metalness: 0                 # ★ 钉死 0：F0 = mix(0.04, diffuse, metalness)，非 0 会连带放大高光
```

外观层，随时可迭代；`length` / `joints` / `actuators` **一行未动**（D55，`git diff --numstat` = `+36 −0`）。

### 七、★ 量测纪律：固定 ROI 只在**同一机位**下可比

`light_v7_home.png` 是**等轴测机位**，我拿前视的固定 ROI 去量它，读出
`L=27.68 / hp_std=5.37 / 泄漏 25.51%` —— **三个数全是假的**（ROI 大部分落在背景网格上）。
换机位必须重定 ROI，且**必须目视复核 ROI 是否真的落在物体上**（D39/D40）。

`measure_base.py` 第一版同样错：按"强蓝像素的 **bbox** 画矩形" ⇒ 矩形横扫下半幅
`x[243,1247)×y[450,899)`，混进**确实该变**的贴图黑板与 UI 面板 ⇒ 假 `×1.0546`。
正解是**用基线图算像素集合、在固定集合上量**（+3×3 腐蚀去抗锯齿边）。

### 后果

- ✅ 板面细节能量 ×3.41，肉眼可见表面纹理与螺栓；且**仍暗于背景**（12.21 < 22.7）⇒ 剪影不糊进背景
- ✅ 非贴图件**逐像素不变**（`max|Δ| = 0.00`）；IBL 程序化生成 ⇒ 零外部资产、零产物体积
- ✅ 四件套：`tsc` 0 error · `vitest` **313/313** · `vite build` 通过（`__armPilot` **0 命中**）· e2e `ui-smoke` **88/88**
- ⚠️ **上限到此为止**：本节所有手段都只是"把已经存在的镜面信息显示出来"。板面在 tile 里
  只用到 41 档码值，**真正能把细节送进数据的是拍摄端**（对板测光 / 补光重拍）。
  这是下一步"补拍其余 7 块板"该一并解决的问题，不是渲染层能翻过去的
- ⚠️ 本轮验收途中另踩两个坑，另记 D65

---

## D65 · 验收纪律：**恒假的端口预检** + 复用真机后端会把「前提不成立」读成「代码回归」

### 问题

本轮 e2e 出现 2 个 FAIL，全部指向「连着后端（末端**非** serial）点 Real Robot → **拒绝切换**」。
但环境里 8090 上有一个**上一轮遗留的真机后端**（`device=serial`，还开着串口）——
此时"**允许**切换"才是**正确行为**（同批 `真机链路 + Real Robot → 提示"正在驱动真实机械臂"` 就 PASS 了，自相印证）。
**这是环境差异，不是代码回归**（D40 / D43 的同一类错误）。

### 一、为什么开跑前没发现：预检本身是**恒假**的

我用的预检：

```bash
netstat -ano | grep -E "LISTENING" | grep -E ":(5173|5273|8090|5210|3000)\b"
```

它在 8090 **确实在监听**时打印了「无残留监听」。**两个独立缺陷叠在同一行上**：

| # | 缺陷 | 后果 |
|---|---|---|
| 1 | **漏了 `-E`** | BRE 里 `(` `\|` `)` 是**字面字符** ⇒ 这条正则**永不可能命中**（不是"偶尔漏"，是恒假） |
| 2 | 补上 `-E` 后，**`\b` 紧跟分组右括号 `)`** 在本机 **GNU grep 3.0** 下失效 | `(8090)\b` 不命中；改成裸字面量 `8090\b` 才命中 |

⇒ 叠加后预检**恒返回"干净"**。**一个恒假的检查比没有检查更危险：它把"我没查"伪装成"我查过了"。**

**可靠写法**（不依赖正则语义，按字段做字符串比较）：

```bash
netstat -ano | tr -d '\r' | awk '$4=="LISTENING"{n=split($2,p,":"); if (p[n]==8090) print $5}'
```

### 二、修法：把「前提不成立」显式建模成 **SKIP**

`ui-smoke.mjs` 的 `startBackend()` 会**复用** 8090 上的既有实例（设计如此：允许用户手工起后端），
但复用到 `device=serial` 时，依赖「末端非 serial」前提的子项**必然 FAIL**。修法：

1. 新增 `skip(name, reason)`：计入独立 `skips` 数组，**不计入 PASS/FAIL**，汇总时单列；
2. `isSerialLink` 为真时，那两个子项报 `[SKIP]` 并写明"serial 末端下该行为才正确"；
3. 复用真机后端时打**3 行醒目横幅**，明说"**这属于环境差异而非代码回归**"。

**两路都做了受控验证**：

| 场景 | 结果 |
|---|---|
| 干净环境（脚本自起 sim 后端） | `88/88 PASS, 0 FAIL, 0 SKIP` —— 无回归 |
| 末端 serial（`.workbuddy/captures/stub_serial_backend.mjs` 谎报 `device=serial`） | 那两条 → `[SKIP]`，**不再产生假 FAIL**（total `87 → 85`，正是这两条移出 `results`） |

⚠️ 桩**只回答 `/healthz`**：真起一个 `config.serial.yaml` 后端会去**打开串口**，
若机械臂正连着会产生**实际硬件副作用** —— 所以受控实验用桩，不用真机后端。

### 后果

- ✅ 端口预检换成字段级精确比较；e2e 对"前提不成立"有了 SKIP 语义，
  不再把环境差异伪造成回归，并主动提示怎么拿到完整结果
- ✅ 两路受控验证通过；清理也不再用 `taskkill //PID`（本机 Git Bash 下 `//PID` 不被转义成 `/PID`，
  报 `无效参数/选项`，而且是**静默失败**——我第一轮"杀掉 dev server"其实没杀掉），
  改用 PowerShell `Stop-Process`，并用 awk 复查
- ⚠️ `skip()` 目前只用在 Phase 8 那两条；**后续新增"依赖特定链路末端"的断言时应默认走 `skip()` 而非 `check()`**

---

## D63 · 前端接入照片纹理：**按面材质数组**；一条被 `flipY` 掀翻的朝向推理

### 问题

纹理已出 2 块（D60/D62），但 `frontend/` 里**一个图像素材都没有**（模型全靠程序化 primitive
+ 单色十六进制）—— 这就是"不像真机"的根因。本轮把它接进去。

### 一、必须是**按面材质数组**，不能整块板一个材质

实测（见下表）：`RoundedBoxGeometry` 的 UV 是「**每个面各自铺满 [0,1]**」，与长宽比无关。
若整块板用一个贴图材质，5mm 窄边也会被整张照片铺满 ⇒ 侧面出现被拉扁的螺栓，**比不贴更假**。
所以：大面贴照片、其余四面保持板色。大面 = `size` 里**最小那一维**所在的轴（薄板的两个最大平面垂直于它）。

### 二、实测 UV 表（**不许凭记忆推**）

`.workbuddy/captures/uv_probe.mjs` 直接读 geometry 的 `position/normal/uv/groups` 得出
（`RoundedBoxGeometry(22,5,74, 2, 2)`）：

| group | 法向 | Δu 方向 | Δv 方向 |
|---|---|---|---|
| 0 | +X | `-Z` | `+Y` |
| 1 | −X | `+Z` | `+Y` |
| **2** | **+Y** | **`+X`** | **`-Z`** |
| **3** | **−Y** | **`+X`** | **`+Z`** |
| 4 | +Z | `+X` | `+Y` |
| 5 | −Z | `-X` | `+Y` |

⚠️ 同一批实测还发现：**`RoundedBoxGeometry` 是「非索引几何」**（`g.index === null`），
group 的 `start/count` 是**顶点范围**而不是索引范围 —— 按索引去遍历会直接 `TypeError`。

规律：两个大面的 `Δu/Δv` **必有一个相反**（盒体展开的必然结果）⇒ 同一张照片只能在一面原样显示，
另一面必须镜像，否则从两侧看必有一侧左右/上下翻转。**"哪一面需要镜像"由代码推导**
（`mirrorAxisForThinAxis`，从上面这张表算出来），照片那一侧的不确定才交给 `robot.yaml`。

### 三、★ 踩到的坑：`TextureLoader` 的 **`flipY` 默认 `true`**

我据上表推导得出「面 3 (−Y) 需要翻 v」，实现后**实测完全相反**。根因：

> three 在**上传**纹理时默认做一次垂直翻转（`flipY = true`）
> ⇒ **图像顶行 ⇔ `uv_v = 1`**（而不是 `v = 0`）。

漏掉这一条，会让**所有 v 方向的推理整体反号**。修正后「基准面 = + 轴侧那个 group」需镜像，
即面 2 (+Y) 翻 v、面 3 (−Y) 原样 —— 与实测一致。**这一条比表本身更容易错**，
因为它是一个不会报错、也不会被类型检查发现的隐式默认值。

### 四、★ 怎么定案的：四象限**探针纹理**（受控实验）

拿真实纹理去比"螺栓该在哪"**定不下来** —— 真实纹理是近黑工程塑料，
特征只有"螺栓亮点 + 镂空暗缝"，而模型的螺栓位置（端盖在中线）与实物（有偏移）本就不同，
实测读数在 0.13~0.55 之间飘，不足以定案。

改用受控实验（`.workbuddy/captures/probe_axes.py` + `read_probe.py`）：
把纹理临时替换成**四象限纯色 + 对角白带**，同机位各拍一张，读数后**还原**（`git status` 复核干净）：

| 视角 | 期望（左上/右上/左下/右下） | 实测 |
|---|---|---|
| 相机在 **−Y** 侧（看面 3） | 红 / 绿 / 黄 / 蓝 | ✅ 完全一致 |
| 相机在 **+Y** 侧（看面 2） | 绿 / 红 / 蓝 / 黄 | ✅ 完全一致 |

⇒ **两侧都正确，且互为镜像**（这正是真实物体两面的关系）。这是"贴图朝向"唯一无歧义的判据；
也再次印证 D61 一 的教训：**目视/推断都不可靠，受控实验才算证据**。

### 五、工程侧的两个必要守卫

| 守卫 | 不加会怎样 |
|---|---|
| **无 DOM 时不加载纹理**（`CAN_LOAD_TEXTURE`） | `TextureLoader` 内部走 `document.createElementNS`，而测试跑在 **node 环境**（`environment: 'node'`）⇒ 5 个几何/验收测试直接 `ReferenceError: document is not defined`。纹理不参与任何几何判定，"无 DOM ⇒ 按没配纹理处理"是正确取舍 |
| **key 未登记时 warn + 回退纯色** | 贴图失败**不报错**（静默），"文件名打错一个字"会安静地表现为"板没变色" |

配套：`tests/unit/plateTexture.test.ts` 加了 7 项守卫（含**纹理长宽比必须与板大面一致**，
抓"把 forearm 的图配到 upper_arm"，以及"声明的 key 必须真的被登记"）。

### 六、观察：贴上去之后，板**变黑**了

大臂/小臂由亮蓝（`#4d8fe8` / `#6aa0ff`）变成**近黑** + 几个螺栓亮点，确实更像真机
（实物就是黑色工程塑料）。但**细节在深色 UI 光照下几乎不可见** —— 若后续要"更像"，
下一步该动的不是纹理而是**光照 / 材质**（如提亮或加 `emissiveMap`），这属于外观层，随时可迭代。

---

## D62 · 孔洞掩膜：实板**镂空** vs 模型**实心 box** ⇒ 把非板面区域填成板面色

### 问题

`*_vis` 是**实心 box**，而实板是**镂空桁架**。照片里"镂空透出的背景 / 被四角框进来的
相邻板 / 悬在板前的线缆"烘到实心盒上，会把背景色一起带进去 —— **比不贴更假**（D61 残留）。

### 三层判据（缺一不可，每层都是被实测逼出来的）

| 层 | 判据 | 不加会怎样 |
|---|---|---|
| ① 连通性 | 亮区**接触纹理边界** ⇒ 填（板外）；**被板面完全包围** ⇒ 再按面积定（小 = 螺栓保留，大 = 镂空填） | 纯亮度阈值会把板面上的**螺栓一起填掉** |
| ② 形状 | 保留的亮斑还须 bbox 填充率 ≥ 0.60 且长宽比 ≤ 2.0 | 只判面积 ⇒ 被暗区包围的**线缆小段**也被当螺栓留下（实测大臂纹理左下多出一块橙色残留） |
| ③ 色差 | `R − B > 15` ⇒ 填 | 暗部线缆的**亮度低于阈值**（中位 32 vs 阈值 60），只判亮度**永远去不掉** |

第三条的实测依据：板面像素 `R−B` 中位 **−19 / −16**、p90 **−8 / −10**，而暖色像素远超 15
⇒ 色差把两者分得很开。带占比守卫（暖色 > 20% 视为判据前提不成立，自动弃用）。

**填充色必须取实测板面中位色 = `RGB(1,1,20)`** —— 板面是**近黑带蓝**，不是纯黑；
填 `(0,0,0)` 会在板面上留下肉眼可见的色差补丁。

### 效果

| 纹理 | 填入比例 | 暖色残留 | 亮(>60) 像素 |
|---|---|---|---|
| `upper_arm_link` | 33.4% | **0.00%** | 1.25% |
| `forearm_link` | 35.2% | **0.00%** | 0.85% |

填充后纹理 = **近黑底 + 几个螺栓亮点**。这不是"丢了信息"：实板本来就是这样，
只是模型还没有镂空几何（把镂空建成几何是另一条可走的路，留作后续）。

### ★ 新增诊断判据：**四边亮比例**（"四角贴合吗"的定义本身就是它）

框精确贴合板 ⇒ 纹理四条边全落在板上 ⇒ 亮比例应为 0。实测：

| 纹理 | 上 | 下 | 左 | 右 |
|---|---|---|---|---|
| `upper_arm_link` | 45.2% | 18.5% | 31.1% | 46.4% |
| `forearm_link` | 57.6% | 33.7% | 61.5% | 19.7% |

⇒ **框比板大，且两个框偏心方向相反**（一个偏右上、一个偏左）。

⚠️ 早先加的"框外 / 镂空"二分统计**不能单独用来判贴合**：三角镂空若开口到板端，
也会接触纹理边界而被算进"框外"。四边亮比例才是定义上无歧义的判据。

### 判断：四角仍不完美，但**不阻塞**这一步

纹理信息量极低（亮像素仅 **0.85~1.25%**，其余是均匀近黑）⇒ 板面几乎不含空间信息，
四角偏差主要影响**螺栓位置的比例**，不是"整块板贴错"。这也意味着：照片纹理这条路的
边际收益**主要来自螺栓高光与表面细纹**，与"直接给一个黑色材质"差别不大 ——
所以要验证它值不值，得**先贴上去看效果**，而不是继续在像素上抠四角。

### 自测（`--selftest`）

合成"暗板 + 接触边界的亮带 + 被包围的大亮区 + 被包围的小亮斑"，断言
① 边界亮带填掉 ② 被包围的大亮区填掉 ③ **被包围的小亮斑保留**。
第 ③ 条是"判据是连通性、不是亮度"的证据（纯亮度会把它一起填掉）。

### 教训

**同一个数在不同前提下会有相反解读。** "框外面积大"一开始被我读成"框比板大"，
但镂空开口到板端同样会算进框外 —— 于是换了一个**定义上无歧义**的判据（四边亮比例）。
**当两种解释都能套上同一个数时，该换判据，而不是换参数。**

## D61 · 渲染**纵横比搞反** ⇒ 两条旧结论作废；修正手性后 IoU 30%→43%；四角改「投影初值 + 梯度吸附」

### 一、发现的 Bug（影响面很大）

```python
mujoco.Renderer(model, height, width)   # 参数顺序是 (height, width)！
```

我全程按 `mj.Renderer(m, 640, 360)` 调用，得到的是 **640 高 × 360 宽**的**竖幅**图；
后续又按横幅 `paste` / `resize` 贴片、裁剪 ⇒ **所有渲染对照图都是被拉伸或裁切的**。

### 二、由此得出的两条结论**作废**

| 旧结论 | 更正 |
|---|---|
| "azimuth=270 与实拍机位一致（臂向右）" | 正确渲染显示 **azimuth=90** 才是实拍机位；此前模型是**镜像**的 |
| D60「模型的臂整体比实拍高一个量级 ⇒ 模型几何与 3D 打印实件有系统差异」 | **撤回**，那是渲染伪影。模型结构与实拍一致 |

> 教训：**工具调用的参数语义必须先验证**。把 `(H,W)` 当 `(W,H)` 传不报错、不告警，
> 但会让下游**每一条**目视结论都错，而且错得"看起来很有道理"。

### 三、镜像的代价：IoU 永远上不去

整臂剪影 IoU（已知模型 + 拟合姿态）在**镜像**条件下：v1 36.3% / v2 25.3% / v3 30.2%。
修正手性后 **43.2%**（`s=4.05, θs=66.5°, θe=109.5°`）。

**s 第三次独立吻合**：3.50（IoU v2）/ 3.80（旧 CALIB）/ 4.05（修正手性后）。
⇒ `s ≈ 3.7–4.1 px/mm` 可信。D59 说"s=3.80 是搜索下界截断值、真值 2.58"**只对了一半**：
截断确有其事，但 2.58 是欠定下的另一个假解，不该当成真值。

### 四、线缆**不在板面上** —— 选项 A 不是必需的

4× 放大后看清：橙色线缆在板的**背面**，是从**三角镂空里透出来**的，并没有挡住板面。
⇒ 上一轮"线缆无解、必须物理拨开（选项 A）"的判断**过强**。
真正的问题是**四角把镂空区一起圈了进来**。

### 五、新增可用方法：**投影初值 + 梯度吸附**（`.workbuddy/captures/snap_quads.py`）

单靠模型投影不够（IoU 43% ⇒ 误差几十像素），单靠目视也不够（±15px，且板与关节块连成一片）。
两者**合起来**可行：投影给"大致位置与走向"，梯度吸附给"像素级贴合"（edge-snapping）。

| 板 | 边分（初始→吸附） | 吸附后 短边×长边 | 长宽比 vs 真值 |
|---|---|---|---|
| `upper_arm_link` | 8.78 → **19.30** | 96.2 × 323.7 px | **3.37** vs 3.36 ✓ |
| `forearm_link` | 4.65 → **18.74** | 75.8 × 286.4 px | **3.78** vs 3.78 ✓ |

长宽比同时精确吻合，是这次吸附的主要可信度来源。

### 六、残留问题（未解决，如实记录）

纹理一侧仍有**大块亮楔形**（镂空/背景），线缆从镂空里透出。
根因是**语义不匹配**：实板是**镂空**的，而模型的 `*_vis` 是**实心 box**。
⇒ 需要孔洞掩膜（skill 已有条目）或把镂空建成几何，不能靠调四角解决。

## D60 · 新机位后：**窄带 PCA** 给出四角，纹理已出 2 块；模型投影到此为止

**背景**：用户按 D59 的要求**重新取景并挪近**了机位。新帧（1280×720）里
**整臂完整入画**（不再被顶边裁掉），背景干净（背景最暗 115，目标板中位 2~8，阈值可提到 65）。

### 一、先纠一个误报：`ramp = 10.78px 失焦` 是**假的**

`project_plates.py` 的 `ramp` 是在**投影框边缘**测"暗↔亮"过渡的。旧标定定位本身就错（D59），
框落在**暗区内部**，边缘压根没有真实边界 ⇒ `ramp` 被拉到 10px。
放大目视：板边缘过渡约 **2px**，**并未失焦**。
⇒ **`ramp` 只有在四角正确时才有意义**，绝不能拿它反推"相机失焦/别再靠近"。

### 二、整臂剪影 IoU 配准（`fit_arm.py`）—— 失败，但失败得有信息量

思路：用已知模型 + B 位姿态（`shoulder=45`）投影**整条臂的剪影**，与实际剪影做 IoU 配准。
整臂轮廓是独特形状，约束远强于 D58 的"三个矩形框"（那个是欠定的）。

| 版本 | 做法 | IoU | 结果 |
|---|---|---|---|
| v1 | (s, ox, oy) 全自由 | 36.3% | 最优解把模型推到 **oy=830（画面外）** —— 又是 D58 的退化模式 |
| v2 | **bbox 中心强制对齐**（解析解 ox/oy），只搜 s | **25.3%** | s=3.50 / ox=540 / **oy=644** |
| v3 | 再联合搜 (s, shoulder, elbow) | **30.2%** | s=4.35 / shoulder=41.5° / elbow=115.0° |

- v2 的 `3.50 / 540 / 644` 与 `project_plates.py` 内置 `CALIB = 3.80 / 530 / 645`
  **高度接近** ⇒ 两个独立方法交叉印证，这部分可信。
- **30.2% 是上限**：叠加图显示**模型的臂整体比实拍"高"**，不是调角度能补的
  ⇒ 模型几何与 3D 打印实件存在**系统性差异**。
- 同时 v3 反解出：**物理臂实际角度与下发命令差 ~15°**（`shoulder 45→41.5`、`elbow 112.6→115.0`）
  —— MG90S 开环无回读的必然（D34）。
- ⚠️ 踩坑：`table_top`（约 1m 见方）也在投影件里 ⇒ 剪影被撑成**全屏矩形**，
  IoU 退化成"实际面积占比"（19.3% ≈ 10069/57600）。**必须排除桌面。**

⇒ **基于模型的投影到此为止**：模型与实机的差异 + 姿态不可知，两者叠加把精度锁死在 30%。

### 三、可用方案：**窄带 PCA**（`plates_corners.py`）

板本身是一条**直带** ⇒ 沿长轴开一条窄带（半宽 ~35px），**带内**暗像素做 PCA 取主轴极值四角。
带外的一切（螺栓、立柱、平行杆、线缆）被**结构性排除** —— 这是绕开"整条臂是同一连通域"（D58 ③）的正解。

实测：大臂 **241×65px**（长宽比 3.69，真值 3.36）；小臂 **200×72px**（2.76，真值 3.78）。
⚠️ 中心线端点仍来自**目视读数**（±15px）⇒ 精度有限，但比整图分割和模型投影都好，且**可复现**。

### 四、纹理已出 2 块（`make_texture.py --corners`，四角再**向心收缩 8%** 以避边缘）

| 纹理 | 尺寸 | 质量 |
|---|---|---|
| `forearm_link` | 18×68mm → 180×680px | ✅ **干净可用**（黑底 + 两枚螺栓 + 镂空） |
| `upper_arm_link` | 22×74mm → 220×740px | ⚠️ 中部有**一条橙色线缆横穿** |

**大臂那条线缆是无解的**：实拍中线缆**悬在板前面**，只要框圈住板就必然含它 ——
向心收缩只能去**边缘**、去不掉**中间**。**软件层面没有办法。**

其余 7 块（`base_link` / `column_link` / `column_link_side` / `upper_arm_brace` /
`forearm_brace` / `tool_link` / `jaw_link`）在 B 位**被遮挡或不在视野**，需要额外姿态/机位。

### 五、结论：这条路的天花板由**物理条件**决定，不由算法决定

要拿到干净纹理，必须同时满足：
1. **目标板完整入画**，且**不被线缆横穿**；
2. **目标面正对相机**（法向平行于视线）。

两条都是物理条件。算法能做的只是"在满足条件后把四角标出来"。

### 六、顺带记两条环境事实

- MuJoCo 离屏渲染在本机**上限约 320×180**（640×360 会失败）；
- 渲染要看到颜色必须 `m.geom_matid[i] = -1` 先清材质，否则 `rgba` 被 material 覆盖成白。

## D59 · `s = 3.80` 是**搜索边界截断值**；模型投影在本机场景**无法定位**；改走「致动并观察」

**背景**：用户拍板走 A 方案（物理靠近相机提高 `px/mm`）。为此给 `tools/project_plates.py`
加了距离扫描模式（`--capture` 抓帧 + 一句话 `CONCLUSION` + **搜索边界命中检测**）。
**结果第一次跑就推翻了 D58 里被当成真值的 `s = 3.80`。**

### ① 旧标定值是被截断的，不是测出来的

D58 的 `CALIB = {s: 3.80, ox: 530, oy: 645}` 中，`s = 3.80` **恰好等于当时的搜索下界
`S_LO = 3.8`** ⇒ 它是被区间**截断**的边界值，**不是自由极值**。
把区间放宽到 `2.28 ~ 6.84`（`CALIB.s × 0.6~1.8`）后，最优解落到 **`s = 2.58`**。
⇒ 同一台机器、同一个场景，"测量值"从 3.8 变成 2.58 ——
**这解释了为什么按 3.80 出的三块纹理全都不合格。**

**已加永久防护**：最优值落在区间端点 2% 以内时打印
`⚠️ s 落在搜索边界上 ⇒ 范围设窄了，用 --s-range … 放宽重跑`。

> **可复用教训**：**任何带区间的搜索都必须显式检测"最优解是否贴边"。**
> 贴边 = 该值不是测量结果、是区间的人为截断。这比"分数不高"更隐蔽 ——
> 分数看着还行、数值看着合理，实际毫无意义。

### ② 但放宽区间后，投影**依然对不上**

在新搜索最优（`s=2.58 / ox=660 / oy=445`，score **18.2**）下做叠加复核：
**三个框都没贴合板边**（大臂框落在立柱中段，边缘全是"暗↔暗"、没有亮度跃变）。
把参数手动拨回旧标定附近（`s ∈ 3.4~5.0`、`ox=560`、`oy ∈ 600~690`）逐点扫描，
score 只有 **1.9 ~ 4.2**，**比 18.2 还差 4~9 倍**。

⇒ **在本机这套场景里，「缩放 + 平移」模型无法定位目标板**，原因不止在搜索算法：

- 板、立柱、底座、肩座**同一种黑且物理相连**，投影框落在哪都在暗区里；
- 三块目标板在 HOME 位**竖直方向首尾相接**（立柱 Z[38,60] 紧接大臂 Z[59.8,134.2]），
  投影上连成一条 ⇒ **没有任何一条边能提供唯一定位约束**；
- 相机相对机械臂的**真实位姿从来没标定过** —— `s/ox/oy` 是从单帧拟合的，
  3 个自由度只有"三项乘积"这一个标量约束。

### ③ 新发现：机械臂被画面**顶边裁掉**了

行剖面实测：`y = 0` 处有一条 **x[506,760] 的连续暗带**，从 y=2 起才裂成多段
⇒ 结构在顶边被**切断**。**目标板不在画面内，纹理四角必然缺角。**
⇒ 无论走 A 还是 B，**重新取景都是必须先做的一步**。

### ④ 可行的新方法：致动并观察（motion-diff）

纯几何推理走不通时，改用**机械臂自己的动作**建立"图像区域 ↔ 连杆"的对应：
动一个关节 → 拍前后两帧 → 差分。差分区域 = **该关节及其下游零件**。
完全不依赖"相机在 −Y""板面平行于像平面"等任何假设。

实测（`HOME / shoulder=45 / elbow=141.8 / base=60`，各拍一帧）：

| 动作 | 变化像素 | 占比 |
|---|---|---|
| `shoulder` 0.85 → 45 | 183097 | **19.87%** |
| `elbow` 112.62 → 141.8 | 131320 | **14.25%** |
| `base` 0 → 60 | 148177 | **16.08%** |

**两个结论**：

- ★ **物理臂确实在跟命令走** ⇒ 排除"掉电 / 堵转 / 打滑"。
  「MG90S 无位置回读」只是说**不能证明"到位"**，不等于"没动"——现在有了正面证据。
- 用「**随 shoulder 变 且 不随 elbow 变**」筛出的区域 = **大臂板**
  （实测落在 x≈622–706，宽 84px，随肩转而不随肘动）——
  这正是纯几何做不到的**归属判定**。

> ⚠️ 但 motion-diff **不能直接给出干净的单板四角**：肩座、平行连杆、线缆、阴影
> 与目标板连成一片，PCA 外接四边形得到的是"整块区域"（实测 bbox 395×445 px）。
> ⇒ 它适合**归属判定与粗定位**，精确四角仍需**人工标 + 目视复核**。

### ⑤ 当前判定：A 必须与「重新取景 + 人工标四角」配套

`px/mm` 只由物理距离决定（D58 ②），所以"靠近"仍然是对的、也是唯一的方向；
但它**不能单独解决问题** —— 自动定位在这套场景下不可用。
⇒ 顺序应为：**重新取景（三块板完整入画、不贴边）→ 物理靠近到目标 px/mm →
人工标四角 + 目视复核 → 出纹理**。

## D58 · 真机摆位：动关节**改变不了分辨率**；「自动分割出单块板」不可行

**背景**：为了让照片纹理贴图（D55–D57）真正用上真机，把「本机摄像头 + 机械臂自己摆位」
打通：起真机后端（`config.serial.yaml`，COM16 CH340 实测在线）→ 新增
`tools/set_joints.mjs` 走关节级 WebSocket 驱动 → 抓帧 → 投影/判定。

**三条实测结论（都推翻了先前的方案）**

1. **`base` 旋转对分辨率是零和的。**
   远端放大实测 **×1.08**（base=0 → −20°），但板法向随之偏 20°、投影缩短 `cos20° = 0.94`
   ⇒ **净 1.015**，还白搭一层透视。**转 base 是纯亏。**
2. **肩/肘摆动改变不了 `px/mm`。**
   相机在正前方（−Y），摆动平面（X–Z）**平行于像平面** ⇒ 摆动不改变距离、也不改变投影长度。
   仿真扫 shoulder：大臂板倾角 **89.2° → 79.2° → 59.2° → 41°**（**板随肩同步转**，长度不变）；
   真机 HOME vs `shoulder=30` 两帧**暗色投影面积 236345 → 230804（−2.3%**，差值来自遮挡变化）。
   ⇒ **提高分辨率只有物理靠近一条路**，没有任何软件办法。
3. **白卡隔离不成立。**
   实测暗像素占整帧 **25.7%**，左下象限高达 **65%**；板、立柱、底座、舵机、控制板**同一种黑**，
   且板与板**由螺栓物理相连** ⇒ **不存在"只含一块板"的连通域**。
   白卡挡得住背景，挡不住**结构性连通**。
   （早期建议已从 `docs/texture-capture-guide.md` 与 README §12 全部删除。）

**替代方案：相机标定 + 运动学投影（`tools/project_plates.py`）**

三块目标板的大面法向实测均为 `(0,1,0)` ⇒ 板面**平行于像平面** ⇒ **正交模型就够**
（`u = ox + s·X`、`v = oy − s·Z`）。

目标函数**三项相乘**（任一项退化都会压低总分）：
① 框内暗色比例 · ② 板间空隙亮色比例 · ③ 框边命中「暗↔亮」边界比例。

> ★ **②是防退化的关键**。实测只优化①时，`s=2.5` 会把三块板挤成一团塞进左下底座暗区，
> 拿到 **97.4% 的假性满分**（而 `oy=770` 已经在画面外）—— 与 D56「静默错法」同族。

**结论与边界（必须写明）**

- 标定得到 `s ≈ 3.80 px/mm · ox ≈ 530 · oy ≈ 645`，**数值自洽**：三块板的投影与图像位置
  一一对应（大臂板 Z[59.8,134.2]→画面 y[130,413]、立柱侧板 Z[38,60]→y[412,496]）。
- ⚠️ **但精度不足以直接产出可用纹理**：框边缘会夹进橙色线缆/背景，
  3 块纹理经**目视复核判定不合格并已删除**。
- ⇒ 目标函数**不足以唯一定位**（暗区太大，框整体平移 ~70px 后三项分数照样不低）。
  **一律要目视复核叠加图 + 纹理**；必要时用 `make_texture.py --corners` 人工标四角。

**顺带钉住的两个机制性教训**

- **`hello` 里没有 `connected` 字段**（`docs/serial-v1.md` §5.1 写错了）。
  判"是否真机"必须用 `device == "serial"`；否则跑 `config.yaml` 时会把虚拟臂当真机，
  "摆位成功"是幻觉。`set_joints.mjs` 据此**拒绝向非串口链路发送**。
- **`hello` 与 `joint_state` 同时到达、`hello` 在前**。若在 `hello` 分支里就发指令，
  "保持不动"的关节会 fallback 到 `homePose` —— 那不是保持，是**把臂拉回 home**。
  发送点必须挪到收齐 `joint_state` 之后。另加 `--home` 保证每组实验从**已知状态**出发
  （否则状态跨实验累积，两组对比图的机位根本不同，读数全部作废）。

## D57 · 采集判定：**量错对象**与**说不该说的话**，比不量不说更贵

**背景**：为了让「照片纹理贴图」（D55/D56）能闭环，做了 `tools/capture_texture.py`：
从本机摄像头抓帧 → 当场判定能不能用作纹理 → **给出该往哪动的指令**。
产物是**给操作者的行动指令**，而指令错了**不会报错** —— 人会照着一条错的指令
去调设备，调很久也调不好。以下五条都是本轮实测踩出来的。

---

### 一、局部轴 ≠ 世界轴：文档因此错了两行

`docs/texture-capture-guide.md` 的朝向表原本把 plate `size` 的**最小维**
（局部薄轴）直接写成"大面法向 ±X/Y/Z"，并据此给出"侧面拍 / 俯拍"。
第一批 4 张**全部正确**，但第二批有两个是错的：

| 板件 | 局部薄轴 | 本位姿**实测**世界方向 | 指南原文 | 判定 |
|---|---|---|---|---|
| `upper_arm_link_vis` | Y | 世界 **Y**，\|1.00\| | 侧面 | ✅ |
| `forearm_link_vis` | Y | 世界 **Y**，\|1.00\| | 侧面 | ✅ |
| `base_link_vis` | Z | 世界 **Z**，\|1.00\| | 俯拍 | ✅ |
| `column_link_vis` | Z | 世界 **Z**，\|1.00\| | 俯拍 | ✅ |
| `upper_arm_link_vis3` 端部横撑 | Z | 世界 **Z**，\|1.00\| | 俯拍 | ✅ |
| **`forearm_link_vis3` 腕部横撑** | Z | **世界 X**（沿臂伸出方向），\|0.92\| | 俯拍 | ❌ |
| **`tool_link_vis` 腕部本体** | Z | **世界 X**（沿臂伸出方向），\|0.92\| | 俯拍 | ❌ |

**根因**：位姿一折叠，局部轴映射到哪个世界轴就变了。局部 ±Z 不等于世界竖直。

**修法**：表格分成「局部薄轴」与「本位姿实测朝向」两列，朝向从 MuJoCo 模型
**读出来**而不是推。并在表前写明**别背轴向 —— 拿实物转一圈，找面积最大的那个面**。

**实测依据**（也是本轮的坐标系基准）：`base` 关节轴 = 世界 Z、`shoulder`/`elbow` 轴 =
世界 Y ⇒ 机械臂在 **X–Z 竖直平面**内摆动、本位姿朝 +X 伸出；
`base_link` 的 7mm 薄轴 = 世界 Z（竖直，平放）。

---

### 二、「不可判定」≠「判定不通过」

第一版把"画面里最大的暗色连通域"当成目标板。可**臂板由螺栓连成一体**，
暗色部分天然连通 ⇒ 那个连通域是**整条臂**。于是报告输出：

```
✗ 左右有透视：左边 885 / 右边 313 px（差 182.8%）—— 右边离镜头更远
✗ 板出画了（左上贴边）
```

数字全是真实测量的，**却归因错了对象**。人照着去调倾斜，调一天也不会好 ——
因为问题根本不在倾斜。这与 D56（PIL QUAD 静默平移）同族：*不报错、看着合理、方向全错*。

**修法**：`evaluate()` 改为**三态** —— `ok` / `reject` / `undecidable`，
并先判"这一帧能不能判"，再判"板好不好"：

- 最大连通域**触边**，或轮廓**长宽比**与目标板差 >30% ⇒ `undecidable`
- `undecidable` 时**不打**四角/对边/px-mm 这些**拿整条臂算出来的虚数**
  （第一版打了"12 px/mm"，看着还挺好），只打真正被用到的原始事实，
  并给出可执行办法（白卡隔离 / 推到只剩目标板）

> 通用规则：**当"这一帧判不了"时，绝不要输出任何"看起来像结论"的数字。**

---

### 三、指标必须量在**对的对象**上：从"板内部方差"到"归一化边缘过渡宽度"

清晰度第一版用**板 bbox 内部**的拉普拉斯方差。**量错了对象**：
板面是均匀的黑色亚克力，内部本来就没有高频 —— 一块对不上焦但表面干净的板，
内部方差同样接近 0。合成帧上它给出 **0**。

**修法**：改为量**轮廓**上的梯度。真正会因失焦而垮掉的是边界：
对焦时 240→30 的跳变集中在 1px，失焦时摊成 5~10px 的斜坡。

第二版又差点错第二次：**用梯度的绝对值当阈值**。绝对值同时被**光照**缩放 ——
真机那一帧背景墙只比黑板亮 **119** 级、斜率只有 **53**；换个亮场景同一条边给出 100+。
用绝对值当判据，等于把"灯够不够亮"当成"焦准不准"。

**最终判据**：归一化的**边缘过渡宽度**

```
ramp_px = ΔI / (2 · 斜率)        理想硬边 = 1.0，与光照、与放大率都无关
```

实测校准（Integrated Camera 720p，真机场景）：

| | 真机抓拍 | 高斯模糊 r=1 | r=2 | r=3 |
|---|---|---|---|---|
| ramp_px | **1.17** | 1.8 | 3.1 | 4.6 |

取上限 **2.5** ⇒ 放行真机现在画质，挡住明显虚焦。
（顺带修正我自己一个错误印象：真机那帧肉眼看"偏糊"，实测边缘已接近理想硬边。）

---

### 四、真机臂板是**镂空桁架**，`geometry.size` 只是外接盒

抓帧放大后可直接看到**透过臂板三角开口背后的墙**。而前端
（`src/components/RobotScene/buildRobotObject3D.ts:140`）把每块 `plate` 都渲染成
`RoundedBoxGeometry` —— **实心倒角盒**。

推论（决定了贴图策略，已写入采集指南 §2.5）：

- 轮廓填充率**天然只有 40~50%** ⇒ 这必须是**提示**而不是**阻塞**。
  否则每一张真机照片都会被判死，工具变成"永远说不"的噪音源，人就开始忽略它 ——
  **那等于没有工具**。
- **直接在实心盒上烘照片纹理会把背景色带进孔洞**，贴上去比现在更假。
  ⇒ 贴图阶段需要**孔洞掩膜**（孔洞填成板面色），或把镂空建成几何。列为待办。

---

### 五、一条我自己写错的指令：横向增益

分辨率不足时，工具原本说"转成横向，同样取景下 px/mm 直接 ×1.78，这是白拿的"。
**这是错的**：同取景下旋转不改变 px/mm。

真实的增益是：长边横过来后，**顶到画面边之前还能再靠近 1.78 倍**，
所以 px/mm 的**上限**提高 1.78 倍。已改为该说法。

> 之所以要单列这条：硬件事实是 **相机上限只有 1280×720**
> （`ffmpeg -f dshow -list_options true` 实测，只有 720p/540p/480p/360p）。
> 增益数字由**画面尺寸**推出并**写进测试** —— 换相机时测试会失败并提醒改文案，
> 而不是让文档留着旧数字。

---

### 防倒退

`tests/sim/test_capture_judge.py`（16 项），刻意分**两条测试线**：

- **纯逻辑线**：直接构造指标字典调 `evaluate()`。倾斜方向、分辨率建议、归一化判据
  都是对指标的**算术**，用构造字典测才精确。
  *（实测教训：想合成一个"只有左右倾斜、没有上下倾斜"的多边形是**做不到的** ——
  等腰梯形左右边必然等长。硬凑只会得到脆弱且失真的样本。）*
- **图像线**：`analyze → evaluate` 全链路，守住两者的**数据契约**
  （少一个键就是运行时 `KeyError`，只在真机采集时才暴露）。

其中三条是直接钉住本 ADR 的错误：

| 测试 | 钉住的错误 |
|---|---|
| `test_undecidable_when_blob_touches_border_and_never_blames_tilt` | 断言**不得**出现"离镜头更远" —— 禁止再把"框错对象"说成"板倾斜" |
| `test_dark_scene_is_not_mistaken_for_blur` | 喂**真机实测值**（ΔI=119、斜率 53）必须通过 —— 禁止退回绝对值判据 |
| `test_truss_openings_are_a_note_not_a_blocker` | 填充率 0.40 时仍须 `ok` —— 禁止把镂空升级成阻塞 |

## D56 · PIL 的 `Image.transform(QUAD)` 不能用于透视校正 —— 它会引入 +12~17px 的**静默**平移

**背景**：为「照片纹理贴图」写透视校正管线（`tools/make_texture.py`），
把照片里板的任意四边形拉成按真实尺寸归一化的矩形。第一版用 PIL 自带的
`Image.transform(QUAD)`（Pillow 有现成的四点透视变换，看着正合适）。

**现象**：合成自测报告刻度线位置偏差 **+12 / +17 / +13 px**，而四角估计本身
误差只有 **1.00 px**（说明问题不在找角上）。

### 排除法（每一步都排掉一个假设）

| 假设 | 实验 | 结果 |
|---|---|---|
| 四角估计不准 | 合成图里"估计四角 vs 真值四角" | max\|Δ\| = **1.00 px** ⇒ 排除 |
| 角点顺序搞反了 | 用四象限着色的源图（TL=10/TR=90/BR=170/BL=250）做变换 | 输出四角**完全一致** ⇒ 顺序假设成立 ⇒ 排除 |
| 缩放比例错 | 实测刻度间距比 | **1.0027 ≈ 1.0** ⇒ 缩放对 ⇒ 排除 |
| **重采样本身** | 换掉 PIL QUAD，用自实现 homography 重采样，**同一组四角、同一次 round-trip** | 刻度线 **[185, 370, 554] 精确命中**（偏差 ±0.8px）⇒ **PIL QUAD 是元凶** |

```
真值            [185, 370, 554]
PIL QUAD        [197, 387, 567]   ← +12~17px
自实现 homography [185, 370, 554]   ← 精确
```

### 结论与修法

`rectify()` 改为**自实现 homography + 双线性重采样**（`homography()` / `_warp()` /
`_sample_bilinear()`），不再使用 PIL 的 QUAD/AFFINE 类变换。

> **坐标约定必须写死在代码注释里**：输出像素**索引** (i,j) 对应源坐标 `H·(i,j,1)`，
> 且两图四角按 左上/右上/右下/左下 一一对应。这条约定不写清楚，下一个接手的人
> 会重新踩一遍 —— 因为 PIL 自己的约定与它**不一致**，而"不一致"这件事不报错。

### 为什么这类错误最贵

它**不抛异常、不崩、输出尺寸正确、内容也大致对**。表现只是"纹理好像被挪了一点点"，
而在 3D 场景里看起来像"模型细节没对齐"或"贴图有点怪" —— 归因方向全错。
这与 D51（接触记录≠接触力）、D54（只在特定调用方式下暴露的静默错误）是同一族。

### 附带修掉的一个小问题

`_sample_bilinear` 的区域判据原本用严格的 `u ∈ [0, w-1]`。homography 有浮点误差
（输出 (0,0) 反投影出 `99.9999998`）⇒ 边界行被判成"区域外"填背景色 ⇒
**纹理边缘出现一圈白边**。真实照片的四角估计永远差半像素，所以这在生产中**必然**发生。
改为半像素容差 `u ∈ [-0.5, w-0.5]`（双线性在边缘本就该 clamp 到边界像素）。

### 防倒退

- `tests/sim/test_texture_pipeline.py::test_rectify_rejects_pil_quad_based_mapping`
  直接扫描 `rectify` 的源码，禁止 `Transform.QUAD` / `Image.QUAD` 重新出现。
- `test_roundtrip_is_identity_for_rectangular_quad`：矩形情形 round-trip 恒等（无透视，
  任何偏差都只可能来自重采样约定）。

## D55 · 「冻结运动学/物理真值」必须分**两级判据** —— 整文件哈希会误伤视觉层

**背景**：用户要求「保存目前的运动学物理数据不变」。字面做法是给 `config/robot.yaml`
与 `config/physics.yaml` 存一份快照 + 一个整文件哈希守卫。

### 为什么整文件哈希是**错误的**判据

`robot.yaml` 同时装了两类完全不同的东西：

| 类别 | 字段 | 是否该冻结 |
|---|---|---|
| **运动学真值** | `links[].length` · `joints[]` · `actuators[]` · `robot.tcp` · `robot.homePose` | ✅ 是 |
| **外观几何** | `links[].geometry` · `links[].details`（板件/舵机/轴销 + 颜色） | ❌ 否 |

而文件里早就明确写着「**改 geometry 只影响外观，绝不影响 FK / IK / 标定**」。
后续「图像采集 → 更真实的视觉建模」**必然要动外观那一层**。整文件哈希会把**合法**的
外观变更判成违规 ⇒ 守卫要么被绕过、要么被删掉。

> **一个会被绕过的守卫，等于没有守卫。** 这是本条的核心。

### 做法：两级判据（`tools/freeze_baseline.py`）

```
Level 1  整文件 SHA256         记录用。变了就提示"去看看是不是只动了外观"
Level 2  **语义核心 SHA256**   真正的判据，只覆盖运动学 / 物理字段
```

```python
KINEMATIC_LINK_KEYS = ("id", "parent", "length")          # ✗ 不含 geometry / details
PHYSICS_SECTIONS    = ("timestep","solver","gravity","servo","inertia",
                       "contact","limits","calibration")   # ✗ 不含 recording / deterministic
```

- 字段白名单（**不是黑名单**）：将来 yaml 新增运动学字段而忘了登记时，守卫会**漏检**
  而不是**误报**。漏检可以接受，误报会让人把守卫关掉 —— 那才是真正的失败。
- `recording`（I/O 配置）与 `deterministic.seed`（运行参数）**不是物理量**，
  改记录开关不该报"物理参数被改"。

于是：

| 改动 | L1 | L2 | 结果 |
|---|---|---|---|
| 改 `geometry.color` / `size` / `details` | 变 | 不变 | **放行**（视觉建模正需要） |
| 改 `length` / 限位 / `coupling.gain` / 标定 offset | 变 | **变** | ❌ 报错 |
| 改 `gravity` / `friction` / `timestep` | 变 | **变** | ❌ 报错 |
| 改 `recording.enabled` / `deterministic.seed` | 变 | 不变 | **放行** |

### 守卫的"辨识力"必须被测试 —— 否则它可能是假绿

一条永远返回通过的守卫，和没有守卫在报告里长得**一模一样**（都是绿色）。
所以 `tests/sim/test_baseline_frozen.py` 除"真值没变"外，还**用内存变异证明守卫对
外观不敏感、对真值敏感**（`copy.deepcopy` 后改 dict，**不碰** yaml 文件 ——
测试不该有"跑挂了就把真值改坏"的尾部风险）。

另用一次性脚本做过**反向验证**：把 `column_link.length` 60→61 ⇒ 测试**确实失败**，
且报错精确到 `kinematic_summary/links/column_link: {'length': (60, 61)}`；还原后
字节一致、11 passed。

> 报错只说"某个哈希变了"帮不上任何忙。`_diff_summary` 逐字段列差异，是这条守卫
> 能被真正使用（而不是被忽略）的前提。

### 推论：视觉层从此可以自由迭代

外观（几何 primitive / 颜色 / 未来的照片纹理）**不再受冻结约束**，
而运动学与物理被钉死。这正是「图像采集 → 更真实建模」能推进的前提条件。

## D54 · 三个**只在特定调用方式下暴露**的静默错误：累加器 · settle 判据 · 双重换算

**背景**：MuJoCo 后端落地过程中踩到三个 bug，共同特征是**代码本身看起来完全正常、
单元测试也能过，只在换了调用方式或换了输入量级时才现形**。它们的代价都极高
（表现成"命令没生效"/"模型是坏的"），因此值得单独记一条。

### 一、控制周期累加器写成了局部变量

```python
def step(self, n=1):
    for _ in range(n):
        accum = 0.0                      # ✗ 每次 from 0
        accum += dt
        if accum >= ts_ctrl:  ...        # 永远达不到 10ms
```
- `settle()` 内部逐次调 `step(1)` ⇒ 累加器每次都从 0 开始 ⇒ **ctrl 永不更新 ⇒ 命令完全不生效**
- 而 `step(200)` 这类"一次调用多步"的写法却**完全正常**

⇒ 症状是「用 `settle()` 测就全挂、用 `step(200)` 测就全过」，极难归因。
**修法**：累加器提升为实例状态（`self._ctrl_accum`），`reset()` 里归零。
**回归**：`test_step_batching_does_not_change_result`（`step(1)`×10 ≡ `step(10)`，按位比较）。

### 二、`settle()` 用"单步速度低"当静止判据

`reset()` 之后 `qvel` 恒为 0 ⇒ 第一次 `step` 就判定"已静止"并返回 ⇒
所有"下命令然后 settle"的测试都读到**从未移动过的初始位形**（又是"命令没生效"）。
**修法**：改成「连续 `hold_s=0.2s` 内速度都低于阈值」。

### 三、惯量的**双重单位换算**（静默率最高）

`physics.yaml` 已声明全 SI（米），生成器又对 `com`/`length` 做了一次 `mm2m()`
⇒ 40 mm 被写成 40 µm，惯量掉到 **1e-11**，再经 `%.6f` 格式化后**显示成 "0"**。
MuJoCo 对零惯量要么报错、要么**静默产生一个不受力矩的关节**。
**修法**：不再换算；格式化改用 `%g`（自动切科学计数法，MuJoCo 解析器支持）。

### 四、顺带澄清一个语义（不是 bug，但会被误读）

`SimState.target_angles` / `record.py` 的 `target_joint` 记的是**上层请求的目标**，
不是"限速后真正写进 `ctrl` 的目标"。两者差 = 舵机速度限幅欠的债。
旧 docstring 写成了后者（"速率受限目标"），与代码不符 —— 已修正，并新增
`MeArmSim.ctrl_angles_deg()` 把后者也暴露出来（Phase 11 的跟踪误差面板需要分开看这两项）。

---

## D53 · IK 验收必须加载**真实的 `ik.ts`**，不能用 Python 重写一份

**背景**：spec §21 要求「≥100 随机可达点，统计 success rate / mean / max error」。
最省事的做法是在验收脚本里用 numpy 再写一遍平面 2R 解析解。

**为什么不行**：那样只能证明"我又写了一遍、而且它自洽"，**证明不了 `ik.ts` 是对的**。
而 `ik.ts` 恰恰是**真正驱动用户机械臂**的那一份；它的错不会被前端自己的 FK 测试发现
（两者共享同一套旋转约定，会一起错）。要下判断，判据必须来自**另一个实现**。

**做法**：写一个进程级桥 `frontend/tests/tools/kinematics-bridge.mjs`，
用 **Vite 自己的 SSR 加载器**（`server.ssrLoadModule`）把 `ik.ts` / `fk.ts` 拉进 Node。
为什么必须用 Vite：`loadRobotModel.ts` 里写着 `import ... from '@config/robot.yaml?raw'`
（`?raw` 与 `@config` 都是 Vite 专属语法），且所有内部导入都是**无扩展名**的 ——
Node 的解析器两条都不认。
桥用 `--in` / `--out` **文件**传 JSON 而不是 stdout：Vite 与插件会往 stdout 打日志，
任何一行都会让对方解析失败。

**判据结构**（让被测对象只剩一个）：

```
生成：真机限位内随机采样关节角 ──► MuJoCo FK ──► 目标点 T    （两侧同源，不偏袒）
被测：前端真实 ik.ts：T ──► 关节角                          （唯一被检验的对象）
评判：关节角 ──► MuJoCo FK ──► TCP'   误差 = |TCP' − T|
```

**实测**：120 随机 + 256 限位角点，成功率 **376/376 = 100%**，
`max = 1.137e-13 mm`、`mean = 3.76e-14 mm`。
同一个桥顺带把 **前端 `fk.ts` 与 MuJoCo** 也对了（`max = 1.137e-13 mm`）——
`fk.ts` 是真正画 3D 视图、做鼠标拖动的那一份，它若有系统偏差，用户看到的位姿就是错的。

**脚手架自检**：`test_bridge_model_matches_robot_yaml` 先逐项比对桥加载到的模型与
`config/robot.yaml`（关节/轴/限位/耦合/TCP/HOME/执行器）。**这一步不过，后面所有数字都无效** ——
否则会出现"拿 A 配置的 IK 对 B 配置的 MuJoCo"，表面上一切绿灯。

---

## D52 · 「不伪造真实物理」做成**机器可检查**的，而不是 README 里的一句话

**背景**：spec §37 禁止把参数化仿真说成真实模型，要求 README 显式声明 Level。
但"写在文档里"的声明会随重构漂移 —— 换个人来改，很容易顺手把
`calibrated: false` 改成 `true`。

**做法**：把声明变成**测试**（`test_level_declaration_is_machine_checkable`）：

1. `config/physics.yaml` 的 `calibration.calibrated` 必须为 `false`；
2. 七个可标定量段（`servo_offset` / `joint_scale` / `joint_zero` / `max_velocity` /
   `max_torque` / `damping` / `friction`）**必须全为空字典**
   —— 有实测值却仍标 `false` 是自相矛盾的；
3. `calibrate.collect_status()` 报出的待标定项数必须等于可标定项数。

**同时钉住「真值只有一份」**：`test_config_truth_is_not_duplicated` 检查
`physics.yaml` 里既没有 `robot`/`links`/`joints`/`tcp` 这些运动学段，
也没有出现 `elbow`/`shoulder` 限位的字面量。抄一份不会报错，只会静默漂移。

**当前 Level = 3→4（参数化物理仿真）**。要升到 5，必须先把七个量用真实实验填满、
再把 `calibrated` 改成 `true` —— 测试会强制这个顺序。

---

## D51 · 「接触表里有记录」和 `dist` 都**不是**接触力的证据

**背景**：下压位形（`shoulder=49.4549°, elbow=141.8582°`）实测
`jaw_link_coll ↔ table_top` 的 `dist = +1.44 mm`（**正值**，按常理"没接触"），
但 TCP 被稳稳托在 44.05 mm 高处不下沉 —— 到底有没有力？

**排查过程**（每一步都排除了一个假设）：

| 假设 | 实测 | 结论 |
|------|------|------|
| 是 `margin` 造成的"预备接触" | 把 `margin` 从 1mm 改成 0，结果一字不变 | ✗ 排除 |
| 根本没接触，只是位置环的稳态误差 | `qfrc_constraint = [0, −0.23325, −0.145184, 0]` —— 肩/肘上有**真实反力矩** | ✓ 有力 |
| 力矩是执行器硬顶出来的 | `qfrc_actuator[shoulder] = 0.1765` = **满力矩上限** | ✓ 有力 |
| 托举只是巧合 | 禁用台面 ⇒ TCP z 从 44.05 掉到 **15.79 mm**（Δz = −28.26 mm） | ✓ 有力 |

**结论**：`dist > 0` 的**软接触**（`solref`/`solimp` 定义的柔性约束）依然会施力；
`dist` 只是软接触平衡态下的**名义间距**，不是"有没有力"的判据。

**纪律（写进 `tests/sim/test_collision.py` 文件头）**：
1. **有接触记录 ≠ 有力** ⇒ 必须看 `qfrc_constraint` / 执行器力 / 对照位移；
2. **"接触没了"也不是证据** —— 必须配一个**只改一件事**的对照实验
   （这里就是"禁用台面 vs 启用台面"，测同一个位形的 TCP z）。

---

## D50 · 几何只能**加载期**改（`MjSpec`）；但碰撞掩码可以运行时改

**背景**：碰撞测试需要一个变体场景（抬高地面、改碰撞体尺寸）。
最自然的写法是直接写 `model.geom_pos[i] = ...`。

**实测**：这样写进 `MjModel` 之后，`mj_forward` **不会**重算静态 geom 的
`data.geom_xpos` 与 broadphase AABB —— 碰撞检测**完全无视**这次改动。
把工作台从 `z=15mm` 抬到 `z=100mm`，臂的稳态位置与接触对**一字不变**（假绿灯）。

**修法**：给 `MeArmSim` 加一个"直接喂 XML 文本"的入口，测试里用 `mujoco.MjSpec`
在**加载期**改几何：

```python
spec = mujoco.MjSpec.from_file(str(SIM_DIR / "mearm.xml"))
spec.geom("floor").pos = [0, 0, 0.030]
sim = MeArmSim(xml_text=spec.to_xml())      # 实测 geom_xpos=[0,0,0.03]，接触如期出现
```

**但**：`geom_contype` / `geom_conaffinity` **运行时改是有效的** ——
碰撞过滤是逐对查询掩码，不走 AABB 缓存。所以"开关某个碰撞体"用掩码改，
"改某个碰撞体的形状/位置"必须走 `MjSpec`。两者能力不同，别混用。

---

## D49 · `elbow` 的合法域是**斜的** ⇒ MuJoCo 的 hinge range 只承担"数值保护"

**背景**：`elbow` 存的是**绝对倾角**（[108.4415°, 141.8582°]），
而它与 `shoulder` 通过平行四连杆**耦合**（`gain = −1`）。
MuJoCo 的 hinge `qpos` 是**局部角** `θe − θs`。

**算术结论**（`tests/sim/test_joint_limits.py::test_arithmetic_legal_region_is_oblique`）：

- 局部角的外接区间 = `[108.4415 − 49.4549, 141.8582 + 6.0937] = [58.9866, 147.9519]`
- 取该区间**下界**配肩角**下界**：`θe = 58.9866 + (−6.0937) = 52.89°` < 108.4415° ✗ **越界**
- 反过来求"内切"：下界 114.5352 > 上界 92.4033 ⇒ **空集**

⇒ 外接盒**严格大于**合法域，而单一 `range` 只能表达盒。**没有任何 range 能表达它。**

**实测把这条路走实**：MuJoCo 会**照常执行** `(shoulder=−6°, elbow=53°)` ——
它只看到局部角 59° 落在外接区间内。而真机 S8 舵机结构上转不到那个绝对角。

**因此定下架构**：

| 层 | 的角色 |
|----|--------|
| `config/robot.yaml` | **唯一限位真值** |
| `physics.yaml: range_padding_deg = 2.0` | 刻意让 hinge range 比真值更**宽** ⇒ 上层拒绝可被观测 |
| Go controller / `limits.py` / `ik.ts` | **限位一致性的唯一把关人** |
| MuJoCo hinge range | 只做**数值保护**，不是限位真值 |

`test_physical_range_includes_padding` 断言物理上限超过真机限位 + padding/2 ——
如果 padding 被改成 0，物理层会先钳住命令，我们就再也分不清"是上层拒了"还是"物理层钳了"。

**同一族的两条推论**（都由配置算术得出，各有回归测试）：

1. **`elbow-down` 支恒不可行**：`θe = θs + α` 且 `elbow.limit_min (108.44°) > shoulder.limit_max (49.45°)`
   ⇒ α 恒 > 58.99° > 0。所以本机只有 `elbow-up` 一支存在。
   `test_requesting_elbow_down_falls_back_to_the_only_feasible_branch` 还要求
   显式请求 `elbow-down` 时**退回可行支**，绝不静默返回越界解。
2. **可达工作空间的内锥是空的**：`dr = off_r + l1·sin θs + l2·sin θe` 对 θs 单调递增、
   对 θe 单调递减 ⇒ 最小值在角点 `(θs_min, θe_max)`。
   ~~实测 **65.6208 mm > 0**~~ → **已由 D70 重测为 80.9164 mm > 0**（`off_r` 是 D70 引入的
   腕枢轴→TCP 常量径向偏移 40mm；结论方向不变，数值变了）。
   ⇒ 真机**够不到自己的中轴线**（"离底盘越近越容易够到"是错的）。
   闭式与 MuJoCo 网格扫描同值（`test_yaw_axis_is_outside_the_reachable_workspace`）。

**四对限位恰好等价**：`S9 [30,150]` · `S7 [80,160]` · `S8 [20,100]` · `S6 [40,130]`
—— 关节限位换算到舵机后**无一越界**。所以真配置下**构造不出**"关节入限但舵机出限"
的反例（`test_joint_and_servo_limits_are_equivalent`）。

---

## D48 · MuJoCo 作为 `device.Device` 的**第三个实现**接入（不改现有链路一行）

**背景**：spec §2 要求「MuJoCo 不得直接侵入现有 Web UI」；§25 要求「不要重新设计协议」。
同时项目已经有一套成熟的链路：浏览器 → WS → controller → `device.Device`。

**决策**：把 MuJoCo 做成 `device.Device` 的**第三个实现**，与 `sim.go` / `serial.go` 并列。

理由：`device.Device` 是一个 **7 方法的极小接口**
（`Kind` / `Connected` / `UnavailableReason` / `WriteLine` / `Lines` / `OnStatus` / `Close`），
它恰好就是"物理后端"的抽象边界。于是：

- WebSocket 层、`controller`、`protocol`、前端 **全部零改动**（实测实装后一个字没改）；
- Go 侧只是 `main.go` 的 `switch c.Device.Mode` 多一个 `case "mujoco"`；
- Python 侧 `server.py` 在 stdio 上跑**与固件逐字节相同的文本协议**。

**§25 的落地**：`ServerMessage` 只加**一个可选字符串** `simulation_mode`
（`omitempty` ⇒ 老前端读不到就按 `kinematic` 处理），值为
`SimulationModeFor(deviceKind)`：`sim→kinematic` / `mujoco→mujoco` / `serial→real`。
不加新消息类型、不改 `joints` 的结构。这就是"不侵入 Web UI"的具体形态。

**§32/§33 的落地**：`reset()` 是唯一允许写 `qpos` 的地方（`data.qpos[...] = target`
**不是**控制方式）；跨进程确定性由 `run.py --demo` 跑两遍、数值行逐字比对来证明。

**为什么握手要显式 `PING → OK PING`**：没有它，"Python 解释器不对 / mujoco 没装 /
XML 编译失败"这三类最常见的故障会以**"设备可用但永远没有回执"**的形式出现，
上层只能看到 ACK 超时 —— 那是最难查的一种失败。
`exec.LookPath(python)` 失败、`stderr` 里出现 `No module named 'mujoco'` 都会给出明确修复指引。

---

## D47 · 先证「测量可复现」，再谈「模型对不对」—— A1/A2 双双判定为**不改**

**背景**：D38 提出「反解主误差源是平行四连杆耦合增益，须改骨架模型而非调参数」，
D40 复现了 S7 肩 +34% / S8 肘 −50% 的增益残差，目视复核也看到骨架贴在臂的**外轮廓边**
而不是中心线。于是自然推演出的两条待办是：

- **A1**：把 `fit_pose.py` 的三连杆骨架改成**双杆**（平行四连杆的真实形状），消除系统偏置；
- **A2**：把 `config/robot.yaml` 的 `coupling.gain` 从 −1 改成实测值 ≈ **−0.81**。

两条都"看起来有证据"。但它们**都跳过了一个前置问题**：这份测量本身，换个台面还成立吗？

### 一、做法：把"改模型"降级为可判定实验

不直接改 `robot.yaml`，而是新增两件**判定工具**（不是新的测量，只是把已有判据固化成命令）：

| 工具 | 作用 |
|------|------|
| `tools/verify_pose.py --rod-gap MM` + `--selftest` 自检 C | 骨架小臂可按 `rod_gap` 画成平行双杆（`0` = 原单折线，**逐值向后兼容**）。自检 C 用**合成真值**（不依赖任何实拍照片）分别用单杆/双杆去拟合"双杆渲染出来的图"，把「模型表达力」这一项的贡献**单独**量出来 |
| `tools/verify_calib_repro.py` | 打印「批次 × 底座掩膜 × 骨架杆距」的**增益散布矩阵**，并写死判据：**极差 ≤ 5% 才允许据偏差改模型** |

### 二、自检 C 的结论：双杆的"改善"在噪声量级里

| 杆距 | 单杆 Δ肘 | 双杆 Δ肘 | 改善 |
|------|---------|---------|------|
| 4mm | 2.00° | 1.74° | +0.26° |
| 8mm | — | — | **方向不一致**（肩 +0.76° / 肘 −1.97°） |

**无单调趋势，改善量 0.3° 量级** —— 相比 D40 §4 记录的 **±5° 绝对角不确定度**可忽略。
即：**即便假设成立，改双杆也换不来可测量的精度提升。**

### 三、实拍 `--rod-gap` 矩阵：又一次"单批最优 = 过拟合"

| `rod_gap` | w2_S7（base-anywhere） | w2_S8 | dir_S7 |
|-----------|----------------------|-------|--------|
| 0 | −3.7% | +0.9% | −2.3% |
| **4** | **+0.7%**（"修好了"） | **−0.5%**（"修好了"） | — |
| **10** | — | — | **−46.3%（拟合退化）** |

`rod_gap=4` 在两个配置上"正好修好"，杆距稍大到 10mm 就崩到 −46.3%。
**同一组参数既有"修好"也有"崩掉"** —— 与 D37 的 ROI 教训是**同一个形状**。

### 四、最终判定

```
elbow      偏差范围 [+0.9%, +53.8%]  极差 52.9%  (容差 5%) ⇒ 先锁台面，禁止据单点结论改模型
shoulder   偏差范围 [-13.1%, -2.3%]  极差 10.8%  (容差 5%) ⇒ 先锁台面，禁止据单点结论改模型
⚠️ 有 2 个组合的 |偏差| > 40%：拟合在该配置下退化
```

| 项 | 判定 |
|---|---|
| A1 改骨架双杆 | **不改**。`robot.yaml` 不动；双杆能力作为工具保留（默认 `rod_gap=0`） |
| A2 改 `coupling.gain` → −0.81 | **作废**。`gain` 是 `robot.yaml` 的**唯一直值源**，同时影响前端 FK/IK；测量不可复现时任何取值都是猜 |
| 真正短板 | **测量可复现性**（台面 / 相机 / 曝光 / 底座掩膜未锁死）—— 与 D39/D40 一脉相承 |

**A3（锁台面重测）是唯一正确的下一步**，步骤见 `docs/hardware-measurement.md` §7，
判据已固化为 `python tools/verify_calib_repro.py`（极差 ≤ 5% 才解锁改模型）。

> **教训一句话**：*"改模型让它对上"和"调参数让它对上"是同一类错误。*
> 必须先证明**测量对得上**，才有资格谈**模型对不对得上**。
> D37 说的是"别信单批 ROI 最优"，D47 把它升级成了**可执行脚本判据**。

---

## D46 · Phase 13 示教录制 / 回放：回放**必须**走既有命令路径

**背景**：项目此前只能逐次操作（拖滑杆 / 拖末端 / HOME），没有"录一串动作再整体重放"的能力。

### 一、录 `commandJoints`，不录 `actualJoints`

`actual` 是链路回推的滞后值；MG90S 无位置回读，它甚至就是**由目标值反算**的（D34）。
录 actual 等于把链路时延焊进轨迹，回放时再叠加一次时延 —— 越放越偏。

### 二、回放不做"直发通道"，复用 `store.setCommandJoints()`

这是本 ADR 的核心纪律。命令一旦进 store，**尾沿节流（30Hz）与安全门（mode=simulation
不驱动真机）全部自动生效**。若为了"回放"另开一条直发通道，等于**绕过所有安全约束** ——
本项目反复要消灭的正是这类静默危险。e2e 里专门有一条断言从**下游日志**
（`joint_command`）反证回放确实经过了下发路径，而不是只改了 UI 读数。

### 三、录制策略：节流 + 去抖 + 上限拒绝 + 停录补帧

| 机制 | 参数 | 为什么 |
|------|------|--------|
| 采样节流 | `TEACH_MIN_FRAME_INTERVAL_MS = 50`（20Hz） | 拖动时命令每帧都在变，逐帧记录会把轨迹撑爆 |
| 静止去抖 | `TEACH_STILL_EPS_DEG = 0.5` | 与上一帧各轴差都在 0.5° 内 = 无新信息 |
| 上限 | `TEACH_MAX_FRAMES = 2000`（≈100s） | **拒绝新帧，而不是丢旧帧** —— 悄悄丢掉开头会让回放缺少起手，比"录不进去"更难发现 |
| 停录补帧 | `appendFrame(..., { force: true })` | 否则末帧可能停在被去抖吃掉的旧值上，**轨迹终点 ≠ 臂当前所在位置**，回放结束时会莫名回退一点 |

跳过时 `appendFrame` 返回**同一个引用** ⇒ store `set` 之后 React 直接 bail out，
订阅者不会被无谓唤醒（有单测钉住）。

### 四、回放必须**插值**，且终态必须**精确取末帧**

- `sampleAt()` 在关节空间线性插值：逐帧原样抛出的话，两个采样点之间命令是**阶跃**的，
  真机会一跳一跳；插值让每一拍都是连续值，回放时长也不被采样率量化。
- 结束时刻**不走插值**，直接取末帧：这样「回放终点 == 录制终点」是**逐值成立**的不变量。
  e2e 用 `1e-9` 容差断言它，并同时断言终点与"回放前故意挪开的位置"明显不同 ——
  否则"回放到位"是恒真的假断言。

### 五、面板不持局部副本

`teachTrack` / `teachRecording` 放 **store**（本文件 D41 同源的纪律：机器人相关状态必须经
RobotState 体系）。理由有一条很实际：e2e 探针要读到**末帧原始数值**做 1e-9 判定；
若留在组件里，探针只能读 DOM 上的 1 位小数文本。

### 六、验收

| 项 | 结果 |
|---|---|
| tsc | **0 error** |
| vitest | **293/293**（256 → 293：teachTrack +20、teachPlayback +17） |
| `vite build` | ✓ built in 990ms；产物 `__armPilot` 命中 **0** |
| e2e | **88/88 PASS**（+19 条 Phase 13） |

e2e 关键实测：
```
[PASS] Phase 13：轨迹记录了多帧且时长 > 0  — 4 帧 / 436 ms
[PASS] Phase 13：回放期间不产生新帧（回放不录自己）  — 4 vs 4
[PASS] Phase 13：回放终点**逐值**等于录制末帧（不留插值残差）
       — cmd {"base":0,"shoulder":45.9063172659,…} / rec {…同上…}
[PASS] Phase 13：回放经既有命令路径下发（日志可见 joint_command，未绕过节流与安全门）
```

---

## D45 · Phase 12 实际臂幽灵：e2e 必须取**渲染后矩阵**，不能拿 FK 当证据

**背景**：Phase 12 让虚拟场景同时渲染**两条臂** —— 主臂跟随 `commandJoints`（意图）、
半透明幽灵跟随 `actualJoints`（现状）。未被遮挡时露出的部分就是**滞后量**。

### 一、为什么幽灵用半透明而不是醒目色

项目 UI 约定「无装饰色」（`styles.css` 头部），且颜色不是数据。信号是**位置分离**本身。
`depthWrite = false` 是必需的 —— 否则半透明面互相遮挡会出脏面。

### 二、探针为什么必须取 `matrixWorld`

`ActualGhostArm` 把 `objects.tcpMarker.matrixWorld` 暴露到 `window.__armPilotGhost`。
e2e 一边是 **three.js 对象图算出的世界矩阵**，一边是 store 里**纯数学 FK**
（`actualEndEffector`）—— 两条代码路径独立，所以这不是自证。

**若改用 `FK(actualJoints)` 当证据，只能证明数学，证不了渲染。**
渲染树挂错父节点、可见性被误关、材质全透明 —— 这三种情况下画面全空而断言全绿。

### 三、两个渲染进度的探针必须 `import.meta.env.DEV` 守卫

连 `delete holder.__armPilotGhost` 这一行也要守卫，否则字面量残留进生产包，
触发"产物不得含探针字样"的验收检查（与 `TestProbe` 同一条纪律，D27）。

### 四、e2e 时序：相关读数必须合并在**同一次** `cdp.evaluate`

Mock 的 34° 跳变约 **150ms** 就收敛完。分两次 CDP 往返（十几毫秒/次）会让第二次读到
归零值 ⇒ 断言变成**间歇性失败**而非稳定复现。这是 `READ_LAG_WITH_GHOST` 合并求值的原因。

### 五、验收

tsc 0 · vitest 256/256（actualGhost +4）· build ✓ · e2e 65/65（+6）：
```
[PASS] Phase 12：滞后瞬间实际臂幽灵已渲染并与主臂分离（分离量=滞后量）  — 分离 47.1 mm / 误差 -34.26°
[PASS] Phase 12：幽灵逐值跟随 actualEndEffector（非 command）—— 渲染路径与纯数学 FK 互证  — 0
[PASS] Phase 12：关闭 Actual Arm 后幽灵确实停止渲染  — null
```
`actualGhost.test.ts` 里那条"幽灵 TCP == FK(ACTUAL)、主臂 TCP == FK(COMMAND)、两者互不等
且分离 > 10mm"是**唯一**能抓住"把源接反"的断言。

---

## D44 · Phase 11 误差面板：**不能**用 `TransportStats.moving` 判"卡死"

**背景**：Phase 11 把"Actual 追 Command"的偏差可视化。`StatusPanel` 早已把
Command / Actual / Error 三列数值列全了 —— 但**光有数值看不出时序**。
真机调试时最常问的是"这是**还在追**，还是**卡住了**？"这需要趋势与时间，不是某一帧的数字。

### 一、`moving` 是 lag 的同义重写，永远推不出"卡死"

实测 `WebSocketTransport.stats()` 里 `moving = lagDeg > arrivedEps()` —— 它**由 lag 推导**。
拿它判"是否在追"等于把 `lag > tol` 重写一遍，`stalled` 这个结论**永远不可能出现**。
判据只能自己从**时间序列**里得出。

### 二、趋势判定用**四分位中位数**，不用首末值 / 均值

`classifyTrend` 取窗口前 1/4 与后 1/4 的**中位数**比较。
实现过程中先用均值，被"尾部单点尖峰"测试打回：回推序列混有掉帧与量化台阶，
**个别异常帧就能把结论整个翻面**；而"误报卡死"会让人去查并不存在的问题。

### 三、纪律：没有正面证据就不下"卡死"断言

只有「命令已静止 ≥ `STALL_HOLD_MS`(700ms)」**且**「趋势不再下降（flat / growing）」
才判 `stalled`。`unknown`（样本不足）/ `shrinking`（还在收敛）一律 `tracking` ——
宁可提示"还在追"，也不误报异常。

**命令一变就清空历史**（`nextHistory`）：跨命令的趋势没有意义 ——
命令一改，误差从 0 跳到很大，`growing` 是必然的，会被误判成卡死。

### 四、能力边界（面板里显式声明）

MG90S 无位置回读，`Actual` 是由目标值反算的（D34）。因此在 sim / 现有固件下，
面板反映的是**链路时延与限位截断**，**不是**"物理臂真的到了没有" ——
后者只能靠相机反解（Phase 4.5 工具链）。固件一旦具备回读，本面板无需改动。

### 五、验收

tsc 0 · vitest 252/252（linkFeedback +19）· build ✓ · e2e 59/59（+7）：
```
[PASS] Phase 11：滞后期间健康结论**不是**"已到位"  — 跟踪中（滞后 34.26°，命令到位前属正常）
[PASS] Phase 11：收敛后健康结论变为"已到位"  — 已到位（最大偏差 0.00°）
[PASS] Phase 11：断开后健康结论回到"未接入传输"  — 未接入传输 · Actual ≡ Command，误差恒为 0
```

---

## D43 · **`setMode('real')` 的准入校验必须"拒绝"，不能只发日志** —— D41 修了一半

**背景**：用户报障「前端显示 Real 模式后端末端是 sim，非真机」。

排查结果：**这不是连接问题，也不是 `start.bat` 的问题**。用户没加 `--real`，
后端跑 `config.yaml`（`device: sim`）是**完全正确**的默认行为。
错的是前端——它一边把按钮切成 Real、一边显示"末端是 sim"，
把一个"本该拒绝的操作"呈现成了"已完成但有警告"。

### 一、根因（代码层铁证）

`robotStore.setMode` 把状态写入放在了准入校验**之前**：

```ts
setMode(mode) {
  set({ mode });          // ← 无条件先写！D41 留下的
  if (mode !== 'real') { …; return; }
  // ---- 以下才是准入校验 ----
  if (device !== null && device !== 'serial') {
    get().pushLog('err', `…末端是「${device}」而非 serial…`);
    return;               // ← 只 return，可 mode 已经是 'real' 了
  }
}
```

于是「点了 Real Robot」的**唯一实际效果**是：按钮高亮切到 Real + 日志里多一条 err。
UI 结论与事实相反，这正是本项目反复要消灭的那类**静默失败**——
只不过它伪装成了"有警告"，看起来像已处理。

**更糟的是测试把这个 bug 固化成了契约**（`mode-transport-link.test.ts` 旧版）：

```ts
// 模式本身仍然切换（用户意图被记录），但告警必须出现
expect(useRobotStore.getState().mode).toBe('real');   // ← 给 bug 背书
```

`expect(mode).toBe('real')` 让这个错误行为成了"受保护的设计"，
所以 D41 之后没人发现它还错着。**测试锁定了错误，比没有测试更危险。**

### 二、修法（三层，缺一不可）

| # | 位置 | 变化 |
|---|------|------|
| 1 | `robotStore.setMode` | 校验**全部通过**才 `set({ mode: 'real' })`；任何一条不满足则**保持 simulation** 并 pushLog 说明原因与修法 |
| 2 | `robotStore.setMode` | 新增 `device === null`（hello 未到）分支：**拒绝**。旧版在此"乐观放行"，是 `--real` 自动切能"成功"的真凶 |
| 3 | `useAutoConnect` | 抽 `decideAutoRealSwitch()` 纯函数；订阅 **connection + transportStats** 双变化，等 `device` 到达再切（否则会撞上第 2 条的拒绝） |
| 4 | `JointControl` | 两条 `mode==='real'` 但末端不对的提示改为"⚠ 状态异常（应由 setMode 拒绝）"—— 降级为**纵深防御**，正常路径已不可达 |

### 三、关键设计决策

**为什么"拒绝"而不是"切换 + 醒目警告"**

用户明确拍板：拒绝切换，按钮保持 Simulation 高亮。理由是所见即所是 ——
`mode` 的语义是"**要不要发给真实机械臂**"（本文件 D41 §一），
它不是一个可以"记录意图"的 UI 开关。一个不能反映现实的 `mode` 只会继续误导。

**拒绝的代价必须由日志补上**。点按钮"没反应"体验很差，所以每条拒绝都要说清
**为什么**（末端是 sim / 未连接 / hello 未到）和**怎么修**（用 config.serial.yaml 启动）。
拒绝不是目的，让用户知道当前到底在驱动谁才是。

**`device === null` 从"乐观放行"改为"拒绝"** —— 这条最容易漏

旧逻辑的注释写着「宁可少拦，不要误判仿真」。**在安全门（`flush`）场景下这是对的**
（少拦一条命令 ≠ 危险），但在 **UI 切换**场景下它是错的：`--real` 自动切常常抢在
hello 之前执行，于是切"成功"了、末端实为 sim。两个场景的取向必须分开——

| 场景 | 未知 device 时 | 理由 |
|------|---------------|------|
| 安全门 `flush`（D41） | **放行** | 少拦一条，无害 |
| UI 切换 `setMode`（D43） | **拒绝** | 假"已切"会误导操作，有害 |

### 四、连带发现：跨批次状态残留（第 6 例）

e2e 首轮出现 `[FAIL] 状态表 Command 列跟随滑杆（≈40°） — 0 / 0`。
根因不是代码：5273 上跑着**注入了 `VITE_AUTO_CONNECT=ws` 的旧 vite dev 实例**
（本会话早些时候起的），页面自动连了后端，`readCommandCell` 读到被回推干扰的值。
换**干净 vite dev** 后同一条断言 `39.9°` PASS。

⇒ 与 D40 §四、D41 §四同源：**跑 e2e 前必须清掉带 env 的残留 dev server**，
否则"环境差异"会被读成"代码回归"。

### 五、验收

| 项 | 结果 |
|---|---|
| tsc | **0 error** |
| vitest | **233/233**（226 → 233：autoConnect +5、mode-transport-link +2） |
| `vite build` | ✓ built in 671ms |
| e2e | **52/52 PASS**（46 → 52，新增 Phase 8 (h2) 2 条 + Phase 10.6 4 条） |

**直接复现用户报障场景的断言**（e2e Phase 8 (h2)，链路**连着 sim 后端**时点 Real Robot）：

```
[PASS] 连着后端（末端非 serial）点 Real Robot → 拒绝切换，按钮仍在 Simulation
       — simActive=true realActive=false
[PASS] 拒绝原因必须点名链路末端
       — ERR Real Robot 未启用：后端链路末端是「sim」而非 serial，保持 Simulation。
         请用 config.serial.yaml 启动后端（并把机械臂接到配置的串口）
```

---

## D42 · 一键启动脚本：页面默认停在 Mock ⇒ 必须由**环境变量**驱动自动连接

**背景**：用户提出「实现一键启动脚本」，随后立刻指出真正的问题——
「一键启动后未操作真实设备，而是 sim 数据，改为 ws 模式连接真实硬件」。

这说明脚本起好了前后端**并不等于**接上了硬件。根因在网页侧：

### 一、缺口：启动后停在 Mock，且不会自动连接

`ConnectionControl.tsx` 里：

```ts
const [mode, setMode] = useState<TransportChoice>('mock');   // ← 默认 Mock
```

且**全仓没有任何自动连接逻辑**（`App` / `main.tsx` / `ConnectionControl` 都没有
相关 `useEffect`）。于是：

| 步骤 | 实际发生 |
|------|---------|
| `start.bat` 起后端（甚至 `-c config.serial.yaml`） | ✅ 后端就绪、串口已连 |
| 页面打开 | 停在 MockTransport |
| 用户拖动滑杆 | **只动浏览器内的仿真臂**，真机纹丝不动 |
| UI 表现 | 一切正常（读数在变），**看不出哪里不对** |

这正是 D41 同类的"静默失败"：链路在、后端在，但**命令根本没走出去**。

### 二、决策：用环境变量区分「一键启动」与「手动开发」

候选方案与取舍：

| 方案 | 问题 |
|------|------|
| 前端默认改成 `websocket` | `npm run dev` 的手动调试也被改（想试 Mock 得先断开）；生产构建会无条件连一个可能不存在的后端 |
| 页面永远自动连 WS | 同上，且手机等局域网访问会意外连到别人的后端 |
| **环境变量驱动（采纳）** | `start.bat` 注入 → 一键启动自动连；`npm run dev` 不注入 → 行为完全不变 |

注入的两个变量（`frontend/src/hooks/useAutoConnect.ts`）：

| 变量 | SIM | REAL | 语义 |
|------|-----|------|------|
| `VITE_AUTO_CONNECT` | `ws` | `ws` | 挂载后自动连 WebSocket（替代浏览器内 Mock） |
| `VITE_AUTO_REAL` | 不设 | `1` | 连接成功后自动切 Real Robot |

> `VITE_WS_URL` **刻意不设**：让前端按 `window.location.hostname` 推导，
> 局域网设备访问 `http://<本机IP>:5273` 时才能连到正确的后端（D40 已确立的原则）。

### 三、`VITE_AUTO_REAL` 必须**带校验**，不得无条件切

自动切 Real Robot 若不加判断，会把 D41 的准入校验绕过去。故实现为：

1. 调用 `store.setMode('real')` —— **复用** D41 的三层校验（未连接 / 连 Mock / 末端非 serial）
2. 若 `setMode` 后 `mode` 仍是 `'simulation'`（说明校验拒绝），再补一条 `err` 日志说明原因
3. **不阻断**：命令仍下发给当前链路

即"**告警但不阻断**"。理由：`--real` 但后端实际跑在 sim（忘了插硬件 / 串口号写错）
是常见的排查场景，硬拦下发会让人看不到"连上了但不发"的中间状态；
而只要提示条明确显示实际末端，"以为在驱动机器其实没有"这个风险就已消除。

实测（无硬件、`start.bat --real`）：

```
backend: device=serial
AUTO-CONNECT badge = Connected
mode-routing = Real 模式下后端末端是「未知」，非真机
SYS  Real Robot 已启用：等待后端 hello 确认链路末端…
SYS  Real Robot 未生效：…（链路末端不可用）
```

—— 自动连上了、自动尝试切了、切不动的原因**明写在日志与提示条里**。

### 四、`start.bat` 的两个 .bat 陷阱（本次实测踩到）

1. **`if (...)` 块内的裸 `>` 是重定向，会掐死整个块**
   写 `echo  step 1. Connection -> WebSocket` 时，cmd 在**解析期**就把 `>` 当重定向，
   该行文本丢失、块内后续语句被跳过、并报「此时不应有 to。」，
   更糟的是**生成了名为 `WebSocket` / `inside` 的垃圾文件**（实测已误建并清理）。
   ⇒ 所有作为文本打印的 `>` 必须写成 `^>`；`(` `)` 同理写 `^(` `^)`。

2. **`start "标题" cmd /k "cd /d "X" && prog"` 引号嵌套是坏的**
   cmd 剥掉外层引号后把首个 token 当目录，然后试图执行一个名为 `then` 的命令
   （报「'then' 不是内部或外部命令」）。
   ⇒ 改用 `start "标题" /d "目录" "程序" -c "参数"`（`/d` 交由 start 处理，
   不自己拼 `cd &&`）。

### 五、e2e 侧顺带修掉的两个**跨批次状态残留**（同一类问题第三次出现）

| 断言 | 残留原因 | 修法 |
|------|---------|------|
| 重连后补发当前命令 | 写死期望值 `30`，而命令值由 (c) 段按当前位姿自适应挑选，实测落在 20.9° | 改为**取重连前实测到的命令值**作期望 |
| 后端初始状态 = HOME 位 | 复用 8090 上的既有实例（上一轮的 `shoulder=20.9`），却按"全新启动"去比 HOME | 排除 `backend.external`，复用实例只校验"四轴为有限数" |

> 规律已稳定：**凡断言依赖"上一轮终态"的地方，都必须显式设定入口状态或放弃该假设。**
> 与 D40 §四、D41 §四同源。

**验收**：tsc 0 error · vitest **226/226**（214 + 12 个新增意图解析单测）·
`vite build` ✓ · e2e **50/50**（连跑三轮稳定）。

## D41 · **`mode` 曾是"死状态"**：点 Real Robot 只改样式、命令照样走当前 transport

**背景**：用户反馈「real 真机执行时未运行」。排查发现这不是连接问题，
而是 **`mode` 从未接入命令路径** —— 一个"看起来成功了"的静默失败。

### 一、缺口（代码层铁证）

`robotStore` 头部注释写着：

> `mode`（simulation / real）… **前者决定"要不要发给真实机械臂"**

但实际用法**全部是 UI 渲染**：

| 位置 | 用途 |
|------|------|
| `JointControl.tsx` | 按钮 `active` 样式 |
| `StatusPanel.tsx` | 文案"Real（真实机械臂）"/"Virtual（仿真）" |
| `ViewportOverlay.tsx` | 角标 |
| `robotStore.setMode` | `set({ mode })` + 一条日志 |

而命令下发的**唯一判据**在 `transportBridge`：

```ts
this.unsubscribeStore = useRobotStore.subscribe((state, prev) => {
  if (state.commandJoints !== prev.commandJoints) this.enqueue(state.commandJoints);
  //  ↑ 只比 commandJoints，mode 完全没参与
});
```

**后果两条，都很糟**：

| 场景 | 期望 | 实际 |
|------|------|------|
| 点 Real Robot，但没连后端 | 提示去连接 | 命令发给 Mock，**真机不动**，UI 却显示"真实机械臂" |
| 切回 Simulation，但连着真机 | 真机不动 | 命令**照常下发给真机**，用户以为在仿真 |

### 二、修法（三层，缺一不可）

1. **`setMode('real')` 变成带校验的意图**（`robotStore.setMode`）
   按链路状态分别告警，四种情况都说清楚：
   - 未连接 → "未连接任何传输，请先连接后端"
   - 连 Mock → "当前是浏览器内仿真，不碰硬件"
   - 后端末端 `device !== 'serial'` → "末端是「sim」而非 serial"
   - 末端 serial → "Real Robot 已启用"

2. **安全门**（`transportBridge.flush`，唯一的实际发送点）
   `mode === 'simulation'` 且链路是**真机**（`kind==='websocket'` **且** `device==='serial'`）
   ⇒ **拒绝下发**，并累积 `blockedSince` 在日志里明示"已拦截 N 条"。

3. **UI 显式化**（`JointControl` 的 `mode-routing` 提示条）
   把"命令到底发给谁"写在按钮旁边，三态：纯仿真 / 已拦截 / 正在驱动真机。
   **不留静默状态** —— 这正是本次缺口的本质。

### 三、关键设计决策：安全门只拦"真机链路"

**不能**简单地"`mode==='simulation'` 就不下发"，那会打死 Phase 7/8 的仿真闭环
（Mock 与 `device=sim` 的后端本就是仿真，必须照常放行）。

判据收紧为**两条同时成立**：`transport.kind === 'websocket'` **且** `device === 'serial'`。

**`device` 未知（尚未 hello）时按 `false` 处理 —— 宁可少拦，不要误判。**
理由：握手前把正常仿真当成真机拒绝，会让用户看到"滑杆没反应"却毫无线索，
比"多下发一条"更糟。此条已有专门单测锁定。

### 四、连带发现的第三例"跨批次状态残留"（e2e）

新增联动断言后 e2e 出现 `Actual 收敛` FAIL（`cmd 20.9° / act 0.8°`）。
根因不是代码错，而是：**上一轮结束时页面停在 Simulation**，
于是 (c) 段设的目标被安全门正确拦下 —— 断言观察不到变化。

⇒ 修法与 D40 §四同类：**(c) 段前显式切到 Real Robot**、**(h) 段末尾再切回**，
不依赖上一轮终态。**凡是依赖 mode/连接状态的分段，都要显式设定入口与出口状态。**

**验收**：tsc 0 error · vitest **214/214**（新增 10 项联动验收）·
vite build ✓ · e2e **49/49**（连跑三次稳定）。

---

## D40 · **「手入镜」是新的头号污染源**；重复性判据按 D39 设计成功拦截；局域网访问须按来源推导 ws 地址

**背景**：2026-09-12 20:42 按 D39 前置（锁相机 + 固定线缆）重跑真机验收。
D39 的修复**得到决定性验证**，同时暴露出**第四个、与前三个完全独立的根因**。

### 一、D39 修复生效（直接对照）

| 指标 | 20:31 批（相机漂 6px） | **20:42 批（线缆已固定）** |
|------|----------------------|--------------------------|
| 拟合尺度 `s` | 3.670 px/mm（−7.2%） | **3.969 px/mm** ✅ |
| 肩枢轴 | 漂移，锚点偏 −4.12° | **(548.7, 520.2)**，锚点偏 −1.62° ✅ |
| 重复性（肩） | 2.81° ❌ | **0.83°** ✅ |
| 反解 PASS | 1/7 | **3/7** |

**新增采集前置探针** `.workbuddy/analysis/_cam_stability.py`（10 帧静止场景）：

```
逐帧 vs 首帧：dx=0 dy=0（全部 10 帧）
相邻帧：      dx=0 dy=0（全部 9 对）
整图中位：    155.0（波动 0.00）  整图均值波动 0.10
```

⇒ **位移恒为 0**。这给了"可否开跑"一个**开跑前的量化闸门**，
不必等跑完 7 步才发现数据全废。

### 二、新根因：**手入镜**（与相机/固件/标定全无关）

`06_reset2` 帧异常，三个信号同时出现：

| 信号 | 正常帧 | `06_reset2` |
|------|--------|------------|
| JPEG 体积 | 112290 B | **114952 B**（+2.7%） |
| 反解残差 | 28~42 px | **69.76 px** |
| 肘帧间差 | ≤0.27° | **−3.81°** |

**目视确认：画面右上有一只手（正在调线缆）。** 手的像素被分割器当成臂的一部分，
把联合拟合拉到错误解 → 肘被反解成 +110.88°（比 HOME 低 3.81°）。

**这不是臂的误差，是画面里有个人。**

**决策**：
1. **采集期间人员离场**：与「锁曝光 / 锁相机 / 锁线缆」并列为第四条前置硬要求。
   手、反光物、可动背景一律不得进入 ROI。
2. **残差是廉价的脏数据探针**：`verify_pose.py` 已逐帧打印残差，**残差异常放大
   （>1.5× 中位）就该目视该帧**，而不是直接读角度。比 JPEG 体积更直接。
3. **重复性判据按设计成功拦截**：这次 FAIL 的是"重复性"，而它正是 D39 §2 定的
   健康检查。**判据抓到了污染源，而不是被污染误导** —— 说明 D39 那条决策是对的。

### 三、局域网访问（本轮附带落地）

实测两处缺口，均已修复：

| 缺口 | 症状 | 修复 |
|------|------|------|
| Vite 未设 `server.host` | 只绑 `[::1]`，局域网打不开 | `host: true` → `0.0.0.0` + `[::]` |
| ws 地址硬编码 `localhost` | **页面能开但一直重连** | 按 `location.hostname` 推导 |

第 2 条是**最容易误判的坑**：局域网设备打开 `http://<本机IP>:5273` 时，
浏览器里的 `localhost` 指**访问者自己那台设备**，于是去连自己的 8090（不存在）。
表现是"页面正常、连接不上"，很容易误判成后端/固件故障。

**实测证据**（headless Edge + CDP，`ws_probe.mjs` / `lan_e2e_probe.mjs`）：

| 访问来源 | 推导出的 ws | 结果 |
|---------|------------|------|
| `http://localhost:5273/` | `ws://localhost:8090/ws/joint` | ✅ |
| `http://192.168.3.5:5273/` | `ws://192.168.3.5:8090/ws/joint` | ✅ 收到 hello + joint_state |

**保留 `VITE_WS_URL` 覆盖**：需要指向别的后端时仍可显式指定，优先级高于推导。

**后果**
- ✅ D39「重复性当健康检查」被实践验证有效 —— 它正确拦下了一批脏数据
- ✅ 采集前置从三条（锁曝光/锁相机/锁线缆）升为**四条**（+ 人员离场）
- ✅ 局域网可用；`host: true` + 来源推导 + 防火墙三条齐备（本机已有
  `node.exe` / `armpilot-backend` 入站规则，生效于**公用**配置档）
- ⚠️ 反面教材：若只看"肘偏 3.81°"就去调标定/固件，会完全错过"画面里有只手"

---

## D39 · **相机整体位移 6px = 反解退化的头号杀手**（须锁相机 + 固定线缆）

**背景**：2026-09-12 20:31 复跑真机验收，结果比 19:37 那批**明显退化**：

| 指标 | 19:37 批 | 20:31 批 |
|------|---------|---------|
| 反解 PASS | 4/7 | **1/7** |
| 锚点偏置（肩/肘） | −2.20 / +0.44 | **−4.12 / −2.64** |
| 重复性（同位姿两帧） | 0.89° | **2.81°** ❌ |
| 拟合尺度 `s` | 3.955 px/mm | **3.670 px/mm**（−7.2%） |

**排查（逐层排除）**：

1. **不是曝光**：两批整图中位 155 vs 156、背景中位 175 vs 175、过曝 0.00% vs 0.02%
   —— 完全一致。
2. **不是机械臂没回位**：`00_reset` 与 `06_reset2` 两帧肉眼**几乎完全一样**，
   但像素上 40948 个点差 >60，且差异图**画出完整臂轮廓**（不是局部形状变化）。
3. **是画面整体平移**：互相关（±6px 搜索）给出决定性证据：

   | 对比 | 最优整体位移 | 位移前残差 | 位移后残差 |
   |------|-------------|-----------|-----------|
   | 同批 `00_reset` vs `06_reset2` | **dx = −6 px** | 19.58 | **11.66** |
   | 本批 vs 上批 `00_reset` | **dx = −6, dy = −3 px** | — | 10.73 |

   整体平移后残差显著下降 ⇒ **确实是刚体平移**，不是臂形变。

**根因**：**相机本身挪动了**。最可能是 **USB 线缆被机械臂运动拖拽**带偏了摄像头角度
（照片里那条 USB 线正从画面右侧穿过，且末段就在臂的运动路径上），或首帧拍在相机未稳定的时刻。

**后果链（为什么 6px 能毁掉一切）**：坐标系整体平移
⇒ 肩枢轴 `(ox,oy)` 定位偏 ⇒ 拟合尺度 `s` 偏 −7.2%
⇒ 锚点偏置算成 −4.12°/−2.64°（**纯属假偏置**）
⇒ 后续每帧的绝对角、增益复核（肩 +27.4% / 肘 +25.2%）全部被污染
⇒ 重复性从 0.89° 恶化到 2.81°（两帧间隔了 7 步运动，相机已漂）。

**决策**：
1. **验收前必须固定相机**：三脚架/夹具锁死，且**把 USB 线缆固定住**（远离臂的运动路径，
   或留出足够松弛量）。**线缆拖拽是相机位移的最常见来源。**
2. **验收后自检**：把「重复性」当**硬性健康检查**读 —— 若同位姿两帧差 >1.5°，
   本批数据**一律作废**（相机漂了），不要去解释反解结果。
3. **`--drift` 只能补偿角度耦合，修不了平移**：平移是**外参**变化，必须在采集端解决。
4. 与 D36（锁曝光）并列：**锁曝光 + 锁相机 + 锁线缆** 是三条并列的采集前置硬要求。

**后果**
- ✅ 把「本批为什么全 FAIL」从"模型问题"正确归因到"采集问题"，**避免了去改本已正确的模型**
- ✅ 给出了可执行的健康检查（重复性 >1.5° 即作废）
- ⚠️ 反面教材：若不查互相关，很容易把 −4.12° 锚点偏置当成"臂没回位"→ 去调固件/舵机，
  与真因（相机位移）南辕北辙

---

## D38 · 反解主误差源是**平行四连杆耦合增益**，须改骨架模型而非调参数

**背景**：D37 排除 ROI 之后，问题落到「为什么 `02_sh_p15` / `04_el_p15` / `05_combo`
反复 FAIL」。`verify_pose.py` 的标定增益复核给出直接证据：

| 通道 | 实测 Δ关节/Δ舵机 | yaml `1/scale` | 偏差 |
|------|-----------------|---------------|------|
| S7 肩 | +0.93 | +0.69 | **+34%** |
| S8 肘 | −0.21 | −0.42 | **−50%** |

且误差是**乘性**的（随行程放大）：命令 +5° 时误差 ~0.5°，命令 +15° 时涨到 **+3.6~4.5°**。

**根因（目视复核 `overlay/02_sh_p15_fit.jpg` 得出）**：骨架拟合的绿色折线贴的是
机械臂的**外轮廓边**，而非连杆轴线。meArm 的"小臂"实际是**两根平行杆**（平行四连杆，
§2 已确证其为绝对角），而 `verify_pose.py` 用**三段连杆串联骨架**去拟合 ——
模型结构上就表达不了四连杆，于是骨架被厚轮廓的一侧吸住 → 角度系统性虚大。

**这与 §2 早已挂起的残差完全吻合**：该节记载耦合增益实测 ≈ **−0.81** 而非理论 **−1**，
并原文写着「下一次实测应重测这一点」。

**决策**：
1. **提精度的正确路径 = 改 `verify_pose.py` 的骨架模型**，使其能表达平行四连杆
   （例如：小臂按"绝对角 + 平行杆对"建模，或对肩/肘引入耦合项 `the = θ_elbow + k·θ_shoulder`）。
   **不要**试图用调 ROI / 阈值 / `--base-region` 来修 —— D37 已证其无效。
2. **在模型修正前，绝对角 FAIL 不得判为固件/链路问题**：帧间差（`--dtol`）才是
   指向固件/链路的判据（D35）。
3. `coupling.gain` 从 −1 改到实测值（≈−0.81）这一步，**与骨架模型修正一并做**，
   并各跑 ≥2 批验证 —— 与 D37 第 2 条同一纪律。

**后果**
- ✅ 把「相机反解精度不足」这个笼统问题，**定位到具体的模型缺陷**（三连杆 vs 四连杆）
- ✅ 为 §5 待办第 4 条（重测解耦残差）给出了本次的独立复现证据
- ⚠️ **当前的 `--tol 5°` 之所以还在用，正是因为模型偏置未修**；修好后应能收紧
- ⚠️ 诚实声明：本轮**未**修模型（改动大、需重布台面 + 锁死曝光配合），
  仅完成定位与记录


---

## D37 · **ROI 维持 `(330,150,1040,530)` 不变**；改 ROI 不是提精度的手段

**背景**：Phase 9 真机闭环（D34–D36）跑完，用户按「台面已重新铺白分割板」的前提
要求"用新 ROI 更新数据"。直觉上，旧 ROI 是更早的取景下标定的，台面变了就该重标 ——
于是走了一遍「量 bbox → 单批实验 → 看起来变好」的流程：

- `tools/measure_roi.py` 量出臂紧包围盒 → 数值像 `(0, 87, ~510, 349)`（半分辨率口径）；
- 单批 `e2e_192500` 实验：新 ROI `(150,170,1010,690) --base-anywhere`
  把**锚点偏置从 −2.20° 改善到 −0.45°**、各帧误差均值 4.14 → 4.48 —— **看起来是改进**。

**但跨批交叉验证推翻了这个结论**：

| 批次 | 旧 ROI | 新候选 |
|------|--------|--------|
| `e2e_191514` | 偏置 −4.27/−3.67，各帧 \|err\| 均值 **6.29** | 偏置 −5.06/−3.96，均值 **12.17**（**明显变差**） |
| `e2e_192500` | 偏置 −2.20/+0.44，均值 4.14 | 偏置 −0.45/+0.20，均值 4.48（略好） |

**新 ROI 在一批上变好、在另一批上翻倍恶化** ⇒ 单批"最优"是**对该批的过拟合**。

进一步查根因：两批**曝光几乎相同**（中位 145 vs 148，背景 169 vs 172，0% 过曝），
两批 RESET 帧相减显示**整幅画面所有边缘都亮**（亚像素整体位移/失焦特征），
⇒ 差异来自**取景在两批之间整体挪动**，而不是 ROI 选得不好。

**决策**：
1. **ROI 维持 `DEFAULT_ROI = (330, 150, 1040, 530)` 不变**（唯一来源 `tools/analyze_sweep.py`，
   被 `verify_pose.py` / `fit_pose.py` / `fit_pivot.py` / `segment_arm.py` 共 5 处 import）。
2. **禁止用单批实验改 ROI / 阈值**。任何 ROI / `--thresh` / `--base-region` 的调整
   **必须在 ≥2 个独立批次**上同时不劣化，才允许落地。理由：这类参数与
   「取景、曝光、底座掩膜」强耦合，单批数据自由度不足以约束它。
3. `tools/measure_roi.py` 定位为**辅助目视工具**，其数值输出**必须配 `--view` 目视复核**：
   实测 bbox 会被 **USB 线缆 + 桌沿暗带**污染成 `x0=0, x1=1279`（"臂有整幅宽"的假象）。
4. 精度问题的**正确着力点**记录在 D38（四连杆耦合增益），不是 ROI。

**后果**
- ✅ 阻止了一次会**降低**另一批精度的"优化"，并把「参数不可单批定」写成了硬规则
- ✅ 明确了 `measure_roi.py` 的正确用法（辅助 + 目视，而非自动定值）
- ⚠️ 反面教材：若只看单批实验的"锚点偏置 −2.20 → −0.45"，会得出与事实相反的结论，
  并在 5 个下游工具里引入一个更差的 ROI

---

## D1 · `link.length` 是机构尺寸的唯一真值

**背景**：原 spec 的 `Link` 有 `length`，`Joint` 又有 `origin.position`，两者都能表达
"下一个关节在哪"。若二者都作为输入，就会出现两个互相矛盾的真值来源，
而验收要求是「修改 link length → 模型正确更新」。

**决策**：`link.length` 为**运动学量、唯一真值**；`joint.origin.position` 退化为
**可选附加偏移**（默认 `[0,0,0]`，只在关节与连杆不共线时才需要）。

```
T_joint = T_parentJoint · Tz(parentLink.length) · T(origin.position) · R_euler · R_axis(θ)
```

**后果**
- ✅ 改一个数字即改变机构尺寸（Phase 1 验收项直接通过）
- ✅ 模型校验里加了一条启发式告警 `JOINT_OFFSET_SUSPICIOUS`：若某关节在
  `length = 0` 的父连杆上使用了非零 `origin.position`，提示"疑似把连杆长度写错了地方"
- ⚠️ 与传统 URDF（关节 origin 携带全部几何）语义不同，需在 `docs/coordinate-system.md` 明确

---

## D2 · 关节轴修正为 `base = Z`、`shoulder/elbow = Y`

**背景**：spec §八 示例 yaml 给的是 `base.axis = [0,1,0]`、`shoulder/elbow.axis = [0,0,1]`，
但 spec §九 又明确「Z：机械臂上下」。二者矛盾：若底座绕 Y 转、肩肘绕 Z 转，
得到的是**水平面内的 SCARA 机构**，与 meArm（竖直平面内 2R 机构）完全不符。

**决策**：按 §九 的坐标系语义取 `base = [0,0,1]`（绕竖直轴偏航）、
`shoulder/elbow/tool = [0,1,0]`（在 XZ 平面内俯仰）、`gripper = [1,0,0]`（爪沿 ±Y 分开）。

**后果**：FK/IK 与物理机构自洽；spec 示例 yaml 中的 axis 字段被**有意改写**，
并已在 `docs/coordinate-system.md` 记录。

---

## D3 · `gripper` 是**叶关节**，不参与末端定位

**背景**：spec §十三 明确「IK 第一阶段只要求 XYZ → J1/J2/J3」——夹爪不在定位链里。
真实 meArm 也只有 4 个舵机：3 个定位 + 1 个开合，夹爪不改变工具中心点。

**决策**：关节链为
`base_link →[base]→ column_link →[shoulder]→ upper_arm_link →[elbow]→ forearm_link →[tool]→ tool_link →[gripper]→ jaw_link(叶)`。
`gripper` 挂在 `tool_link` 之后作为叶关节，其旋转只影响爪的姿态，**不影响 TCP**。

**后果**
- ✅ IK 只需解 3 个关节，与 spec 一致
- ✅ 有专门的回归测试锁定「夹爪开合不改变末端定位」（单测 + e2e 各一条）
- ✅ 3D 显示特例：爪的两片沿关节轴 ±θ/2 对称开合（爪片挂在工具系，绕掌心中线对称）

---

## D4 · TCP 参考 `tool` 关节而非 `gripper`

**决策**：`robot.tcp = { joint: tool, offset: [0,0,40] }`。

**理由**：若参考 `gripper`，TCP 的**姿态**会被夹爪开合角污染（末端朝向随爪子转），
而"末端位姿"应当只由定位关节决定。参考 `tool` + 40mm 前伸，位置仍落在爪铰点，
但姿态干净。

---

## D5 · Three.js 直接用 Z-up + 1 单位 = 1 mm，而不是 Y-up + 转换层

**备选方案**：Three.js 保持默认 Y-up，用一个 `rotation.x = -90°` 的包装组或矩阵转换。

**决策**：设置 `THREE.Object3D.DEFAULT_UP = (0,0,1)`，场景与机器人共用 Z-up，1 单位 = 1 mm。

**理由**
- Phase 3 的验收条件是「FK 计算结果 = Three.js 实际模型位置，误差 < 0.1mm」。
  共用坐标系后这个比较是**逐值比较**，不存在"转换层写错导致两边都对不上"的排查成本。
- 实测误差 7.1e-14 mm，说明这个选择把一类整类风险直接消掉了。

**代价**：drei `<Grid>` 需要转 90°；OrbitControls 必须在相机创建前设置 `up`。
两处都已封装并注释。

---

## D6 · 运动学层零依赖（不 import three）

**决策**：`src/robot/kinematics/{transform,coordinate,fk}.ts` 只用自写的最小 4×4 矩阵
（`transform.ts`，约 150 行），**不 import Three.js**。

**理由**
- 可以在 node 里毫秒级跑 200 组随机位姿的一致性测试，不需要 WebGL
- 将来后端（Go）或固件若要复算 FK，可以逐行移植
- 渲染库升级不会波及运动学

**代价**：欧拉角提取需要自己写。已用 `Euler.setFromRotationMatrix(..., 'XYZ')` 的同一分支逻辑
实现，并由 Phase 3 测试与 Three.js 逐元素对比锁定（矩阵元素误差 5.7e-14）。

---

## D7 · 前端测试放在 `frontend/tests/`，仓库根 `tests/` 留给后端与固件

spec §三十七 的树把 `tests/` 放在根。实际取舍：前端单测/验收必须与前端源码共享
依赖解析与路径别名（vitest + vite 配置），放到根目录会造成 `node_modules` 分辨率分裂。

**落法**
- `frontend/tests/unit/` —— vitest 单元测试
- `frontend/tests/acceptance/` —— 跨层验收（FK↔Three.js）
- `frontend/tests/e2e/` —— 真浏览器 e2e 冒烟（零依赖 CDP）
- `tests/`（仓库根）—— 预留：Go 后端、串口、硬件在环

---

## D8 · 标定默认值是"推定值"，Phase 10 必须实测修正  ~~（已由 D15–D17 兑现，保留备查）~~

**决策**：`offset / reverse` 取「S9=90 正前方、S8=100 大臂竖直、S7=80 小臂共线、S6=40 闭合」
这套假设值，并在 `robot.yaml` 与文档里显著标注"待实测"。

**理由**：没有真实机械臂在手时无法确定装向；与其猜一个"看起来合理"的值藏起来，
不如**显式声明假设 + 把修正路径收敛到配置文件**。Phase 10 的验收流程（单关节 ±5° 试动，
比对虚拟方向 = 真实方向）就是为修正它而设计的。

> ✅ **已兑现（2026-09-12）**：真机到位后按本文流程实测，四个舵机的角色/零位/增益全部重算，
> 见 `docs/hardware-measurement.md` 与 D15–D17。D8 里"S8 = 肩、S7 = 肘"的假设被证伪。

---

## D9 · 舵机无位置回读 ⇒ `Actual ≡ Command`，但接口先留好

MG90S 是开环舵机，固件只能回报"上次命令的角度"，没有真实位置反馈。
但 spec §二十六/§三十四 要求区分 Command / Actual 并显示误差。

**决策**：`RobotState` 层的 `commandJoints` 与 `actualJoints` **从一开始就分开**，
状态面板同时显示两者与误差；仿真态下 actual 立即跟随 command，误差恒为 0。
Phase 11 一旦有真实反馈，只需让 transport 调 `setActualJoints()`，**UI 与误差计算零改动**。

---

## D10 · 不使用 React.StrictMode

**理由**：R3F 的 `<primitive object={...}>` 在 StrictMode 双调用下会重复触发 three 对象的
挂载/卸载生命周期；本项目还需在每帧做 FK 一致性校验，保持单次挂载更可预测。
（这是有意的取舍，不是遗漏。）

---

## D11 · 显示几何参数化进 `robot.yaml`，不写进渲染代码

**背景**：Phase 2 初版的连杆是 4 段实心方块，与实物 meArm（蓝色薄板 + 4 个舵机）差异很大。
若按常规做法在 `buildRobotObject3D.ts` 里硬编码板厚 / 舵机位置，就会重演 spec §9 禁止的
"大量写死坐标"，并且用户"后续微调设计"时还得改代码。

**决策**：在 `LinkGeometry` 中新增 `plate`（倒角薄板）与 `servo`（舵机壳体）两种类型，
并为 `Link` 增加 `details?: LinkGeometry[]` 承载附加件；**全部机构外观只存在于 `robot.yaml`**，
渲染层只做「参数 → 网格」的翻译，不含任何机构尺寸常量。

**理由**：满足 spec §9 / §30，"改 yaml 即改外观"；同时因为 `geometry` / `details`
不参与运动学，这次外观重做**没有动到任何 FK 数字**（回归断言见 D13）。

---

## D12 · 舵机不用带臂舵盘，改用圆舵盘

**背景**：要一眼看出"这是 meArm"，4 个舵机是识别度最高的特征，所以必须在模型里画出来。

**决策**：舵机 = 壳体 + 两侧安装耳 + 金属输出轴 + **圆盘**舵盘。

**理由**：舵机画在**父连杆**上（S7 画在大臂、S8 画在立柱…），而舵臂在真实机构里是随
**子连杆**转的。若画成带臂舵盘，关节一转就会出现"舵臂不动、连杆转"的方向错觉。
圆盘无方向性，规避了这个矛盾——沿用 spec §35「结构正确性优先于视觉」。

---

## D13 · 几何回归测试：抹掉外观后 FK 必须逐位不变

**决策**：新增 `frontend/tests/unit/linkGeometry.test.ts`（13 项），把三条不变量钉死：

1. `plate` / `servo` 的解析与默认值补齐正确，非法 `geometry.type` 被拦截；
2. **把整份 `geometry` / `details` 换成 `{ type: 'none' }`，`endEffectorPosition()` 返回值不变**
   —— 几何与运动学严格解耦；
3. 场景里真的建出了 4 个舵机；夹爪 `θ=0` 两爪贴合、`θ=90` 向外张开**不互穿**
   （用"相对掌心中线的坐标"断言，与整机姿态无关）。

**理由**：显示层一旦悄悄参与运动学，会表现为"虚拟臂看着对、真机角度不对"的隐性事故，
且极难定位。第 3 条还顺带修掉了一个既有缺陷：原实现的爪片沿 **X** 偏移，而铰轴也是 X，
导致 `applyJointState` 的开合符号写反、两爪实际是越过中线互穿（因爪片很薄而肉眼难辨，
之前的 e2e 只断言"夹爪不改 TCP"所以没暴露）。现改为沿 **Y**（垂直于铰轴）偏移并修正符号。

---

## D14 · 构树后立即应用一次关节状态（消除首帧零位瞬态）

**背景**：e2e 偶发失败——首次采样到的 `FK↔3D` 读数为 55.6 mm，而稳态是 0 mm。
55.6 正是「关节全零位时 TCP `(0,0,260)`」与「HOME 位姿 FK `(54.9,0,251.6)`」之间的距离：
`useFrame` 的第一次节流采样可能落在 `applyJointState` 的 effect 之前。

**决策**：`RobotArm` 的 `useMemo` 里构树后**立刻**调用一次 `applyJointState`，
使对象树从第一帧起就处于目标位姿；同时把 e2e 的等待条件从"读数出现数字"改为
"读数收敛到 < 0.1 mm"（超时仍会断言失败，不掩盖问题）。

**理由**：这是一帧的显示瞬态而非 FK 缺陷，但会让"最强证据"读数在被观测时不可信；
在源头消除 + 断言侧加固，两边都做。


---

## D15 · 舵机角色以实测为准：S7 = 肩、S8 = 肘（**推翻 D8 的安装位假设**）

**背景**：D8 按固件命名（`bsp/servo.h` 的 `left / right`）把 S8 当作肩、S7 当作肘。
真机到位后，用 3 种互相独立的方法测量，结论相反：

| 方法 | 观察 | 判定 |
|------|------|------|
| 帧间"画面最高点"位移 | 扫 **S8** 全程位移 ≈0 px；扫 **S7** 位移 194 px 且下沉 101 px | S7 才是肩 |
| 公共静止区自动分段 | `inter_S8 − inter_S7 ≈ 6342 px` 恰好是大臂的面积 | 同上 |
| 逐帧目视（3×2 网格） | S8 变化时大臂角度逐帧不变，只有肘端往下那段在摆 | 同上 |

**决策**：`S7 -> shoulder`、`S8 -> elbow`，并同步修正 `tools/mearm_hw.py` 的 `SERVO_SPEC`
角色标签与 `robot.yaml` 的显示几何注释。**固件的 `left/right` 只是安装位称呼，不代表运动学角色。**

**理由**：这是"虚拟臂看着对、真机角度不对"的典型来源；且 D8 的假设当时就无法自证，
实测是唯一的收敛手段。

---

## D16 · 小臂建模为**绝对角**关节（平行四连杆），用 `coupling` 表达

**背景**：实测扫 S7 使**肩关节**转过 **52.91°** 时，小臂的**绝对倾角只漂 10.08°**，
而它的**相对角**变了 −42.83°。⇒ 真机小臂由独立舵机经**平行四连杆**驱动，
连杆参考系挂在底座侧，小臂角与肩角**解耦** —— 这是标准的串联 FK 表达不了的。

**决策**：给 `Joint` 增加可选 `coupling: { joint, gain }`，语义为

```
relative = value + gain × otherJointValue        # 平行四连杆取 gain = −1
```

即 **`elbow` 关节存"绝对倾角"**。`effectiveJointAngle()`（`fk.ts`）是唯一取值口，
FK 与 Three.js `applyJointState()` **共用它**，保证"屏幕上的臂就是 FK 算的那台"。

**后果**
- ✅ 关节限位变成 `108.44° … 141.86°`（不再含 0）：这是真机约束，不是缺陷。
  `zeroJointState()` 的 0° 因此**物理不可达**，`robotStore.goZero()` 会把它钳到最竖直可达角，
  日志也照实打印钳位后的值（见 `RobotModel.zeroJointState` 注释）。
- ✅ 校验器新增 4 条：`JOINT_COUPLING_SELF / TARGET / FIXED / ZERO`
- ⚠️ **已知残差**：实测漂移 10.08° ≠ 0，硬拟合等效增益 ≈ −0.81。模型取理论值 −1，
  因为这正是机构本义，且 10° 落在拟合残差（≈9.5 mm）之内。下次实测复测（见测量文档 §5）。

---

## D17 · 标定与限位全部由实拍反解，测试期望改为"跟随配置"而非硬编码

**背景**：D8 的推定值（`offset/scale`）与角色假设一起被推翻后，13 项硬编码了旧配置期望值的
测试全部失败。这些失败**不是代码 bug**，而是"期望值写死在测试里"这一做法本身的脆弱性暴露。

**决策**：
1. 用 `tools/fit_pose.py`（对称 Chamfer + 模式搜索）从照片反解 `offset / scale / 限位 / HOME`，
   写入 `config/robot.yaml`（唯一数据源）；
2. **测试改成从模型派生期望值**（`jointById(model,'elbow').limits.max`、
   `homeJointState(model)`、解析式）而不是抄常数；只有"解析解锚点"（HOME TCP、舵机 90°）保留数值，
   因为它们本身就是要验证的物理自洽性。

**后果**
- ✅ 43 项单测 / 验收测试 + 12 项 e2e 全绿；`tsc -b` 零错误；`vite build` 通过
- ✅ 以后再改 `robot.yaml` 的数值，只有"解析解锚点"需要跟着更新，不会连锁 13 处
- ⚠️ 解析式与模型公式同构，属于"回归锚点"而非"独立实现"，这一点在测试注释里写明

---

## D18 · IK 直接输出 `elbow` 的**绝对角**，不叠加肩角

**背景**：`elbow` 存的是离开天顶的绝对倾角（D16，平行四连杆解耦）。
平面 2R 的解析式里，`α = θe − θs` 是**相对**肘角，因此解出 `θe` 后有两种写法：

| 写法 | 结果 |
|------|------|
| 把 α 当"相对角"存进 JointState（常规 meArm 写法） | ❌ 机构错位：FK 里再减一次肩角，小臂实际只转了相对量 |
| `elbow = θs + α` 且语义是**绝对角**，由 `effectiveJointAngle()` 负责减肩角 | ✅ 正确 |

**决策**：`ik.ts` 解出的 `the = ths + relativeAngle` **就是** `JointState.elbow` 的最终值，
不再做任何叠加/补偿。相对角的换算**只在一处**发生 —— FK 的 `effectiveJointAngle()`
（`relative = value + gain × other`，gain = −1）。IK 与 FK 之间**零补偿**。

**后果**
- ✅ `FK(IK(XYZ))` 2000 组随机位姿最大残差 **1.401e-13 mm**（浮点噪声级）
- ✅ IK 不需要知道 `coupling` 的存在，"绝对角"只是关节角的语义
- ⚠️ 若将来某关节从"绝对角"改回"相对角"，改 `robot.yaml` 的 `coupling` 即可，
  **IK 代码无需改动**（它对两种语义是无感的）

---

## D19 · IK 的多解策略：默认就近，但本机实际只有一支解

**背景**：平面 2R 有两支解，用相对肘角符号区分：`elbow-up`（`α > 0`）、`elbow-down`（`α < 0`）。
固定选支会在末端经过工作空间内边界时突然翻肘（Phase 6 拖动会非常明显）。

**决策**：
1. `prefer` 支持 `'elbow-up' | 'elbow-down' | 'nearest'`，**默认 `'nearest'`**
   —— 按 `near` 当前状态取"关节空间距离最小"的可行解，从根上消除翻肘跳变；
   缺 `near` 时退化为 `elbow-up`（不抛异常）。
2. `solveIkAll()` 把**两支候选连同越界量**一并返回，UI / 调试面板可以展示"另一支差多少度"，
   而不是只给一个"无解"。
3. 指定支不可行时**回退到可行支**，并保证返回值**恒在限位内**（绝不静默返回越界解）。
4. 模型形状不符合"平面 2R"前提（`base.axis ≠ Z`、`shoulder/elbow.axis ≠ Y`、
   `origin.rotation ≠ 0`）时**抛 `IkModelError`**，把"改配置后的静默错解"变成明确报错。

**⚠️ 实测得出的结构不变量（本机只有一支解）**：
`elbow.limits.min − shoulder.limits.max = 108.4415 − 49.4549 = 58.99° > 0`
⇒ **相对肘角恒为正** ⇒ `elbow-down` 支在本机**永远越界**。
2000 组随机可达目标的实测分支分布为 `{elbow-up: 2000, elbow-down: 0}`。

**后果**
- ✅ 多解策略是**通用性预留**（换机构才会激活），本机当前不会翻肘 ——
  Phase 6 拖动可以直接用 `prefer: 'nearest'`，但即使写成固定 `'elbow-up'` 也不会有跳变
- ✅ 该不变量写成测试锁死（`tests/unit/ik.test.ts`、`tests/acceptance/ik-fk-roundtrip.test.ts`）：
  一旦有人放宽 `shoulder` 上限到 ≥108°，测试会立刻失败并提示原因
- ⚠️ 失败信息的 `reason` 只有 `OUT_OF_WORKSPACE` / `JOINT_LIMIT` 两种（对齐 spec 与
  `docs/serial-v1.md`）；**模型配置错误走抛异常**，不复用这两个码，避免二者混淆

---

## D20 · 末端目标 `target` 与命令关节 `commandJoints` 严格分离，越界时**不钳位**

**背景**：Phase 6 引入「指定末端目标」（XYZ 直输 / 鼠标拖动）。目标可能落在工作空间之外，
此时有两种做法：把关节钳到边界最近点，或干脆不动。

**决策**：`target` 与 `commandJoints` 是两个独立的量。目标越界时只更新 `target` 与状态栏
（`OUT_OF_WORKSPACE` / `JOINT_LIMIT`），`commandJoints` / `endEffector` **逐位不变**。

| 量 | 含义 | 越界时 |
|----|------|--------|
| `target` | 我想去哪 | 照常保留 |
| `commandJoints` | 实际去哪 | 逐位不变 |
| `endEffector` | `FK(commandJoints)` | 恒在量程内 |

**理由**

1. 钳位会让工作空间边界从界面上「消失」，用户无法建立对机构能力的直觉；
2. **球壳边界与关节限位不是同一个约束**：`[|L1−L2|, L1+L2]` 上的点
   （D20 当时 = `[40, 200]`；**D70 后 = `[0, 160]`** —— `L1 == L2` 使内半径退化为 0，
   于是"离枢轴太近"不再是越界理由，约束改由关节限位 + 腕的 hinge 区间给出）
   仍可能违反 shoulder / elbow 限位，钳位会造出「看似合法、真机做不到」的姿态；
3. Phase 9 下发真实指令时只读 `commandJoints`，分离后天然安全。

**后果**

- ✅ 场景里把手球与 TCP 圆点分离并画虚线，「还差多远」直接可见
- ✅ 拖动越界段关节**零变化**（`tests/acceptance/drag-tracking.test.ts` 断言连续越界帧逐位相同）
- ⚠️ 手感上是「拖不动」而非「贴边滑动」——这是刻意取舍：边界可见性优先于顺滑

---

## D21 · 拖动平面在 `pointerdown` 时冻结，中途不可变

**背景**：把鼠标射线投影成 3D 点需要一个平面。若每帧按当前 TCP 重建平面：

```
目标点变 → 关节变 → TCP 变 → 平面跟着变 → 目标点再变 → …
```

这是一个**自反馈环**，鼠标与目标点永远无法收敛，手感表现为「飘走」。

**决策**：`pointerdown` 时一次性冻结三件东西 —— 平面锚点（= 按下瞬间的 TCP）、
平面法线（由模式决定）、平面模式本身。整个拖动过程不再改变。
同时 `resetTarget()` 把起点吸附到真实 TCP，保证「抓住的就是末端」。

| 模式 | 法线 | 自由度 | 定位 |
|------|------|--------|------|
| `xy`（默认） | `(0, 0, 1)` | X / Y | 桌面臂最常用；Z 固定时成功率最高 |
| `camera` | `−相机视线` | 全自由 | 鼠标最跟手，但容易一下甩到不可达区 |
| `xz` | `(0, 1, 0)` | X / Z | 专调高度与前后 |

**后果**

- ✅ 水平面拖动时 `target[2]` **逐位等于**按下瞬间的 TCP Z（e2e 实测：
  ~~`93.83984236589028`~~ → **已由 D70 重测为 `109.22362788339143`**，仍是逐位相同
  —— 被锁分量是**定义**而非数值巧合；数值随 HOME TCP 一起变，**结论不变**）
- ✅ 拖动中途切换下拉不影响进行中的这次拖动
- ⚠️ 拖动期间必须 `controls.enabled = false`，否则会边转相机边拖末端；
  并用 `setPointerCapture` 保证指针移出 canvas 不中断

---

## D22 · e2e 用 dev-only 探针 + CDP 真实鼠标事件验证拖动

**背景**：验证「鼠标拖动末端」需要知道把手在屏幕上的像素坐标，否则只能做弱断言。
合成 DOM 事件（`dispatchEvent`）绕不过 R3F 的射线拾取，证明力不足。

**决策**：在 `import.meta.env.DEV` 门控下挂 `window.__armPilot`
（暴露 `tcpScreen()` / `state()` / `moveTo()`），e2e 用 CDP `Input.dispatchMouseEvent`
派发**真实鼠标事件**精确命中场景里的把手。

**后果**

- ✅ e2e 拿到真实拖动证据：`Δ 17.47 mm`，且拖动结束后 `dragging` 正确复位
- ✅ 生产构建下该组件不渲染（Vite 静态消除）；验收脚本额外检查产物中不含 `__armPilot` 字样
- ⚠️ 这是**产品代码里的测试钩子**，属受控例外：新增钩子必须同等门控 + 产物检查

---

## D23 · Transport 回推**只写** `actualJoints`，绝不回写 `commandJoints`

**背景**：闭环接上后，`RobotState` 在 store 与 transport 之间有两条可能的流向。
若回推时把 `state.joints` 同时写进 `commandJoints`，就形成
`命令 → 状态 → 命令` 的无限回环（spec §十九明确禁止）。

**决策**：
1. `transportBridge.handleState()` **只调** `setActualJoints()`，不碰命令。
2. 与此同时，`actualJoints` 的写入者必须**排他**：接入后命令路径不再写它
   （由 `transportDriven` 切换，见 D25）。
3. 断开时把 `actual` 拉回 `command` —— 否则最后那次回推的滞后值会永久留在面板上，
   而此时已无 transport 驱动它收敛，显示的是一个**假误差**。

**后果**
- ✅ 400 点拖动轨迹中，回推期间的 `commandJoints` **对象引用一次都没变**
  （`transport-loop.test.ts` 用 `toBe` 断言引用，比逐位比较更硬）
- ✅ 用户意图（命令）与机械臂实际位置（actual）成为两个独立事实，
  这正是 ArmPilot 能回答"真机跟上了没有"的前提

---

## D24 · `MockTransport` 如实模拟物理约束，不做理想回显

**背景**：Phase 7 的目标是"闭环走通"。最省事的实现是收到命令立即原样回推 ——
接口通了，但 `actual ≡ command`、误差恒为 0。

**决策**：模拟的不是"接口"，而是"机械臂"，如实建模四件事：

| 模拟项 | 默认值 | 用途 |
|--------|--------|------|
| 舵机有限角速度 | 240°/s | 拖动中误差非零、松手后收敛 —— Phase 11 误差链路的预演 |
| 传输延迟 | 15ms | 命令送达前 Actual 不动 |
| 回推丢帧 | 0% | 验证"丢帧只影响上报，不影响机械臂" |
| 限位校验 | 开 | 越界回 `ERR JOINT ...` 且**位置不动**，为 Phase 9/10 错误路径铺路 |

**为什么值得**：理想回显会让四条路径（误差显示 / 收敛 / 丢帧容错 / 限位拒绝）
**一条都没被验证**，全部推迟到真机阶段首次暴露。

**后果**
- ✅ e2e 有一条断言专门守这点：设完滑杆立刻读数，Actual 必须明显滞后
  （实测 `cmd 39.9° / act 0.8° / 差 39.1°`）
- ✅ 单测用可注入虚拟时钟（`FakeTimer`）精确验证"每 tick 恰好 4.8°""收敛后 tick 归零"，
  无一处真实等待
- ⚠️ 参数默认值是**估算**（MG90S 带载 ≈240°/s），真机实测后可调，不影响接口

---

## D25 · 命令下发用**尾沿合并**节流；`actualJoints` 的写入者由 `transportDriven` 排他

**背景**：拖动时每帧都会改 `commandJoints`（60fps ⇒ 每秒 60 条）。Phase 9 串口在
115200 + ACK 门控下不可能承受（同源问题见 skill `arm-robot-serial` 关键坑 6：
周期查询会饿死遥控流）。

**决策**：
1. **尾沿合并（trailing edge）**，`MIN_SEND_INTERVAL_MS = 33`（约 30Hz）：
   间隔内只保留最新一帧；**拖动结束时由 trailing 定时器补发最后一帧**。
2. `deriveVirtual()` 增加 `transportDriven` 参数：接入后不再写 `actualJoints`。
3. 日志按 `LOG_FLUSH_MS = 500` 合并成一条 `TX ×N` / `RX ×N`，
   对齐 `LogPanel` 已有的"连续拖动不逐帧记录"约定。
4. 统计刷新带**脏检查**，收敛后不再触发 React 重渲染。

**后果**
- ✅ 400 次命令变化只下发 10–80 条（测试断言区间）
- ✅ 少了尾沿补发会让机械臂停在半路 —— 这条影响"手感"而非"正确性"，但很显眼
- ⚠️ `RobotTransport` 接口本身**不变**：节流是 bridge 的职责，不是 transport 的。
  Phase 8 的 WebSocketTransport 自动继承同一套节流

**踩到的坑**：`TransportBridge.dispose()` 最初先退订 status 监听、再 `transport.disconnect()`，
于是 `disconnected` 事件**没有听众** —— 断开后状态灯仍显示 Connected、Actual 挂着滞后值。
改为在 `dispose()` 末尾**显式**调用 `setConnection(null, 'disconnected', ...)`，不依赖事件时序。

---

## D26 · `docs/serial-v1.md` 的角色映射按实测修正

**背景**：该文档 §2 的关节↔舵机映射仍沿用早期**推定值** `shoulder→S8 / elbow→S7`，
与 Phase 4.5 实测结论（**S7 = 肩、S8 = 肘**，见 D15）**正好相反**。
这是 Phase 9 固件要照抄的文档，不改会把错映射带进固件、造出一台反着动的机械臂。

**决策**：按 `config/robot.yaml` 的实际值重写 §2 表格（含标定式），并修正：
- 关节角范围改为实测值（`elbow` 是 108.44…141.86，不是 0…90）
- `JR` 示例改用**真实可达**的整帧，回执示例给出换算后的舵机角
- `ERR JOINT` 示例的限位区间同步更新
- §5 的 JSON 错误消息里的旧限位 `0..80` 一并修正

**后果**
- ✅ 文档与 `config/robot.yaml` 单一真值一致；固件照抄不会错
- ✅ 新增的 `JR 0 30 120 50 → S7=131.98 S8=72.33` 示例**显式展示了绝对角语义**：
  肩转 30° 时小臂绝对角不变，但两个舵机都要重算 —— 按相对角实现会算出完全不同的值

---

## D27 · 后端落点：新建 `MeArm-3D/backend/`（自包含 Go module），监听 8090

**背景**：关节级 WebSocket 服务可以塞进既有的 `MeArm-RemoteControl`（已有串口 + Web 经验），
也可以新建。前者的风险是：那是个**舵机级摇杆**服务，混入关节级逻辑会让"谁负责标定"
变得模糊 —— 而本项目的铁律是**标定只有 `config/robot.yaml` 一份**。

**决策**：
1. 新建 `MeArm-3D/backend/`，自包含 module `armpilot/backend`，只依赖 `gopkg.in/yaml.v3`。
2. 端口 **8090**（`MeArm-RemoteControl` 占 8080），两者可**同时运行**。
3. 模型真值直接读 `../config/robot.yaml`（三级回退，见 `cfg.ResolveRobotConfig`），
   `backend/config.yaml` **只放运行参数**，写注释禁止出现限位/标定数值。
4. 分层 `wsserver → controller → device`，外加 `protocol`（编解码）与 `robot`（标定/限位）。

**后果**
- ✅ 后端与前端**共享同一个真值文件**，`hello` 里回传的就是它读到的内容 ⇒ 两端不一致会被当场发现
- ✅ `go test ./...` 56 项、`go vet` / `gofmt` 干净；前端侧一行 Go 都不需要
- ⚠️ 两个 Go 服务各有一份 HTTP 栈；Phase 9 若合并需重新评估（当前无必要）

---

## D28 · 链路末端先用 Go 内置 sim（假固件），且**必须非等值回显**

**背景**：Phase 8 验收明确要求"先不接真机，用 Mock 验证完整闭环"。最省事的做法是
WebSocket 层收到命令就直接把同值回推 —— 接口通了，但 `Actual ≡ Command`、误差恒为 0。

**决策**：`internal/device/sim.go` 实现一台**假固件**，走**真实字节流**（`JR` / `OK JR` / `STATE`
都是字符串），只有"串口驱动"是假的。它维护**舵机空间**的 target/actual，如实建模：

| 模拟项 | 默认值 | 复现的真实现象 |
|--------|--------|----------------|
| 舵机有限角速度 | 240°/s、tick 20ms | 每 tick 恰好 4.8°；拖动时 Actual 滞后，松手后收敛 |
| 指令送达延迟 | 15ms | 延迟期内**不回执**，Actual 一动不动 |
| 舵机硬限位 | 开 | 越界回 `ERR SERVO S8 …` 且**位置保持不动** |
| 开机静默窗口 | 0（可配） | 模拟 Uno bootloader 交权期：指令被吞且不回执 |
| 状态回推 | 由**实际舵机角反算** | 标定表写错 ⇒ Actual ≠ Command，立刻暴露 |

**后果**
- ✅ 标定**可逆性**被真实检验：`JointToServo → 舵机走位 → ServoToJoint` 走完整往返
- ✅ e2e 抓到真实滞后（`cmd 29.9° / act 0.8° / 差 29.1°`）→ 收敛 `29.9° / 29.9°`
- ✅ 后端集成测试断言"**必有中间态**"（`4.18 → 7.52 → 10.85 → 14.18 → 17.51 → 20.80`），
  等值回显会让这条直接失败

---

## D29 · `OK JR` 携带的是**目标**舵机角 ⇒ 只做标定核对，状态只认 `STATE`

**背景**：协议 §4 让 `JR` 的回执带上换算后的舵机角，本意是"便于上位机核对标定"。
很容易顺手拿它当"机械臂现在的位置"发布出去。

**决策**：
1. `OK JR` **只**走 `verifyCalibrationEcho()`：与本地标定算出的舵机角比对，
   偏差 > `calibToleranceDeg = 0.1°` 只**告警不阻断**（链路仍可用）。
2. **绝不**用 `OK JR` 的值发布 `joint_state`。状态唯一来源是 `STATE` 行（舵机**实际角**反算）。
3. `Error()` 文案稳定（`ERR JOINT elbow 95.00 (limit 108.44..141.86)`），50 次重复断言首个越界项恒为 `base`
   —— map 迭代顺序随机，但用户看到的必须确定。

**后果**
- ✅ 误差链路真的活着：`OK JR` 若被当成 Actual，误差**恒为 0**，Phase 11 才会第一次暴露
- ✅ 单测有一条专守这点："ACK 到达**不发布**状态"
- ⚠️ 标定核对只报警告：真实机构磨损会让回执与本地标定有零点几度偏差，不该导致链路不可用

---

## D30 · ACK 门控 + latest-wins（**不排队**）；间隔节流禁止用 `time.Sleep`

**背景**：Phase 9 串口 115200 + ACK 门控下不可能并发下发。拖动时命令高频变化，
若全部排队，回执永远在追历史的某个中间位置（用户看到"越拖越滞后"）。

**决策**：
1. 同一时刻**只有 1 条 `JR` 在途**；在途期间新命令**只覆盖"待发槽"**（silent drop 旧待发值）。
2. 回执到达 / ACK 超时 → `drainPending()` 把待发槽发出去。
3. `min_send_interval_ms > 0` 时**禁止用 `time.Sleep`**：睡眠会卡住读循环，
   而回执正需要读循环消费 ⇒ **自锁**。改为"撤下在途、放回待发槽、`time.AfterFunc` 补发"。

**踩到的坑**：首版就是这么写的 —— `send()` 里 `time.Sleep` 把 `readLoop` 阻塞，
表现为"发完第一条就再也收不到回执"。`Controller.finishInflight()` 改成无参、
超时回调统一走 `onAckTimeout()` 之后才正确。

**后果**
- ✅ 单测覆盖：latest-wins 合并（400 次变化只下发首帧与最新帧）、ERR 放行门控、
  超时上报并补发、部分帧保留其余关节
- ✅ 前端侧另有**尾沿合并**（33ms）做第一道节流（D25）—— 两道节流职责不同，都要留

---

## D31 · 跟踪误差必须在**链路精度**上定义（0.1°）

**背景**：e2e 抓到一条**假误差**：面板永久显示「跟踪误差 `0.02°`」、
「正在逼近目标」永不熄灭。根因不是机械臂没到位，而是**双方用了不同精度表述同一个位置**：
前端内部 `elbow = 112.6185771989`，`JR` 线只保留 1 位小数（`112.6`），
直接相减恒差 `0.0186°`。

**决策**：
1. 在 `wsProtocol.ts` 显式声明链路精度 `WIRE_JOINT_STEP_DEG = 0.1`（依据 `docs/serial-v1.md` §4）。
2. `lagDeg()` 比对前先 `quantizeForWire(命令)`，之后误差可**精确归零**，
   0.1° 以上的**真实**滞后照实上报。
3. 只量化**命令侧**，不量化测量的状态 —— 后端回推的 `joint_state` 已经受 `STATE` 行精度约束，
   再对它取整会**掩盖**真实误差。
4. 实现细节：必须 `Math.round(v * 10) / 10`，不能 `Math.round(v / 0.1) * 0.1`
   —— 后者乘上不精确的 `0.1` 会差 1 ulp，让"误差 0"退化成 1e-14、`moving` 再次永真。

**后果**
- ✅ e2e 的 `stat-lag` 由 `0.02°` 变为 `0.00°`（本轮实测）
- ✅ 新增 8 项单测（`quantizeForWire` 6 项 + 误差语义 2 项），其中一条专门守住上面那个 ulp 陷阱
- ⚠️ 这是**协议层**的精度事实，不是前端的一个补丁：0.1° 关节 ≙ 肩 `0.14°`/肘 `0.24°` 舵机角，
  与舵机自身约 `0.18°` 的分辨率同量级 ⇒ 没有浪费也没有损失

---

## D32 · 断线重连后**补发当前命令**，且只在重连时补

**背景**：断线期间用户可能已经拖到别处。重连成功若不补发，虚拟臂与实际臂会**永久错开，
而 UI 上看不出任何异常** —— 属于"看起来正常"里最危险的一类。

**决策**：`transportBridge` 在 `handleStatus('connected')` 时，若 `hasConnectedOnce` 为真
则 `enqueue(store.commandJoints)` 补发一次。

**为什么必须加 `hasConnectedOnce` 条件**：首轮实现无条件补发，
结果**打掉了 Phase 7 已锁定的时序契约** —— 首次连接白白占用一次节流窗口，
`transport-loop.test.ts` 的阶跃响应断言立刻失败（`expected 0.8498937633 to be close to 5.6498937633`）。
首次连接时命令本来就是初始值，补发毫无意义。

**后果**
- ✅ e2e 端到端验证：杀掉后端 → 重启（新进程 sim 从 HOME 起步）→ Actual 必须回到 `29.9°`。
  **没补发就会永久停在 HOME，这条断言必失败**
- ✅ 单测另有"断线期间改动也被补发"（补发的是 `42` 而非断线前的 `10`）

---

## D33 · `hello` 携带模型真值做**在线互检**；心跳必须是两层

**背景**：本项目最贵的一次错误是"按固件命名推定舵机角色"（D15），它靠**相机实测**才发现。
同类错误（两端读到不同的 `robot.yaml`、标定表抄了两份）必须能在**运行时**立刻暴露。

**决策**：
1. 后端接入即推 `hello`，携带它读到的**关节顺序 / 限位 / 舵机通道 / 标定 offset·scale·reverse**。
2. 前端 `describeModelMismatch()` 逐项比对本地 `RobotModel`（容差 `1e-6`），
   不一致**告警但不断开**（链路仍可用，先让人看见）。
3. 心跳**两层**：应用层 `ping`/`pong`（浏览器 `WebSocket` API 不暴露传输层 ping，
   只能自己做）+ 服务端 RFC6455 `ping`（浏览器自动回 `pong`）由服务端看门狗判死。
4. 在途 ping 未回时**不重复发**，否则超时判定会被自己不断推后。

**后果**
- ✅ 单测覆盖全部差异类型（关节顺序 / 数量 / 限位 / 通道颠倒 / 标定 / 未知关节）
- ✅ `SocketLike` + `SocketFactory` 注入让"心跳超时判死""指数退避封顶""旧 socket 迟到
  `onopen` 不得把状态拽回 connected"这类时序逻辑全部**确定性**可测（无真实等待、无真实网络）
- ✅ e2e 实测 RTT `20 ms`，且模型一致性无告警（前后端读同一份 yaml）
