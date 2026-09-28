package tcpserver

// TCP JSON 控制协议：解析 / 校验 / 分发。
//
// ## 本包**只做协议转换**，不做控制
//
//	TCP JSON  →  校验参数  →  调用现有 controller.ApplyFrom（关节角整帧 + 来源）
//
// 这里**没有**、也不允许出现：IK / FK 算法、舵机运动学、软限位逻辑、
// Sim / Real / Serial 的任何分支判断。切换 `device.mode` 不影响本文件一个字。
//
// ## 写命令一律声明来源 `protocol.OriginExternal`
//
// TCP 客户端是**另一个进程**（如 MeArm-RemoteControl）。它的命令会经
// controller 广播给 MeArm-3D 自己的页面，而那台页面靠帧上的 `origin` 决定
// "要不要让指令侧跟随"：`command` = "本页自己发的，实际位姿在追" ⇒ 只更新 Actual；
// 外部驱动必须让它**跟随**（否则画面上只有半透明的实际臂在动、主臂不动，
// 且那台页面的指令侧停在旧值，用户下次动本页控件会把整组旧指令下发）。
// 所以这里用 `ApplyFrom` 显式声明来源，而不是图省事借用 `Apply`。
//
// v1 的三条命令（刻意最小化）：
//
//	move     XYZ **单轴**相对位移 → 当前位姿(FK) → 目标点 → IK → 关节角 → Apply
//	gripper  open / close → 取该关节限位的端点 → Apply
//	servo    1..4 直接给舵机角 → 舵机硬件限位校验 → 舵机角转关节角 → Apply
//
// 外加一条**只读**命令（v1.1 补，见 docs/protocol/tcp-v1.md §3.4）：
//
//	state    无参数、无副作用 → 直接回当前 state → 不调用 Apply
//
// v2 追加的 XYZ **矢量**命令（见 docs/protocol/tcp-xyz-v2.md）：
//
//	movexyz  三轴同时相对位移 [dx,dy,dz] → 一次 IK → 一次 Apply
//	moveto   绝对目标点 [x,y,z] → 一次 IK → 一次 Apply
//	home     回 HOME 位姿（取自 robot.yaml）
//	caps     只读：回报几何 / 限位 / 行程 / 可达包围盒，供外部项目做坐标映射
//
// ⚠️ v2 **只新增命令**，不改 v1 任何一条的语义、参数、错误文案与优先级
//    （判据：protocol_xyz_test.go 的 `TestMove_SingleAxisEqualsMoveXYZ`）。
//
// 为什么必须有它：上面前三条**全是写命令**。任何外部控制器（如
// MeArm-RemoteControl）在"首次下发前"都拿不到当前舵机角 —— 而舵机级控制是
// 绝对角语义（`servo` 要目标角），没有基准就只能猜，猜错的第一帧就是一次跳变。
// 它也兑现了 §13「状态反馈优先复用服务端真值」的要求：不新增状态系统，
// 只是把现成的 `state()` 暴露成一个入口。
//
// ⚠️ `state` **不持 h.mu**：加锁会让一次只读查询挡在别人的"读当前→算目标→下发"
//    中间。而 `Snapshot()` 本身在 controller 内已加锁、返回的是**拷贝**，
//    读到的必然是一个自洽的目标帧，不会看到半更新的 map。
//
// ## 限位与校验一律复用模型真值
//
// 关节限位走 `Model.Validate`（由 `controller.Apply` 内部调用）；
// 舵机硬件限位走 `Actuator.Limits`（robot.yaml 的 actuators[].limits）。
// 本文件不持有任何角度/尺寸常数 —— 那是"第二份真值"的经典成因。

import (
	"encoding/json"
	"fmt"
	"math"
	"strings"
	"sync"

	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// Arm 是 TCP 层对机械臂的**全部认知**。
//
// 刻意只留这四个方法：接口越窄，越难在 TCP 层长出自己的控制逻辑。
// `*controller.Controller` 天然满足它（main.go 直接传即可）。
type Arm interface {
	// Apply 下发关节角整帧（内部做限位校验 + ACK 门控 + latest-wins）。
	Apply(joints map[string]float64) error
	// ApplyFrom 同上，但**声明来源**（`protocol.Origin*`）。
	//
	// 外部入口必须走这一个：借 `Apply`（= `OriginCommand`）会让对端页面把它当成
	// "本页自己发的命令"，只更新 Actual ⇒ 画面上**实际臂在动、主臂不动**，
	// 且那台页面的指令侧停在旧值（下一次动本页控件会把整组旧指令下发）。
	ApplyFrom(origin string, joints map[string]float64) error
	// Snapshot 取当前命令值与状态值
	Snapshot() (command, state map[string]float64)
	DeviceKind() string
	DeviceConnected() bool
}

// protocolVersion 协议版本，由 `caps` 回报给外部项目做能力探测。
//
//	1 = v1（move / gripper / servo / state）
//	2 = 追加 XYZ 矢量命令（movexyz / moveto / home）与 caps
//
// ⚠️ 只在**新增**命令时递增。改现有命令的语义或字段格式是破坏性变更，
// 不能靠版本号糊过去 —— v1 客户端不会因为多几个 `cmd` 而失效，
// 但会因为 `move` 的含义变了而失效（那正是本次要避免的）。
const protocolVersion = 2

// workspaceSamples `caps.workspace` 包围盒的每轴采样份数。
//
// 3 个定位关节 ⇒ n³ 次 FK（n=12 ⇒ 1728 次，微秒级）。只在 `caps` 被调用时算，
// 不在热路径上。外部项目要复现同一组边界就得用同一份数，所以它是应答的一部分。
const workspaceSamples = 12

// supportedCommands 本服务支持的命令名（`caps.commands`）。
func supportedCommands() []string {
	return []string{"move", "gripper", "servo", "state", "movexyz", "moveto", "home", "caps"}
}

// Command 一条 TCP 指令（JSON Lines 的一行）。
//
// 指针字段用来区分"没给"与"给了 0"：
// `step` 缺省与 `step=0` 是两回事（前者报 missing，后者报 invalid）。
//
// v2 新增的 `delta` / `xyz` 是**矢量**参数（长度必须恰好 3），沿用同一条规则：
// 切片字段用 nil 表示"没给"，用长度 ≠3 表示"给了但形状不对"。
// 这两个字段是**纯新增** —— v1 的七个字段一个字节都没动，
// 老客户端发什么都不受影响（见 docs/protocol/tcp-xyz-v2.md §6 兼容性）。
type Command struct {
	Cmd       string   `json:"cmd"`
	Axis      string   `json:"axis,omitempty"`
	Direction string   `json:"direction,omitempty"`
	Step      *float64 `json:"step,omitempty"`
	Action    string   `json:"action,omitempty"`
	Servo     *int     `json:"servo,omitempty"`
	Angle     *float64 `json:"angle,omitempty"`
	// Delta v2 `movexyz`：相对当前**命令位姿**的三轴位移 [dx, dy, dz]（mm）。
	Delta []float64 `json:"delta,omitempty"`
	// XYZ v2 `moveto`：绝对目标点 [x, y, z]（mm，项目既有坐标系）。
	XYZ []float64 `json:"xyz,omitempty"`
}

// State 执行成功后的状态回读（§21：不新建状态系统，读现成的）。
type State struct {
	// Joints 关节角（degree），键 = robot.yaml 的关节 id
	Joints map[string]float64 `json:"joints,omitempty"`
	// Servo 舵机角，按 TCP 的 servo 编号 1..N 排列（= JointOrder 的顺序）
	Servo []float64 `json:"servo,omitempty"`
	// TCP 末端位置 [x, y, z]（mm，项目既有坐标系：+X 前 / +Y 左 / +Z 上）
	TCP []float64 `json:"tcp,omitempty"`
	// Device 链路末端（sim / serial / mujoco）—— 调用方不需要知道，但日志里要有
	Device string `json:"device,omitempty"`
}

// Caps v2 `caps` 的应答体：把**外部控制器需要知道的全部常量**一次交代清楚。
//
// 存在的理由：外部项目（如 visionflow 的手势控制）要把"手的归一化坐标"映射到
// "机械臂的 mm"，就必须知道可达范围与限位 —— 不给它这些，它只能把边界抄进自己
// 的代码里，那立刻就是**第二份真值**（本工程反复踩的坑）。
//
// 这里的每个数字都由 robot.yaml 派生，本文件不持任何常量。
type Caps struct {
	// Protocol 协议版本（1 = v1 四命令；2 = 含 XYZ 矢量命令）。
	Protocol int `json:"protocol"`
	// Device 链路末端（sim / serial / mujoco）—— 调用方不需要，但日志里要有。
	Device string `json:"device,omitempty"`
	// Units 单位，写死在应答里而不是靠文档约定（外部项目最容易猜错的就是单位）。
	Units struct {
		Length string `json:"length"`
		Angle  string `json:"angle"`
	} `json:"units"`
	// Frame 坐标系朝向（出处 docs/coordinate-system.md §1）。
	Frame struct {
		X      string `json:"x"`
		Y      string `json:"y"`
		Z      string `json:"z"`
		Handed string `json:"handed"`
		Origin string `json:"origin"`
	} `json:"frame"`
	// Geom 机构几何（mm），由 robot.yaml 的 links[].length 按关节角色求导。
	//
	// 外部项目可以拿它自己算 FK/IK（公式见 robot/kinematics.go 文件头），
	// 但**不要**拿它自己判可达性 —— 关节限位才是最终判据。
	Geom struct {
		PivotZ float64 `json:"pivotZ"`
		L1     float64 `json:"l1"`
		L2     float64 `json:"l2"`
		ToolR  float64 `json:"toolR"`
		// ReachMin / ReachMax 可达壳半径（距肩枢轴），= |L1−L2| .. L1+L2。
		//
		// ⚠️ 这是**腕枢轴**的可达壳，不是 TCP 的：TCP 还要再水平前伸 ToolR
		//    （腕是被动关节，爪被四连杆锁成水平）。外部项目做钳位时用
		//    `workspace` 那一组，别自己从这两个数推。
		ReachMin float64 `json:"reachMin"`
		ReachMax float64 `json:"reachMax"`
	} `json:"geom"`
	// Joints 关节软限位（degree），顺序 = `Model.JointOrder()`（= JR 的位次）。
	Joints []CapsJoint `json:"joints"`
	// Servos 舵机硬件行程（degree），`n` 与 `servo` 命令的编号一致（1..N）。
	Servos []CapsServo `json:"servos"`
	// Home HOME 位姿（关节角）。
	Home map[string]float64 `json:"home,omitempty"`
	// Workspace TCP 可达包围盒（mm），由关节限位**采样 FK** 求出，不是解析解。
	//
	// 它是"能在哪动"的**外边界**，用来给手势坐标做粗钳位（超出就一定不可达）；
	// 盒内仍可能不可达（壳层是甜甜圈状），最终判据永远是命令的成败。
	Workspace struct {
		Min [3]float64 `json:"min"`
		Max [3]float64 `json:"max"`
		// Samples 采样网格每轴的份数（外部项目复现时用同一份数才能对齐）。
		Samples int `json:"samples"`
	} `json:"workspace"`
	// Commands 本服务支持的命令名（外部项目据此做能力探测）。
	Commands []string `json:"commands"`
}

// CapsJoint 一个关节的限位摘要。
type CapsJoint struct {
	ID   string  `json:"id"`
	Role string  `json:"role,omitempty"`
	Min  float64 `json:"min"`
	Max  float64 `json:"max"`
}

// CapsServo 一个舵机的行程摘要。
type CapsServo struct {
	// N 与 `servo` 命令的编号一致（1..N）。
	N int `json:"n"`
	// ID robot.yaml 的 actuators[].id（本项目是 S9 / S7 / S8 / S6）。
	ID string `json:"id,omitempty"`
	// Joint 该舵机驱动的关节 id。
	Joint string  `json:"joint"`
	Min   float64 `json:"min"`
	Max   float64 `json:"max"`
	// Unit `deg`（0..180 舵机行程）/ `joint`（关节空间，行程 ≡ 关节限位）。
	Unit string `json:"unit"`
}

// Result 每条指令的应答（也是 JSON Lines 的一行）。
type Result struct {
	OK    bool   `json:"ok"`
	Cmd   string `json:"cmd,omitempty"`
	Error string `json:"error,omitempty"`
	State *State `json:"state,omitempty"`
	// Caps 仅 `caps` 命令返回（v2 新增；v1 应答不含此字段）。
	Caps *Caps `json:"caps,omitempty"`
}

// Handler 把 Command 翻译成 Arm 的调用。
type Handler struct {
	arm   Arm
	model *robot.Model
	geom  robot.Geom

	// mu 串行化命令执行。
	//
	// 为什么需要它：多个 TCP 客户端可以同时连上来（§18），而命令执行是
	//「读当前 → 算目标 → 下发」三步 —— 交错执行会让相对位移互相踩踏，
	// 后算的那个基准是过期的。这里**不改 controller**（它有 ACK 门控与
	// latest-wins），只在 adapter 层把"一次完整命令"做成原子操作。
	mu sync.Mutex
}

// NewHandler 构造分发器。
func NewHandler(arm Arm, model *robot.Model, geom robot.Geom) *Handler {
	return &Handler{arm: arm, model: model, geom: geom}
}

// Execute 解析一行 JSON 文本并执行，返回应答。
//
// 它对**任何**输入都返回结果，不返回 error、不 panic —— 非法输入的影响范围
// 被限制在"这一条命令"内（§17）。
func (h *Handler) Execute(line string) Result {
	if strings.TrimSpace(line) == "" {
		return Result{OK: false, Error: "empty command"}
	}
	var c Command
	if err := json.Unmarshal([]byte(line), &c); err != nil {
		return Result{OK: false, Error: "invalid json"}
	}
	return h.dispatch(c)
}

// dispatch 分发一条命令。
//
// ⚠️ 三条**写**命令（move / gripper / servo）一律以 `protocol.OriginExternal` 下发 ——
// 它们来自**另一个进程**，对端页面必须"跟随"（写指令侧 + 抑制回发），
// 而不是把它们当成"本页自己发的命令"（那样只更新 Actual，画面显示错、且指令侧滞留旧值）。
// `state` 是只读查询，不碰 Apply，因此不需要来源。
func (h *Handler) dispatch(c Command) Result {
	cmd := strings.ToLower(strings.TrimSpace(c.Cmd))
	switch cmd {
	case "":
		return Result{OK: false, Error: "missing cmd"}
	case "move":
		return h.move(c)
	case "gripper":
		return h.gripper(c)
	case "servo":
		return h.servo(c)
	case "state":
		// 只读：不碰 Apply、不碰 device，因此不需要 h.mu（见文件头注释）。
		return h.readState()
	// ---- v2：XYZ 矢量命令（见 docs/protocol/tcp-xyz-v2.md） ----
	case "movexyz":
		return h.movexyz(c)
	case "moveto":
		return h.moveto(c)
	case "home":
		return h.home()
	case "caps":
		return h.caps()
	default:
		return Result{OK: false, Cmd: cmd, Error: fmt.Sprintf("unknown cmd %q", c.Cmd)}
	}
}

// readState 回当前状态，供外部控制器建立基准 / 同步界面。
//
// 唯一可能与"没状态"撞上的时刻是 `controller.Snapshot()` 返回空 map —— 实际
// 不可达（`controller.New` 把 lastCommand 初始化为 HomePose），但仍要给出可读的
// 错误而不是 `ok:true` 配一个空 state：调用方据此区分"状态还没准备好"与"读到了"。
func (h *Handler) readState() Result {
	st := h.state()
	if st == nil {
		return Result{OK: false, Cmd: "state", Error: "no state available yet"}
	}
	return Result{OK: true, Cmd: "state", State: st}
}

// move XYZ 相对位移。
//
// 基准取 `Snapshot()` 的**命令值**（不是状态值）：
// 状态值来自设备回读，命令刚下发时它还在斜坡上，用它做基准会让连续
// +X 逐步"追不上"（每一次都从半路的读数再往前加）。这与前端
// "能不能跟随 Actual" 是同源的坑 —— 相对运动必须基于目标，不能基于过程。
func (h *Handler) move(c Command) Result {
	axis := strings.ToLower(strings.TrimSpace(c.Axis))
	if axis != "x" && axis != "y" && axis != "z" {
		return Result{OK: false, Cmd: "move", Error: "invalid axis"}
	}
	var sign float64
	switch strings.TrimSpace(c.Direction) {
	case "+":
		sign = 1
	case "-":
		sign = -1
	default:
		return Result{OK: false, Cmd: "move", Error: "invalid direction"}
	}
	if c.Step == nil {
		return Result{OK: false, Cmd: "move", Error: "missing step"}
	}
	step := *c.Step
	if math.IsNaN(step) || math.IsInf(step, 0) || step <= 0 {
		return Result{OK: false, Cmd: "move", Error: "invalid step"}
	}

	// ⚠️ 参数校验（上面四步）**必须**全部先于取快照：错误优先级是 v1 就定下的
	//    契约（axis → direction → step 缺失 → step 非法 → 无位姿 → IK），
	//    改顺序会让"给了非法 axis 却报 OUT_OF_WORKSPACE"这类回归溜过去。
	h.mu.Lock()
	defer h.mu.Unlock()

	cur, _ := h.arm.Snapshot()
	if len(cur) == 0 {
		return Result{OK: false, Cmd: "move", Error: "no current pose yet"}
	}
	target := h.geom.FK(cur)
	switch axis {
	case "x":
		target.X += sign * step
	case "y":
		target.Y += sign * step
	case "z":
		target.Z += sign * step
	}
	return h.applyTargetLocked("move", cur, target)
}

// applyTarget 「目标点 → IK → Apply」的公共尾巴（**会加锁**）。
//
// v2 的 movexyz / moveto 与 v1 的 move 走的是同一条尾巴，只是"目标点怎么来的"
// 不同。刻意让 move 也走这里：单轴 move 与 movexyz 单轴分量必须**逐位一致**，
// 靠"两套代码各自对"是保证不了的（判据见 protocol_xyz_test.go 的等价性测试）。
func (h *Handler) applyTarget(cmd string, target robot.Vec3) Result {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.applyTargetLocked(cmd, h.currentPose(), target)
}

// applyTargetLocked 同上，但**假定调用者已持有 h.mu**。
//
// 拆出这个形态是因为"取当前位姿 → 由它算出目标"这两步本身也必须在锁内 ——
// 合成一个函数就没法让调用方在锁内先算目标了。
func (h *Handler) applyTargetLocked(cmd string, cur map[string]float64, target robot.Vec3) Result {
	if len(cur) == 0 {
		return Result{OK: false, Cmd: cmd, Error: "no current pose yet"}
	}
	sol, ierr := h.geom.SolveIK(target, cur, h.model.Validate)
	if ierr != nil {
		// 原因透传（OUT_OF_WORKSPACE / JOINT_LIMIT），调用方据此分支。
		return Result{OK: false, Cmd: cmd, Error: ierr.Reason + ": " + ierr.Message}
	}
	if err := h.arm.ApplyFrom(protocol.OriginExternal, sol); err != nil {
		return Result{OK: false, Cmd: cmd, Error: err.Error()}
	}
	return h.ok(cmd)
}

// currentPose 取当前**命令位姿**（= 最近一次被受理的目标），空则 nil。
func (h *Handler) currentPose() map[string]float64 {
	cur, _ := h.arm.Snapshot()
	return cur
}

// movexyz v2：三轴**同时**相对位移。
//
// ```json
// {"cmd":"movexyz","delta":[5,0,-2]}
// ```
//
// 与发三条 `move` 的区别不只是"少两趟往返"：
//
//   - 三次 move 是三次「读当前 → IK → Apply」，第二次的基准已经是第一次下发后的
//     命令值 —— 串行叠加在**限位边界附近**会逐步走到一个单次直达不会选的分支；
//   - 手势控制这类"每帧一个增量"的场景，一次到位才谈得上实时。
//
// 语义与 `move` 完全同源：基准是命令值不是设备过程值，失败原因同样透传。
func (h *Handler) movexyz(c Command) Result {
	d, err := vec3Arg(c.Delta, "delta", true)
	if err != nil {
		return Result{OK: false, Cmd: "movexyz", Error: err.Error()}
	}

	h.mu.Lock()
	defer h.mu.Unlock()

	cur := h.currentPose()
	if len(cur) == 0 {
		return Result{OK: false, Cmd: "movexyz", Error: "no current pose yet"}
	}
	t := h.geom.FK(cur)
	return h.applyTargetLocked("movexyz", cur, robot.Vec3{X: t.X + d[0], Y: t.Y + d[1], Z: t.Z + d[2]})
}

// moveto v2：绝对目标点。
//
// ```json
// {"cmd":"moveto","xyz":[120,0,100]}
// ```
//
// v1 刻意没有它（docs/protocol/tcp-v1.md §7 把它列为 v2）。手势控制需要它：
// "手指向某个位置"是**绝对**语义，用一串相对增量去逼近会累积漂移，
// 而且每帧都要先读一次 state 才知道基准。
func (h *Handler) moveto(c Command) Result {
	p, err := vec3Arg(c.XYZ, "xyz", false)
	if err != nil {
		return Result{OK: false, Cmd: "moveto", Error: err.Error()}
	}
	return h.applyTarget("moveto", robot.Vec3{X: p[0], Y: p[1], Z: p[2]})
}

// home v2：回 HOME 位姿。
//
// 目标取自 `Model.HomePose`（robot.yaml），不写死角度。它不是"xyz 命令"，
// 但外部控制器都要一个"归位"动作 —— 手势交互里那通常就是"张开手掌/握拳"的复位。
func (h *Handler) home() Result {
	if len(h.model.HomePose) == 0 {
		return Result{OK: false, Cmd: "home", Error: "model has no home pose"}
	}
	pose := map[string]float64{}
	for k, v := range h.model.HomePose {
		pose[k] = v
	}

	h.mu.Lock()
	defer h.mu.Unlock()
	if err := h.arm.ApplyFrom(protocol.OriginExternal, pose); err != nil {
		return Result{OK: false, Cmd: "home", Error: err.Error()}
	}
	return h.ok("home")
}

// caps v2：把外部控制器需要的常量一次交代清楚（只读）。
//
// 返回值全部由 robot.yaml 派生 —— 外部项目（visionflow）拿它做坐标映射与粗钳位，
// 就不必把限位抄进自己的代码里形成第二份真值。
func (h *Handler) caps() Result {
	return Result{OK: true, Cmd: "caps", Caps: h.buildCaps(), State: h.state()}
}

// vec3Arg 校验一个长度必须恰好为 3 的矢量参数。
//
// `rejectZero`：全零矢量是否算非法。`movexyz` 传 true（零位移 = 参数错误，
// 与 v1 `move` 的 `step > 0` 同一条规则）；`moveto` 传 false（(0,0,0) 是一个
// 合法的、虽然通常不可达的目标点，该由 OUT_OF_WORKSPACE 去说不可达，
// 而不是由参数校验去说"你这坐标不对"）。
func vec3Arg(v []float64, name string, rejectZero bool) ([3]float64, error) {
	var zero [3]float64
	if v == nil {
		return zero, fmt.Errorf("missing %s", name)
	}
	if len(v) != 3 {
		return zero, fmt.Errorf("invalid %s (expected [%s] with 3 numbers, got %d)", name, name, len(v))
	}
	for _, x := range v {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return zero, fmt.Errorf("invalid %s", name)
		}
	}
	out := [3]float64{v[0], v[1], v[2]}
	if rejectZero && out == zero {
		return zero, fmt.Errorf("invalid %s: zero displacement", name)
	}
	return out, nil
}

// buildCaps 由模型真值派生能力描述。
func (h *Handler) buildCaps() *Caps {
	c := &Caps{Protocol: protocolVersion, Device: h.arm.DeviceKind()}
	c.Units.Length = "mm"
	c.Units.Angle = "deg"
	c.Frame.X = "front"
	c.Frame.Y = "left"
	c.Frame.Z = "up"
	c.Frame.Handed = "right"
	c.Frame.Origin = "shoulder pivot axis, on the base plate plane"
	c.Geom.PivotZ = h.geom.PivotZ
	c.Geom.L1 = h.geom.L1
	c.Geom.L2 = h.geom.L2
	c.Geom.ToolR = h.geom.ToolR
	c.Geom.ReachMin = math.Abs(h.geom.L1 - h.geom.L2)
	c.Geom.ReachMax = h.geom.L1 + h.geom.L2

	for _, id := range h.model.JointOrder() {
		j := h.model.Joint(id)
		if j == nil {
			continue
		}
		c.Joints = append(c.Joints, CapsJoint{ID: id, Role: j.Role, Min: j.Limit.Min, Max: j.Limit.Max})
	}

	// 舵机条目按 `servo` 命令的编号 1..N（= JointOrder 位次）排列，
	// 与 state.servo 数组同序 —— 外部项目不必再猜哪个下标对应哪一路。
	for n, id := range h.model.JointOrder() {
		acts := h.model.ActuatorsForJoint(id)
		if len(acts) == 0 {
			continue
		}
		a := acts[0]
		c.Servos = append(c.Servos, CapsServo{
			N: n + 1, ID: a.ID, Joint: id,
			Min: a.Limits.Min, Max: a.Limits.Max, Unit: a.UnitOf(),
		})
	}

	if len(h.model.HomePose) > 0 {
		c.Home = map[string]float64{}
		for k, v := range h.model.HomePose {
			c.Home[k] = v
		}
	}

	c.Workspace.Min, c.Workspace.Max = h.workspaceBounds(workspaceSamples)
	c.Workspace.Samples = workspaceSamples
	c.Commands = supportedCommands()
	return c
}

// workspaceBounds 由关节限位**采样 FK** 求 TCP 可达包围盒。
//
// 为什么是采样而不是解析：可达域是被关节限位切过的甜甜圈壳层，解析边界要分段
// 讨论限位是否binding，写错还不会报错。采样只在启动时/`caps` 时算一次，
// 成本无所谓，而且它天然包含限位。
//
// ⚠️ 它是**外边界**：盒内仍可能不可达（壳层中空）。用途是给手势坐标做粗钳位，
// 真正判据永远是命令的 ok 字段。
func (h *Handler) workspaceBounds(n int) (min, max [3]float64) {
	min = [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}
	max = [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)}

	// 只取参与定位的三个关节做网格（base / shoulder / elbow）；
	// 夹爪与被动腕不影响 TCP 位置，进网格只会白白放大采样量。
	type axis struct {
		id     string
		lo, hi float64
	}
	var grid []axis
	for _, id := range h.model.JointOrder() {
		if id != robot.JointBase && id != robot.JointShoulder && id != robot.JointElbow {
			continue
		}
		j := h.model.Joint(id)
		if j == nil {
			continue
		}
		grid = append(grid, axis{id: id, lo: j.Limit.Min, hi: j.Limit.Max})
	}
	if len(grid) == 0 {
		return [3]float64{}, [3]float64{}
	}

	pose := map[string]float64{}
	var walk func(i int)
	walk = func(i int) {
		if i == len(grid) {
			p := h.geom.FK(pose)
			for k, v := range [3]float64{p.X, p.Y, p.Z} {
				if v < min[k] {
					min[k] = v
				}
				if v > max[k] {
					max[k] = v
				}
			}
			return
		}
		g := grid[i]
		if n <= 1 {
			pose[g.id] = (g.lo + g.hi) / 2
			walk(i + 1)
			return
		}
		for s := 0; s < n; s++ {
			pose[g.id] = g.lo + (g.hi-g.lo)*float64(s)/float64(n-1)
			walk(i + 1)
		}
	}
	walk(0)
	return min, max
}

// gripper 开合。
//
// 端点取自该关节的**限位**（robot.yaml），不写死角度：
// θ=0 闭合、+θ 张开（docs/coordinate-system.md §3），故 open=max / close=min。
func (h *Handler) gripper(c Command) Result {
	action := strings.ToLower(strings.TrimSpace(c.Action))
	if action == "" {
		return Result{OK: false, Cmd: "gripper", Error: "missing action"}
	}
	if action != "open" && action != "close" {
		return Result{OK: false, Cmd: "gripper", Error: "invalid action"}
	}

	jointID, limit := h.jointLimitByRole("gripper")
	if jointID == "" {
		return Result{OK: false, Cmd: "gripper", Error: "model has no gripper joint"}
	}
	v := limit.Max
	if action == "close" {
		v = limit.Min
	}

	h.mu.Lock()
	defer h.mu.Unlock()
	if err := h.arm.ApplyFrom(protocol.OriginExternal, map[string]float64{jointID: v}); err != nil {
		return Result{OK: false, Cmd: "gripper", Error: err.Error()}
	}
	return h.ok("gripper")
}

// servo 直接给某个舵机一个角度。
//
// 编号 1..N 按 `Model.JointOrder()`（= 前端 movableJoints 的顺序，也是 JR 的位次）。
// 本机型：1=base(S9) 2=shoulder(S7) 3=elbow(S8) 4=gripper(S6)。
//
// 校验**复用** `Actuator.Limits`（舵机硬件行程），再经 `ServoToJoint` 换成关节角
// 交给 `Apply` —— 于是关节限位仍由 controller 兜底。两套限位都来自 robot.yaml。
func (h *Handler) servo(c Command) Result {
	if c.Servo == nil {
		return Result{OK: false, Cmd: "servo", Error: "missing servo"}
	}
	n := *c.Servo
	order := h.model.JointOrder()
	if n < 1 || n > len(order) {
		return Result{OK: false, Cmd: "servo", Error: fmt.Sprintf("invalid servo (expected 1..%d)", len(order))}
	}
	if c.Angle == nil {
		return Result{OK: false, Cmd: "servo", Error: "missing angle"}
	}
	angle := *c.Angle
	if math.IsNaN(angle) || math.IsInf(angle, 0) {
		return Result{OK: false, Cmd: "servo", Error: "invalid angle"}
	}

	jointID := order[n-1]
	acts := h.model.ActuatorsForJoint(jointID)
	if len(acts) == 0 {
		return Result{OK: false, Cmd: "servo", Error: "invalid servo"}
	}
	a := acts[0]

	theta := angle
	if !a.IsJointSpace() {
		// 舵机空间：先过舵机硬件行程，再换算成关节角。
		if angle < a.Limits.Min-1e-9 || angle > a.Limits.Max+1e-9 {
			return Result{OK: false, Cmd: "servo", Error: fmt.Sprintf("angle out of range (%s %.2f..%.2f)",
				a.ID, a.Limits.Min, a.Limits.Max)}
		}
		theta = robot.ServoToJoint(a, angle)
	}

	h.mu.Lock()
	defer h.mu.Unlock()
	if err := h.arm.ApplyFrom(protocol.OriginExternal, map[string]float64{jointID: theta}); err != nil {
		return Result{OK: false, Cmd: "servo", Error: err.Error()}
	}
	return h.ok("servo")
}

// jointLimitByRole 按角色找关节与其限位。
func (h *Handler) jointLimitByRole(role string) (string, robot.Limit) {
	for i := range h.model.Joints {
		j := &h.model.Joints[i]
		if j.Role == role {
			return j.ID, j.Limit
		}
	}
	return "", robot.Limit{}
}

func (h *Handler) ok(cmd string) Result {
	return Result{OK: true, Cmd: cmd, State: h.state()}
}

// state 读现有状态回传。
//
// 关节角来自 `Snapshot()` 的命令值（刚下发的目标，不是设备过程值），
// 舵机角用 `JointToServo` 由它换算 —— 不另开一套状态缓存。
func (h *Handler) state() *State {
	cmd, _ := h.arm.Snapshot()
	if len(cmd) == 0 {
		return nil
	}
	st := &State{Joints: cmd, Device: h.arm.DeviceKind()}

	servo := make([]float64, 0, len(cmd))
	for _, id := range h.model.JointOrder() {
		v, ok := cmd[id]
		if !ok {
			continue
		}
		acts := h.model.ActuatorsForJoint(id)
		if len(acts) == 0 {
			continue
		}
		servo = append(servo, robot.JointToServo(acts[0], v))
	}
	if len(servo) > 0 {
		st.Servo = servo
	}
	tcp := h.geom.FK(cmd)
	st.TCP = []float64{tcp.X, tcp.Y, tcp.Z}
	return st
}
