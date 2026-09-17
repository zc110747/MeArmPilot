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
// ## 三条写命令一律声明来源 `protocol.OriginExternal`
//
// TCP 客户端是**另一个进程**（如 MeArm-RemoteControl）。它的命令会经
// controller 广播给 MeArm-3D 自己的页面，而那台页面靠帧上的 `origin` 决定
// "要不要让指令侧跟随"：`command` = "本页自己发的，实际位姿在追" ⇒ 只更新 Actual；
// 外部驱动必须让它**跟随**（否则画面上只有半透明的实际臂在动、主臂不动，
// 且那台页面的指令侧停在旧值，用户下次动本页控件会把整组旧指令下发）。
// 所以这里用 `ApplyFrom` 显式声明来源，而不是图省事借用 `Apply`。
//
// 三条命令（v1 刻意最小化）：
//
//	move     XYZ 相对位移 → 当前位姿(FK) → 目标点 → IK → 关节角 → Apply
//	gripper  open / close → 取该关节限位的端点 → Apply
//	servo    1..4 直接给舵机角 → 舵机硬件限位校验 → 舵机角转关节角 → Apply
//
// 外加一条**只读**命令（v1.1 补，见 docs/tcp-control-v1.md §3.4）：
//
//	state    无参数、无副作用 → 直接回当前 state → 不调用 Apply
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

// Command 一条 TCP 指令（JSON Lines 的一行）。
//
// 指针字段用来区分"没给"与"给了 0"：
// `step` 缺省与 `step=0` 是两回事（前者报 missing，后者报 invalid）。
type Command struct {
	Cmd       string   `json:"cmd"`
	Axis      string   `json:"axis,omitempty"`
	Direction string   `json:"direction,omitempty"`
	Step      *float64 `json:"step,omitempty"`
	Action    string   `json:"action,omitempty"`
	Servo     *int     `json:"servo,omitempty"`
	Angle     *float64 `json:"angle,omitempty"`
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

// Result 每条指令的应答（也是 JSON Lines 的一行）。
type Result struct {
	OK    bool   `json:"ok"`
	Cmd   string `json:"cmd,omitempty"`
	Error string `json:"error,omitempty"`
	State *State `json:"state,omitempty"`
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

	sol, ierr := h.geom.SolveIK(target, cur, h.model.Validate)
	if ierr != nil {
		// 原因透传（OUT_OF_WORKSPACE / JOINT_LIMIT），调用方据此分支。
		return Result{OK: false, Cmd: "move", Error: ierr.Reason + ": " + ierr.Message}
	}
	if err := h.arm.ApplyFrom(protocol.OriginExternal, sol); err != nil {
		return Result{OK: false, Cmd: "move", Error: err.Error()}
	}
	return h.ok("move")
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
