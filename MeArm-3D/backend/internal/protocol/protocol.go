// Package protocol 定义两个方向的编解码，本包**不认识机械结构**：
//
//	Browser ↔ Go      JSON（spec §二十一 / docs/serial-v1.md §5）
//	Go ↔ Device       arm-device 文本协议（JR / STATE / OK / ERR，docs/serial-v1.md §4）
//
// 分层铁律（docs/serial-v1.md §1）：本包只负责把关节角编成字节 / 把字节解回关节角，
// 不做标定、不做限位判断 —— 那是 `internal/robot`（真值）与 `internal/controller`
// （策略）的职责。协议层一旦开始"懂关节"，双份真值就回来了。
package protocol

import (
	"fmt"
	"regexp"
	"sort"
	"strconv"
	"strings"

	"armpilot/backend/internal/robot"
)

// Version 是消息 schema 版本；前端与后端必须一致，不一致直接拒绝（避免静默错解）。
const Version = 1

// ---------------------------------------------------------------------------
// 消息类型
// ---------------------------------------------------------------------------

// 浏览器 → 服务器
const (
	TypeJointCommand  = "joint_command"  // 关节角整帧命令
	TypePing          = "ping"           // 心跳
	TypeStatusRequest = "status_request" // 请求重发 hello + 设备状态
	// TypeDeviceCommand 把一行**原始设备指令**直通给链路末端（调试 / 验收用）。
	//
	// 为什么需要它：真机上"下位机被外部改动"来自硬件摇杆 / 红外遥控，
	// 仿真里对应的固件级入口是 `SET` / `JOY` —— 但它们既不是本机命令，
	// 也没有别的通道能下发。没有这条通路，"摇杆 → 界面跟随"就只能靠
	// 手动拨硬件来验证，而**仿真里根本无法复现**。
	//
	// ⚠️ 它绕过 controller 的关节限位校验（固件自己按舵机硬限位钳位），
	//    因此只应当被调试脚本 / 验收探针使用，不要接到用户界面上。
	//    每次使用都会记一条日志，便于事后回答"机械臂为什么自己动了"。
	TypeDeviceCommand = "device_command"
)

// 服务器 → 浏览器
const (
	TypeHello        = "hello"         // 握手元数据（模型 / 限位 / 标定真值）
	TypeJointState   = "joint_state"   // 关节角状态（由设备回执反算）
	TypeError        = "error"         // 错误（含固定错误码）
	TypePong         = "pong"          // 心跳应答
	TypeDeviceStatus = "device_status" // 设备（串口/sim）连接状态变更
)

// 关节状态的**来源**（`ServerMessage.Origin`）。
//
// 为什么必须把来源告诉前端（而不是让它自己猜）：
//
//	设备侧会**自主变化** —— 硬件摇杆 / 红外遥控 / 面板手拧，都不经过本机命令。
//	这类变化若只写进 Actual，界面上"命令"那一侧就永远不动（滑杆不动、主臂不动、
//	目标点不动），用户看到的是一个与现场不符的姿态，而且**看不出任何异常**。
//
// 但反过来，若让前端"看到 Actual ≠ Command 就跟随"，就会把**命令→状态→命令**
// 的回环重新引进来（拖动时设备还在斜坡上，Actual 必然落后于 Command，
// 前端会把 Command 一路拉回半路的位置）。两者都不可能靠猜区分，
// 所以在协议里**标注**：由谁引起，就由谁决定要不要跟随。
//
// ⚠️ 第三类来源必须与 `command` 分开：**其它上位机**经外部入口（TCP JSON v1，
// 将来的 MQTT / HTTP…）下发的命令走的是**与本页同一条** `Apply` 路径。
// 它若借用 `command`，对端界面会判定成"我自己发的命令、实际位姿在追"，
// 于是只更新 Actual —— 画面上表现为**半透明实际臂在动、主臂（指令）不动**；
// 更危险的是那台页面的 `commandJoints` 停在旧值，用户下一次动本页控件会把
// **整组旧指令**下发（真机上就是机械臂突然跳回旧位姿）。
// 语义上它就是"外部驱动"，前端应与 `device` 同等对待（跟随 + 抑制回发）。
const (
	OriginCommand  = "command"  // 由本机命令引起（JR 的应答 / 设备回读）
	OriginDevice   = "device"   // 设备侧自主变化（摇杆 / 红外 / 手拧 / 调试直控）
	OriginExternal = "external" // 其它上位机经外部入口（TCP JSON v1）下发的命令
)

// 错误码（前端按码分支，不要匹配 message 文本）
const (
	CodeBadMessage = "BAD_MESSAGE"
	CodeVersion    = "VERSION_MISMATCH"
	CodeJointLimit = "JOINT_LIMIT"
	CodeDeviceDown = "DEVICE_UNAVAILABLE"
	CodeInternal   = "INTERNAL"
	CodeAckTimeout = "ACK_TIMEOUT"
)

// 仿真模式（spec §25）。
//
// 三者对应 `device.Device` 的三个实现，区别是**末端发生了什么**：
//
//	kinematic  sim 设备：纯运动学 + 速率限制。关节"瞬间听话"，没有重力、没有接触。
//	mujoco     MuJoCo 刚体动力学：有重力、有接触、有有限力矩，会**压不到位**。
//	real       真机串口：物理世界（唯一的外部地面真值是相机）。
//
// ⚠️ 这不是"UI 开关"，只是一份**事实声明** —— 前端拿它决定要不要提示
//    "当前是参数化物理仿真，不是真机标定模型"（spec §37 的 Level 声明）。
const (
	SimulationKinematic = "kinematic"
	SimulationMujoco    = "mujoco"
	SimulationReal      = "real"
)

// SimulationModeFor 把 `device.Kind()` 映射成对前端友好的仿真模式。
func SimulationModeFor(deviceKind string) string {
	switch deviceKind {
	case "mujoco":
		return SimulationMujoco
	case "serial":
		return SimulationReal
	default:
		return SimulationKinematic
	}
}

// ClientMessage 是浏览器下行消息。
//
// `seq` 是本项目在 spec 字段之外加的**可选项**：拖动时命令高频变化，
// 前端用单调递增序号做去重与乱序丢弃；旧客户端不发 seq 也能工作（默认 0）。
type ClientMessage struct {
	Version   int                `json:"version"`
	Type      string             `json:"type"`
	Timestamp int64              `json:"timestamp,omitempty"`
	Seq       uint64             `json:"seq,omitempty"`
	Joints    map[string]float64 `json:"joints,omitempty"`
	// Line 是 `device_command` 的原样文本（不含行尾）。
	//
	// ⚠️ 它**不做任何校验**就交给链路末端 —— 这里的"不校验"是刻意的：
	//    校验器一旦开始理解 `SET` / `JOY` 的语义，就会长出第二份"设备认识什么"
	//    的知识，而那正是 `internal/device` 的职责。代价是它只能给调试用，
	//    绝不能接到用户界面上。
	Line string `json:"line,omitempty"`
}

// ServerMessage 是服务器上行消息（一个结构覆盖全部上行类型，未用字段省略）。
type ServerMessage struct {
	Version   int                `json:"version"`
	Type      string             `json:"type"`
	Timestamp int64              `json:"timestamp"`
	Seq       uint64             `json:"seq,omitempty"`
	Joints    map[string]float64 `json:"joints,omitempty"`
	Code      string             `json:"code,omitempty"`
	Message   string             `json:"message,omitempty"`
	Model     *ModelInfo         `json:"model,omitempty"`
	Device    string             `json:"device,omitempty"`
	Connected *bool              `json:"connected,omitempty"`
	// SimulationMode 说明末端是哪种"仿真"（spec §25）。
	//
	// ⚠️ 刻意只加这**一个可选字符串**，不加新消息类型、不改 joints 的结构：
	//    前端不认识它也照常工作（omitempty ⇒ 老前端读不到就按 kinematic 处理）。
	//    这就是 spec §2「MuJoCo 不侵入现有 Web UI」的落地方式 —— UI 零改动。
	SimulationMode string `json:"simulation_mode,omitempty"`
	// Origin 说明这帧关节状态的**来源**（`OriginCommand` / `OriginDevice`）。
	//
	// 与 SimulationMode 同样的兼容策略：可选字符串、老前端读不到就当作命令来源
	// （即"只更新 Actual，不跟随"）—— 那正是引入本字段之前的行为。
	Origin string `json:"origin,omitempty"`
}

// ModelInfo 是握手时下发的模型真值快照。前端拿它与本地 RobotModel 比对：
// 若限位/标定对不上，说明两侧读的不是同一份 robot.yaml —— 这正是
// "标定表只有一份"这条铁律的**在线校验手段**。
type ModelInfo struct {
	ID          string                 `json:"id"`
	Name        string                 `json:"name"`
	Source      string                 `json:"source"`
	JointOrder  []string               `json:"jointOrder"`
	Limits      []robot.LimitRow       `json:"limits"`
	Calibration []robot.CalibrationRow `json:"calibration"`
	HomePose    map[string]float64     `json:"homePose"`
}

// BuildModelInfo 从模型真值构造元数据。
func BuildModelInfo(m *robot.Model) *ModelInfo {
	return &ModelInfo{
		ID:   m.ID,
		Name: m.Name,
		// Source 是"真值文件在哪"的显示标签（UI / 日志用），不是定位用的路径。
		// ★ 取**加载时实际用的那个路径**（`Model.SourcePath`），不写死字符串 ——
		//   写死的话，真值随包搬迁后这里会继续报旧路径，而它看起来仍然"很合理"。
		Source:      m.SourcePath,
		JointOrder:  m.JointOrder(),
		Limits:      m.LimitTable(),
		Calibration: m.CalibrationTable(),
		HomePose:    m.HomePose,
	}
}

// ---------------------------------------------------------------------------
// 设备文本协议（arm-device）
// ---------------------------------------------------------------------------

// EncodeJR 把关节角编成 JR 整帧文本。
//
// 协议 §4 规定**保留 1 位小数**，这是文本协议的固有量化：
// 往返一次会带来 ≤0.05° 的偏差，属预期行为（不是 bug），验收断言按此设容差。
func EncodeJR(order []string, joints map[string]float64) string {
	parts := make([]string, 0, len(order)+1)
	parts = append(parts, "JR")
	for _, id := range order {
		parts = append(parts, strconv.FormatFloat(joints[id], 'f', 1, 64))
	}
	return strings.Join(parts, " ")
}

// EncodeState 把关节角编成 STATE 文本（sim 设备模拟固件回读用）。
func EncodeState(order []string, joints map[string]float64) string {
	parts := make([]string, 0, len(order)+1)
	parts = append(parts, "STATE")
	for _, id := range order {
		parts = append(parts, strconv.FormatFloat(joints[id], 'f', 2, 64))
	}
	return strings.Join(parts, " ")
}

// EncodeOKJR 编成 `OK JR S9=.. S8=.. S7=.. S6=..`。
//
// ⚠️ 它携带的是**目标角**（设备"被要求去哪"），不是实际位置 —— 别拿它当 Actual。
// 它的用途是核对标定（本地按标定算出的舵机角 vs 设备回声），客户端在
// `Controller.verifyCalibrationEcho` 里按这个语义使用。
func EncodeOKJR(servoAngles map[int]float64) string {
	return encodeServoLine("OK JR", servoAngles, "%.2f")
}

// EncodeServoReport 编成设备侧的**异步上报**行：`# SERVO S9=.. S8=.. S7=.. S6=..`。
//
// 语义（三条都要成立，否则会误用）：
//   - `#` 前缀 = 异步事件。协议 §3 已把 `#` 定为"非应答"，因此它**天然不参与
//     命令-应答门控** —— 上层不需要为它做任何特判，也不该拿它去放行某条在途命令。
//   - 它携带的是设备侧**实际**角度（固件 `arm_get_angle()` / 舵机空间的当前值），
//     与 `OK JR` 的目标角是两个不同的量。**这是它存在的全部意义**：
//     命令应答说"我要去哪"，本行说"现在在哪"。
//   - 它表达的是**设备自主变化**：摇杆、红外、面板手拧等不经由本机命令的改动。
//     因此上层据此发布的状态必须标 `OriginDevice`，让界面跟随。
//
// ⚠️ 设备不认识关节 —— 所以这里只能是舵机角，关节换算留在上层唯一那一处
//    （`ServoAnglesToJoints`），与"标定真值只有一份"这条铁律一致。
func EncodeServoReport(servoAngles map[int]float64) string {
	return encodeServoLine("# SERVO", servoAngles, "%.2f")
}

// encodeServoLine 是所有"舵机角快照行"的公共编码：按通道**降序**排列（S9 S8 S7 S6）。
//
// 固定降序而不是 map 迭代序：map 迭代序在 Go 里是随机的，同一份数据会编出
// 不同字节序的行 —— 测试会变成随机失败，抓包对照也没法做。
func encodeServoLine(prefix string, servoAngles map[int]float64, numFmt string) string {
	chans := make([]int, 0, len(servoAngles))
	for ch := range servoAngles {
		chans = append(chans, ch)
	}
	sort.Sort(sort.Reverse(sort.IntSlice(chans)))
	parts := make([]string, 0, len(chans)+1)
	parts = append(parts, prefix)
	for _, ch := range chans {
		parts = append(parts, fmt.Sprintf("S%d="+numFmt, ch, servoAngles[ch]))
	}
	return strings.Join(parts, " ")
}

var (
	reServoKV  = regexp.MustCompile(`S(\d+)=(-?\d+(?:\.\d+)?)`)
	reState    = regexp.MustCompile(`(?i)^\s*STATE\b(.*)$`)
	reErrJoint = regexp.MustCompile(`(?i)^\s*ERR\s+JOINT\s+(\w+)\s+(-?\d+(?:\.\d+)?)\s+\(limit\s+(-?\d+(?:\.\d+)?)\.\.(-?\d+(?:\.\d+)?)\)\s*$`)
)

// ReplyKind 区分设备回执的类别。
type ReplyKind int

const (
	ReplyOther ReplyKind = iota // 无法识别的行（异步事件 / 未实现命令）
	ReplyOKJR                   // OK JR S9=.. S7=.. S8=.. S6=..   ← **目标角**
	ReplyState                  // STATE <j1> <j2> <j3> <grip>
	ReplyServo                  // # SERVO S..=.. / STATUS S..=..    ← **实际角**（设备自主变化）
	ReplyError                  // ERR ...
)

func (k ReplyKind) String() string {
	switch k {
	case ReplyOKJR:
		return "OK_JR"
	case ReplyState:
		return "STATE"
	case ReplyServo:
		return "SERVO"
	case ReplyError:
		return "ERR"
	default:
		return "OTHER"
	}
}

// Reply 是解析后的一条设备回执。
type Reply struct {
	Kind ReplyKind
	Raw  string
	// ServoAngles：ReplyOKJR 时是**目标角**（核对标定用），
	// ReplyServo 时是**实际角**（设备侧自主变化，可据此更新界面）。
	// 两者语义不同，调用方必须按 Kind 分支，不可混用。
	ServoAngles map[int]float64
	Joints      []float64 // ReplyState（按 JointOrder 位次）
	ErrText     string    // ReplyError
}

// ParseReply 解析一行设备回执。
//
// ⚠️ 判定顺序**必须**是：ERR → STATE → ERR JOINT → OK JR → `#`/STATUS 舵机快照。
// 理由都在"含 `S<n>=` 的行不止一种"这一件事上：
//
//	`OK JR S9=..`  含 S=，但它携带**目标角**，必须先被判成 ReplyOKJR；
//	`OK SET S9=..` 同样含 S=，携带**目标角**，必须留在 ReplyOther（见下方白名单注释）；
//	`ERR JOINT elbow ...` 不含 S=，但 ERR 本就要最先判（它是唯一的失败信号）；
//	`STATE` 行不含 S=，先判它只是为了省掉后面的正则。
//
// 把 ReplyServo 放在最后，是因为它是唯一"靠形状而不是靠谓词"识别的类别，
// 放最后才能既覆盖到、又不会抢走前面几类的行。
func ParseReply(line string) Reply {
	trimmed := strings.TrimSpace(line)
	r := Reply{Raw: trimmed}

	if strings.HasPrefix(strings.ToUpper(trimmed), "ERR") {
		r.Kind = ReplyError
		r.ErrText = trimmed
		return r
	}
	if m := reState.FindStringSubmatch(trimmed); m != nil {
		fields := strings.Fields(m[1])
		vals := make([]float64, 0, len(fields))
		for _, f := range fields {
			v, err := strconv.ParseFloat(f, 64)
			if err != nil {
				return r // 有一个不是数字就整体不认
			}
			vals = append(vals, v)
		}
		if len(vals) > 0 {
			r.Kind = ReplyState
			r.Joints = vals
		}
		return r
	}
	if m := reErrJoint.FindStringSubmatch(trimmed); m != nil {
		r.Kind = ReplyError
		r.ErrText = trimmed
		return r
	}
	if strings.HasPrefix(strings.ToUpper(trimmed), "OK JR") {
		if kv := reServoKV.FindAllStringSubmatch(trimmed, -1); len(kv) > 0 {
			angles := make(map[int]float64, len(kv))
			for _, pair := range kv {
				ch, err1 := strconv.Atoi(pair[1])
				ang, err2 := strconv.ParseFloat(pair[2], 64)
				if err1 != nil || err2 != nil {
					continue
				}
				angles[ch] = ang
			}
			r.Kind = ReplyOKJR
			r.ServoAngles = angles
		}
		return r
	}

	// ---- 设备侧的**实际**舵机角快照 → ReplyServo ----
	//
	// 前缀是**白名单**（不是"含 S<n>= 就算"）：
	//
	//	`# ...`      固件的异步上报（`# SERVO S9=.. S8=.. S7=.. S6=..`）
	//	`STATUS ...` 查询应答（sim 的假固件；真机的 STATUS 由 serial.go 翻译成 STATE）
	//
	// ⚠️ 为什么必须是白名单：`OK SET S6=88` 的形状与它们**一模一样**，但它携带的是
	// `arm_set_angle()` 返回的**钳位后目标角**，不是实际位置。一并当 Actual 收下，
	// 就会出现「状态永远等于命令、误差恒为 0」—— 正是 `controller.handleLine` 里
	// 特意防的那件事（一个乐观 ACK 把整条"滞后 → 收敛"语义抹掉）。
	//
	// ⚠️ `# IR RAW=0x…` 这类异步事件不含 `S<n>=`，kv 为空，天然不会误入。
	if kv := reServoKV.FindAllStringSubmatch(trimmed, -1); len(kv) > 0 {
		if strings.HasPrefix(trimmed, "#") || strings.HasPrefix(strings.ToUpper(trimmed), "STATUS") {
			angles := make(map[int]float64, len(kv))
			for _, pair := range kv {
				ch, err1 := strconv.Atoi(pair[1])
				ang, err2 := strconv.ParseFloat(pair[2], 64)
				if err1 != nil || err2 != nil {
					continue
				}
				angles[ch] = ang
			}
			if len(angles) > 0 {
				r.Kind = ReplyServo
				r.ServoAngles = angles
			}
		}
	}
	return r
}

// ParseErrorCode 把设备 ERR 文本归类成前端可分支的错误码。
func ParseErrorCode(errText string) (code, message string) {
	if m := reErrJoint.FindStringSubmatch(strings.TrimSpace(errText)); m != nil {
		return CodeJointLimit, errText
	}
	return CodeInternal, errText
}

// ServoAnglesToJoints 用模型标定表把舵机角反算回关节角。
//
// 这是闭环的关键一步：设备只回舵机角（它不认识关节），后端必须自己换回来。
// 走这一步而不是"把命令原样当状态回推"，才能让标定表的**可逆性**被真实检验 ——
// 若 offset/scale/reverse 写错，Actual 会立刻偏离 Command，而不是永远相等。
func ServoAnglesToJoints(m *robot.Model, servoAngles map[int]float64) map[string]float64 {
	out := make(map[string]float64, len(m.JointOrder()))
	for _, id := range m.JointOrder() {
		acts := m.ActuatorsForJoint(id)
		if len(acts) == 0 {
			continue
		}
		// 多舵机关节取平均（spec §二十五：上层无感）
		sum, n := 0.0, 0
		for _, a := range acts {
			servo, ok := servoAngles[a.Channel]
			if !ok {
				continue
			}
			sum += robot.ServoToJoint(a, servo)
			n++
		}
		if n > 0 {
			out[id] = sum / float64(n)
		}
	}
	return out
}
