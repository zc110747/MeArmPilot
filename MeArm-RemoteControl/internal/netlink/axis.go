// package netlink 是 MeArm-RemoteControl 的 **TCP 传输**实现：把网页摇杆的
// 控制意图通过 JSON Lines 送到 MeArm-3D 的 TCP 控制接口（默认 127.0.0.1:9100）。
//
// 分层（与 internal/serial 严格对称 —— 只是"串口"换成"TCP socket"）：
//
//	web (WS)  →  netlink.Client  →  TCP  →  MeArm-3D tcpserver → controller.Apply
//
// # 本包**只做传输与接口适配**，不做机械臂的事
//
// 这里**没有**、也不允许出现：IK / FK、坐标变换、关节↔舵机的标定换算、
// 软限位逻辑、Sim / Real 判断。它只做两件本分内的事：
//
//  1. 一行 JSON 的收发（连接、重连、读写、超时、串行化）；
//  2. 把"网页摇杆的归一化坐标"翻译成 MeArm-3D **已有**的 `servo` 命令参数
//     —— 也就是一个**速度 → 角度积分**的适配层，而不是运动学。
//
// # 为什么要"积分"而不是直接映射
//
// 串口链路的 `JOY <r9> <r8> <r6> <r7>` 是**增量**语义：固件按 raw 偏离中位的
// 幅度决定这一拍走多少度。MeArm-3D 的 `servo` 是**绝对角**语义。两者中间必须
// 有人持有"当前目标角"，本包就是那个人（见 axis.go）。
//
// ⚠️ 上限值域（`ServoMin/Max`）**不是软限位** —— 真正的硬件行程只在
//    `robot.yaml` 的 `actuators[].limits` 里，由 MeArm-3D 的 `servo` 命令把关。
//    这里的 clamp 只是"别把 NaN / 离谱数字发到线上"的数值保护。
package netlink

import "math"

// AxisMap 把网页双摇杆映射到 MeArm-3D 的舵机编号。
//
// 编号语义：**1..N 按 `Model.JointOrder()`**（MeArm-3D docs/tcp-control-v1.md §3.3）。
// 本机型为 `1=base(S9) 2=shoulder(S7) 3=elbow(S8) 4=gripper(S6)`；
// 而网页摇杆的既有权重是 arm-device 的舵机 id（`9=底座 8=左舵 6=夹取 7=右舵`，
// 见 internal/protocol/AxisMap），所以两者之间需要一张**接线表**。
//
// 方向语义与串口模式保持**完全一致**：`invert_*` 的取值直接复用
// `joystick` 配置段 —— 网页上"推杆 = 角度增大"的手感在两种模式下不该变。
type AxisMap struct {
	// 各轴对应的 TCP 舵机编号（1..N）。
	LXServo int // 左摇杆 X
	LYServo int // 左摇杆 Y
	RXServo int // 右摇杆 X
	RYServo int // 右摇杆 Y

	InvLX bool
	InvLY bool
	InvRX bool
	InvRY bool

	// DeadFrac 动作死区（占半程的比例，0..1）。|v| <= DeadFrac 的轴视为居中、
	// 既不累加也不下发 —— 与串口模式共用同一个 `deadband_deg` 换算结果。
	DeadFrac float64

	// MinSpeedDegPerS / MaxSpeedDegPerS 舵机角速度区间（度/秒）。
	// 刚越过死区取 Min，满偏取 Max；中间线性。
	MinSpeedDegPerS float64
	MaxSpeedDegPerS float64
}

// DefaultAxisMap 返回与 MeArm-3D mearm-v1 机型一致的默认接线表。
//
// 对照关系（来源：`MeArm-3D/config/robots.yaml` → robot.yaml 的
// `actuators[].channel` 与 `Model.JointOrder()` 的顺序）：
//
//	左 X → S9 底座    → servo 1
//	左 Y → S8 左舵    → servo 3
//	右 X → S6 夹取    → servo 4
//	右 Y → S7 右舵    → servo 2
func DefaultAxisMap() AxisMap {
	return AxisMap{
		LXServo: 1, LYServo: 3, RXServo: 4, RYServo: 2,
		DeadFrac:        0,
		MinSpeedDegPerS: 12,
		MaxSpeedDegPerS: 90,
	}
}

// Frame 一帧摇杆输入（归一化坐标，∈[-1,1]；0 = 居中）。
type Frame struct {
	LX, LY, RX, RY float64
}

// Centered 报告四轴是否全部在死区内（= 无需下发）。
func (m AxisMap) Centered(f Frame) bool {
	for _, v := range [4]float64{f.LX, f.LY, f.RX, f.RY} {
		if a := math.Abs(v); a > m.DeadFrac {
			return false
		}
	}
	return true
}

// axisSpeed 单轴的角速度（度/秒，带符号）。返回 0 表示该轴不产生运动。
//
// 映射：先过死区，再把剩余行程线性铺到 [MinSpeed, MaxSpeed]。
// 这样"刚出死区"就是一个可见的小速度、"推到底"最快，与硬件摇杆的体感一致。
func (m AxisMap) axisSpeed(v float64, inv bool) float64 {
	if inv {
		v = -v
	}
	a := math.Abs(v)
	if a <= m.DeadFrac {
		return 0
	}
	den := 1 - m.DeadFrac
	if den <= 0 { // 死区被配成 1 ⇒ 永远居中（防御，不让它除零）
		return 0
	}
	u := (a - m.DeadFrac) / den
	if u > 1 {
		u = 1
	}
	sp := m.MinSpeedDegPerS + u*(m.MaxSpeedDegPerS-m.MinSpeedDegPerS)
	if v < 0 {
		sp = -sp
	}
	return sp
}

// Deltas 按 dt（秒）把一帧摇杆输入积分成**每个舵机编号**的角度增量。
//
// 只返回非零项 —— 调用方据此只发"确实要动"的舵机（静止轴零流量）。
func (m AxisMap) Deltas(f Frame, dt float64) map[int]float64 {
	if dt <= 0 {
		return nil
	}
	out := map[int]float64{}
	add := func(servo int, v float64, inv bool) {
		if servo < 1 {
			return
		}
		sp := m.axisSpeed(v, inv)
		if sp == 0 {
			return
		}
		out[servo] += sp * dt
	}
	add(m.LXServo, f.LX, m.InvLX)
	add(m.LYServo, f.LY, m.InvLY)
	add(m.RXServo, f.RX, m.InvRX)
	add(m.RYServo, f.RY, m.InvRY)
	if len(out) == 0 {
		return nil
	}
	return out
}

// ServoCount 舵机总数（取接线表里最大的编号）。
//
// 它只用于**分配数组长度**，不参与任何语义判断。
func (m AxisMap) ServoCount() int {
	n := 1
	for _, v := range [4]int{m.LXServo, m.LYServo, m.RXServo, m.RYServo} {
		if v > n {
			n = v
		}
	}
	return n
}

// clampAngle 值域保护（**不是**软限位，见包注释）。
func clampAngle(v, lo, hi float64) float64 {
	if math.IsNaN(v) || math.IsInf(v, 0) {
		return lo
	}
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
