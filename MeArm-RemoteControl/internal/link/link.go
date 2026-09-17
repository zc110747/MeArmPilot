// Package link 是网页层与控制通道之间的**最小适配层**。
//
// 引入它的唯一理由是让同一套网页摇杆能换传输：
//
//	web (WS) ──▶ link.Link ──┬── serialLink : arm-device 文本协议（原有真机链路）
//	                          └── netLink    : JSON Lines → MeArm-3D TCP 控制接口
//
// # 它**不是**重构
//
// 串口实现（`serialLink`）里的每一行原本就写在 `internal/web/server.go` 的
// `handleWS` 里，这里只是**原样搬家**，语义、顺序、日志一字未改 ——
// 串口链路的行为必须逐字节保持不变。
//
// 网络实现（`netLink`）只做转发，不含任何运动学（见 `internal/netlink`）。
//
// 两个实现的能力差异通过 `Caps` 显式暴露给网页，而不是让网页猜。
package link

import "errors"

// ErrUnsupported 当前通道不支持该操作。
//
// 设计原则：**明确报错，不静默成功**。网页据此给出可读提示，
// 而不是让用户以为"点了没反应 = 坏了"。
var ErrUnsupported = errors.New("当前通道不支持该指令")

// 运行模式标识（同时用于网页显示）。
const (
	ModeSerial  = "serial"
	ModeNetwork = "network"
)

// Caps 通道能力 —— 决定网页显示哪些控件、标签怎么写。
type Caps struct {
	// Mode 通道类型（serial / network）。
	Mode string `json:"mode"`
	// ArmDeviceCmds 是否支持 arm-device 文本指令（RESET / STATUS / JOYHW …）。
	// 这些是**固件概念**，网络模式下没有对应物，故为 false。
	ArmDeviceCmds bool `json:"arm_device_cmds"`
	// XYZ 是否支持 XYZ 相对位移。
	XYZ bool `json:"xyz"`
	// Gripper 是否支持夹爪开合。
	Gripper bool `json:"gripper"`
	// ServoDirect 是否支持"直接给某个舵机一个绝对角"。
	ServoDirect bool `json:"servo_direct"`
	// RefreshState 是否支持主动拉取状态。
	RefreshState bool `json:"refresh_state"`
	// ServoIDs TCP 舵机编号 1..N → arm-device 舵机 id（网页把服务端状态
	// 显示成 S6..S9 时要用）。串口模式为空。
	ServoIDs []int `json:"servo_ids,omitempty"`
}

// Status 连接状态（与既有的 serial_status 消息字段一一对应）。
type Status struct {
	Connected  bool
	Err        string
	CommErr    bool
	CommErrMsg string
}

// Link 是网页层对"控制下发通道"的全部认知。
//
// 接口刻意保持窄：网页**不能**通过它碰机械臂的几何、限位或标定 ——
// 那些只在链路末端（固件 / MeArm-3D）手里。
type Link interface {
	// Joy 提交一帧双摇杆输入（归一化坐标 ∈[-1,1]，左/右各 x,y）。
	Joy(lx, ly, rx, ry float64) error
	// Raw 下发一条 arm-device 文本指令（仅串口模式）。
	Raw(cmd string) error
	// XYZ 相对位移（仅网络模式）。
	XYZ(axis, direction string, step float64) error
	// Gripper 夹爪开合（仅网络模式）。
	Gripper(action string) error
	// ServoDirect 直接给某个舵机一个绝对角（仅网络模式）。
	// `n` 是 **TCP 编号 1..N**，不是 arm-device 的舵机 id。
	ServoDirect(n int, angle float64) error
	// RefreshState 主动拉一次链路末端状态（仅网络模式）。
	RefreshState() error

	// Caps 通道能力。
	Caps() Caps
	// Status 连接状态。
	Status() Status
	// Close 释放本 Link 持有的资源（**不含**底层连接本身，那由装配方负责）。
	Close()
}
