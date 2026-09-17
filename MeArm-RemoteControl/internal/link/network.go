package link

// network.go —— 网络模式（TCP Client → MeArm-3D TCP 控制接口）的 Link 实现。
//
// 它只是 `internal/netlink` 的一层薄适配：做参数校验与"能力差异"的表达，
// 不含任何控制逻辑。摇杆 → 舵机角的换算在 netlink/axis.go，传输在 netlink/client.go。
//
// ⚠️ 网络模式下**不支持** arm-device 文本指令（RESET / STATUS / JOYHW …）：
//    那些是固件概念（下位机的摇杆硬件开关、红外、序列），MeArm-3D 的 TCP 协议里
//    没有对应物。要做到"网页按钮在网络模式下也有效"，需要对方新增入口 ——
//    本次刻意不做（方案 §2.1：不动 MeArm-3D 的协议面）。
//    `Caps.ArmDeviceCmds=false` 会让网页把这些按钮置灰并给出原因。

import (
	"errors"

	"arm-web/internal/netlink"
)

type netLink struct {
	cli      *netlink.Client
	servoIDs []int
}

// NewNetwork 构造网络通道。`servoIDs` 是 TCP 序号 → arm-device 舵机 id 的接线表，
// 仅用于向网页描述"服务端回的状态该怎么显示"。
func NewNetwork(cli *netlink.Client, servoIDs []int) Link {
	return &netLink{cli: cli, servoIDs: append([]int(nil), servoIDs...)}
}

// Joy 双摇杆 → netlink 的增量积分。
//
// 基准（当前舵机角）还没到位时 netlink 会返回 ErrNoBaseline —— 那是连接刚建立
// 的一瞬间，**不上报为错误**（否则拖拽会把界面错误行刷满），由连接状态条体现。
func (l *netLink) Joy(lx, ly, rx, ry float64) error {
	err := l.cli.Joy(netlink.Frame{LX: lx, LY: ly, RX: rx, RY: ry})
	if errors.Is(err, netlink.ErrNoBaseline) {
		return nil
	}
	return err
}

// Raw 网络模式没有 arm-device 文本指令的对应物 —— 明确拒绝。
func (l *netLink) Raw(string) error { return ErrUnsupported }

func (l *netLink) XYZ(axis, direction string, step float64) error {
	return l.cli.Xyz(axis, direction, step)
}

func (l *netLink) Gripper(action string) error { return l.cli.Gripper(action) }

func (l *netLink) ServoDirect(n int, angle float64) error { return l.cli.Servo(n, angle) }

func (l *netLink) RefreshState() error { return l.cli.Sync() }

func (l *netLink) Caps() Caps {
	return Caps{
		Mode:          ModeNetwork,
		ArmDeviceCmds: false,
		XYZ:           true,
		Gripper:       true,
		ServoDirect:   true,
		RefreshState:  true,
		ServoIDs:      append([]int(nil), l.servoIDs...),
	}
}

func (l *netLink) Status() Status {
	c, e := l.cli.Status()
	return Status{Connected: c, Err: e}
}

// Close 空实现：netlink.Client 的生命周期由装配方负责。
func (l *netLink) Close() {}
