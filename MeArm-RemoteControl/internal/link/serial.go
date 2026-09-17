package link

// serial.go —— 原有真机链路（arm-device 串口协议）的 Link 实现。
//
// ⚠️ 本文件的内容**逐字迁自** `internal/web/server.go` 的 `handleWS`
// （在引入 Link 抽象之前，摇杆与指令的处理就内联在那里）。
// 迁移只改"在哪调用"，不改任何行为：
//
//	摇杆：四轴全居中 → 整帧跳过（串口零流量）→ JOY 四轴帧
//	指令：protocol.Validate 校验 → 归一化 → 下发（带原样的日志）
//
// 串口模式**不支持**任何网络专属指令（XYZ / 爪 / 直接舵机角），
// 一律返回 ErrUnsupported —— 明确失败优于静默无反应。

import (
	"log"

	"arm-web/internal/protocol"
	"arm-web/internal/serial"
)

type serialLink struct {
	ser     *serial.Serial
	axisMap protocol.AxisMap
}

// NewSerial 构造串口通道。
func NewSerial(ser *serial.Serial, axisMap protocol.AxisMap) Link {
	return &serialLink{ser: ser, axisMap: axisMap}
}

// Joy 双摇杆 → 一条 `JOY <r9> <r8> <r6> <r7>` 四轴帧。
func (l *serialLink) Joy(lx, ly, rx, ry float64) error {
	// 四轴全部在动作死区内时整帧跳过：不下发任何指令（串口零流量）。
	if !protocol.JoystickHasCommand(lx, ly, rx, ry, l.axisMap) {
		return nil
	}
	return l.ser.WriteLine(protocol.JoystickToJOYDual(lx, ly, rx, ry, l.axisMap))
}

// Raw 一条 arm-device 文本指令。
func (l *serialLink) Raw(raw string) error {
	ok, normalized, verr := protocol.Validate(raw)
	if !ok {
		return verr
	}
	log.Printf("[web] 指令 %q -> 下发(归一化) %q", raw, normalized)
	return l.ser.WriteLine(normalized)
}

func (l *serialLink) XYZ(string, string, float64) error { return ErrUnsupported }
func (l *serialLink) Gripper(string) error              { return ErrUnsupported }
func (l *serialLink) ServoDirect(int, float64) error    { return ErrUnsupported }
func (l *serialLink) RefreshState() error               { return ErrUnsupported }

func (l *serialLink) Caps() Caps {
	return Caps{Mode: ModeSerial, ArmDeviceCmds: true}
}

func (l *serialLink) Status() Status {
	c, e, ce, cem := l.ser.Status()
	return Status{Connected: c, Err: e, CommErr: ce, CommErrMsg: cem}
}

// Close 空实现：串口的生命周期由装配方（main）负责关闭，
// 这里不重复持有所有权。
func (l *serialLink) Close() {}
