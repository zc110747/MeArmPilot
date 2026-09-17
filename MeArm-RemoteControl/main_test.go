package main

// main_test.go —— 装配层的护栏测试。
//
// 这里测的是 `deriveAxisMap`：它把配置里的两套编号 + 两个方向体系拼成一张
// netlink 接线表。**它是串口手感与网络手感之间的唯一换算点**，错了不会有
// 任何一处报错 —— 只会表现为"网页推右，机械臂往左转"。
//
// 所以本文件的重点不是"函数返回了什么"，而是把配置里的语义翻译成
// **可复算的物理断言**：同一个推杆动作，两个模式是否让同一个舵机同向转。

import (
	"testing"

	"arm-web/internal/config"
	"arm-web/internal/netlink"
)

// defaultJoystick 复刻 config.yaml 的 `joystick` 段默认值。
func defaultJoystick() config.JoystickConfig {
	return config.JoystickConfig{
		LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7,
		InvLX: true, InvLY: false, InvRX: true, InvRY: true,
		DeadbandDeg: 5,
	}
}

// defaultNetwork 复刻 config.yaml 的 `network` 段默认值。
func defaultNetwork() config.NetworkConfig {
	return config.NetworkConfig{
		Host: "127.0.0.1", Port: 9100,
		ServoIDs:        []int{9, 7, 8, 6},
		DeadbandDeg:     5,
		MinSpeedDegPerS: 12, MaxSpeedDegPerS: 90,
		TickMs: 25, MaxTickMs: 200,
	}
}

// TestDeriveAxisMap_DefaultWiring 出厂配置下，四轴应落到
// mearm-v1 的 TCP 编号（1=base 2=shoulder 3=elbow 4=gripper）上。
//
// 依据：robot.yaml 的 actuators 顺序 channel = 9,7,8,6，而 Model.JointOrder()
// 是 base/shoulder/elbow/gripper ⇒ servo_ids=[9,7,8,6] 的**位次**即 TCP 编号。
func TestDeriveAxisMap_DefaultWiring(t *testing.T) {
	m, err := deriveAxisMap(defaultJoystick(), defaultNetwork())
	if err != nil {
		t.Fatalf("deriveAxisMap: %v", err)
	}
	// 左X=9(底座) -> 位次1 ; 左Y=8(肘) -> 位次3 ; 右X=6(夹取) -> 位次4 ; 右Y=7(肩) -> 位次2
	if m.LXServo != 1 || m.LYServo != 3 || m.RXServo != 4 || m.RYServo != 2 {
		t.Errorf("接线表错位: LX=%d LY=%d RX=%d RY=%d（期望 1/3/4/2）",
			m.LXServo, m.LYServo, m.RXServo, m.RYServo)
	}
}

// TestDeriveAxisMap_DefaultInvertIsAllFalse 出厂配置下方向标志应全为 false。
//
// 这不是"照抄公式"，而是**串口语义的直接推论**：config.yaml 的 invert 默认值
// （9/6/7 为 true）是为了抵消固件的方向差异（MeArm-Device/core/joystick.c），
// 使"推杆正方向 → 角度增大"成立；TCP 链路没有固件这一层，抵消项必须剥掉。
func TestDeriveAxisMap_DefaultInvertIsAllFalse(t *testing.T) {
	m, err := deriveAxisMap(defaultJoystick(), defaultNetwork())
	if err != nil {
		t.Fatalf("deriveAxisMap: %v", err)
	}
	if m.InvLX || m.InvLY || m.InvRX || m.InvRY {
		t.Errorf("方向标志应为全 false（推杆正方向 = 角度增大），实际 LX=%v LY=%v RX=%v RY=%v",
			m.InvLX, m.InvLY, m.InvRX, m.InvRY)
	}
}

// TestDeriveAxisMap_PushDirectionMatchesSerial 是本次改造最关键的一致性断言。
//
// 串口侧（internal/protocol）在出厂 invert 下，推杆正方向会让舵机角度**增大**
// —— 证据见 protocol_test.go 的 TestDeadbandInverted：推 + 满偏得到 raw=0，
// 而固件 core/joystick.c 对 id 9/6/7 在 raw<200 时给**正**步长。
//
// 网络侧必须同向。这里用 netlink 的积分函数复算：dt=1s、满偏 ⇒ 增量应 > 0。
func TestDeriveAxisMap_PushDirectionMatchesSerial(t *testing.T) {
	m, err := deriveAxisMap(defaultJoystick(), defaultNetwork())
	if err != nil {
		t.Fatalf("deriveAxisMap: %v", err)
	}
	cases := []struct {
		name  string
		frame netlink.Frame
		servo int
	}{
		{"左摇杆X 正偏 -> 底座(TCP#1) 角度增大", netlink.Frame{LX: 1}, 1},
		{"左摇杆Y 正偏 -> 肘(TCP#3)   角度增大", netlink.Frame{LY: 1}, 3},
		{"右摇杆X 正偏 -> 夹取(TCP#4) 角度增大", netlink.Frame{RX: 1}, 4},
		{"右摇杆Y 正偏 -> 肩(TCP#2)   角度增大", netlink.Frame{RY: 1}, 2},
	}
	for _, c := range cases {
		d := m.Deltas(c.frame, 1.0)
		got, ok := d[c.servo]
		if !ok {
			t.Errorf("%s: 该轴没有产生增量（死区/接线错误？）", c.name)
			continue
		}
		if got <= 0 {
			t.Errorf("%s: 增量为 %+.2f°，应为正 —— 与串口模式推杆方向相反", c.name, got)
		}
		// 反向推杆必须同幅反向（对称性）
		if back := m.Deltas(c.frame, 1.0); back[c.servo] != got {
			t.Errorf("%s: 复算不稳定", c.name)
		}
	}
}

// TestDeriveAxisMap_NegativePush 反向推杆必须是负增量（防"只测了一侧"）。
func TestDeriveAxisMap_NegativePush(t *testing.T) {
	m, err := deriveAxisMap(defaultJoystick(), defaultNetwork())
	if err != nil {
		t.Fatalf("deriveAxisMap: %v", err)
	}
	if got := m.Deltas(netlink.Frame{LX: -1}, 1.0)[1]; got >= 0 {
		t.Errorf("左摇杆X 负偏: 增量为 %+.2f°，应为负", got)
	}
}

// TestDeriveAxisMap_RejectsUnknownServoID 接线表里没有该 id 时必须**报错**，
// 不能静默兜底 —— 否则会得到"推左摇杆却动了别的关节"。
func TestDeriveAxisMap_RejectsUnknownServoID(t *testing.T) {
	joy := defaultJoystick()
	joy.LXServo = 5 // 固件里不存在这个舵机 id
	if _, err := deriveAxisMap(joy, defaultNetwork()); err == nil {
		t.Fatalf("LXServo=5 不在 servo_ids 里，应当报错")
	}
}

// TestDeriveAxisMap_FollowsServoIDsOrder 换了接线表顺序，轴映射必须跟着走
// （只此一份真值：改 servo_ids 一处，摇杆与状态显示同时生效）。
func TestDeriveAxisMap_FollowsServoIDsOrder(t *testing.T) {
	net := defaultNetwork()
	net.ServoIDs = []int{6, 7, 8, 9} // 故意倒序
	m, err := deriveAxisMap(defaultJoystick(), net)
	if err != nil {
		t.Fatalf("deriveAxisMap: %v", err)
	}
	// 9 在新表里的位次是 4 ⇒ 左X 应落到 TCP#4
	if m.LXServo != 4 {
		t.Errorf("servo_ids 倒序后 LX 应映射到 4，实际 %d", m.LXServo)
	}
	// 6 的位次是 1 ⇒ 右X 应落到 TCP#1
	if m.RXServo != 1 {
		t.Errorf("servo_ids 倒序后 RX 应映射到 1，实际 %d", m.RXServo)
	}
}
