package protocol

import (
	"strconv"
	"strings"
	"testing"
)

func TestValidate(t *testing.T) {
	cases := []struct {
		in      string
		wantOK  bool
		wantOut string
	}{
		{"SET 9 120", true, "SET 9 120"},
		{"SET 9 120 8 90 7 100", true, "SET 9 120 8 90 7 100"},
		{"SET 9 120 8 90 7 100 6 50", false, ""}, // >3
		{"S7=90", true, "S7=90"},
		{"S5=90", false, ""}, // bad id
		{"STOP 8", true, "STOP 8"},
		{"AUTO 9", true, "AUTO 9"},
		{"JOY 900 200 512 800", true, "JOY 900 200 512 800"},
		{"JOY 8 50", true, "JOY 8 50"},
		{"JOY 8 2000", false, ""}, // raw>1023
		{"IR 0xF708FF00", true, "IR 0xF708FF00"},
		{"IR DEADBEEF", true, "IR DEADBEEF"},
		{"SEQ 1", true, "SEQ 1"},
		{"SEQ STOP", true, "SEQ STOP"},
		{"SEQ 2", false, ""},
		{"JOYHW ON", true, "JOYHW ON"},
		{"JOYHW maybe", false, ""},
		{"RESET", true, "RESET"},
		{"STATUS", true, "STATUS"},
		{"?", true, "?"},
		{"ADC", true, "ADC"},
		{"HELP", true, "HELP"},
		{"garbage", false, ""},
	}
	for _, c := range cases {
		ok, out, err := Validate(c.in)
		if ok != c.wantOK {
			t.Errorf("Validate(%q) ok=%v want %v (err=%v)", c.in, ok, c.wantOK, err)
			continue
		}
		if ok && out != c.wantOut {
			t.Errorf("Validate(%q) out=%q want %q", c.in, out, c.wantOut)
		}
	}
}

func TestJoystickToJOY(t *testing.T) {
	m := DefaultAxisMap()
	// 中位 -> 全 512（设备死区，不动作）
	if got := JoystickToJOY(0, 0, m); got != "JOY 512 512 512 512" {
		t.Errorf("center: %q", got)
	}
	// 右满 -> raw9 接近 1023
	if got := JoystickToJOY(1, 0, m); got != "JOY 1023 512 512 512" {
		t.Errorf("right: %q", got)
	}
	// 左满 -> raw9 接近 0
	if got := JoystickToJOY(-1, 0, m); got != "JOY 0 512 512 512" {
		t.Errorf("left: %q", got)
	}
	// 前满 -> raw8 = 1023
	if got := JoystickToJOY(0, 1, m); got != "JOY 512 1023 512 512" {
		t.Errorf("forward: %q", got)
	}
	// 反相 左X
	m2 := AxisMap{LXServo: 9, LYServo: 8, InvLX: true, InvLY: false}
	if got := JoystickToJOY(1, 0, m2); got != "JOY 0 512 512 512" {
		t.Errorf("invLX right: %q", got)
	}
}

func TestDeadFracFromDeg(t *testing.T) {
	if got := DeadFracFromDeg(0); got != 0 {
		t.Errorf("0 deg should disable: %v", got)
	}
	if got := DeadFracFromDeg(-3); got != 0 {
		t.Errorf("negative should disable: %v", got)
	}
	if got := DeadFracFromDeg(10); got < 0.31 || got > 0.33 {
		t.Errorf("10 deg of 31.5 deg full tilt: %v", got)
	}
	if got := DeadFracFromDeg(100); got != 0.9 {
		t.Errorf("clamped to 0.9: %v", got)
	}
}

// joyRawOf 从 "JOY a b c d" 中取第 idx 个 raw。
func joyRawOf(t *testing.T, cmd string, idx int) int {
	t.Helper()
	fields := strings.Fields(cmd)
	if len(fields) != 5 {
		t.Fatalf("bad JOY cmd: %q", cmd)
	}
	v, err := strconv.Atoi(fields[1+idx])
	if err != nil {
		t.Fatalf("bad raw in %q: %v", cmd, err)
	}
	return v
}

func TestDeadband(t *testing.T) {
	m := DefaultAxisMap() // DeadFrac = 5°/31.5° ≈ 0.159

	// 死区内（≤5° 视觉倾角，≈16% 行程）-> raw 512（固件不动作）
	if got := JoystickToJOY(0.15, -0.1, m); got != "JOY 512 512 512 512" {
		t.Errorf("inside deadband should be centered: %q", got)
	}
	// 四轴全居中 -> 无命令，调用方应整帧跳过下发
	if JoystickHasCommand(0.15, 0.1, -0.1, 0, m) {
		t.Errorf("all axes inside deadband: expect no command")
	}
	// 任一轴超出死区 -> 有命令
	if !JoystickHasCommand(0.3, 0.2, -0.1, 0.4, m) {
		t.Errorf("axis beyond deadband: expect command")
	}

	// 刚出死区即进入固件命令区间：raw>800（正向）
	if r := joyRawOf(t, JoystickToJOY(0.35, 0, m), 0); r <= 800 {
		t.Errorf("just beyond deadband should command (raw>800): %d", r)
	}
	// 负向对称：raw<200
	if r := joyRawOf(t, JoystickToJOY(-0.35, 0, m), 0); r >= 200 {
		t.Errorf("just beyond deadband (neg) should command (raw<200): %d", r)
	}
	// 满偏 -> raw 1023 / 0（固件最大步长）
	if got := JoystickToJOY(1, -1, m); got != "JOY 1023 0 512 512" {
		t.Errorf("full travel: %q", got)
	}
	// 行程越大步长越大（单调）
	r35 := joyRawOf(t, JoystickToJOY(0.35, 0, m), 0)
	r80 := joyRawOf(t, JoystickToJOY(0.8, 0, m), 0)
	if !(r35 > 800 && r80 > r35 && r80 < 1023) {
		t.Errorf("monotonic response: at0.35=%d at0.8=%d", r35, r80)
	}
}

func TestDeadbandInverted(t *testing.T) {
	// 与 config.yaml 默认一致的镜像：9/6/7 轴 invert
	mi := AxisMap{LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7,
		InvLX: true, InvRX: true, InvRY: true, DeadFrac: DefaultDeadFrac()}
	// 推 + 满偏：invert 后 raw=0，对应固件 + 步长侧
	if got := JoystickToJOYDual(1, 1, 1, 1, mi); got != "JOY 0 1023 0 0" {
		t.Errorf("inverted full dual: %q", got)
	}
	// 死区内：invert 不影响居中值 512
	if got := JoystickToJOYDual(0.1, -0.1, 0.15, 0.05, mi); got != "JOY 512 512 512 512" {
		t.Errorf("inverted inside deadband: %q", got)
	}
}

// TestFirmwareInvertsAxis 钉住固件的方向差异表 —— 它是 NetInvertFor 的唯一输入。
//
// 真值：MeArm-Device/core/joystick.c 的 `bool positive = (id == 8) ? past_hi : past_lo;`
// 谁改了固件这一行，就必须同时改 protocol.go 的 FirmwareInvertsAxis 与本测试。
func TestFirmwareInvertsAxis(t *testing.T) {
	for _, id := range []int{6, 7, 9} {
		if FirmwareInvertsAxis(id) {
			t.Errorf("id %d 在固件里是正常轴（raw<200 -> +step），不该标为反相", id)
		}
	}
	if !FirmwareInvertsAxis(8) {
		t.Errorf("id 8 在固件里是反相轴（raw>800 -> +step）")
	}
}

// TestNetInvertFor 钉住"串口 invert_* → 网络 invert"的换算。
//
// 判据不是"公式长什么样"，而是**两种模式下同一个推杆动作是否让同一个舵机
// 朝同一个方向转**：
//
//	串口（axisRaw 镜像 + 固件方向）：
//	    推杆正向 s=+1 时，步长符号 =
//	        axes 9/6/7 : inv ? +1 : -1
//	        axis 8     : inv ? -1 : +1
//	网络（netlink.axisSpeed 只在 inv 时取负）：
//	    步长符号 = inv ? -1 : +1
//
// 本测试把上面这张表逐格算出来，要求 NetInvertFor 使其与串口侧逐格相等。
func TestNetInvertFor(t *testing.T) {
	// serialStepSign 复刻串口链路对 s=+1 的步长符号（-1 = 角度减小）。
	serialStepSign := func(armServoID int, inv bool) int {
		// axisRaw: out>0 → raw = 511+|out|*512；inv → raw = 1023-raw
		// ⇒ P = sign(raw-512) = inv ? -1 : +1
		p := 1
		if inv {
			p = -1
		}
		// 固件：9/6/7 是 raw<200 才正步进 ⇒ step = -P；8 轴反过来 ⇒ step = +P
		if FirmwareInvertsAxis(armServoID) {
			return p
		}
		return -p
	}
	// netStepSign 复刻网络链路对 s=+1 的步长符号。
	netStepSign := func(netInv bool) int {
		if netInv {
			return -1
		}
		return 1
	}

	for _, id := range []int{6, 7, 8, 9} {
		for _, inv := range []bool{false, true} {
			netInv := NetInvertFor(id, inv)
			want := serialStepSign(id, inv)
			got := netStepSign(netInv)
			if got != want {
				t.Errorf("id=%d inv=%v: NetInvertFor=%v 给出步长符号 %+d，串口侧是 %+d —— 两模式方向相反",
					id, inv, netInv, got, want)
			}
		}
	}
}

// TestNetInvertFor_DefaultConfigIsAllFalse 是上面那张表在 config.yaml
// 出厂默认值上的直接推论：四轴都应换算成 inv=false
// （= 网络模式下"推杆正方向 → 该舵机角度增大"）。
//
// 哪一个不等于 false，就说明有人改了 invert 的默认值却没同步两种模式的手感。
func TestNetInvertFor_DefaultConfigIsAllFalse(t *testing.T) {
	// config.yaml 的 joystick 段默认值：lx_servo=9 ly_servo=8 rx_servo=6 ry_servo=7
	//                                   invert_lx=true invert_ly=false
	//                                   invert_rx=true invert_ry=true
	cases := []struct {
		name string
		id   int
		inv  bool
	}{
		{"lx_servo=9/invert_lx=true", 9, true},
		{"ly_servo=8/invert_ly=false", 8, false},
		{"rx_servo=6/invert_rx=true", 6, true},
		{"ry_servo=7/invert_ry=true", 7, true},
	}
	for _, c := range cases {
		if got := NetInvertFor(c.id, c.inv); got {
			t.Errorf("%s: NetInvertFor=%v，应为 false（否则网络模式推杆方向与串口相反）",
				c.name, got)
		}
	}
}
