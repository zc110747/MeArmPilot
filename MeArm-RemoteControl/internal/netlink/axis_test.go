package netlink

// axis_test.go —— 摇杆 → 舵机角增量的换算（纯函数，无 IO）。
//
// ★ 期望值**从映射表派生**，不写"某个具体角度"当判据。

import (
	"math"
	"testing"
)

func testMap() AxisMap {
	m := DefaultAxisMap()
	m.DeadFrac = 0.2
	m.MinSpeedDegPerS = 10
	m.MaxSpeedDegPerS = 100
	return m
}

// TestAxisMap_DeltasFollowsTime 增量必须是**时间驱动**的：同样的输入，
// dt 翻倍 ⇒ 增量翻倍。这是"调用频率不影响速度"的可判定形式。
func TestAxisMap_DeltasFollowsTime(t *testing.T) {
	m := testMap()
	f := Frame{LX: 1, LY: 0, RX: 0, RY: 0}

	d1 := m.Deltas(f, 0.05)
	d2 := m.Deltas(f, 0.10)
	if len(d1) != 1 || len(d2) != 1 {
		t.Fatalf("满偏单轴应只影响一个舵机，实际 d1=%v d2=%v", d1, d2)
	}
	a, b := d1[m.LXServo], d2[m.LXServo]
	if math.Abs(b-2*a) > 1e-9 {
		t.Errorf("dt 翻倍后增量应翻倍：%.6f vs %.6f", a, b)
	}
	if math.Abs(a-m.MaxSpeedDegPerS*0.05) > 1e-9 {
		t.Errorf("满偏应按 MaxSpeed 走：得 %.6f，期望 %.6f", a, m.MaxSpeedDegPerS*0.05)
	}
}

// TestAxisMap_DeadZoneSilent 死区内的轴一律不产生增量（四轴全居中 ⇒ 零流量）。
func TestAxisMap_DeadZoneSilent(t *testing.T) {
	m := testMap()
	// 恰在死区边界上：|v| <= DeadFrac 视为居中。
	for _, v := range []float64{0, 0.05, m.DeadFrac, -m.DeadFrac} {
		f := Frame{LX: v, LY: v, RX: v, RY: v}
		if got := m.Deltas(f, 0.05); got != nil {
			t.Errorf("v=%.2f 在死区内，不应产生增量，实际 %v", v, got)
		}
		if !m.Centered(f) {
			t.Errorf("v=%.2f 应判为居中", v)
		}
	}
	// 刚出死区：应当有非零增量，且取 MinSpeed。
	f := Frame{LX: m.DeadFrac + 1e-6}
	d := m.Deltas(f, 1.0)
	if d == nil {
		t.Fatal("刚出死区应有增量")
	}
	if got := math.Abs(d[m.LXServo]); math.Abs(got-m.MinSpeedDegPerS) > 1e-3 {
		t.Errorf("刚出死区应取 MinSpeed=%.1f，实际 %.3f", m.MinSpeedDegPerS, got)
	}
	if m.Centered(f) {
		t.Error("刚出死区不应判为居中")
	}
}

// TestAxisMap_InvertFlipsSign invert 必须只翻方向、不改大小。
func TestAxisMap_InvertFlipsSign(t *testing.T) {
	base := testMap()
	inv := testMap()
	inv.InvLX = true

	f := Frame{LX: 0.8}
	a := base.Deltas(f, 0.05)
	b := inv.Deltas(f, 0.05)
	if math.Abs(a[base.LXServo]+b[inv.LXServo]) > 1e-9 {
		t.Errorf("invert 应只翻符号：%.6f vs %.6f", a[base.LXServo], b[inv.LXServo])
	}
}

// TestAxisMap_ServoWiring 四个轴必须落在**配置给的**舵机编号上。
func TestAxisMap_ServoWiring(t *testing.T) {
	m := testMap()
	d := m.Deltas(Frame{LX: 1, LY: 1, RX: 1, RY: 1}, 0.05)
	want := []int{m.LXServo, m.LYServo, m.RXServo, m.RYServo}
	for _, s := range want {
		if _, ok := d[s]; !ok {
			t.Errorf("舵机 %d（接线表）没有收到增量，实际 %v", s, d)
		}
	}
	if len(d) != len(want) {
		t.Errorf("增量轴数 = %d，期望 %d（实际 %v）", len(d), len(want), d)
	}
	if n := m.ServoCount(); n < 4 {
		t.Errorf("ServoCount = %d，默认接线表应有 4 个舵机", n)
	}
}

// TestDefaultAxisMap_MatchesMeArm3DWiring 默认接线表是**跨项目约定**，
// 写死在这里作为"被改动会被发现"的护栏（真值来源见 axis.go 注释）。
func TestDefaultAxisMap_MatchesMeArm3DWiring(t *testing.T) {
	m := DefaultAxisMap()
	// 左 X → servo 1（S9 底座）/ 左 Y → servo 3（S8 肘）/ 右 X → servo 4（S6 爪）/ 右 Y → servo 2（S7 肩）
	if m.LXServo != 1 || m.LYServo != 3 || m.RXServo != 4 || m.RYServo != 2 {
		t.Errorf("默认接线表 = LX:%d LY:%d RX:%d RY:%d，期望 1/3/4/2",
			m.LXServo, m.LYServo, m.RXServo, m.RYServo)
	}
}

// TestAxisMap_RejectsBadWiring 非法接线表要被 applyDefaults 挡掉。
func TestAxisMap_RejectsBadWiring(t *testing.T) {
	bad := DefaultAxisMap()
	bad.LXServo = 0
	if bad.valid() {
		t.Error("编号 0 应判为非法")
	}
	bad2 := DefaultAxisMap()
	bad2.RYServo = maxJoySlots + 1
	if bad2.valid() {
		t.Error("越界编号应判为非法")
	}
	if !DefaultAxisMap().valid() {
		t.Error("默认接线表必须合法")
	}
}

// TestClampAngle 值域保护（NaN / Inf 不得发到线上）。
func TestClampAngle(t *testing.T) {
	if got := clampAngle(math.NaN(), 0, 180); got != 0 {
		t.Errorf("NaN 应回落下限，得 %v", got)
	}
	if got := clampAngle(math.Inf(1), 0, 180); got != 0 {
		t.Errorf("+Inf 应回落下限，得 %v", got)
	}
	if got := clampAngle(200, 0, 180); got != 180 {
		t.Errorf("超上限应被夹住，得 %v", got)
	}
	if got := clampAngle(-5, 0, 180); got != 0 {
		t.Errorf("低于下限应被夹住，得 %v", got)
	}
	if got := clampAngle(90, 0, 180); got != 90 {
		t.Errorf("区间内不得改动，得 %v", got)
	}
}

// TestTrimFloat JSON 数字要短且精确。
func TestTrimFloat(t *testing.T) {
	cases := map[float64]string{
		90:     "90",
		90.5:   "90.5",
		90.123: "90.123",
		-0.5:   "-0.5",
	}
	for in, want := range cases {
		if got := trimFloat(in); got != want {
			t.Errorf("trimFloat(%v) = %q，期望 %q", in, got, want)
		}
	}
}
