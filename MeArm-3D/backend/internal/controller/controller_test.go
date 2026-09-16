package controller

import (
	"fmt"
	"math"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"armpilot/backend/internal/device"
	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// ---------------------------------------------------------------------------
// 可控的假设备：WriteLine 只记录，回执由测试手工注入 —— 这样门控/超时/合并
// 这些**时序**行为才能被精确断言，而不是被一个"过于配合"的模拟器掩盖。
// ---------------------------------------------------------------------------

type fakeDevice struct {
	mu        sync.Mutex
	written   []string
	connected bool
	reason    string
	lines     chan device.Line
	closed    bool
}

func newFakeDevice() *fakeDevice {
	return &fakeDevice{connected: true, lines: make(chan device.Line, 128)}
}

func (f *fakeDevice) Kind() string { return "fake" }

func (f *fakeDevice) Connected() bool {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.connected
}

func (f *fakeDevice) UnavailableReason() string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.reason
}

func (f *fakeDevice) WriteLine(line string) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if !f.connected {
		return fmt.Errorf("设备不可用: %s", f.reason)
	}
	f.written = append(f.written, line)
	return nil
}

func (f *fakeDevice) Lines() <-chan device.Line { return f.lines }

func (f *fakeDevice) OnStatus(fn device.StatusHandler) func() { return func() {} }

func (f *fakeDevice) Close() error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.closed {
		return nil
	}
	f.closed = true
	close(f.lines)
	return nil
}

func (f *fakeDevice) setConnected(v bool, reason string) {
	f.mu.Lock()
	f.connected = v
	f.reason = reason
	f.mu.Unlock()
}

func (f *fakeDevice) lines_() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]string(nil), f.written...)
}

func (f *fakeDevice) count() int {
	f.mu.Lock()
	defer f.mu.Unlock()
	return len(f.written)
}

func (f *fakeDevice) emit(text string) { f.lines <- device.Line{Text: text, At: time.Now()} }

// ---------------------------------------------------------------------------

func loadModel(t *testing.T) *robot.Model {
	t.Helper()
	p, err := filepath.Abs(filepath.Join("..", "..", "..", "robot-package", "mearm-v1", "model", "robot.yaml"))
	if err != nil {
		t.Fatalf("路径解析失败: %v", err)
	}
	m, err := robot.Load(p)
	if err != nil {
		t.Fatalf("Load 失败: %v", err)
	}
	return m
}

type harness struct {
	ctl    *Controller
	dev    *fakeDevice
	mu     sync.Mutex
	st     map[string]float64
	origin string
	err    []string
}

func newHarness(t *testing.T, cfg Config) *harness {
	t.Helper()
	m := loadModel(t)
	dev := newFakeDevice()
	ctl := New(m, dev, cfg)
	h := &harness{ctl: ctl, dev: dev}
	ctl.OnJointState(func(j map[string]float64, _ time.Time, origin string) {
		h.mu.Lock()
		h.st = j
		h.origin = origin
		h.mu.Unlock()
	})
	ctl.OnError(func(code, msg string) {
		h.mu.Lock()
		h.err = append(h.err, code+": "+msg)
		h.mu.Unlock()
	})
	ctl.Start()
	t.Cleanup(func() { ctl.Close() })
	return h
}

func (h *harness) state() map[string]float64 {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.st
}

// stateOrigin 最近一帧状态的来源（`protocol.OriginCommand` / `OriginDevice`）。
func (h *harness) stateOrigin() string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return h.origin
}

func (h *harness) errors() []string {
	h.mu.Lock()
	defer h.mu.Unlock()
	return append([]string(nil), h.err...)
}

// wait 轮询直到条件满足或超时。
func wait(t *testing.T, timeout time.Duration, cond func() bool) bool {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if cond() {
			return true
		}
		time.Sleep(2 * time.Millisecond)
	}
	return cond()
}

// ---------------------------------------------------------------------------

// 限位越界必须在**同步返回**里拒绝，且一个字节都不能写到设备。
func TestApplyRejectsOutOfRangeSynchronously(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	err := h.ctl.Apply(map[string]float64{"elbow": 95})
	if err == nil {
		t.Fatal("肘角 95° 应被拒绝")
	}
	re, ok := err.(*RejectError)
	if !ok {
		t.Fatalf("错误类型 = %T, 期望 *RejectError", err)
	}
	if re.Code != protocol.CodeJointLimit {
		t.Errorf("错误码 = %q, 期望 %q", re.Code, protocol.CodeJointLimit)
	}
	if want := "ERR JOINT elbow 95.00 (limit 108.44..141.86)"; re.Message != want {
		t.Errorf("错误文案 = %q, 期望 %q", re.Message, want)
	}
	if n := h.dev.count(); n != 0 {
		t.Errorf("被拒绝的命令不应写设备，却写了 %d 条: %v", n, h.dev.lines_())
	}
}

func TestApplyEncodesJR(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	if err := h.ctl.Apply(map[string]float64{"shoulder": 20.8}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	if !wait(t, time.Second, func() bool { return h.dev.count() == 1 }) {
		t.Fatalf("应写设备 1 条，实际 %v", h.dev.lines_())
	}
	if got := h.dev.lines_()[0]; got != "JR 0.0 20.8 112.6 50.0" {
		t.Errorf("下发文本 = %q, 期望 %q", got, "JR 0.0 20.8 112.6 50.0")
	}
}

// ⚠️ 核心语义：OK JR 携带的是**目标角**，绝不能当作实际位置回推状态。
//
//	否则 Actual 永远等于 Command、误差恒为 0，"滞后→收敛"整条语义消失。
func TestAckDoesNotPublishState(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	if err := h.ctl.Apply(map[string]float64{"shoulder": 20.8}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	wait(t, time.Second, func() bool { return h.dev.count() == 1 })

	m := loadModel(t)
	// 设备回 OK，且回执里的舵机角正是命令的目标角
	servo := map[int]float64{}
	for _, id := range m.JointOrder() {
		for _, a := range m.ActuatorsForJoint(id) {
			servo[a.Channel] = robot.JointToServo(a, 20.8)
		}
	}
	h.dev.emit(protocol.EncodeOKJR(servo))

	// 给足时间让 readLoop 处理
	time.Sleep(80 * time.Millisecond)
	if st := h.state(); st != nil {
		t.Errorf("OK JR 不应产生 joint_state（那是目标角，不是实际位置），却发布了: %v", st)
	}
}

func TestStatePublishesJoints(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	h.dev.emit("STATE 0.00 20.80 120.00 50.00")
	if !wait(t, time.Second, func() bool { return h.state() != nil }) {
		t.Fatal("STATE 应收敛为 joint_state 事件")
	}
	st := h.state()
	if math.Abs(st["shoulder"]-20.8) > 1e-9 || math.Abs(st["elbow"]-120) > 1e-9 {
		t.Errorf("状态解析错误: %v", st)
	}
	// STATE 是**命令引起**的状态：前端据此只更新 Actual，不跟随命令侧
	if got := h.stateOrigin(); got != protocol.OriginCommand {
		t.Errorf("STATE 的来源 = %q, 期望 %q", got, protocol.OriginCommand)
	}
}

// ---------------------------------------------------------------------------
// 设备侧自主变化（摇杆 / 红外 / 手拧）
// ---------------------------------------------------------------------------

// findActuator 取某通道的执行器（真值表来自 robot.yaml，不在这里写死标定）。
func findActuator(t *testing.T, m *robot.Model, ch int) *robot.Actuator {
	t.Helper()
	for i := range m.Actuators {
		if m.Actuators[i].Channel == ch {
			return &m.Actuators[i]
		}
	}
	t.Fatalf("robot.yaml 里没有通道 S%d 的执行器", ch)
	return nil
}

// 设备侧自主变化必须以 `# SERVO` 上报，并标成 **OriginDevice**。
//
// 这是"摇杆把机械臂拧了 30°、上位机界面跟不跟"这条需求的入口。
// 标错来源的两种后果都很隐蔽：
//   - 标成 OriginCommand ⇒ 界面不跟随，滑杆停在旧位置（用户看到的界面在说假话）
//   - 由前端自己猜（比较 Actual 与 Command）⇒ 拖动时设备还在斜坡上，
//     前端会把命令侧一路拉回半路位置（回环）
func TestDeviceServoReportPublishesDeviceOrigin(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	m := loadModel(t)

	// 四个舵机停在 HOME，唯独把 S7（肩）拧到 120° —— 模拟"有人动了摇杆"
	servo := map[int]float64{}
	for _, id := range m.JointOrder() {
		for _, a := range m.ActuatorsForJoint(id) {
			servo[a.Channel] = robot.JointToServo(a, m.HomePose[id])
		}
	}
	servo[7] = 120
	h.dev.emit(protocol.EncodeServoReport(servo))

	if !wait(t, time.Second, func() bool { return h.state() != nil }) {
		t.Fatal("设备上报应收敛为 joint_state 事件")
	}
	if got := h.stateOrigin(); got != protocol.OriginDevice {
		t.Errorf("来源 = %q, 期望 %q（否则界面不会跟随）", got, protocol.OriginDevice)
	}
	// ★ 关节角必须由**后端唯一一处**用标定表反算 —— 设备不认识关节，
	//   它报的永远是舵机角。这里用真值表算出的期望值来证明换算真的发生了。
	shoulder := findActuator(t, m, 7)
	want := robot.ServoToJoint(shoulder, 120)
	if got := h.state()[shoulder.JointID]; math.Abs(got-want) > 1e-9 {
		t.Errorf("反算关节角 = %.6f, 期望 %.6f（标定未被应用？）", got, want)
	}
	// 未参与的关节必须**沿用上一次状态**，不能被补齐成限位下界
	if got := h.state()["base"]; got != m.HomePose["base"] {
		t.Errorf("未上报的关节 = %v, 期望沿用旧值 %v", got, m.HomePose["base"])
	}
}

// `# SERVO` 是**异步事件**，不是某条在途命令的应答 ⇒ 不得放行 ACK 门控。
//
// 若误当应答处理：拖动时每来一行摇杆上报就放行一次门控，命令下发频率
// 会被上报频率带飞（真机上就是把自己打爆）。
func TestDeviceServoReportDoesNotFinishInflight(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	if err := h.ctl.Apply(map[string]float64{"shoulder": 20.8}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	if !wait(t, time.Second, func() bool { return h.dev.count() == 1 }) {
		t.Fatalf("应写设备 1 条，实际 %v", h.dev.lines_())
	}

	// 在途期间来一行设备上报
	h.dev.emit(protocol.EncodeServoReport(map[int]float64{7: 120}))
	time.Sleep(60 * time.Millisecond)

	// 再下一条命令：门控**仍未**放行 ⇒ 只应进待发槽，不写设备
	if err := h.ctl.Apply(map[string]float64{"shoulder": 30}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	time.Sleep(60 * time.Millisecond)
	if n := h.dev.count(); n != 1 {
		t.Errorf("异步上报不该放行门控，设备却被写了 %d 条: %v", n, h.dev.lines_())
	}
}

// ACK 门控 + latest-wins：在途期间新命令只覆盖待发槽，绝不排队。
func TestLatestWinsMergesPending(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	m := loadModel(t)

	cmds := []float64{10, 20, 30}
	for _, v := range cmds {
		if err := h.ctl.Apply(map[string]float64{"shoulder": v}); err != nil {
			t.Fatalf("Apply(%v) 失败: %v", v, err)
		}
	}
	// 尚未回复执：只应有第 1 条在途
	if n := h.dev.count(); n != 1 {
		t.Fatalf("在途期间应只写 1 条，实际 %d: %v", n, h.dev.lines_())
	}

	// 回第 1 条的 OK → 门控放行 → 补发**最新**的（30），而不是排队发 20
	servo := map[int]float64{}
	for _, id := range m.JointOrder() {
		for _, a := range m.ActuatorsForJoint(id) {
			servo[a.Channel] = 80
		}
	}
	h.dev.emit(protocol.EncodeOKJR(servo))

	if !wait(t, time.Second, func() bool { return h.dev.count() == 2 }) {
		t.Fatalf("应补发 1 条，实际 %v", h.dev.lines_())
	}
	lines := h.dev.lines_()
	if lines[1] != "JR 0.0 30.0 112.6 50.0" {
		t.Errorf("补发的应是**最新**命令（shoulder=30），实际 %q —— latest-wins 失效", lines[1])
	}
	if n := h.dev.count(); n != 2 {
		t.Errorf("中间命令 20 不该被下发，总条数 = %d: %v", n, lines)
	}
}

// ERR 回执同样放行门控（否则一次拒绝会把链路彻底卡死）。
func TestErrReplyReleasesGate(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	if err := h.ctl.Apply(map[string]float64{"shoulder": 10}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	if err := h.ctl.Apply(map[string]float64{"shoulder": 20}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	h.dev.emit("ERR SERVO S7 200.00 (limit 80.00..160.00)")

	if !wait(t, time.Second, func() bool { return h.dev.count() == 2 }) {
		t.Fatalf("ERR 后应放行门控并补发，实际 %v", h.dev.lines_())
	}
	if len(h.errors()) == 0 {
		t.Error("ERR 回执应产生 error 事件")
	}
}

func TestAckTimeoutReportsErrorAndDrains(t *testing.T) {
	// 缩短超时以便快速验证
	h := newHarness(t, Config{AckTimeoutMs: 150, MinSendIntervalMs: 0, EchoJointState: true})
	if err := h.ctl.Apply(map[string]float64{"shoulder": 10}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	// 第二条进待发槽
	if err := h.ctl.Apply(map[string]float64{"shoulder": 20}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	// 设备不回执 → 超时后应报 ACK_TIMEOUT 并补发
	if !wait(t, 2*time.Second, func() bool { return len(h.errors()) > 0 }) {
		t.Fatal("回执超时应产生 error 事件")
	}
	if !wait(t, 2*time.Second, func() bool { return h.dev.count() == 2 }) {
		t.Errorf("超时后应补发待发命令，实际 %v", h.dev.lines_())
	}
}

func TestApplyWhenDeviceDown(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	h.dev.setConnected(false, "串口未插")
	err := h.ctl.Apply(map[string]float64{"shoulder": 10})
	if err == nil {
		t.Fatal("设备不可用时应拒绝")
	}
	re, ok := err.(*RejectError)
	if !ok {
		t.Fatalf("错误类型 = %T", err)
	}
	if re.Code != protocol.CodeDeviceDown {
		t.Errorf("错误码 = %q, 期望 %q", re.Code, protocol.CodeDeviceDown)
	}
	if n := h.dev.count(); n != 0 {
		t.Errorf("设备不可用时不该写设备，实际 %d 条", n)
	}
}

// 部分关节帧（如只改夹爪）必须沿用其余关节的当前命令值，不能补 0。
func TestPartialFrameKeepsOtherJoints(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	if err := h.ctl.Apply(map[string]float64{"shoulder": 25}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	wait(t, time.Second, func() bool { return h.dev.count() == 1 })
	h.dev.emit("OK JR S9=90.00 S8=90.00 S7=90.00 S6=90.00")
	wait(t, time.Second, func() bool { return h.dev.count() >= 1 })
	time.Sleep(30 * time.Millisecond)

	if err := h.ctl.Apply(map[string]float64{"gripper": 80}); err != nil {
		t.Fatalf("Apply 失败: %v", err)
	}
	if !wait(t, time.Second, func() bool { return h.dev.count() == 2 }) {
		t.Fatalf("应下发第 2 条，实际 %v", h.dev.lines_())
	}
	lines := h.dev.lines_()
	// 肩角 25 必须保留（若实现补 0，这里会变成 0.0）
	if lines[1] != "JR 0.0 25.0 112.6 80.0" {
		t.Errorf("部分帧下发 = %q, 期望 %q（其余关节应沿用当前命令值）", lines[1], "JR 0.0 25.0 112.6 80.0")
	}
}

func TestSnapshotInitiallyHome(t *testing.T) {
	h := newHarness(t, DefaultConfig())
	m := loadModel(t)
	cmd, st := h.ctl.Snapshot()
	for _, id := range m.JointOrder() {
		if math.Abs(cmd[id]-m.HomePose[id]) > 1e-9 {
			t.Errorf("初始命令 %s = %v, 期望 HOME %v", id, cmd[id], m.HomePose[id])
		}
		if math.Abs(st[id]-m.HomePose[id]) > 1e-9 {
			t.Errorf("初始状态 %s = %v, 期望 HOME %v", id, st[id], m.HomePose[id])
		}
	}
}
