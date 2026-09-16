package tcpserver

// protocol_test.go —— 协议层：解析 / 校验 / 分发 / 错误分支。
//
// ★ 期望值**不硬编码**：HOME 位姿取自 `Model.HomePose`，舵机行程取自
//   `Actuator.Limits`，夹爪端点取自关节限位 —— 全部来自 robot.yaml。
//   测试里出现"具体角度"只会让真值多一份副本，那正是本工程反复踩的坑。

import (
	"encoding/json"
	"math"
	"strings"
	"sync"
	"testing"

	"armpilot/backend/internal/robot"
)

// modelPath 模型真值（与 robot 包测试用同一份）。
const modelPath = "../../../robot-package/mearm-v1/model/robot.yaml"

// fakeArm 记录被下发的关节角，不碰任何真实链路。
type fakeArm struct {
	mu      sync.Mutex
	last    map[string]float64
	applied []map[string]float64
	err     error
	kind    string
}

func newFakeArm(m *robot.Model) *fakeArm {
	home := map[string]float64{}
	for k, v := range m.HomePose {
		home[k] = v
	}
	return &fakeArm{last: home, kind: "sim"}
}

func (f *fakeArm) Apply(joints map[string]float64) error {
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.err != nil {
		return f.err
	}
	next := map[string]float64{}
	for k, v := range f.last {
		next[k] = v
	}
	for k, v := range joints {
		next[k] = v
	}
	f.last = next
	cp := map[string]float64{}
	for k, v := range joints {
		cp[k] = v
	}
	f.applied = append(f.applied, cp)
	return nil
}

func (f *fakeArm) Snapshot() (map[string]float64, map[string]float64) {
	f.mu.Lock()
	defer f.mu.Unlock()
	c := map[string]float64{}
	s := map[string]float64{}
	for k, v := range f.last {
		c[k], s[k] = v, v
	}
	return c, s
}

func (f *fakeArm) DeviceKind() string      { return f.kind }
func (f *fakeArm) DeviceConnected() bool   { return true }
func (f *fakeArm) lastApplied() map[string]float64 {
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.applied) == 0 {
		return nil
	}
	return f.applied[len(f.applied)-1]
}

func newFixture(t *testing.T) (*Handler, *fakeArm, *robot.Model, robot.Geom) {
	t.Helper()
	m, err := robot.Load(modelPath)
	if err != nil {
		t.Fatalf("加载模型失败: %v", err)
	}
	g, err := robot.LoadGeom(modelPath)
	if err != nil {
		t.Fatalf("求导几何失败: %v", err)
	}
	arm := newFakeArm(m)
	return NewHandler(arm, m, g), arm, m, g
}

// TestExecute_RejectsMalformedInput 畸形输入必须被**挡在协议层**，且不影响后续命令。
func TestExecute_RejectsMalformedInput(t *testing.T) {
	h, _, _, _ := newFixture(t)

	cases := []struct {
		name string
		line string
		want string
	}{
		{"空数据", "", "empty command"},
		{"空白", "   ", "empty command"},
		{"非法 JSON", `{"cmd":`, "invalid json"},
		{"空对象", `{}`, "missing cmd"},
		{"未知命令", `{"cmd":"teleport"}`, "unknown cmd"},
		{"未知命令（保留原样）", `{"cmd":"Teleport"}`, "unknown cmd"},
	}
	for _, c := range cases {
		got := h.Execute(c.line)
		if got.OK {
			t.Errorf("%s: 应当失败，却 ok=true", c.name)
			continue
		}
		if !strings.Contains(got.Error, c.want) {
			t.Errorf("%s: error = %q，应含 %q", c.name, got.Error, c.want)
		}
	}
	// 关键：畸形输入之后服务仍可用（错误不累积、不污染状态）。
	if got := h.Execute(`{"cmd":"gripper","action":"open"}`); !got.OK {
		t.Errorf("畸形输入之后合法命令应当照常工作，实际 error=%q", got.Error)
	}
}

// TestMove_AllSixDirections 六个方向都要真的把 TCP 推到对应位置。
//
// 判据是**几何**（FK 回读），不是"Apply 被调用过"。
func TestMove_AllSixDirections(t *testing.T) {
	// ⚠️ 步长不能随手挑：HOME 的 elbow=112.62 距下限 108.44 只有 4.18°，
	//    朝"伸展"的方向（+X / +Z）走太远会撞 JOINT_LIMIT —— 那是**正确**行为，
	//    不是缺陷。3mm 是六个方向都还走得动的量（手算过两支解）。
	step := 3.0
	for _, axis := range []string{"x", "y", "z"} {
		for _, dir := range []string{"+", "-"} {
			h, arm, _, g := newFixture(t)
			before := g.FK(mustSnapshot(arm))

			sign := 1.0
			if dir == "-" {
				sign = -1
			}
			res := h.Execute(mustJSON(t, Command{
				Cmd: "move", Axis: axis, Direction: dir, Step: &step,
			}))
			if !res.OK {
				t.Fatalf("move %s%s 失败: %s", axis, dir, res.Error)
			}
			want := before
			switch axis {
			case "x":
				want.X += sign * step
			case "y":
				want.Y += sign * step
			case "z":
				want.Z += sign * step
			}
			got := g.FK(mustSnapshot(arm))
			d := math.Sqrt(sq(got.X-want.X) + sq(got.Y-want.Y) + sq(got.Z-want.Z))
			if d > 1e-6 {
				t.Errorf("move %s%s: FK = (%.4f, %.4f, %.4f)，期望 (%.4f, %.4f, %.4f)，差 %.3e mm",
					axis, dir, got.X, got.Y, got.Z, want.X, want.Y, want.Z, d)
			}
		}
	}
}

// TestMove_RejectsBadParameters 参数错误必须报**具体**原因，不能笼统失败。
func TestMove_RejectsBadParameters(t *testing.T) {
	h, _, _, _ := newFixture(t)
	good, bad, zero, neg := 10.0, 10.0, 0.0, -5.0

	cases := []struct {
		name string
		cmd  Command
		want string
	}{
		{"invalid axis", Command{Cmd: "move", Axis: "w", Direction: "+", Step: &good}, "invalid axis"},
		{"missing axis", Command{Cmd: "move", Direction: "+", Step: &good}, "invalid axis"},
		{"invalid direction", Command{Cmd: "move", Axis: "x", Direction: "*", Step: &good}, "invalid direction"},
		{"missing direction", Command{Cmd: "move", Axis: "x", Step: &good}, "invalid direction"},
		{"missing step", Command{Cmd: "move", Axis: "x", Direction: "+"}, "missing step"},
		{"step = 0", Command{Cmd: "move", Axis: "x", Direction: "+", Step: &zero}, "invalid step"},
		{"step < 0", Command{Cmd: "move", Axis: "x", Direction: "+", Step: &neg}, "invalid step"},
	}
	for _, c := range cases {
		if got := h.dispatch(c.cmd); got.OK || !strings.Contains(got.Error, c.want) {
			t.Errorf("%s: 应报含 %q 的错误，实际 ok=%v error=%q", c.name, c.want, got.OK, got.Error)
		}
	}

	// NaN / Inf 走不到 JSON（json.Unmarshal 会先判非法数字），但 dispatch 必须挡住。
	nan := math.NaN()
	inf := math.Inf(1)
	for name, v := range map[string]float64{"NaN": nan, "Inf": inf} {
		if got := h.dispatch(Command{Cmd: "move", Axis: "x", Direction: "+", Step: &v}); got.OK || !strings.Contains(got.Error, "invalid step") {
			t.Errorf("step=%s: 应报 invalid step，实际 ok=%v error=%q", name, got.OK, got.Error)
		}
	}
	_ = bad
}

// TestGripper_OpenCloseUsesModelLimits 端点取自关节限位，不是抄在代码里的常数。
func TestGripper_OpenCloseUsesModelLimits(t *testing.T) {
	h, arm, m, _ := newFixture(t)

	jointID, limit := "", robot.Limit{}
	for i := range m.Joints {
		if m.Joints[i].Role == "gripper" {
			jointID, limit = m.Joints[i].ID, m.Joints[i].Limit
		}
	}
	if jointID == "" {
		t.Fatal("模型没有 gripper 关节 —— 判据缺失")
	}

	for action, want := range map[string]float64{"open": limit.Max, "close": limit.Min} {
		if got := h.Execute(`{"cmd":"gripper","action":"` + action + `"}`); !got.OK {
			t.Fatalf("gripper %s 失败: %s", action, got.Error)
		}
		v := arm.lastApplied()[jointID]
		if math.Abs(v-want) > 1e-9 {
			t.Errorf("gripper %s 下发 %.4f，期望限位端点 %.4f", action, v, want)
		}
	}
}

func TestGripper_RejectsBadAction(t *testing.T) {
	h, _, _, _ := newFixture(t)
	for _, c := range []struct {
		line, want string
	}{
		{`{"cmd":"gripper"}`, "missing action"},
		{`{"cmd":"gripper","action":""}`, "missing action"},
		{`{"cmd":"gripper","action":"half"}`, "invalid action"},
	} {
		if got := h.Execute(c.line); got.OK || !strings.Contains(got.Error, c.want) {
			t.Errorf("%s: 应报 %q，实际 ok=%v error=%q", c.line, c.want, got.OK, got.Error)
		}
	}
}

// TestServo_AllFourChannels 四个舵机都要能下单号，且换算后落在关节限位内。
func TestServo_AllFourChannels(t *testing.T) {
	h, arm, m, _ := newFixture(t)
	order := m.JointOrder()
	if len(order) < 4 {
		t.Fatalf("可动关节少于 4 个（%d）—— 与协议 servo 1..4 不匹配", len(order))
	}

	for n := 1; n <= 4; n++ {
		jointID := order[n-1]
		acts := m.ActuatorsForJoint(jointID)
		if len(acts) == 0 {
			t.Fatalf("关节 %s 没有执行器", jointID)
		}
		// 取舵机行程中点 —— 不挑一个"我记得能用"的角度。
		mid := (acts[0].Limits.Min + acts[0].Limits.Max) / 2
		res := h.Execute(mustJSON(t, Command{Cmd: "servo", Servo: &n, Angle: &mid}))
		if !res.OK {
			t.Fatalf("servo %d 失败: %s", n, res.Error)
		}
		theta := arm.lastApplied()[jointID]
		if v := m.Validate(map[string]float64{jointID: theta}); v != nil && !acts[0].IsJointSpace() {
			t.Errorf("servo %d 换算出的关节角 %.4f 越界: %v", n, theta, v)
		}
	}
}

func TestServo_RejectsBadInput(t *testing.T) {
	h, _, m, _ := newFixture(t)
	order := m.JointOrder()
	acts := m.ActuatorsForJoint(order[0])
	mid := (acts[0].Limits.Min + acts[0].Limits.Max) / 2
	tooHigh := acts[0].Limits.Max + 10
	tooLow := acts[0].Limits.Min - 10
	one := 1
	zero, five := 0, len(order)+1

	cases := []struct {
		name string
		cmd  Command
		want string
	}{
		{"missing servo", Command{Cmd: "servo", Angle: &mid}, "missing servo"},
		{"servo 0", Command{Cmd: "servo", Servo: &zero, Angle: &mid}, "invalid servo"},
		{"servo 越界", Command{Cmd: "servo", Servo: &five, Angle: &mid}, "invalid servo"},
		{"missing angle", Command{Cmd: "servo", Servo: &one}, "missing angle"},
		{"angle 过大", Command{Cmd: "servo", Servo: &one, Angle: &tooHigh}, "angle out of range"},
		{"angle 过小", Command{Cmd: "servo", Servo: &one, Angle: &tooLow}, "angle out of range"},
	}
	for _, c := range cases {
		if got := h.dispatch(c.cmd); got.OK || !strings.Contains(got.Error, c.want) {
			t.Errorf("%s: 应报含 %q，实际 ok=%v error=%q", c.name, c.want, got.OK, got.Error)
		}
	}
}

// TestExecute_NeverPanics 畸形输入不得让调用方 panic（§17 的第一条）。
func TestExecute_NeverPanics(t *testing.T) {
	h, _, _, _ := newFixture(t)
	weird := []string{
		"", " ", "\n", "{", "[]", "null", "true", "123",
		`{"cmd":"move"}`, `{"cmd":"move","axis":1}`, `{"cmd":"move","step":"10"}`,
		`{"cmd":"servo","servo":"1","angle":90}`, `{"cmd":"servo","servo":1.5,"angle":90}`,
		`{"cmd":"gripper","action":null}`, `{"cmd":null}`,
		`{"cmd":"move","axis":"x","direction":"+","step":1e999}`,
		strings.Repeat(`{"cmd":"move","axis":"x","direction":"+","step":1}`, 200),
	}
	for _, line := range weird {
		func() {
			defer func() {
				if r := recover(); r != nil {
					t.Errorf("输入 %q 触发 panic: %v", line, r)
				}
			}()
			_ = h.Execute(line)
		}()
	}
}

// TestHandler_SerializesConcurrentCommands 多客户端并发时命令不得互相踩踏（§18）。
//
// 用 `-race` 跑才有意义：`go test -race ./internal/tcpserver/`。
func TestHandler_SerializesConcurrentCommands(t *testing.T) {
	h, arm, _, g := newFixture(t)
	before := g.FK(mustSnapshot(arm))

	const n = 20
	step := 1.0
	var wg sync.WaitGroup
	for i := 0; i < n; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			_ = h.Execute(mustJSON(t, Command{Cmd: "move", Axis: "x", Direction: "+", Step: &step}))
		}()
	}
	wg.Wait()

	// 每一步都从"当前命令值"再 +1 ⇒ 累加必须精确等于 n×step，不能丢步。
	got := g.FK(mustSnapshot(arm))
	if d := math.Abs(got.X - (before.X + float64(n)*step)); d > 1e-6 {
		t.Errorf("并发 %d 次 +%.1f 后 X = %.4f，期望 %.4f（差 %.3e ⇒ 有步被并发吃掉）",
			n, step, got.X, before.X+float64(n)*step, d)
	}
}

func mustSnapshot(a *fakeArm) map[string]float64 {
	c, _ := a.Snapshot()
	return c
}

func mustJSON(t *testing.T, c Command) string {
	t.Helper()
	raw, err := json.Marshal(c)
	if err != nil {
		t.Fatalf("序列化失败: %v", err)
	}
	return string(raw)
}

func sq(v float64) float64 { return v * v }
