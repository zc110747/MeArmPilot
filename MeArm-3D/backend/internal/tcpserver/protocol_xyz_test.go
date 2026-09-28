package tcpserver

// protocol_xyz_test.go —— v2 的 XYZ 矢量命令（movexyz / moveto / home / caps）。
//
// ★ 与 protocol_test.go 同一条纪律：**期望值一律从 robot.yaml 派生**。
//   可达目标点不写"我记得能到的坐标"，而是取**某个合法关节位姿的 FK** ——
//   那样"可达"是构造出来的事实，不是凭手感挑的数字。
//
// ★ 本文件另一半职责是**守住 v1 不被改坏**：`TestMove_SingleAxisEqualsMoveXYZ`
//   逐位比对单轴 move 与 movexyz 的结果。这是本次改动唯一的回归红线 ——
//   现有功能必须一个字节都不变。

import (
	"encoding/json"
	"math"
	"strings"
	"sync"
	"testing"

	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// reachablePose 一个**限位于内部**的合法关节位姿，用作可达目标点的构造源。
//
// 取各关节限位的内部点（留 10% 余量，不取端点 —— 端点可能因浮点量化落到限位外），
// 于是"它的 FK 一定可达"是**构造出来的**，不需要手算可达壳。
func reachablePose(t *testing.T, m *robot.Model) map[string]float64 {
	t.Helper()
	pose := map[string]float64{}
	for _, id := range []string{robot.JointBase, robot.JointShoulder, robot.JointElbow} {
		j := m.Joint(id)
		if j == nil {
			t.Fatalf("模型缺少关节 %s", id)
		}
		span := j.Limit.Max - j.Limit.Min
		pose[id] = j.Limit.Min + span*0.3 // 30% 处：离两端都远
	}
	if v := m.Validate(pose); v != nil {
		t.Fatalf("构造出的位姿自己就越界: %v", v)
	}
	return pose
}

// TargetOf 用 FK 把合法位姿换成可达目标点（moveto / movexyz 的输入）。
func targetOf(g robot.Geom, pose map[string]float64) robot.Vec3 { return g.FK(pose) }

// TestMoveXYZ_AppliesAllThreeAxes 三轴位移必须**同时**生效，且几何上精确。
//
// 判据是 FK 回读，不是"Apply 被调用过" —— 后者在算错目标点时照样是绿的。
func TestMoveXYZ_AppliesAllThreeAxes(t *testing.T) {
	h, arm, m, g := newFixture(t)

	// 目标点 = 合法位姿的 FK ⇒ 可达性由构造保证，不靠手挑坐标。
	target := targetOf(g, reachablePose(t, m))
	cur := mustSnapshot(arm)
	before := g.FK(cur)
	delta := [3]float64{target.X - before.X, target.Y - before.Y, target.Z - before.Z}

	// 三轴分量都非零 —— 否则这条测试退化成单轴用例，测不到"同时"。
	nz := 0
	for _, d := range delta {
		if math.Abs(d) > 1e-6 {
			nz++
		}
	}
	if nz < 2 {
		t.Fatalf("构造出的位移只有 %d 个非零分量（%v）—— 判据失效，换个位姿", nz, delta)
	}

	res := h.Execute(mustJSON(t, Command{Cmd: "movexyz", Delta: delta[:]}))
	if !res.OK {
		t.Fatalf("movexyz %v 失败: %s", delta, res.Error)
	}
	got := g.FK(mustSnapshot(arm))
	if d := dist3(got, target); d > 1e-6 {
		t.Errorf("movexyz 后 FK = (%.4f, %.4f, %.4f)，期望 (%.4f, %.4f, %.4f)，差 %.3e mm",
			got.X, got.Y, got.Z, target.X, target.Y, target.Z, d)
	}
}

// TestMove_SingleAxisEqualsMoveXYZ ★ 红线：单轴 `move` 与 `movexyz` 必须**逐位一致**。
//
// v2 把 move / movexyz / moveto 收敛到同一条 `applyTarget` 尾巴上。收敛是对的
// （单轴与矢量本就该同一个语义），但"收敛"也可能顺手改掉 v1 的行为 ——
// 这条测试就是那个可疑动作的警报器：一旦 move 的基准、分支选择或下发顺序变了，
// 这里立刻红。
func TestMove_SingleAxisEqualsMoveXYZ(t *testing.T) {
	step := 3.0 // 与 TestMove_AllSixDirections 同量级：HOME 附近六个方向都走得动
	for _, axis := range []string{"x", "y", "z"} {
		for _, dir := range []string{"+", "-"} {
			sign := 1.0
			if dir == "-" {
				sign = -1.0
			}
			d := [3]float64{0, 0, 0}
			d[map[string]int{"x": 0, "y": 1, "z": 2}[axis]] = sign * step

			// 两个**独立**的 fixture，都从同一个 HOME 起步。
			hMove, armMove, _, _ := newFixture(t)
			hXYZ, armXYZ, _, _ := newFixture(t)

			rMove := hMove.Execute(mustJSON(t, Command{Cmd: "move", Axis: axis, Direction: dir, Step: &step}))
			rXYZ := hXYZ.Execute(mustJSON(t, Command{Cmd: "movexyz", Delta: d[:]}))
			if !rMove.OK || !rXYZ.OK {
				t.Fatalf("%s%s: move ok=%v (%s) / movexyz ok=%v (%s)",
					axis, dir, rMove.OK, rMove.Error, rXYZ.OK, rXYZ.Error)
			}

			a, b := mustSnapshot(armMove), mustSnapshot(armXYZ)
			for _, id := range []string{robot.JointBase, robot.JointShoulder, robot.JointElbow} {
				if a[id] != b[id] {
					t.Errorf("%s%s: 关节 %s 不一致 —— move=%.17g movexyz=%.17g（差 %.3e）",
						axis, dir, id, a[id], b[id], math.Abs(a[id]-b[id]))
				}
			}
			// 成功应答的 state 也必须一致（外部控制器靠它做闭环）。
			if len(rMove.State.TCP) == 3 && len(rXYZ.State.TCP) == 3 {
				for i := range rMove.State.TCP {
					if rMove.State.TCP[i] != rXYZ.State.TCP[i] {
						t.Errorf("%s%s: state.tcp[%d] 不一致 —— move=%.17g movexyz=%.17g",
							axis, dir, i, rMove.State.TCP[i], rXYZ.State.TCP[i])
					}
				}
			}
		}
	}
}

// TestMoveTo_AbsoluteTarget moveto 是**绝对**语义：连续两次下发同一点必须停在同一处。
//
// 这条专治"把绝对当相对" —— 那种实现第一次到位、第二次就开始漂，
// 而"命令被受理了"的用例完全看不见。
func TestMoveTo_AbsoluteTarget(t *testing.T) {
	h, arm, m, g := newFixture(t)
	target := targetOf(g, reachablePose(t, m))

	p := [3]float64{target.X, target.Y, target.Z}
	for i := 0; i < 3; i++ {
		res := h.Execute(mustJSON(t, Command{Cmd: "moveto", XYZ: p[:]}))
		if !res.OK {
			t.Fatalf("第 %d 次 moveto 失败: %s", i+1, res.Error)
		}
		got := g.FK(mustSnapshot(arm))
		if d := dist3(got, target); d > 1e-6 {
			t.Errorf("第 %d 次 moveto 后 FK = (%.4f, %.4f, %.4f)，期望 (%.4f, %.4f, %.4f)，差 %.3e",
				i+1, got.X, got.Y, got.Z, target.X, target.Y, target.Z, d)
		}
	}

	// 明确不可达的点必须被挡下，且原因透传（外部项目据此分支）。
	far := robot.Vec3{X: 0, Y: 0, Z: g.PivotZ + g.L1 + g.L2 + 200}
	p2 := [3]float64{far.X, far.Y, far.Z}
	res := h.Execute(mustJSON(t, Command{Cmd: "moveto", XYZ: p2[:]}))
	if res.OK {
		t.Fatal("远超可达壳的目标点应当被拒绝")
	}
	if !strings.Contains(res.Error, robot.ReasonOutWorkspace) {
		t.Errorf("不可达目标的错误应含 %q，实际 %q", robot.ReasonOutWorkspace, res.Error)
	}
	// 被拒绝时不得改动当前位姿。
	if d := dist3(g.FK(mustSnapshot(arm)), target); d > 1e-6 {
		t.Errorf("被拒绝的 moveto 不应改动位姿，实际漂移 %.3e mm", d)
	}
}

// TestMoveXYZ_RejectsBadDelta 矢量参数的形状错误要给**具体**原因。
func TestMoveXYZ_RejectsBadDelta(t *testing.T) {
	h, _, _, _ := newFixture(t)
	nan := math.NaN()
	inf := math.Inf(1)

	cases := []struct {
		name string
		cmd  Command
		want string
	}{
		{"missing delta", Command{Cmd: "movexyz"}, "missing delta"},
		{"delta 只有 2 个数", Command{Cmd: "movexyz", Delta: []float64{1, 2}}, "invalid delta"},
		{"delta 有 4 个数", Command{Cmd: "movexyz", Delta: []float64{1, 2, 3, 4}}, "invalid delta"},
		{"delta 含 NaN", Command{Cmd: "movexyz", Delta: []float64{nan, 0, 0}}, "invalid delta"},
		{"delta 含 Inf", Command{Cmd: "movexyz", Delta: []float64{0, inf, 0}}, "invalid delta"},
		{"全零位移", Command{Cmd: "movexyz", Delta: []float64{0, 0, 0}}, "invalid delta"},
		{"moveto missing xyz", Command{Cmd: "moveto"}, "missing xyz"},
		{"moveto xyz 形状错", Command{Cmd: "moveto", XYZ: []float64{1, 2}}, "invalid xyz"},
		{"moveto xyz 含 NaN", Command{Cmd: "moveto", XYZ: []float64{nan, nan, nan}}, "invalid xyz"},
	}
	for _, c := range cases {
		if got := h.dispatch(c.cmd); got.OK || !strings.Contains(got.Error, c.want) {
			t.Errorf("%s: 应报含 %q 的错误，实际 ok=%v error=%q", c.name, c.want, got.OK, got.Error)
		}
	}

	// 全零要给出"零位移"这个具体说法，而不是笼统的 invalid delta ——
	// 手势控制每帧都可能算出零增量，客户端要能据此直接跳过而不是当故障处理。
	zero := h.dispatch(Command{Cmd: "movexyz", Delta: []float64{0, 0, 0}})
	if !strings.Contains(zero.Error, "zero displacement") {
		t.Errorf("全零位移的错误应点明 zero displacement，实际 %q", zero.Error)
	}

	// 与 v1 对齐：moveto 的 (0,0,0) 是**合法目标点**（不是参数错误），
	// 它该由几何/限位去拒绝，而不是被参数校验当成"坐标格式不对"。
	origin := h.dispatch(Command{Cmd: "moveto", XYZ: []float64{0, 0, 0}})
	if origin.OK {
		t.Error("moveto (0,0,0) 应当不可达")
	}
	if strings.Contains(origin.Error, "invalid xyz") {
		t.Errorf("moveto (0,0,0) 不是参数错误，实际 %q", origin.Error)
	}
}

// TestHome_UsesModelHomePose 归位点取自 robot.yaml，不是抄在代码里的角度。
func TestHome_UsesModelHomePose(t *testing.T) {
	h, arm, m, g := newFixture(t)

	// 先跑远一点，否则"归位"在 HOME 起步下是个空操作，测不出任何东西。
	away := targetOf(g, reachablePose(t, m))
	p := [3]float64{away.X, away.Y, away.Z}
	if got := h.Execute(mustJSON(t, Command{Cmd: "moveto", XYZ: p[:]})); !got.OK {
		t.Fatalf("先移开失败: %s", got.Error)
	}
	if d := dist3(g.FK(mustSnapshot(arm)), g.FK(m.HomePose)); d < 1e-6 {
		t.Fatal("前提不成立：起始位姿就等于 HOME，换一个 reachablePose")
	}

	res := h.Execute(`{"cmd":"home"}`)
	if !res.OK {
		t.Fatalf("home 失败: %s", res.Error)
	}
	for id, want := range m.HomePose {
		if got := mustSnapshot(arm)[id]; math.Abs(got-want) > 1e-9 {
			t.Errorf("home 后关节 %s = %.6f，期望 HOME 的 %.6f", id, got, want)
		}
	}
}

// TestCaps_DerivesFromModelTruth caps 的每个数字都必须能从真值对上账。
//
// 为什么值得一条测试：caps 的**唯一**存在理由就是"外部项目不必抄一份真值"。
// 它一旦和 robot.yaml 对不上，就比没有更糟 —— 对方会拿错的边界去钳位，
// 而且错得很安静（钳出来的坐标照样能发出去）。
func TestCaps_DerivesFromModelTruth(t *testing.T) {
	h, arm, m, g := newFixture(t)

	res := h.Execute(`{"cmd":"caps"}`)
	if !res.OK || res.Caps == nil {
		t.Fatalf("caps 应当成功并携带 caps 字段: ok=%v error=%q", res.OK, res.Error)
	}
	c := res.Caps

	if c.Protocol != protocolVersion {
		t.Errorf("protocol = %d，期望 %d", c.Protocol, protocolVersion)
	}
	if c.Device != arm.kind {
		t.Errorf("device = %q，期望 %q", c.Device, arm.kind)
	}
	if c.Units.Length != "mm" || c.Units.Angle != "deg" {
		t.Errorf("units = %+v，期望 mm/deg", c.Units)
	}

	// 几何：与 LoadGeom 求导结果逐位一致。
	if c.Geom.PivotZ != g.PivotZ || c.Geom.L1 != g.L1 || c.Geom.L2 != g.L2 || c.Geom.ToolR != g.ToolR {
		t.Errorf("geom = %+v，期望 (%.6f, %.6f, %.6f, %.6f)",
			c.Geom, g.PivotZ, g.L1, g.L2, g.ToolR)
	}
	if want := math.Abs(g.L1 - g.L2); c.Geom.ReachMin != want {
		t.Errorf("reachMin = %.6f，期望 |L1−L2| = %.6f", c.Geom.ReachMin, want)
	}
	if want := g.L1 + g.L2; c.Geom.ReachMax != want {
		t.Errorf("reachMax = %.6f，期望 L1+L2 = %.6f", c.Geom.ReachMax, want)
	}

	// 关节限位：顺序 = JointOrder，逐项对真值。
	order := m.JointOrder()
	if len(c.Joints) != len(order) {
		t.Fatalf("joints 数量 = %d，期望 %d（= JointOrder）", len(c.Joints), len(order))
	}
	for i, id := range order {
		j := m.Joint(id)
		if c.Joints[i].ID != id || c.Joints[i].Min != j.Limit.Min || c.Joints[i].Max != j.Limit.Max {
			t.Errorf("joints[%d] = %+v，期望 id=%s min=%.6f max=%.6f",
				i, c.Joints[i], id, j.Limit.Min, j.Limit.Max)
		}
	}

	// 舵机行程：编号 1..N 与 servo 命令一致，且与 state.servo 同序。
	if len(c.Servos) != len(order) {
		t.Fatalf("servos 数量 = %d，期望 %d", len(c.Servos), len(order))
	}
	for i, id := range order {
		acts := m.ActuatorsForJoint(id)
		if len(acts) == 0 {
			t.Fatalf("关节 %s 没有执行器", id)
		}
		s := c.Servos[i]
		if s.N != i+1 || s.Joint != id || s.Min != acts[0].Limits.Min || s.Max != acts[0].Limits.Max {
			t.Errorf("servos[%d] = %+v，期望 n=%d joint=%s min=%.6f max=%.6f",
				i, s, i+1, id, acts[0].Limits.Min, acts[0].Limits.Max)
		}
	}

	// HOME 位姿原样回报。
	for id, want := range m.HomePose {
		if got, ok := c.Home[id]; !ok || math.Abs(got-want) > 1e-12 {
			t.Errorf("home[%s] = %.6f（存在=%v），期望 %.6f", id, got, ok, want)
		}
	}

	// 可达包围盒：必须**包含** HOME 的 TCP（采样网格含限位端点，HOME 在限位内）。
	homeTCP := g.FK(m.HomePose)
	for k, v := range [3]float64{homeTCP.X, homeTCP.Y, homeTCP.Z} {
		if v < c.Workspace.Min[k]-1e-9 || v > c.Workspace.Max[k]+1e-9 {
			t.Errorf("workspace 的第 %d 维 [%.3f, %.3f] 未包含 HOME 的 %.3f —— 采样漏了",
				k, c.Workspace.Min[k], c.Workspace.Max[k], v)
		}
	}
	if c.Workspace.Samples != workspaceSamples {
		t.Errorf("workspace.samples = %d，期望 %d", c.Workspace.Samples, workspaceSamples)
	}

	// 命令清单必须含全部 v1 命令（v1 客户端据此做能力探测）。
	joined := strings.Join(c.Commands, ",")
	for _, want := range []string{"move", "gripper", "servo", "state", "movexyz", "moveto", "home", "caps"} {
		if !strings.Contains(joined, want) {
			t.Errorf("commands = %v，缺少 %q", c.Commands, want)
		}
	}
}

// TestXYZWriteCommands_CarryExternalOrigin v2 的写命令同样必须声明来源。
//
// 与 v1 同源的那条坑：借用 `command` 会让对端页面只更新 Actual
// （画面上半透明臂在动、主臂不动，且那台页面的指令侧滞留旧值）。
func TestXYZWriteCommands_CarryExternalOrigin(t *testing.T) {
	h, arm, m, g := newFixture(t)
	target := targetOf(g, reachablePose(t, m))
	cur := g.FK(mustSnapshot(arm))
	p := [3]float64{target.X, target.Y, target.Z}
	d := [3]float64{target.X - cur.X, target.Y - cur.Y, target.Z - cur.Z}

	cases := []struct {
		name string
		cmd  Command
	}{
		{"movexyz", Command{Cmd: "movexyz", Delta: d[:]}},
		{"moveto", Command{Cmd: "moveto", XYZ: p[:]}},
		{"home", Command{Cmd: "home"}},
	}
	for _, c := range cases {
		if got := h.dispatch(c.cmd); !got.OK {
			t.Fatalf("%s 失败: %q", c.name, got.Error)
		}
		if o := arm.lastOrigin(); o != protocol.OriginExternal {
			t.Errorf("%s 的来源 = %q，期望 %q", c.name, o, protocol.OriginExternal)
		}
	}

	// caps 是只读查询，不得产生任何下发。
	before := len(arm.applied)
	if got := h.Execute(`{"cmd":"caps"}`); !got.OK {
		t.Fatalf("caps 失败: %q", got.Error)
	}
	if len(arm.applied) != before {
		t.Errorf("caps 不得下发命令，applied 从 %d 变成 %d", before, len(arm.applied))
	}
}

// TestV1Commands_UnchangedAfterV2 v2 落地以后 v1 四条命令必须原样可用。
//
// 它不是重复 protocol_test.go —— 那里测的是 v1 **自身**，这里测的是
// "加了 v2 之后 v1 还在"。新增命令最容易犯的错就是顺手改了分发或错误文案。
func TestV1Commands_UnchangedAfterV2(t *testing.T) {
	h, arm, m, g := newFixture(t)
	acts := m.ActuatorsForJoint(m.JointOrder()[0])
	mid := (acts[0].Limits.Min + acts[0].Limits.Max) / 2
	step := 3.0

	// 四条命令都能成功，且 v1 的错误文案一字未改。
	if got := h.Execute(`{"cmd":"gripper","action":"open"}`); !got.OK {
		t.Errorf("v1 gripper 应当仍可用，实际 %q", got.Error)
	}
	if got := h.Execute(mustJSON(t, Command{Cmd: "servo", Servo: intPtr(1), Angle: &mid})); !got.OK {
		t.Errorf("v1 servo 应当仍可用，实际 %q", got.Error)
	}
	if got := h.Execute(`{"cmd":"state"}`); !got.OK || got.State == nil {
		t.Errorf("v1 state 应当仍可用，实际 ok=%v error=%q", got.OK, got.Error)
	}
	if got := h.Execute(mustJSON(t, Command{Cmd: "move", Axis: "x", Direction: "+", Step: &step})); !got.OK {
		t.Errorf("v1 move 应当仍可用，实际 %q", got.Error)
	}

	// v1 的错误优先级：非法 axis 必须先于"缺 step"报出来。
	if got := h.dispatch(Command{Cmd: "move", Axis: "w"}); got.OK || !strings.Contains(got.Error, "invalid axis") {
		t.Errorf("v1 move 的错误优先级变了: ok=%v error=%q", got.OK, got.Error)
	}
	if got := h.dispatch(Command{Cmd: "move", Axis: "x", Direction: "+"}); got.OK || !strings.Contains(got.Error, "missing step") {
		t.Errorf("v1 move 的 missing step 变了: ok=%v error=%q", got.OK, got.Error)
	}

	_ = g
	_ = arm
}

// TestXYZCommands_NeverPanics 矢量参数也不得让调用方 panic。
func TestXYZCommands_NeverPanics(t *testing.T) {
	h, _, _, _ := newFixture(t)
	weird := []string{
		`{"cmd":"movexyz"}`,
		`{"cmd":"movexyz","delta":[]}`,
		`{"cmd":"movexyz","delta":[1,2,3,4,5]}`,
		`{"cmd":"movexyz","delta":"1,2,3"}`,
		`{"cmd":"movexyz","delta":{"x":1,"y":2,"z":3}}`,
		`{"cmd":"movexyz","delta":[null,1,2]}`,
		`{"cmd":"movexyz","delta":[1e999,0,0]}`,
		`{"cmd":"moveto","xyz":null}`,
		`{"cmd":"moveto","xyz":[1,2]}`,
		`{"cmd":"home","delta":[1,2,3]}`,
		`{"cmd":"caps","xyz":[1,2,3]}`,
		`{"cmd":"MOVEXYZ","delta":[1,2,3]}`,
		strings.Repeat(`{"cmd":"movexyz","delta":[1,2,3]}`, 200),
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

// TestHandler_SerializesXYZCommands 并发 movexyz 不得互相吃步（`-race` 下才有意义）。
func TestHandler_SerializesXYZCommands(t *testing.T) {
	h, arm, _, g := newFixture(t)
	before := g.FK(mustSnapshot(arm))

	const n = 20
	var wg sync.WaitGroup
	for i := 0; i < n; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			// 每步 +X 1mm：全部生效时位移必须精确等于 n mm。
			_ = h.Execute(`{"cmd":"movexyz","delta":[1,0,0]}`)
		}()
	}
	wg.Wait()

	got := g.FK(mustSnapshot(arm))
	if d := math.Abs(got.X - (before.X + float64(n))); d > 1e-6 {
		t.Errorf("并发 %d 次 +1mm 后 X = %.4f，期望 %.4f（差 %.3e ⇒ 有步被并发吃掉）",
			n, got.X, before.X+float64(n), d)
	}
}

// TestCaps_JSONShapeIsStable caps 的应答字段是外部项目的**编译期契约** ——
// 改名会让对方静默读到零值。用固定键集把形状钉住。
func TestCaps_JSONShapeIsStable(t *testing.T) {
	h, _, _, _ := newFixture(t)
	raw, err := json.Marshal(h.Execute(`{"cmd":"caps"}`).Caps)
	if err != nil {
		t.Fatalf("序列化 caps 失败: %v", err)
	}
	var m map[string]json.RawMessage
	if err := json.Unmarshal(raw, &m); err != nil {
		t.Fatalf("反序列化失败: %v", err)
	}
	for _, key := range []string{"protocol", "device", "units", "frame", "geom", "joints", "servos", "home", "workspace", "commands"} {
		if _, ok := m[key]; !ok {
			t.Errorf("caps 缺少字段 %q —— 外部项目会静默读到零值", key)
		}
	}
}

func intPtr(v int) *int { return &v }

func dist3(a, b robot.Vec3) float64 {
	return math.Sqrt(sq(a.X-b.X) + sq(a.Y-b.Y) + sq(a.Z-b.Z))
}
