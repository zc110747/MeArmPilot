package robot

// 后端侧的矢状面运动学：FK + 2R 解析 IK。
//
// ## 为什么会有这个文件
//
// 本后端到今天为止是**纯关节级**的：WebSocket 只认 `joint_command`（关节角整帧），
// IK 的唯一权威实现在前端 `frontend/src/robot/kinematics/ik.ts`。
// 新增的 TCP JSON 接口要求"XYZ 相对移动"，而它必然要走 XYZ→关节角这一步，
// 所以后端需要一份自己的解算。
//
// ⚠️ **这里没有引入第二份真值**：
//   - 几何（枢轴高度 / L1 / L2 / 腕→TCP 偏移）在**运行时**从
//     `robot-package/mearm-v1/model/robot.yaml` 的 `links[].length` 求导；
//   - 限位校验**不在这里做** —— 由调用方注入 `Model.Validate`（见 `SolveIK` 的
//     `validate` 形参），TCP 层最终仍走 `controller.Apply` → `Model.Validate`。
//
// ## 判据（关键：不能"我又写了一遍且自洽"）
//
// `kinematics_test.go` 拿**冻结基线**逐例比对，而不是拿本文件的输出自证：
//
//	FK —— robot-package/mearm-v1/tests/cases/fk_cases.json（116 例，前端 FK 产出）
//	IK —— 同目录 ik_cases.json（121 例，含 expect.success / expect.reason 分类）
//
// 任一侧漂移都会当场红 —— 与 sim2sim 的"多实现互证"是同一条思路
// （本文件等于给后端加了第 4 个侧面，并用同一批冻结用例把它关进判据里）。
//
// ## 公式与坐标系（出处：docs/coordinate-system.md §1 / §3 / §3.1）
//
//	右手系 Z-up；+X 正前方、+Y 左、+Z 上；长度 mm、角度 degree。
//
//	FK（被动腕把爪锁成水平后，"腕枢轴→TCP"是**常量水平偏移**，不是随肘转的杆）：
//	    r = L1·sin θs + L2·sin θe + ToolR
//	    z = PivotZ + L1·cos θs + L2·cos θe
//	    x = r·cos θb      y = r·sin θb
//
//	IK（先扣掉腕偏移，再解矢状面 2R）：
//	    θb = atan2(y, x)          r = hypot(x, y)
//	    dr = r − ToolR            dz = z − PivotZ
//	    D  = hypot(dr, dz)        φ = atan2(dr, dz)      // 0 = 天顶
//	    α  = ±acos((D² − L1² − L2²) / (2·L1·L2))
//	    θs = φ − atan2(L2·sin α, L1 + L2·cos α)
//	    θe = θs + α                // ⚠️ elbow 是**绝对角**，不要再叠加肩角
//
// 两处最容易错、且错了不会自己暴露的地方（改本文件前先看这两条）：
//  1. **漏掉 ToolR 那次减法** ⇒ 所有解系统性偏 40mm，而残差自查同样是偏的。
//  2. **把 θe 当相对角再叠一次肩角** ⇒ 机构立刻错位（docs/decisions.md D18）。

import (
	"fmt"
	"math"
	"os"

	"gopkg.in/yaml.v3"
)

// 关节 id（= robot.yaml 的 joints[].id，也是 role 名）。
const (
	JointBase     = "base"
	JointShoulder = "shoulder"
	JointElbow    = "elbow"
	JointTool     = "tool"
)

// IK 失败原因。值与前端 `IkReason` / protocol 错误码对齐，便于直接透传给调用方。
const (
	ReasonOutWorkspace = "OUT_OF_WORKSPACE" // 几何上够不到（超出 2R 可达壳）
	ReasonJointLimit   = "JOINT_LIMIT"      // 几何可达，但两支解都撞关节限位
)

// Geom 机构在矢状面内的几何常量（mm）。
type Geom struct {
	// PivotZ 肩枢轴高度（= shoulder.parentLink 的 length）
	PivotZ float64
	// L1 肩枢轴 → 肘枢轴（= shoulder.childLink 的 length，大臂）
	L1 float64
	// L2 肘枢轴 → **腕枢轴**（= elbow.childLink 的 length，小臂）
	//
	// ⚠️ 它**不是**"肘 → TCP"：腕是被动关节，爪被连杆锁成水平，
	//    所以"腕枢轴 → TCP"是一段常量水平偏移（ToolR），不参与 2R。
	L2 float64
	// ToolR 腕枢轴 → TCP 的**水平**前伸量（= tool.childLink 的 length）。
	ToolR float64
}

// Vec3 三维位置（mm）。
type Vec3 struct{ X, Y, Z float64 }

// IKError 一次 IK 失败。
type IKError struct {
	Reason  string // ReasonOutWorkspace / ReasonJointLimit
	JointID string // ReasonJointLimit 时指向越界的关节；其余为空
	Message string
}

func (e *IKError) Error() string { return e.Message }

// kinematicsFile 只取求导所需的字段。
//
// 刻意**不复用** `yamlFile`：那份结构服务于控制（joints/actuators），
// 不含 `parentLink` / `childLink`，也不含 `links`。往里塞字段会改变
// `Load` 的既有行为 —— 本任务要求"不改现有行为"，所以另起一份。
type kinematicsFile struct {
	Links []struct {
		ID     string  `yaml:"id"`
		Parent string  `yaml:"parent"`
		Length float64 `yaml:"length"`
	} `yaml:"links"`
	Joints []struct {
		ID         string `yaml:"id"`
		Role       string `yaml:"role"`
		ParentLink string `yaml:"parentLink"`
		ChildLink  string `yaml:"childLink"`
	} `yaml:"joints"`
}

// LoadGeom 从 robot.yaml 求导机构几何。
//
// 求导依据是**关节角色**（shoulder / elbow / tool），不是写死 link id：
// 换一台机器人时只要它的关节仍带这些 role，本文件就照样成立。
// 任一环节缺失都**报错**，绝不静默回退到一个"看起来合理"的数 ——
// 静默回退正是"所有解系统性偏 40mm 却不报错"的成因。
func LoadGeom(path string) (Geom, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return Geom{}, fmt.Errorf("读取模型失败: %w", err)
	}
	var f kinematicsFile
	if err := yaml.Unmarshal(raw, &f); err != nil {
		return Geom{}, fmt.Errorf("解析模型失败: %w", err)
	}

	lenOf := map[string]float64{}
	for _, l := range f.Links {
		lenOf[l.ID] = l.Length
	}
	missing := func(id string) error {
		return fmt.Errorf("robot.yaml 缺少 link %q 的 length（或该 link 不存在）", id)
	}

	var shoulderParent, shoulderChild, elbowChild, toolChild string
	for _, j := range f.Joints {
		switch j.Role {
		case JointShoulder:
			shoulderParent, shoulderChild = j.ParentLink, j.ChildLink
		case JointElbow:
			elbowChild = j.ChildLink
		case JointTool:
			toolChild = j.ChildLink
		}
	}

	g := Geom{}
	pick := func(linkID, what string) (float64, error) {
		if linkID == "" {
			return 0, fmt.Errorf("robot.yaml 的 joints 段缺少 %s 关节（或它没有 %s）", what, "parentLink/childLink")
		}
		v, ok := lenOf[linkID]
		if !ok {
			return 0, missing(linkID)
		}
		return v, nil
	}
	if g.PivotZ, err = pick(shoulderParent, "shoulder.parentLink"); err != nil {
		return Geom{}, err
	}
	if g.L1, err = pick(shoulderChild, "shoulder.childLink"); err != nil {
		return Geom{}, err
	}
	if g.L2, err = pick(elbowChild, "elbow.childLink"); err != nil {
		return Geom{}, err
	}
	if g.ToolR, err = pick(toolChild, "tool.childLink"); err != nil {
		return Geom{}, err
	}
	if g.L1 <= 0 || g.L2 <= 0 {
		return Geom{}, fmt.Errorf("求导出的杆长为非正值（L1=%v L2=%v）—— 模型或 role 标注有误", g.L1, g.L2)
	}
	return g, nil
}

// FK 由关节角求 TCP 位置（degree 输入，mm 输出）。
//
// 只关心定位三关节（base / shoulder / elbow）；夹爪与被动腕不影响 TCP 位置。
func (g Geom) FK(joints map[string]float64) Vec3 {
	b := deg2rad(joints[JointBase])
	s := deg2rad(joints[JointShoulder])
	e := deg2rad(joints[JointElbow])

	r := g.L1*math.Sin(s) + g.L2*math.Sin(e) + g.ToolR
	z := g.PivotZ + g.L1*math.Cos(s) + g.L2*math.Cos(e)
	return Vec3{X: r * math.Cos(b), Y: r * math.Sin(b), Z: z}
}

// SolveIK 由目标点求定位三关节的关节角（degree）。
//
// `cur` 是当前关节角，只用于**两支解之间选更近的一支**（2R 天然有 elbow-up /
// elbow-down 两支）。固定选支会在内边界附近翻肘，所以 nearest 是必需的。
//
// `validate` 由调用方注入（正常就是 `(*Model).Validate`）—— 本文件**不自己判限位**，
// 否则就会出现第二份限位真值。传 nil 时跳过限位筛选（仅测试用）。
func (g Geom) SolveIK(target Vec3, cur map[string]float64, validate func(map[string]float64) *Violation) (map[string]float64, *IKError) {
	thetaB := math.Atan2(target.Y, target.X)
	r := math.Hypot(target.X, target.Y)
	dr := r - g.ToolR
	dz := target.Z - g.PivotZ
	d := math.Hypot(dr, dz)

	// 可达壳：|L1−L2| ≤ D ≤ L1+L2。留 1e-9 容差，避免边界点被浮点抖动误杀。
	if d > g.L1+g.L2+1e-9 || d < math.Abs(g.L1-g.L2)-1e-9 {
		return nil, &IKError{
			Reason: ReasonOutWorkspace,
			Message: fmt.Sprintf("目标 (%.2f, %.2f, %.2f) 不可达：距肩枢轴 %.2fmm 超出可达壳 [%.2f, %.2f]",
				target.X, target.Y, target.Z, d, math.Abs(g.L1-g.L2), g.L1+g.L2),
		}
	}

	cosA := (d*d - g.L1*g.L1 - g.L2*g.L2) / (2 * g.L1 * g.L2)
	cosA = math.Min(1, math.Max(-1, cosA)) // 数值钳位：D 落在边界时可能算出 1±eps
	a := math.Acos(cosA)
	phi := math.Atan2(dr, dz)

	up := g.branch(thetaB, phi, +a)
	down := g.branch(thetaB, phi, -a)

	if validate == nil {
		return nearest(cur, up, down), nil
	}

	upOK := validate(up) == nil
	downOK := validate(down) == nil
	switch {
	case upOK && downOK:
		return nearest(cur, up, down), nil
	case upOK:
		return up, nil
	case downOK:
		return down, nil
	default:
		// 两支都越界：报**离当前姿态近的那一支**所涉关节 —— 与前端一致
		// （ik_cases.json 的 expect.joint 就是这么标的）。
		picked := nearest(cur, up, down)
		v := validate(picked)
		jointID := ""
		if v != nil {
			jointID = v.JointID
		}
		return nil, &IKError{
			Reason:  ReasonJointLimit,
			JointID: jointID,
			Message: fmt.Sprintf("目标 (%.2f, %.2f, %.2f) 几何可达，但两支解都越界",
				target.X, target.Y, target.Z),
		}
	}
}

// branch 由矢状面的 φ 与肘相对角 α 解出一支完整解。
func (g Geom) branch(thetaB, phi, alpha float64) map[string]float64 {
	thetaS := phi - math.Atan2(g.L2*math.Sin(alpha), g.L1+g.L2*math.Cos(alpha))
	return map[string]float64{
		JointBase:     rad2deg(thetaB),
		JointShoulder: rad2deg(thetaS),
		// ⚠️ elbow 是绝对角，直接就是 θs+α —— 不要再叠加肩角（D18）。
		JointElbow: rad2deg(thetaS + alpha),
	}
}

// nearest 两支里挑离当前姿态更近的一支（只在 base/shoulder/elbow 上比）。
func nearest(cur, a, b map[string]float64) map[string]float64 {
	da, db := 0.0, 0.0
	for _, id := range []string{JointBase, JointShoulder, JointElbow} {
		da += math.Abs(a[id] - cur[id])
		db += math.Abs(b[id] - cur[id])
	}
	if da <= db {
		return a
	}
	return b
}

func deg2rad(d float64) float64 { return d * math.Pi / 180 }
func rad2deg(r float64) float64 { return r * 180 / math.Pi }
