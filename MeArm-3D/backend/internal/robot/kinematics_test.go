package robot

// kinematics_test.go —— 后端运动学的判据。
//
// ★ **判据不是本包自己的输出**：全部期望值来自**冻结基线**
//   `robot-package/mearm-v1/tests/cases/{fk,ik}_cases.json` —— 那是前端 `fk.ts` / `ik.ts`
//   实跑采集的行为快照。拿本文件的实现去比对它自己算出来的数，等于自证。
//
// 这正是 sim2sim 的同一条思路：多实现互证。后端这份是第 4 个侧面，
// 用同一批冻结用例把它关进判据里 —— 它一旦与前端漂移，这里当场红。

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// casesDir 冻结基线目录（相对本包目录 backend/internal/robot）。
const casesDir = "../../../robot-package/mearm-v1/tests/cases"

// modelPath 模型真值（几何的唯一来源）。
const modelPath = "../../../robot-package/mearm-v1/model/robot.yaml"

// fkTolMm FK 比对容差。与 sim2sim 给 mearm-v1 登记的容差同档（1e-6 mm）：
// 实测残差在 1e-13 量级，留 7 个数量级余量。
const fkTolMm = 1e-6

type fkCase struct {
	ID          string             `json:"id"`
	Joints      map[string]float64 `json:"joints"`
	TCPFrontend []float64          `json:"tcpFrontend"`
}

type ikCase struct {
	ID     string    `json:"id"`
	Kind   string    `json:"kind"`
	Target []float64 `json:"target"`
	Expect struct {
		Success bool               `json:"success"`
		Reason  string             `json:"reason"`
		Joint   string             `json:"joint"`
		Branch  string             `json:"branch"`
		Joints  map[string]float64 `json:"joints"`
	} `json:"expect"`
}

// dist3 三维欧氏距离。
//
// 不用 `math.Hypot` 三参版 —— 标准库的 `Hypot` 只接受两个参数（Go 1.21 仍如此），
// 传三个是编译错误。
func dist3(a Vec3, b []float64) float64 {
	dx, dy, dz := a.X-b[0], a.Y-b[1], a.Z-b[2]
	return math.Sqrt(dx*dx + dy*dy + dz*dz)
}

func readJSON(t *testing.T, name string, dst any) {
	t.Helper()
	p := filepath.Join(casesDir, name)
	raw, err := os.ReadFile(p)
	if err != nil {
		t.Fatalf("读冻结基线失败（%s）: %v —— 判据缺失，测试不能跳过", p, err)
	}
	if err := json.Unmarshal(raw, dst); err != nil {
		t.Fatalf("解析冻结基线失败（%s）: %v", p, err)
	}
}

func mustLoad(t *testing.T) (*Model, Geom) {
	t.Helper()
	m, err := Load(modelPath)
	if err != nil {
		t.Fatalf("加载模型失败: %v", err)
	}
	g, err := LoadGeom(modelPath)
	if err != nil {
		t.Fatalf("求导几何失败: %v", err)
	}
	return m, g
}

// TestLoadGeom_DerivesFromModelLinks 几何必须**从模型求导**，不是抄在代码里的常数。
//
// 这里的期望值是当前 mearm-v1 的冻结几何（立柱 60 / 大臂 80 / 小臂 80 / 腕→TCP 40）。
// 改 robot.yaml 的 links[].length ⇒ 本测试必须红，并由人来确认这是一次有意的改真值。
func TestLoadGeom_DerivesFromModelLinks(t *testing.T) {
	_, g := mustLoad(t)

	want := Geom{PivotZ: 60, L1: 80, L2: 80, ToolR: 40}
	if g != want {
		t.Errorf("求导出的几何 = %+v，期望 %+v", g, want)
	}
	// 额外的不变式：2R 可达壳必须非退化，否则 IK 的 acos 会一直钳在边界。
	if g.L1 <= 0 || g.L2 <= 0 {
		t.Errorf("杆长必须为正：L1=%v L2=%v", g.L1, g.L2)
	}
}

// TestFK_MatchesFrozenFrontendBaseline 116 例逐点比对前端 FK 的冻结输出。
func TestFK_MatchesFrozenFrontendBaseline(t *testing.T) {
	_, g := mustLoad(t)

	var doc struct {
		Cases []fkCase `json:"cases"`
	}
	readJSON(t, "fk_cases.json", &doc)
	if len(doc.Cases) == 0 {
		t.Fatal("冻结基线为空 —— 判据缺失，不能算通过")
	}

	worst := 0.0
	worstID := ""
	for _, c := range doc.Cases {
		if len(c.TCPFrontend) != 3 {
			t.Fatalf("用例 %s 的 tcpFrontend 不是三维: %v", c.ID, c.TCPFrontend)
		}
		got := g.FK(c.Joints)
		d := dist3(got, c.TCPFrontend)
		if d > worst {
			worst, worstID = d, c.ID
		}
		if d > fkTolMm {
			t.Errorf("FK(%s) = (%.6f, %.6f, %.6f)，冻结基线 (%.6f, %.6f, %.6f)，差 %.3e mm > %.0e",
				c.ID, got.X, got.Y, got.Z,
				c.TCPFrontend[0], c.TCPFrontend[1], c.TCPFrontend[2], d, fkTolMm)
		}
	}
	t.Logf("FK %d 例 · 最差残差 %.3e mm（容差 %.0e）· 用例 %s", len(doc.Cases), worst, fkTolMm, worstID)
}

// TestIK_RoundTripsFrozenTargets 115 例可达目标：IK → FK 必须回到原目标。
//
// 只回读"定位三关节"，夹爪不参与（IK 也不该碰它）。
func TestIK_RoundTripsFrozenTargets(t *testing.T) {
	m, g := mustLoad(t)

	var doc struct {
		Cases []ikCase `json:"cases"`
	}
	readJSON(t, "ik_cases.json", &doc)

	checked, worst := 0, 0.0
	for _, c := range doc.Cases {
		if !c.Expect.Success || len(c.Target) != 3 {
			continue
		}
		target := Vec3{X: c.Target[0], Y: c.Target[1], Z: c.Target[2]}
		// cur 用冻结基线记录的解：nearest 选支在"当前就是它自己"时应当选到同一支。
		got, ierr := g.SolveIK(target, c.Expect.Joints, m.Validate)
		if ierr != nil {
			t.Errorf("IK(%s) 应当可解，却被判 %s: %s", c.ID, ierr.Reason, ierr.Message)
			continue
		}
		back := g.FK(got)
		d := dist3(back, c.Target)
		if d > worst {
			worst = d
		}
		if d > fkTolMm {
			t.Errorf("IK(%s) 回程残差 %.3e mm > %.0e（解 = base %.4f / shoulder %.4f / elbow %.4f）",
				c.ID, d, fkTolMm, got[JointBase], got[JointShoulder], got[JointElbow])
		}
		checked++
	}
	if checked == 0 {
		t.Fatal("没有一例被检查 —— 判据缺失，不能算通过")
	}
	t.Logf("IK 回程 %d 例 · 最差残差 %.3e mm（容差 %.0e）", checked, worst, fkTolMm)
}

// TestIK_ReproducesFrozenFailureReasons 6 例不可达目标：失败**分类**必须与前端一致。
//
// 为什么连失败都要比对：`OUT_OF_WORKSPACE`（几何够不到）与 `JOINT_LIMIT`
// （够得到但撞限位）在调用方是两条完全不同的分支 —— 分类错了，
// 上层的错误码就是错的，而"命令被拒"这件事本身看起来是一样的。
func TestIK_ReproducesFrozenFailureReasons(t *testing.T) {
	m, g := mustLoad(t)

	var doc struct {
		Cases []ikCase `json:"cases"`
	}
	readJSON(t, "ik_cases.json", &doc)

	checked := 0
	for _, c := range doc.Cases {
		if c.Expect.Success || len(c.Target) != 3 {
			continue
		}
		target := Vec3{X: c.Target[0], Y: c.Target[1], Z: c.Target[2]}
		_, ierr := g.SolveIK(target, map[string]float64{}, m.Validate)
		if ierr == nil {
			t.Errorf("IK(%s) 应当失败（冻结基线 reason=%s），却解出来了", c.ID, c.Expect.Reason)
			continue
		}
		if ierr.Reason != c.Expect.Reason {
			t.Errorf("IK(%s) 失败原因 = %s，冻结基线 = %s", c.ID, ierr.Reason, c.Expect.Reason)
		}
		checked++
	}
	if checked == 0 {
		t.Fatal("没有一例被检查 —— 判据缺失，不能算通过")
	}
	t.Logf("IK 失败分类 %d 例全部与冻结基线一致", checked)
}

// TestSolveIK_NearestPicksRecordedBranch 两支都合法时，nearest 必须选到前端记录的那支。
//
// 固定选支会在内边界附近翻肘（前端 `prefer:'nearest'` 就是为此存在的），
// 所以这里钉住的不是"某一支对不对"，而是**选支策略本身**。
func TestSolveIK_NearestPicksRecordedBranch(t *testing.T) {
	m, g := mustLoad(t)

	var doc struct {
		Cases []ikCase `json:"cases"`
	}
	readJSON(t, "ik_cases.json", &doc)

	checked := 0
	for _, c := range doc.Cases {
		if !c.Expect.Success || len(c.Target) != 3 {
			continue
		}
		// 只比对前端**明确标注了支**的用例。
		if c.Expect.Branch == "" || c.Expect.Joints == nil {
			continue
		}
		target := Vec3{X: c.Target[0], Y: c.Target[1], Z: c.Target[2]}
		got, ierr := g.SolveIK(target, c.Expect.Joints, m.Validate)
		if ierr != nil {
			continue // 分类由另一条测试负责
		}
		// elbow-up = 肘相对角 α>0 ⇒ θe > θs；elbow-down 反之。
		rel := got[JointElbow] - got[JointShoulder]
		gotBranch := "elbow-down"
		if rel > 0 {
			gotBranch = "elbow-up"
		}
		if gotBranch != c.Expect.Branch {
			t.Errorf("IK(%s) 选支 = %s（θe−θs=%.4f），冻结基线 = %s",
				c.ID, gotBranch, rel, c.Expect.Branch)
		}
		checked++
	}
	t.Logf("选支策略比对 %d 例", checked)
}

// TestLoadGeom_FailsLoudlyOnBrokenModel 模型缺 link 时必须**报错**，不能静默回退。
//
// 静默回退正是"所有解系统性偏 40mm 却不报错"的成因（见 kinematics.go 文件头）。
func TestLoadGeom_FailsLoudlyOnBrokenModel(t *testing.T) {
	dir := t.TempDir()
	bad := filepath.Join(dir, "robot.yaml")
	content := "links:\n  - {id: base_link, parent: null, length: 0}\njoints:\n  - {id: shoulder, role: shoulder, parentLink: nope, childLink: nope}\n"
	if err := os.WriteFile(bad, []byte(content), 0o600); err != nil {
		t.Fatalf("写临时模型失败: %v", err)
	}
	if _, err := LoadGeom(bad); err == nil {
		t.Error("link 缺失时应当报错，却返回了成功")
	}
}
