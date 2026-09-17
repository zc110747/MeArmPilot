// package protocol 定义了与 arm-device 固件对接的串口指令集。
//
// 串口处理严格依赖 arm-device 的指令协议（详见 arm-device/README.md 与
// core/cmd.c）：所有下发给设备的内容都必须是此处 grammar 认可的命令，避免
// 非法字符串烧入固件造成未知行为。Web / TCP 收到的控制意图最终都归一化为
// 下面这些命令文本，再经串口写出。
package protocol

import (
	"fmt"
	"strconv"
	"strings"
)

// 设备支持的舵机 id 与固件一致：6=夹取 7=右 8=左 9=底座
var validIDs = map[int]bool{6: true, 7: true, 8: true, 9: true}

// AxisMap 描述双 3D 摇杆（遥控形式）到 4 路舵机的映射关系：
//   - 左摇杆 X -> SERVO_BASE(9)，Y -> SERVO_LEFT(8)
//   - 右摇杆 X -> SERVO_GRIP(6)，Y -> SERVO_RIGHT(7)
// 每个轴可独立反转方向（invert_*），无需改代码即可适配实际安装。
// DeadFrac 为动作死区占满偏行程的比例（由 config deadband_deg 换算，见
// DeadFracFromDeg）：|v| <= DeadFrac 的轴视为居中、不产生任何下发。
type AxisMap struct {
	LXServo int // 左摇杆 X 轴 -> 舵机 id（默认 9=底座）
	LYServo int // 左摇杆 Y 轴 -> 舵机 id（默认 8=左舵）
	RXServo int // 右摇杆 X 轴 -> 舵机 id（默认 6=夹取）
	RYServo int // 右摇杆 Y 轴 -> 舵机 id（默认 7=右舵）
	InvLX   bool
	InvLY   bool
	InvRX   bool
	InvRY   bool
	DeadFrac float64
}

// DefaultAxisMap 返回推荐映射（遥控形式：左=底座/左舵，右=夹取/右舵）。
func DefaultAxisMap() AxisMap {
	return AxisMap{
		LXServo: 9, LYServo: 8, RXServo: 6, RYServo: 7,
		InvLX: false, InvLY: false, InvRX: false, InvRY: false,
		DeadFrac: DefaultDeadFrac(),
	}
}

// ---------------------------------------------------------------------------
// 摇杆方向：串口 invert_* ⇄ 网络链路 invert 的换算
// ---------------------------------------------------------------------------

// FirmwareInvertsAxis 报告 arm-device 固件对该舵机 id 的「摇杆 raw → 步长方向」
// 是否与其它三轴相反。
//
// 真值来源（唯一）：`MeArm-Device/core/joystick.c` 的 joystick_delta()
//
//	bool positive = (id == 8) ? past_hi : past_lo;
//
// 即 9/6/7 三轴是 raw<200 → 正步进，唯独 8 轴反过来（raw>800 → 正步进）。
// `config.yaml` 里 `lx/rx/ry = true、ly = false` 这组出厂默认值，
// 正是为了把这个不一致抹平，使"推杆正方向 → 该舵机角度增大"对四轴都成立。
func FirmwareInvertsAxis(armServoID int) bool { return armServoID == 8 }

// NetInvertFor 把串口链路的 `joystick.invert_*` 换算成**网络链路**
// （TCP → MeArm-3D 的 `servo` 绝对角）应当使用的反向标志。
//
// ⚠️ 为什么不能原样复用：串口链路上"推杆正方向 → 舵机角度增大"这个结果，
//    是由 `invert_*` **和固件的方向差异**共同决定的；而 TCP 链路直达
//    MeArm-3D 的 `servo`，中间**没有固件这一层**。把 invert 原样搬过去，
//    两种模式的推杆方向会**正好相反**（推右：串口角度增大 / 网络角度减小）。
//
// 推导（s = 推杆方向符号，+1 为网页上的正方向）：
//
//	串口：axisRaw 先把 |v| 映到 raw，再在 inv 时镜像（raw ← 1023-raw），
//	      故 raw-512 的符号 P：
//	          P = inv ? -s : +s        （居中值 512 不参与镜像）
//	      固件步长符号：
//	          axes 9/6/7（固件正常）  = -P = inv ? +s : -s
//	          axis 8    （固件反相）  = +P = inv ? -s : +s
//	网络：axisSpeed 只在 inv 时取负，故步长符号 = netInv ? -s : +s
//
// 令两种模式对同一个 s 得到**同号**的步长：
//
//	axes 9/6/7 → netInv = !inv
//	axis 8     → netInv = inv
//
// 代入出厂默认（9/6/7 inv=true、8 inv=false）⇒ 四轴 netInv 全为 false，
// 即网络模式下"推杆正方向 = 该舵机角度增大"，与串口模式手感一致。
// 用户若为适配实际机构改了某个 invert，两种模式会**一起**翻转 ——
// 这正是"一个旋钮管两处"的意图：不出现第二份方向真值。
func NetInvertFor(armServoID int, inv bool) bool {
	if FirmwareInvertsAxis(armServoID) {
		return inv
	}
	return !inv
}

// Validate 校验一条原始命令是否符合 arm-device 语法。
// 返回 (ok, normalized, error)。normalized 是规整后的下发文本（去除多余空白）。
// 由于固件对所有字面量用 PSTR 且容忍大小写，这里做宽松但安全的校验：
// 拒绝明显越界/非法 id，避免把垃圾写进串口。
func Validate(raw string) (bool, string, error) {
	line := strings.TrimSpace(raw)
	if line == "" {
		return false, "", fmt.Errorf("空命令")
	}
	// 替换逗号分隔为空格，便于统一 tokenize
	norm := strings.ReplaceAll(line, ",", " ")
	fields := strings.Fields(norm)
	if len(fields) == 0 {
		return false, "", fmt.Errorf("空命令")
	}
	verb := strings.ToUpper(fields[0])

	// 简写 S<id>=<angle> 单独处理：verb 形如 "S7=90"，与下面的 switch verb 不匹配
	if len(verb) >= 3 && verb[0] == 'S' && verb[1] >= '6' && verb[1] <= '9' && verb[2] == '=' {
		id, _ := strconv.Atoi(verb[1:2])
		parts := strings.SplitN(verb, "=", 2)
		ang, err := strconv.Atoi(parts[1])
		if err != nil || !validIDs[id] || ang < 0 || ang > 180 {
			return false, "", fmt.Errorf("S 简写非法: %s", verb)
		}
		return true, fmt.Sprintf("S%d=%d", id, ang), nil
	}

	switch verb {
	case "HELP", "STATUS", "?", "RESET", "ADC":
		if len(fields) != 1 {
			return false, "", fmt.Errorf("%s 不接受参数", verb)
		}
		return true, verb, nil

	case "JOYHW", "IRHW":
		if len(fields) != 2 || !isOnOff(fields[1]) {
			return false, "", fmt.Errorf("%s 需要 ON|OFF", verb)
		}
		return true, fmt.Sprintf("%s %s", verb, strings.ToUpper(fields[1])), nil

	case "SEQ":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("SEQ 需要 1|3|7|9|STOP|?")
		}
		arg := strings.ToUpper(fields[1])
		if arg == "STOP" || arg == "?" {
			return true, fmt.Sprintf("SEQ %s", arg), nil
		}
		n, err := strconv.Atoi(fields[1])
		if err != nil || (n != 1 && n != 3 && n != 7 && n != 9) {
			return false, "", fmt.Errorf("SEQ 参数非法: %s", fields[1])
		}
		return true, fmt.Sprintf("SEQ %d", n), nil

	case "STOP", "AUTO":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("%s 需要 <id>", verb)
		}
		id, err := strconv.Atoi(fields[1])
		if err != nil || !validIDs[id] {
			return false, "", fmt.Errorf("%s 非法 id: %s", verb, fields[1])
		}
		return true, fmt.Sprintf("%s %d", verb, id), nil

	case "JOY":
		// JOY <id> <raw> 或 JOY <r9> <r8> <r6> <r7>
		if len(fields) == 3 {
			id, err := strconv.Atoi(fields[1])
			if err != nil || !validIDs[id] {
				return false, "", fmt.Errorf("JOY 非法 id: %s", fields[1])
			}
			raw, err := strconv.Atoi(fields[2])
			if err != nil || raw < 0 || raw > 1023 {
				return false, "", fmt.Errorf("JOY 非法 raw: %s (0..1023)", fields[2])
			}
			return true, fmt.Sprintf("JOY %d %d", id, raw), nil
		}
		if len(fields) == 5 {
			raws := make([]int, 4)
			for i := 0; i < 4; i++ {
				v, err := strconv.Atoi(fields[1+i])
				if err != nil || v < 0 || v > 1023 {
					return false, "", fmt.Errorf("JOY 非法 raw: %s (0..1023)", fields[1+i])
				}
				raws[i] = v
			}
			return true, fmt.Sprintf("JOY %d %d %d %d", raws[0], raws[1], raws[2], raws[3]), nil
		}
		return false, "", fmt.Errorf("JOY 语法: JOY <id> <raw> | JOY <r9> <r8> <r6> <r7>")

	case "IR":
		if len(fields) != 2 {
			return false, "", fmt.Errorf("IR 需要 <hexcode>")
		}
		if !isHex32(fields[1]) {
			return false, "", fmt.Errorf("IR 非法 hex: %s", fields[1])
		}
		return true, fmt.Sprintf("IR %s", fields[1]), nil

	case "IRLEARN", "IRCODES", "IRCLEAR":
		// IRLEARN 需要 1|3|5|7|9，其余无参
		if verb == "IRLEARN" {
			if len(fields) != 2 {
				return false, "", fmt.Errorf("IRLEARN 需要 <1|3|5|7|9>")
			}
			n, err := strconv.Atoi(fields[1])
			if err != nil || (n != 1 && n != 3 && n != 5 && n != 7 && n != 9) {
				return false, "", fmt.Errorf("IRLEARN 非法槽位: %s", fields[1])
			}
			return true, fmt.Sprintf("IRLEARN %d", n), nil
		}
		if len(fields) != 1 {
			return false, "", fmt.Errorf("%s 不接受参数", verb)
		}
		return true, verb, nil

	case "SET":
		if len(fields) < 3 || len(fields)%2 != 1 {
			return false, "", fmt.Errorf("SET 语法: SET <id> <ang> [id ang]..")
		}
		pairs := (len(fields) - 1) / 2
		if pairs > 3 {
			return false, "", fmt.Errorf("SET 最多 3 个舵机")
		}
		out := strings.Builder{}
		out.WriteString("SET")
		for i := 1; i+1 < len(fields); i += 2 {
			id, err := strconv.Atoi(fields[i])
			if err != nil || !validIDs[id] {
				return false, "", fmt.Errorf("SET 非法 id: %s", fields[i])
			}
			ang, err := strconv.Atoi(fields[i+1])
			if err != nil || ang < 0 || ang > 180 {
				return false, "", fmt.Errorf("SET 非法角度: %s (0..180)", fields[i+1])
			}
			out.WriteString(fmt.Sprintf(" %d %d", id, ang))
		}
		return true, out.String(), nil
	}

	return false, "", fmt.Errorf("未知命令: %s", verb)
}

func isOnOff(s string) bool {
	u := strings.ToUpper(s)
	return u == "ON" || u == "OFF"
}

func isHex32(s string) bool {
	t := strings.TrimPrefix(strings.ToUpper(s), "0X")
	if t == "" || len(t) > 8 {
		return false
	}
	for _, c := range t {
		if !((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) {
			return false
		}
	}
	return true
}

// ---- 网页摇杆响应整形（10° 动作死区 + 命令区间映射）----
//
// 网页摇杆视觉倾角满偏 = maxTilt 0.55 rad ≈ 31.5°（web/static/js/joystick3d.js）。
// 需求：偏移 ≤ 10°（约 32% 行程）的轴一律视为居中、不下发；超过 10° 后直接映射
// 进"能让下位机动作"的 raw 区间——固件摇杆死区为 raw 200..800（偏离中位 > 60.9%
// 才触发步进），故死区后最小输出取满偏的 65%，保证一出手即为有效步进（约 2°/次），
// 满偏行程对应 raw 0/1023（固件最大步长 10°/次）。
const (
	// JoyMaxTiltDeg 网页摇杆视觉倾角满偏（度），与 joystick3d.js maxTilt 对应。
	JoyMaxTiltDeg = 31.5
	// DefaultDeadbandDeg 默认动作死区（度）。
	DefaultDeadbandDeg = 5.0
	// joyCmdMin 死区后的最小输出（占半程 512 的比例）。必须 > 0.6094
	// （= 312/512，即固件 raw 200/800 阈值），否则发出的 raw 仍在固件死区内。
	joyCmdMin = 0.65
)

// DeadFracFromDeg 把死区（度，视觉倾角）换算为归一化行程比例。
// deg<=0 表示禁用死区（返回 0）；上限 0.9 防止把摇杆配成"永远不动"。
func DeadFracFromDeg(deg float64) float64 {
	if deg <= 0 {
		return 0
	}
	f := deg / JoyMaxTiltDeg
	if f > 0.9 {
		f = 0.9
	}
	return f
}

// DefaultDeadFrac 默认死区比例（对应 10°）。
func DefaultDeadFrac() float64 { return DeadFracFromDeg(DefaultDeadbandDeg) }

// axisRaw 把归一化坐标 v∈[-1,1] 转换为设备 raw 0..1023（512=中位）：
//   - |v| <= deadFrac（动作死区）：输出 512（固件死区，不动作）；
//   - |v| >  deadFrac：线性映射到满偏的 65%..100%（raw≈179..0 / 845..1023），
//     一旦超出死区即为有效步进，偏移越大步长越大。
//
// inv=true 时左右镜像（适配实际安装方向）。
func axisRaw(v float64, inv bool, deadFrac float64) int {
	a := v
	if a < 0 {
		a = -a
	}
	if a > 1 {
		a = 1
	}
	var out float64
	if a > deadFrac {
		u := (a - deadFrac) / (1 - deadFrac)
		out = joyCmdMin + u*(1-joyCmdMin)
		if v < 0 {
			out = -out
		}
	}
	r := clampRaw(512 + int(out*512))
	if inv && out != 0 { // 居中值 512 不参与镜像，保证死区语义精确
		r = 1023 - r
	}
	return r
}

// JoystickToJOYDual 把左右两个 3D 摇杆的归一化坐标 (∈[-1,1]) 合并为一条设备
// JOY 四轴帧：JOY <raw9> <raw8> <raw6> <raw7>（顺序与固件一致，ids={9,8,6,7}）。
//   - 左摇杆 X -> 底座(9)，Y -> 左舵(8)
//   - 右摇杆 X -> 夹取(6)，Y -> 右舵(7)
// 每轴先过动作死区（m.DeadFrac，默认 10°≈0.32 行程）再转 raw：
// 死区内 -> raw 512（固件不动作）；超出 -> 立即进入有效步进区间。
// 配合 JoystickHasCommand，四轴全居中时调用方可整帧跳过下发（串口零流量）。
func JoystickToJOYDual(lx, ly, rx, ry float64, m AxisMap) string {
	r9 := axisRaw(lx, m.InvLX, m.DeadFrac) // 底座
	r8 := axisRaw(ly, m.InvLY, m.DeadFrac) // 左舵
	r6 := axisRaw(rx, m.InvRX, m.DeadFrac) // 夹取
	r7 := axisRaw(ry, m.InvRY, m.DeadFrac) // 右舵
	return fmt.Sprintf("JOY %d %d %d %d", r9, r8, r6, r7)
}

// JoystickHasCommand 报告是否存在超出动作死区的轴。
// 返回 false 时（四轴全居中）调用方应跳过下发——不下发任何指令。
func JoystickHasCommand(lx, ly, rx, ry float64, m AxisMap) bool {
	for _, v := range [4]float64{lx, ly, rx, ry} {
		if v < 0 {
			v = -v
		}
		if v > m.DeadFrac {
			return true
		}
	}
	return false
}

// JoystickToJOY 兼容旧的单摇杆调用：仅驱动左摇杆，右摇杆保持中位。
func JoystickToJOY(x, y float64, m AxisMap) string {
	return JoystickToJOYDual(x, y, 0, 0, m)
}

func clampRaw(v int) int {
	if v < 0 {
		return 0
	}
	if v > 1023 {
		return 1023
	}
	return v
}
