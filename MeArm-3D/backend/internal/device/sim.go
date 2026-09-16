package device

import (
	"fmt"
	"log"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"

	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// SimTuning 控制"假固件"的物理特性。默认值与前端 MockTransport 对齐（240°/s、
// 15ms 延迟、20ms tick），这样两条链路的观感一致，验收数字可以互相印证。
type SimTuning struct {
	// MaxServoSpeed 舵机最大角速度（度/秒）；0 或负 = 瞬时到位
	MaxServoSpeed float64
	// LatencyMs 指令送达延迟（ms）—— 延迟期内不回复执
	LatencyMs int
	// TickMs 位置推进周期（ms）
	TickMs int
	// EnforceLimits 是否做限位校验（模拟固件的 ERR JOINT）
	EnforceLimits bool
	// BootMs 开机静默窗口（ms）：模拟 Uno bootloader 交权期，
	// 窗口内下发的指令被吞掉且不回执（对齐 skill arm-robot-serial 关键坑 1/2）
	BootMs int
}

// DefaultSimTuning 返回推荐参数。
func DefaultSimTuning() SimTuning {
	return SimTuning{
		MaxServoSpeed: 240,
		LatencyMs:     15,
		TickMs:        20,
		EnforceLimits: true,
		BootMs:        0,
	}
}

// 本文件的 sim 模拟的是**固件**：它只认识舵机（`S6..S9`），
// 关节↔舵机的换算在 `tick()` 的上报里做，与真机链路的 `serial.go` 同构。
//
// # 两条上报路径（这是"上位机状态跟随"的关键，别把它们混成一条）
//
//	JR 受理 → 位置推进 → `STATE …`      关节角，**命令引起**（OriginCommand）
//	SET/JOY/S<n>= → 位置推进 → `# SERVO …` 舵机角，**设备侧外部变化**（OriginDevice）
//
// 为什么要分成两条：JR 是上位机命令，设备只是在执行 —— 状态跟随命令即可；
// 而 SET / JOY 在本项目里模拟的是**命令之外的改动**（真机上对应硬件摇杆、红外遥控、
// 面板手拧、调试直控）。这类改动必须让界面**跟着走**，否则 UI 会停在一个
// 与现场不符的姿态上，而且看不出任何异常。
//
// ⚠️ 因此不要把 `# SERVO` 也用在 JR 路径上：那会让界面把命令侧一路拖向
//    实际位置，拖滑杆时会看到滑杆被"回拉"（命令语义被跟随语义吃掉）。

var (
	reJR = regexp.MustCompile(`(?i)^\s*JR\b(.*)$`)
	// S<id>=<angle> 简写（固件 cmd.c 的 shorthand，id 只能 6..9）
	reServoShorthand = regexp.MustCompile(`(?i)^\s*S([6-9])=(-?\d+)\s*$`)
)

// SimDevice 模拟 arm-device 固件。
//
// 它**不是**等值回显器：内部维护舵机空间的 target/actual，以有限角速度逼近，
// 并把 actual 反算回关节角主动上报 STATE。这样"标定可逆性 + 状态滞后 + 收敛"
// 三件事才会被真实检验，而不是被一个理想回显掩盖到 Phase 11。
type SimDevice struct {
	model   *robot.Model
	order   []string
	tuning  SimTuning
	startAt time.Time

	mu       sync.Mutex
	actualS  map[int]float64 // 舵机实际角（度）
	targetS  map[int]float64 // 舵机目标角（度）
	closed   bool
	ticking  bool
	bootDone bool

	// externalMotion 当前这轮运动是否由**设备侧外部手段**引起（SET / JOY / S<n>=）。
	// 决定 `tick()` 用哪条上报路径（见文件头"两条上报路径"）。
	externalMotion bool
	// jrBusy 是否有 JR 在途（含 LatencyMs 延迟窗口）。
	// ★ 这是 sim 侧的"低优先级"：外部上报**让位于**上位机命令 ——
	//   在途期间只记脏，不下发，等命令了结后再补报最新值（latest-wins）。
	//   与固件侧"TX 环低水位才发"是同一条规则的两端实现。
	jrBusy bool
	// reportDirty 有尚未报出的外部位置变化；值不缓存，补报时取当时的 actual，
	// 所以天然是 latest-wins（中间帧被合并掉，不会积压）。
	reportDirty bool

	lines   chan Line
	statusF []StatusHandler
}

// NewSim 创建 sim 设备，初始位置取模型 HOME 位（四个舵机恰好 90°）。
func NewSim(m *robot.Model, t SimTuning) (*SimDevice, error) {
	home := make(map[int]float64, len(m.Actuators))
	for i := range m.Actuators {
		a := &m.Actuators[i]
		j := m.Joint(a.JointID)
		if j == nil {
			return nil, fmt.Errorf("执行器 %s 指向未知关节 %s", a.ID, a.JointID)
		}
		home[a.Channel] = robot.JointToServo(a, m.HomePose[j.ID])
	}
	return &SimDevice{
		model:    m,
		order:    m.JointOrder(),
		tuning:   t,
		startAt:  time.Now(),
		actualS:  copyMap(home),
		targetS:  copyMap(home),
		lines:    make(chan Line, 256),
		bootDone: t.BootMs <= 0,
	}, nil
}

func copyMap(in map[int]float64) map[int]float64 {
	out := make(map[int]float64, len(in))
	for k, v := range in {
		out[k] = v
	}
	return out
}

func (d *SimDevice) Kind() string { return "sim" }

func (d *SimDevice) Connected() bool {
	d.mu.Lock()
	defer d.mu.Unlock()
	return !d.closed
}

// bootRemaining 必须在**持锁状态下**调用（它会写入 bootDone）。
func (d *SimDevice) bootRemaining() time.Duration {
	if d.bootDone {
		return 0
	}
	remain := time.Duration(d.tuning.BootMs)*time.Millisecond - time.Since(d.startAt)
	if remain <= 0 {
		d.bootDone = true
		return 0
	}
	return remain
}

func (d *SimDevice) UnavailableReason() string {
	d.mu.Lock()
	defer d.mu.Unlock()
	if d.closed {
		return "sim 设备已关闭"
	}
	return ""
}

func (d *SimDevice) Lines() <-chan Line { return d.lines }

func (d *SimDevice) OnStatus(fn StatusHandler) func() {
	d.mu.Lock()
	d.statusF = append(d.statusF, fn)
	idx := len(d.statusF) - 1
	d.mu.Unlock()
	return func() {
		d.mu.Lock()
		defer d.mu.Unlock()
		if idx < len(d.statusF) {
			d.statusF[idx] = nil
		}
	}
}

// WriteLine 模拟固件的指令受理。
func (d *SimDevice) WriteLine(line string) error {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return fmt.Errorf("sim 设备已关闭")
	}
	apiLatency := d.tuning.LatencyMs
	enforce := d.tuning.EnforceLimits
	bootRemain := d.bootRemaining()
	d.mu.Unlock()

	trimmed := strings.TrimSpace(line)
	upper := strings.ToUpper(trimmed)

	// 开机静默窗口：模拟 Uno bootloader，指令被吞掉且不回执
	if bootRemain > 0 {
		log.Printf("[sim] 开机静默窗口内丢弃指令（剩余 %v）: %q", bootRemain.Round(time.Millisecond), trimmed)
		return nil
	}

	switch {
	case strings.HasPrefix(upper, "JR"):
		joints, err := d.parseJR(trimmed)
		if err != nil {
			d.emit(fmt.Sprintf("ERR ARG %s", err.Error()))
			return nil
		}
		// ① 关节限位（固件内置同一张标定表 + 关节限位，见 docs/serial-v1.md §4）
		if enforce {
			if v := d.model.Validate(joints); v != nil {
				d.emit(v.Error())
				return nil
			}
		}
		// ② 换算舵机角 + 舵机硬限位
		servo := make(map[int]float64, len(d.order))
		for _, id := range d.order {
			for _, a := range d.model.ActuatorsForJoint(id) {
				s := robot.JointToServo(a, joints[id])
				if enforce && (s < a.Limits.Min-1e-6 || s > a.Limits.Max+1e-6) {
					d.emit(fmt.Sprintf("ERR SERVO S%d %.2f (limit %.2f..%.2f)",
						a.Channel, s, a.Limits.Min, a.Limits.Max))
					return nil
				}
				servo[a.Channel] = s
			}
		}
		// ③ 受理：进入"有命令在途"状态
		//
		// ⚠️ 这两行是"优先级"在 sim 侧的落点：
		//   externalMotion=false → 本次位置推进用 `STATE` 上报（命令引起）
		//   jrBusy=true          → 在途期间**外部**上报让位（只记脏，不发）
		// 与服务端口径一致：交互指令优先，状态上报不许插队。
		d.mu.Lock()
		d.externalMotion = false
		d.jrBusy = true
		d.mu.Unlock()
		// ④ 延迟送达：延迟期内不回执（印证"命令送达前不回推"的传输语义）
		deliver := func() {
			d.mu.Lock()
			if d.closed {
				d.mu.Unlock()
				return
			}
			for ch, v := range servo {
				d.targetS[ch] = v
			}
			d.mu.Unlock()
			d.emit(protocol.EncodeOKJR(servo))
			d.tick() // 送达瞬间先走一步（与前端 Mock 的 applyTarget 行为一致）
			d.mu.Lock()
			d.jrBusy = false
			d.mu.Unlock()
			d.flushReport() // 在途期间被让位掉的外部变化，在这里补报最新值
			d.ensureTicking()
		}
		if apiLatency <= 0 {
			deliver()
		} else {
			time.AfterFunc(time.Duration(apiLatency)*time.Millisecond, deliver)
		}
		return nil

	// ---- 真机舵机级入口（SET / S<n>= / JOY） ------------------------------
	//
	// 为什么 sim 要认这几条**固件级**命令：它们代表"命令之外的改动"。
	// 真机上对应的是硬件摇杆（`joystick_scan()`）、红外遥控、面板手拧 ——
	// 都不会经过本机的 JR 通路。把它们做成可下发的命令，是为了让"下位机被
	// 外部手段改动 → 上位机状态跟随"这条链路**在仿真里就能端到端验证**，
	// 而不必等到接上真机才发现界面不动。
	//
	// ⚠️ 语义边界：`SET` 在这里**不是** JR 的翻译结果（真机链路里 JR→SET 的翻译
	//    在 `serial.go`，sim 收到 JR 直接处理，不需要拆 SET）。sim 收到的 SET
	//    只可能来自调试 / 验收脚本，所以按"外部改动"对待是对的。
	case strings.HasPrefix(upper, "SET") || reServoShorthand.MatchString(trimmed):
		pairs, err := d.parseSetPairs(trimmed)
		if err != nil {
			d.emit(fmt.Sprintf("ERR ARG %s", err.Error()))
			return nil
		}
		applied := make(map[int]float64, len(pairs))
		for ch, ang := range pairs {
			applied[ch] = d.setServoTarget(ch, ang)
		}
		d.markExternal()
		d.emit(encodeServoKV("OK SET", applied))
		d.tick() // 送达瞬间先走一步，与 JR 路径一致
		d.ensureTicking()
		return nil

	case strings.HasPrefix(upper, "JOY"):
		fields := strings.Fields(trimmed)
		deltas := map[int]int{}
		switch len(fields) {
		case 3: // JOY <id> <raw>：单轴拨动
			ch, err1 := strconv.Atoi(fields[1])
			raw, err2 := strconv.Atoi(fields[2])
			if err1 != nil || err2 != nil || !d.hasServo(ch) {
				d.emit(fmt.Sprintf("ERR ARG JOY %s", strings.Join(fields[1:], " ")))
				return nil
			}
			if dl := joystickDelta(ch, raw); dl != 0 {
				deltas[ch] = dl
			}
		case 5: // JOY <r9> <r8> <r6> <r7>：整帧（位次与固件一致）
			ids := [4]int{9, 8, 6, 7}
			for i := 0; i < 4; i++ {
				raw, err := strconv.Atoi(fields[1+i])
				if err != nil {
					d.emit(fmt.Sprintf("ERR ARG JOY %s", fields[1+i]))
					return nil
				}
				if !d.hasServo(ids[i]) {
					continue
				}
				if dl := joystickDelta(ids[i], raw); dl != 0 {
					deltas[ids[i]] = dl
				}
			}
		default:
			d.emit("ERR SYNTAX JOY")
			return nil
		}
		for ch, dl := range deltas {
			d.nudgeServo(ch, dl)
		}
		if len(deltas) > 0 {
			d.markExternal()
		}
		d.emit(encodeServoKV("OK JOY", d.snapshot()))
		d.tick()
		d.ensureTicking()
		return nil

	case upper == "STATUS" || upper == "STATE?":
		// v0 固件的 STATUS：回舵机角（协议 §3）
		cur := d.snapshot()
		d.emit(fmt.Sprintf("STATUS S9=%d S7=%d S8=%d S6=%d",
			int(round(cur[9])), int(round(cur[7])), int(round(cur[8])), int(round(cur[6]))))
		return nil

	case upper == "RESET":
		// RESET 语义 = 全部舵机 90°（= HOME）。协议 §4 明确不许改成"关节全 0"：
		// 关节全 0 对肘（绝对角 108..142）是不可达位姿。
		target := make(map[int]float64, len(d.model.Actuators))
		for i := range d.model.Actuators {
			target[d.model.Actuators[i].Channel] = 90
		}
		d.mu.Lock()
		for ch, v := range target {
			d.targetS[ch] = v
		}
		// RESET 是**命令**路径（上位机要求回中位）⇒ 后续推进走 STATE 上报
		d.externalMotion = false
		d.mu.Unlock()
		d.emit("OK RESET")
		d.emit(protocol.EncodeOKJR(target))
		d.ensureTicking()
		return nil

	case upper == "PING":
		d.emit("OK PING")
		return nil

	default:
		d.emit(fmt.Sprintf("ERR UNKNOWN %s", trimmed))
		return nil
	}
}

// parseJR 解析 `JR <j1> <j2> <j3> <grip>`（位次 = model.JointOrder()）。
func (d *SimDevice) parseJR(line string) (map[string]float64, error) {
	m := reJR.FindStringSubmatch(line)
	if m == nil {
		return nil, fmt.Errorf("不是 JR 指令")
	}
	fields := strings.Fields(m[1])
	if len(fields) != len(d.order) {
		return nil, fmt.Errorf("JR 需要 %d 个关节角，收到 %d 个", len(d.order), len(fields))
	}
	joints := make(map[string]float64, len(fields))
	for i, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil {
			return nil, fmt.Errorf("第 %d 个关节角 %q 不是数字", i+1, f)
		}
		joints[d.order[i]] = v
	}
	return joints, nil
}

func (d *SimDevice) snapshot() map[int]float64 {
	d.mu.Lock()
	defer d.mu.Unlock()
	return copyMap(d.actualS)
}

func (d *SimDevice) ensureTicking() {
	d.mu.Lock()
	if d.ticking || d.closed {
		d.mu.Unlock()
		return
	}
	d.ticking = true
	tickMs := d.tuning.TickMs
	d.mu.Unlock()

	if tickMs <= 0 {
		tickMs = 20
	}
	go func() {
		t := time.NewTicker(time.Duration(tickMs) * time.Millisecond)
		defer t.Stop()
		for range t.C {
			d.mu.Lock()
			closed := d.closed
			d.mu.Unlock()
			if closed {
				d.mu.Lock()
				d.ticking = false
				d.mu.Unlock()
				return
			}
			if !d.tick() {
				d.mu.Lock()
				d.ticking = false
				d.mu.Unlock()
				return
			}
		}
	}()
}

// tick 推进一个仿真步，返回是否仍在运动。
func (d *SimDevice) tick() bool {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return false
	}
	tickMs := d.tuning.TickMs
	if tickMs <= 0 {
		tickMs = 20
	}
	maxStep := d.tuning.MaxServoSpeed * (float64(tickMs) / 1000)
	instant := maxStep <= 0

	changed := false
	for ch, goal := range d.targetS {
		cur := d.actualS[ch]
		delta := goal - cur
		if delta == 0 {
			continue
		}
		if instant || abs(delta) <= maxStep {
			d.actualS[ch] = goal
		} else {
			d.actualS[ch] = cur + sign(delta)*maxStep
		}
		changed = true
	}
	moving := false
	for ch, goal := range d.targetS {
		if abs(d.actualS[ch]-goal) > 1e-6 {
			moving = true
			break
		}
	}
	actual := copyMap(d.actualS)
	externalMotion := d.externalMotion
	d.mu.Unlock()

	if changed {
		if externalMotion {
			// 设备侧外部变化 → 舵机角异步上报（OriginDevice）。
			// ⚠️ 不用 STATE：那是"命令引起"的通道，混用会让界面把命令侧
			//    拖向实际位置（滑杆被回拉）。
			d.reportServos(actual)
		} else {
			// ⚠️ 用舵机**实际角反算**关节角上报 —— 走真实路径，不做等值回显。
			//    若 offset/scale/reverse 写错，这里立刻暴露为 Actual ≠ Command。
			joints := protocol.ServoAnglesToJoints(d.model, actual)
			d.emit(protocol.EncodeState(d.order, joints))
		}
	}
	if !moving {
		// 已静止：若还有被"命令在途"让位掉的外部变化，在此补报最新值。
		// 值不缓存（取当时的 actual）⇒ 天然 latest-wins，不会积压中间帧。
		d.mu.Lock()
		dirty := d.reportDirty
		d.mu.Unlock()
		if dirty {
			d.reportServos(actual)
		}
	}
	return moving
}

// reportServos 发一行 `# SERVO …`。
//
// ★ 这里是 sim 侧的**低优先级**落点：命令在途（`jrBusy`）时**只记脏、不发**，
// 让位给交互指令；等命令了结后由 `flushReport()` 或下一次 tick 补报。
// 与固件侧"TX 环低水位才发、发不出就丢弃（保留脏标志）"是同一条规则的两端。
func (d *SimDevice) reportServos(actual map[int]float64) {
	d.mu.Lock()
	if d.jrBusy {
		d.reportDirty = true
		d.mu.Unlock()
		return
	}
	d.reportDirty = false
	d.mu.Unlock()
	d.emit(protocol.EncodeServoReport(actual))
}

// flushReport 在命令了结后补报被让位掉的外部变化（若有）。
func (d *SimDevice) flushReport() {
	d.mu.Lock()
	dirty := d.reportDirty
	actual := copyMap(d.actualS)
	d.mu.Unlock()
	if dirty {
		d.reportServos(actual)
	}
}

// markExternal 把后续的位置推进标记为"设备侧外部变化"（上报走 `# SERVO`）。
func (d *SimDevice) markExternal() {
	d.mu.Lock()
	d.externalMotion = true
	d.mu.Unlock()
}

// ---------------------------------------------------------------------------
// 舵机级操作（SET / JOY 的实现；单位一律**舵机度**，与固件同一坐标系）
// ---------------------------------------------------------------------------

// actuatorByChannel 按通道号找执行器（`-1` 表示不存在）。
//
// ⚠️ 通道号（6/7/8/9）不等于关节顺序里的位次 —— 别用 `d.order` 去索引它。
func (d *SimDevice) actuatorByChannel(ch int) int {
	for i := range d.model.Actuators {
		if d.model.Actuators[i].Channel == ch {
			return i
		}
	}
	return -1
}

func (d *SimDevice) hasServo(ch int) bool { return d.actuatorByChannel(ch) >= 0 }

// setServoTarget 直接设定某舵机的目标角（`SET` / `S<n>=` 语义）。
//
// 与固件 `arm_set_angle()` 一致：**钳位到该舵机的硬限位并返回生效值**，
// 越界不报错（真机就是在舵机允许的行程内截断）。
func (d *SimDevice) setServoTarget(ch int, angle float64) float64 {
	idx := d.actuatorByChannel(ch)
	if idx < 0 {
		return angle
	}
	a := &d.model.Actuators[idx]
	if angle < a.Limits.Min {
		angle = a.Limits.Min
	}
	if angle > a.Limits.Max {
		angle = a.Limits.Max
	}
	d.mu.Lock()
	d.targetS[ch] = angle
	d.mu.Unlock()
	return angle
}

// nudgeServo 在当前**目标角**上增量（`JOY` 语义）。
//
// ⚠️ 基于 target 而不是 actual 增量，且不改写 actual —— 与固件
// `arm_nudge()` 同一取向：斜坡中途的拨动只微调终点，不中断正在进行的运动。
func (d *SimDevice) nudgeServo(ch int, delta int) {
	idx := d.actuatorByChannel(ch)
	if idx < 0 || delta == 0 {
		return
	}
	a := &d.model.Actuators[idx]
	d.mu.Lock()
	v := d.targetS[ch] + float64(delta)
	if v < a.Limits.Min {
		v = a.Limits.Min
	}
	if v > a.Limits.Max {
		v = a.Limits.Max
	}
	d.targetS[ch] = v
	d.mu.Unlock()
}

// parseSetPairs 解析 `SET <id> <ang> [<id> <ang>…]` 与简写 `S<id>=<ang>`。
func (d *SimDevice) parseSetPairs(line string) (map[int]float64, error) {
	if m := reServoShorthand.FindStringSubmatch(line); m != nil {
		ch, _ := strconv.Atoi(m[1])
		ang, _ := strconv.ParseFloat(m[2], 64)
		if !d.hasServo(ch) {
			return nil, fmt.Errorf("未知舵机 S%d", ch)
		}
		return map[int]float64{ch: ang}, nil
	}
	fields := strings.Fields(line)
	if len(fields) < 3 || len(fields)%2 != 1 {
		return nil, fmt.Errorf("SET 需要成对的 <id> <angle>")
	}
	if (len(fields)-1)/2 > 3 {
		// 与固件 MAX_PAIRS=3 对齐 —— sim 不该比真机宽容
		return nil, fmt.Errorf("SET 最多 3 组")
	}
	out := make(map[int]float64, (len(fields)-1)/2)
	for i := 1; i+1 < len(fields); i += 2 {
		ch, err1 := strconv.Atoi(fields[i])
		ang, err2 := strconv.ParseFloat(fields[i+1], 64)
		if err1 != nil || !d.hasServo(ch) {
			return nil, fmt.Errorf("未知舵机 %q", fields[i])
		}
		if err2 != nil {
			return nil, fmt.Errorf("角度 %q 不是数字", fields[i+1])
		}
		out[ch] = ang
	}
	return out, nil
}

// joystickDelta 与固件 `core/joystick.c` 的 `joystick_delta()` **同一条公式**。
//
// 两处必须一致，否则 sim 上"拨一下走多远"与真机会对不上，
// 而验收数字看起来仍然自洽（这是最容易蒙混过去的一类偏差）。
//
//	raw < 200 → 一个方向；raw > 800 → 另一方向；中间死区不动
//	id == 8（左舵）方向取反 —— 与原始 Arduino 草图一致
//	步长与推杆深度成比例：min 2、max 10（度）
func joystickDelta(id int, raw int) int {
	if raw < 0 {
		raw = 0
	}
	if raw > 1023 {
		raw = 1023
	}
	pastHi := raw > 800
	pastLo := raw < 200
	if !pastHi && !pastLo {
		return 0
	}
	beyond := 200 - raw
	if pastHi {
		beyond = raw - 800
	}
	step := 2 + beyond/30
	if step > 10 {
		step = 10
	}
	positive := pastLo
	if id == 8 {
		positive = pastHi
	}
	if positive {
		return step
	}
	return -step
}

// encodeServoKV 编 `OK SET S7=90 S6=88` 这类**应答**行（回执格式对齐固件）。
//
// ⚠️ 只用于"我收到了"这种应答，**不要**拿它当状态上报 —— 应答里的角度是
// 目标/钳位值，不是实际位置；状态上报走 `protocol.EncodeServoReport`。
func encodeServoKV(prefix string, m map[int]float64) string {
	chans := make([]int, 0, len(m))
	for ch := range m {
		chans = append(chans, ch)
	}
	if len(chans) == 0 {
		return prefix
	}
	sort.Sort(sort.Reverse(sort.IntSlice(chans)))
	parts := []string{prefix}
	for _, ch := range chans {
		parts = append(parts, fmt.Sprintf("S%d=%.0f", ch, m[ch]))
	}
	return strings.Join(parts, " ")
}

func (d *SimDevice) emit(text string) {
	d.mu.Lock()
	closed := d.closed
	d.mu.Unlock()
	if closed {
		return
	}
	line := Line{Text: text, At: time.Now()}
	select {
	case d.lines <- line:
	default:
		// 接收端落后时丢弃最旧的行，保证设备侧永不阻塞
		select {
		case <-d.lines:
		default:
		}
		select {
		case d.lines <- line:
		default:
		}
	}
}

func (d *SimDevice) Close() error {
	d.mu.Lock()
	if d.closed {
		d.mu.Unlock()
		return nil
	}
	d.closed = true
	fns := append([]StatusHandler(nil), d.statusF...)
	d.mu.Unlock()
	for _, fn := range fns {
		if fn != nil {
			fn(false, "sim 设备已关闭")
		}
	}
	return nil
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}

func sign(v float64) float64 {
	if v < 0 {
		return -1
	}
	return 1
}

func round(v float64) float64 {
	return float64(int(v + 0.5))
}
