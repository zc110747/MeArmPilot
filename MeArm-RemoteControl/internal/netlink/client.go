package netlink

// client.go —— JSON Lines TCP 客户端：连接生命周期 / 读写 / 串行化 / 状态回调。
//
// 传输格式完全遵循 MeArm-3D 的既有协议（docs/tcp-control-v1.md）：
// **一行 JSON + `\n`**，一行一条应答，严格一问一答。
//
// # 三条硬要求（对应方案 §9.3 的并发与连接安全）
//
//  1. **同一连接只有一个 writer**：所有写只发生在 `pump()` 这一个 goroutine 里。
//     上层（WebSocket 的多个客户端 goroutine）只往队列里放"意图"，从不碰 socket
//     ⇒ JSON 不可能交叉。
//  2. **绝不无限堆积**：摇杆按舵机维度做 **latest-wins**（同一轴只保留最新目标角），
//     离散命令走有界 FIFO（满了丢最旧）。这直接对齐 `internal/serial` 的既有设计。
//  3. **连接关闭后不再写**：`Close()` 先 `close(done)` 再掐 socket，最后 `wg.Wait()`；
//     `pump()` 在每轮循环开头检查 `done`。goroutine 一个都不留。
//
// # 与串口模式的关系
//
// 本包与 `internal/serial` 是**对等的两种 Transport**：上层 `web` 只认一个小接口
// （见 internal/web 的 Sink），换传输不换控制语义。串口模式一行代码都没动。

import (
	"bufio"
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"net"
	"strings"
	"sync"
	"time"
)

// ErrNoBaseline 还没拿到"当前舵机角"基准，无法做增量控制。
//
// 正常路径下这个窗口只有连接建立后的一两毫秒（`AutoSync` 立刻发 `state`）。
var ErrNoBaseline = errors.New("尚未取得状态基准")

// maxJoySlots 摇杆目标数组长度上限。
//
// 只是"够用即可"的分配上限：MeArm-3D 的 mearm-v1 是 4 舵机（servo 1..4）。
// 超出的编号在 applyDefaults 里会被拒绝，不会越界。
const maxJoySlots = 8

// Config TCP 客户端参数。
type Config struct {
	Host string
	Port int

	// Axis 摇杆 → 舵机编号的接线表（见 axis.go）。
	Axis AxisMap

	// AngleMin / AngleMax 值域保护（**不是软限位**，见包注释）。
	AngleMin float64
	AngleMax float64

	// ConnectTimeoutMs 单次建连超时。
	ConnectTimeoutMs int
	// RequestTimeoutMs 单次"发一行 + 读一行"的超时。
	RequestTimeoutMs int
	// TickMs 摇杆积分的最小间隔。比它更密的输入只更新当前输入值，
	// 不重复积分 —— 于是累加速率只由**真实经过的时间**决定，与调用频率无关。
	TickMs int
	// MaxTickMs 单次积分的 dt 上限，防止长时间无输入后一次积爆。
	MaxTickMs int

	// ReconnectMinMs / ReconnectMaxMs 指数退避的下限/上限（防快速重连风暴）。
	ReconnectMinMs int
	ReconnectMaxMs int

	// QueueSize 离散命令 FIFO 容量（满则丢最旧）。
	QueueSize int

	// AutoSync 连接建立后自动发一条 `state` 取基准（推荐）。
	AutoSync bool
	// FallbackAngle 服务端不提供 `state` 时使用的兜底基准角（各舵机同值）。
	//
	// 该值来自 MeArm-3D 的模型真值：mearm-v1 的 homePose 经 actuators 标定
	// 换算后四个舵机都是 90°（robot.yaml 注释亦明确此自洽关系），
	// 且 sim device 的初始位就是 HOME、固件 RESET 也是全部 90°。
	FallbackAngle float64
}

// queued 一条待发的离散命令。
//
// `servo` > 0 表示"这条命令是针对该舵机的**绝对角**指令" —— 被服务端拒绝时
// 需要把该轴目标回滚（见 handleResponse）。摇杆产生的命令走另一条路径
// （joyDirty），不需要排队。
type queued struct {
	line  string
	servo int
}

// State 服务端回读的状态（成功应答里的 `state` 字段）。
type State struct {
	Joints map[string]float64 `json:"joints,omitempty"`
	Servo  []float64          `json:"servo,omitempty"`
	TCP    []float64          `json:"tcp,omitempty"`
	Device string             `json:"device,omitempty"`
}

// response 一条应答（JSON Lines 的一行）。
type response struct {
	OK    bool   `json:"ok"`
	Cmd   string `json:"cmd,omitempty"`
	Error string `json:"error,omitempty"`
	State *State `json:"state,omitempty"`
}

// StatusHandler 连接状态翻转。
type StatusHandler func(connected bool, errMsg string)

// StateHandler 收到服务端状态（每次成功应答都会触发）。
type StateHandler func(st *State)

// ErrorHandler 服务端拒绝或链路错误（不影响连接存续的"软错误"走这里）。
type ErrorHandler func(msg string)

// Client 一个到 MeArm-3D TCP 控制接口的长连接客户端。
type Client struct {
	cfg Config

	done      chan struct{}
	closeOnce sync.Once
	startOnce sync.Once
	wg        sync.WaitGroup
	wakeCh    chan struct{}

	nowFn func() time.Time

	mu        sync.Mutex
	conn      net.Conn
	connected bool
	lastErr   string

	// frame 最近一次摇杆输入（归一化）。
	frame Frame
	// lastInt 上次积分的时刻（dt 由此而来）。
	lastInt time.Time
	// haveBase 是否已取得基准。
	haveBase bool
	// target 当前目标舵机角（下标 = servo-1）。
	target [maxJoySlots]float64
	// confirm 最近一次服务端确认的舵机角（错误回滚用）。
	confirm [maxJoySlots]float64
	// joyDirty 该轴有未下发的摇杆增量。
	joyDirty [maxJoySlots]bool

	// queue 离散命令 FIFO（XYZ / 爪 / 单舵机 / state 查询）。
	queue []queued

	lastState *State

	onStatus StatusHandler
	onState  StateHandler
	onErr    ErrorHandler
}

// New 构造客户端（不连接；调用 Start 才开始）。
func New(cfg Config) *Client {
	applyDefaults(&cfg)
	c := &Client{
		cfg:    cfg,
		done:   make(chan struct{}),
		wakeCh: make(chan struct{}, 1),
		nowFn:  time.Now,
	}
	return c
}

func applyDefaults(c *Config) {
	if c.Host == "" {
		c.Host = "127.0.0.1"
	}
	if c.Port <= 0 {
		c.Port = 9100
	}
	if c.AngleMin == 0 && c.AngleMax == 0 {
		// 舵机空间的物理值域。**不是**硬件行程 —— 那是 robot.yaml 的真值，
		// 由 MeArm-3D 的 servo 命令负责校验。这里只是数值保护。
		c.AngleMin, c.AngleMax = 0, 180
	}
	if c.ConnectTimeoutMs <= 0 {
		c.ConnectTimeoutMs = 1500
	}
	if c.RequestTimeoutMs <= 0 {
		c.RequestTimeoutMs = 1500
	}
	if c.TickMs <= 0 {
		c.TickMs = 25
	}
	if c.MaxTickMs <= 0 {
		c.MaxTickMs = 200
	}
	if c.ReconnectMinMs <= 0 {
		c.ReconnectMinMs = 500
	}
	if c.ReconnectMaxMs <= 0 {
		c.ReconnectMaxMs = 5000
	}
	if c.ReconnectMaxMs < c.ReconnectMinMs {
		c.ReconnectMaxMs = c.ReconnectMinMs
	}
	if c.QueueSize <= 0 {
		c.QueueSize = 16
	}
	if c.FallbackAngle <= 0 {
		c.FallbackAngle = 90
	}
	if c.Axis.MinSpeedDegPerS <= 0 {
		c.Axis.MinSpeedDegPerS = 12
	}
	if c.Axis.MaxSpeedDegPerS <= 0 {
		c.Axis.MaxSpeedDegPerS = 90
	}
	// 接线表里有非法编号就退回默认（不静默带病运行）。
	if !c.Axis.valid() {
		log.Printf("[netlink] ⚠️ 摇杆接线表非法，已退回默认（1/3/4/2）")
		def := DefaultAxisMap()
		def.DeadFrac = c.Axis.DeadFrac
		def.MinSpeedDegPerS = c.Axis.MinSpeedDegPerS
		def.MaxSpeedDegPerS = c.Axis.MaxSpeedDegPerS
		c.Axis = def
	}
}

// valid 接线表编号必须落在 1..maxJoySlots。
func (m AxisMap) valid() bool {
	for _, v := range [4]int{m.LXServo, m.LYServo, m.RXServo, m.RYServo} {
		if v < 1 || v > maxJoySlots {
			return false
		}
	}
	return true
}

// Addr 目标地址。
func (c *Client) Addr() string {
	return net.JoinHostPort(c.cfg.Host, fmt.Sprintf("%d", c.cfg.Port))
}

// OnStatus 订阅连接状态。
func (c *Client) OnStatus(h StatusHandler) {
	c.mu.Lock()
	c.onStatus = h
	c.mu.Unlock()
}

// OnState 订阅服务端状态。
func (c *Client) OnState(h StateHandler) {
	c.mu.Lock()
	c.onState = h
	c.mu.Unlock()
}

// OnError 订阅错误。
func (c *Client) OnError(h ErrorHandler) {
	c.mu.Lock()
	c.onErr = h
	c.mu.Unlock()
}

// Start 启动连接管理（幂等）。
func (c *Client) Start() {
	c.startOnce.Do(func() {
		c.wg.Add(1)
		go c.manage()
	})
}

// Close 停止客户端：先掐连接唤醒阻塞的读写，再等 goroutine 全部退出。
func (c *Client) Close() {
	c.closeOnce.Do(func() {
		close(c.done)
		c.mu.Lock()
		conn := c.conn
		c.mu.Unlock()
		if conn != nil {
			// ⚠️ 顺序不能反：只 close(done) 的话，pump 还阻塞在 Read 上，
			//    wg.Wait() 会一直挂着（表现为"进程关不掉"）。这与 MeArm-3D
			//    tcpserver.Close 的教训同源。
			_ = conn.Close()
		}
		c.wg.Wait()
		// 显式落到"未连接"：manage 在收到 done 时是直接 return 的，
		// 不会自己改状态 —— 不在这里落一次，界面会停在"已连接"骗人。
		c.setStatus(false, "已关闭")
	})
}

// Status 返回 (是否已连接, 最近错误)。
func (c *Client) Status() (bool, string) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.connected, c.lastErr
}

// LastState 最近一次服务端状态（无则 nil）。**是拷贝**，调用方可安全读。
func (c *Client) LastState() *State {
	c.mu.Lock()
	defer c.mu.Unlock()
	return copyState(c.lastState)
}

// ---------------------------------------------------------------------------
// 连接管理
// ---------------------------------------------------------------------------

func (c *Client) manage() {
	defer c.wg.Done()
	backoff := time.Duration(c.cfg.ReconnectMinMs) * time.Millisecond
	maxBackoff := time.Duration(c.cfg.ReconnectMaxMs) * time.Millisecond

	for {
		if c.stopped() {
			return
		}
		conn, err := net.DialTimeout("tcp", c.Addr(),
			time.Duration(c.cfg.ConnectTimeoutMs)*time.Millisecond)
		if err != nil {
			c.setStatus(false, err.Error())
			log.Printf("[netlink] 连接 %s 失败: %v（%v 后重试）", c.Addr(), err, backoff)
			if !c.sleep(backoff) {
				return
			}
			backoff = nextBackoff(backoff, maxBackoff)
			continue
		}

		backoff = time.Duration(c.cfg.ReconnectMinMs) * time.Millisecond
		c.attach(conn)
		c.setStatus(true, "")
		log.Printf("[netlink] 已连接 %s（JSON Lines）", c.Addr())

		err = c.pump(conn)

		_ = conn.Close()
		c.detach()
		if c.stopped() {
			return
		}
		if err != nil {
			log.Printf("[netlink] 链路中断: %v", err)
		}
		c.setStatus(false, "连接已断开，重连中…")
		if !c.sleep(backoff) {
			return
		}
		backoff = nextBackoff(backoff, maxBackoff)
	}
}

func nextBackoff(cur, max time.Duration) time.Duration {
	next := cur * 2
	if next > max {
		next = max
	}
	return next
}

func (c *Client) stopped() bool {
	select {
	case <-c.done:
		return true
	default:
		return false
	}
}

func (c *Client) sleep(d time.Duration) bool {
	select {
	case <-c.done:
		return false
	case <-time.After(d):
		return true
	}
}

func (c *Client) attach(conn net.Conn) {
	c.mu.Lock()
	c.conn = conn
	c.mu.Unlock()
}

// detach 断开后清掉连接与基准 —— 重连必须重新取基准，否则会拿着过期角度发绝对角。
func (c *Client) detach() {
	c.mu.Lock()
	c.conn = nil
	c.haveBase = false
	for i := range c.joyDirty {
		c.joyDirty[i] = false
	}
	c.mu.Unlock()
}

// pump 是**唯一**的读写循环（= 唯一的 writer）。
func (c *Client) pump(conn net.Conn) error {
	r := bufio.NewReaderSize(conn, 8192)

	c.syncBaseline(r, conn)
	c.wake()

	for {
		if c.stopped() {
			return nil
		}
		line, servo, ok := c.nextLine()
		if !ok {
			// 空闲：等唤醒，或周期性醒来看一眼是否已停止。
			select {
			case <-c.done:
				return nil
			case <-c.wakeCh:
			case <-time.After(200 * time.Millisecond):
			}
			continue
		}
		resp, err := c.roundTrip(r, conn, line)
		if err != nil {
			return err
		}
		c.handleResponse(resp, servo)
	}
}

// roundTrip 写一行 + 读一行（严格一问一答）。
func (c *Client) roundTrip(r *bufio.Reader, conn net.Conn, line string) (*response, error) {
	wTimeout := time.Duration(c.cfg.RequestTimeoutMs) * time.Millisecond
	rTimeout := time.Duration(c.cfg.RequestTimeoutMs) * time.Millisecond

	if err := conn.SetWriteDeadline(time.Now().Add(wTimeout)); err != nil {
		return nil, err
	}
	if _, err := conn.Write([]byte(line + "\n")); err != nil {
		return nil, fmt.Errorf("写入失败: %w", err)
	}
	if err := conn.SetReadDeadline(time.Now().Add(rTimeout)); err != nil {
		return nil, err
	}
	raw, err := r.ReadBytes('\n')
	if err != nil {
		return nil, fmt.Errorf("读取应答失败: %w", err)
	}
	var resp response
	if err := json.Unmarshal(bytes.TrimSpace(raw), &resp); err != nil {
		return nil, fmt.Errorf("应答不是合法 JSON（%q）: %w", strings.TrimSpace(string(raw)), err)
	}
	return &resp, nil
}

// ---------------------------------------------------------------------------
// 基准（当前舵机角）
// ---------------------------------------------------------------------------

// syncBaseline 建立"当前舵机角"基准。
//
// 优先向服务端要真值（`state`，只读、无副作用）；服务端不认识这条命令时
// **明确降级**并用配置兜底 —— 降级要留痕，不能悄悄猜一个角度当事实。
func (c *Client) syncBaseline(r *bufio.Reader, conn net.Conn) {
	if !c.cfg.AutoSync {
		c.applyFallback(c.cfg.FallbackAngle, "未启用自动同步（auto_sync=false）")
		return
	}
	resp, err := c.roundTrip(r, conn, `{"cmd":"state"}`)
	if err != nil {
		// 连基准都问不到 ⇒ 这一轮链路本身有问题，交给 pump 的调用方处理。
		c.fireErr("读取服务端状态失败: " + err.Error())
		return
	}
	if resp.OK && resp.State != nil {
		c.applyState(resp.State, true)
		log.Printf("[netlink] 状态基准已取得：servo=%v device=%s",
			formatServo(resp.State.Servo), resp.State.Device)
		return
	}
	reason := resp.Error
	if reason == "" {
		reason = "应答不含 state"
	}
	c.applyFallback(c.cfg.FallbackAngle,
		fmt.Sprintf("服务端未提供 state（%s）", reason))
}

// applyFallback 用配置值建立基准，并把降级原因**明确上报**。
func (c *Client) applyFallback(angle float64, why string) {
	c.mu.Lock()
	for i := range c.target {
		c.target[i] = angle
		c.confirm[i] = angle
	}
	c.haveBase = true
	c.lastInt = c.nowFn()
	c.mu.Unlock()
	log.Printf("[netlink] ⚠️ 降级为配置初始角 %.1f° 作为基准（%s）—— 首帧可能有跳变", angle, why)
	c.fireErr(fmt.Sprintf("未能取得服务端状态，已用 %.0f° 作基准（%s）", angle, why))
}

// applyState 用服务端状态建立/校准基准。
//
// full=true 表示这是一次**权威**同步（连接建立时 / 错误回滚后）：
// 未被摇杆改过的轴一律对齐服务端真值。
func (c *Client) applyState(st *State, full bool) {
	if st == nil {
		return
	}
	c.mu.Lock()
	if n := len(st.Servo); n > 0 {
		for i := 0; i < n && i < maxJoySlots; i++ {
			c.confirm[i] = st.Servo[i]
			if full || !c.joyDirty[i] {
				c.target[i] = st.Servo[i]
			}
		}
	}
	c.lastState = copyState(st)
	if !c.haveBase {
		// 服务端回了一个"空 servo 数组"的状态：仍有基准意义（关节帧可用），
		// 但角度基准得靠兜底值，否则增量控制会从 0° 起步。
		if len(st.Servo) == 0 {
			for i := range c.target {
				c.target[i] = c.cfg.FallbackAngle
				c.confirm[i] = c.cfg.FallbackAngle
			}
		}
		c.haveBase = true
		c.lastInt = c.nowFn()
	}
	c.mu.Unlock()
	c.fireState(st)
}

// ---------------------------------------------------------------------------
// 命令入口（上层只调这些）
// ---------------------------------------------------------------------------

// Joy 提交一帧摇杆输入。按真实经过时间把输入积分成目标角增量。
//
// 这是**时间驱动**的：调用频率高低不影响结果，只影响"多久下发一次"。
// 太密的调用只更新当前输入值（下次一起积），于是不会因为前端 16ms/30ms 两个
// 定时器叠加而把速度翻倍。
func (c *Client) Joy(f Frame) error {
	now := c.nowFn()

	c.mu.Lock()
	c.frame = f
	if !c.haveBase {
		c.mu.Unlock()
		// 没有基准时动不了是合理行为（连接刚建立的一瞬间）。静默丢弃，
		// 不往上刷错误 —— 否则拖拽时会瞬间灌满界面错误行。
		return ErrNoBaseline
	}
	dt := now.Sub(c.lastInt)
	if dt < time.Duration(c.cfg.TickMs)*time.Millisecond {
		c.mu.Unlock()
		return nil
	}
	maxDt := time.Duration(c.cfg.MaxTickMs) * time.Millisecond
	if dt > maxDt {
		dt = maxDt
	}
	c.lastInt = now

	deltas := c.cfg.Axis.Deltas(f, dt.Seconds())
	for servo, d := range deltas {
		i := servo - 1
		if i < 0 || i >= maxJoySlots {
			continue
		}
		c.target[i] = clampAngle(c.target[i]+d, c.cfg.AngleMin, c.cfg.AngleMax)
		c.joyDirty[i] = true
	}
	c.mu.Unlock()

	c.wake()
	return nil
}

// Xyz 请求一次 XYZ 相对位移（`{"cmd":"move",...}`）。
func (c *Client) Xyz(axis, direction string, step float64) error {
	axis = strings.ToLower(strings.TrimSpace(axis))
	if axis != "x" && axis != "y" && axis != "z" {
		return fmt.Errorf("轴只能是 x / y / z，收到 %q", axis)
	}
	if direction != "+" && direction != "-" {
		return fmt.Errorf("方向只能是 + / -，收到 %q", direction)
	}
	if !(step > 0) {
		return fmt.Errorf("步长必须 > 0，收到 %v", step)
	}
	return c.enqueue(fmt.Sprintf(`{"cmd":"move","axis":%q,"direction":%q,"step":%s}`,
		axis, direction, trimFloat(step)), 0)
}

// Gripper 请求夹爪开合（`{"cmd":"gripper","action":...}`）。
func (c *Client) Gripper(action string) error {
	action = strings.ToLower(strings.TrimSpace(action))
	if action != "open" && action != "close" {
		return fmt.Errorf("爪动作只能是 open / close，收到 %q", action)
	}
	return c.enqueue(fmt.Sprintf(`{"cmd":"gripper","action":%q}`, action), 0)
}

// Servo 直接给某个舵机一个绝对角（`{"cmd":"servo",...}`）。
//
// `servo` 是 1..N 的 **TCP 编号**（MeArm-3D 的 JointOrder 顺序），不是 arm-device
// 的舵机 id。行程校验在服务端（robot.yaml 的真值），本函数只做数值保护。
func (c *Client) Servo(n int, angle float64) error {
	if n < 1 || n > maxJoySlots {
		return fmt.Errorf("舵机编号 %d 超出 1..%d", n, maxJoySlots)
	}
	if math.IsNaN(angle) || math.IsInf(angle, 0) {
		return fmt.Errorf("角度非法: %v", angle)
	}
	angle = clampAngle(angle, c.cfg.AngleMin, c.cfg.AngleMax)
	if err := c.enqueue(fmt.Sprintf(`{"cmd":"servo","servo":%d,"angle":%s}`, n, trimFloat(angle)), n); err != nil {
		return err
	}
	// 这个轴的控制权刚被"绝对角"接管：本地**目标**直接对齐，避免随后的摇杆增量
	// 从一个过期的基准继续加。
	//
	// ⚠️ 只动 `target`，**不动 `confirm`** —— confirm 的语义是"服务端确认过的值"，
	//    只能在收到应答时更新。这里顺手也写 confirm 会让"被拒绝后回滚"回滚到
	//    一个服务端从没接受过的角度（实测踩过：回滚值变成了刚被拒的那个 180°）。
	c.mu.Lock()
	c.target[n-1] = angle
	c.joyDirty[n-1] = false
	c.mu.Unlock()
	return nil
}

// Sync 主动拉一次服务端状态（网页"刷新状态"用）。
func (c *Client) Sync() error { return c.enqueue(`{"cmd":"state"}`, 0) }

// enqueue 把一条离散命令放进 FIFO（有界，满则丢最旧 —— 绝不无限堆积）。
func (c *Client) enqueue(line string, servo int) error {
	if c.stopped() {
		return errors.New("网络链路已关闭")
	}
	c.mu.Lock()
	if len(c.queue) >= c.cfg.QueueSize {
		c.queue = c.queue[1:]
	}
	c.queue = append(c.queue, queued{line: line, servo: servo})
	c.mu.Unlock()
	c.wake()
	return nil
}

// wake 非阻塞唤醒写泵。
func (c *Client) wake() {
	select {
	case c.wakeCh <- struct{}{}:
	default:
	}
}

// nextLine 取下一条要发的命令。
//
// 顺序：**离散命令优先**（用户点按的意图比连续摇杆更该被立刻兑现），
// 其次才是一个"有未下发增量"的摇杆轴（按下标升序，确定性）。
func (c *Client) nextLine() (line string, servo int, ok bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if len(c.queue) > 0 {
		q := c.queue[0]
		c.queue = c.queue[1:]
		return q.line, q.servo, true
	}
	for i := 0; i < maxJoySlots; i++ {
		if !c.joyDirty[i] {
			continue
		}
		c.joyDirty[i] = false
		return fmt.Sprintf(`{"cmd":"servo","servo":%d,"angle":%s}`, i+1, trimFloat(c.target[i])), i + 1, true
	}
	return "", 0, false
}

// handleResponse 处理一条应答。
//
// `sentServo` > 0 表示刚刚发的是一条摇杆产生的 servo 命令 —— 失败时要回滚该轴，
// 否则目标角会带着"服务端没接受的那一截"继续累加，越走越远。
func (c *Client) handleResponse(resp *response, sentServo int) {
	if resp.State != nil {
		// 离散命令（绝对语义）会重置位置语义 ⇒ 顺便把未被摇杆改动的轴对齐真值。
		c.applyState(resp.State, sentServo == 0)
	}
	if resp.OK {
		return
	}
	msg := resp.Error
	if msg == "" {
		msg = "服务端返回 ok=false"
	}
	c.fireErr(msg)

	if sentServo > 0 && isRangeOrLimitError(msg) {
		c.rollback(sentServo)
	}
}

// isRangeOrLimitError 服务端因超出可行范围而拒绝。
func isRangeOrLimitError(msg string) bool {
	m := strings.ToLower(msg)
	return strings.Contains(m, "out of range") ||
		strings.Contains(m, "joint_limit") ||
		strings.Contains(m, "workspace")
}

// rollback 把该轴目标回滚到最近一次服务端确认值，并给出可读日志。
func (c *Client) rollback(servo int) {
	i := servo - 1
	if i < 0 || i >= maxJoySlots {
		return
	}
	c.mu.Lock()
	c.target[i] = c.confirm[i]
	c.joyDirty[i] = false
	v := c.confirm[i]
	c.mu.Unlock()
	log.Printf("[netlink] 舵机 %d 被服务端拒绝，目标回滚到 %.2f°", servo, v)
}

// ---------------------------------------------------------------------------
// 回调与工具
// ---------------------------------------------------------------------------

func (c *Client) setStatus(connected bool, errMsg string) {
	c.mu.Lock()
	changed := c.connected != connected || c.lastErr != errMsg
	c.connected = connected
	c.lastErr = errMsg
	h := c.onStatus
	c.mu.Unlock()
	if changed && h != nil {
		h(connected, errMsg)
	}
}

func (c *Client) fireState(st *State) {
	c.mu.Lock()
	h := c.onState
	c.mu.Unlock()
	if h != nil && st != nil {
		h(copyState(st))
	}
}

func (c *Client) fireErr(msg string) {
	c.mu.Lock()
	h := c.onErr
	c.mu.Unlock()
	if h != nil {
		h(msg)
	}
}

func copyState(st *State) *State {
	if st == nil {
		return nil
	}
	out := &State{Device: st.Device}
	if len(st.Servo) > 0 {
		out.Servo = append([]float64(nil), st.Servo...)
	}
	if len(st.TCP) > 0 {
		out.TCP = append([]float64(nil), st.TCP...)
	}
	if len(st.Joints) > 0 {
		out.Joints = make(map[string]float64, len(st.Joints))
		for k, v := range st.Joints {
			out.Joints[k] = v
		}
	}
	return out
}

// trimFloat 输出尽量短的合法 JSON 数字（3 位小数足够表达舵机角精度）。
func trimFloat(v float64) string {
	return strings.TrimRight(strings.TrimRight(fmt.Sprintf("%.3f", v), "0"), ".")
}

func formatServo(s []float64) string {
	if len(s) == 0 {
		return "[]"
	}
	parts := make([]string, 0, len(s))
	for _, v := range s {
		parts = append(parts, fmt.Sprintf("%.1f", v))
	}
	return "[" + strings.Join(parts, " ") + "]"
}
