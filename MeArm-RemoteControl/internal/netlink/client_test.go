package netlink

// client_test.go —— TCP 客户端的连接 / 编码 / 解码 / 错误 / 并发 / 关闭 / 重连。
//
// 假服务端是一个**最小 MeArm-3D 替身**：按 docs/tcp-control-v1.md 的 JSON Lines
// 语义逐行应答（含 `state`、`servo` 的行程拒绝）。这样测试测的是"我们对协议的
// 理解"和"并发与生命周期"这两件真会出问题的事，不依赖 MeArm-3D 在不在跑。

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"net"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// ---------------------------------------------------------------------------
// 假服务端
// ---------------------------------------------------------------------------

type hookMode int

const (
	hookDefault hookMode = iota // 用内置 respond
	hookCustom                  // 用 hook 返回的字符串应答（空串 = 不回）
	hookDrop                    // 直接断开这条连接
)

type fakeServer struct {
	t  *testing.T
	ln net.Listener

	mu    sync.Mutex
	lines []string
	servo []float64
	dev   string

	hook func(line string) (string, hookMode)

	drops int32
}

func newFakeServer(t *testing.T) *fakeServer {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("监听失败: %v", err)
	}
	s := &fakeServer{
		t:     t,
		ln:    ln,
		servo: []float64{90, 90, 90, 90},
		dev:   "sim",
	}
	go s.accept()
	t.Cleanup(func() { _ = ln.Close() })
	return s
}

func (s *fakeServer) Port() int { return s.ln.Addr().(*net.TCPAddr).Port }

func (s *fakeServer) accept() {
	for {
		conn, err := s.ln.Accept()
		if err != nil {
			return
		}
		go s.serve(conn)
	}
}

func (s *fakeServer) serve(conn net.Conn) {
	defer conn.Close()
	r := bufio.NewReader(conn)
	for {
		raw, err := r.ReadString('\n')
		if err != nil {
			return
		}
		line := strings.TrimSpace(raw)
		if line == "" {
			continue
		}
		s.mu.Lock()
		s.lines = append(s.lines, line)
		s.mu.Unlock()

		if s.hook != nil {
			out, mode := s.hook(line)
			switch mode {
			case hookDrop:
				atomic.AddInt32(&s.drops, 1)
				return
			case hookCustom:
				if out != "" {
					_, _ = io.WriteString(conn, out+"\n")
				}
				continue
			}
		}
		if out := s.respond(line); out != "" {
			if _, err := io.WriteString(conn, out+"\n"); err != nil {
				return
			}
		}
	}
}

func (s *fakeServer) respond(line string) string {
	var req struct {
		Cmd   string   `json:"cmd"`
		Servo *int     `json:"servo"`
		Angle *float64 `json:"angle"`
	}
	if err := json.Unmarshal([]byte(line), &req); err != nil {
		return `{"ok":false,"error":"invalid json"}`
	}
	switch req.Cmd {
	case "state":
		return s.okState("state")
	case "servo":
		if req.Servo == nil || req.Angle == nil {
			return `{"ok":false,"cmd":"servo","error":"missing servo"}`
		}
		n := *req.Servo
		if n < 1 || n > len(s.servo) {
			return `{"ok":false,"cmd":"servo","error":"invalid servo"}`
		}
		// 模拟服务端的**硬件行程**校验（真值在 robot.yaml，这里只取一段代表性区间）。
		if *req.Angle < 20 || *req.Angle > 160 {
			return `{"ok":false,"cmd":"servo","error":"angle out of range (sx 20.00..160.00)"}`
		}
		s.mu.Lock()
		s.servo[n-1] = *req.Angle
		s.mu.Unlock()
		return s.okState("servo")
	case "move", "gripper":
		return s.okState(req.Cmd)
	}
	return fmt.Sprintf(`{"ok":false,"error":"unknown cmd %q"}`, req.Cmd)
}

func (s *fakeServer) okState(cmd string) string {
	s.mu.Lock()
	servo := append([]float64(nil), s.servo...)
	dev := s.dev
	s.mu.Unlock()
	body, _ := json.Marshal(map[string]any{
		"ok":  true,
		"cmd": cmd,
		"state": map[string]any{
			"joints": map[string]float64{"base": 0, "shoulder": 0.85},
			"servo":  servo,
			"tcp":    []float64{100, 0, 100},
			"device": dev,
		},
	})
	return string(body)
}

// linesCopy 已收到的原始行快照。
func (s *fakeServer) linesCopy() []string {
	s.mu.Lock()
	defer s.mu.Unlock()
	return append([]string(nil), s.lines...)
}

// servoOf 假服务端当前记录的某路舵机角（1-based）。
func (s *fakeServer) servoOf(n int) float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	if n < 1 || n > len(s.servo) {
		return -1
	}
	return s.servo[n-1]
}

// hasLine 是否收到过满足条件的行。
func (s *fakeServer) hasLine(pred func(string) bool) bool {
	for _, l := range s.linesCopy() {
		if pred(l) {
			return true
		}
	}
	return false
}

// ---------------------------------------------------------------------------
// 工具
// ---------------------------------------------------------------------------

func testConfig(port int) Config {
	c := Config{
		Host:             "127.0.0.1",
		Port:             port,
		Axis:             DefaultAxisMap(),
		ConnectTimeoutMs: 300,
		RequestTimeoutMs: 500,
		TickMs:           1,
		MaxTickMs:        200,
		ReconnectMinMs:   40,
		ReconnectMaxMs:   120,
		QueueSize:        16,
		AutoSync:         true,
		FallbackAngle:    90,
	}
	c.Axis.DeadFrac = 0.1
	return c
}

func waitFor(t *testing.T, what string, timeout time.Duration, cond func() bool) {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if cond() {
			return
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatalf("等待超时：%s", what)
}

// startClient 起客户端并等它连上、且拿到基准。
func startClient(t *testing.T, srv *fakeServer) *Client {
	t.Helper()
	c := New(testConfig(srv.Port()))
	return startClientWith(t, c)
}

func startClientWith(t *testing.T, c *Client) *Client {
	t.Helper()
	c.Start()
	t.Cleanup(c.Close)
	waitFor(t, "客户端连上并同步基准", 3*time.Second, func() bool {
		ok, _ := c.Status()
		if !ok {
			return false
		}
		c.mu.Lock()
		base := c.haveBase
		c.mu.Unlock()
		return base
	})
	return c
}

// targetOf 读某轴本地目标角（同包白盒，测试用）。
func targetOf(c *Client, servo int) float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.target[servo-1]
}

// ---------------------------------------------------------------------------
// 用例
// ---------------------------------------------------------------------------

// TestClient_ConnectsAndSyncsBaseline 连接建立后必须**主动取基准**（发 state）。
func TestClient_ConnectsAndSyncsBaseline(t *testing.T) {
	srv := newFakeServer(t)
	c := startClient(t, srv)

	if !srv.hasLine(func(l string) bool { return strings.Contains(l, `"cmd":"state"`) }) {
		t.Fatalf("未收到 state 同步命令，实际收到 %v", srv.linesCopy())
	}
	st := c.LastState()
	if st == nil || len(st.Servo) != 4 {
		t.Fatalf("基准状态异常: %+v", st)
	}
	if st.Device != "sim" {
		t.Errorf("device = %q，期望 sim", st.Device)
	}
	for i, v := range st.Servo {
		if targetOf(c, i+1) != v {
			t.Errorf("servo %d 本地基准 %.2f ≠ 服务端 %.2f", i+1, targetOf(c, i+1), v)
		}
	}
}

// TestClient_ConnectFailureDoesNotCrash 服务端不在时：不崩、状态明确、持续退避重连。
func TestClient_ConnectFailureDoesNotCrash(t *testing.T) {
	// 借一个"已经被释放"的端口。
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	port := ln.Addr().(*net.TCPAddr).Port
	_ = ln.Close()

	c := New(testConfig(port))
	c.Start()
	defer c.Close()

	waitFor(t, "状态变为未连接且有原因", 3*time.Second, func() bool {
		ok, msg := c.Status()
		return !ok && msg != ""
	})
	// 连不上时命令不得 panic，且要能立刻返回（不阻塞调用方）。
	if err := c.Xyz("x", "+", 1); err != nil {
		t.Errorf("入队不应失败（链路错误走状态回调）: %v", err)
	}
	if err := c.Joy(Frame{LX: 1}); err != ErrNoBaseline {
		t.Errorf("无基准时 Joy 应返回 ErrNoBaseline，实际 %v", err)
	}
}

// TestClient_CommandEncoding 发出去的每一行必须是对端协议要的 JSON。
func TestClient_CommandEncoding(t *testing.T) {
	srv := newFakeServer(t)
	c := startClient(t, srv)

	if err := c.Xyz("x", "+", 5); err != nil {
		t.Fatal(err)
	}
	if err := c.Gripper("open"); err != nil {
		t.Fatal(err)
	}
	if err := c.Servo(2, 120); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "三条命令全部到达", 3*time.Second, func() bool {
		return srv.hasLine(func(l string) bool {
			var m map[string]any
			if json.Unmarshal([]byte(l), &m) != nil {
				return false
			}
			return m["cmd"] == "move" && m["axis"] == "x" && m["direction"] == "+" && m["step"] == 5.0
		}) && srv.hasLine(func(l string) bool {
			return strings.Contains(l, `"cmd":"gripper"`) && strings.Contains(l, `"action":"open"`)
		}) && srv.hasLine(func(l string) bool {
			return strings.Contains(l, `"cmd":"servo"`) && strings.Contains(l, `"servo":2`) && strings.Contains(l, `"angle":120`)
		})
	})

	// 每一行都必须是合法 JSON（不存在半包/交叉）。
	for _, l := range srv.linesCopy() {
		var m map[string]any
		if err := json.Unmarshal([]byte(l), &m); err != nil {
			t.Fatalf("收到非法 JSON 行 %q: %v", l, err)
		}
		if m["cmd"] == nil {
			t.Fatalf("行缺少 cmd 字段: %q", l)
		}
	}
}

// TestClient_LocalArgValidation 明显非法的参数在**本地**就被挡掉（不污染链路）。
func TestClient_LocalArgValidation(t *testing.T) {
	srv := newFakeServer(t)
	c := startClient(t, srv)
	before := len(srv.linesCopy())

	cases := []struct {
		name string
		err  error
	}{
		{"非法轴", c.Xyz("w", "+", 5)},
		{"非法方向", c.Xyz("x", ">", 5)},
		{"步长为 0", c.Xyz("x", "+", 0)},
		{"步长为负", c.Xyz("z", "-", -1)},
		{"非法爪动作", c.Gripper("shake")},
		{"舵机编号越界", c.Servo(0, 90)},
		{"角度 NaN", c.Servo(1, nanValue())},
	}
	for _, tc := range cases {
		if tc.err == nil {
			t.Errorf("%s: 应当报错", tc.name)
		}
	}
	time.Sleep(120 * time.Millisecond)
	if after := len(srv.linesCopy()); after != before {
		t.Errorf("非法参数不应产生任何下发：之前 %d 行，之后 %d 行", before, after)
	}
}

// TestClient_ServerErrorReported 服务端 ok=false 必须上报，且**连接保持可用**。
func TestClient_ServerErrorReported(t *testing.T) {
	srv := newFakeServer(t)
	var mu sync.Mutex
	var errs []string
	srv.hook = func(line string) (string, hookMode) {
		if strings.Contains(line, `"cmd":"state"`) {
			return "", hookDefault
		}
		return `{"ok":false,"cmd":"move","error":"OUT_OF_WORKSPACE: beyond reach"}`, hookCustom
	}
	c := New(testConfig(srv.Port()))
	c.OnError(func(msg string) {
		mu.Lock()
		errs = append(errs, msg)
		mu.Unlock()
	})
	c.Start()
	defer c.Close()

	waitFor(t, "连上", 3*time.Second, func() bool { ok, _ := c.Status(); return ok })
	if err := c.Gripper("open"); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "收到服务端错误", 3*time.Second, func() bool {
		mu.Lock()
		defer mu.Unlock()
		for _, e := range errs {
			if strings.Contains(e, "OUT_OF_WORKSPACE") {
				return true
			}
		}
		return false
	})
	// 错误之后连接必须还能用（硬指标：错误只影响那一条命令）。
	srv.hook = nil
	if err := c.Xyz("x", "+", 1); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "错误之后仍可下发", 3*time.Second, func() bool {
		return srv.hasLine(func(l string) bool { return strings.Contains(l, `"direction":"+"`) })
	})
}

// TestClient_RollsBackOnRejection 服务端拒绝（越界）后，本地目标必须回滚到
// 最近一次确认值 —— 否则会带着"没被接受的那一截"继续累加，越走越远。
func TestClient_RollsBackOnRejection(t *testing.T) {
	srv := newFakeServer(t)
	c := startClient(t, srv)

	// 先落到一个服务端接受的位置（120 在假服务端 20..160 之内）。
	if err := c.Servo(1, 120); err != nil {
		t.Fatal(err)
	}
	// ⚠️ 必须等**服务端真的收到并接受**了这条，再去改 hook —— 只看本地 target
	//    会误判（`Servo` 是入队即返回，命令还在队列里）。
	waitFor(t, "服务端接受 120", 3*time.Second, func() bool { return srv.servoOf(1) == 120 })
	if targetOf(c, 1) != 120 {
		t.Fatalf("本地目标未对齐确认值：%.2f", targetOf(c, 1))
	}

	// 再要求一个服务端拒绝的位置 —— 但本地 clamp 到 180，服务端判越界。
	srv.hook = func(line string) (string, hookMode) {
		if strings.Contains(line, `"cmd":"servo"`) {
			return `{"ok":false,"cmd":"servo","error":"angle out of range (sx 20.00..160.00)"}`, hookCustom
		}
		return "", hookDefault
	}
	if err := c.Servo(1, 180); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "越界被拒绝后目标回滚到确认值 120", 3*time.Second, func() bool {
		return targetOf(c, 1) == 120
	})
}

// TestClient_ConcurrentWriters 多 goroutine 并发提交时，链路上不得出现交叉的 JSON。
//
// 配合 `go test -race`：写路径只有 pump 一个 goroutine，因此这里同时验证
// "有锁保护"和"没有多余写者"。
func TestClient_ConcurrentWriters(t *testing.T) {
	srv := newFakeServer(t)
	cfg := testConfig(srv.Port())
	// 并发压测要给够队列：`enqueue` 的"满则丢最旧"是**设计使然**（绝不无限堆积），
	// 这里测的是"不交叉"，不是"不丢"。
	cfg.QueueSize = 512
	c := startClientWith(t, New(cfg))

	var wg sync.WaitGroup
	const workers, rounds = 8, 40
	for w := 0; w < workers; w++ {
		wg.Add(1)
		go func(w int) {
			defer wg.Done()
			for i := 0; i < rounds; i++ {
				_ = c.Joy(Frame{LX: 1, LY: -1, RX: 1, RY: -1})
				if i%7 == 0 {
					_ = c.Xyz("x", "+", 1)
				}
				if i%11 == 0 {
					_ = c.Gripper("close")
				}
				if i%13 == 0 {
					_ = c.Servo(3, 90)
				}
			}
		}(w)
	}
	wg.Wait()

	waitFor(t, "离散命令全部到达", 5*time.Second, func() bool {
		n := 0
		for _, l := range srv.linesCopy() {
			if strings.Contains(l, `"cmd":"gripper"`) {
				n++
			}
		}
		return n >= workers*3 // 每 worker 至少 ceil(rounds/11) 次，留余量
	})

	for _, l := range srv.linesCopy() {
		var m map[string]any
		if err := json.Unmarshal([]byte(l), &m); err != nil {
			t.Fatalf("并发下发产生了非法 JSON 行 %q", l)
		}
	}
}

// TestClient_ReconnectsAfterServerDrop 服务端断开后必须自动重连，**并重新取基准**。
func TestClient_ReconnectsAfterServerDrop(t *testing.T) {
	srv := newFakeServer(t)
	var calls, dropped int32
	srv.hook = func(line string) (string, hookMode) {
		// 第 1 条（连接建立时的 state）照常回；第 2 条掐掉连接（且只掐一次，
		// 否则重连后会被反复掐死，测的就不是重连而是死循环）。
		if atomic.AddInt32(&calls, 1) >= 2 && atomic.CompareAndSwapInt32(&dropped, 0, 1) {
			return "", hookDrop
		}
		return "", hookDefault
	}
	c := startClient(t, srv)

	// 主动发一条 ⇒ 这就是第 2 条命令 ⇒ 触发断开。
	if err := c.Gripper("open"); err != nil {
		t.Fatal(err)
	}
	waitFor(t, "断开后自动重连", 6*time.Second, func() bool {
		ok, _ := c.Status()
		return ok && atomic.LoadInt32(&srv.drops) > 0
	})
	// 重连之后必须**再发一次 state**（基准不能沿用断线前的）。
	waitFor(t, "重连后重新取基准", 6*time.Second, func() bool {
		cnt := 0
		for _, l := range srv.linesCopy() {
			if strings.Contains(l, `"cmd":"state"`) {
				cnt++
			}
		}
		return cnt >= 2
	})
}

// TestClient_FallsBackWhenServerHasNoState 老服务端不认识 `state` 时必须明确降级，
// 而不是静默用一个猜的角度当事实。
func TestClient_FallsBackWhenServerHasNoState(t *testing.T) {
	srv := newFakeServer(t)
	srv.hook = func(line string) (string, hookMode) {
		if strings.Contains(line, `"cmd":"state"`) {
			return `{"ok":false,"error":"unknown cmd \"state\""}`, hookCustom
		}
		return "", hookDefault
	}
	var mu sync.Mutex
	var errs []string
	c := New(testConfig(srv.Port()))
	c.OnError(func(msg string) {
		mu.Lock()
		errs = append(errs, msg)
		mu.Unlock()
	})
	c.Start()
	defer c.Close()

	waitFor(t, "降级上报", 3*time.Second, func() bool {
		mu.Lock()
		defer mu.Unlock()
		for _, e := range errs {
			if strings.Contains(e, "基准") {
				return true
			}
		}
		return false
	})
	// 降级之后仍然可用（走配置兜底角）。
	c.mu.Lock()
	base, fb := c.haveBase, c.target[0]
	c.mu.Unlock()
	if !base || fb != 90 {
		t.Errorf("降级应建立 %.0f° 基准，实际 haveBase=%v target=%.1f", 90.0, base, fb)
	}
}

// TestClient_CloseReleasesEverything Close 必须在有限时间内返回，且之后状态归零。
func TestClient_CloseReleasesEverything(t *testing.T) {
	srv := newFakeServer(t)
	c := New(testConfig(srv.Port()))
	c.Start()
	waitFor(t, "连上", 3*time.Second, func() bool { ok, _ := c.Status(); return ok })

	done := make(chan struct{})
	go func() { c.Close(); close(done) }()
	select {
	case <-done:
	case <-time.After(3 * time.Second):
		t.Fatal("Close 超时 —— 有 goroutine 没退出（典型的'阻塞在 Read 上'）")
	}
	if ok, _ := c.Status(); ok {
		t.Error("Close 之后状态应为未连接")
	}
	if err := c.Xyz("x", "+", 1); err == nil {
		t.Error("Close 之后入队应报错")
	}
	_ = srv
}

// TestClient_JoyProducesServoCommands 摇杆必须真的变成 `servo` 命令，
// 且**只动推的那一轴**（静止轴零流量）。
func TestClient_JoyProducesServoCommands(t *testing.T) {
	srv := newFakeServer(t)
	c := startClient(t, srv)

	// 只推左摇杆 X（→ 接线表的 LXServo）。
	if err := c.Joy(Frame{LX: 1}); err != nil {
		t.Fatal(err)
	}
	want := fmt.Sprintf(`"servo":%d`, c.cfg.Axis.LXServo)
	waitFor(t, "摇杆产生 servo 命令", 3*time.Second, func() bool {
		return srv.hasLine(func(l string) bool {
			return strings.Contains(l, `"cmd":"servo"`) && strings.Contains(l, want)
		})
	})
	if targetOf(c, c.cfg.Axis.LXServo) <= 90 {
		t.Errorf("推 +X 后目标角应增大，实际 %.2f", targetOf(c, c.cfg.Axis.LXServo))
	}
	// 其它轴不得被摇杆改动（初始 90）。
	for s := 1; s <= 4; s++ {
		if s == c.cfg.Axis.LXServo {
			continue
		}
		if v := targetOf(c, s); v != 90 {
			t.Errorf("未推的舵机 %d 目标角被改动: %.2f", s, v)
		}
	}
}

func nanValue() float64 {
	var z float64
	return z / z
}
