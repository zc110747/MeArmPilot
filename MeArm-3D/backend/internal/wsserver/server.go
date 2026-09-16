package wsserver

import (
	"encoding/json"
	"fmt"
	"log"
	"net"
	"net/http"
	"strconv"
	"strings"
	"sync"
	"time"

	"armpilot/backend/internal/controller"
	"armpilot/backend/internal/protocol"
)

// Config 服务参数。
type Config struct {
	Host string
	Port int
	// Path WebSocket 端点路径（默认 /ws/joint）
	Path string
	// PingIntervalMs 向客户端发 ping 的间隔（ms）
	PingIntervalMs int
	// ClientTimeoutMs 客户端静默多久判死（ms）。必须 > PingIntervalMs。
	ClientTimeoutMs int
}

// DefaultConfig 返回推荐参数。端口刻意避开 MeArm-RemoteControl 的 8080，
// 使"摇杆服务"与"数字孪生关节服务"可以同时跑在一台机器上。
func DefaultConfig() Config {
	return Config{
		Host:            "0.0.0.0",
		Port:            8090,
		Path:            "/ws/joint",
		PingIntervalMs:  15000,
		ClientTimeoutMs: 40000,
	}
}

// Server 聚合 HTTP/WS 与控制器。
type Server struct {
	cfg  Config
	ctl  *controller.Controller
	http *http.Server

	mu      sync.Mutex
	clients map[*client]struct{}
}

// New 创建服务（不启动）。
func New(cfg Config, ctl *controller.Controller) *Server {
	if cfg.Path == "" {
		cfg.Path = "/ws/joint"
	}
	if cfg.PingIntervalMs <= 0 {
		cfg.PingIntervalMs = 15000
	}
	if cfg.ClientTimeoutMs <= cfg.PingIntervalMs {
		cfg.ClientTimeoutMs = cfg.PingIntervalMs*2 + 5000
	}
	s := &Server{cfg: cfg, ctl: ctl, clients: make(map[*client]struct{})}
	// 控制器事件 → 广播到所有客户端（只挂一次，与客户端数量无关）
	ctl.OnJointState(func(joints map[string]float64, at time.Time, origin string) {
		s.broadcast(protocol.ServerMessage{
			Version: protocol.Version, Type: protocol.TypeJointState,
			Timestamp: at.UnixMilli(), Joints: joints, Origin: origin,
		})
	})
	ctl.OnError(func(code, message string) {
		s.broadcast(protocol.ServerMessage{
			Version: protocol.Version, Type: protocol.TypeError,
			Timestamp: nowMs(), Code: code, Message: message,
		})
	})
	ctl.OnStatus(func(connected bool, reason string) {
		t, f := connected, false
		msg := protocol.ServerMessage{
			Version: protocol.Version, Type: protocol.TypeDeviceStatus,
			Timestamp: nowMs(), Device: ctl.DeviceKind(), Connected: &t,
			SimulationMode: protocol.SimulationModeFor(ctl.DeviceKind()),
		}
		if !connected {
			msg.Connected = &f
			msg.Message = reason
		}
		s.broadcast(msg)
	})
	return s
}

// Addr 返回监听地址。
func (s *Server) Addr() string {
	return net.JoinHostPort(s.cfg.Host, strconv.Itoa(s.cfg.Port))
}

// URL 返回可供浏览器使用的地址（用 127.0.0.1 而非 0.0.0.0）。
func (s *Server) URL() string {
	host := s.cfg.Host
	if host == "" || host == "0.0.0.0" {
		host = "127.0.0.1"
	}
	return fmt.Sprintf("ws://%s%s", net.JoinHostPort(host, strconv.Itoa(s.cfg.Port)), s.cfg.Path)
}

// Handler 返回 HTTP 路由表。
//
// 抽成独立方法而不是内联在 ListenAndServe 里，是为了让测试能直接复用**同一份路由**
// —— 早先的测试自己写了个 handler、只转发了 WS 端点，结果 /healthz 静默变成 404，
// 测试通过而生产坏掉这类问题就藏在这种"两份路由"里。
func (s *Server) Handler() http.Handler {
	mux := http.NewServeMux()
	path := s.cfg.Path
	if path == "" {
		path = "/ws/joint"
	}
	if path[0] != '/' {
		path = "/" + path
	}
	mux.HandleFunc(path, s.handleWS)
	mux.HandleFunc("/healthz", s.handleHealth)
	mux.HandleFunc("/", s.handleRoot)
	return mux
}

// ListenAndServe 启动 HTTP 服务（阻塞）。
func (s *Server) ListenAndServe() error {
	s.http = &http.Server{Addr: s.Addr(), Handler: s.Handler()}

	log.Printf("[web] 关节级 WebSocket 已启动: %s", s.URL())
	return s.http.ListenAndServe()
}

// Close 关闭服务并断开所有客户端。
func (s *Server) Close() error {
	s.mu.Lock()
	cs := make([]*client, 0, len(s.clients))
	for c := range s.clients {
		cs = append(cs, c)
	}
	httpSrv := s.http
	s.mu.Unlock()
	for _, c := range cs {
		c.close()
	}
	if httpSrv != nil {
		return httpSrv.Close()
	}
	return nil
}

// handleHealth 供 e2e / 编排脚本等待服务就绪。
func (s *Server) handleHealth(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "application/json")
	_, state := s.ctl.Snapshot()
	_ = json.NewEncoder(w).Encode(map[string]any{
		"ok":      true,
		"device":  s.ctl.DeviceKind(),
		"linked":  s.ctl.DeviceConnected(),
		"state":   state,
		"clients": s.clientCount(),
	})
}

func (s *Server) handleRoot(w http.ResponseWriter, r *http.Request) {
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}
	w.Header().Set("Content-Type", "text/plain; charset=utf-8")
	fmt.Fprintf(w, "ArmPilot joint backend\n\nWebSocket: %s\nHealth:    http://%s/healthz\n",
		s.URL(), s.Addr())
}

// ---------------------------------------------------------------------------
// 客户端
// ---------------------------------------------------------------------------

type client struct {
	conn *Conn
	out  chan string

	closeOnce sync.Once
	done      chan struct{}
}

func newClient(conn *Conn) *client {
	return &client{conn: conn, out: make(chan string, 64), done: make(chan struct{})}
}

// Send 非阻塞投递；慢客户端丢最旧、保最新（状态帧的价值在"新鲜"而非"完整"）。
func (c *client) Send(msg string) {
	select {
	case c.out <- msg:
	default:
		select {
		case <-c.out:
		default:
		}
		select {
		case c.out <- msg:
		default:
		}
	}
}

func (c *client) close() {
	c.closeOnce.Do(func() {
		close(c.done)
		_ = c.conn.Close()
	})
}

func (s *Server) add(c *client) {
	s.mu.Lock()
	s.clients[c] = struct{}{}
	s.mu.Unlock()
}

func (s *Server) remove(c *client) {
	s.mu.Lock()
	delete(s.clients, c)
	s.mu.Unlock()
}

func (s *Server) clientCount() int {
	s.mu.Lock()
	defer s.mu.Unlock()
	return len(s.clients)
}

func (s *Server) broadcast(m protocol.ServerMessage) {
	data, err := json.Marshal(m)
	if err != nil {
		return
	}
	msg := string(data)
	s.mu.Lock()
	cs := make([]*client, 0, len(s.clients))
	for c := range s.clients {
		cs = append(cs, c)
	}
	s.mu.Unlock()
	for _, c := range cs {
		c.Send(msg)
	}
}

// ---------------------------------------------------------------------------
// 连接处理
// ---------------------------------------------------------------------------

func (s *Server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := Upgrade(w, r)
	if err != nil {
		log.Printf("[web] WS 握手失败: %v", err)
		return
	}
	c := newClient(conn)
	s.add(c)
	defer s.remove(c)
	defer c.close()

	log.Printf("[web] 客户端接入: %s", r.RemoteAddr)

	go s.writeLoop(c)
	go s.heartbeat(c)

	// 接入即下发模型真值 + 当前状态，避免新页面停留在"未知"
	c.Send(s.helloJSON())
	_, state := s.ctl.Snapshot()
	c.Send(s.stateJSON(state))

	for {
		raw, err := conn.ReadMessage()
		if err != nil {
			break
		}
		s.handleMessage(c, raw)
	}
	log.Printf("[web] 客户端断开: %s", r.RemoteAddr)
}

func (s *Server) handleMessage(c *client, raw string) {
	var m protocol.ClientMessage
	if err := json.Unmarshal([]byte(raw), &m); err != nil {
		c.Send(s.errJSON(protocol.CodeBadMessage, "JSON 解析失败: "+err.Error()))
		return
	}
	// 版本校验：schema 不一致必须显式拒绝，不能让字段静默错解
	if m.Version != 0 && m.Version != protocol.Version {
		c.Send(s.errJSON(protocol.CodeVersion,
			fmt.Sprintf("协议版本不匹配：客户端 %d，服务器 %d", m.Version, protocol.Version)))
		return
	}

	switch m.Type {
	case protocol.TypeJointCommand:
		if len(m.Joints) == 0 {
			c.Send(s.errJSON(protocol.CodeBadMessage, "joint_command 缺少 joints"))
			return
		}
		if err := s.ctl.Apply(m.Joints); err != nil {
			if re, ok := err.(*controller.RejectError); ok {
				c.Send(s.errJSON(re.Code, re.Message))
				return
			}
			c.Send(s.errJSON(protocol.CodeInternal, err.Error()))
		}
	case protocol.TypeDeviceCommand:
		// 调试 / 验收直通：把一行原始设备指令（`JOY …` / `SET …`）发给链路末端。
		// 回执不走这里 —— `JOY` 触发的 `# SERVO` 会经 device.Lines() 由
		// controller 解析成 OriginDevice 状态后广播，与正常状态同一条路。
		if strings.TrimSpace(m.Line) == "" {
			c.Send(s.errJSON(protocol.CodeBadMessage, "device_command 缺少 line"))
			return
		}
		if err := s.ctl.RawLine(m.Line); err != nil {
			if re, ok := err.(*controller.RejectError); ok {
				c.Send(s.errJSON(re.Code, re.Message))
				return
			}
			c.Send(s.errJSON(protocol.CodeInternal, err.Error()))
		}
	case protocol.TypePing:
		c.Send(s.pongJSON())
	case protocol.TypeStatusRequest:
		c.Send(s.helloJSON())
		_, state := s.ctl.Snapshot()
		c.Send(s.stateJSON(state))
		conn := s.ctl.DeviceConnected()
		c.Send(mustJSON(protocol.ServerMessage{
			Version: protocol.Version, Type: protocol.TypeDeviceStatus,
			Timestamp: nowMs(), Device: s.ctl.DeviceKind(), Connected: &conn,
		}))
	default:
		c.Send(s.errJSON(protocol.CodeBadMessage, "未知消息类型: "+m.Type))
	}
}

// writeLoop 串行化所有出站写（Conn 内部虽有写锁，这里再串一层保证顺序）。
func (s *Server) writeLoop(c *client) {
	for {
		select {
		case <-c.done:
			return
		case msg := <-c.out:
			if err := c.conn.WriteMessage(msg); err != nil {
				c.close()
				return
			}
		}
	}
}

// heartbeat 每 PingInterval 发一次传输层 ping，并检查客户端是否已静默超时。
func (s *Server) heartbeat(c *client) {
	interval := time.Duration(s.cfg.PingIntervalMs) * time.Millisecond
	timeout := time.Duration(s.cfg.ClientTimeoutMs) * time.Millisecond
	t := time.NewTicker(interval)
	defer t.Stop()
	for {
		select {
		case <-c.done:
			return
		case <-t.C:
			if err := c.conn.Ping(); err != nil {
				c.close()
				return
			}
			// LastActivity 由帧层刷新（含浏览器自动回的 pong）
			if time.Since(c.conn.LastActivity()) > timeout {
				log.Printf("[web] 客户端静默超过 %v，判定链路已死并断开", timeout)
				c.close()
				return
			}
		}
	}
}

// ---------------------------------------------------------------------------
// 消息构造
// ---------------------------------------------------------------------------

func (s *Server) helloJSON() string {
	return mustJSON(protocol.ServerMessage{
		Version:   protocol.Version,
		Type:      protocol.TypeHello,
		Timestamp: nowMs(),
		Model:     protocol.BuildModelInfo(s.ctl.Model()),
		Device:    s.ctl.DeviceKind(),
		// sim / mujoco / serial 三态（spec §25）。前端不认识也照常工作。
		SimulationMode: protocol.SimulationModeFor(s.ctl.DeviceKind()),
	})
}

// stateJSON 是客户端接入 / status_request 时的状态快照。
//
// `Origin` 固定为 `command`：快照给的是"命令侧认为机器在哪"，
// **不是**一次设备侧自主变化的通知。标错会让界面在每次重连时
// 把命令悄悄对齐到快照值（跟随的语义被滥用）。
func (s *Server) stateJSON(joints map[string]float64) string {
	return mustJSON(protocol.ServerMessage{
		Version:   protocol.Version,
		Type:      protocol.TypeJointState,
		Timestamp: nowMs(),
		Joints:    joints,
		Origin:    protocol.OriginCommand,
	})
}

func (s *Server) pongJSON() string {
	return mustJSON(protocol.ServerMessage{
		Version: protocol.Version, Type: protocol.TypePong, Timestamp: nowMs(),
	})
}

func (s *Server) errJSON(code, message string) string {
	return mustJSON(protocol.ServerMessage{
		Version: protocol.Version, Type: protocol.TypeError,
		Timestamp: nowMs(), Code: code, Message: message,
	})
}

func mustJSON(m protocol.ServerMessage) string {
	data, err := json.Marshal(m)
	if err != nil {
		return `{"version":1,"type":"error","code":"INTERNAL","message":"encode failed"}`
	}
	return string(data)
}

func nowMs() int64 { return time.Now().UnixMilli() }
