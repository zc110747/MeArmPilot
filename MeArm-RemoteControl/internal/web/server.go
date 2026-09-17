// package web 提供本机控制用的 HTTP 服务与 WebSocket。
//
//   - HTTP 服务 web/static 下的静态页面（Three.js 3D 摇杆）。
//   - /ws 提供 WebSocket：网页把摇杆坐标 / 指令发上来，经 `link.Link` 下发到
//     当前通道（串口真机 / MeArm-3D TCP），设备回显经 hub 以 JSON 推回网页。
//
// 本包**不知道**当前是串口还是网络 —— 那由装配方注入的 `link.Link` 决定；
// 两者的能力差异通过 `Caps` 显式告诉网页（见 internal/link）。
package web

import (
	"encoding/json"
	"io/fs"
	"log"
	"math"
	"net/http"
	"regexp"
	"strconv"
	"strings"
	"sync"

	"arm-web/internal/config"
	"arm-web/internal/hub"
	"arm-web/internal/link"
)

// Server 聚合 HTTP/WS 与依赖。
type Server struct {
	cfg      config.WebConfig
	link     link.Link
	hub      *hub.Hub
	servoIDs []int
	staticFS fs.FS

	mu      sync.Mutex
	clients map[*wsClient]struct{} // 仅 WebSocket 客户端，用于下发连接状态变更

	// 双摇杆合并状态：左右摇杆各自最后上报的坐标，下发时合并为一帧交给 link。
	joyMu sync.Mutex
	joyL  [2]float64 // {x, y} 左摇杆
	joyR  [2]float64 // {x, y} 右摇杆

	// 最近已知舵机角度（S6..S9）。**仅串口模式**使用：角度从设备回显文本里
	// 解析（设备仅在 STATUS / SET / JOY 应答里携带角度，可能只带部分轴），
	// 这里做合并，保证单轴回显不会把其它轴清零。
	//
	// 网络模式不走这里 —— 它的角度来自服务端结构化的 `state.servo`。
	anglesMu   sync.Mutex
	lastAngles [4]int  // 下标 0..3 对应 S6..S9
	haveAngles [4]bool // 各轴是否已有过回显
}

func New(cfg config.WebConfig, l link.Link, h *hub.Hub, servoIDs []int, staticFS fs.FS) *Server {
	return &Server{
		cfg:      cfg,
		link:     l,
		hub:      h,
		servoIDs: append([]int(nil), servoIDs...),
		staticFS: staticFS,
		clients:  make(map[*wsClient]struct{}),
	}
}

// ListenAndServe 启动 HTTP 服务（阻塞）。
func (s *Server) ListenAndServe() error {
	mux := http.NewServeMux()
	wsPath := s.cfg.WSPath
	if !strings.HasPrefix(wsPath, "/") {
		wsPath = "/" + wsPath
	}
	mux.HandleFunc(wsPath, s.handleWS)
	// 静态资源（编译期内嵌，无需磁盘目录）
	fileServer := http.FileServer(http.FS(s.staticFS))
	mux.Handle("/", fileServer)

	addr := s.cfg.Addr()
	log.Printf("[web] 本机控制页面已启动: http://%s%s  (WebSocket: %s)", addr, "/", wsPath)
	return http.ListenAndServe(addr, mux)
}

// wsClient 实现 hub.Client：把设备回显以 JSON 形式发往浏览器。
type wsClient struct {
	conn *Conn
	out  chan string
	mu   sync.Mutex
}

func (c *wsClient) Send(line string) {
	select {
	case c.out <- line:
	default:
	}
}

// 客户端 -> 服务器 的消息
type clientMsg struct {
	T    string  `json:"t"`    // joy | cmd | xyz | grip | servo | refresh | ping
	Side string  `json:"side"` // joy 专用：L=左摇杆 / R=右摇杆
	X    float64 `json:"x"`    // 摇杆 X ∈ [-1,1]
	Y    float64 `json:"y"`    // 摇杆 Y ∈ [-1,1]
	C    string  `json:"c"`    // 原始指令文本（cmd 类型，仅串口模式）

	// ---- 以下仅网络模式使用（xyz / grip / servo）----
	Axis      string   `json:"axis"`      // xyz：x | y | z
	Direction string   `json:"direction"` // xyz：+ | -
	Step      *float64 `json:"step"`      // xyz：步长（mm）
	Servo     *int     `json:"servo"`     // servo：TCP 编号 1..N
	Angle     *float64 `json:"angle"`     // servo：舵机角（度）
	Action    string   `json:"action"`    // grip：open | close
}

// 服务器 -> 客户端 的消息
//
// 消息类型：
//
//	serial       链路回显（串口模式：设备文本行；含解析出的 angles）
//	caps         通道能力（接入时一次，决定网页显示哪些控件）
//	link_status  链路连接状态（接入时一次 + 每次翻转）
//	state        网络模式的状态快照（角度 + 末端位置 + 链路末端类型）
//	err / pong
type serverMsg struct {
	T          string     `json:"t"`
	Line       string     `json:"line,omitempty"`
	Angles     *armAngles `json:"angles,omitempty"`
	Msg        string     `json:"msg,omitempty"`
	Connected  *bool      `json:"connected,omitempty"` // link_status: 链路是否已连接
	SerialErr  string     `json:"serial_err,omitempty"` // link_status: 未连接原因（字段名为兼容沿用）
	CommErr    *bool      `json:"comm_err,omitempty"`   // link_status: 通讯是否失败
	CommErrMsg string     `json:"comm_err_msg,omitempty"`
	// ---- 网络模式 ----
	Caps   *link.Caps `json:"caps,omitempty"`   // caps 消息
	TCP    []float64  `json:"tcp,omitempty"`    // state 消息：末端位置 [x,y,z] mm
	Device string     `json:"device,omitempty"` // state 消息：链路末端（sim / serial / mujoco）
}

type armAngles struct {
	S6 int `json:"s6"`
	S7 int `json:"s7"`
	S8 int `json:"s8"`
	S9 int `json:"s9"`
	OK bool `json:"ok"`
}

// reStatus 兼容固件 STATUS 应答的模式后缀：S6=90(H) / S8=100(L)
var reStatus = regexp.MustCompile(`S6=(\d+)(?:\([HL]\))? S7=(\d+)(?:\([HL]\))? S8=(\d+)(?:\([HL]\))? S9=(\d+)(?:\([HL]\))?`)

// reSingle 单舵机角度（带可选模式后缀）
var reSingle = regexp.MustCompile(`S([6789])=(\d+)(?:\([HL]\))?`)

func (s *Server) handleWS(w http.ResponseWriter, r *http.Request) {
	conn, err := Upgrade(w, r)
	if err != nil {
		log.Printf("[web] WS 握手失败: %v", err)
		return
	}
	defer conn.Close()

	c := &wsClient{conn: conn, out: make(chan string, 64)}
	s.hub.Register(c)
	s.addClient(c)
	defer s.hub.Unregister(c)
	defer s.removeClient(c)

	done := make(chan struct{})
	defer close(done)
	go s.wsWriter(c, done)

	// 接入即同步当前通道能力（决定网页显示哪些控件），避免界面靠猜
	s.sendCaps(c)
	// 接入即同步链路连接/通讯状态，避免界面停留在"未知"
	st := s.link.Status()
	s.sendStatusTo(c, st.Connected, st.Err, st.CommErr, st.CommErrMsg)
	// 串口模式：接入即推送当前角度快照（若有），新开的页面立刻显示既有角度
	if ang := s.anglesSnapshot(); ang != nil {
		s.wsSend(c, serverMsg{T: "serial", Angles: ang})
	}
	// 网络模式：状态是**推**给客户端的（每条应答带一次），后接入的页面会错过
	// 之前那些推送，于是刚打开时界面停在 `--` 直到用户动一下。
	// 这里主动拉一次只读状态把缺口补上 —— 复用 RefreshState 而不是另加一个
	// "读缓存"接口：不走缓存就不会出现"显示的是服务端早已变更的旧值"。
	//
	// 失败**不上报**：链路还没连上时它只是入队等待，串口模式则是不支持 ——
	// 两者都不是需要打扰用户的错误，连接状态条已经在如实表达链路情况。
	_ = s.link.RefreshState()

	log.Printf("[web] WebSocket 客户端接入: %s", r.RemoteAddr)

	for {
		raw, err := conn.ReadMessage()
		if err != nil {
			break
		}
		var m clientMsg
		if err := json.Unmarshal([]byte(raw), &m); err != nil {
			s.sendErr(c, "JSON 解析失败")
			continue
		}
		switch m.T {
		case "ping":
			s.wsSend(c, serverMsg{T: "pong"})
		case "joy":
			// 双摇杆：左右摇杆各自上报坐标，合并后交给当前通道。
			// 串口通道内部做命令-应答门控 + 摇杆最新值合并（中间位置不重复下发）；
			// 网络通道内部按时间积分并按舵机维度做 latest-wins。
			side := m.Side
			if side != "L" && side != "R" {
				side = "L"
			}
			s.joyMu.Lock()
			if side == "L" {
				s.joyL[0], s.joyL[1] = clampF(m.X), clampF(m.Y)
			} else {
				s.joyR[0], s.joyR[1] = clampF(m.X), clampF(m.Y)
			}
			lx, ly, rx, ry := s.joyL[0], s.joyL[1], s.joyR[0], s.joyR[1]
			s.joyMu.Unlock()
			if err := s.link.Joy(lx, ly, rx, ry); err != nil {
				s.sendErr(c, err.Error())
			}
		case "cmd":
			if err := s.link.Raw(m.C); err != nil {
				s.sendErr(c, err.Error())
			}
		case "xyz":
			if m.Step == nil {
				s.sendErr(c, "xyz 缺少 step")
				continue
			}
			log.Printf("[web] XYZ %s%s step=%v", m.Axis, m.Direction, *m.Step)
			if err := s.link.XYZ(m.Axis, m.Direction, *m.Step); err != nil {
				s.sendErr(c, err.Error())
			}
		case "grip":
			log.Printf("[web] 夹爪 %s", m.Action)
			if err := s.link.Gripper(m.Action); err != nil {
				s.sendErr(c, err.Error())
			}
		case "servo":
			if m.Servo == nil || m.Angle == nil {
				s.sendErr(c, "servo 缺少 servo / angle")
				continue
			}
			log.Printf("[web] 直接舵机 %d = %v", *m.Servo, *m.Angle)
			if err := s.link.ServoDirect(*m.Servo, *m.Angle); err != nil {
				s.sendErr(c, err.Error())
			}
		case "refresh":
			if err := s.link.RefreshState(); err != nil {
				s.sendErr(c, err.Error())
			}
		default:
			s.sendErr(c, "未知消息类型: "+m.T)
		}
	}
	log.Printf("[web] WebSocket 客户端断开: %s", r.RemoteAddr)
}

// sendCaps 把通道能力推给客户端（接入时一次）。
func (s *Server) sendCaps(c *wsClient) {
	caps := s.link.Caps()
	s.wsSend(c, serverMsg{T: "caps", Caps: &caps})
}

// wsWriter 把设备回显（经 hub）封装成 JSON 推送给浏览器，
// 同时解析角度（合并进最近已知值）供网页舵机角度显示。
func (s *Server) wsWriter(c *wsClient, done chan struct{}) {
	for {
		select {
		case <-done:
			return
		case line := <-c.out:
			msg := serverMsg{T: "serial", Line: line}
			if m := parseAngles(line); len(m) > 0 {
				msg.Angles = s.mergeAngles(m)
			}
			if data, err := json.Marshal(msg); err == nil {
				c.conn.WriteMessage(string(data))
			}
		}
	}
}

// mergeAngles 把本条回显里出现的舵机角度合并进最近已知值。
// 单轴回显（如 "OK SET S9=120"）只更新对应轴，不会把其它轴清零。
// 返回完整快照；尚有轴从未见过回显时返回 nil（避免 UI 把未知轴显示成 0）。
func (s *Server) mergeAngles(m map[int]int) *armAngles {
	s.anglesMu.Lock()
	defer s.anglesMu.Unlock()
	for id, v := range m {
		if id < 6 || id > 9 {
			continue
		}
		s.lastAngles[id-6] = v
		s.haveAngles[id-6] = true
	}
	return s.snapshotLocked()
}

// anglesSnapshot 返回当前完整角度快照（未凑齐四轴时返回 nil）。
func (s *Server) anglesSnapshot() *armAngles {
	s.anglesMu.Lock()
	defer s.anglesMu.Unlock()
	return s.snapshotLocked()
}

func (s *Server) snapshotLocked() *armAngles {
	for i := range s.haveAngles {
		if !s.haveAngles[i] {
			return nil
		}
	}
	return &armAngles{S6: s.lastAngles[0], S7: s.lastAngles[1], S8: s.lastAngles[2], S9: s.lastAngles[3], OK: true}
}

func (s *Server) wsSend(c *wsClient, m serverMsg) {
	if data, err := json.Marshal(m); err == nil {
		c.conn.WriteMessage(string(data))
	}
}

func (s *Server) sendErr(c *wsClient, msg string) {
	s.wsSend(c, serverMsg{T: "err", Msg: msg})
}

// addClient / removeClient 维护 WebSocket 客户端集合（仅用于串口状态广播）。
func (s *Server) addClient(c *wsClient) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.clients[c] = struct{}{}
}

func (s *Server) removeClient(c *wsClient) {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.clients, c)
}

// sendStatusTo 向单个客户端推送链路连接/通讯状态。
func (s *Server) sendStatusTo(c *wsClient, connected bool, lerr string, commErr bool, commErrMsg string) {
	s.wsSend(c, serverMsg{
		T:          "link_status",
		Connected:  &connected,
		SerialErr:  lerr,
		CommErr:    &commErr,
		CommErrMsg: commErrMsg,
	})
}

// BroadcastStatus 把链路连接/通讯状态变更广播给所有 WebSocket 客户端。
// 由装配方在 serial / netlink 的状态回调里调用。
//
// 消息类型原名 `serial_status`，随 Link 抽象统一为 `link_status`——
// 它现在描述的是"当前通道"而不是"串口"。字段名 `serial_err` 沿用以免
// 前后端两处不同步地各改一次。
func (s *Server) BroadcastStatus(connected bool, lerr string, commErr bool, commErrMsg string) {
	s.broadcast(serverMsg{
		T:          "link_status",
		Connected:  &connected,
		SerialErr:  lerr,
		CommErr:    &commErr,
		CommErrMsg: commErrMsg,
	})
}

// BroadcastNetState 网络模式：把服务端状态快照广播给所有网页客户端。
//
// ⚠️ 角度一律来自服务端**结构化**的 `state.servo`（成功应答里带回来的），
//    不是本地缓存推定的值，也不是"命令发出去就算到位"。若服务端没给状态，
//    这里什么也不推 —— 界面停在 `--`，而不是显示一个编出来的角度。
func (s *Server) BroadcastNetState(servo []float64, tcp []float64, device string) {
	msg := serverMsg{T: "state", Angles: s.netAngles(servo), TCP: tcp, Device: device}
	s.broadcast(msg)
}

// BroadcastError 把一条链路级错误广播到网页。
//
// 网络模式用它上报"服务端拒绝"与"降级提示"（例如服务端不认识 `state`、
// 目标角被行程拒绝并已回滚）；串口模式不用它（错误随回显文本一起走）。
func (s *Server) BroadcastError(msg string) {
	s.broadcast(serverMsg{T: "err", Msg: msg})
}

// broadcast 把一条消息发给所有 WebSocket 客户端。
//
// 快照客户端集合后再写：WriteMessage 是阻塞式网络写，绝不能在持锁状态下执行，
// 否则一个半死连接会把所有下发路径一起拖住。
func (s *Server) broadcast(msg serverMsg) {
	data, err := json.Marshal(msg)
	if err != nil {
		return
	}
	s.mu.Lock()
	clients := make([]*wsClient, 0, len(s.clients))
	for c := range s.clients {
		clients = append(clients, c)
	}
	s.mu.Unlock()
	for _, c := range clients {
		c.conn.WriteMessage(string(data))
	}
}

// netAngles 把 MeArm-3D 的 `state.servo` 数组（按 TCP 编号 1..N 排列）
// 转成网页既有的 S6..S9 视角。
//
// 映射表就是 `network.servo_ids` —— 与摇杆轴映射**共用同一份接线表**，
// 不另开一张（两张表必然漂移）。
func (s *Server) netAngles(servo []float64) *armAngles {
	if len(servo) == 0 || len(s.servoIDs) == 0 {
		return nil
	}
	out := &armAngles{OK: len(servo) >= len(s.servoIDs)}
	for i, id := range s.servoIDs {
		if i >= len(servo) {
			return nil
		}
		v := int(math.Round(servo[i]))
		switch id {
		case 6:
			out.S6 = v
		case 7:
			out.S7 = v
		case 8:
			out.S8 = v
		case 9:
			out.S9 = v
		}
	}
	return out
}

// parseAngles 从回显行中提取出现的舵机角度（可能只有部分轴）。
// 支持：
//   - "OK JOY S6=89 S7=89 S8=89 S9=89"
//   - "STATUS S6=90(H) S7=90(H) S8=100(H) S9=76(H)"（带 H/L 模式后缀）
//   - 单舵机 "OK SET S9=120" / "S8=75"
//
// 返回 id->角度 映射；调用方（mergeAngles）负责与最近已知值合并。
func parseAngles(line string) map[int]int {
	m := map[int]int{}
	for _, kv := range reStatus.FindAllStringSubmatch(line, -1) {
		s6, _ := strconv.Atoi(kv[1])
		s7, _ := strconv.Atoi(kv[2])
		s8, _ := strconv.Atoi(kv[3])
		s9, _ := strconv.Atoi(kv[4])
		m[6], m[7], m[8], m[9] = s6, s7, s8, s9
	}
	for _, kv := range reSingle.FindAllStringSubmatch(line, -1) {
		id, _ := strconv.Atoi(kv[1])
		v, _ := strconv.Atoi(kv[2])
		m[id] = v
	}
	return m
}

func clampF(v float64) float64 {
	if v < -1 {
		return -1
	}
	if v > 1 {
		return 1
	}
	return v
}
