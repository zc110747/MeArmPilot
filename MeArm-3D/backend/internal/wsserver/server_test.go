package wsserver

import (
	"bufio"
	"crypto/rand"
	"encoding/base64"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"armpilot/backend/internal/controller"
	"armpilot/backend/internal/device"
	"armpilot/backend/internal/protocol"
	"armpilot/backend/internal/robot"
)

// ---------------------------------------------------------------------------
// 最小 WebSocket 客户端（标准库）。之所以不用现成库：本项目的服务端就是手写的
// 极简 RFC6455，用一个第三方客户端测它反而测不出"掩码/长度扩展/控制帧"这些
// 自己实现才有的细节。客户端帧按规范必须掩码。
// ---------------------------------------------------------------------------

type wsClient struct {
	conn net.Conn
	r    *bufio.Reader
}

func dialWS(t *testing.T, httpURL, path string) *wsClient {
	t.Helper()
	addr := strings.TrimPrefix(httpURL, "http://")
	conn, err := net.DialTimeout("tcp", addr, 3*time.Second)
	if err != nil {
		t.Fatalf("TCP 连接失败: %v", err)
	}
	keyRaw := make([]byte, 16)
	_, _ = rand.Read(keyRaw)
	key := base64.StdEncoding.EncodeToString(keyRaw)
	req := fmt.Sprintf("GET %s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\n"+
		"Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n",
		path, addr, key)
	if _, err := conn.Write([]byte(req)); err != nil {
		t.Fatalf("握手写失败: %v", err)
	}
	r := bufio.NewReader(conn)
	status, err := r.ReadString('\n')
	if err != nil {
		t.Fatalf("读握手响应失败: %v", err)
	}
	if !strings.Contains(status, "101") {
		t.Fatalf("握手未返回 101: %q", status)
	}
	// 吃掉响应头
	for {
		line, err := r.ReadString('\n')
		if err != nil {
			t.Fatalf("读响应头失败: %v", err)
		}
		if line == "\r\n" || line == "\n" {
			break
		}
	}
	c := &wsClient{conn: conn, r: r}
	t.Cleanup(func() { conn.Close() })
	return c
}

func (c *wsClient) send(t *testing.T, payload string) {
	t.Helper()
	data := []byte(payload)
	n := len(data)
	var hdr []byte
	switch {
	case n < 126:
		hdr = []byte{0x81, byte(0x80 | n)}
	case n < 65536:
		hdr = []byte{0x81, 0x80 | 126, byte(n >> 8), byte(n)}
	default:
		hdr = []byte{0x81, 0x80 | 127}
		var ext [8]byte
		binary.BigEndian.PutUint64(ext[:], uint64(n))
		hdr = append(hdr, ext[:]...)
	}
	mask := []byte{0x12, 0x34, 0x56, 0x78}
	frame := append(hdr, mask...)
	for i := range data {
		frame = append(frame, data[i]^mask[i%4])
	}
	if _, err := c.conn.Write(frame); err != nil {
		t.Fatalf("发送帧失败: %v", err)
	}
}

// recv 读取一条**文本**消息；控制帧（ping/pong/close）自动跳过或应答。
func (c *wsClient) recv(t *testing.T, timeout time.Duration) string {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for {
		if err := c.conn.SetReadDeadline(deadline); err != nil {
			t.Fatalf("设置读超时失败: %v", err)
		}
		opcode, payload, err := c.readFrame()
		if err != nil {
			t.Fatalf("读帧失败: %v", err)
		}
		switch opcode {
		case 0x1, 0x2:
			t.Logf("RECV: %s", truncate(string(payload), 120))
			return string(payload)
		case 0x9:
			// 回 pong（真实浏览器会自动做；这里手工实现以验证服务端心跳链路）
			_ = c.writeFrame(0xA, payload)
		case 0xA:
			continue
		case 0x8:
			t.Fatal("服务端主动关闭连接")
		}
	}
}

func (c *wsClient) writeFrame(opcode byte, data []byte) error {
	mask := []byte{0xAB, 0xCD, 0xEF, 0x01}
	n := len(data)
	var hdr []byte
	if n < 126 {
		hdr = []byte{0x80 | opcode, byte(0x80 | n)}
	} else {
		hdr = []byte{0x80 | opcode, 0x80 | 126, byte(n >> 8), byte(n)}
	}
	frame := append(hdr, mask...)
	for i := range data {
		frame = append(frame, data[i]^mask[i%4])
	}
	_, err := c.conn.Write(frame)
	return err
}

func (c *wsClient) readFrame() (byte, []byte, error) {
	var b [2]byte
	if _, err := io.ReadFull(c.r, b[:]); err != nil {
		return 0, nil, err
	}
	opcode := b[0] & 0x0f
	masked := b[1]&0x80 != 0
	length := int(b[1] & 0x7f)
	switch length {
	case 126:
		var ext [2]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint16(ext[:]))
	case 127:
		var ext [8]byte
		if _, err := io.ReadFull(c.r, ext[:]); err != nil {
			return 0, nil, err
		}
		length = int(binary.BigEndian.Uint64(ext[:]))
	}
	var mask [4]byte
	if masked {
		if _, err := io.ReadFull(c.r, mask[:]); err != nil {
			return 0, nil, err
		}
	}
	payload := make([]byte, length)
	if length > 0 {
		if _, err := io.ReadFull(c.r, payload); err != nil {
			return 0, nil, err
		}
		if masked {
			for i := range payload {
				payload[i] ^= mask[i%4]
			}
		}
	}
	return opcode, payload, nil
}

// recvType 一直读到指定 type 的消息（跳过中间的状态帧）。
func (c *wsClient) recvType(t *testing.T, want string, timeout time.Duration) protocol.ServerMessage {
	t.Helper()
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		raw := c.recv(t, time.Until(deadline))
		var m protocol.ServerMessage
		if err := json.Unmarshal([]byte(raw), &m); err != nil {
			t.Fatalf("上行消息不是合法 JSON: %q", raw)
		}
		if m.Type == want {
			return m
		}
	}
	t.Fatalf("超时未收到 type=%s 的消息", want)
	return protocol.ServerMessage{}
}

// ---------------------------------------------------------------------------
// 测试夹具：真实 sim 设备 + 真实控制器 + httptest 服务器
// ---------------------------------------------------------------------------

type fixture struct {
	srv  *Server
	http *httptest.Server
	ctl  *controller.Controller
}

func newFixture(t *testing.T, tune device.SimTuning) *fixture {
	t.Helper()
	p, err := filepath.Abs(filepath.Join("..", "..", "..", "robot-package", "mearm-v1", "model", "robot.yaml"))
	if err != nil {
		t.Fatalf("路径解析失败: %v", err)
	}
	m, err := robot.Load(p)
	if err != nil {
		t.Fatalf("Load 失败: %v", err)
	}
	dev, err := device.NewSim(m, tune)
	if err != nil {
		t.Fatalf("NewSim 失败: %v", err)
	}
	ctl := controller.New(m, dev, controller.Config{AckTimeoutMs: 800, EchoJointState: true})
	ctl.Start()

	srv := New(Config{Host: "127.0.0.1", Path: "/ws/joint", PingIntervalMs: 1000, ClientTimeoutMs: 5000}, ctl)
	// 复用生产路由（含 /healthz），避免"测试路由与生产路由不一致"
	hs := httptest.NewServer(srv.Handler())
	t.Cleanup(func() {
		hs.Close()
		ctl.Close()
	})
	return &fixture{srv: srv, http: hs, ctl: ctl}
}

func homeCommand(t *testing.T, m *robot.Model) map[string]float64 {
	out := make(map[string]float64)
	for _, id := range m.JointOrder() {
		out[id] = m.HomePose[id]
	}
	return out
}

// ---------------------------------------------------------------------------

// spec §25 的仿真模式映射：三个 device 实现 → 前端可读的三态字符串。
// 单独测一遍，是因为这条映射决定了前端会不会提示"物理仿真 ≠ 真机标定模型"。
func TestSimulationModeFor(t *testing.T) {
	cases := map[string]string{
		"sim":    protocol.SimulationKinematic,
		"mujoco": protocol.SimulationMujoco,
		"serial": protocol.SimulationReal,
		"":       protocol.SimulationKinematic, // 未知一律当运动学，不假装是物理
	}
	for kind, want := range cases {
		if got := protocol.SimulationModeFor(kind); got != want {
			t.Errorf("SimulationModeFor(%q) = %q，期望 %q", kind, got, want)
		}
	}
}

// 接入必须立刻拿到 hello（模型真值）与当前状态，否则新页面会停在"未知"。
func TestHelloCarriesModelTruth(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")

	hello := c.recvType(t, protocol.TypeHello, 2*time.Second)
	if hello.Model == nil {
		t.Fatal("hello 必须携带 model 元数据")
	}
	if hello.Model.ID != "mearm" {
		t.Errorf("model.id = %q", hello.Model.ID)
	}
	// spec §25：hello 必须声明仿真模式，前端据此提示"当前是哪种仿真"。
	// 这个 fixture 用的是 SimDevice（纯运动学）⇒ 应为 kinematic。
	if hello.SimulationMode != protocol.SimulationKinematic {
		t.Errorf("simulation_mode = %q，期望 %q",
			hello.SimulationMode, protocol.SimulationKinematic)
	}
	want := []string{"base", "shoulder", "elbow", "gripper"}
	if len(hello.Model.JointOrder) != len(want) {
		t.Fatalf("jointOrder = %v", hello.Model.JointOrder)
	}
	for i := range want {
		if hello.Model.JointOrder[i] != want[i] {
			t.Errorf("jointOrder[%d] = %q, 期望 %q", i, hello.Model.JointOrder[i], want[i])
		}
	}
	if len(hello.Model.Limits) != 4 || len(hello.Model.Calibration) != 4 {
		t.Fatalf("limits=%d calibration=%d, 期望各 4", len(hello.Model.Limits), len(hello.Model.Calibration))
	}
	// 标定的通道映射必须与 docs/serial-v1.md §2 一致（S7=肩 / S8=肘）
	chByJoint := map[string]int{}
	for _, row := range hello.Model.Calibration {
		chByJoint[row.JointID] = row.Channel
	}
	if chByJoint["shoulder"] != 7 || chByJoint["elbow"] != 8 || chByJoint["base"] != 9 || chByJoint["gripper"] != 6 {
		t.Errorf("通道映射错误: %v", chByJoint)
	}
	if hello.Device != "sim" {
		t.Errorf("device = %q, 期望 sim", hello.Device)
	}

	st := c.recvType(t, protocol.TypeJointState, 2*time.Second)
	if len(st.Joints) != 4 {
		t.Fatalf("初始 joint_state = %v", st.Joints)
	}
}

// 完整往返：下发关节命令 → sim 受理 → STATE 回推 → joint_state 收敛到命令值。
func TestJointCommandRoundTrip(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	c.recvType(t, protocol.TypeJointState, 2*time.Second)

	target := homeCommand(t, f.ctl.Model())
	target["shoulder"] = 20.8
	body, _ := json.Marshal(protocol.ClientMessage{
		Version: protocol.Version, Type: protocol.TypeJointCommand,
		Timestamp: time.Now().UnixMilli(), Seq: 1, Joints: target,
	})
	c.send(t, string(body))

	// 收敛：收到肩角 ≈20.8 的状态（JR 量化 1 位小数，容差 0.1）；
	// 并且必须**先经过中间态**才到位 —— 否则就是等值回显而非真实逼近。
	sawIntermediate := false
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		m := c.recvType(t, protocol.TypeJointState, time.Until(deadline))
		v, ok := m.Joints["shoulder"]
		if !ok {
			continue
		}
		if v >= 20.7 && v <= 20.9 {
			if !sawIntermediate {
				t.Error("直接到位、从未出现中间态 —— 说明不是有限角速度逼近")
			}
			if got := m.Joints["elbow"]; got < 112.5 || got > 112.7 {
				t.Errorf("肘角应保持在 HOME 112.62 附近，实际 %.4f", got)
			}
			// 其余关节不得漂移
			if b := m.Joints["base"]; b < -1e-6 || b > 1e-6 {
				t.Errorf("底座应保持 0，实际 %.6f", b)
			}
			if g := m.Joints["gripper"]; g < 49.9 || g > 50.1 {
				t.Errorf("夹爪应保持 50，实际 %.4f", g)
			}
			return
		}
		if v > 1 && v < 20 {
			sawIntermediate = true
		}
	}
	t.Error("未在 3s 内收敛到 shoulder=20.8")
}

// 越界命令必须被同步拒绝，并且**不产生任何位置变化**。
func TestJointCommandLimitRejected(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	c.recvType(t, protocol.TypeJointState, 2*time.Second)

	bad := homeCommand(t, f.ctl.Model())
	bad["elbow"] = 95 // 真机不可达区间
	body, _ := json.Marshal(protocol.ClientMessage{
		Version: protocol.Version, Type: protocol.TypeJointCommand, Joints: bad,
	})
	c.send(t, string(body))

	m := c.recvType(t, protocol.TypeError, 2*time.Second)
	if m.Code != protocol.CodeJointLimit {
		t.Errorf("错误码 = %q, 期望 %q", m.Code, protocol.CodeJointLimit)
	}
	if !strings.Contains(m.Message, "elbow") || !strings.Contains(m.Message, "108.44") {
		t.Errorf("错误文案 = %q, 应含关节名与限位", m.Message)
	}
}

func TestVersionMismatchRejected(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	c.recvType(t, protocol.TypeJointState, 2*time.Second)

	body, _ := json.Marshal(protocol.ClientMessage{
		Version: 99, Type: protocol.TypeJointCommand, Joints: homeCommand(t, f.ctl.Model()),
	})
	c.send(t, string(body))
	m := c.recvType(t, protocol.TypeError, 2*time.Second)
	if m.Code != protocol.CodeVersion {
		t.Errorf("错误码 = %q, 期望 %q", m.Code, protocol.CodeVersion)
	}
}

func TestPingPong(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	c.recvType(t, protocol.TypeJointState, 2*time.Second)

	body, _ := json.Marshal(protocol.ClientMessage{Version: protocol.Version, Type: protocol.TypePing})
	c.send(t, string(body))
	m := c.recvType(t, protocol.TypePong, 2*time.Second)
	if m.Type != protocol.TypePong {
		t.Errorf("type = %q, 期望 pong", m.Type)
	}
}

func TestBadMessage(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	c.recvType(t, protocol.TypeJointState, 2*time.Second)

	c.send(t, "{ 这不是 JSON")
	m := c.recvType(t, protocol.TypeError, 2*time.Second)
	if m.Code != protocol.CodeBadMessage {
		t.Errorf("错误码 = %q, 期望 %q", m.Code, protocol.CodeBadMessage)
	}
}

// 设备侧自主变化（摇杆 / 红外 / 手拧）这条链的**端到端**验证：
//
//	device_command{"JOY 7 1023"} → sim 受理 → `# SERVO …` 异步上报
//	  → controller 解析成 ReplyServo → 广播 **origin=device** 的 joint_state
//
// 为什么这条 e2e 必须存在：它是"下位机被外部手段改动 → 上位机界面跟随"在
// **没有真机**时唯一可复现的入口。摇杆拨不了，就用调试直通把同一行指令塞进链路
// 末端，走的是与固件完全相同的 `# SERVO` 回执路径。
//
// 这条测试还顺带钉住一个极易回归的约束：直通指令**不得**参与 ACK 门控。
// `JOY` 只回 `OK JOY`，不会回 `OK`；若控制器把它当交互指令挂上在途状态，
// 800ms 后必然超时报错。所以这里遇到任何 error 消息就直接判失败。
func TestDeviceCommandPublishesDeviceOrigin(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c := dialWS(t, f.http.URL, "/ws/joint")
	c.recvType(t, protocol.TypeHello, 2*time.Second)
	home := c.recvType(t, protocol.TypeJointState, 2*time.Second).Joints["shoulder"]

	body, _ := json.Marshal(protocol.ClientMessage{
		Version: protocol.Version, Type: protocol.TypeDeviceCommand,
		Timestamp: time.Now().UnixMilli(), Line: "JOY 7 1023",
	})
	c.send(t, string(body))

	sawDeviceOrigin := false
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		raw := c.recv(t, time.Until(deadline))
		var m protocol.ServerMessage
		if err := json.Unmarshal([]byte(raw), &m); err != nil {
			t.Fatalf("上行消息不是合法 JSON: %q", raw)
		}
		switch m.Type {
		case protocol.TypeError:
			t.Fatalf("直通指令不应产生错误（是否被卷进 ACK 门控？）: %s %s", m.Code, m.Message)
		case protocol.TypeJointState:
			if m.Origin != protocol.OriginDevice {
				// 命令来源的状态帧不该出现在这条链上；出现了说明上报走错了通道
				// （`STATE` 而非 `# SERVO`），界面会把命令侧拖向实际位置。
				t.Errorf("joint_state 的 origin = %q, 期望 %q", m.Origin, protocol.OriginDevice)
				continue
			}
			sawDeviceOrigin = true
			v, ok := m.Joints["shoulder"]
			if !ok {
				t.Errorf("origin=device 的状态缺少 shoulder：%v", m.Joints)
				continue
			}
			if math.Abs(v-home) > 0.5 {
				return // 肩角已离开 HOME ⇒ 链路打通
			}
		}
	}
	if !sawDeviceOrigin {
		t.Fatalf("未收到 origin=%q 的 joint_state", protocol.OriginDevice)
	}
	t.Errorf("收到 origin=%q 状态但肩角仍停在 HOME %.4f（未跟随）", protocol.OriginDevice, home)
}

// 多客户端：状态必须广播给所有人（数字孪生页面 + 观测页面同时在线）。
func TestBroadcastToMultipleClients(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	c1 := dialWS(t, f.http.URL, "/ws/joint")
	c2 := dialWS(t, f.http.URL, "/ws/joint")
	for _, c := range []*wsClient{c1, c2} {
		c.recvType(t, protocol.TypeHello, 2*time.Second)
		c.recvType(t, protocol.TypeJointState, 2*time.Second)
	}

	target := homeCommand(t, f.ctl.Model())
	target["shoulder"] = 20.8
	body, _ := json.Marshal(protocol.ClientMessage{
		Version: protocol.Version, Type: protocol.TypeJointCommand, Joints: target,
	})
	c1.send(t, string(body))

	// 两个客户端都应收到状态帧（c2 只是旁观者）
	c2.recvType(t, protocol.TypeJointState, 3*time.Second)
	// c1 也必须收到
	found := false
	deadline := time.Now().Add(2 * time.Second)
	for time.Now().Before(deadline) {
		m := c1.recvType(t, protocol.TypeJointState, time.Until(deadline))
		if v, ok := m.Joints["shoulder"]; ok && v > 1 {
			found = true
			break
		}
	}
	if !found {
		t.Error("下发命令的客户端自身也应收到状态广播")
	}
}

// 健康检查端点：e2e / 编排脚本靠它等待服务就绪。
func TestHealthEndpoint(t *testing.T) {
	f := newFixture(t, device.DefaultSimTuning())
	resp, err := httpGet(f.http.URL + "/healthz")
	if err != nil {
		t.Fatalf("健康检查失败: %v", err)
	}
	var body map[string]any
	if err := json.Unmarshal([]byte(resp), &body); err != nil {
		t.Fatalf("健康检查返回不是 JSON: %q", resp)
	}
	if body["ok"] != true {
		t.Errorf("ok = %v", body["ok"])
	}
	if body["device"] != "sim" {
		t.Errorf("device = %v", body["device"])
	}
}

func truncate(s string, n int) string {
	if len(s) <= n {
		return s
	}
	return s[:n] + "…"
}

func httpGet(url string) (string, error) {
	rest := strings.TrimPrefix(url, "http://")
	slash := strings.Index(rest, "/")
	if slash < 0 {
		return "", fmt.Errorf("URL 缺少路径: %q", url)
	}
	host, path := rest[:slash], rest[slash:]
	conn, err := net.DialTimeout("tcp", host, 3*time.Second)
	if err != nil {
		return "", err
	}
	defer conn.Close()
	fmt.Fprintf(conn, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", path, host)
	all, err := io.ReadAll(conn)
	if err != nil {
		return "", err
	}
	parts := strings.SplitN(string(all), "\r\n\r\n", 2)
	if len(parts) < 2 {
		return "", fmt.Errorf("响应格式异常: %q", string(all))
	}
	return parts[1], nil
}
