package tcpserver

// server_test.go —— 传输层：连接生命周期 / 一行一答 / 异常输入 / 多客户端 / 关闭。
//
// 端口一律用 `Port: 0`（OS 分配），避免与开发机上正在跑的实例抢端口 ——
// 那会造出"测试在本地绿、在别人机器上红"的环境差异型失败。

import (
	"bufio"
	"encoding/json"
	"net"
	"strings"
	"sync"
	"testing"
	"time"

	"armpilot/backend/internal/robot"
)

// startServer 起一个监听在随机端口的服务，并注册 t.Cleanup 关闭。
func startServer(t *testing.T, h *Handler, cfg Config) *Server {
	t.Helper()
	cfg.Host = "127.0.0.1"
	cfg.Port = 0
	s := New(cfg, h)
	if err := s.Listen(); err != nil {
		t.Fatalf("监听失败: %v", err)
	}
	done := make(chan struct{})
	go func() {
		defer close(done)
		_ = s.Serve()
	}()
	t.Cleanup(func() {
		_ = s.Close()
		select {
		case <-done:
		case <-time.After(3 * time.Second):
			// ★ 这条超时本身就是判据：Close 若因为"客户端还连着"而挂住，
			//   这里会红 —— 那正是后端 Ctrl+C 关不掉的那类故障。
			t.Error("Close 之后 Serve 未在 3s 内返回（很可能被活动连接挂住了）")
		}
	})
	return s
}

func dial(t *testing.T, s *Server) net.Conn {
	t.Helper()
	conn, err := net.DialTimeout("tcp", s.BoundAddr(), 2*time.Second)
	if err != nil {
		t.Fatalf("连接 %s 失败: %v", s.BoundAddr(), err)
	}
	return conn
}

// roundTrip 发一行、读一行应答。
func roundTrip(t *testing.T, conn net.Conn, line string) Result {
	t.Helper()
	if _, err := conn.Write([]byte(line + "\n")); err != nil {
		t.Fatalf("写失败: %v", err)
	}
	_ = conn.SetReadDeadline(time.Now().Add(3 * time.Second))
	raw, err := bufio.NewReader(conn).ReadString('\n')
	if err != nil {
		t.Fatalf("读应答失败: %v", err)
	}
	var res Result
	if err := json.Unmarshal([]byte(raw), &res); err != nil {
		t.Fatalf("应答不是合法 JSON（%q）: %v", raw, err)
	}
	return res
}

func newTestHandler(t *testing.T) *Handler {
	t.Helper()
	m, err := robot.Load(modelPath)
	if err != nil {
		t.Fatalf("加载模型失败: %v", err)
	}
	g, err := robot.LoadGeom(modelPath)
	if err != nil {
		t.Fatalf("求导几何失败: %v", err)
	}
	return NewHandler(newFakeArm(m), m, g)
}

// TestServer_AnswersEveryLine 每条命令恰好一行应答（JSON Lines）。
func TestServer_AnswersEveryLine(t *testing.T) {
	s := startServer(t, newTestHandler(t), Config{})
	conn := dial(t, s)
	defer conn.Close()

	lines := []string{
		`{"cmd":"gripper","action":"open"}`,
		`{"cmd":"gripper","action":"close"}`,
		`{"cmd":"move","axis":"x","direction":"+","step":5}`,
	}
	for _, l := range lines {
		if res := roundTrip(t, conn, l); !res.OK {
			t.Errorf("%s → 期望 ok=true，实际 error=%q", l, res.Error)
		}
	}
}

// TestServer_SurvivesInvalidJson 非法 JSON 只让**这一条**失败，连接继续可用。
func TestServer_SurvivesInvalidJson(t *testing.T) {
	s := startServer(t, newTestHandler(t), Config{})
	conn := dial(t, s)
	defer conn.Close()

	if res := roundTrip(t, conn, `{"cmd":`); res.OK || res.Error != "invalid json" {
		t.Errorf("非法 JSON 应报 invalid json，实际 %+v", res)
	}
	if res := roundTrip(t, conn, `{"cmd":"nope"}`); res.OK {
		t.Error("未知命令应当失败")
	}
	// 关键：之后的合法命令照常工作。
	if res := roundTrip(t, conn, `{"cmd":"gripper","action":"open"}`); !res.OK {
		t.Errorf("畸形输入之后应当照常工作，实际 error=%q", res.Error)
	}
}

// TestServer_SurvivesAbruptDisconnect 客户端说一半就断：服务不崩、不影响别人。
func TestServer_SurvivesAbruptDisconnect(t *testing.T) {
	s := startServer(t, newTestHandler(t), Config{})
	conn := dial(t, s)
	_, _ = conn.Write([]byte(`{"cmd":"move","axis":"x","dir`)) // 半行
	_ = conn.Close()

	// 另一个客户端必须照样能用。
	c2 := dial(t, s)
	defer c2.Close()
	if res := roundTrip(t, c2, `{"cmd":"gripper","action":"open"}`); !res.OK {
		t.Errorf("前一客户端异常断开后，新客户端应当正常，实际 error=%q", res.Error)
	}
}

// TestServer_RejectsOversizedLine 超长行只断那一条连接，不动进程、不影响别人。
func TestServer_RejectsOversizedLine(t *testing.T) {
	s := startServer(t, newTestHandler(t), Config{MaxLineBytes: 128})
	conn := dial(t, s)
	defer conn.Close()

	res := roundTrip(t, conn, strings.Repeat("a", 4096))
	if res.OK || res.Error == "" || !strings.Contains(res.Error, "too long") {
		t.Errorf("超长行应报 too long，实际 %+v", res)
	}
	// 该连接随后被服务端断开。
	_ = conn.SetReadDeadline(time.Now().Add(2 * time.Second))
	buf := make([]byte, 1)
	if n, _ := conn.Read(buf); n != 0 {
		t.Error("超长行之后服务端应当关闭该连接")
	}

	c2 := dial(t, s)
	defer c2.Close()
	if res := roundTrip(t, c2, `{"cmd":"gripper","action":"close"}`); !res.OK {
		t.Errorf("超长行不影响其它连接，实际 error=%q", res.Error)
	}
}

// TestServer_MultipleClientsConcurrently 多客户端同时发命令（§18）：
// 不 panic、不丢步、每个都能收到自己的应答。
func TestServer_MultipleClientsConcurrently(t *testing.T) {
	s := startServer(t, newTestHandler(t), Config{})

	const clients = 4
	// ⚠️ 累计位移必须算：4 个客户端共享同一台臂，clients×perClient×step 会累加。
	//    +X 走 40mm 时 elbow 会掉到 105.4°（下限 108.44）⇒ JOINT_LIMIT。
	//    8mm 时 elbow ≈ 112.25°，仍在限位内（手算过两支解）。
	const perClient = 2
	var wg sync.WaitGroup
	errCh := make(chan error, clients*perClient)

	for i := 0; i < clients; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			conn := dial(t, s)
			defer conn.Close()
			for j := 0; j < perClient; j++ {
				// +X：累计 8mm 内 elbow 仍在限位内（手算过两支解）。
				// ⚠️ 别换成 +Z：HOME 的 elbow 距下限只有 4.18°，朝上走几毫米就撞限位。
				res := roundTrip(t, conn, `{"cmd":"move","axis":"x","direction":"+","step":1}`)
				if !res.OK {
					errCh <- errString(res.Error)
				}
			}
		}()
	}
	wg.Wait()
	close(errCh)
	for err := range errCh {
		t.Errorf("并发客户端失败: %v", err)
	}
}

// TestServer_BoundAddrBeforeListen 未监听时地址为空（供调用方判断状态）。
func TestServer_BoundAddrBeforeListen(t *testing.T) {
	s := New(Config{Host: "127.0.0.1", Port: 0}, newTestHandler(t))
	if got := s.BoundAddr(); got != "" {
		t.Errorf("未监听时 BoundAddr 应为空，实际 %q", got)
	}
	// Addr 是**配置**地址，与 BoundAddr 不同 —— 两者语义必须分得清。
	if got := s.Addr(); got != "127.0.0.1:0" {
		t.Errorf("Addr() = %q，期望 127.0.0.1:0", got)
	}
}

type errString string

func (e errString) Error() string { return string(e) }
