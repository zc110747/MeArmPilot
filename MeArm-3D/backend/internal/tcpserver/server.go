package tcpserver

// TCP JSON Lines 服务端：监听 / 连接生命周期 / 读写。
//
// 协议转换在 `protocol.go`；本文件只管传输，两者刻意分开 ——
// "怎么收一行"与"收到什么"不该互相污染。
//
// ## 三条硬要求（方案 §17 / §18 / §19）
//
//  1. **不让后端崩**：非法 JSON / 错误参数 / 超长数据 / 客户端异常断开，
//     影响范围都只到"这一条命令 / 这一个连接"。每个连接 goroutine 都有
//     `recover` —— goroutine 里的 panic 会带走**整个进程**，而"某个客户端
//     发了怪东西"绝不该有这个后果。
//  2. **允许多客户端**：每个连接一个 goroutine；命令执行由 `Handler.mu` 串行化。
//  3. **与 HTTP / WebSocket 并行**：本服务只多开一个端口，不替换任何现有入口。

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"net"
	"sync"
	"time"
)

// defaultMaxLineBytes 单行上限。
const defaultMaxLineBytes = 64 * 1024

// writeTimeout 单次写应答的超时。
//
// 客户端连着但不读（或对端窗口满）时，没有写超时的 `Write` 会一直阻塞，
// 把 goroutine 和 fd 一起占住 —— 那是一类"慢慢死"的故障。
const writeTimeout = 5 * time.Second

// Config TCP 服务运行参数。
type Config struct {
	Host string
	Port int
	// MaxLineBytes 单行上限（≤0 用默认 64KB）。超长 → 回一条 error 并断开该连接。
	MaxLineBytes int
	// ReadTimeoutMs 单次读行超时（0 = 不设）。用来回收僵死连接。
	ReadTimeoutMs int
}

// Server TCP 服务端。
type Server struct {
	cfg Config
	h   *Handler

	mu     sync.Mutex
	ln     net.Listener
	conns  map[net.Conn]struct{} // 活动连接，供 Close 时逐个掐断
	closed bool
	wg     sync.WaitGroup
}

// New 构造服务端（还不监听）。
func New(cfg Config, h *Handler) *Server {
	return &Server{cfg: cfg, h: h, conns: map[net.Conn]struct{}{}}
}

// Addr 监听地址（host:port 规范化后的形式）。
func (s *Server) Addr() string {
	host := s.cfg.Host
	if host == "" {
		host = "0.0.0.0"
	}
	return net.JoinHostPort(host, fmt.Sprintf("%d", s.cfg.Port))
}

// Listen 只完成监听（不阻塞），供调用方在 `Serve` 之前拿到**实际**端口。
//
// `Port: 0` 时由 OS 分配端口 —— 测试靠这个避免端口冲突，
// 再用 `BoundAddr()` 取回真实地址。
// isClosed 是否已请求关闭。
func (s *Server) isClosed() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.closed
}

func (s *Server) Listen() error {
	ln, err := net.Listen("tcp", s.Addr())
	if err != nil {
		return fmt.Errorf("TCP 监听失败: %w", err)
	}
	s.mu.Lock()
	s.ln = ln
	s.mu.Unlock()
	log.Printf("[tcp] 监听 %s（JSON Lines：一行一条命令）", ln.Addr())
	return nil
}

// Serve 接受连接（阻塞）。必须先 `Listen`。
//
// 返回的 error 只表示**监听本身**出问题（端口被占 / 地址非法）；
// 单个连接的错误一律记日志，不向上冒泡。
func (s *Server) Serve() error {
	s.mu.Lock()
	ln := s.ln
	s.mu.Unlock()
	if ln == nil {
		return fmt.Errorf("Serve 前必须先 Listen")
	}

	for {
		conn, err := ln.Accept()
		if err != nil {
			s.mu.Lock()
			closed := s.closed
			s.mu.Unlock()
			if closed || errors.Is(err, net.ErrClosed) {
				return nil
			}
			// 临时性失败（fd 耗尽等）重试；否则交给调用方决定。
			var ne net.Error
			if errors.As(err, &ne) && ne.Temporary() {
				log.Printf("[tcp] accept 临时失败，稍后重试: %v", err)
				time.Sleep(50 * time.Millisecond)
				continue
			}
			return fmt.Errorf("TCP accept 失败: %w", err)
		}
		s.wg.Add(1)
		go func() {
			defer s.wg.Done()
			s.serve(conn)
		}()
	}
}

// ListenAndServe = Listen + Serve。
func (s *Server) ListenAndServe() error {
	if err := s.Listen(); err != nil {
		return err
	}
	return s.Serve()
}

// BoundAddr 实际监听地址（`Listen` 之后才有意义；未监听时返回空串）。
func (s *Server) BoundAddr() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.ln == nil {
		return ""
	}
	return s.ln.Addr().String()
}

// Close 停止监听，并等待已建立的连接收尾。
//
// ⚠️ 必须**先掐掉活动连接**再 `wg.Wait()`：只关 listener 的话，
// 那些 goroutine 还阻塞在 `Scan`（客户端没断、也没说话），`Wait` 会一直挂着
// —— 表现为"后端收到 Ctrl+C 却关不掉"。这个坑只在"有客户端连着不动"时出现，
// 单跑 accept 循环永远测不出来。
func (s *Server) Close() error {
	s.mu.Lock()
	s.closed = true
	ln := s.ln
	live := make([]net.Conn, 0, len(s.conns))
	for c := range s.conns {
		live = append(live, c)
	}
	s.mu.Unlock()

	var err error
	if ln != nil {
		err = ln.Close()
	}
	for _, c := range live {
		_ = c.Close()
	}
	s.wg.Wait()
	return err
}

// serve 处理一个连接：循环读行 → 执行 → 写应答。
func (s *Server) serve(conn net.Conn) {
	s.mu.Lock()
	s.conns[conn] = struct{}{}
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		delete(s.conns, conn)
		s.mu.Unlock()
		_ = conn.Close()
	}()
	if s.isClosed() {
		return
	}
	// ★ panic 隔离：没有它，一个畸形输入就能把整个后端进程带走。
	defer func() {
		if r := recover(); r != nil {
			log.Printf("[tcp] %s 处理时发生 panic（已隔离，不影响其它连接）: %v", conn.RemoteAddr(), r)
		}
	}()

	addr := conn.RemoteAddr().String()
	log.Printf("[tcp] 客户端连接 %s", addr)
	defer log.Printf("[tcp] 客户端断开 %s", addr)

	max := s.cfg.MaxLineBytes
	if max <= 0 {
		max = defaultMaxLineBytes
	}
	sc := bufio.NewScanner(conn)
	sc.Buffer(make([]byte, 0, 4096), max)

	for {
		if t := s.cfg.ReadTimeoutMs; t > 0 {
			_ = conn.SetReadDeadline(time.Now().Add(time.Duration(t) * time.Millisecond))
		}
		if !sc.Scan() {
			break
		}
		writeResult(conn, s.h.Execute(sc.Text()))
	}

	if err := sc.Err(); err != nil {
		if errors.Is(err, bufio.ErrTooLong) {
			writeResult(conn, Result{OK: false, Error: fmt.Sprintf("line too long (max %d bytes)", max)})
			log.Printf("[tcp] %s 单行超过 %d 字节，已断开", addr, max)
			return
		}
		// 客户端直接断开（ECONNRESET / EOF）是**正常**的，不记错误级别。
		log.Printf("[tcp] %s 读结束: %v", addr, err)
	}
}

// writeResult 写一行 JSON 应答。
func writeResult(conn net.Conn, res Result) {
	raw, err := json.Marshal(res)
	if err != nil {
		// 结构固定，理论上不可达；但绝不能因为序列化失败把连接悬住。
		raw = []byte(`{"ok":false,"error":"internal"}`)
	}
	raw = append(raw, '\n')
	_ = conn.SetWriteDeadline(time.Now().Add(writeTimeout))
	if _, err := conn.Write(raw); err != nil {
		log.Printf("[tcp] 写应答失败: %v", err)
	}
}
