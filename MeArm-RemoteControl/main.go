// arm-web：基于 Go 的机械臂控制服务（Web + WebSocket 双 3D 摇杆）。
//
// 两种运行模式，由 `-mode` 参数或配置文件的 `mode` 决定（`start.bat` 负责选）：
//
// # serial（默认；原有真机链路，行为与引入本模式前逐字节一致）
//
//	arm-device (串口 COMx / 115200 8N1)
//	   ▲      ▼
//	[ Serial 管理 + 自动重连 + ACK 门控 ]
//	   ▲      ▼
//	[   Hub 广播总线（设备回显 → 多端）  ]
//	   ▲                    ▲
//	[ TCP 局域网透传 ]     [ Web + WebSocket 3D 摇杆 ]
//
// # network（新增；TCP Client → MeArm-3D，不打开串口）
//
//	MeArm-3D TCP 控制接口（JSON Lines，默认 127.0.0.1:9100）
//	   ▲      ▼
//	[ netlink.Client：指数退避重连 + 单写泵 + 每轴 latest-wins ]
//	   ▲      ▼
//	[   Hub 广播总线  ]
//	   ▲
//	[ Web + WebSocket 3D 摇杆 ]
//
// 两条链路共用**同一套网页**与**同一个 `link.Link` 接口**；
// 网络模式不构造 Serial、不开局域网透传端口，串口模式不构造任何 TCP Client
// —— 两种模式在装配层就完全分开，互不影响。
package main

import (
	"embed"
	"flag"
	"fmt"
	"io/fs"
	"log"
	"os"
	"os/signal"
	"syscall"

	"arm-web/internal/config"
	"arm-web/internal/hub"
	"arm-web/internal/link"
	"arm-web/internal/netlink"
	"arm-web/internal/protocol"
	"arm-web/internal/serial"
	"arm-web/internal/tcp"
	"arm-web/internal/web"
)

// webFS 在编译期把 web/static 全部嵌入二进制，使产物成为自包含单文件，
// 运行时不再依赖磁盘上的 web/static 目录（“一键编译 web 文件”即指此步）。
//
//go:embed all:web/static
var webFS embed.FS

func main() {
	cfgPath := flag.String("c", "config.yaml", "配置文件路径 (YAML)")
	modeFlag := flag.String("mode", "", "覆盖运行模式：serial（真机串口，默认）/ network（TCP 连 MeArm-3D）")
	flag.Parse()

	cfg, err := config.Load(*cfgPath)
	if err != nil {
		log.Fatalf("[fatal] %v", err)
	}
	if *modeFlag != "" {
		// 命令行覆盖后必须复检：非法值要在装配之前就失败，而不是半路才发现。
		if err := cfg.SetMode(*modeFlag); err != nil {
			log.Fatalf("[fatal] %v", err)
		}
	}
	setupLog(cfg.LogLevel)

	log.Printf("========== arm-web 启动 ==========")
	log.Printf("配置: %s", *cfgPath)
	log.Printf("运行模式: %s", cfg.Mode)

	// 广播总线：设备/服务端状态 → 所有订阅者（Web / TCP 透传）
	h := hub.New()

	// ---- 控制通道：二选一装配（另一条链路的代码不激活）------------------------
	var (
		lk      link.Link
		wire    func(*web.Server) // web 就绪后完成状态回调接线 + 启动后台连接
		cleanup func()
	)

	switch cfg.Mode {
	case config.ModeNetwork:
		cli, aerr := buildNetlinkClient(cfg)
		if aerr != nil {
			log.Fatalf("[fatal] %v", aerr)
		}
		lk = link.NewNetwork(cli, cfg.Network.ServoIDs)
		cleanup = cli.Close
		wire = func(ws *web.Server) {
			// 三条回调都必须在 Start **之前**注册：连接是立刻建立的，
			// 建立后再注册会漏掉"首次连上"与"首帧基准"。
			cli.OnStatus(func(connected bool, errMsg string) {
				ws.BroadcastStatus(connected, errMsg, false, "")
			})
			cli.OnError(func(msg string) { ws.BroadcastError(msg) })
			cli.OnState(func(st *netlink.State) {
				ws.BroadcastNetState(st.Servo, st.TCP, st.Device)
			})
			cli.Start()
		}
		log.Printf("网络模式: 目标 %s（JSON Lines；`state` 建基准 → `servo` 增量下发）", cli.Addr())
		log.Printf("网络模式: **不打开串口**、不开局域网透传端口")
		log.Printf("网络模式: 舵机接线表 servo_ids=%v（TCP 编号 1..N → arm-device 舵机 id）",
			cfg.Network.ServoIDs)
		log.Printf("网络模式: 摇杆速度 %.0f..%.0f°/s · 死区 %.1f° · 积分 tick %dms · 退避 %d..%dms",
			cfg.Network.MinSpeedDegPerS, cfg.Network.MaxSpeedDegPerS, cfg.Network.DeadbandDeg,
			cfg.Network.TickMs, cfg.Network.ReconnectMinMs, cfg.Network.ReconnectMaxMs)

	default: // config.ModeSerial
		log.Printf("串口: %s @ %d %dN%d (%c)", cfg.Serial.Port, cfg.Serial.Baud,
			cfg.Serial.DataBits, cfg.Serial.StopBits, upcase(cfg.Serial.Parity))
		log.Printf("摇杆映射: 左 X->S%d 左 Y->S%d | 右 X->S%d 右 Y->S%d",
			cfg.Joystick.LXServo, cfg.Joystick.LYServo, cfg.Joystick.RXServo, cfg.Joystick.RYServo)

		if cfg.LogLevel == "debug" {
			serial.SetDebug(true)
		}
		ser := serial.Open(serial.Config{
			Port:            cfg.Serial.Port,
			Baud:            cfg.Serial.Baud,
			DataBits:        cfg.Serial.DataBits,
			StopBits:        cfg.Serial.StopBits,
			Parity:          cfg.Serial.Parity,
			ReconnectSec:    cfg.Serial.ReconnectSec,
			MinIntervalMs:   cfg.Serial.MinIntervalMs,
			AckTimeoutMs:    cfg.Serial.AckTimeoutMs,
			ConnectSettleMs: cfg.Serial.ConnectSettleMs,
		})
		cleanup = func() { ser.Close() }

		// 把设备回显广播给所有订阅者（Web / TCP）
		ser.SetLineHandler(func(line string) { h.Broadcast(line) })

		lk = link.NewSerial(ser, buildSerialAxisMap(cfg.Joystick))
		wire = func(ws *web.Server) {
			// 串口连接状态 / 通讯失败 -> 广播给所有 Web 客户端
			ser.SetStatusHandler(func(connected bool, serialErr string, commErr bool, commErrMsg string) {
				ws.BroadcastStatus(connected, serialErr, commErr, commErrMsg)
			})
		}

		// TCP 局域网转发（**仅串口模式**：它转发的是 arm-device 文本指令，
		// 网络模式下没有串口可转发，开着只会误导）。
		if cfg.TCP.Enabled {
			ts := tcp.New(tcp.Config{Host: cfg.TCP.Host, Port: cfg.TCP.Port}, ser, h)
			go func() {
				if err := ts.Listen(); err != nil {
					log.Printf("[tcp] 监听失败: %v", err)
				}
			}()
			log.Printf("提示：局域网设备可 telnet <本机IP> %d 下发 arm-device 指令", cfg.TCP.Port)
		}
	}
	defer cleanup()

	// ---- Web + WebSocket ------------------------------------------------------
	staticFS, err := fs.Sub(webFS, "web/static")
	if err != nil {
		log.Fatalf("[fatal] 内嵌 web 资源取出失败: %v", err)
	}
	ws := web.New(cfg.Web, lk, h, cfg.Network.ServoIDs, staticFS)
	wire(ws)

	if cfg.Web.Enabled {
		go func() {
			if err := ws.ListenAndServe(); err != nil {
				log.Printf("[web] 服务失败: %v", err)
			}
		}()
		log.Printf("提示：浏览器打开 http://<本机IP>:%d 进行控制", cfg.Web.Port)
	}

	// ---- 等待退出 -------------------------------------------------------------
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	<-sig
	log.Printf("========== arm-web 关闭 ==========")
}

// buildSerialAxisMap 串口模式的摇杆映射（直接用 arm-device 的舵机 id）。
func buildSerialAxisMap(joy config.JoystickConfig) protocol.AxisMap {
	return protocol.AxisMap{
		LXServo:  joy.LXServo,
		LYServo:  joy.LYServo,
		RXServo:  joy.RXServo,
		RYServo:  joy.RYServo,
		InvLX:    joy.InvLX,
		InvLY:    joy.InvLY,
		InvRX:    joy.InvRX,
		InvRY:    joy.InvRY,
		DeadFrac: protocol.DeadFracFromDeg(joy.DeadbandDeg),
	}
}

// buildNetlinkClient 网络模式的 TCP 客户端（含摇杆接线表派生）。
func buildNetlinkClient(cfg *config.Config) (*netlink.Client, error) {
	axis, err := deriveAxisMap(cfg.Joystick, cfg.Network)
	if err != nil {
		return nil, err
	}
	return netlink.New(netlink.Config{
		Host:             cfg.Network.Host,
		Port:             cfg.Network.Port,
		Axis:             axis,
		AngleMin:         cfg.Network.AngleMin,
		AngleMax:         cfg.Network.AngleMax,
		ConnectTimeoutMs: cfg.Network.ConnectTimeoutMs,
		RequestTimeoutMs: cfg.Network.RequestTimeoutMs,
		TickMs:           cfg.Network.TickMs,
		MaxTickMs:        cfg.Network.MaxTickMs,
		ReconnectMinMs:   cfg.Network.ReconnectMinMs,
		ReconnectMaxMs:   cfg.Network.ReconnectMaxMs,
		QueueSize:        cfg.Network.QueueSize,
		AutoSync:         cfg.Network.AutoSyncEnabled(),
		FallbackAngle:    cfg.Network.FallbackAngle,
	}), nil
}

// deriveAxisMap 把 `joystick` 段的 arm-device 舵机 id 换算成 MeArm-3D 的 TCP 编号。
//
// 两个配置段用的是**两套编号**：`joystick.lx_servo = 9` 是 arm-device 的舵机 id，
// 而 MeArm-3D 的 `servo` 命令要的是 1..N。换算依据是 `network.servo_ids`
// 这张接线表的**位次**（该表来自 `Model.JointOrder()`，见配置注释）。
//
// 找不到就**明确报错**：静默兜底会得到"推左摇杆却动了别的关节"这种最难查的故障。
//
// ⚠️ 方向标志必须过一次 `protocol.NetInvertFor`：`invert_*` 里含固件的方向
//    差异补偿，而 TCP 链路没有固件这一层，照搬会让两种模式的推杆方向相反。
//    详见该函数注释（含推导与真值出处）。
func deriveAxisMap(joy config.JoystickConfig, net config.NetworkConfig) (netlink.AxisMap, error) {
	m := netlink.DefaultAxisMap()
	m.DeadFrac = protocol.DeadFracFromDeg(net.DeadbandDeg)
	m.MinSpeedDegPerS = net.MinSpeedDegPerS
	m.MaxSpeedDegPerS = net.MaxSpeedDegPerS

	pairs := []struct {
		key string
		arm int
		dst *int
		inv *bool
		src bool
	}{
		{"joystick.lx_servo", joy.LXServo, &m.LXServo, &m.InvLX, joy.InvLX},
		{"joystick.ly_servo", joy.LYServo, &m.LYServo, &m.InvLY, joy.InvLY},
		{"joystick.rx_servo", joy.RXServo, &m.RXServo, &m.InvRX, joy.InvRX},
		{"joystick.ry_servo", joy.RYServo, &m.RYServo, &m.InvRY, joy.InvRY},
	}
	for _, p := range pairs {
		idx := net.ServoIndex(p.arm)
		if idx == 0 {
			return m, fmt.Errorf(
				"%s = %d 不在 network.servo_ids = %v 里：无法换算成 MeArm-3D 的舵机编号",
				p.key, p.arm, net.ServoIDs)
		}
		*p.dst = idx
		*p.inv = protocol.NetInvertFor(p.arm, p.src)
	}
	return m, nil
}

func setupLog(level string) {
	// 简单级别过滤：低于设定级别的日志不打印（此处全部输出，级别存入结构化前缀）
	_ = level
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)
}

func upcase(s string) byte {
	if len(s) == 0 {
		return 'N'
	}
	return s[0]
}
