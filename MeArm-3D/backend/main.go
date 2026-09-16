// armpilot-backend：ArmPilot 数字孪生的关节级后端（Phase 8）。
//
// 架构（spec §二十二；分层图见 docs/serial-v1.md §5）：
//
//	Browser ──WebSocket(JSON)──▶ wsserver
//	                                 │
//	                            controller   ← 标定 / 限位 / ACK 门控 / latest-wins
//	                                 │
//	                              device      ← sim（内置假固件）| serial（Phase 9）
//	                                 │
//	                          JR 文本协议 ──▶ AVR
//
// 两条铁律：
//  1. **模型/标定/限位真值只有一份**，来自 robot-package/mearm-v1/model/robot.yaml（本服务启动时读入）。
//  2. **WebSocket 层不碰设备**。所有控制意图必须经 controller。
package main

import (
	"flag"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"

	"armpilot/backend/internal/cfg"
	"armpilot/backend/internal/controller"
	"armpilot/backend/internal/device"
	"armpilot/backend/internal/robot"
	"armpilot/backend/internal/wsserver"
)

func main() {
	cfgPath := flag.String("c", "config.yaml", "配置文件路径 (YAML)")
	robotID := flag.String("robot", "", "机器人 id（config/robots.yaml 的 key）；留空 = 选择器的 default")
	flag.Parse()
	if err := run(*cfgPath, *robotID); err != nil {
		log.Fatalf("[fatal] %v", err)
	}
}

func run(cfgPath, robotIDFlag string) error {
	log.SetFlags(log.LstdFlags | log.Lmicroseconds)

	c, err := cfg.Load(cfgPath)
	if err != nil {
		return err
	}
	log.Printf("========== armpilot-backend 启动 ==========")
	log.Printf("配置: %s", absOrSelf(c.Path))

	// ---- 模型真值（选择器 → 记录 → 模型）-----------------------------------
	// 优先级：命令行 `-robot` > `backend/config.yaml` 的 `robot.model_id` > 选择器的 default。
	selectorPath, err := cfg.ResolveRobotSelector(c.Robot.SelectorPath)
	if err != nil {
		return err
	}
	wantID := robotIDFlag
	if wantID == "" {
		wantID = c.Robot.ModelID
	}
	model, sel, entry, err := robot.LoadByID(selectorPath, wantID)
	if err != nil {
		return err
	}
	log.Printf("模型选择器: %s（default=%s，可选 %s）", selectorPath, sel.Default, sel.IDList())
	log.Printf("模型真值: %s", entry.ConfigPath)
	log.Printf("robot: %s (%s)  %s", model.Name, model.ID, model.Describe())
	log.Printf("仿真资产: mjcf=%s · tcpsite=%s · physics=%s（形态 %s）",
		orNone(entry.MJCFPath), entry.TCPSite, entry.PhysicsPath, entry.PhysicsKind)

	// ---- 链路末端 ----------------------------------------------------------
	var dev device.Device
	var mujocoScript string
	var mujocoPython string
	switch c.Device.Mode {
	case "sim":
		dev, err = device.NewSim(model, device.SimTuning{
			MaxServoSpeed: c.Device.Sim.MaxServoSpeed,
			LatencyMs:     c.Device.Sim.LatencyMs,
			TickMs:        c.Device.Sim.TickMs,
			EnforceLimits: c.Device.Sim.EnforceLimits,
			BootMs:        c.Device.Sim.BootMs,
		})
	case "serial":
		warm := c.Device.Serial.WarmupEnabled()
		dev, err = device.NewSerial(device.SerialConfig{
			Port:            c.Device.Serial.Port,
			Baud:            c.Device.Serial.Baud,
			DataBits:        c.Device.Serial.DataBits,
			StopBits:        c.Device.Serial.StopBits,
			Parity:          c.Device.Serial.Parity,
			ReconnectSec:    c.Device.Serial.ReconnectSec,
			AckTimeoutMs:    c.Device.Serial.AckTimeoutMs,
			ConnectSettleMs: c.Device.Serial.ConnectSettleMs,
			Warmup:          warm,
		}, model)
	case "mujoco":
		// 第三个 device 实现：链路末端是一台跑 MuJoCo 的 Python 子进程。
		// 协议与固件逐字节一致 ⇒ 上层（controller / WS / 前端）零改动。
		script, rerr := cfg.ResolveMujocoScript(c.Device.Mujoco.Script)
		if rerr != nil {
			return rerr
		}
		mujocoScript = script
		// 解释器同理：**不写死本机绝对路径**（换机器即失效）。解析顺序见
		// cfg.ResolveMujocoPython：显式配置 → ARMPILOT_MUJOCO_PYTHON → 用户目录下的
		// 隔离环境 → PATH 里的 python。
		mujocoPython = cfg.ResolveMujocoPython(c.Device.Mujoco.Python)
		dev, err = device.NewMujoco(model, device.MujocoConfig{
			Python:         mujocoPython,
			Script:         mujocoScript,
			ReportHz:       c.Device.Mujoco.ReportHz,
			PhysHz:         c.Device.Mujoco.PhysHz,
			BatchMs:        c.Device.Mujoco.BatchMs,
			NoRealtime:     c.Device.Mujoco.NoRealtime,
			StartTimeoutMs: c.Device.Mujoco.StartTimeoutMs,
		})
	default:
		log.Fatalf("[fatal] 未知 device.mode=%q（应为 sim / serial / mujoco）", c.Device.Mode)
	}
	if err != nil {
		return err
	}
	defer dev.Close()

	switch c.Device.Mode {
	case "sim":
		log.Printf("链路末端: %s (舵机 %.0f°/s · 延迟 %dms · tick %dms · 限位校验 %v)",
			dev.Kind(), c.Device.Sim.MaxServoSpeed, c.Device.Sim.LatencyMs, c.Device.Sim.TickMs, c.Device.Sim.EnforceLimits)
	case "mujoco":
		log.Printf("链路末端: %s (脚本 %s · 解释器 %s · 物理 %.0fHz · 上报 %.0fHz · 实时 %v)",
			dev.Kind(), mujocoScript, mujocoPython,
			floatOr(c.Device.Mujoco.PhysHz, 1000), floatOr(c.Device.Mujoco.ReportHz, 30),
			!c.Device.Mujoco.NoRealtime)
		// ⚠️ 这两条告警的**适用对象不同**，必须分开说。
		//
		// 早期的写法是"无条件先说一句'物理量是公开值或估算值（config/physics.yaml）'"，
		// 那句话对 MeArm 成立（物理量确实是我们估的），但对 SO-101 **是错的** ——
		// 它的物理量真值在官方 MJCF 里，我们一个字都没覆盖。把错的告警留在日志里，
		// 比没有告警更糟：读日志的人会以为官方模型也被"估"过。
		if entry.PhysicsKind == "driver" {
			// SO-101 这类：物理量**不是**我们估的，官方 MJCF 已写全。
			log.Printf("⚠️ 本机型的物理量真值来源 = 官方 MJCF（%s）："+
				"ArmPilot 未估算、未覆盖任何物理量（%s 只放驱动参数 + 审计快照）。",
				orNone(entry.MJCFPath), entry.PhysicsPath)
			log.Printf("⚠️ 官方 MJCF **未声明**：地面/工作台、相邻连杆的 contact exclude、" +
				"关节速度上限（→ 速率限制在控制层做）、独立标定段；质量来自 CAD 而非称重。")
		} else {
			log.Printf("⚠️ 这是**参数化物理仿真**（Level 3），不是真机标定模型：" +
				"质量/惯量/摩擦为公开值或估算值（robot-package/mearm-v1/physics/physics.yaml），" +
				"限位与标定仍沿用 robot-package/mearm-v1/model/robot.yaml。跑 calibrate.py 可见哪些项还是猜的。")
		}
	default:
		log.Printf("链路末端: %s (%s @ %d %d%s%d · 静默窗口 %dms · 暖机 %v · 单条固件指令超时 %dms)",
			dev.Kind(), c.Device.Serial.Port, c.Device.Serial.Baud,
			c.Device.Serial.DataBits, c.Device.Serial.Parity, c.Device.Serial.StopBits,
			c.Device.Serial.ConnectSettleMs, c.Device.Serial.WarmupEnabled(), c.Device.Serial.AckTimeoutMs)
		log.Printf("⚠️ 真机**没有位置反馈**：joint_state 是固件内部目标值（开环），" +
			"不代表已物理到位；机械臂是否真的动到目标，只能用相机验收（robot-package/mearm-v1/tools/verify_pose.py）")
	}

	// ---- 控制器 ------------------------------------------------------------
	ctl := controller.New(model, dev, controller.Config{
		AckTimeoutMs:      c.Control.AckTimeoutMs,
		MinSendIntervalMs: c.Control.MinSendIntervalMs,
		EchoJointState:    true,
		CalibToleranceDeg: c.Control.CalibToleranceDeg,
	})
	ctl.Start()
	defer ctl.Close()

	// ---- WebSocket 服务 ----------------------------------------------------
	srv := wsserver.New(wsserver.Config{
		Host:            c.Web.Host,
		Port:            c.Web.Port,
		Path:            c.Web.Path,
		PingIntervalMs:  c.Web.PingIntervalMs,
		ClientTimeoutMs: c.Web.ClientTimeoutMs,
	}, ctl)
	errCh := make(chan error, 1)
	go func() {
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			errCh <- err
		}
	}()
	defer srv.Close()

	log.Printf("就绪：浏览器连接 %s", srv.URL())
	log.Printf("健康检查: http://%s/healthz", srv.Addr())

	// ---- 等待退出 ----------------------------------------------------------
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	select {
	case s := <-sig:
		log.Printf("收到信号 %v，正在关闭…", s)
	case err := <-errCh:
		return err
	}
	log.Printf("========== armpilot-backend 退出 ==========")
	return nil
}

func absOrSelf(p string) string {
	if p == "" {
		return "(默认)"
	}
	if abs, err := os.Getwd(); err == nil {
		return abs + string(os.PathSeparator) + p
	}
	return p
}

// orNone 空串显示成 `(生成)` —— MJCF 为空表示"由 gen_model.py 从 robot.yaml 生成"，
// 这个区别在日志里必须看得见（否则"用的是官方模型还是我们生成的"就成了猜）。
func orNone(p string) string {
	if p == "" {
		return "(由 gen_model.py 生成)"
	}
	return p
}

// floatOr 取配置值，为 0 时回落到默认（仅用于**日志展示**，
// 真正的默认值由 server.py 自己持有，不在这里重复定义）。
func floatOr(v, def float64) float64 {
	if v == 0 {
		return def
	}
	return v
}
