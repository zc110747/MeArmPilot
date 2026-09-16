package controller

import (
	"testing"
	"time"

	"armpilot/backend/internal/device"
	"armpilot/backend/internal/protocol"
)

// 真 sim 设备 + 真控制器的集成测试（不经 WebSocket）。
// 覆盖"命令 → ACK 门控 → sim 受理 → 有限角速度推进 → STATE 反算 → 状态事件"全链。
func TestWithSimDevice(t *testing.T) {
	m := loadModel(t)
	dev, err := device.NewSim(m, device.DefaultSimTuning())
	if err != nil {
		t.Fatalf("NewSim: %v", err)
	}
	ctl := New(m, dev, Config{AckTimeoutMs: 800, EchoJointState: true})
	got := make(chan map[string]float64, 200)
	ctl.OnJointState(func(j map[string]float64, _ time.Time, _ string) { got <- j })
	ctl.OnError(func(code, msg string) { t.Logf("ERR %s: %s", code, msg) })
	ctl.Start()
	defer ctl.Close()

	if err := ctl.Apply(map[string]float64{"shoulder": 20.8}); err != nil {
		t.Fatalf("Apply: %v", err)
	}

	n := 0
	sawIntermediate := false
	last := 0.0
	for {
		select {
		case j := <-got:
			n++
			last = j["shoulder"]
			t.Logf("state#%d shoulder=%.4f elbow=%.4f", n, last, j["elbow"])
			if last > 5 && last < 20 {
				sawIntermediate = true
			}
			if last >= 20.7 && last <= 20.9 {
				if !sawIntermediate {
					t.Error("从未出现中间态 —— 说明不是有限角速度逼近（疑似等值回显）")
				}
				// 肘角应保持在 HOME 附近（命令里是 112.6，量化后 112.6）
				if e := j["elbow"]; e < 112.5 || e > 112.7 {
					t.Errorf("肘角 = %.4f, 期望 ≈112.6", e)
				}
				return
			}
		case <-time.After(3 * time.Second):
			t.Fatalf("3s 内未收敛，共 %d 个状态，最后 shoulder=%.4f", n, last)
		}
	}
}

// sim 上的「下位机被外部手段改动 → 上位机状态跟随」全链路。
//
// 为什么这条测试能在**没有真机**时验证本次需求：
// 真机上"外部改动"来自硬件摇杆 / 红外遥控（固件 `joystick_scan()` /
// `ir_ctrl_poll()`），它们不经过 JR 通路；sim 里与之对等的是固件级命令
// `JOY` / `SET` —— 后端的 `serial.go` 从不下发它们，所以能到 sim 的
// 只可能是调试或验收入口，语义上确实是"命令之外的改动"。
//
// 断言三件事（缺一不可）：
//  1. 界面上能拿到状态 —— 即 `# SERVO` 被解析并反算成关节角；
//  2. 来源标成 OriginDevice —— 前端据此才会让命令侧跟随（否则滑杆不动）；
//  3. 数值确实变了 —— 防止"发了一行但内容是旧值"这种假通过。
func TestWithSimDeviceExternalChangePublishesDeviceOrigin(t *testing.T) {
	m := loadModel(t)
	dev, err := device.NewSim(m, device.DefaultSimTuning())
	if err != nil {
		t.Fatalf("NewSim: %v", err)
	}
	ctl := New(m, dev, Config{AckTimeoutMs: 800, EchoJointState: true})

	type ev struct {
		joints map[string]float64
		origin string
	}
	got := make(chan ev, 200)
	ctl.OnJointState(func(j map[string]float64, _ time.Time, origin string) {
		got <- ev{joints: j, origin: origin}
	})
	ctl.OnError(func(code, msg string) { t.Logf("ERR %s: %s", code, msg) })
	ctl.Start()
	defer ctl.Close()

	home := m.HomePose["shoulder"]

	// 不经过 JR：直接把肩轴推到底（raw=1023 > 800 ⇒ 单向拨动）
	if err := dev.WriteLine("JOY 7 1023"); err != nil {
		t.Fatalf("WriteLine(JOY): %v", err)
	}

	deadline := time.After(3 * time.Second)
	lastOrigin, lastShoulder := "", home
	for {
		select {
		case e := <-got:
			lastOrigin = e.origin
			lastShoulder = e.joints["shoulder"]
			if e.origin != protocol.OriginDevice {
				continue // 命令来源的帧不参与判定（本链路不该有）
			}
			if lastShoulder >= home-1 {
				continue // 位置还没动（或方向不对），继续等
			}
			t.Logf("外部改动已上报：shoulder %.4f → %.4f（origin=%s）", home, lastShoulder, e.origin)
			return
		case <-deadline:
			t.Fatalf("3s 内未收到 OriginDevice 状态（最后 origin=%q shoulder=%.4f）",
				lastOrigin, lastShoulder)
		}
	}
}
