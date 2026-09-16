// Package cfg 读取后端运行配置（backend/config.yaml）。
//
// 注意与 robot-package/mearm-v1/model/robot.yaml 的分工：
//
//	robot-package/mearm-v1/model/robot.yaml   —— **模型/标定/限位真值**（前后端共用，唯一来源）
//	backend/config.yaml —— 仅本服务的**运行参数**（端口、设备模式、模拟器参数）
//
// 运行配置里绝不允许出现关节限位或标定数值：那会立刻造成双份真值。
package cfg

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"gopkg.in/yaml.v3"
)

// Config 后端运行配置。
type Config struct {
	Web     WebConfig     `yaml:"web"`
	Robot   RobotConfig   `yaml:"robot"`
	Device  DeviceConfig  `yaml:"device"`
	Control ControlConfig `yaml:"control"`

	Path string `yaml:"-"` // 配置文件自身的路径（日志用）
}

// WebConfig HTTP / WebSocket 监听参数。
type WebConfig struct {
	Host string `yaml:"host"`
	Port int    `yaml:"port"`
	Path string `yaml:"path"`
	// PingIntervalMs 向浏览器发传输层 ping 的间隔
	PingIntervalMs int `yaml:"ping_interval_ms"`
	// ClientTimeoutMs 浏览器静默多久判死
	ClientTimeoutMs int `yaml:"client_timeout_ms"`
}

// RobotConfig 指向模型真值文件。
type RobotConfig struct {
	// ModelID 要加载哪台机器人（`config/robots.yaml` 的 key）。
	// **留空 = 用选择器的 `default`** ⇒ 不写这一行时行为与单机器人时代一致。
	ModelID string `yaml:"model_id"`
	// SelectorPath 模型选择器（`config/robots.yaml`）路径。支持相对路径；
	// 不存在时按候选列表回退（见 ResolveRobotSelector）。
	SelectorPath string `yaml:"selector_path"`
	// ConfigPath **已废弃**：单机器人时代直接指向 `robot.yaml` 的写法。
	// 保留它只是为了让旧的 `backend/config.yaml` 不至于读不懂；
	// 现在由选择器决定加载哪一份配置（见 ResolveRobotSelector）。
	ConfigPath string `yaml:"config_path"`
}

// DeviceConfig 链路末端。
type DeviceConfig struct {
	// Mode: sim（内置假固件）| serial（真串口）| mujoco（MuJoCo 物理仿真）
	Mode   string       `yaml:"mode"`
	Sim    SimConfig    `yaml:"sim"`
	Serial SerialConfig `yaml:"serial"`
	Mujoco MujocoConfig `yaml:"mujoco"`
}

// MujocoConfig MuJoCo 物理后端（`device.Device` 的第三个实现）。
//
// ⚠️ 与 SimConfig 一样，这里**只放运行参数**（解释器 / 脚本路径 / 时间尺度）。
//    质量、惯量、摩擦、增益、关节限位一律来自 robot-package/mearm-v1/physics/physics.yaml 与
//    robot-package/mearm-v1/model/robot.yaml —— 在这里再抄一份，就是第二份真值。
type MujocoConfig struct {
	// Python 解释器命令。留空 = 按 ResolveMujocoPython 的优先级解析
	// （环境变量 → 用户目录下的隔离环境 → PATH 里的 "python"）。
	// ⚠️ 必须指向装了 mujoco 包的解释器。**不要在这里写死本机绝对路径**。
	Python string `yaml:"python"`
	// Script server.py 路径；留空按 ResolveMujocoScript 的候选列表回退。
	Script string `yaml:"script"`
	// ReportHz STATE 上报频率（Hz）；0 = 用 server.py 默认 30
	ReportHz float64 `yaml:"report_hz"`
	// PhysHz 物理步频率（Hz）；0 = 用默认 1000
	PhysHz float64 `yaml:"phys_hz"`
	// BatchMs 每次实时对齐前连续推进的物理时长（ms）；0 = 用默认 10
	// （Windows 的 sleep 粒度约 1~2ms，逐步 sleep 会让仿真慢一个数量级）
	BatchMs float64 `yaml:"batch_ms"`
	// NoRealtime 关闭实时对齐（离线回归 / 压测用）
	NoRealtime bool `yaml:"no_realtime"`
	// StartTimeoutMs 启动握手超时（ms）；0 = 20000（首次 import mujoco 较慢）
	StartTimeoutMs int `yaml:"start_timeout_ms"`
}

// SimConfig 模拟固件参数。
type SimConfig struct {
	MaxServoSpeed float64 `yaml:"max_servo_speed"`
	LatencyMs     int     `yaml:"latency_ms"`
	TickMs        int     `yaml:"tick_ms"`
	EnforceLimits bool    `yaml:"enforce_limits"`
	BootMs        int     `yaml:"boot_ms"`
}

// SerialConfig 真串口参数（Phase 9 使用）。
//
// ⚠️ 这里**只放运行参数**。关节限位与舵机标定一律来自 robot-package/mearm-v1/model/robot.yaml，
// 写进运行配置就会立刻造成双份真值。
type SerialConfig struct {
	Port     string `yaml:"port"`
	Baud     int    `yaml:"baud"`
	DataBits int    `yaml:"data_bits"`
	StopBits int    `yaml:"stop_bits"`
	Parity   string `yaml:"parity"`
	// ReconnectSec 打开失败/断链后的重试间隔（秒）
	ReconnectSec int `yaml:"reconnect_sec"`
	// AckTimeoutMs 单条**固件指令**的等待上限（ms）。
	// 注意它与 control.ack_timeout_ms 不是一回事：一条关节级 JR 会被拆成
	// 多条固件 SET（MAX_PAIRS=3 ⇒ 四舵机 2 条），故 control 层的超时必须
	// ≥ 本条 × 拆包数 + 余量，否则会误报 ACK_TIMEOUT。
	AckTimeoutMs int `yaml:"ack_timeout_ms"`
	// ConnectSettleMs 打开端口后的 bootloader 静默窗口（ms）。
	// Uno 被 DTR 复位后 optiboot 等待 ~2500ms 才交权，窗口内指令会被丢弃。
	ConnectSettleMs int `yaml:"connect_settle_ms"`
	// Warmup 暖机包 + STATUS 探测（Uno 引导交接后首个数据包常被吞）
	Warmup *bool `yaml:"warmup"`
}

// WarmupEnabled 默认开启暖机（yaml 未写时为 true）。
func (s SerialConfig) WarmupEnabled() bool {
	return s.Warmup == nil || *s.Warmup
}

// ControlConfig 控制器策略。
type ControlConfig struct {
	AckTimeoutMs      int `yaml:"ack_timeout_ms"`
	MinSendIntervalMs int `yaml:"min_send_interval_ms"`
	// CalibToleranceDeg 标定回执核对容差（舵机角度）。
	//
	// 必须按**链路末端的量化步长**设置，否则会刷出满屏假告警：
	//   sim    —— 回执是内部计算值，量化 0.01°      => 0.1 足够
	//   serial —— 固件舵机角是**整数**，取整误差 ≤0.5° => 需要 ≥0.5
	CalibToleranceDeg float64 `yaml:"calib_tolerance_deg"`
}

// Default 返回默认配置（文件缺失时使用）。
func Default() Config {
	return Config{
		Web: WebConfig{
			Host: "0.0.0.0", Port: 8090, Path: "/ws/joint",
			PingIntervalMs: 15000, ClientTimeoutMs: 40000,
		},
		Robot: RobotConfig{SelectorPath: "../config/robots.yaml"},
		Device: DeviceConfig{
			Mode: "sim",
			Sim: SimConfig{
				MaxServoSpeed: 240, LatencyMs: 15, TickMs: 20,
				EnforceLimits: true, BootMs: 0,
			},
			Serial: SerialConfig{
				Port: "COM16", Baud: 115200,
				DataBits: 8, StopBits: 1, Parity: "N",
				ReconnectSec: 3,
				// 单条固件 SET 的等待上限；4 舵机会被拆成 2 条 SET。
				AckTimeoutMs: 600,
				// Uno 复位后 optiboot 交权窗口实测 ~2.2~2.6s
				ConnectSettleMs: 2600,
			},
		},
		Control: ControlConfig{AckTimeoutMs: 800, MinSendIntervalMs: 0, CalibToleranceDeg: 0.1},
	}
}

// Load 读取配置文件；文件不存在时返回默认配置（不算错误，便于零配置启动）。
func Load(path string) (Config, error) {
	c := Default()
	c.Path = path
	raw, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return c, nil
		}
		return c, fmt.Errorf("读取配置失败: %w", err)
	}
	if err := yaml.Unmarshal(raw, &c); err != nil {
		return c, fmt.Errorf("解析配置失败: %w", err)
	}
	if c.Web.Port <= 0 {
		c.Web.Port = 8090
	}
	if c.Device.Mode == "" {
		c.Device.Mode = "sim"
	}
	// 兜底：yaml 里写了 `control:` 却漏了容差时，给一个安全带默认值。
	// （yaml.Unmarshal 不会重置未出现的字段，所以这里只处理显式写 0 的情况。）
	if c.Control.CalibToleranceDeg <= 0 {
		c.Control.CalibToleranceDeg = 0.1
	}
	if c.Control.AckTimeoutMs <= 0 {
		c.Control.AckTimeoutMs = 800
	}
	return c, nil
}

// ResolveRobotSelector 定位**模型选择器** `config/robots.yaml`。
//
// 候选顺序（相对当前工作目录）：
//  1. 配置里写的路径（默认 ../config/robots.yaml —— 从 backend/ 启动）
//  2. config/robots.yaml（从 MeArm-3D/ 启动）
//  3. ../MeArm-3D/config/robots.yaml（从仓库根启动）
//
// 全部失败时返回错误并列出全部候选，避免"文件找不到"变成猜谜。
//
// ⚠️ 与已废弃的 `ResolveRobotConfig` 的区别：那个直接找 `robot.yaml`（单机器人时代），
// 这个找**选择器**，再由选择器决定加载哪一份 `robot.yaml`。三端（前端 / 本服务 /
// Python 仿真）之所以能指向同一台机器人，靠的就是都从这里出发。
func ResolveRobotSelector(configured string) (string, error) {
	candidates := []string{configured, "../config/robots.yaml", "config/robots.yaml", "../MeArm-3D/config/robots.yaml"}
	seen := map[string]bool{}
	tried := make([]string, 0, len(candidates))
	for _, c := range candidates {
		if c == "" || seen[c] {
			continue
		}
		seen[c] = true
		abs, err := filepath.Abs(c)
		if err != nil {
			continue
		}
		tried = append(tried, abs)
		if st, err := os.Stat(abs); err == nil && !st.IsDir() {
			return abs, nil
		}
	}
	return "", fmt.Errorf("找不到 robots.yaml（模型选择器），已尝试:\n  %s", joinLines(tried))
}

// ResolveRobotConfig 定位 `robot.yaml`（**单机器人时代的入口**）。
//
// 保留它是因为 `backend/config.yaml` 的旧字段 `robot.config_path` 仍可能被写下；
// 但正常的启动路径已经改走 `ResolveRobotSelector` + `robot.LoadByID`
// —— 后者的好处是"加载哪一台"由**三端共用的那一份配置**决定，而不是各端各写一个路径。
func ResolveRobotConfig(configured string) (string, error) {
	candidates := []string{configured, "../robot-package/mearm-v1/model/robot.yaml", "robot-package/mearm-v1/model/robot.yaml", "../MeArm-3D/robot-package/mearm-v1/model/robot.yaml"}
	seen := map[string]bool{}
	tried := make([]string, 0, len(candidates))
	for _, c := range candidates {
		if c == "" || seen[c] {
			continue
		}
		seen[c] = true
		abs, err := filepath.Abs(c)
		if err != nil {
			continue
		}
		tried = append(tried, abs)
		if st, err := os.Stat(abs); err == nil && !st.IsDir() {
			return abs, nil
		}
	}
	return "", fmt.Errorf("找不到 robot.yaml（模型真值），已尝试:\n  %s", joinLines(tried))
}

// ResolveMujocoScript 解析 MuJoCo 服务脚本 server.py 的路径。
//
// 与 ResolveRobotConfig 同风格：支持相对路径 + 候选回退，让 `backend/` 与
// `MeArm-3D/` 两种工作目录都能找到同一份脚本，也避免写死本机绝对路径
// （换机器即失效的写法在本项目一律禁止）。
func ResolveMujocoScript(configured string) (string, error) {
	candidates := []string{
		configured,
		"../simulation/mujoco/server.py",
		"simulation/mujoco/server.py",
		"../MeArm-3D/simulation/mujoco/server.py",
	}
	seen := map[string]bool{}
	tried := make([]string, 0, len(candidates))
	for _, c := range candidates {
		if c == "" || seen[c] {
			continue
		}
		seen[c] = true
		abs, err := filepath.Abs(c)
		if err != nil {
			continue
		}
		tried = append(tried, abs)
		if st, err := os.Stat(abs); err == nil && !st.IsDir() {
			return abs, nil
		}
	}
	return "", fmt.Errorf("找不到 MuJoCo 服务脚本 server.py，已尝试:\n  %s", joinLines(tried))
}

// ResolveMujocoPython 解析 MuJoCo 子进程要用的 Python 解释器。
//
// 为什么不能把解释器路径写进 `config.yaml`：那是一个**只在本机成立**的绝对路径，
// 换机器/换用户名即失效，而失效的表现是"设备可用但永远没有回执"（最贵的一类失败）。
// 本项目对这类写法的既有态度见 ResolveMujocoScript 的注释（换机器即失效的写法一律禁止）。
//
// 优先级：
//  1. **显式配置**（`device.mujoco.python`）—— 配了就照用，不做存在性校验，
//     因为 `python` 这类裸命令本身就是合法的（由 exec.LookPath 解析）；
//  2. 环境变量 `ARMPILOT_MUJOCO_PYTHON` —— 给 CI / 临时切换用，不必改被跟踪的文件；
//  3. <用户目录>/.workbuddy/binaries/python/envs/default 下的隔离环境
//     —— 这是本仓一直沿用的**相对**位置（`Scripts/python.exe` on Windows、
//        `bin/python` 其它平台），换用户名照样成立；
//  4. 兜底 `python`（PATH）。
//
// ⚠️ 装没装 `mujoco` 包由 `device.NewMujoco` 的启动握手负责暴露（它会给出
//    指向正确解释器的修复指引），这里不重复做一遍探测 —— 那种"先猜一次"的写法
//    会在解释器存在但缺包时给出误导性的错误信息。
func ResolveMujocoPython(configured string) string {
	if s := strings.TrimSpace(configured); s != "" {
		return s
	}
	if s := strings.TrimSpace(os.Getenv("ARMPILOT_MUJOCO_PYTHON")); s != "" {
		return s
	}
	if home, err := os.UserHomeDir(); err == nil && home != "" {
		for _, rel := range [][]string{
			{".workbuddy", "binaries", "python", "envs", "default", "Scripts", "python.exe"},
			{".workbuddy", "binaries", "python", "envs", "default", "bin", "python"},
		} {
			p := filepath.Join(append([]string{home}, rel...)...)
			if st, err := os.Stat(p); err == nil && !st.IsDir() {
				return p
			}
		}
	}
	return "python"
}

func joinLines(items []string) string {
	out := ""
	for i, s := range items {
		if i > 0 {
			out += "\n  "
		}
		out += s
	}
	return out
}
