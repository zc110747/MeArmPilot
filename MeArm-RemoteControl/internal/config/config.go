package config

import (
	"fmt"
	"os"
	"strings"

	"gopkg.in/yaml.v3"
)

// Config 是 arm-web 的顶层配置，全部字段均可通过 YAML 修改。
type Config struct {
	// Mode 运行模式：
	//
	//	"serial"  = 原有真机模式（默认，保持向后兼容）：打开串口，走 arm-device 协议
	//	"network" = 网络模式：**不开串口**，用 TCP Client 连 MeArm-3D 的 TCP 控制接口
	//
	// ⚠️ 这里的 "network" 与 MeArm-3D 的 `--real`（TCP Real 模式）是**两回事**：
	//    前者是本服务"不走串口"，后者是对方"驱动真机"。由 `start.bat` 决定用哪个。
	Mode     string          `yaml:"mode"`
	Serial   SerialConfig    `yaml:"serial"`
	Web      WebConfig       `yaml:"web"`
	TCP      TCPConfig       `yaml:"tcp"`
	Network  NetworkConfig   `yaml:"network"`
	Joystick JoystickConfig  `yaml:"joystick"`
	LogLevel string          `yaml:"log_level"`
}

// 运行模式取值。
const (
	ModeSerial  = "serial"
	ModeNetwork = "network"
)

// NetworkConfig 网络模式参数：TCP Client 连 MeArm-3D 的 TCP 控制接口。
//
// 协议真值见 `MeArm-3D/docs/tcp-control-v1.md`；本段**只放接线与节奏参数**，
// 不含任何角度/限位常数（那是 robot.yaml 的事）。
type NetworkConfig struct {
	Host string `yaml:"host"` // MeArm-3D 所在地址；本机 Sim2Sim 用 127.0.0.1
	Port int    `yaml:"port"` // MeArm-3D 的 tcp.port（默认 9100）

	// ServoIDs TCP 舵机编号 1..N → **arm-device 舵机 id** 的接线表。
	//
	// 它一次回答两个问题，因此**只此一份**：
	//   ① 网页摇杆轴（配置在 `joystick.lx_servo` 等，用 arm-device id 表达）
	//      该落到哪个 TCP 编号上 —— 由 id 在本表里的**位次**决定；
	//   ② 服务端回的 `state.servo` 数组（按 TCP 编号排列）该显示成哪个 S6..S9。
	//
	// 默认 `[9, 7, 8, 6]` 对应 MeArm-3D mearm-v1 的 `Model.JointOrder()`
	// （`1=base(S9) 2=shoulder(S7) 3=elbow(S8) 4=gripper(S6)`），
	// 也就是把 `robot-package/mearm-v1/model/robot.yaml` 里各 actuator 的
	// `channel` 按关节顺序读出来。改机器人换型号时改这里。
	ServoIDs []int `yaml:"servo_ids"`

	// DeadbandDeg 摇杆动作死区（视觉倾角，度）。与 `joystick.deadband_deg` 同义，
	// 但**独立可调**：网络链路的节奏与串口不同，允许分开标定。
	DeadbandDeg float64 `yaml:"deadband_deg"`

	// MinSpeedDegPerS / MaxSpeedDegPerS 刚越死区 / 满偏时的舵机角速度（度/秒）。
	MinSpeedDegPerS float64 `yaml:"min_speed_deg_per_s"`
	MaxSpeedDegPerS float64 `yaml:"max_speed_deg_per_s"`

	// TickMs 摇杆积分的最小间隔（毫秒）；MaxTickMs 单次积分的 dt 上限（防积爆）。
	TickMs    int `yaml:"tick_ms"`
	MaxTickMs int `yaml:"max_tick_ms"`

	// ConnectTimeoutMs / RequestTimeoutMs 建连与单次请求-应答超时（毫秒）。
	ConnectTimeoutMs int `yaml:"connect_timeout_ms"`
	RequestTimeoutMs int `yaml:"request_timeout_ms"`

	// ReconnectMinMs / ReconnectMaxMs 断线重连的指数退避区间（毫秒）。
	// 上限的存在是为了**不做无限快速重连**。
	ReconnectMinMs int `yaml:"reconnect_min_ms"`
	ReconnectMaxMs int `yaml:"reconnect_max_ms"`

	// QueueSize 离散命令（XYZ / 爪 / 单舵机）待发队列容量，满则丢最旧。
	QueueSize int `yaml:"queue_size"`

	// AutoSync 连上后自动向服务端要一次状态（`{"cmd":"state"}`）建立基准。
	// 用指针是为了区分"没写"（默认开）与"显式写 false"。
	AutoSync *bool `yaml:"auto_sync"`

	// FallbackAngle 服务端不提供 `state` 时（旧版）用的兜底基准角。
	// 默认 90 = mearm-v1 的 HOME 舵机角（robot.yaml homePose 经标定换算即四轴 90°）。
	FallbackAngle float64 `yaml:"fallback_angle"`

	// AngleMin / AngleMax 客户端值域保护（**不是软限位**；硬件行程由服务端把关）。
	AngleMin float64 `yaml:"angle_min"`
	AngleMax float64 `yaml:"angle_max"`
}

// AutoSyncEnabled 是否自动同步基准（未配置时默认开）。
func (n NetworkConfig) AutoSyncEnabled() bool {
	return n.AutoSync == nil || *n.AutoSync
}

// ServoIndex 把 **arm-device 舵机 id** 换算成 **TCP 舵机编号**（1..N）。
// 不在接线表里时返回 0（调用方据此明确报错，不静默兜底）。
func (n NetworkConfig) ServoIndex(armServoID int) int {
	for i, id := range n.ServoIDs {
		if id == armServoID {
			return i + 1
		}
	}
	return 0
}

// JoystickConfig 描述网页双 3D 摇杆（遥控形式）到 4 路舵机的映射，均可在 YAML 配置。
type JoystickConfig struct {
	LXServo int  `yaml:"lx_servo"` // 左摇杆 X 轴 -> 舵机 id（默认 9=底座）
	LYServo int  `yaml:"ly_servo"` // 左摇杆 Y 轴 -> 舵机 id（默认 8=左舵）
	RXServo int  `yaml:"rx_servo"` // 右摇杆 X 轴 -> 舵机 id（默认 6=夹取）
	RYServo int  `yaml:"ry_servo"` // 右摇杆 Y 轴 -> 舵机 id（默认 7=右舵）
	InvLX   bool   `yaml:"invert_lx"`
	InvLY   bool   `yaml:"invert_ly"`
	InvRX   bool   `yaml:"invert_rx"`
	InvRY   bool   `yaml:"invert_ry"`
	// DeadbandDeg 摇杆动作死区（视觉倾角，度）。偏移 ≤ 该值的轴视为居中、
	// 不产生任何下发；四轴全部在死区内时整帧 JOY 都不下发（串口零流量）。
	// 网页摇杆满偏 ≈ 31.5°，默认 5°（约 16% 行程）。负值 = 禁用死区。
	DeadbandDeg float64 `yaml:"deadband_deg"`
}

type SerialConfig struct {
	Port          string `yaml:"port"`           // Windows: COMx ; Linux: /dev/ttyUSB0
	Baud          int    `yaml:"baud"`           // 波特率
	DataBits      int    `yaml:"databits"`       // 数据位
	StopBits      int    `yaml:"stopbits"`       // 停止位
	Parity        string `yaml:"parity"`         // N / E / O
	ReconnectSec  int    `yaml:"reconnect_sec"`  // 断线重连间隔(秒)
	MinIntervalMs int    `yaml:"min_interval_ms"` // 两条指令下发的最小间隔(毫秒)，防止高频冲刷设备
	AckTimeoutMs  int    `yaml:"ack_timeout_ms"`  // 等待下位机应答的超时(毫秒)；超时即判定通讯失败
	ConnectSettleMs int  `yaml:"connect_settle_ms"` // 连接建立后等待下位机 bootloader 交出的静默窗口(毫秒)；Arduino Uno 打开串口会触发自动复位，bootloader 约 2.5s 后才交权，窗口内指令会被丢弃
}

type WebConfig struct {
	Enabled bool   `yaml:"enabled"`
	Host    string `yaml:"host"`    // 绑定 IP，0.0.0.0 = 所有网卡
	Port    int    `yaml:"port"`
	WSPath  string `yaml:"ws_path"` // WebSocket 路径
}

type TCPConfig struct {
	Enabled bool   `yaml:"enabled"`
	Host    string `yaml:"host"` // 绑定 IP，0.0.0.0 = 局域网可访问
	Port    int    `yaml:"port"`
}

// Addr 返回 "ip:port" 形式，供 net.Listen 使用。
func (w WebConfig) Addr() string { return fmt.Sprintf("%s:%d", w.Host, w.Port) }
func (t TCPConfig) Addr() string { return fmt.Sprintf("%s:%d", t.Host, t.Port) }

// SetMode 覆盖运行模式（命令行 `-mode` 用）并立即复检 ——
// 非法值必须在装配之前失败，而不是走到一半才发现。
func (c *Config) SetMode(mode string) error {
	m := strings.ToLower(strings.TrimSpace(mode))
	switch m {
	case ModeSerial, ModeNetwork:
		c.Mode = m
		return nil
	}
	return fmt.Errorf("mode 非法: %q（应为 %s / %s）", mode, ModeSerial, ModeNetwork)
}

// Load 从指定路径读取 YAML 并填充默认值。
func Load(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("读取配置文件 %s 失败: %w", path, err)
	}
	var c Config
	if err := yaml.Unmarshal(data, &c); err != nil {
		return nil, fmt.Errorf("解析配置文件 %s 失败: %w", path, err)
	}
	c.applyDefaults()
	if err := c.validate(); err != nil {
		return nil, err
	}
	return &c, nil
}

func (c *Config) applyDefaults() {
	if c.Mode == "" {
		// 默认 serial：直接跑 `arm-web.exe -c config.yaml` 的行为与引入本字段前**完全一致**。
		// 「默认网络模式」是 `start.bat` 的职责（见该脚本头注释），不是二进制的默认值。
		c.Mode = ModeSerial
	}
	// ---- 网络模式（TCP Client → MeArm-3D）----
	if c.Network.Host == "" {
		c.Network.Host = "127.0.0.1"
	}
	if c.Network.Port == 0 {
		c.Network.Port = 9100
	}
	if len(c.Network.ServoIDs) == 0 {
		c.Network.ServoIDs = []int{9, 7, 8, 6}
	}
	if c.Network.DeadbandDeg == 0 {
		c.Network.DeadbandDeg = 5
	}
	if c.Network.MinSpeedDegPerS <= 0 {
		c.Network.MinSpeedDegPerS = 12
	}
	if c.Network.MaxSpeedDegPerS <= 0 {
		c.Network.MaxSpeedDegPerS = 90
	}
	if c.Network.TickMs <= 0 {
		c.Network.TickMs = 25
	}
	if c.Network.MaxTickMs <= 0 {
		c.Network.MaxTickMs = 200
	}
	if c.Network.ConnectTimeoutMs <= 0 {
		c.Network.ConnectTimeoutMs = 1500
	}
	if c.Network.RequestTimeoutMs <= 0 {
		c.Network.RequestTimeoutMs = 1500
	}
	if c.Network.ReconnectMinMs <= 0 {
		c.Network.ReconnectMinMs = 500
	}
	if c.Network.ReconnectMaxMs <= 0 {
		c.Network.ReconnectMaxMs = 5000
	}
	if c.Network.QueueSize <= 0 {
		c.Network.QueueSize = 16
	}
	if c.Network.FallbackAngle <= 0 {
		c.Network.FallbackAngle = 90
	}
	if c.Network.AngleMin == 0 && c.Network.AngleMax == 0 {
		c.Network.AngleMin, c.Network.AngleMax = 0, 180
	}
	if c.Serial.Port == "" {
		c.Serial.Port = "COM4"
	}
	if c.Serial.Baud == 0 {
		c.Serial.Baud = 115200
	}
	if c.Serial.DataBits == 0 {
		c.Serial.DataBits = 8
	}
	if c.Serial.StopBits == 0 {
		c.Serial.StopBits = 1
	}
	if c.Serial.Parity == "" {
		c.Serial.Parity = "N"
	}
	if c.Serial.ReconnectSec <= 0 {
		c.Serial.ReconnectSec = 3
	}
	if c.Serial.MinIntervalMs < 0 {
		c.Serial.MinIntervalMs = 0
	}
	// 注意：命令-应答(ACK)门控 + 摇杆最新值合并已能防止高频冲刷设备，
	// 故默认 0（不额外节流）。如需兜底再按需调大。
	if c.Serial.AckTimeoutMs <= 0 {
		c.Serial.AckTimeoutMs = 800
	}
	// Arduino Uno(ATmega328P) 打开串口会触发 DTR 自动复位进入 optiboot，
	// bootloader 约 2.5s 后才把控制权交给固件；窗口内下发的首条指令会被丢弃。
	// 默认 2500ms 与 host_verify.py 的等待对齐，避免“首条指令无应答”。
	if c.Serial.ConnectSettleMs <= 0 {
		c.Serial.ConnectSettleMs = 2500
	}
	if c.Web.WSPath == "" {
		c.Web.WSPath = "/ws"
	}
	if c.Web.Port == 0 {
		c.Web.Port = 8080
	}
	if c.Web.Host == "" {
		c.Web.Host = "0.0.0.0"
	}
	if c.TCP.Port == 0 {
		c.TCP.Port = 9001
	}
	if c.TCP.Host == "" {
		c.TCP.Host = "0.0.0.0"
	}
	if c.LogLevel == "" {
		c.LogLevel = "info"
	}
	if c.Joystick.LXServo == 0 {
		c.Joystick.LXServo = 9
	}
	if c.Joystick.LYServo == 0 {
		c.Joystick.LYServo = 8
	}
	if c.Joystick.RXServo == 0 {
		c.Joystick.RXServo = 6
	}
	if c.Joystick.RYServo == 0 {
		c.Joystick.RYServo = 7
	}
	if c.Joystick.DeadbandDeg == 0 {
		c.Joystick.DeadbandDeg = 5 // 默认动作死区 5°；负值显式禁用
	}
}

func (c *Config) validate() error {
	switch c.Mode {
	case ModeSerial, ModeNetwork:
	default:
		return fmt.Errorf("mode 非法: %q（应为 %s / %s）", c.Mode, ModeSerial, ModeNetwork)
	}
	if c.Mode == ModeNetwork {
		if len(c.Network.ServoIDs) == 0 {
			return fmt.Errorf("network.servo_ids 不能为空（它是摇杆轴与服务端状态共用的接线表）")
		}
		seen := make(map[int]bool, len(c.Network.ServoIDs))
		for _, id := range c.Network.ServoIDs {
			if seen[id] {
				return fmt.Errorf("network.servo_ids 有重复项: %d", id)
			}
			seen[id] = true
		}
		if c.Network.AngleMax <= c.Network.AngleMin {
			return fmt.Errorf("network.angle_max 必须大于 angle_min（%v <= %v）",
				c.Network.AngleMax, c.Network.AngleMin)
		}
	}
	switch c.Serial.Parity {
	case "N", "E", "O", "n", "e", "o":
	default:
		return fmt.Errorf("serial.parity 非法: %q (应为 N/E/O)", c.Serial.Parity)
	}
	if c.Serial.DataBits < 5 || c.Serial.DataBits > 8 {
		return fmt.Errorf("serial.databits 非法: %d (5..8)", c.Serial.DataBits)
	}
	if c.Serial.StopBits != 1 && c.Serial.StopBits != 2 {
		return fmt.Errorf("serial.stopbits 非法: %d (1/2)", c.Serial.StopBits)
	}
	return nil
}
