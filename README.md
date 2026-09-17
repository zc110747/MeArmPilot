# MeArmPilot

**mARM 机械臂全栈平台** —— 从**裸机固件** → **串口 / Web 网关** → **数字孪生与真机同步控制**，
用**一份模型定义**贯穿仿真与物理硬件。
 

> MeArmPilot is an open-source robotic arm platform focused on simulation,   
> real-robot control, data collection, and extensible robot development.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Firmware](https://img.shields.io/badge/firmware-ATmega328P%20%C2%B7%20avr--libc-8a2be2)](MeArm-Device)
[![Gateway](https://img.shields.io/badge/gateway-Go%201.21-00ADD8)](MeArm-RemoteControl)
[![Frontend](https://img.shields.io/badge/frontend-React%2019%20%C2%B7%20Three.js-61DAFB)](MeArm-3D)
[![Physics](https://img.shields.io/badge/physics-MuJoCo%203.13-brightgreen)](MeArm-3D/simulation)

![MeArmPilot 控制台 · HOME / RESET 位](MeArm-3D/docs/images/armpilot-console.png)

<sub>HOME 位（四舵机全 90° = 固件 RESET 位）的数字孪生渲染，与实拍照片目视一致。</sub>

---

## 目录

- [1. 这是什么](#1-这是什么)
- [2. 系统架构](#2-系统架构)
- [3. 三个子项目](#3-三个子项目)
- [4. 功能矩阵](#4-功能矩阵)
- [5. 技术栈总览](#5-技术栈总览)
- [6. 完成情况](#6-完成情况)
- [7. 快速开始](#7-快速开始)
- [8. 仓库结构](#8-仓库结构)
- [9. 能力边界（诚实声明）](#9-能力边界诚实声明)
- [10. 文档索引](#10-文档索引)
- [11. License](#11-license)

---

## 1. 这是什么

MeArmPilot 是一台 **meArm 型 4 自由度舵机机械臂**的完整开源实现，覆盖「硬件固件 → 网关服务 → 数字孪生」全链路：

- **真实硬件**：Arduino Uno（ATmega328P）裸机固件驱动 4 路舵机，支持串口指令、硬件摇杆（ADC）、红外遥控（NEC）与动作序列。
- **虚拟模型**：浏览器里的 3D 机械臂与真机共享同一个 `RobotModel`，做到「**你看到的虚拟臂，就是现实机械臂的实时映射**」。
- **真实物理**：MuJoCo 物理仿真作为**末端第三实现**接入，与运动学实现共享同一套协议与前端，**零改动**切换。

### 设计原则

| 原则 | 落地方式 |
|---|---|
| **唯一真值源** | 尺寸 / 关节 / 限位 / 标定只存在于 `MeArm-3D/robot-package/<id>/model/robot.yaml`，物理量只存在于同包 `physics/physics.yaml`；前端、Go 后端、MuJoCo 与 URDF 生成器**都读它**，代码里禁止硬编码 |
| **真值随包走** | 每台机器人一个包（`robot-package/<id>/`）：真值 / 运动学引擎 / 物理量 / 测试 / 工具全在包内。`config/robots.yaml` 只剩**选择器指针**（只有"加载谁"和"文件在哪"），路径一律由 `robopkg.declared_path()` 解析 |
| **先量后改** | 机构参数由「相机照片 + 舵机逐度扫描 + 3D 结构 STEP 反解」三方交叉得到，**实测推翻假设**（最初按固件命名推定的肩/肘角色是错的；夹爪标定方向也被真机实测推翻，见 §6.3） |
| **不许伪造** | 物理仿真显式声明「标定层次」，能力边界（如无位置回读）在文档与 UI 中明确写出 |
| **真值冻结** | 运动学 / 物理语义核心哈希入库，**改外观放行、改真值报错**，必须有意识地解冻 |
| **可验收** | 每个阶段都有可复现的 pass/fail 判据与量化数字，不靠目视 |

---

## 2. 系统架构

三个子项目共用**同一片 Arduino Uno**，但提供**两条互相独立、端口刻意错开的控制路径**：

```
                         ┌──────────────── 浏览器 ────────────────┐
                         │                                        │
        ┌────────────────┴──────────────┐        ┌────────────────┴──────────────┐
        │  MeArm-3D  · 数字孪生          │        │  MeArm-RemoteControl · 遥控台   │
        │  React 19 + Three.js           │        │  Go + 内嵌 Web（three.js 双摇杆）│
        │  关节级：滑杆 / XYZ / 拖动 / 示教│        │  舵机级：归一化摇杆帧            │
        │  :8090 (/ws/joint)  :5273 dev  │        │  :9001 (WS)  :9002 (TCP)        │
        └────────────────┬──────────────┘        └────────────────┬──────────────┘
                         │ WebSocket（JSON，关节级）                │ WebSocket（JSON，摇杆帧）
                         ▼                                        ▼
        ┌──────────────────────────────────┐      ┌──────────────────────────────┐
        │  MeArm-3D/backend （Go 1.21）      │      │  arm-web （Go 1.21）          │
        │  device：sim │ serial │ mujoco    │      │  serial ➜ hub ➜ tcp/web       │
        │  ACK 门控 · latest-wins · 标定核对 │      │  或 netlink ➜ TCP Client       │
        └────────────────┬─────────────────┘      └──────────────┬───────────────┘
                         │ 串口 115200 8N1                          │ 串口 115200 8N1
                         └──────────────────┬───────────────────────┘
                                            ▼
                         ┌──────────────────────────────────────┐
                         │  MeArm-Device · ATmega328P 裸机固件    │
                         │  串口指令 / 硬件摇杆(ADC) / 红外(NEC)   │
                         │  Timer1 PWM 四路舵机 · Timer2 1ms 时基 │
                         └──────────────────────────────────────┘
                                            ▼
                          底座(S9) · 左舵(S8) · 右舵(S7) · 夹取(S6)
                          ⚠️ 以上是**固件命名**；运动学角色见 §6.3（S9 底座 / S7 肩 / S8 肘 / S6 夹取）
```

除上面这条"两条串口"路径外，遥控台**默认走第三条路径**（`start.bat` 的默认模式，
不打开串口）—— 它作为 TCP Client 接入 MeArm-3D 的 TCP 控制接口，由 MeArm-3D
决定把命令交给 Sim 还是 Real：

```
 浏览器 ──WS(摇杆帧)──▶ arm-web ──TCP Client(JSON Lines)──▶ MeArm-3D tcpserver :9100
                                                             └─▶ Controller ─▶ device: sim / real
```

- **9001（遥控台 Web）与 8090（数字孪生前端）刻意错开** ⇒ 两套前端可同时运行；
  遥控台自己的串口透传用 **9002**，MeArm-3D 的 TCP 控制接口用 **9100**，三者互不相干。
- 两条路径最终都归一化为**同一套文本指令**下发固件，因此行为完全一致。
- MuJoCo 物理仿真作为 `device` 接口的**第三实现**（`sim | serial | mujoco`）接入，
  协议层 / WebSocket / controller / 前端**全部零改动**。
- `robot-package/<id>/model/robot.yaml` + `physics/physics.yaml` 是**唯一真值**；
  由它派生出**两条生成链**（都是产物，改真值必须重新生成）：
  `gen_model.py` → MJCF（`simulation/mujoco/mearm.xml`）·
  `gen_urdf.py` → 标准 URDF（`robot-package/mearm-v1/urdf/mearm-v1.urdf`）。

---

## 3. 三个子项目

| 子项目 | 角色 | 技术栈 | 状态 |
|---|---|---|---|
| [**MeArm-Device**](MeArm-Device) | ATmega328P 裸机固件 | C/C++ · avr-libc · 直接寄存器 · PlatformIO(工具链) · avrdude | ✅ 零警告，指令回归 67/67 |
| [**MeArm-RemoteControl**](MeArm-RemoteControl) | 串口 / TCP → Web 网关（双传输模式） | Go 1.21 标准库 · `//go:embed` · RFC6455 · 内嵌 three.js 前端 | ✅ 门控 ~13ms，端到端 ~10ms；网络模式 Sim2Sim 30/30 |
| [**MeArm-3D**](MeArm-3D) | 数字孪生 + 关节级后端 + 物理仿真 | React 19 · TS · Vite · Three.js(R3F) · Zustand · Go 1.21 · MuJoCo 3.13 · URDF/STEP | ✅ Phase 1–14 + M1–M10 + 包化重构 Phase 0–2 |

### 3.1 MeArm-Device · 裸机 AVR 固件

**不使用 Arduino 框架**，纯 C/C++ + avr-libc + 直接寄存器编程。

| 模块 | 功能 |
|---|---|
| `bsp/servo` | Timer1 CTC（TOP=ICR1=39999，20 ms 帧），COMPA 中断状态机**顺序调度 4 路脉宽**，按各舵机强制范围钳位 |
| `bsp/systick` | Timer2 **1 ms 时基**，主循环彻底**零阻塞**（取代忙等 `_delay_ms`） |
| `bsp/uart` | USART0 **115200 8N1（U2X）**，RX/TX 环形缓冲 + 中断驱动，`uart_printf` 基于 `vsnprintf_P` |
| `bsp/adc` | 4 路摇杆模拟量采样（AVCC 参考，预分频 /128，10-bit） |
| `bsp/ir` | **NEC 32-bit 解码**（PD2/INT0 双边沿 + Timer0/64 4 µs 时基，地址与命令双重取反校验） |
| `bsp/eeprom` | 红外按键**运行时学习**结果持久化，断电不丢 |
| `bsp/led` | 板载 D13 心跳指示（500 ms 翻转） |
| `core/arm_control` | 角度斜坡平滑（3°/20 ms）、目标/当前双角、`arm_all_reached()` 到位判定 |
| `core/cmd` | 串口指令解析：`SET` / `S<n>=` / `STOP` / `AUTO` / `JOY` / `IR` / `SEQ` / `JOYHW` / `IRHW` / `ADC` / `RESET` / `STATUS` / `IRLEARN` / `IRCODES` / `IRCLEAR` / `HELP` |
| `core/joystick` | 硬件摇杆扫描 + **比例步进**（偏移越大越快，2–10°/20 ms） |
| `core/ir_ctrl` | 13 键表（8 微调 + 4 动作集 + 1 停止）→ `arm_nudge` / 序列触发 |
| `core/ir_seq` | **动作序列引擎**：4 套各 7 帧关键帧（存 `PROGMEM`），"到位 + 保持间隔"才进下一帧，可循环 / 切换 / 被摇杆打断 |

- **命令-应答（ACK）契约**：每条指令**必须且仅有一次**应答行（`OK ...` / `ERR ...`）；
  异步事件统一以 `# ` 开头，供上位机门控区分 —— 这是网关侧 ACK 门控能成立的前提。
- **⚠️ AVR 内存铁律**：ATmega328P 仅 2 KB RAM，`avr-gcc` 默认把字符串字面量放进 `.data`（RAM）。
  本工程所有字面量走 `PSTR()` + `_P` 变体函数、所有常量表走 `PROGMEM` + `pgm_read_*`。
  当前：`.data` 138 B · `.bss` 706 B · **RAM 844 B，栈余量 ≈ 1204 B** ·
  **FLASH 12,848 B（≈40%）**，零警告。
- 串口发送**不忙等**：TX 走中断驱动的环形缓冲，「发完才返回」的老写法会让主循环在
  高波特率下丢字节；改为入队即返回后，连续指令不再丢。
- 上位机回归：`python tools/host_verify.py COM4 115200` → **67/67 PASS**。

### 3.2 MeArm-RemoteControl · 串口 / TCP → Web 网关（`arm-web`）

**双传输模式**（`start.bat` 默认 network、`--real` 走串口；见其 README）：

- **串口模式（serial）** —— 原有路径，行为逐字节不变：
  - **串口层**：纯标准库实现（Windows 走 `syscall` 直连 kernel32，Linux/macOS 走 `stty`）。
    自动重连、**ACK 门控**（同一时刻仅 1 条在途）、**连接静默窗口 + 暖机包**
    （等 Uno bootloader 交权，吞掉首包再下发真指令）。
  - **TCP 转发**：局域网设备 raw TCP 连接后可直接下发固件指令，为远程控制预留统一接口。
- **网络模式（network）** —— 作为 **TCP Client** 连 [MeArm-3D 的 TCP 控制接口](MeArm-3D/docs/tcp-control-v1.md)
  （JSON Lines，`:9100`），把网页意图翻译成对方已有的 `servo` / `move` / `gripper` / `state` 命令：
  - **单一写泵**：所有写只在一个 goroutine 里，同一连接上 JSON 不可能交叉；
    摇杆按舵机维度 latest-wins、离散命令走有界 FIFO（满则丢最旧）。
  - **时间驱动的角度积分**：摇杆增量 = 速度 × 真实 dt ⇒ 前端发送频率不影响速度。
  - **越界即回滚**：服务端拒绝某轴时本地目标回滚到确认值（否则该轴"推杆完全不动"且不报错）。
  - **刻意不做**：IK / FK / 坐标变换 / 标定换算 / 软限位 / Sim·Real 选择 —— 全归 MeArm-3D。
- **Web 控制台**（两模式共用同一份页面，能力差异由 WS `caps` 告知）：
  Three.js **双 3D 摇杆**（左：底座 + 左舵；右：夹取 + 右舵）、
  **动作死区 5°**（四轴全居中 ⇒ 整帧不发，串口零流量）、按住偏转 **30 ms 心跳持续步进**、
  S6–S9 角度面板（从所有含角度的应答**合并更新**）、**30 行滑动窗口**回显终端；
  网络模式另开 **XYZ 步进 / 爪 open·close / 四舵机滑条 + TCP 末端坐标回读**面板。
- **性能根治**：Windows 非重叠 I/O 读写互斥曾导致每条命令 218–932 ms、丢帧 75%；
  改用「立即返回」读模式 + 手动行缓冲（弃用 `bufio`，同时修掉空闲 20 s 必断连）
  ⇒ **门控单条周期 ~13 ms · WebSocket 端到端 ~10 ms**。
- 端口：**9001**（Web / WebSocket）· **9002**（TCP 透传，仅串口模式）· 目标侧 **9100**（MeArm-3D TCP）·
  均可在 YAML 配置。

### 3.3 MeArm-3D · 数字孪生 + 关节级后端 + 物理仿真

让**虚拟机械臂**与**真实机械臂**共享同一个 `RobotModel`，实现双向同步。

**前端（React + Three.js）**

| 能力 | 说明 |
|---|---|
| 唯一模型源 | `RobotModel` 从包内 `model/robot.yaml` 加载，几何 / 运动学 / 限位 / 标定全部由配置派生 |
| 机器人切换 | 活动机器人是 **store 状态**（非模块常量）：初值来自 `config/robots.yaml`，切换有守卫；`hello` 只互检、**不静默切换** |
| 3D 场景 | `buildRobotObject3D` 按件生成（倒角板 / 舵机 / 轴销 / 夹爪轮廓），支持**实拍照片纹理贴图** |
| 正向运动学 | `fk.ts` 与 Three.js 渲染矩阵**互相独立**、交叉验证 |
| 逆运动学 | 包内 `ik.ts` 平面 2R 解析解，错误码 `OUT_OF_WORKSPACE` / `JOINT_LIMIT`，多解 `elbow-up/down/nearest` |
| 关节控制 | 逐关节滑杆 + 角度限位 + 舵机标定换算 |
| 末端目标 | XYZ 直输 + **鼠标真实拖拽**（拖动平面在 pointerdown **冻结**，越界不钳位） |
| 末端目标安全参数 | 判据容差（≤1°）· **球壳内径覆写**（≥0.1 mm，只做前置检查）· **球壳外径覆写**（1–160 mm，前置检查**并真正传入 `ik.ts` 参与 `cosAlpha` 求解**）· 改动**实时重解**。约束的是**腕枢轴**距离 `d`，不是 TCP 到原点的距离 |
| 传输抽象 | `RobotTransport` 接口：`MockTransport`（模拟有限角速度 / 延迟 / 丢帧 / 限位拒绝）与 `WebSocketTransport` |
| 链路反馈 | Phase 11 逐关节带符号偏差条 + 误差趋势 + 健康结论；Phase 12 **实际臂幽灵**（半透明第二条臂，露出的就是滞后量） |
| 示教 | Phase 13 `Record / Play / Pause / Stop / Clear / Export / Import`，录 `commandJoints`（不含链路时延），回放复用既有命令通道 |
| 状态仓库 | Zustand 单一状态源 + `transportBridge`（尾沿合并 33 ms / 回推只写 `actual` / 回环打破 / 重连补发） |
| 包边界 | `corePackageBoundary.test.ts` **双向断言** Core ↔ 包的依赖边：多一条少一条都失败，防止 Core 反向依赖某台机器人 |

**后端（Go 1.21，自包含 module）**

| 包 | 职责 |
|---|---|
| `internal/cfg` | 配置解析（`config.yaml` / `config.serial.yaml`）与启动期一致性核对 |
| `internal/robot` | `registry.go` 读 `config/robots.yaml` 选择器（**只做选择**，禁止 `if robot ==`）→ `robot.go` 读包内 `model/robot.yaml`；关节 ↔ 舵机换算；限位校验（唯一「真值」入口） |
| `internal/protocol` | JSON / `JR` / `OK JR` / `STATE` / `ERR` 编解码（不认识机械结构） |
| `internal/controller` | **唯一「懂机械臂」处**：ACK 门控 · latest-wins · 标定核对 · 状态发布 |
| `internal/device` | `sim.go`（内置假固件）· `serial.go`（真串口）· `mujoco.go`（Python 子进程） |
| `internal/wsserver` | 标准库 RFC6455 服务端 · 路由 · 广播 · 两层心跳 |

**模型生成链与物理仿真（MuJoCo 轨 M1–M10）**

- **两条生成链，产物禁止手改**：`gen_model.py` → MJCF（`simulation/mujoco/mearm.xml`）·
  `gen_urdf.py` → 标准 **URDF**（`robot-package/mearm-v1/urdf/mearm-v1.urdf`，供 Three.js / ROS / MoveIt 等标准工具消费）。
  两条链都有测试盯着「与配置同步」，且生成器必须住在包内。
- **3D 结构回溯**：`robot-package/mearm-v1/3d-structure/mearm3Dasm.STEP` 是**几何/装配真值**，
  配置里未知的尺寸由它反解；STEP 一律走 **OCCT 参考实现**，不写自研解析器。
- 三层时间步解耦（物理 1 kHz / 控制 100 Hz / 渲染只读），批量步进与渲染帧率**按位可复现**。
- 用 **Vite SSR 桥**把前端真实 `ik.ts` 当**独立裁判**（不是用 Python 再写一份自证）。
- **统一 Sim2Sim 框架**：判据只有**一份**（`runSim2Sim(robot)`），容差**按机器人登记且必须写明理由**；
  黄金数据把**行为**落盘，判据从「两套实现互相比对」改成「与冻结时一致」。
- Level 声明机器可检查（`calibrated == false`），不伪造「已标定」。

---

## 4. 功能矩阵

| 能力 | MeArm-Device | MeArm-RemoteControl | MeArm-3D |
|---|:---:|:---:|:---:|
| 4 路舵机 PWM 控制 | ✅ | — | ✅（虚拟） |
| 串口指令协议 | ✅（固件侧） | ✅（网关侧） | ✅（关节级） |
| ACK 命令-应答门控 | ✅（契约方） | ✅ | ✅ |
| 硬件摇杆（ADC） | ✅ | — | — |
| 红外遥控（NEC）+ 按键学习 | ✅ | — | — |
| 红外动作序列（4 套） | ✅ | ✅（可下发） | — |
| 局域网 TCP 透传（文本指令，仅串口模式） | — | ✅ | — |
| 关节级 TCP 控制接口（JSON Lines） | — | ✅（Client） | ✅（Server） |
| Web 3D 摇杆（舵机级） | — | ✅ | — |
| 末端 XYZ 步进 / 爪开合 / 舵机滑条面板 | — | ✅（网络模式） | — |
| 数字孪生（关节级） | — | — | ✅ |
| FK / IK | — | — | ✅ |
| 鼠标拖拽末端 | — | — | ✅ |
| 末端安全参数（球壳内径 / 外径覆写） | — | — | ✅ |
| 多机器人分派（注册表，禁 `if robot ==`） | — | — | ✅ |
| 标准 URDF 导出 | — | — | ✅ |
| STEP 装配体反解几何 | — | — | ✅ |
| 示教录制 / 回放 | — | — | ✅ |
| 链路误差面板 / 幽灵臂 | — | — | ✅ |
| MuJoCo 真实物理仿真 | — | — | ✅ |
| 实拍照片纹理贴图 | — | — | ✅ |
| 真值冻结（语义哈希） | — | — | ✅ |

---

## 5. 技术栈总览

| 层 | 选型 |
|---|---|
| 固件 | **C / C++ · avr-libc · 直接寄存器编程**（ATmega328P @16 MHz）；PlatformIO 仅作工具链与板卡管理，**不链接 Arduino 框架**；avrdude 烧录 |
| 网关 | **Go 1.21**（标准库 + `gopkg.in/yaml.v3`）· `//go:embed` 自包含单文件 · Windows `syscall` / Unix `stty` · RFC6455 WebSocket |
| 数字孪生前端 | **React 19 · TypeScript 5.9 · Vite 8 · Three.js · React Three Fiber · @react-three/drei · Zustand** |
| 关节级后端 | **Go 1.21**（标准库 `net/http` + RFC6455 WebSocket + `yaml.v3`） |
| 物理仿真 | **MuJoCo 3.13（Python）** · MJCF 由 YAML 生成 · `numpy` |
| 模型描述 | **URDF**（`gen_urdf.py` 生成，标准工具可消费）· **MJCF**（`gen_model.py` 生成） |
| CAD / 结构 | **STEP 装配体**（AP214）· **OCCT 参考实现**解析（`step_obb.py` / `step_report.py`）· 按装配体包围盒反解连杆几何 |
| 包契约 | 自研 `robopkg`（manifest 解析 / `declared_path()` / 语义内容哈希 / 校验 CLI），随仓提供、**无第三方依赖** |
| 上位机工具 | Python 3（numpy / Pillow / pyserial / pytest / OCCT）· Node.js（e2e 与链路探针） |
| 测试 | **Vitest**（单元 / 验收 / Sim2Sim）· `go test` · **pytest**（包契约 / 仿真轨）· 零依赖 **CDP e2e**（无头 Edge/Chrome）· Node 端到端脚本 |
| 图像 | Otsu 分割 · 对称 Chamfer 拟合 + Hooke-Jeeves · PCA 定角 · homography 透视校正 |

---

## 6. 完成情况

### 6.1 里程碑

| 子项目 | 里程碑 | 状态 |
|---|---|---|
| MeArm-Device | 裸机固件（PWM / 时基 / 串口 / 摇杆 / 红外 / 序列 / EEPROM） | ✅ 零警告 · 67/67 PASS |
| MeArm-RemoteControl | 串口网关 · ACK 门控 · TCP 透传 · Web 双 3D 摇杆 | ✅ 端到端 ~10 ms |
| MeArm-RemoteControl | TCP 接入 MeArm-3D（network 模式 · `start.bat` · XYZ/爪/舵机面板 · 只读 `state` 基准 · 对端页面跟随 `origin=external`） | ✅ Sim2Sim 30/30 |
| MeArm-3D | Phase 1–14（模型 / 3D / FK / IK / 拖动 / Mock / Go WS / 真串口 / 反馈 / 幽灵 / 示教 / 被动腕） | ✅ |
| MeArm-3D | MuJoCo 物理轨 M1–M10 | ✅ |
| MeArm-3D | 照片纹理贴图（D63 / D64 / D66 / D68 / D69） | ✅ |
| MeArm-3D | Robot Package 重构 Phase 0–2（只读分析 / Core 边界 / 真值·工具·实现·测试随包走） | ✅ |
| MeArm-3D | 3D 结构（STEP）接入 + 标准 URDF 生成链 | ✅ 产物入库 · 测试盯同步 |
| MeArm-3D | 末端目标安全参数改版（删限位调节 / 加球壳外径覆写 / 修 d 口径不一致） | ✅ |
| MeArm-3D | ADR 决策记录 D1–D83 | ✅ 80 条 |

### 6.2 验收数据（可复现）

**MeArm-3D**

```
类型检查      tsc -b                        0 error
单元/验收测试  vitest run                   439 / 439 PASS（33 文件）
后端单测      go test ./...                 111 PASS + go vet 干净
浏览器 e2e    node tests/e2e/ui-smoke.mjs   87 / 88 PASS（1 项为测试时序问题，见下注）
仓库 Python   pytest -q（仓库根）            198 PASS（core/tests 31 · tests 160 · robot-package 7）
  其中仿真轨   pytest tests/sim              151 PASS（13 文件）
包契约        robopkg validate --all        1 个包全部通过 · 内容哈希 0780e71c…（35 个文件进哈希）
生产构建      vite build                    1,358.40 kB（gzip 389.62 kB）

FK ↔ Three.js   200 组随机位姿              末端最大误差 8.710e-14 mm
FK(IK(XYZ))     2000 组随机可达位姿          最大残差 1.180e-13 mm（失败 0 组）
拖动连续性      300 点跟随                  最大误差 9.210e-14 mm · 支解切换 0 次
MJCF 结构       gen_model.py                9312 B · nq=5 nv=5 njnt=5 nu=4
                                            nbody=8 ngeom=34 nexclude=5 neq=1 ntendon=1 · 0.1415 kg
URDF 结构       gen_urdf.py                 10,521 B · 7 link / 10 joint / 6 inertial / 6 collision / 26 visual
HOME 位 TCP     [115.0335, 0, 109.2236] mm  闭式解与前端 IK 逐位吻合
重力对照        无驱动 3 s                  Δshoulder 32.095° / Δelbow 30.990°（Δbase = Δgripper = 0.000°）
接触力判据      dist = +1.353 mm            qfrc_constraint 非零 · 禁用台面后 TCP 下落 14.62 mm
跨进程确定性    run.py --demo 跑两遍         12 行数值载荷逐字相同（seed=0）
```

> 注：e2e 的 1 项失败是「页面无控制台错误」——该段测试**故意停掉后端**来验证自动重连，
> 浏览器必然在停机窗口留下一条 `ERR_CONNECTION_REFUSED` 日志。它是测试自身的时序敏感项，
> 与产品行为无关（87 项功能断言全过）。

**MeArm-Device**

```
构建          avr-gcc + avrdude             零警告
FLASH                                        12,848 B（text 12,710 + data 138）/ 32,256 B 可用（≈40%）
RAM                                          844 B（.data 138 + .bss 706）/ 2 KB · 栈余量 ≈ 1,204 B
指令回归      python tools/host_verify.py   67 / 67 PASS（需真机串口）
```

**MeArm-RemoteControl**

```
静态检查      go vet ./...                  干净
单元测试      go test ./...                 39 PASS（含 -race 干净）
网络模式 e2e  node tools/sim2sim-check.mjs  30 / 30 PASS（Sim2Sim，期望值从 config.yaml + robot.yaml 派生）
门控性能      ACK 门控单条周期                ~13 ms（改造前 218–932 ms）
端到端        浏览器 → 服务器 → 串口           ~10 ms
受控流        50 ms 节奏连续 JOY              40 / 40 全部下发
```

**跨项目（TCP 接入）**

```
MeArm-3D TCP 只读 state 命令  go test ./internal/tcpserver/   新增用例 PASS（只读性 + 反映最新命令 + 大小写不敏感）
两模式方向等价性              main_test.go 护栏              满偏 dt=1s 的增量符号 > 0（与串口同向）
端到端 Sim2Sim                arm-web(network) ⇄ MeArm-3D(sim)  30 / 30 PASS
真机验证                      未做                             ⚠️ 见 §9
```

### 6.3 关键结论（实测推翻假设）

- **舵机命名 ≠ 运动学角色**：最初按固件 `SERVO_LEFT/RIGHT` 推定肩/肘，被相机实测推翻，
  真实映射为 **S9 底座 / S7 肩 / S8 肘 / S6 夹取**。
- **小臂是平行四连杆（绝对角耦合，增益 `-1`）**：由逐度扫描独立确证。
- **爪被连杆锁成水平（Phase 14）**：小臂绝对倾角变 **29.24°** 时爪的画面倾角只变 **7.31°**
  ⇒ 旧模型「爪固连小臂」被否决，改为**被动腕关节**，连带重推 IK 几何、MuJoCo 约束、
  自由度计数口径与台面高度。
- **肩标定增益仍有 −13.1% 偏差**，但 `tools/verify_calib_repro.py` 判定**跨批测量本身不可复现**
  （极差 10.8% / 52.9% > 5% 容差）⇒ 此时改模型或改标定表都不成立，**唯一入口是重布台面重测**（A3）。
- **夹爪标定方向被真机推翻（D80）**：真机实测 **S6=40° 张开 / S6=130° 闭合**，与旧标定表
  （`servo = θ + 40`，θ=0 完全闭合）**恰好相反** ⇒ 网页拖到「张开」时真机在闭合。
  关键判据：**网页 3D 显示是对的，错的是标定表**，所以症状表现为「虚拟臂正常、真机反向」。
- **球壳安全参数约束的是「腕枢轴」距离 `d`，不是 TCP 到原点的距离**：
  `d = hypot(r − pivotR − toolOffset[0], z − pivotZ − toolOffset[1])`，**球心在肩枢轴**。
  同一个点，`ik.ts` 口径算出 111.542 mm，而 store 里另一份自研几何算成 92.159 mm
  ——**差 19.4 mm 且不报错**，症状是「明明是范围内却被判越界」。
  修法不是对齐数字，而是**删掉第二份几何、改为转发唯一实现**（与「同一个量只能有一套口径」同源）。
  ⚠️ 两个推论：**「臂展 160」≠「能伸到 160 mm 远」**（TCP 径向恒比腕枢轴多 40 mm）；
  UI 上 1–160 是**几何球壳**的范围，而关节限位把它裁到实际有效域 **`d ∈ [44.17, 139.27]`**。
- **几何真值来自 3D 装配体（STEP）**：未知尺寸由 OCCT 解析装配体反解，
  而非按照片目测；STEP 解析一律用参考实现，不写自研解析器。
- **同一条"推杆方向"在两条链路上不是同一个意思**：串口链路的 `invert_*` 里含一层
  **arm-device 固件方向补偿**（`core/joystick.c`：8 轴出厂即反相，9/6/7 轴 `raw>800 → 负步长`），
  而 TCP 链路**没有固件层**。直接把配置搬过去，会让两种模式的推杆方向**正好相反**
  （实测同一满偏输入：串口 +90°/s、TCP −90°/s），且两边都"看起来正常工作"。
  修法不是改配置，而是加一次显式换算（`NetInvertFor`），并用
  「满偏 dt=1s 的增量符号必须为正」写成护栏测试。

---

## 7. 快速开始

### 7.1 只跑数字孪生（不需要硬件）

```bash
cd MeArm-3D/frontend
npm install
npm run dev            # 打开提示的地址（默认 :5273）
```

前端默认停在 `MockTransport` 且**不自动连接**（刻意设计，防误驱真机）。

### 7.2 一键启动（前端 + 后端）

```bat
cd MeArm-3D
start.bat              # 前置检查 + 端口探测 + 起前后端 + 打印本机/局域网地址
```

### 7.3 驱动真实机械臂

```bat
REM 1) 烧录固件（Uno 接在 COM4）：预检 → 编译 → 烧录；缺依赖会先一次列全再拒跑
cd MeArm-Device && start.bat flash COM4

REM 2) 用真机配置启动关节级后端（默认 config.yaml 是 sim，不会碰硬件）
cd ..\MeArm-3D\backend && go run . -c config.serial.yaml

REM 3) 前端 Connection 面板 → ws://localhost:8090/ws/joint → Connect → 「关节控制」点 Real Robot
```

> 点 Real Robot 时后端会**核对链路末端**：末端不是 `serial` 就**拒绝切换并给出原因**（不留静默）。

### 7.4 物理仿真（MuJoCo）

```bash
# 起一个 MuJoCo 末端（Python 子进程由 Go 侧拉起，走同一套文本协议）
python MeArm-3D/simulation/mujoco/server.py
# 前端零改动连上即可；hello.simulation_mode = "mujoco"
```

### 7.5 遥控台（舵机级摇杆台，串口 / 网络双模式）

**默认：网络模式** —— 先把 MeArm-3D 起起来（见 §7.1 / §7.2，Sim 即可），再：

```bat
cd MeArm-RemoteControl
start.bat            # 默认 network：不打开串口，TCP Client 连 MeArm-3D(:9100)
# 浏览器 http://127.0.0.1:9001
```

**串口模式**（驱动真机，机械臂会动）：

```bat
cd MeArm-RemoteControl
start.bat --real     # 等价 --serial：打开串口走 arm-device 协议
# 浏览器 http://127.0.0.1:9001    串口模式下另开局域网透传 telnet <IP> 9002
```

`start.bat` 会自动在源码比二进制新时重建（未知参数报错退出，`--help` 看用法）。
只想编译不动模式：`build.bat`（Windows）/ `bash build.sh` / `make`。

端到端自检（不需要浏览器）：

```bat
node tools/e2e-sim.js           # 串口模式：摇杆帧 / 死区零流量 / 角度解析 / TCP 转发
node tools/sim2sim-check.mjs    # 网络模式：30 项断言（Sim2Sim）
```

### 7.6 校验机器人包 / 重新生成产物

```bash
cd MeArm-3D
python core/python/robopkg/cli.py validate --all    # 包契约（路径 / 能力 / 生成器是否真的存在）
python core/python/robopkg/cli.py show mearm-v1     # 摊开推导结果（dof / qpos / 执行器 / 内容哈希）

# 改过真值后必须重新生成产物，否则「与配置同步」的测试会失败
python robot-package/mearm-v1/tools/gen_model.py    # → simulation/mujoco/mearm.xml
python robot-package/mearm-v1/tools/gen_urdf.py     # → robot-package/mearm-v1/urdf/mearm-v1.urdf
```

---

## 8. 仓库结构

```
MeArmPilot/
├── README.md                     # 本文件（总览）
├── LICENSE                       # MIT
├── docs/                         # 平台级文档（开发提示词记录等）
├── MeArm-Device/                 # ① ATmega328P 裸机固件
│   ├── bsp/                      #    硬件驱动（uart / servo / led / adc / ir / eeprom / systick）
│   ├── core/                     #    应用（arm_control / cmd / joystick / ir_ctrl / ir_seq / main）
│   └── tools/                    #    host_verify.py（上位机指令回归）+ NEC 解码单测
├── MeArm-RemoteControl/          # ② 串口 / TCP → Web 网关（默认 network，--real 走串口）
│   ├── start.bat                 #    一键启动：选模式（默认 network / --real 串口 / --help）
│   ├── internal/                 #    config / protocol / serial / hub / tcp / web
│   │                             #    + netlink（TCP Client：单写泵 · latest-wins · 退避重连）
│   │                             #    + link（Link 抽象：serialLink / netLink 两实现）
│   ├── web/static/               #    内嵌前端（three.js 双摇杆 + 角度面板 + 回显终端 + 网络面板）
│   └── tools/                    #    e2e-sim.js · tcp-test.js · sim2sim-check.mjs · headless-joystick-test.js
└── MeArm-3D/                     # ③ 数字孪生 + 关节级后端 + 物理仿真
    ├── config/                   # robots.yaml —— 只放「选择器指针」（默认机器人 id + 文件路径）
    ├── robot-package/            # ★ 每台机器人一个包：真值 / 运动学 / 物理 / 测试 / 工具 / URDF
    │   └── mearm-v1/             #   manifest.yaml（身份 · 能力声明 · 指针）
    │                             #   model/robot.yaml（运动学真值）· physics/physics.yaml（物理量真值）
    │                             #   kinematics/（engine.ts + ik.ts）· urdf/mearm-v1.urdf（产物）
    │                             #   3d-structure/mearm3Dasm.STEP（几何真值）· tests/（用例 JSON + 包内测试）
    │                             #   tools/（20 个：生成 / 校验 / 相机标定 / 真机操作）
    ├── core/                     # Core 层（不属于任何机器人）：python/robopkg（manifest 解析 ·
    │                             #   declared_path 路径解析 · 语义内容哈希 · 校验 CLI）· baseline · tools
    ├── frontend/                 # React 19 + Three.js（robot / components / store / tests）
    ├── backend/                  # Go 关节级服务（cfg / robot / protocol / controller / device / wsserver）
    ├── simulation/mujoco/        # MJCF（产物）+ 仿真服务 + Viewer + 记录
    ├── tests/sim/                # MuJoCo 轨验收（pytest）
    └── docs/                     # 坐标系 / 模型结构 / 真机实测 / ADR 决策 / STEP 反解 / 安全参数 / 采集指南
```

> ⚠️ **真值随包走**：`config/` 只剩**选择器**（`robots.yaml` 只有指针，没有数值）；
> 运动学 / 标定 / 限位在 `robot-package/<id>/model/robot.yaml`，
> 物理量在 `robot-package/<id>/physics/physics.yaml`。
>
> ⚠️ **产物与真值分开**：`urdf/` 与 `simulation/mujoco/*.xml` 都是**生成物**（不进内容哈希），
> 改真值必须重新生成，否则「与配置同步」的测试会失败。

---

## 9. 能力边界（诚实声明）

本项目刻意**不夸大**能力，以下限制在文档与 UI 中均显式写出：

- **舵机没有位置回读**。`OK SET` / `STATUS` / 后端 `joint_state` 回传的都是**目标值**
  （「我打算去哪」），**没有一条能证明「它实际上在哪」** —— 哪怕机械臂卡死在桌上，回执依然一字不差。
  唯一的外部地面真值是**相机**。
- 因此「链路误差面板」反映的是**链路时延与限位截断**，不是「物理臂到位没有」；
  「幽灵臂」证明**链路走通了**，**不**证明物理到位。
- **物理仿真的 Level 声明机器可检查**（`calibrated == false`）：当前是「参数化物理」，
  尚未做真机逐点标定，目标是 3 → 4 而非 5。
- **球壳安全参数是「前置检查 + 求解覆写」，不是物理限位**：内径只在前置检查里拒单
  （`ik.ts` 里 `reachMin` 恒为求导值、不可覆写）；外径既做前置检查、**也真正传进 `ik.ts`**
  参与 `cosAlpha` 求解。两者都不改变几何，只是把「拒绝」或「夹紧」的位置挪一挪。
- **UI 的 1–160 mm 是几何球壳范围，不是有效工作域**：关节限位把它裁到
  `d ∈ [44.17, 139.27]`，所以外径填 150 / 160 **等于没有覆写**。
- **肩标定增益偏差 −13.1% 未解决**，且跨批测量不可复现（详见 §6.3）；
  夹爪标定方向已于 D80 依真机实测修正。
- **遥控台的网络模式只在 Sim 上验证过**：`arm-web(network) ⇄ MeArm-3D(sim)` 端到端 30/30；
  `MeArm-3D --real`（真机）+ network 模式、以及网络模式下的真机串口链路**未做真机验证**。
  串口模式（`start.bat --real`）的既有结论不受影响，但本次也**未接真机复测**。
  > 对端页面的表现已单独验证（2026-09-17）：`arm-web` 驱动时 MeArm-3D 页面**主臂与幽灵一起动**
  > （`|tcp − actualTcp| = 0.000 mm`，渲染侧两棵树 TCP 世界坐标重合 0.000 mm）。
  > 修复记录见 `MeArm-3D/docs/decisions.md` **D84**；判据脚本
  > `MeArm-3D/frontend/tests/e2e/external-origin-follow.mjs`（**9/9**）与
  > `MeArm-3D/core/tools/verify_external_origin.mjs`（**3/3**）。

---

## 10. 文档索引

| 想了解 | 去看 |
|---|---|
| 平台构成 / 铁律 / skill 路由 | 本文件 + [`.workbuddy/memory/MEMORY.md`](.workbuddy/memory/MEMORY.md) |
| 固件指令协议与内存铁律 | [`MeArm-Device/README.md`](MeArm-Device/README.md) |
| 串口网关 / ACK 门控 / 性能 | [`MeArm-RemoteControl/README.md`](MeArm-RemoteControl/README.md) |
| 数字孪生全貌与阶段验收 | [`MeArm-3D/README.md`](MeArm-3D/README.md) |
| 机器人包契约（真值 / 能力 / 指针） | [`MeArm-3D/robot-package/mearm-v1/manifest.yaml`](MeArm-3D/robot-package/mearm-v1/manifest.yaml) |
| 包重构的分层边界与搬迁纪律 | [`MeArm-3D/docs/architecture/robot-package-phase0.md`](MeArm-3D/docs/architecture/robot-package-phase0.md) · [`phase1`](MeArm-3D/docs/architecture/robot-package-phase1.md) · [`phase2`](MeArm-3D/docs/architecture/robot-package-phase2.md) |
| 坐标系 / 单位 / 运动学链 | [`MeArm-3D/docs/coordinate-system.md`](MeArm-3D/docs/coordinate-system.md) |
| 模型结构（几何 vs 运动学边界） | [`MeArm-3D/docs/model-structure.md`](MeArm-3D/docs/model-structure.md) |
| 3D 结构（STEP）反解与运动学验证 | [`MeArm-3D/docs/STEP_KINEMATICS_VALIDATION.md`](MeArm-3D/docs/STEP_KINEMATICS_VALIDATION.md) |
| 外观参数从哪里来 | [`MeArm-3D/docs/APPEARANCE_PARAMETERS.md`](MeArm-3D/docs/APPEARANCE_PARAMETERS.md) |
| 末端目标安全参数（球壳内/外径、容差） | [`MeArm-3D/docs/target-guard-analysis.md`](MeArm-3D/docs/target-guard-analysis.md) |
| 工作空间边界实测（有效域怎么量出来的） | [`MeArm-3D/docs/workspace-boundary.md`](MeArm-3D/docs/workspace-boundary.md) |
| 真机实测记录与不确定度 | [`MeArm-3D/docs/hardware-measurement.md`](MeArm-3D/docs/hardware-measurement.md) |
| 设计决策 ADR（D1–D83） | [`MeArm-3D/docs/decisions.md`](MeArm-3D/docs/decisions.md) |
| 物理仿真怎么跑 / 判据纪律 | [`MeArm-3D/simulation/README.md`](MeArm-3D/simulation/README.md) |
| 物理量真值（质量 / 摩擦 / 惯量） | [`MeArm-3D/robot-package/mearm-v1/physics/physics.yaml`](MeArm-3D/robot-package/mearm-v1/physics/physics.yaml) |
| 串口 / WS 协议基线 | [`MeArm-3D/docs/serial-v1.md`](MeArm-3D/docs/serial-v1.md) |
| TCP 控制协议基线（JSON Lines：`move`/`gripper`/`servo`/`state`） | [`MeArm-3D/docs/tcp-control-v1.md`](MeArm-3D/docs/tcp-control-v1.md) |
| 图像采集指南 | [`MeArm-3D/docs/texture-capture-guide.md`](MeArm-3D/docs/texture-capture-guide.md) |
| 开发提示词记录 | [`docs/`](docs) |

---

## 11. License

[MIT](LICENSE) © 2026 听心跳的声音
