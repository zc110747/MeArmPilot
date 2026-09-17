# arm-web · 基于 Go 的机械臂控制服务（串口 / 网络双模式）

把机械臂的控制能力封装成本机 Web 页面（双 3D 摇杆）与局域网 TCP 透传。
**两种传输模式，由 `start.bat` 或 `-mode` 选择**：

| 模式 | 链路 | 说明 |
| --- | --- | --- |
| `serial`（默认） | 网页 → arm-web → **串口** → `arm-device` 固件 | 原有真机链路，行为与引入网络模式前**逐字节一致** |
| `network` | 网页 → arm-web → **TCP Client** → `MeArm-3D` → Sim/Real | 新增：不打开串口，走 JSON Lines 连 MeArm-3D |

```
# serial（默认；`start.bat --real`）
 arm-device (串口 COMx / 115200 8N1)
    ▲      ▼
[ Serial 管理 + 自动重连 + ACK 门控 ]   ← internal/serial
    ▲      ▼
[   Hub 广播总线（设备回显 → 多端）  ]  ← internal/hub
    ▲                    ▲
[ TCP 局域网透传 ]     [ Web + WebSocket 双 3D 摇杆 ]  ← internal/tcp  internal/web

# network（`start.bat` 默认；不开串口、不开 9002）
 MeArm-3D TCP 控制接口（JSON Lines，默认 127.0.0.1:9100）
    ▲      ▼
[ netlink.Client：退避重连 + 单写泵 + 每轴 latest-wins ]  ← internal/netlink
    ▲      ▼
[   Hub 广播总线  ]
    ▲
[ Web + WebSocket 双 3D 摇杆 ]  ← internal/web（与串口模式同一份页面）
```

两条链路共用**同一套网页**与**同一个 `link.Link` 接口**（`internal/link`）；
能力差异通过 WS 的 `caps` 消息显式告知页面（不支持的按钮置灰并给出原因）。
网络模式下 arm-web **不做** IK/FK、坐标变换、标定换算、软限位 —— 那些都是
MeArm-3D 的职责（见下文"网络模式"一节）。

> `serial` 模式下，串口指令语法严格依赖 arm-device 协议（见 `arm-device/README.md`
> 与 `core/cmd.c`），所有控制意图最终都归一化为 arm-device 指令文本再下发，
> 保证与固件行为完全一致。


## 功能

1. **串口层（internal/serial）**
   - 纯标准库实现：Windows 走 `syscall` 直连 `kernel32`，Linux/macOS 走 `stty` + 文件。
   - 串口未连接 / 意外断开时**自动重连**（间隔可配）。
   - **命令-应答（ACK）门控**：同一时刻仅 1 条指令在途；摇杆走 latest-wins 通道、
     离散指令走 FIFO 通道（详见下文）。
   - **连接静默窗口 + 暖机包**：连接建立后等 `connect_settle_ms`（默认 2500ms）让
     Uno bootloader 交权，结束后先发一个 `\n` 暖机包（被链路吞掉的首包），再下发
     真实指令——解决"开机指令无应答 / 首条指令被吞"两大 Uno 串口陷阱。
   - **高性能读模式**：Windows 串口用「立即返回」读超时（MAXDWORD 三元组）+ 手动
     行缓冲（不用 bufio，杜绝空闲 20s 断连），命令-应答门控单条周期 ~13ms，
     WebSocket 端到端延迟 ~10ms（旧实现 218~932ms，根因是 Windows 非重叠 I/O
     读写互斥，见下文"性能"一节）。
   - `log_level: "debug"` 时打印每条 TX / RX 行及时间戳，便于定位链路时序问题。
   - 线程安全：`WriteLine` 可被 Web / TCP 多 goroutine 并发调用。
2. **TCP 局域网转发（internal/tcp）**：其它设备（或后期远程客户端）以 raw TCP 连接
   本端口，直接下发 arm-device 指令（换行结束），服务器原样转发串口；设备回显原样
   写回。为**远程控制预留统一接口**（见下文）。
3. **Web 本机控制（internal/web + Three.js）**
   - **双 3D 摇杆（遥控形式）**：左摇杆 X→底座(S9)、Y→左舵(S8)；右摇杆 X→夹取(S6)、
     Y→右舵(S7)。拖拽产生归一化 (x,y)∈[-1,1]，经 WebSocket 下发；松手回中即停，
     **按住偏转持续步进**（30ms 心跳，与硬件摇杆扫描节奏一致）。
   - **动作死区**：偏移 ≤`deadband_deg`（默认 5°，视觉倾角满偏约 31.5°）的轴不下发；
     四轴全部在死区内时整帧 JOY 都不发（串口零流量）；一旦超过立即进入有效步进区间。
   - **舵机角度面板**：S6~S9 实时角度，从所有含角度的应答（STATUS / SET / JOY）
     **合并更新**（单轴应答不清零其它轴）；STATUS 仅在点击时下发查询（不做周期轮询）。
   - **串口回显终端**：**30 行滑动窗口**，始终自动滚动显示最新一行。
   - **建模模式**：已在导航栏占位（disabled），待摇杆模式完成后继续规划开发。
4. **网络链路（internal/netlink + internal/link，仅 `mode: network`）**
   - 作为 **TCP Client** 连接 MeArm-3D 的 TCP 控制接口（JSON Lines，默认
     `127.0.0.1:9100`），把网页的操作意图翻译成 MeArm-3D 已有的
     `servo` / `move` / `gripper` / `state` 命令。
   - **单一写泵**：所有写只发生在一个 goroutine 里，上层只往队列放"意图"
     ⇒ 同一连接上 JSON 不可能交叉。
   - **绝不无限堆积**：摇杆按舵机维度 latest-wins，离散命令走有界 FIFO（满则丢最旧）。
   - **时间驱动的角度积分**：网络模式摇杆发的是 MeArm-3D 的**绝对角** `servo` 命令，
     增量按**真实经过的 dt** 积分 ⇒ 前端发送频率高低只影响"多久下发一次"，
     不影响速度。
   - **指数退避重连**（500ms → 5000ms 封顶）+ 连接建立后自动取 `state` 建立基准；
     服务端不认识 `state` 时**明确降级**到配置兜底角并上报，不静默猜。
   - **被拒即回滚**：服务端因越界拒绝某轴时，该轴本地目标回滚到服务端确认值；
     若回滚失效，后续每一帧都会被拒、表现为"推杆完全不动"（这是最难查的故障）。
   - 断线后**不自动重发**被中断的命令，也不在关闭后继续写（`Close()` 先掐连接
     再等 goroutine 退出）。

## 网络模式（TCP 接入 MeArm-3D）

`mode: network`（`start.bat` 的默认）下 arm-web **不打开串口**，而是作为 **TCP Client**
连 MeArm-3D 的 TCP 控制接口（JSON Lines，默认 `127.0.0.1:9100`）：

```
网页摇杆/面板 → WebSocket → arm-web(internal/netlink) → TCP → MeArm-3D tcpserver
                                                            → Controller → Sim / Real
```

网页操作到 MeArm-3D 命令的映射（**只用对方已有的命令，不新增协议**）：

| 网页操作 | 下发命令 | 语义 |
| --- | --- | --- |
| 左摇杆 X / Y | `servo` | 底座 / 左舵的**绝对角**（增量按真实 dt 积分后再发） |
| 右摇杆 X / Y | `servo` | 夹取 / 右舵的绝对角 |
| XYZ 面板 ± | `move` | `{"axis":"x|y|z","direction":"+|-","step":N}`（毫米） |
| 爪 开 / 合 | `gripper` | `{"action":"open"\|"close"}` |
| 四舵机滑条 | `servo` | 直接下发绝对角（1..N 为 MeArm-3D 侧下标） |
| 刷新 / 接入时 | `state` | **只读**查询：当前舵机角 + 末端 TCP 坐标 + device(sim/real) |

### 三个容易搞错的地方

1. **接线表只有一份**：`network.servo_ids: [9, 7, 8, 6]` 同时决定"摇杆推哪个舵机"和
   "面板显示哪个舵机的角"。顺序即 MeArm-3D 侧 `servo` 数组的 1..N 下标
   （其 `Model.JointOrder()` = base / shoulder / elbow / gripper）。
   改这一行 = 同时改两件事，别在别处再抄一份。
2. **方向语义跨模式必须一致，但 `invert_*` 不能直接复用**：串口链路的 `invert_*` 里
   含一层 **arm-device 固件方向补偿**（`MeArm-Device/core/joystick.c`：8 轴出厂即反相，
   9/6/7 轴 `raw>800 → 负步长`）。TCP 链路没有固件层，照抄会让两种模式推杆方向**正好相反**。
   故由 `internal/protocol.NetInvertFor(armServoID, inv)` 做一次换算（8 轴保持、其余取反），
   单测 `main_test.go` 用"满偏 dt=1s 的增量符号"锁住这个不变量。
3. **基准（当前舵机角）与 `state`**：MeArm-3D 的 TCP v1 原有命令全是写命令，拿不到
   "当前角"，首次摇杆动作会失去参照。本次为它新增**只读** `state`（不改 controller /
   device / 几何 / IK / MJCF / WS / 前端）。若对端不支持 `state`，本服务**不猜**：
   退回 `network.fallback_angle`（90° = mearm-v1 HOME 的换算值）起步，并在界面显式上报。

> 越界由 MeArm-3D 依 `robot.yaml` 判定并回 `error`；本服务收到后把该轴**本地目标回滚**到
> 服务端确认值。`angle_min/angle_max` 只是"明显越界"的本地前置拒绝，**不是软限位**。

## 更新记录

### 2026-09-17 · TCP 接入 MeArm-3D

| 更新 | 说明 |
| --- | --- |
| 网络模式 | 新增 `mode: network`：作为 TCP Client 连 MeArm-3D，**保留** `serial` 模式且行为逐字节不变 |
| 一键启动 | 新增 `start.bat`：默认 network、`--real`/`--serial` 走串口、未知参数报错退出（2） |
| 摇杆方向换算 | 串口 `invert_*` 含固件方向补偿，TCP 无该层 ⇒ 新增 `NetInvertFor` 保证两模式推杆同向 |
| 状态基准 | 新增只读 `state` 命令依赖；取不到时降级 `fallback_angle` 并显式上报（不猜） |
| 网页面板 | 新增 XYZ 步进 / 爪 open·close / 四舵机滑条 + TCP 末端坐标回读；不支持的按钮置灰并给原因 |
| 回滚可观测 | 越界被拒 → 该轴本地目标回滚，脚本用"之后还能不能驱动该轴"验证（唯一判据） |
| Sim2Sim 验收 | 新增 `tools/sim2sim-check.mjs`：30 项端到端断言，期望值全部从 `config.yaml` + `robot.yaml` 派生 |

### 2026-09-06 · 串口层性能与摇杆语义

| 更新 | 说明 |
| --- | --- |
| 串口性能根治 | Windows 非重叠 I/O 读写互斥导致每条命令 218~560ms、丢帧 75%；改「立即返回」读模式后门控周期 **~13ms**、WS 端到端 **10ms**、受控流 40/40 |
| 空闲断连修复 | 弃用 bufio 手动行缓冲，修复空闲 20s 必断连（bufio 对 `(0,nil)` 空读抛 `ErrNoProgress`）导致舵机莫名归位的问题 |
| 双摇杆方向/死区 | 方向统一为"推杆=角度增大"（`invert_*` 可配）；动作死区默认 **5°**，四轴全居中零流量 |
| 按住持续步进 | 拖拽中 30ms 心跳重发当前位，对齐硬件摇杆"按住持续累加"语义；松手停+回中 |
| 角度面板 | 点击 STATUS 查询 + 从 OK JOY/SET 应答合并刷新（不做周期轮询——会饿死摇杆流） |
| 回显终端 | 30 行滑动窗口 + 始终显示最新 |
| 舵机脉宽标定(固件) | `angle_to_ticks` 对齐 Arduino Servo.h 默认 544–2400µs，修复"实际角度只有一半" |

## 配置（YAML，全部可改）

`config.yaml`（复制 `config.yaml.example` 修改）：

```yaml
mode: "serial"        # serial(默认) / network；start.bat 会用命令行 -mode 覆盖本值
serial:
  port: "COM16"       # Windows: COMx ；Linux/macOS: /dev/ttyUSB0（以你机器实际端口为准）
  baud: 115200        # 与 arm-device 固件一致（U2X 模式）
  databits: 8
  stopbits: 1
  parity: "N"         # N / E / O
  reconnect_sec: 3    # 断线重连间隔(秒)
  min_interval_ms: 0  # ACK 门控已防冲刷，默认不额外节流
  ack_timeout_ms: 800 # 应答超时：超时判通讯失败（不自动重发）
                      # 另有 connect_settle_ms（默认 2500）：Uno 开串口会复位，等 bootloader 交权
web:
  enabled: true
  host: "0.0.0.0"     # 绑定 IP；0.0.0.0 = 本机所有网卡（含局域网）
  port: 9001          # 浏览器访问 http://<本机IP>:9001
  ws_path: "/ws"
tcp:                  # ⚠️ 仅 serial 模式生效（network 模式无串口可转发，9002 不监听）
  enabled: true
  host: "0.0.0.0"     # 局域网可访问
  port: 9002          # 其它设备 telnet/raw 连接此端口即可下发 arm-device 指令
network:              # ⚠️ 仅 mode: network 生效：TCP Client 连 MeArm-3D 的 TCP 控制接口
  host: "127.0.0.1"   # MeArm-3D TCP Server 地址（同机回环；跨机填对方 IP）
  port: 9100          # MeArm-3D TCP 端口（见其 config.yaml 的 tcp.port）
  servo_ids: [9, 7, 8, 6]   # 唯一接线表：顺序即 MeArm-3D 侧 servo 数组下标(1-based)
  deadband_deg: 5           # 摇杆死区(度)；<=该值视为居中
  min_speed_deg_per_s: 12   # 刚出死区时的角速度
  max_speed_deg_per_s: 90   # 满偏(31.5°)角速度上限
  tick_ms: 25               # 积分节流；不满该间隔的帧被合并(latest-wins)
  max_tick_ms: 200          # 单次积分 dt 上限（兜底）
  connect_timeout_ms: 3000
  request_timeout_ms: 1000
  reconnect_min_ms: 500     # 断线重连退避下限
  reconnect_max_ms: 5000    # 退避上限（不做无限快速重连）
  queue_size: 64            # 离散命令队列；满则丢最旧
  auto_sync: true           # 连上后自动发 state 建立基准
  fallback_angle: 90        # 拿不到基准时的起步角（= mearm-v1 HOME 的换算值）
  angle_min: 0              # 本地前置拒绝边界（**不是软限位**）
  angle_max: 180
joystick:             # 网页双摇杆 -> 舵机映射（方向语义：推杆 = 角度增大）
  lx_servo: 9         # 左摇杆 X -> 底座(S9)
  ly_servo: 8         # 左摇杆 Y -> 左舵(S8)
  rx_servo: 6         # 右摇杆 X -> 夹取(S6)
  ry_servo: 7         # 右摇杆 Y -> 右舵(S7)
  invert_lx: true     # 9/6/7 轴固件语义 raw>800->负步长，默认 invert 对齐"推杆=角度增大"
  invert_ly: false    # 8 轴固件出厂即反相，保持 false
  invert_rx: true
  invert_ry: true
  deadband_deg: 5     # 动作死区(度)：偏移<=该值不下发；四轴全居中时整帧跳过(串口零流量)
log_level: "info"     # debug 可看每条 TX/RX 时序
```

> 串口、IP 地址、端口均可在 YAML 中配置；修改后重启生效。
> `mode` 只是"直接跑 `arm-web.exe` 时"的默认值，`start.bat` 一定用 `-mode` 覆盖它。

## 构建与运行

> 本工程**零外部依赖**（仅用标准库 + 已缓存的 `gopkg.in/yaml.v3`），可离线构建。
> 前端 `web/static` 在编译期经 `//go:embed` 编入二进制，产物为**自包含单文件**，
> 运行时不再依赖磁盘上的 `web/static` 目录。因此修改前端后需重新构建。

### 一键启动（推荐）

`start.bat` 负责选模式，并把结果作为一个 `-mode` 参数交给程序
（**YAML 不会被脚本改写**；配置里的 `mode` 只是"直接跑 exe 时"的默认值）：

| 命令 | 模式 | 说明 |
| --- | --- | --- |
| `start.bat` | **network** | 默认。TCP Client 连 MeArm-3D，**不打开串口** |
| `start.bat --real` | serial | 打开串口控制真机（**机械臂会动**） |
| `start.bat --serial` | serial | 同 `--real` |
| `start.bat --network` | network | 显式指定（等同默认） |
| `start.bat -c FILE` | — | 用别的 YAML 配置（默认 `config.yaml`） |
| `start.bat --build` / `--no-build` | — | 强制重建 / 跳过重建检查 |
| `start.bat --help` | — | 用法 |

`start.bat` 还会在源码比二进制新时自动重建（`web/static` 经 `go:embed` 内嵌，
**改了前端也必须重编**）。注意它比较的是 `%%~tT` 时间戳，**只有分钟精度** ——
同一分钟内的源码改动可能被判为"已最新"；此时用 `start.bat --build` 强制重建。

> ⚠️ **`start.bat` 不会杀已有实例**。若已有一个 `arm-web` 占着 Web 端口，新实例的 Web
> 监听会失败但**进程不退出**（`main.go` 里 listen 失败只打日志，见 `[web] 服务失败: ... bind ...`），
> 于是你得到"多了一个 `arm-web.exe`，页面却还是旧那个在服务"——而且重建时旧二进制会被
> 改名为 `arm-web.exe~`。启动前先确认端口空闲（任务管理器里 `arm-web.exe` 只应有一个）。

> ⚠️ `start.bat --real` 里的 `--real` 指"**本服务**走串口"，
> 与 MeArm-3D 的 `--real`（"**那个服务**驱动真机"）是两回事，别混。

### 一键编译（在 arm-web 目录内运行）

| 平台 | 命令 |
| --- | --- |
| Windows | `build.bat` |
| Linux / macOS | `bash build.sh` |
| 通用（有 make） | `make`（`make test` 跑测试，`make clean` 清产物） |

脚本统一执行 `go vet ./...` + `go build`，产物跨平台命名：`arm-web.exe`（Windows）/ `arm-web`（其它）。

```bat
# Windows（在 arm-web 目录）
build.bat
arm-web.exe -c config.yaml
```

```sh
# Linux / macOS
bash build.sh
./arm-web -c config.yaml
```

启动后：
- 浏览器打开 `http://<本机IP>:9001`（本机可用 `http://127.0.0.1:9001`）。
- **串口模式**下，局域网内其它设备：`telnet <本机IP> 9002`，直接下发 `SET 9 120`、`RESET`、`STATUS` 等指令。
  （网络模式不监听 9002 —— 控制意图走 WS，不要再指望这个透传口。）
- 网络模式下页面会自动连 MeArm-3D；若对端没起，顶栏链路状态点会红并显示原因，服务器照常运行并退避重连。

串口未连接时服务器照常启动，并在后台持续重连；连上设备后即可正常控制。

### 网页串口连接状态（重要）

顶栏有两个状态点：**WebSocket**（浏览器↔服务器）与**串口**（服务器↔arm-device）。
串口点变绿 = `串口: 已连接`；变红 = `串口: 未连接（<原因>）`。状态在客户端接入时
立即同步，之后每次翻转都会主动推送，无需刷新页面。

**若串口显示未连接：**
- 提示含“端口可能被其它程序占用”→ 关闭占用该 COM 的程序（串口助手 / 下载工具 /
  另一个 arm-web 实例）后服务器会自动重连（已对端口加共享打开，多数情况可并存）。
- 提示含“系统找不到指定的文件”→ `config.yaml` 的 `serial.port` 写错了 COM 号，
  在设备管理器中核对实际 COM 号后修改。
- 确认 arm-device 已上电并插好 USB；COM 号以 `serial.port` 实际值为准（本仓库默认 `COM16`）。

> **network 模式**下，第二个状态点显示的是**链路**（服务器 ↔ MeArm-3D，TCP），
> 不是串口 —— 文案由后端 `caps.mode` 决定，同一个页面不会说谎。

### 用网页摇杆控制前的建议

固件默认启用**硬件摇杆扫描**（`g_enabled=true`）。用网页摇杆前，先点页面上的
**“摇杆硬件关”**（下发 `JOYHW OFF`），让固件忽略物理摇杆、只响应网页 `JOY` 指令；
控制结束点 **“摇杆硬件开”**（`JOYHW ON`）恢复。若不关硬件摇杆，物理摇杆若不在中位
会与网页指令互相“打架”。

> 以上都是 **serial 模式**的注意事项。network 模式下没有固件可发，`JOYHW` 这类
> arm-device 指令按钮会被**置灰并标注原因**（`caps.arm_device_cmds == false`），
> 硬件摇杆是否在中位与网页无关 —— 控制的是 MeArm-3D 的 Sim/Real，不经过 Uno。

拖拽网页摇杆（双摇杆，遥控形式）：左摇杆 X→底座(S9)旋转、Y→左舵(S8)俯仰；
右摇杆 X→夹取(S6)、Y→右舵(S7)。方向语义统一为**推杆方向 = 舵机角度增大**
（个别轴与机构实际方向相反时改 `invert_*`，无需改代码）。

**动作死区 5°**（视觉倾角，满偏约 31.5°）：偏移 ≤5° 的轴不下发；**四轴全部 ≤5°
时整帧 JOY 都不发**（串口零流量）；一旦超过 5° 立即进入有效步进区间（跨过固件
raw 200/800 阈值），偏移越大步进越快（2~10°/次）。**按住偏转持续步进**（30ms
心跳，与硬件摇杆扫描节奏一致），松手即停。死区可用 `joystick.deadband_deg` 调整。

右侧"舵机角度"面板显示 S6~S9 角度：**仅在点击 STATUS 按钮时下发查询**（不做周期
轮询——周期轮询会与 JOY 抢命令-应答门控导致摇杆失效），并从所有含角度的应答
（STATUS / SET / JOY）合并更新；拖拽摇杆期间每次 `OK JOY` 应答都携带四轴角度，
角度面板会随动刷新。

串口回显终端为 **30 行滑动窗口**，始终自动滚动到最新一行。

## 流量控制与命令-应答门控

早期版本在快速拖拽摇杆时会把命令 FIFO 队列瞬间灌满，出现 `命令队列满，丢弃`；且无法感知
下位机是否真正收到。现采用**命令-应答（ACK）门控**彻底解决：

1. **应答门控（一次一条在途）**：每条指令发出后，必须等到下位机回送应答行（arm-device 对
   每条合法指令都会回 `OK ...` 或 `ERR ...`，即作为应答），才能发下一条。同一时刻只有 1 条
   指令在途，从根本上杜绝“命令队列满”。
2. **中间摇杆位置合并（latest-wins）**：拖拽过程中连续的摇杆位置只保留**最新值**，不排队连发；
   离散指令（`SET`/`RESET`/`STATUS` 等）按 FIFO 逐条下发（受应答限速，但序列不丢）。
   摇杆通道容量 1、离散通道容量 16，永不堆积。
3. **串口下发节流**：每条指令之间保持 `min_interval_ms` 最小间隔（默认 0——ACK 门控已防冲刷）。
4. **通讯失败可显示**：若下位机在 `ack_timeout_ms`（默认 800ms）内未应答，判定**通讯失败**，
   丢弃待发、不再自动重发（满足“不能再次下发”），并通过 `serial_status` 推送界面，顶栏状态点
   变琥珀色并显示“串口: 通讯失败”。

> 设计要点：串口层 `ackPump` 统一管理“取待发 → 写出 → 等应答/超时 → 取下一条”，Web/TCP 只
> 负责把意图 `WriteLine` 入对应缓冲，不再各自处理时序。逻辑由 `internal/serial/ack_test.go`
> 覆盖：门控、latest-wins 合并、应答超时→通讯失败、异步事件(`# ` 行)不误判应答、空闲 `SEQ STOP`
> 仍能正确应答，均有单测验证。

**应答行判定规则**：下位机回显中，**任何非 `# ` 开头的行都视为命令应答**（用于清除“等待应答”）；
以 `# ` 开头的行是**异步事件**（硬件红外回显 `# IR RAW=...`、序列被摇杆中断 `# IRSEQ stop: joystick` 等），
只转发到日志、不推进门控。下位机（arm-device）已适配：每条指令都回 `OK ...`/`ERR ...`，
且 `SEQ STOP` 在序列空闲时回 `OK IRSEQ idle (not running)`，避免门控误判通讯失败。

> 修复记录：`internal/serial/windows_serial.go` 的 `openPort` 曾因 `DataBits` 缺省为 0 导致
> `SetCommState` 报“参数不正确”而打不开串口；现已在 `byteSize<5||>8` 时回退为 8 位。

**串口性能（重要，Windows 特有）**：CH340 等适配卡在**非重叠 I/O** 下读写互斥——若读循环
阻塞在 `ReadFile`（带总超时），并发的 `WriteFile` 必须等读返回，门控每条命令会被拖到
数百毫秒且大量丢帧。现配置为「立即返回」读模式（`readIntervalTimeout=MAXDWORD`、
`readTotalTimeoutMultiplier=MAXDWORD`、`readTotalTimeoutConstant=0`），`ReadFile` 立即带回
缓冲现有字节（无数据返回 0 不报错，readLoop 视为"暂无数据"），空读时 sleep 2ms 节流。
实测：门控单条周期 ~13ms、WS 端到端 ~10ms、50ms 节奏受控流 40/40 全部下发。

**真实硬件集成测试**（`arm-device` 已烧录并接在 COM4 时可直接跑，验证 web 侧门控与真实固件联动）：

```bat
# 需先在 arm-device 目录用 scripts/build_upload.bat 烧录适配后的固件
REAL_COM=COM4 go test ./internal/serial/ -run TestRealHardwareAck -v
# 该测试会：下发 RESET/SEQ STOP/连续 JOY，断言收到固件应答、空闲 SEQ STOP 不误判通讯失败
```

## 验证（无需 Web 界面）

**串口模式** —— 不跑浏览器也能验证“TCP 客户端 → 服务器 → 串口”链路：

```bat
# 终端 1：启动服务器（需真机固件在 COM16；debug 日志可见每条 TX/RX 时序）
arm-web.exe -c config.yaml

# 终端 2：端到端仿真（WS 摇杆帧 / 死区零流量 / 角度解析 / TCP 转发全链路）
node tools/e2e-sim.js

# 或仅测 TCP 透传
node tools/tcp-test.js 127.0.0.1 9002
```

`tools/e2e-sim.js` 会连真实运行的 arm-web 服务，断言：死区内坐标零 `OK JOY` 流量、
出死区恢复下发、`STATUS` 应答携带 angles、TCP 转发一致性。`tools/tcp-test.js` 依次发送
`STATUS` / `JOY ...` / `S9=120` / `JOYHW OFF` / `RESET` 并连发 25 条 `JOY` 压测。

固件侧全指令回归（pyserial）：`arm-device/tools/host_verify.py COM4 115200` → 期望 67/67 PASS。

**网络模式** —— Sim2Sim 端到端（`tools/sim2sim-check.mjs`，零依赖 Node 脚本，仍不需要浏览器）：

```bat
# 终端 1：MeArm-3D 后端（Sim 模式，TCP 9100）—— 在 MeArm-3D 目录按其 README 启动

# 终端 2：arm-web 以 network 模式运行（默认就是 network）
start.bat

# 终端 3：端到端断言
node tools/sim2sim-check.mjs        # 期望 30 PASS / 0 FAIL
```

该脚本**不硬编码任何期望值**：自己读 `config.yaml`（joystick / network / web 段）
与 MeArm-3D 的 `config/robots.yaml` → `robot-package/mearm-v1/model/robot.yaml`
（关节 role 与限位、actuators 的接线顺序），把关节角按 `homePose` 换算成舵机角再比较
—— 改了模型或配置，期望值跟着变，不会出现"测试绿但机器不对"。

| 组 | 断言内容 |
| --- | --- |
| 协议 | `caps` 为 network、`servo_ids` 与配置一致、`link_status` 已连上、首帧 `state` 可取且 `device == sim` |
| 摇杆 | 复位 HOME；左摇杆 X 正偏 → 底座角**增大**（与串口同向）；未被推动的轴不动；回中即停 |
| XYZ | x/y/z 各正负一次，`step: 5` → `state.tcp[]` 对应分量变化方向与量值正确 |
| 爪 | `open` / `close` 各落在 robot.yaml 派生的两端点，且两端不同 |
| 舵机 | `servo 1..4 = 91°` 逐一回读一致（91 ≠ HOME，确认真的听话） |
| 容错 | 179.5° 越界被服务端拒绝 + 网页收到 `err` + 本地目标已回滚（之后摇杆**仍能**驱动该轴） |
| 并发 | 一次性灌 120 帧摇杆后仍收到合法 `state`，`jsonErrors == 0`，链路仍为已连接 |

> 「回滚是否生效」只有这一个可观测判据：若目标没回滚，它会停在 179.5°，之后每一帧都被
> 服务端拒绝，表现为"推杆完全不动且没有任何报错"—— 脚本第 10 组专门盯这个。

> 若本机 COM 为虚拟/无对端端口，`WriteFile` 会阻塞到 `writeTotalTimeoutConstant`
> （默认 500ms）后失败并触发重连——这是 Windows 虚拟串口的无对端特性，真实
> USB 串口（FTDI/CH340/CP2102）写操作会立即返回，不影响实际使用。

## 串口指令协议（与 arm-device 一致）

| 类别 | 示例 |
|------|------|
| 单控 | `SET 9 120` / `S7=90` |
| 组合 | `SET 9 120 8 90 7 100`（≤3 舵机，左右舵可同条） |
| 自变化/冻结 | `AUTO 9` / `STOP 9` |
| 摇杆 | `JOY 900 200 512 800`（4 路 raw 0..1023）/ `JOY 8 50`（单轴） |
| 红外 | `IR 0xF708FF00` |
| 动作序列 | `SEQ 1|3|7|9` / `SEQ STOP` / `SEQ ?` |
| 硬件开关 | `JOYHW ON|OFF` / `IRHW ON|OFF` |
| 查询/复位 | `STATUS` / `?` / `RESET` / `ADC` / `HELP` |

Web 双摇杆拖拽时，服务器按 `internal/protocol` 把左右摇杆坐标合并为
`JOY <r9> <r8> <r6> <r7>`（底座9/左舵8/夹取6/右舵7；死区内轴保持 512），
与硬件摇杆阈值/逻辑完全一致。

## 设计要点 / 预留接口

- **远程控制预留**：TCP 转发目前是纯文本透传（与 Web/固件一致）。代码在
  `internal/tcp/tcp.go` 的 `handle` 中保留了扩展点——若收到以 `@` 开头的行，可解析为
  结构化 JSON 指令（含鉴权/会话），便于后续接入公网远程控制而不破坏现有文本协议。
- **建模模式预留**：`index.html` 已有「建模模式（预留）」标签与占位面板，待摇杆模式
  验收完成后在其上叠加轨迹规划 / 可视化建模。
- **协议集中**：所有对串口下发的文本都经 `internal/protocol.Validate` 校验，非法指令
  被拒绝，避免把垃圾烧入固件。

## 目录结构

```
arm-web/
├── start.bat                            # 一键启动：默认 network，--real 走串口（纯 ASCII，每条路径都 pause）
├── config.yaml / config.yaml.example    # YAML 配置（mode / serial / web / tcp / network / joystick）
├── main.go                              # 装配：按 mode 二选一 —— (serial+hub+tcp) 或 (netlink+link)
├── main_test.go                          # 接线表 + 方向换算不变量（满偏增量符号）
├── internal/
│   ├── config/    # YAML 加载 + 默认值 + 校验（Mode / NetworkConfig / ServoIndex）
│   ├── protocol/  # arm-device 指令校验 + 双摇杆映射 + 死区门控 + 跨模式方向换算（含单测）
│   ├── serial/    # 串口（windows_serial.go / unix_serial.go + ACK 门控 + 自动重连）
│   ├── hub/       # 广播总线
│   ├── tcp/       # 局域网 TCP 透传（**仅 serial 模式**）
│   ├── netlink/   # 网络链路：TCP Client + 单写泵 + 每轴 latest-wins + 时间驱动积分 + 退避重连
│   ├── link/      # Link 抽象（serialLink / netLink 两实现 + Caps/Status）
│   └── web/       # HTTP 静态服务 + 标准库 WebSocket(ws.go) + 角度解析/合并
├── tools/
│   ├── e2e-sim.js            # 串口模式端到端仿真
│   ├── tcp-test.js           # 串口模式 9002 透传压测
│   └── sim2sim-check.mjs     # 网络模式 Sim2Sim 端到端断言（30 项，零依赖）
└── web/static/    # 前端：index.html + css + js(joystick3d / arm3d / wsclient / netpanel / main)
```

## 测试（本地自检）

```bat
go vet ./...                   # 干净（0 warning）
go test ./...                  # 39 用例 PASS（config/protocol/serial/netlink + main 接线表）
go test -race ./...            # 同上，竞态检测干净
node tools/e2e-sim.js          # 串口模式端到端仿真（需 arm-web 运行中 + 固件在 COM16）
node tools/sim2sim-check.mjs   # 网络模式 Sim2Sim 端到端（需 MeArm-3D 在 9100 + arm-web network）
```

覆盖范围：指令校验、双摇杆映射/死区门控、ACK 门控与超时、摇杆方向积分、
**跨模式方向换算不变量**（`main_test.go`）、netlink 的写泵串行化 / latest-wins /
退避重连 / 越界回滚 / 关闭后不写（`internal/netlink/client_test.go`，内置最小 MeArm-3D 替身）。

WebSocket 握手、ping→pong、指令校验回显均已通过本地自检。
