# MeArmPilot Go 后端增加 TCP JSON 控制接口

## 1. 任务目标

在当前 MeArmPilot 项目的 **Go 后端**中增加一个独立的 TCP Server。

目标是在**完全保持现有功能、现有 HTTP/WebSocket/Serial/Sim/Real 控制链路行为不变**的前提下，增加一个新的 TCP 控制入口：

```text
TCP Client
    │
    │ JSON
    ▼
Go TCP Server
    │
    │ 解析/校验/转换
    ▼
现有 MeArmPilot 控制接口
    │
    ├── Sim
    │
    └── Real
```

TCP Server 本身只负责：

1. 接收 TCP 连接
2. 接收 JSON 指令
3. 解析 JSON
4. 校验参数
5. 将 JSON 指令转换为项目当前已经存在的机械臂控制接口
6. 返回 JSON 结果

**不得重新实现 IK/FK、舵机控制、Sim 控制或 Real 控制逻辑。**

---

# 2. 最高优先级约束

这是一个**增量扩展任务，不是重构任务**。

必须严格遵守：

### 2.1 不修改现有业务行为

现有：

* HTTP API
* WebSocket
* Web 前端
* joystick
* Sim
* Real
* Serial
* MeArmPilot 当前机械臂控制流程
* 当前 IK/FK
* 当前舵机角度限制
* 当前设备状态回传
* 当前启动方式
* 当前配置方式

均必须继续正常工作。

除非为了接入 TCP Server 不可避免，否则不要修改现有代码。

---

### 2.2 TCP Server 是新增入口

不要把 TCP Server 改造成新的核心控制架构。

正确设计：

```text
                ┌── HTTP ────────┐
                │                │
                ├── WebSocket ───┤
                │                │
                ├── Serial ──────┤
                ▼                ▼
          现有控制接口 / Device 层
                    │
              ┌─────┴─────┐
              ▼           ▼
             Sim         Real

新增：

TCP Client
    │
    ▼
TCP JSON Server
    │
    ▼
现有控制接口 / Device 层
```

TCP Server 是一个新的 Adapter / Transport Layer。

---

### 2.3 禁止复制控制逻辑

TCP Server 中禁止出现：

* IK 算法
* FK 算法
* 舵机运动学计算
* 舵机软限位逻辑
* Sim 运动控制逻辑
* Real 串口控制逻辑
* Servo PWM 控制逻辑

如果当前项目已有对应函数，应直接调用。

例如：

```go
// 错误
func tcpMoveXYZ(x, y, z float64) {
    // 在这里重新实现 IK
}

// 正确
func tcpMoveXYZ(x, y, z float64) {
    // 转换成现有项目已经支持的控制调用
    device.MoveXYZ(x, y, z)
}
```

如果现有接口名称不同，以项目实际代码为准。

---

# 3. 开始实施前必须先做代码审计

不要直接修改代码。

第一阶段只进行代码分析。

需要检查：

```text
backend/
real-backend/
simulation/
serial/
device/
protocol/
ik/
fk/
websocket/
http/
```

以及项目实际存在的对应目录。

重点回答：

### A. 当前 Go 后端入口在哪里？

找到：

* main
* server 启动位置
* HTTP Server
* WebSocket Server
* Serial 控制入口

---

### B. 当前机械臂控制链路是什么？

明确：

```text
Web / WebSocket
      ↓
Go Controller
      ↓
Device
      ↓
Sim / Real
```

实际项目如果不同，以代码为准。

---

### C. Sim 和 Real 当前如何区分？

明确：

```text
Device
 ├── Sim Device
 └── Real Device
```

或者实际项目中的等价结构。

---

### D. 当前已经存在什么控制接口？

重点寻找：

```text
Move
MoveXYZ
SetServo
SetJoint
OpenGripper
CloseGripper
Reset
Status
State
```

或者实际项目对应接口。

---

### E. 当前状态如何返回？

确认是否已有：

```text
servo1
servo2
servo3
servo4

x
y
z

gripper
```

以及当前状态结构。

---

# 4. 审计完成后先输出设计报告

修改代码之前必须输出：

```text
1. 当前 Go 后端架构
2. 当前 Sim/Real 控制入口
3. 当前可复用的 Device/API
4. 当前 HTTP/WebSocket/Serial 入口
5. TCP Server 最小接入点
6. 哪些文件需要新增
7. 哪些文件需要修改
8. 哪些文件明确禁止修改
9. TCP 指令如何映射到现有 API
10. 如何保证旧功能零影响
```

如果发现现有架构已经有统一 Device 接口，则 TCP Server 必须优先接入该接口。

---

# 5. TCP Server 设计

新增一个 TCP Server。

例如：

```text
TCP Server
    listen: configurable host/port
```

默认端口可以选择一个**不会与现有端口冲突**的端口。

不要硬编码无法修改的端口。

建议：

```yaml
tcp:
  enabled: true
  host: 0.0.0.0
  port: <new-port>
```

如果当前项目配置不是 YAML，则遵循项目现有配置方式。

---

# 6. TCP 协议

采用：

```text
TCP Stream
+
JSON Lines
```

即：

```text
一条 JSON + \n
```

例如：

```json
{"cmd":"move","x":1,"y":0,"z":0}
```

每条命令以：

```text
\n
```

结束。

不要设计复杂的二进制协议。

---

# 7. JSON 基础结构

统一使用：

```json
{
  "cmd": "xxx"
}
```

必要参数放在同一级。

---

# 8. XYZ 方向偏移

增加 XYZ 相对方向控制。

支持：

```text
x+
x-
y+
y-
z+
z-
```

推荐协议：

```json
{"cmd":"move","axis":"x","direction":"+","step":10}
```

例如：

```json
{"cmd":"move","axis":"x","direction":"+","step":10}
```

表示：

```text
X += 10
```

---

```json
{"cmd":"move","axis":"x","direction":"-","step":10}
```

表示：

```text
X -= 10
```

同理：

```json
{"cmd":"move","axis":"y","direction":"+","step":10}
```

```json
{"cmd":"move","axis":"y","direction":"-","step":10}
```

```json
{"cmd":"move","axis":"z","direction":"+","step":10}
```

```json
{"cmd":"move","axis":"z","direction":"-","step":10}
```

---

# 9. XYZ 控制的重要约束

这里的：

```text
x/y/z
```

必须表示**当前项目已经定义的机械臂工作空间坐标系**。

不要重新定义坐标系。

必须先确认：

```text
X 正方向
X 负方向

Y 正方向
Y 负方向

Z 正方向
Z 负方向
```

然后直接复用当前 MeArmPilot 的 XYZ / IK 定义。

TCP Server 不能自行改变坐标系。

---

# 10. Step 参数

`step` 表示一次相对位移。

例如：

```json
{
  "cmd": "move",
  "axis": "x",
  "direction": "+",
  "step": 5
}
```

逻辑：

```text
target_x = current_x + 5
```

然后：

```text
target_x
target_y
target_z
```

交给现有控制链路处理。

---

# 11. 不允许 TCP 层绕过当前运动学限制

例如当前项目已经存在：

```text
workspace limit
joint limit
servo limit
IK validity
```

必须继续使用。

TCP Server 不能：

```text
直接计算舵机角度
绕过 IK
绕过软限位
绕过错误检查
```

---

# 12. 爪控制

支持：

```text
open
close
```

协议建议：

```json
{"cmd":"gripper","action":"open"}
```

以及：

```json
{"cmd":"gripper","action":"close"}
```

TCP Server 应映射到当前已有的：

```text
gripper open
gripper close
```

接口。

如果当前项目使用的是 servo4 表示夹爪，则：

```text
TCP JSON
    ↓
现有 Gripper API
    ↓
Servo4
```

而不是：

```text
TCP JSON
    ↓
TCP Server 自己设置 Servo4
```

---

# 13. 四个舵机直接控制

增加直接控制 4 个舵机的能力。

协议：

```json
{
  "cmd":"servo",
  "servo":1,
  "angle":90
}
```

支持：

```text
servo = 1
servo = 2
servo = 3
servo = 4
```

例如：

```json
{"cmd":"servo","servo":1,"angle":90}
```

```json
{"cmd":"servo","servo":2,"angle":45}
```

```json
{"cmd":"servo","servo":3,"angle":120}
```

```json
{"cmd":"servo","servo":4,"angle":80}
```

---

# 14. Servo Direct Control 的安全约束

这是一个非常重要的区别：

```text
XYZ 控制
```

走：

```text
XYZ
 ↓
IK
 ↓
Joint
 ↓
Servo
```

而：

```text
Servo Direct
```

允许：

```text
Servo
 ↓
现有 Servo 控制接口
```

但是仍然必须使用项目已有的：

```text
servo limit
angle validation
device validation
```

禁止 TCP Server 自己定义第二套舵机限制。

---

# 15. 推荐统一命令集合

TCP v1 只实现：

```text
move
gripper
servo
```

### Move

```json
{
  "cmd":"move",
  "axis":"x",
  "direction":"+",
  "step":10
}
```

### Gripper

```json
{
  "cmd":"gripper",
  "action":"open"
}
```

### Servo

```json
{
  "cmd":"servo",
  "servo":1,
  "angle":90
}
```

暂时不要增加：

```text
reset
speed
trajectory
IK
FK
macro
AI
camera
gesture
```

除非当前项目已有明确接口且属于兼容性需要。

第一版保持协议最小化。

---

# 16. 返回协议

每条 JSON 指令必须返回 JSON。

成功：

```json
{
  "ok":true
}
```

或者：

```json
{
  "ok":true,
  "cmd":"move"
}
```

错误：

```json
{
  "ok":false,
  "error":"invalid axis"
}
```

参数错误：

```json
{
  "ok":false,
  "error":"invalid servo"
}
```

角度非法：

```json
{
  "ok":false,
  "error":"angle out of range"
}
```

---

# 17. 不要让 TCP Server 崩溃

必须保证：

```text
非法 JSON
错误参数
未知 cmd
断开连接
客户端异常退出
空数据
超长数据
```

都不会导致整个 Go 后端退出。

错误只影响当前 TCP connection/request。

---

# 18. 多客户端

第一版允许多个 TCP Client 连接。

但是必须考虑机械臂控制冲突。

至少保证：

```text
TCP Client A
TCP Client B
```

同时发送命令时：

```text
不会产生 data race
不会导致 Go panic
不会破坏 Device 状态
```

如果当前 Device 本身不支持并发控制，则在 TCP Adapter 层使用现有项目允许的串行化机制。

**不要重新设计整个控制系统。**

---

# 19. TCP Server 与现有服务器并行运行

启动后应该类似：

```text
HTTP Server       : xxxx
WebSocket Server  : xxxx
TCP Server        : new-port
```

TCP Server 是新增服务。

不能：

```text
TCP Server 替换 HTTP
TCP Server 替换 WebSocket
TCP Server 替换 Serial
```

---

# 20. Sim / Real 必须保持统一

TCP 指令不应该知道底层是：

```text
Sim
```

还是：

```text
Real
```

正确：

```text
TCP
 ↓
Controller / Device
 ↓
当前 active device
 ↓
Sim / Real
```

例如：

```text
TCP:
{"cmd":"servo","servo":1,"angle":90}
```

当前运行：

```text
Sim
```

则控制 Sim。

当前运行：

```text
Real
```

则控制 Real。

TCP 层不复制：

```text
SimServo()
RealServo()
```

这样的分支逻辑。

---

# 21. 状态同步

如果当前项目已有 Device State / Robot State，可以在成功执行后返回必要状态。

例如：

```json
{
  "ok":true,
  "state":{
    "servo":[90,80,70,60],
    "gripper":"open"
  }
}
```

但是：

**不要为了 TCP 接口重新创建一套状态系统。**

优先读取现有状态。

如果当前项目已经具备状态回传能力，复用现有状态。

如果没有，则第一版可以只返回：

```json
{"ok":true}
```

不要因为 TCP 接口强行修改整个状态架构。

---

# 22. TCP Server 文件组织

优先采用独立文件/模块。

例如：

```text
backend/
    ...
    tcp_server.go
    tcp_protocol.go
```

或者按照当前项目实际目录：

```text
server/
    tcp.go
    tcp_protocol.go
```

最终目录结构必须服从当前项目，而不是为了这个功能大规模重构。

建议职责：

### tcp_server.go

负责：

```text
listen
accept
connection lifecycle
read lines
write response
```

### tcp_protocol.go

负责：

```text
JSON struct
parse
validate
command dispatch
```

### 现有 Device / Controller

负责：

```text
实际机械臂控制
```

---

# 23. 推荐的数据结构

可以采用类似：

```go
type TCPCommand struct {
    Cmd       string  `json:"cmd"`
    Axis      string  `json:"axis,omitempty"`
    Direction string  `json:"direction,omitempty"`
    Step      float64 `json:"step,omitempty"`
    Action    string  `json:"action,omitempty"`
    Servo     int     `json:"servo,omitempty"`
    Angle     float64 `json:"angle,omitempty"`
}
```

但必须根据项目现有代码调整。

不要为了这个协议修改现有业务数据结构。

---

# 24. Command Dispatcher

TCP 层可以有：

```go
switch cmd.Cmd {
case "move":
    ...
case "gripper":
    ...
case "servo":
    ...
default:
    ...
}
```

但 Dispatcher 只负责：

```text
协议 → 现有 API
```

例如：

```text
move
 ↓
current position
 ↓
target XYZ
 ↓
existing MoveXYZ()
```

---

# 25. XYZ 相对移动的关键处理

由于命令是：

```text
+X
-X
+Y
-Y
+Z
-Z
```

因此必须首先获取当前 XYZ。

例如：

```text
current = (x, y, z)

x+ step=10

target = (x+10, y, z)
```

然后：

```text
existing MoveXYZ(target)
```

而不是：

```text
TCP
 ↓
自己做 IK
 ↓
自己生成 servo angle
```

如果当前项目不存在“读取当前 XYZ”的接口：

### 不允许立即重构整个系统。

先分析当前状态来源。

优先使用：

1. Device 当前状态
2. Controller 当前状态
3. 当前已有 FK/状态转换
4. 项目已有缓存

只有在完全不存在时，才设计最小的 TCP 层状态适配，并明确记录这个限制。

---

# 26. 方向定义必须验证

实施前必须通过当前代码确认：

```text
+X
-X
+Y
-Y
+Z
-Z
```

具体含义。

禁止凭经验假设：

```text
X 一定是左右
Y 一定是上下
Z 一定是前后
```

必须以当前 MeArmPilot 项目坐标系为准。

---

# 27. 不修改前端

本任务原则上：

```text
Frontend = 不修改
```

TCP 是新的外部控制接口。

不要为了测试 TCP 去修改 Web UI。

测试应使用：

```text
netcat
Python TCP client
Go TCP client
```

等独立工具。

---

# 28. 不修改现有 HTTP API

现有：

```text
POST /...
GET /...
```

全部保持原样。

禁止：

```text
修改 endpoint
修改 request
修改 response
修改现有 JSON
```

除非代码审计证明 TCP 接入必须进行极小兼容性修改。

如果确实需要修改，必须先说明原因。

---

# 29. 不修改 WebSocket 协议

现有 WebSocket：

```text
message
command
state
```

全部保持兼容。

TCP Server 不得复用到会改变 WebSocket 行为的方式。

---

# 30. 不修改 Real Serial Protocol

Real 机械臂当前串口协议必须保持不变。

TCP Server：

```text
TCP JSON
 ↓
现有 Device
 ↓
现有 Serial Protocol
```

不要：

```text
TCP JSON
 ↓
重新生成 Serial Command
```

---

# 31. 不修改 Sim Protocol

如果 Sim 当前已经通过：

```text
Device
 ↓
simulation backend
```

控制，则 TCP 必须继续走该路径。

不要为 TCP 单独创建：

```text
TCP → MuJoCo
```

这样的第二条控制链。

---

# 32. 测试要求

必须增加 TCP Server 的测试。

至少覆盖：

### JSON 解析

```text
valid JSON
invalid JSON
empty JSON
unknown command
```

### Move

```text
x+
x-
y+
y-
z+
z-
```

### Gripper

```text
open
close
```

### Servo

```text
servo1
servo2
servo3
servo4
```

### 错误

```text
invalid axis
invalid direction
invalid servo
invalid angle
missing step
missing action
```

---

# 33. 集成测试

至少进行：

```text
启动 Go backend
        ↓
确认现有 HTTP 正常
        ↓
确认 WebSocket 正常
        ↓
确认现有 Web UI 正常
        ↓
确认 Real/Sim 原有功能正常
        ↓
启动 TCP Client
        ↓
发送 JSON
        ↓
观察机械臂
```

---

# 34. TCP 测试示例

提供一个最小 Python Client：

```python
import socket
import json

HOST = "127.0.0.1"
PORT = <tcp-port>

commands = [
    {
        "cmd": "move",
        "axis": "x",
        "direction": "+",
        "step": 10
    },
    {
        "cmd": "gripper",
        "action": "open"
    },
    {
        "cmd": "servo",
        "servo": 1,
        "angle": 90
    }
]

with socket.create_connection((HOST, PORT)) as sock:
    for command in commands:
        data = json.dumps(command) + "\n"
        sock.sendall(data.encode())

        response = sock.recv(4096)
        print(response.decode())
```

实际端口根据项目配置生成。

---

# 35. 回归测试

这是本任务最重要的验收内容。

修改之后必须确认：

### Web

```text
正常
```

### HTTP

```text
正常
```

### WebSocket

```text
正常
```

### Serial

```text
正常
```

### Sim

```text
正常
```

### Real

```text
正常
```

### TCP

```text
正常
```

要求：

```text
旧功能行为 = 修改前
新增功能 = TCP JSON
```

---

# 36. Git Diff 控制

实施完成后检查：

```bash
git diff
git status
```

重点确认：

```text
没有无关文件修改
没有自动格式化整个项目
没有大规模重构
没有修改前端
没有修改协议
没有删除旧代码
```

如果发现大范围 diff：

**停止并重新审查。**

---

# 37. 编译与启动

使用项目当前已有的：

```text
go build
go test
go run
```

或者项目已有 Makefile / CMake / Task / Script。

不要引入新的构建系统。

---

# 38. 最终输出要求

完成后输出：

## 1. 修改文件

```text
新增：
xxx

修改：
xxx
```

## 2. 架构变化

用图说明：

```text
TCP JSON
   ↓
TCP Adapter
   ↓
Existing Controller / Device
   ↓
Sim / Real
```

## 3. 新增协议

完整列出：

```text
move
gripper
servo
```

以及 JSON 示例。

## 4. 现有功能回归结果

列出：

```text
HTTP        PASS/FAIL
WebSocket   PASS/FAIL
Serial      PASS/FAIL
Sim         PASS/FAIL
Real        PASS/FAIL
TCP         PASS/FAIL
```

## 5. 风险

明确说明：

```text
是否修改原有控制链路
是否新增状态
是否修改 Device
是否修改 Serial
是否修改前端
是否存在并发问题
```

## 6. Git Diff 摘要

说明本次修改规模。

---

# 39. 最终验收标准

只有同时满足以下条件才算完成：

### A

TCP Server 可以启动。

### B

可以建立 TCP 连接。

### C

可以接收 JSON Lines。

### D

支持：

```text
X+
X-
Y+
Y-
Z+
Z-
```

### E

支持：

```text
Gripper Open
Gripper Close
```

### F

支持：

```text
Servo1
Servo2
Servo3
Servo4
```

### G

Sim 模式下 TCP 可以控制机械臂。

### H

Real 模式下 TCP 可以控制机械臂。

### I

TCP 不绕过现有：

```text
IK
FK
Limit
Device
Serial
Sim
```

等控制机制。

### J

现有 HTTP/WebSocket/Web UI/Serial 功能不受影响。

### K

非法 JSON / 非法参数不会导致 Go 后端崩溃。

### L

所有新增代码具有基本错误处理。

### M

代码修改范围最小。

---

# 40. 最重要的一条原则

本任务不是：

> “重新设计 MeArmPilot 的控制系统”。

而是：

> **给现有 MeArmPilot 增加一个 TCP 控制端口，把 TCP JSON 转换成现有机械臂控制接口。**

因此最终应该形成：

```text
                ┌────────────── Web UI
                │
                ├────────────── HTTP
                │
                ├────────────── WebSocket
                │
                ├────────────── Serial
                │
TCP JSON ───────┤
                ▼
        Existing Controller
                │
          Existing Device
                │
          ┌─────┴─────┐
          ▼           ▼
         Sim         Real
```

**TCP Server 是“端口扩展 + 协议转换”，不是新的机器人控制核心。**

如果发现实现过程中需要大规模修改现有 Controller、Device、IK、FK、Sim、Real 或 Frontend，应立即停止，并先重新分析接入点，而不是继续扩大修改范围。
