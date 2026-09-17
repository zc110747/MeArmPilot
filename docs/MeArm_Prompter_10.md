# MeArmPilot：MeArm-RemoteControl 通过 TCP 接入 MeArm-3D

> 用途：将本文档直接提供给 OpenCode / WorkBuddy 等 AI 编程工具执行。  
> 目标：在不破坏现有功能的前提下，让 `MeArm-RemoteControl` 默认通过 TCP Client 控制 `MeArm-3D` 的 Sim，并保留 `--real` 串口真机模式。

---

## 1. 任务目标

在现有仓库：

```text
https://github.com/zc110747/MeArmPilot
```

中，为以下两个模块建立兼容性连接：

```text
MeArm-RemoteControl
MeArm-3D
```

新增控制链路：

```text
网页摇杆
    ↓
MeArm-RemoteControl Web UI
    ↓
MeArm-RemoteControl Go Backend
    ↓
TCP Client
    ↓
MeArm-3D TCP Server
    ↓
MeArm-3D Controller / Device
    ↓
Sim / Real
```

最终要求：

- `MeArm-RemoteControl` 默认使用网络模式。
- 网络模式通过 TCP Client 连接 `MeArm-3D` 已有 TCP Server。
- 网页摇杆可以控制 `MeArm-3D` 仿真机械臂。
- 控制结果和可用状态尽可能反映到 RemoteControl 网页。
- `MeArm-RemoteControl` 原有串口真机控制功能必须保留。
- `start.bat` 默认使用网络模式。
- `start.bat --real` 使用原有串口模式。
- 当前没有真实硬件，先完成 Sim2Sim 验证。
- 不重构、不合并、不破坏 `MeArm-3D` 当前完整流程。

---

## 2. 绝对约束

### 2.1 不得破坏 MeArm-3D

原则上只修改：

```text
MeArm-RemoteControl
```

不得主动修改或重构 `MeArm-3D` 的：

- Sim 流程
- Real 流程
- MuJoCo 逻辑
- IK / FK
- 机械臂模型
- MJCF
- Controller
- Device
- HTTP 接口
- WebSocket 接口
- TCP 协议
- 启动流程
- 串口协议

只有在代码审计确认存在必要的兼容性问题时，才允许对 MeArm-3D 做最小修改。

如果必须修改 MeArm-3D，必须先说明：

1. 为什么必须修改。
2. 修改哪些文件。
3. 修改什么内容。
4. 是否影响原有流程。
5. 如何验证兼容性。

禁止借机进行大规模重构。

### 2.2 必须保留 RemoteControl 串口功能

不得删除、替换或破坏：

- 串口初始化
- 串口配置
- 串口协议
- 串口命令生成
- 串口命令发送
- 真机控制入口
- 原有网页控制逻辑
- 原有启动脚本行为

本任务是新增 TCP 传输方式，不是将串口控制改造成 TCP 控制。

### 2.3 不得凭空假设项目结构

必须先阅读实际代码：

- 不根据目录名称猜测函数。
- 不根据示例猜测 TCP 协议。
- 不假设已有接口一定存在。
- 不假设状态反馈一定存在。
- 不假设坐标轴方向。
- 不假设消息边界。
- 不假设 `--real` 当前没有其他语义。

所有实现必须以当前仓库代码为准。

---

## 3. 启动模式要求

### 3.1 默认网络模式

执行：

```bat
start.bat
```

行为：

```text
mode = network
```

要求：

- 不打开 RemoteControl 串口。
- 启动网页和 Go 后端。
- 启动 TCP Client。
- 尝试连接 MeArm-3D TCP Server。
- MeArm-3D 尚未启动时，RemoteControl 不应崩溃。
- TCP 连接失败时，网页和 Go 服务仍应正常运行。
- 连接状态必须通过日志或网页明确体现。
- 默认配置适合本机 Sim2Sim 验证，例如 `127.0.0.1`。

### 3.2 串口模式

执行：

```bat
start.bat --real
```

行为：

```text
mode = serial
```

控制链路：

```text
RemoteControl Web
    ↓
RemoteControl Go Backend
    ↓
原有 Serial Control
    ↓
AVR
    ↓
真实机械臂
```

要求：

- 使用原有串口配置。
- 使用原有串口初始化流程。
- 使用原有串口协议。
- 使用原有串口命令转换。
- 使用原有串口发送逻辑。
- 不强制连接 MeArm-3D。
- 不强制启动 TCP Client。
- 不删除原有真机控制代码。

这里的 `--real` 表示 RemoteControl 进入串口真机模式，不要擅自将其解释为 MeArm-3D 的 TCP Real 模式。

### 3.3 参数解析

至少支持：

```text
start.bat             → network
start.bat --real      → serial
未知参数              → 明确错误提示
```

如果当前脚本已有其他参数，必须保持兼容。

---

## 4. 必须遵循的工作流程

不要直接修改代码。必须按以下顺序执行：

1. 工作区检查。
2. MeArm-RemoteControl 代码审计。
3. MeArm-3D TCP 和控制链路审计。
4. 输出审计报告。
5. 确认真实 TCP 协议。
6. 设计最小接入点。
7. 实现 TCP Client。
8. 实现网络/串口模式切换。
9. 接入网页摇杆。
10. 接入爪和舵机控制。
11. 处理状态反馈。
12. 完成 Sim2Sim 验证。
13. 完成串口模式回归检查。
14. 检查 Git Diff。
15. 输出最终交付报告。

如果发现重大风险，先暂停并报告，不得自行扩大任务范围。

---

## 5. 阶段一：工作区检查

开始前执行：

```bash
git status
git branch --show-current
```

检查：

- 当前分支。
- 未提交修改。
- 用户本地改动。
- 两个项目的实际位置。
- Go 版本。
- 构建方式。
- 启动脚本。
- 现有测试方式。

不得覆盖用户已有的未提交修改。

---

## 6. 阶段二：审计 MeArm-RemoteControl

找到并记录：

- Go 程序入口。
- `main` 函数。
- HTTP Server。
- WebSocket Server（如果存在）。
- 网页摇杆输入入口。
- 前后端通信方式。
- 当前控制命令结构。
- 串口控制入口。
- 串口协议。
- 状态结构。
- 状态回传机制。
- 启动脚本。
- 配置方式。
- 错误处理方式。

必须回答：

1. 网页摇杆发送什么数据？
2. 摇杆数据如何进入 Go 后端？
3. Go 后端如何处理方向、步长和频率？
4. 是否已有 Command、Controller、Device 或 Transport 抽象？
5. 串口命令在哪里生成？
6. 串口命令在哪里发送？
7. 是否有状态缓存？
8. 网页如何更新状态？
9. `start.bat` 当前如何工作？
10. 哪些逻辑可以直接复用？

---

## 7. 阶段三：审计 MeArm-3D

找到并记录：

- TCP Server。
- TCP Server 启动位置。
- TCP 监听地址和端口。
- TCP 消息边界。
- JSON 指令格式。
- 成功响应。
- 错误响应。
- 状态查询。
- 主动状态推送。
- Sim / Real 切换方式。
- Controller。
- Device。
- XYZ 控制接口。
- Gripper 控制接口。
- Servo 控制接口。

必须回答：

1. TCP 使用 JSON Lines、换行分隔还是其他方式？
2. 每条消息是否必须以 `\n` 结束？
3. 服务端是否一次连接处理多个请求？
4. 是否支持请求-响应模式？
5. 是否存在状态反馈？
6. 当前控制入口是什么？
7. Sim / Real 如何选择？
8. RemoteControl 应调用哪些已有接口？
9. 是否需要修改 MeArm-3D？
10. 如果需要，最小修改是什么？

---

## 8. 阶段四：修改前审计报告

在修改任何代码之前，输出：

### 8.1 当前链路

```text
RemoteControl：

Web UI
    ↓
当前通信方式
    ↓
Go Backend
    ↓
当前控制器
    ↓
Serial
```

```text
MeArm-3D：

TCP Server
    ↓
Controller
    ↓
Device
    ↓
Sim / Real
```

### 8.2 最小接入点

明确：

- TCP Client 应放在哪个包或目录。
- 是否可以复用现有命令生成逻辑。
- 是否需要 Transport 抽象。
- 是否需要修改网页。
- 是否需要修改 MeArm-3D。
- 如何避免网络模式和串口模式互相影响。

### 8.3 修改范围

列出：

- 新增文件。
- 修改文件。
- 原则上不应修改的文件。
- 预计影响的功能。

### 8.4 风险

至少分析：

- TCP 协议兼容性。
- JSON 消息边界。
- 摇杆发送频率。
- TCP 连接生命周期。
- 并发写入。
- 状态反馈。
- 串口模式兼容性。
- MeArm-3D 修改风险。

审计报告完成后再实施。

---

## 9. TCP Client 设计要求

TCP Client 只负责网络传输，不负责机械臂运动学。

### 9.1 TCP Client 职责

必须支持：

1. 建立 TCP 连接。
2. 发送符合 MeArm-3D 实际协议的 JSON。
3. 读取响应。
4. 处理消息边界。
5. 处理连接失败。
6. 处理连接断开。
7. 处理服务端错误。
8. 处理超时。
9. 避免并发写入冲突。
10. 正确释放连接和 goroutine。
11. 必要时提供简单重连。
12. 向上层提供连接状态。

### 9.2 TCP Client 不负责

不得在 RemoteControl 中实现：

- IK。
- FK。
- 坐标变换。
- 舵机角度计算。
- 舵机软限位。
- Sim 控制。
- Real 控制。
- 机械臂运动学。
- MeArm-3D 内部设备逻辑。
- 新的串口协议。

如果已有抽象可复用，优先复用；如果没有，采用最小实现，不要大规模重构。

### 9.3 并发和连接安全

必须保证：

- 同一 TCP 连接不会被多个 goroutine 无保护地写入。
- JSON 消息不会交叉。
- 连接关闭后不会继续写入。
- 不会出现 goroutine 泄漏。
- 不会无限快速重连。
- 连接错误不会导致 Go 后端崩溃。
- 网络错误不会破坏串口模式。

---

## 10. TCP 协议适配

必须使用 MeArm-3D 当前代码中已经实现的协议。

以下仅为格式示例，不得直接假设为最终协议。

### 10.1 XYZ 相对移动示例

```json
{
  "cmd": "move",
  "axis": "x",
  "direction": "+",
  "step": 10
}
```

需要确认：

- 命令字段名称。
- 轴字段名称。
- 方向字段名称。
- 步长字段名称。
- 步长单位。
- 坐标系。
- 是否为相对移动。
- 是否需要目标位置。
- 错误响应格式。

### 10.2 爪控制示例

```json
{
  "cmd": "gripper",
  "action": "open"
}
```

需要确认：

- Open / Close 的真实字段。
- 是否使用角度或增量。
- 是否需要响应。
- 是否有软限位。

### 10.3 舵机控制示例

```json
{
  "cmd": "servo",
  "servo": 1,
  "angle": 90
}
```

需要确认：

- 舵机编号范围。
- 角度单位。
- 角度是否允许小数。
- 限位由哪一侧负责。
- 服务端错误处理。
- 四个舵机的映射关系。

---

## 11. 网页摇杆接入

### 11.1 保留现有网页输入逻辑

先确定网页摇杆真实输出格式，例如：

```text
up
down
left
right
forward
backward
```

或者：

```text
x+
x-
y+
y-
z+
z-
```

不得凭空假设。

尽量保持：

```text
网页摇杆
    ↓
现有命令处理逻辑
    ↓
Transport
    ├── Serial
    └── TCP
```

如果当前没有 Transport 抽象，只增加最小适配层。

### 11.2 XYZ 控制

必须验证：

- 网页方向与 XYZ 方向的映射。
- 是否需要轴反转。
- 步长单位。
- 摇杆灵敏度。
- 是否为连续控制。
- 是否存在重复指令。
- 是否需要节流。

不得假设 X、Y、Z 的实际空间方向，必须使用 MeArm-3D 当前坐标系定义。

### 11.3 爪控制

复用当前网页爪控制方式，转换为 MeArm-3D 实际支持的 TCP 指令。

不要在 RemoteControl 中直接操作 MeArm-3D 内部舵机。

### 11.4 四个舵机控制

如果网页当前已有直接控制四个舵机的功能：

- 保留原有功能。
- 网络模式转换为实际 Servo TCP 指令。
- 不复制 MeArm-3D 的软限位逻辑。
- 不修改原有串口命令格式。

如果当前网页没有此功能，不要为了本次任务大规模重新设计网页；先报告当前能力和最小扩展方案。

---

## 12. 摇杆发送频率和队列

必须审计当前摇杆发送方式：

- 鼠标移动时是否立即发送。
- 是否使用定时器。
- 是否持续发送。
- 是否有节流。
- 是否有重复命令。
- 是否有命令队列。

防止：

```text
摇杆持续输入
    ↓
大量 TCP 指令
    ↓
服务端处理不及时
    ↓
延迟、堆积或旧指令持续执行
```

要求：

1. 优先复用现有发送机制。
2. 不随意改变原有串口模式行为。
3. 网络模式不得无限堆积命令。
4. 如需节流，采用最小改动。
5. 不引入运动规划器。
6. 不修改 MeArm-3D 核心控制逻辑。

---

## 13. 状态反馈和网页同步

### 13.1 状态来源

按以下优先级处理：

1. MeArm-3D 已有状态接口。
2. MeArm-3D TCP 状态响应。
3. MeArm-3D 状态查询机制。
4. 已有 WebSocket 状态机制。
5. 必要时进行最小兼容扩展。

不要在 RemoteControl 中把本地发送缓存当成真实设备状态。

必须区分：

- 命令发送成功。
- 服务端接收成功。
- 命令执行成功。
- 当前设备状态。
- 连接状态。
- 错误状态。

### 13.2 状态不可用时

如果当前协议无法提供真实状态：

- 明确报告限制。
- 使用现有可用状态。
- 不虚构机械臂位置。
- 不声称命令已实际执行。
- 可以显示 `unknown`、`unavailable` 或项目已有状态。

### 13.3 网页同步

如果 RemoteControl 当前已经显示：

- 舵机角度。
- XYZ 位置。
- 爪状态。
- 连接状态。
- 控制状态。

尽量复用现有网页显示逻辑。

状态必须来自实际可获得的数据。

---

## 14. Sim2Sim 验证

当前没有真实硬件，第一阶段只验证：

```text
RemoteControl → TCP → MeArm-3D Sim
```

### 14.1 启动顺序

按照项目现有方式启动 MeArm-3D：

- 确认 TCP Server 已监听。
- 确认 Sim 已启动。
- 确认原有 3D 页面正常。
- 确认 MeArm-3D 原有控制流程正常。

然后执行：

```bat
start.bat
```

确认：

- RemoteControl 进入网络模式。
- 不打开串口。
- TCP Client 尝试连接。
- 网页正常启动。
- 连接状态清晰。

### 14.2 XYZ 测试

逐项测试：

```text
X+
X-
Y+
Y-
Z+
Z-
```

每项检查：

1. 网页是否产生正确输入。
2. Go 后端是否正确识别。
3. TCP Client 是否发送正确 JSON。
4. MeArm-3D 是否接收。
5. Sim 是否执行。
6. 3D 模型是否正确运动。
7. 状态是否反馈。
8. 网页是否更新。

### 14.3 爪测试

测试：

```text
Open
Close
```

检查：

```text
网页
    ↓
RemoteControl
    ↓
TCP
    ↓
MeArm-3D
    ↓
Sim Gripper
```

### 14.4 舵机测试

如果当前网页支持直接舵机控制，测试：

```text
Servo1
Servo2
Servo3
Servo4
```

检查：

- 舵机编号。
- 角度传递。
- 服务端错误处理。
- Sim 模型变化。

---

## 15. 串口回归验证

执行：

```bat
start.bat --real
```

在没有真实硬件的情况下，至少检查：

- 参数解析正确。
- 进入串口模式。
- 原有串口初始化代码仍存在。
- 原有串口配置仍有效。
- 原有串口命令转换仍存在。
- 原有串口发送逻辑未删除。
- 不强制连接 MeArm-3D。
- 网络模式不会强制打开串口。

测试结果必须区分：

```text
代码检查：PASS
真实硬件测试：NOT TESTED
```

不得声称真实机械臂已经验证。

---

## 16. MeArm-3D 回归检查

检查并尽量验证：

- 原有网页控制。
- 原有 HTTP。
- 原有 WebSocket。
- 原有 TCP Server。
- 原有 Sim。
- 原有 Real。
- 原有串口流程。
- 原有 MuJoCo。
- 原有启动方式。

如果没有修改 MeArm-3D，说明未修改并给出相应验证结果。

如果修改了 MeArm-3D，必须提供详细影响分析。

---

## 17. 错误处理要求

### 17.1 TCP 错误

必须处理：

- 连接失败。
- 连接断开。
- 发送失败。
- 读取失败。
- 超时。
- 服务端错误。
- 服务端重启。
- 客户端退出。

### 17.2 JSON 错误

必须处理：

- 非法 JSON。
- 未知命令。
- 缺少字段。
- 字段类型错误。
- 非法轴。
- 非法方向。
- 非法步长。
- 非法舵机编号。
- 非法角度。

### 17.3 模式错误

必须处理：

- 未知启动参数。
- 串口初始化失败。
- 网络连接失败。
- 配置错误。

要求：

- 不导致 Go 后端崩溃。
- 错误日志清晰。
- 不吞掉关键错误。
- 不影响另一种模式。
- 正确释放资源。

---

## 18. 测试要求

### 18.1 TCP Client 单元测试

至少覆盖：

- JSON 编码。
- JSON 解码。
- 连接成功。
- 连接失败。
- 发送成功。
- 发送失败。
- 响应读取。
- 服务端错误。
- 连接关闭。
- 并发写入保护。

### 18.2 模式测试

```text
start.bat
    → network

start.bat --real
    → serial
```

### 18.3 控制映射测试

至少覆盖：

```text
X+
X-
Y+
Y-
Z+
Z-
Gripper Open
Gripper Close
Servo1
Servo2
Servo3
Servo4
```

不存在的网页功能必须标记：

```text
NOT IMPLEMENTED
```

不得虚构测试结果。

### 18.4 测试结果标记

统一使用：

```text
PASS
FAIL
NOT TESTED
NOT IMPLEMENTED
```

---

## 19. Git Diff 审计

完成后执行：

```bash
git status
git diff
```

检查：

- 新增文件是否必要。
- 修改文件是否必要。
- 是否意外修改 MeArm-3D。
- 是否删除串口代码。
- 是否修改无关前端。
- 是否修改无关配置。
- 是否产生大规模重构。
- 是否修改已有协议。
- 是否产生大量无关格式化。
- 是否存在调试代码。
- 是否存在硬编码地址或端口。
- 是否存在未处理错误。

清理无关修改。

---

## 20. 最终交付报告

完成后必须输出：

### 20.1 实现摘要

说明本次完成的功能和未完成的部分。

### 20.2 最终架构

```text
RemoteControl Web
    ↓
RemoteControl Go Backend
    ↓
TCP Client
    ↓
MeArm-3D TCP Server
    ↓
MeArm-3D Controller / Device
    ↓
Sim / Real
```

### 20.3 文件修改清单

分别列出：

- 新增文件。
- 修改文件。
- 未修改的关键文件。
- 每个文件的修改原因。

### 20.4 启动方式

```text
start.bat
    → Network Mode

start.bat --real
    → Serial Mode
```

### 20.5 协议映射

列出实际使用的：

- XYZ 指令。
- 爪控制指令。
- 四个舵机指令。
- 消息边界。
- 成功响应。
- 错误响应。
- 状态响应。

必须以实际代码为准。

### 20.6 状态反馈

说明：

- 状态从哪里获取。
- 是否复用 TCP 状态。
- 是否复用 WebSocket。
- 是否增加适配。
- 网页如何更新。
- 当前限制是什么。

### 20.7 测试结果

至少列出：

- TCP Client。
- Network Mode。
- XYZ。
- Gripper。
- Servo。
- MeArm-3D Sim。
- 状态反馈。
- Serial Mode 代码回归。
- MeArm-3D 原有流程。

每项使用：

```text
PASS
FAIL
NOT TESTED
NOT IMPLEMENTED
```

### 20.8 兼容性报告

明确说明：

- 是否修改 MeArm-3D。
- MeArm-3D 原有流程是否受影响。
- RemoteControl 串口代码是否保留。
- 原有启动方式是否兼容。
- 是否修改 WebSocket。
- 是否修改 Serial Protocol。
- 是否存在状态同步限制。
- 是否存在 TCP 并发风险。
- 是否存在未测试项目。

---

## 21. 最终验收标准

### A. 默认网络模式

```bat
start.bat
```

必须：

- 默认网络模式。
- 不打开串口。
- 可以连接 MeArm-3D TCP Server。
- 可以完成 Sim2Sim 控制。

### B. 串口模式

```bat
start.bat --real
```

必须：

- 进入串口模式。
- 保留原有串口控制。
- 不强制依赖 MeArm-3D。
- 不删除原有真机控制能力。

### C. TCP Client

必须：

- 建立连接。
- 发送符合服务端协议的 JSON。
- 读取并处理响应。
- 处理连接失败。
- 处理连接断开。
- 保护并发写入。
- 正确释放资源。

### D. 控制功能

根据现有网页和服务端实际能力支持：

- XYZ 方向控制。
- 爪开关。
- 四个舵机直接控制。

### E. Sim2Sim

必须验证：

```text
网页摇杆
    ↓
RemoteControl Go Backend
    ↓
TCP Client
    ↓
MeArm-3D TCP Server
    ↓
MeArm-3D Sim
```

### F. 兼容性

必须满足：

- MeArm-3D 当前完整流程不被破坏。
- RemoteControl 原有串口功能保留。
- 默认网络模式。
- `--real` 串口模式。
- 不进行无关重构。
- 不虚构未完成的测试结果。

---

## 22. 实施原则总结

本任务的核心不是重构两个项目，而是：

> 为 `MeArm-RemoteControl` 增加 TCP Client 传输方式，使现有网页摇杆可以通过网络控制 `MeArm-3D`，同时保留原有串口真机控制能力。

优先顺序：

```text
代码审计
    ↓
确认 TCP 协议
    ↓
最小 TCP Client
    ↓
默认网络模式
    ↓
网页摇杆接入
    ↓
Sim2Sim
    ↓
状态反馈
    ↓
串口回归检查
    ↓
Git Diff 审计
```

不要一开始同时修改：

- 网络模式
- 状态系统
- 前端摇杆
- MeArm-3D 核心控制逻辑
- 串口协议
- IK / FK

先完成最小闭环：

```text
RemoteControl 摇杆
    ↓
TCP Client
    ↓
MeArm-3D TCP Server
    ↓
MeArm-3D Sim
```

然后再补充状态反馈和其他兼容性优化。
