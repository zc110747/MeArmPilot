# ArmPilot：mARM 虚拟建模 + 真实机械臂同步控制系统

你是一名机器人运动学、Three.js、React、机械臂控制和嵌入式通信专家。

现在开发 MeArmPilot 第一阶段。

本阶段的核心目标只有一个：

> **建立一个虚拟 mARM 机械臂，并让它与真实 mARM 机械臂的一一对应控制。**

暂时不要实现 AI、机器学习、强化学习、视觉识别、数据集、模型训练等功能。

但架构必须为未来接入这些能力保留接口。

---

# 一、核心目标

系统必须实现：

```text
                 MeArmPilot
                    │
              RobotModel
                    │
          ┌─────────┴─────────┐
          │                   │
          ↓                   ↓
    Virtual Robot        Real Robot
      Three.js               AVR
          │                   │
          └─────────┬─────────┘
                    │
                Joint State
                    │
                  FK/IK
```

用户在网页中操作虚拟机械臂：

```text
虚拟机械臂
    ↓
Joint State
    ↓
安全检查
    ↓
Actuator Mapping
    ↓
通信
    ↓
真实机械臂
```

真实机械臂状态发生变化时：

```text
真实机械臂
    ↓
状态反馈
    ↓
Backend
    ↓
WebSocket
    ↓
Virtual Robot
```

虚拟机械臂与真实机械臂始终保持同步。

---

# 二、最重要的设计原则

本项目不是简单的：

```text
网页 3D Demo
```

而是：

```text
Virtual Robot
+
Real Robot
+
Common Robot Model
```

两个机械臂必须使用同一个：

```text
RobotModel
```

因此：

```text
RobotModel
      │
 ┌────┴────┐
 ↓         ↓
Virtual   Real
Robot     Robot
```

禁止分别维护两套：

```text
Joint
Link
Angle
Limit
Coordinate
```

---

# 三、技术栈

前端：

```text
React
TypeScript
Vite
Three.js
React Three Fiber
@react-three/drei
Zustand
```

后端：

```text
Go
WebSocket
HTTP REST API
```

真实机械臂：

```text
AVR
Serial
115200 baud
```

通信方式：

```text
Browser
    ↓
WebSocket
    ↓
Go Server
    ↓
Serial
    ↓
AVR
    ↓
Servo
```

---

# 四、第一阶段明确不实现

禁止主动实现以下内容：

```text
AI
机器学习
强化学习
PPO
SAC
视觉识别
摄像头
目标检测
数据集
模型训练
ONNX
语音控制
动作学习
MuJoCo训练
Sim2Real
```

不要为了“以后扩展”提前实现这些复杂功能。

只保留接口扩展能力。

---

# 五、机器人模型

使用 mARM / meArm 类机械臂。

第一阶段抽象为：

```text
Base
    ↓
Shoulder
    ↓
Elbow
    ↓
Gripper
```

必须严格区分：

```text
Link
Joint
Actuator
```

---

# 六、RobotModel

创建：

```text
src/robot/
```

建议：

```text
robot/
├── model/
│   ├── RobotModel.ts
│   ├── Link.ts
│   ├── Joint.ts
│   ├── Actuator.ts
│   └── Pose.ts
│
├── kinematics/
│   ├── fk.ts
│   ├── ik.ts
│   └── coordinate.ts
│
├── calibration/
│   └── calibration.ts
│
└── transport/
    ├── RobotTransport.ts
    ├── MockTransport.ts
    └── WebSocketTransport.ts
```

---

# 七、RobotModel 数据结构

定义统一模型：

```typescript
interface RobotModel {

    id: string;

    name: string;

    units: "mm";

    links: Link[];

    joints: Joint[];

    actuators: Actuator[];

    homePose: JointState;
}
```

Link：

```typescript
interface Link {

    id: string;

    name: string;

    parent?: string;

    length: number;

    geometry: LinkGeometry;
}
```

Joint：

```typescript
interface Joint {

    id: string;

    name: string;

    parentLink: string;

    childLink: string;

    type: "revolute" | "fixed";

    axis: [number, number, number];

    origin: {
        position: [number, number, number];
        rotation: [number, number, number];
    };

    limits: {
        min: number;
        max: number;
    };
}
```

Actuator：

```typescript
interface Actuator {

    id: string;

    jointId: string;

    offset: number;

    scale: number;

    reverse: boolean;

    limits: {
        min: number;
        max: number;
    };
}
```

---

# 八、配置文件

机器人参数必须独立于代码。

使用：

```text
config/robot.yaml
```

例如：

```yaml
version: 1

robot:
  id: mearm
  name: mARM
  units: mm

links:

  - id: base
    length: 60

  - id: upper_arm
    length: 80

  - id: forearm
    length: 80

  - id: gripper
    length: 40

joints:

  - id: base
    name: Base
    type: revolute
    axis: [0, 1, 0]
    limit:
      min: -90
      max: 90

  - id: shoulder
    name: Shoulder
    type: revolute
    axis: [0, 0, 1]
    limit:
      min: -60
      max: 90

  - id: elbow
    name: Elbow
    type: revolute
    axis: [0, 0, 1]
    limit:
      min: -90
      max: 90

  - id: gripper
    name: Gripper
    type: revolute
    axis: [0, 0, 1]
    limit:
      min: 0
      max: 90
```

以上尺寸和角度只是默认值。

实际机械臂必须允许通过配置修改。

---

# 九、坐标系统

统一使用：

```text
右手坐标系
```

内部长度统一：

```text
mm
```

定义：

```text
X：机械臂前后
Y：机械臂左右
Z：机械臂上下
```

必须在项目文档中明确坐标定义。

Three.js、FK、IK、真实机械臂之间必须有明确转换层。

禁止在业务代码中出现大量：

```text
-x
-y
90-angle
180-angle
```

所有转换必须集中到：

```text
coordinate.ts
```

---

# 十、虚拟机械臂

使用：

```text
React Three Fiber
```

建立真实的 Joint Tree。

必须：

```text
Robot
 └── Base
      └── J1
           └── UpperArm
                └── J2
                     └── ForeArm
                          └── J3
                               └── Gripper
```

每个 Joint 必须对应：

```text
THREE.Group
```

例如：

```text
RobotGroup
 └── BaseGroup
      └── Joint1Group
           └── UpperArmMesh
                └── Joint2Group
                     └── ForeArmMesh
                          └── Joint3Group
                               └── GripperMesh
```

禁止直接修改 Mesh 的位置来模拟关节运动。

---

# 十一、参数化模型

第一阶段不要求真实 CAD 外观。

使用：

```text
Box
Cylinder
Sphere
```

等基础几何体。

重点是：

```text
尺寸正确
关节位置正确
旋转轴正确
层级正确
```

后续可以替换：

```text
GLB
GLTF
OBJ
```

但不能影响运动学。

---

# 十二、FK

实现：

```typescript
forwardKinematics(jointState)
```

输入：

```text
J1
J2
J3
Gripper
```

输出：

```text
Joint Transform
End Effector Pose
```

例如：

```typescript
interface RobotPose {

    joints: Record<string, Transform>;

    endEffector: {
        position: Vector3;
        rotation: Euler;
    };
}
```

要求：

```text
FK计算结果
=
Three.js实际模型位置
```

这是非常重要的验收条件。

---

# 十三、IK

实现：

```typescript
solveIK(targetPose)
```

第一阶段只要求：

```text
XYZ → J1/J2/J3
```

输入：

```text
X
Y
Z
```

输出：

```text
J1
J2
J3
```

必须检查：

```text
工作空间
关节限位
```

无解时返回：

```typescript
{
    success: false,
    reason: "OUT_OF_WORKSPACE"
}
```

或者：

```text
JOINT_LIMIT
```

---

# 十四、虚拟机械臂控制

界面支持：

```text
J1 Slider
J2 Slider
J3 Slider
Gripper Slider
```

改变：

```text
Joint State
```

立即更新：

```text
3D Model
```

同时显示：

```text
Joint Angle
End Effector XYZ
```

---

# 十五、XYZ控制

提供：

```text
X
Y
Z
```

用户输入目标：

```text
X = 120
Y = 20
Z = 80
```

执行：

```text
XYZ
 ↓
IK
 ↓
Joint State
 ↓
FK
 ↓
Virtual Robot
```

---

# 十六、鼠标拖动末端

3D 场景支持：

```text
拖动 End Effector
```

流程：

```text
Mouse
 ↓
Target XYZ
 ↓
IK
 ↓
Joint State
 ↓
3D Robot
```

拖动过程中必须实时更新。

---

# 十七、Robot State

建立唯一状态：

```typescript
interface RobotState {

    joints: Record<string, number>;

    endEffector: Pose;

    timestamp: number;

    source:
        | "virtual"
        | "real"
        | "command";
}
```

任何机器人状态变化都必须经过：

```text
RobotState
```

---

# 十八、虚拟机器人与真实机器人同步

这是本项目最核心的功能。

定义：

```text
Virtual Robot
Real Robot
```

都使用：

```text
RobotState
```

例如：

```text
Virtual Robot
      ↓
RobotState
      ↓
Transport
      ↓
Real Robot
```

反向：

```text
Real Robot
      ↓
RobotState
      ↓
Virtual Robot
```

因此最终形成：

```text
Virtual Robot ←→ RobotState ←→ Real Robot
```

---

# 十九、控制权

必须明确当前控制源：

```text
ControlSource
```

支持：

```text
virtual
real
```

默认：

```text
virtual
```

当用户拖动虚拟机械臂：

```text
virtual
 ↓
command
 ↓
real
```

真实机械臂反馈：

```text
real
 ↓
state
 ↓
virtual
```

避免两个方向互相触发无限循环。

必须设计：

```text
command
state
feedback
```

三种消息概念。

---

# 二十、真实机械臂通信

定义：

```typescript
interface RobotTransport {

    connect(): Promise<void>;

    disconnect(): Promise<void>;

    sendJointState(
        state: JointState
    ): Promise<void>;

    onState(
        callback: (state: RobotState) => void
    ): void;
}
```

实现：

```text
MockTransport
WebSocketTransport
```

第一阶段优先：

```text
MockTransport
```

确保虚拟控制链路完全正确。

---

# 二十一、WebSocket

浏览器：

```text
Browser
 ↓
WebSocket
 ↓
Go Server
```

消息统一：

```json
{
    "version": 1,
    "type": "joint_command",
    "timestamp": 123456,
    "joints": {
        "base": 0,
        "shoulder": 20,
        "elbow": 30,
        "gripper": 10
    }
}
```

真实状态：

```json
{
    "version": 1,
    "type": "joint_state",
    "timestamp": 123456,
    "joints": {
        "base": 0,
        "shoulder": 19.8,
        "elbow": 30.2,
        "gripper": 10
    }
}
```

---

# 二十二、Go Server

后端负责：

```text
WebSocket
Serial
Robot Configuration
```

建议：

```text
server/
├── main.go
├── websocket/
├── serial/
├── robot/
└── protocol/
```

职责：

```text
WebSocket Client
        ↓
Protocol
        ↓
Robot Controller
        ↓
Serial
        ↓
AVR
```

不要让 WebSocket 层直接操作串口。

---

# 二十三、真实机械臂协议

第一阶段使用简单文本协议即可。

例如：

```text
MOVE 90 45 60 20
```

含义：

```text
J1 = 90
J2 = 45
J3 = 60
G  = 20
```

反馈：

```text
STATE 90 45 60 20
```

错误：

```text
ERROR LIMIT
```

后续可以升级为：

```text
MeArmPilot Serial Protocol v1
```

但必须保持：

```text
Transport
Protocol
RobotModel
```

相互独立。

---

# 二十四、舵机校准

真实舵机角度与虚拟 Joint Angle 不一定相同。

建立：

```text
Joint Angle
      ↓
Calibration
      ↓
Servo Angle
```

参数：

```text
offset
scale
reverse
min
max
```

例如：

```typescript
servoAngle =
    jointAngle * scale
    + offset;
```

如果：

```text
reverse = true
```

则：

```text
servoAngle =
    -jointAngle * scale
    + offset;
```

校准不能进入 IK。

---

# 二十五、双舵机关节

如果实际 mARM 某个机械关节由两个舵机共同驱动：

```text
Joint
 ├── Servo A
 └── Servo B
```

则：

```text
IK
 ↓
Joint Angle
 ↓
Actuator Mapping
 ↓
Servo A
Servo B
```

不要让 IK 直接输出：

```text
Servo A
Servo B
```

IK 永远只输出：

```text
Joint State
```

---

# 二十六、真实反馈同步

如果 AVR 可以反馈舵机实际位置：

```text
AVR
 ↓
STATE
 ↓
Go
 ↓
WebSocket
 ↓
RobotState
 ↓
Three.js
```

虚拟机械臂必须显示：

**真实机械臂当前状态。**

因此：

```text
command angle
```

与：

```text
actual angle
```

必须区分。

---

# 二十七、状态显示

网页必须显示：

```text
Connection:
Connected / Disconnected

Robot:
Virtual / Real

Control:
Virtual / Real

J1:
Command xx°
Actual xx°

J2:
Command xx°
Actual xx°

J3:
Command xx°
Actual xx°

Gripper:
Command xx°
Actual xx°
```

同时显示：

```text
End Effector:
X
Y
Z
```

---

# 二十八、3D可视化

场景必须提供：

```text
Grid
World Axis
Robot Axis
Joint Axis
Camera Orbit
Zoom
Pan
```

提供：

```text
Reset Camera
Home Pose
```

---

# 二十九、Home Pose

机器人必须拥有：

```text
homePose
```

点击：

```text
HOME
```

执行：

```text
Virtual Robot
 ↓
Home Joint State
 ↓
IK/FK
```

如果连接真实机械臂：

```text
Home Joint State
 ↓
Safety Check
 ↓
Real Robot
```

禁止直接发送未经检查的 Home 命令。

---

# 三十、安全机制

真实机械臂控制必须经过：

```text
Joint Limit
Workspace Limit
Servo Limit
Connection Check
Emergency Stop
```

最终：

```text
Command
 ↓
Joint Limit
 ↓
Actuator Limit
 ↓
Safety
 ↓
Transport
 ↓
Real Robot
```

任何失败：

```text
禁止发送。
```

---

# 三十一、Simulation / Real模式

网页提供：

```text
Simulation
Real Robot
```

Simulation：

```text
操作
 ↓
Virtual Robot
```

Real Robot：

```text
操作
 ↓
Virtual Robot
 ↓
Safety
 ↓
WebSocket
 ↓
AVR
```

默认启动：

```text
Simulation
```

防止网页启动后直接控制真实机械臂。

---

# 三十二、轨迹

本阶段只实现基础轨迹。

支持：

```text
P0
P1
P2
P3
```

通过：

```text
Linear Interpolation
```

生成：

```text
Joint Trajectory
```

轨迹播放时：

```text
Trajectory
 ↓
Joint State
 ↓
Virtual Robot
```

如果 Real Mode：

```text
Trajectory
 ↓
Safety
 ↓
Real Robot
```

---

# 三十三、虚拟与真实一一对应

最终必须做到：

```text
虚拟 J1 = 30°
        ↓
真实 J1 = 30°

虚拟 J2 = 45°
        ↓
真实 J2 = 45°

虚拟 J3 = 20°
        ↓
真实 J3 = 20°
```

反向：

```text
真实 J1 = 35°
        ↓
虚拟 J1 = 35°
```

但允许存在：

```text
Joint ↔ Servo
```

之间的：

```text
offset
scale
reverse
```

因此“一一对应”指的是：

**Joint State 一一对应，而不是 Servo PWM 数值一一对应。**

---

# 三十四、误差显示

如果真实机械臂反馈：

```text
Command
Actual
```

则实时计算：

```text
Error = Actual - Command
```

显示：

```text
J1 Error
J2 Error
J3 Error
Gripper Error
```

同时：

```text
Position Error
```

---

# 三十五、连接异常

如果：

```text
WebSocket断开
```

必须：

```text
停止发送
```

如果：

```text
Serial断开
```

必须：

```text
Real Mode → Safe State
```

不能继续缓存大量运动命令。

---

# 三十六、日志

网页提供：

```text
Command Log
```

例如：

```text
09:30:01 SEND MOVE 90 45 30 10
09:30:02 STATE 89 45 30 10
09:30:02 ERROR J1 = -1°
```

后端也必须记录：

```text
WebSocket
Serial
Protocol
Error
```

---

# 三十七、文件结构

推荐：

```text
MeArmPilot/

├── frontend/
│
│   ├── src/
│   │
│   ├── robot/
│   │   ├── model/
│   │   ├── kinematics/
│   │   ├── calibration/
│   │   └── transport/
│   │
│   ├── components/
│   │   ├── RobotScene/
│   │   ├── RobotControl/
│   │   ├── RobotConfig/
│   │   └── StatusPanel/
│   │
│   ├── store/
│   │
│   └── protocol/
│
├── backend/
│   ├── main.go
│   ├── websocket/
│   ├── serial/
│   ├── robot/
│   └── protocol/
│
├── config/
│   └── robot.yaml
│
├── protocol/
│   └── serial-v1.md
│
└── tests/
```

---

# 三十八、后续扩展接口

本阶段不要实现 AI。

但以下接口必须保持可扩展：

```text
RobotCommand
RobotState
RobotModel
RobotTransport
```

以后可以加入：

```text
Camera
AI
Voice
MuJoCo
```

但它们都应该最终输出：

```text
RobotCommand
```

例如未来：

```text
Vision
 ↓
Target Pose
 ↓
IK
 ↓
RobotCommand
```

或者：

```text
AI
 ↓
RobotCommand
```

但当前版本不要实现。

---

# 三十九、MuJoCo接口

本阶段不要求真正运行 MuJoCo。

只保留：

```text
SimulationAdapter
```

接口：

```typescript
interface SimulationAdapter {

    loadRobot(
        model: RobotModel
    ): Promise<void>;

    setJointState(
        state: JointState
    ): void;

    getJointState(): JointState;

}
```

当前实现：

```text
ThreeSimulationAdapter
```

以后可以增加：

```text
MuJoCoSimulationAdapter
```

这样不会破坏现有架构。

---

# 四十、第一阶段开发顺序

严格按照：

```text
Phase 1
RobotModel
```

↓

```text
Phase 2
Three.js 3D Robot
```

↓

```text
Phase 3
FK
```

↓

```text
Phase 4
Joint Control
```

↓

```text
Phase 5
IK
```

↓

```text
Phase 6
XYZ / Mouse Control
```

↓

```text
Phase 7
MockTransport
```

↓

```text
Phase 8
Go WebSocket
```

↓

```text
Phase 9
Serial
```

↓

```text
Phase 10
Real Robot
```

↓

```text
Phase 11
Real Feedback
```

↓

```text
Phase 12
Virtual / Real Synchronization
```

---

# 四十一、Phase 1 验收

必须能够：

```text
robot.yaml
 ↓
RobotModel
```

修改：

```text
link length
joint limit
```

模型正确更新。

验收：

```text
RobotModel 是唯一数据源
```

---

# 四十二、Phase 2 验收

3D场景能够正确显示：

```text
Base
Upper Arm
Forearm
Gripper
```

每个 Joint 都有正确：

```text
Origin
Axis
Parent
Child
```

---

# 四十三、Phase 3 验收

执行：

```text
Joint State
 ↓
FK
```

得到：

```text
End Effector XYZ
```

并与 Three.js 实际位置比较。

要求：

```text
误差 < 0.1 mm
```

如果误差无法达到要求，不进入下一阶段。

---

# 四十四、Phase 4 验收

滑动：

```text
J1
J2
J3
```

机械臂必须正确运动。

重点检查：

```text
旋转方向
旋转轴
零位
角度范围
```

---

# 四十五、Phase 5 验收

输入：

```text
XYZ
```

执行：

```text
IK
 ↓
FK
```

要求：

```text
FK(IK(XYZ)) ≈ XYZ
```

同时检查：

```text
Joint Limit
Workspace
```

---

# 四十六、Phase 6 验收

用户拖动末端：

```text
Mouse
 ↓
XYZ
 ↓
IK
 ↓
Virtual Robot
```

要求：

```text
连续
平滑
无明显跳变
```

---

# 四十七、Phase 7～8 验收

先不连接真实机械臂。

使用：

```text
MockTransport
```

测试：

```text
Virtual Robot
 ↓
RobotCommand
 ↓
Mock Robot
 ↓
RobotState
 ↓
Virtual Robot
```

验证完整闭环。

---

# 四十八、Phase 9 验收

Go Server：

```text
WebSocket
+
Serial
```

必须能够：

```text
Browser
 ↓
WebSocket
 ↓
Go
 ↓
Serial
 ↓
AVR
```

同时：

```text
AVR
 ↓
Serial
 ↓
Go
 ↓
WebSocket
 ↓
Browser
```

---

# 四十九、Phase 10 真实机械臂验收

第一次连接真实机械臂时：

**默认禁止自动运动。**

流程：

```text
连接
 ↓
读取状态
 ↓
显示状态
 ↓
用户确认
 ↓
允许控制
```

首先只允许：

```text
单关节小角度移动
```

例如：

```text
J1 +5°
```

确认：

```text
虚拟方向
=
真实方向
```

然后再：

```text
J1 ±10°
J2 ±10°
J3 ±10°
```

最后才允许完整运动。

---

# 五十、最终验收

最终必须实现：

```text
                RobotModel
                     │
          ┌──────────┴──────────┐
          │                     │
          ↓                     ↓
     Virtual Robot          Real Robot
          │                     │
          └──────────┬──────────┘
                     │
                 Joint State
```

完整链路：

```text
用户操作
   ↓
Virtual Robot
   ↓
Joint State
   ↓
Safety
   ↓
Actuator Mapping
   ↓
WebSocket
   ↓
Go
   ↓
Serial
   ↓
AVR
   ↓
Servo
```

反馈：

```text
Servo
 ↓
AVR
 ↓
Serial
 ↓
Go
 ↓
WebSocket
 ↓
RobotState
 ↓
Virtual Robot
```

最终实现：

> **用户看到的虚拟 mARM，就是现实 mARM 的实时数字映射。**

---

# 五十一、Agent执行规则

每完成一个 Phase：

必须：

```text
1. 编译
2. 单元测试
3. 运行
4. 验证
5. 汇报
```

汇报格式：

```text
Phase:
完成内容:

修改文件:

测试:

通过:
失败:

当前已知问题:

下一阶段:
```

禁止一次性跳过多个 Phase。

如果 FK、IK、坐标系存在问题，必须停止，不允许继续堆叠上层功能。

---

# 五十二、核心原则

这个版本只解决：

```text
“虚拟机械臂能不能正确代表真实机械臂”
```

以及：

```text
“操作虚拟机械臂能不能准确控制真实机械臂”
```

因此最重要的三个东西是：

```text
RobotModel
Kinematics
RobotState
```

其次：

```text
Transport
Calibration
Safety
```

最后：

```text
Three.js UI
```

不要为了视觉效果牺牲机器人模型和运动学正确性。
