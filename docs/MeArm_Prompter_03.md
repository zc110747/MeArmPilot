# MeArmPilot MeArm：MuJoCo 真实物理仿真开发任务

## 1. 项目背景

当前项目：

`https://github.com/zc110747/MeArmPilot`

重点目录：

`MeArm-3D`

当前已经完成：

1. MeArm 机械臂网页 3D 模型
2. 机械臂三维结构显示
3. 机械臂运动学控制
4. 远程仿真
5. 虚拟机械臂能够根据控制指令运动
6. 后端可以配置机械臂各模块长度
7. 虚拟机械臂与当前控制接口已经基本打通

现在进入第二阶段：

> 将当前 MeArm-3D 从“几何/运动学仿真”升级为“MuJoCo 真实物理仿真”。

本阶段暂时不加入 AI 学习、自训练、强化学习等功能。

目标是先建立一个稳定、可重复、可调参数、可验证的 MeArm 物理仿真环境，为后续：

* 真实机械臂控制
* Sim2Real
* 数据采集
* AI Agent 控制
* 自动生成训练数据
* 强化学习

提供底层物理环境。

---

# 2. 总体设计原则

必须遵守：

> 保留现有 MeArm-3D 网页系统，不推翻现有架构。

新增 MuJoCo 作为独立 Physics Backend。

推荐架构：

```text
                ┌─────────────────────┐
                │    MeArm-3D Web UI  │
                │                     │
                │ 3D View             │
                │ Joint Control       │
                │ XYZ Control         │
                │ Status              │
                └──────────┬──────────┘
                           │
                           │ API / WebSocket
                           ▼
                ┌─────────────────────┐
                │   MeArmPilot Backend  │
                │                     │
                │ Control API         │
                │ IK / FK             │
                │ State Manager       │
                │ Config              │
                └──────────┬──────────┘
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
      ┌───────────────┐       ┌────────────────┐
      │ Existing      │       │ MuJoCo Physics │
      │ Kinematic     │       │ Backend        │
      │ Simulation    │       │                │
      └───────────────┘       └────────────────┘
                                      │
                                      ▼
                               ┌──────────────┐
                               │ MJCF Model   │
                               │              │
                               │ bodies       │
                               │ joints       │
                               │ geoms        │
                               │ inertia      │
                               │ actuators    │
                               │ contacts     │
                               └──────────────┘
```

不要让 MuJoCo 代码直接侵入现有 Web UI。

---

# 3. 第一原则：先分析现有项目

开始编码之前，必须完整检查：

```text
MeArm-3D/
```

包括：

* 前端目录
* 后端目录
* 3D 模型
* 机械臂尺寸配置
* joint 定义
* joint angle 定义
* XYZ 坐标系
* IK
* FK
* WebSocket
* REST API
* 远程仿真接口
* 状态同步
* 配置文件
* 启动方式
* 测试代码

重点回答：

### 3.1 当前机械臂有几个实际自由度？

明确：

```text
Base
Shoulder
Elbow
Gripper
```

分别对应什么关节。

如果当前项目使用的是：

```text
4 servo
```

必须明确区分：

```text
Servo angle
Joint angle
Physical joint angle
UI angle
IK angle
```

不要默认这些角度完全相同。

---

# 4. 建立统一机械臂参数模型

MuJoCo 不允许单独维护一套机械臂参数。

必须让：

```text
Arm Configuration
```

成为整个系统的唯一参数来源。

例如：

```yaml
robot:
  name: mearm

  dimensions:
    base_height: ...
    upper_arm_length: ...
    forearm_length: ...
    gripper_length: ...

  joints:
    base:
      min: ...
      max: ...
      home: ...

    shoulder:
      min: ...
      max: ...
      home: ...

    elbow:
      min: ...
      max: ...
      home: ...

    gripper:
      min: ...
      max: ...
      home: ...
```

实际字段根据当前项目已有配置调整。

不要为了 MuJoCo 创建完全重复的硬编码参数。

---

# 5. MuJoCo 模型格式

优先使用：

```text
MJCF
```

不要第一版就复杂化成 URDF。

MuJoCo 原生模型格式就是 MJCF，模型通过 XML 描述 body、joint、geom、actuator 等动力学结构。

推荐目录：

```text
MeArm-3D/
├── frontend/
├── backend/
├── config/
├── simulation/
│   └── mujoco/
│       ├── mearm.xml
│       ├── meshes/
│       ├── materials/
│       ├── assets/
│       ├── config/
│       └── scripts/
├── tests/
└── README.md
```

实际目录结构必须根据现有项目调整。

---

# 6. 从现有 3D 模型生成 MuJoCo 几何模型

不要简单把网页渲染模型直接当成物理模型。

必须区分：

```text
Visual Geometry
Collision Geometry
Physics Geometry
```

例如：

```text
Visual:
    高精度 STL / OBJ / GLB

Collision:
    box
    capsule
    cylinder
    simplified mesh

Physics:
    simplified rigid body geometry
```

第一版优先使用 MuJoCo primitive geom：

```text
box
cylinder
capsule
sphere
```

只有必要时才使用 mesh collision。

原因：

1. 更稳定
2. 更容易调碰撞
3. 更容易调摩擦
4. 更容易调仿真速度
5. 更容易定位物理问题

---

# 7. 建立真实机械臂刚体树

必须建立真实的：

```text
world
 └── base
      └── base_joint
           └── upper_arm
                └── shoulder_joint
                     └── forearm
                          └── elbow_joint
                               └── wrist
                                    └── gripper
```

实际层级必须按照 MeArm 的真实结构修正。

每一个运动部件必须对应：

```text
body
+
joint
+
geom
+
inertial
```

不能只为了“看起来能动”而建立模型。

---

# 8. 关节建模

每个实际旋转轴必须使用：

```xml
<joint type="hinge">
```

明确：

```text
axis
range
limited
damping
frictionloss
armature
```

例如：

```xml
<joint
    name="shoulder"
    type="hinge"
    axis="..."
    range="..."
    limited="true"
    damping="..."
    frictionloss="..."
/>
```

具体参数必须根据实际 MeArm 结构和当前项目参数确定。

不要随意复制网上其他机械臂参数。

---

# 9. 坐标系必须统一

这是本阶段最高优先级风险之一。

必须明确：

```text
World coordinate
Robot coordinate
Web 3D coordinate
MuJoCo coordinate
Camera coordinate
End-effector coordinate
```

统一定义：

```text
X = ?
Y = ?
Z = ?
```

并写入：

```text
README.md
```

例如：

```text
Robot:
X = forward
Y = left/right
Z = up

or

X = left/right
Y = forward/back
Z = up
```

具体方向以当前 MeArmPilot 项目为准。

禁止通过“视觉上旋转模型”解决坐标系错误。

---

# 10. 单位统一

所有物理参数必须使用 SI 单位：

```text
length = meter
mass = kilogram
time = second
angle = radian
angular velocity = rad/s
force = Newton
torque = N·m
```

UI 可以继续使用：

```text
degree
millimeter
```

但是 API / MuJoCo 内部必须统一。

必须明确实现：

```text
deg ↔ rad
mm ↔ m
```

转换层。

---

# 11. 惯性参数

必须为每一个运动 body 建立：

```text
mass
center of mass
inertia
```

第一阶段允许：

```text
geom + density
```

自动估算。

但是必须允许后续替换为：

```text
真实质量
真实 COM
真实 inertia tensor
```

MuJoCo 会根据 MJCF 模型编译生成运行时模型，因此 XML 应作为长期维护的模型源文件，而不是依赖编译后的 MJB 文件作为唯一模型。

必须保留：

```text
mearm.xml
```

作为 source of truth。

---

# 12. Actuator 建模

必须建立：

```text
joint
→ actuator
```

不要第一版直接通过瞬间修改 qpos 模拟机械臂运动。

禁止：

```python
data.qpos[...] = target
```

作为正常控制方式。

真实物理仿真必须通过 actuator / torque / position control 产生运动。

优先实现：

```text
position actuator
```

同时保留后续：

```text
velocity control
torque control
```

的扩展能力。

目标：

```text
command
    ↓
target joint angle
    ↓
controller
    ↓
actuator
    ↓
physical dynamics
    ↓
joint state
```

而不是：

```text
command
    ↓
直接修改 joint position
```

---

# 13. Servo 模型

由于真实 MeArm 使用舵机，MuJoCo 中不能简单假设：

```text
target angle = instant angle
```

第一版建立简化舵机模型：

```text
target angle
    ↓
position controller
    ↓
torque
    ↓
joint dynamics
```

至少支持：

```text
max torque
position gain
damping
velocity limit
angle limit
```

参数全部配置化。

例如：

```yaml
servo:
  max_torque: ...
  kp: ...
  kd: ...
  max_velocity: ...
```

---

# 14. Joint limit

必须严格实现真实机械臂的：

```text
minimum angle
maximum angle
```

并且：

```text
UI limit
IK limit
MuJoCo joint limit
Real servo limit
```

最终必须保持一致。

不要出现：

```text
Web 可以转 180°
MuJoCo 只能转 90°
真实舵机只能转 80°
```

这种参数不一致。

建立统一：

```text
JointLimit
```

配置。

---

# 15. 碰撞模型

第一阶段必须实现：

```text
机械臂 ↔ 地面
机械臂 ↔ 自身
机械臂 ↔ 工作台
```

至少验证：

```text
不会穿透
不会明显爆炸
不会高速弹飞
```

MuJoCo 的 contact 系统支持接触点、摩擦以及不同的接触约束模型，因此本阶段必须把 collision 与 friction 作为真实物理仿真的一等公民，而不是仅作为显示效果。

---

# 16. 自碰撞

必须逐步开启。

第一阶段：

```text
base ↔ arm
arm ↔ forearm
forearm ↔ gripper
```

但是要通过：

```text
exclude
contype
conaffinity
```

避免机械连接处产生无意义碰撞。

不能简单：

```text
所有 geom 全部互相碰撞
```

否则非常容易产生不稳定仿真。

---

# 17. 摩擦

为：

```text
joint
contact
gripper
ground
```

配置合理摩擦。

尤其验证：

```text
机械臂停止后是否自然保持
末端是否因为重力慢慢下坠
抓取物体是否滑落
机械臂碰撞后是否合理反弹
```

不要为了“看起来稳定”把 friction 设置成无限大。

---

# 18. 重力测试

必须提供：

```text
Gravity Test
```

步骤：

### Test 1

所有 actuator 禁用。

机械臂处于：

```text
home position
```

观察：

```text
arm 是否因为重力自然下垂
```

### Test 2

设置不同姿态。

观察：

```text
potential energy
joint movement
static equilibrium
```

### Test 3

逐渐增加 actuator 控制。

观察：

```text
是否能够抵抗重力
```

这是判断动力学模型是否正确的重要测试。

---

# 19. 末端状态

必须提供统一状态：

```json
{
  "joint_angles": [],
  "joint_velocities": [],
  "joint_torques": [],
  "end_effector": {
    "x": 0,
    "y": 0,
    "z": 0
  }
}
```

根据当前 MeArmPilot API 实际结构进行兼容。

必须同时支持：

```text
Joint State
Cartesian State
```

---

# 20. FK 验证

MuJoCo 的 FK 结果必须与现有 MeArmPilot FK 结果进行对比。

建立测试：

```text
输入 joint angles
        ↓
MeArmPilot FK
        ↓
XYZ_A

输入相同 joint angles
        ↓
MuJoCo
        ↓
XYZ_B
```

计算：

```text
error = distance(XYZ_A, XYZ_B)
```

要求：

```text
position error < 1 mm
```

第一阶段如果当前模型坐标/尺寸存在历史误差，可以暂时：

```text
< 5 mm
```

但必须明确记录误差来源。

---

# 21. IK 验证

当前：

```text
XYZ
 ↓
IK
 ↓
joint angles
```

得到：

```text
q1 q2 q3 ...
```

将角度发送给 MuJoCo。

然后读取：

```text
end effector position
```

再次计算：

```text
IK error
```

测试：

```text
多个工作空间随机点
```

至少：

```text
100 个测试点
```

统计：

```text
success rate
mean error
max error
```

---

# 22. 动力学验证

必须增加：

```text
Physics Validation
```

至少包括：

### Test A：静态

机械臂保持不同姿态。

验证：

```text
joint torque
```

是否符合重力趋势。

### Test B：单关节

只运动：

```text
base
```

其他关节锁定。

验证：

```text
position
velocity
acceleration
```

### Test C：双关节

同时运动：

```text
shoulder
elbow
```

观察耦合效果。

### Test D：快速运动

发送：

```text
large angle step
```

验证：

```text
不会数值爆炸
不会穿模
不会无限振荡
```

---

# 23. 仿真时间

必须明确：

```text
simulation timestep
control timestep
render timestep
```

例如：

```text
physics:
1 kHz

controller:
100 Hz

render:
30~60 FPS
```

具体数值可以根据性能调整。

关键原则：

> 渲染帧率不能决定物理仿真步长。

---

# 24. Simulation Loop

推荐结构：

```text
while running:

    receive command

    update target joint

    controller

    mujoco step

    read:
        qpos
        qvel
        actuator force
        contact
        end effector pose

    publish state
```

必须保证：

```text
Physics Loop
```

和：

```text
WebSocket / HTTP
```

解耦。

网络卡顿不能导致物理时间停止。

---

# 25. WebSocket 集成

当前网页远程仿真已经完成。

不要重新设计协议。

增加：

```text
simulation_mode
```

例如：

```text
kinematic
mujoco
```

允许：

```text
Web UI
   ↓
Backend
   ↓
MuJoCo
   ↓
state
   ↓
Web UI
```

实现：

```text
真实物理状态回传网页
```

而不是网页自己计算一个状态。

---

# 26. 双仿真模式

最终必须支持：

```text
Kinematic Simulation
Physics Simulation
```

例如：

```yaml
simulation:
  backend: mujoco
```

或者：

```text
backend=kinematic
backend=mujoco
```

这样可以直接比较：

```text
同一个 command

Kinematic:
    position A

MuJoCo:
    position B
```

---

# 27. MuJoCo Viewer

必须提供独立启动方式：

```bash
python simulation/mujoco/run.py
```

启动后：

```text
MuJoCo Viewer
```

能够看到：

```text
MeArm
ground
coordinate frame
joint state
```

最好增加：

```text
FPS
simulation time
joint angle
end-effector XYZ
contact count
```

调试信息。

---

# 28. 物理参数配置化

禁止把重要参数全部硬编码到 Python。

至少分离：

```text
robot dimensions
mass
density
joint limits
servo torque
kp
kd
friction
gravity
simulation timestep
```

推荐：

```text
config/
    robot.yaml
    physics.yaml
```

或者与当前项目配置系统统一。

---

# 29. 真实机械臂对应关系

必须提前为 Sim2Real 保留接口。

最终形成：

```text
                    ┌──────────────┐
                    │ Command      │
                    └──────┬───────┘
                           │
              ┌────────────┴────────────┐
              │                         │
              ▼                         ▼
       ┌──────────────┐         ┌──────────────┐
       │ MuJoCo       │         │ Real Arm     │
       │ Simulation   │         │ Servo        │
       └──────┬───────┘         └──────┬───────┘
              │                        │
              ▼                        ▼
       Simulated State          Real State
```

两边使用尽量相同：

```text
Command
Joint State
End Effector State
```

接口。

---

# 30. 暂时不要实现的内容

本阶段明确禁止加入：

```text
AI
LLM
Agent
RL
Self Training
Vision Learning
Dataset Training
Policy Training
自动数据采集
```

这些全部留到下一阶段。

当前唯一目标：

> 建立可信的 MeArm 物理仿真基础。

---

# 31. 后续扩展接口

但是架构必须预留：

```text
Simulation Observation
Simulation Action
Simulation Reset
Simulation Step
Simulation State
```

未来可以直接变成：

```text
Agent
 ↓
Observation
 ↓
LLM / Policy
 ↓
Action
 ↓
MuJoCo
 ↓
Observation
```

因此不要把 MuJoCo 写成一个只能人工操作的 Viewer。

---

# 32. Reset API

必须支持：

```text
reset()
```

恢复：

```text
initial joint position
initial velocity = 0
initial objects
initial simulation time
```

并保证每次 reset 后结果可重复。

---

# 33. Deterministic Test

提供：

```text
seed
```

机制。

相同：

```text
model
+
parameters
+
initial state
+
command sequence
```

应该得到高度一致的：

```text
simulation result
```

---

# 34. 数据记录

第一版就加入基础 CSV / JSONL 记录。

例如：

```text
timestamp
simulation_time

joint_angle[]
joint_velocity[]
joint_torque[]

target_joint[]

end_effector_x
end_effector_y
end_effector_z

contact_count
```

不要现在做复杂数据库。

简单：

```text
JSONL
CSV
```

即可。

---

# 35. 自动测试

至少建立：

```text
tests/
├── test_fk.py
├── test_ik.py
├── test_joint_limits.py
├── test_mujoco_model.py
├── test_gravity.py
├── test_actuator.py
└── test_simulation.py
```

---

# 36. 必须实现的自动验收

## Acceptance 1

MuJoCo XML 可以正常加载。

```text
PASS
```

不能存在：

```text
NaN
invalid inertia
invalid joint
invalid geom
```

---

## Acceptance 2

机械臂可以在 MuJoCo Viewer 中正常显示。

---

## Acceptance 3

单关节控制正常。

---

## Acceptance 4

全部关节控制正常。

---

## Acceptance 5

关节限位正常。

超过范围：

```text
不能继续运动
```

---

## Acceptance 6

机械臂受到重力影响。

禁用 actuator 后：

```text
机械臂不会保持绝对静止
```

除非姿态本身处于稳定状态。

---

## Acceptance 7

机械臂碰撞正常。

测试：

```text
arm ↔ ground
arm ↔ arm
```

不能明显穿透。

---

## Acceptance 8

FK 对比：

```text
MeArmPilot FK
vs
MuJoCo FK
```

误差：

```text
目标 < 1 mm
允许初期 < 5 mm
```

---

## Acceptance 9

IK 对比：

至少测试：

```text
100 个随机可达点
```

统计：

```text
success rate
mean error
max error
```

---

## Acceptance 10

Web UI 控制 MuJoCo。

最终流程：

```text
Browser
   ↓
WebSocket
   ↓
Backend
   ↓
MuJoCo
   ↓
Physical Simulation
   ↓
State
   ↓
WebSocket
   ↓
Browser
```

完整打通。

---

# 37. 特别注意：不要伪造“真实物理”

必须区分：

### Level 1

```text
3D animation
```

### Level 2

```text
Kinematic simulation
```

### Level 3

```text
Dynamic simulation
```

### Level 4

```text
Contact + friction + actuator
```

### Level 5

```text
Real hardware calibrated simulation
```

当前目标：

```text
Level 3 → Level 4
```

而不是声称已经达到 Level 5。

如果没有真实：

```text
质量
惯量
舵机扭矩
舵机速度
齿轮间隙
摩擦
机械结构误差
```

必须在 README 中明确：

> 当前为参数化物理仿真，而非真实机械臂完全标定模型。

---

# 38. 参数标定接口

为后续真实机械臂标定预留：

```yaml
calibration:
  servo_offset:
  joint_scale:
  joint_zero:
  max_velocity:
  max_torque:
  damping:
  friction:
```

未来可以通过真实机械臂实验逐渐修正。

---

# 39. 开发顺序

严格按照以下阶段执行。

## Phase 1

分析现有：

```text
MeArm-3D
```

输出：

```text
ARCHITECTURE_ANALYSIS.md
```

明确：

```text
joint
coordinate
dimensions
API
state
```

---

## Phase 2

建立：

```text
MJCF
```

完成：

```text
world
base
links
joints
```

先不加入复杂碰撞。

---

## Phase 3

加入：

```text
mass
inertia
gravity
```

完成动力学。

---

## Phase 4

加入：

```text
actuator
position control
joint limit
```

完成基本控制。

---

## Phase 5

加入：

```text
collision
contact
friction
```

---

## Phase 6

实现：

```text
MuJoCo Python Backend
```

提供：

```text
load
reset
step
command
state
```

---

## Phase 7

接入现有：

```text
WebSocket
```

---

## Phase 8

网页实时显示：

```text
MuJoCo state
```

---

## Phase 9

完成：

```text
FK / IK / Physics
```

三者一致性验证。

---

## Phase 10

完善：

```text
tests
README
parameter configuration
logging
```

---

# 40. 最终架构目标

完成后：

```text
                   MeArmPilot
                      │
        ┌─────────────┴─────────────┐
        │                           │
        ▼                           ▼
    MeArm-3D                   Simulation
    Web UI                         │
        │                    ┌─────┴─────┐
        │                    │           │
        │                    ▼           ▼
        │               Kinematic     MuJoCo
        │                             Physics
        │                                │
        └──────────────┬─────────────────┘
                       │
                       ▼
                 Unified State
                       │
                       ▼
                 Real Robot API
```

核心原则：

> UI 不关心底层到底是 Kinematic 还是 MuJoCo。

> Controller 不应该依赖具体 MuJoCo 实现。

> MuJoCo 是 Physics Backend。

> Real Robot 将来也是另一个 Backend。

最终形成：

```text
Command
  ↓
Robot Abstraction
  ↓
┌───────────────┬───────────────┐
│               │               │
Kinematic     MuJoCo         RealRobot
│               │               │
└───────────────┴───────────────┘
                ↓
             State
```

---

# 41. 开发完成后必须输出

最终必须提供：

```text
1. 修改后的项目结构

2. MuJoCo MJCF 模型

3. MuJoCo Python Backend

4. Physics 参数配置

5. Joint 参数配置

6. WebSocket 接口说明

7. MuJoCo Viewer 启动方法

8. Web UI + MuJoCo 联调方法

9. FK/IK 对比结果

10. 物理仿真测试结果

11. 碰撞测试结果

12. 重力测试结果

13. 参数说明

14. README 更新

15. 已知误差和限制
```

---

# 42. 最终验收标准

只有同时满足以下条件才算完成：

```text
[ ] 现有网页 3D 没有被破坏
[ ] 原有远程仿真仍然可以运行
[ ] MuJoCo 模型可以独立启动
[ ] 所有关节具有真实动力学
[ ] gravity 生效
[ ] actuator 生效
[ ] joint limit 生效
[ ] collision 生效
[ ] friction 生效
[ ] FK 与原系统基本一致
[ ] IK + MuJoCo 闭环正常
[ ] WebSocket 可以控制 MuJoCo
[ ] MuJoCo 状态可以回传 Web
[ ] reset 正常
[ ] 多次运行结果稳定
[ ] 测试自动化
[ ] 参数没有大量硬编码
[ ] README 完整
[ ] 没有引入 AI / RL / Training
```

---

# 43. 重要开发纪律

不要一次性把整个 MuJoCo 系统写完。

必须：

```text
分析
↓
建模
↓
加载
↓
重力
↓
单关节
↓
多关节
↓
执行器
↓
碰撞
↓
WebSocket
↓
FK/IK
↓
自动测试
```

每完成一个阶段：

```text
build
run
test
verify
```

确认通过后再进入下一阶段。

如果某一步失败：

> 不允许继续堆代码掩盖问题。

必须定位：

```text
coordinate
geometry
joint axis
inertia
actuator
solver
contact
controller
```

中的具体问题。

---

# 44. 本阶段最终目标

最终不是简单得到：

```text
一个 MuJoCo MeArm 模型
```

而是建立：

> **MeArmPilot 的统一机器人仿真抽象层。**

让同一套：

```text
IK
Command
Joint State
End Effector State
```

可以驱动：

```text
网页运动学模型
        ↓
MuJoCo 物理模型
        ↓
真实 MeArm
```

为下一阶段的：

```text
Camera
+
Vision
+
Agent
+
Data Collection
+
Sim2Real
+
Self Training
```

留下稳定接口。

本阶段不要提前实现这些 AI 功能。

先把：

> **“虚拟机械臂真的按照物理规律运动，并且和未来真实机械臂共享同一套控制接口”**

这件事情做好。
