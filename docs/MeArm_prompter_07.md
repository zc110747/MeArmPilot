# MeArmPilot mini_robot Robot Package 验证任务

## 1. 任务目标

在当前 MeArmPilot 已完成的：

```text
Core
+
Robot Package
+
Working Robot
+
Build / Update
+
Runtime
+
Validation
```

架构基础上，新增一个**纯虚拟测试机器人 `mini_robot`**。

`mini_robot` 的唯一目的不是作为正式机器人使用，而是作为：

> **MeArmPilot Robot Package Framework 的最小验证样本（Reference Test Robot）**

用于验证当前框架是否真正实现：

```text
Robot Package 独立定义机器人
        ↓
Working Robot 自动部署
        ↓
Core 提供共享机制
        ↓
Frontend / Backend / Simulation 使用统一接口
        ↓
FK / IK / Joint / Capability / Validation 正常工作
        ↓
切换机器人时不修改 Core 和其他 Robot Package
```

---

# 2. 核心原则

## 2.1 mini_robot 不是 MeArm 的复制品

禁止直接复制：

```text
robot-package/mearm-v1
```

然后简单修改名称。

必须从 Robot Package Specification 重新实现。

---

## 2.2 mini_robot 必须故意与 MeArm 存在结构差异

建议设计为：

```text
mini_robot
├── 3 个主动关节
├── 1 个夹爪
├── 简单串联机械结构
├── 不使用 MeArm 四连杆机构
├── 不使用 MeArm FK/IK 公式
├── 不依赖 MeArm 特殊关节映射
└── 独立 MJCF
```

推荐：

```text
J1 base_yaw
J2 shoulder
J3 elbow
J4 gripper
```

即：

```text
             gripper
                |
              elbow
                |
             shoulder
                |
              base
```

其中：

```text
J1 = Base Yaw
J2 = Shoulder Pitch
J3 = Elbow Pitch
J4 = Gripper
```

机械臂主体为简单 3R 串联结构。

---

# 3. mini_robot 的运动学定义

采用简单、容易人工验证的参数。

例如：

```yaml
links:
  l1: 100mm
  l2: 80mm
  tool: 50mm
```

坐标系统一采用：

```text
X：机械臂前方
Y：机械臂左方
Z：向上
```

Base 位于：

```text
(0, 0, 0)
```

关节：

```text
J1:
    axis = Z
    type = revolute

J2:
    axis = Y
    type = revolute

J3:
    axis = Y
    type = revolute

J4:
    type = gripper
```

---

# 4. FK

mini_robot 必须拥有自己独立的：

```text
robot-package/mini_robot/kinematics/fk.*
```

不得调用：

```text
mearm-v1/kinematics/fk.*
```

也不得在 Core 中加入：

```text
if robot == "mini_robot"
```

这种机器人特判。

FK 至少支持：

```text
joint angles
    ↓
T_base_tool
    ↓
XYZ
```

验证：

```text
J1 = 0
J2 = 0
J3 = 0
```

应得到一个确定、容易人工计算的末端位置。

另外增加：

```text
J1 = 90°
```

验证机械臂绕 Z 轴旋转后的坐标变化。

---

# 5. IK

mini_robot 实现独立 IK。

由于其结构简单，优先使用标准解析 IK。

至少支持：

```text
XYZ
    ↓
J1
J2
J3
```

并处理：

```text
OUT_OF_WORKSPACE
JOINT_LIMIT
INVALID_INPUT
```

要求：

```text
FK(IK(XYZ))
```

能够恢复目标位置。

测试至少包含：

```text
10 个固定点
100 个随机可达点
```

允许数值误差：

```text
position error < 1e-6 mm
```

如果当前 Core 已经提供通用数值 IK：

```text
DLS
Jacobian
Numerical IK
```

可以额外验证：

```text
Robot Package IK
```

和：

```text
Core Numerical IK
```

是否都可以工作。

但：

> 不要为了 mini_robot 修改 Core IK 算法。

---

# 6. Joint Definition

mini_robot 必须自己定义完整 Joint Metadata。

例如：

```yaml
joints:

  - id: base_yaw
    type: revolute
    min: -90
    max: 90
    home: 0

  - id: shoulder
    type: revolute
    min: -60
    max: 90
    home: 0

  - id: elbow
    type: revolute
    min: -90
    max: 90
    home: 0

  - id: gripper
    type: gripper
    min: 0
    max: 1
    home: 0
```

具体格式必须遵循当前 MeArmPilot Robot Package Specification。

不要为了 mini_robot 新建第二套 Manifest 格式。

---

# 7. Capability

mini_robot 必须完整声明自己的能力。

例如：

```yaml
capabilities:

  simulation: true

  fk: true

  ik: true

  position: true

  orientation: false

  gripper: true

  hardware: false

  vision: false
```

特别注意：

```text
orientation = false
hardware = false
vision = false
```

这是为了验证：

> Robot Package 可以声明“我不支持什么”。

Core / Frontend 不允许默认假设所有机器人都支持：

```text
6D pose
hardware
vision
```

---

# 8. 3D Model

实现：

```text
robot-package/mini_robot/model/
```

提供一个非常简单的 3D 模型。

可以使用：

```text
Box
Cylinder
```

等基础几何体。

不需要复杂 CAD。

建议：

```text
base
 └── shoulder_link
      └── upper_arm
           └── forearm
                └── gripper
```

要求：

1. Three.js 可以正常加载。
2. 关节层级正确。
3. Joint 旋转方向正确。
4. FK 与 3D 模型的位置一致。
5. Gripper 可以开合。
6. 不允许直接引用 MeArm 模型。

---

# 9. MuJoCo

创建：

```text
robot-package/mini_robot/physics/
```

至少包含：

```text
mini_robot.xml
scene.xml
```

实现：

```text
base
 └── shoulder
      └── elbow
           └── gripper
```

要求：

* 使用 MuJoCo 标准 hinge joints。
* Joint limits 与 manifest 一致。
* actuator 定义完整。
* 重力开启。
* 可以进行 forward simulation。
* 可以读取 joint state。
* 可以读取 tool position。
* 可以被 Core MuJoCo Runtime 加载。

不要复制 MeArm MJCF。

---

# 10. Sim2Sim 验证

必须实现：

```text
FK
 ↓
Mini Robot 3D
 ↓
MuJoCo
```

进行至少：

```text
100 组随机 Joint State
```

验证：

```text
FK Tool Position
        vs
MuJoCo Tool Position
```

误差要求：

```text
< 1e-3 mm
```

如果当前框架的单位/坐标系存在统一规范，则必须严格遵循框架规范。

如果存在坐标转换：

```text
Robot Coordinate
↔
MuJoCo Coordinate
↔
Three.js Coordinate
```

必须明确写入 Robot Package，而不能隐藏在 Core 中。

---

# 11. Robot-specific Tests

所有 mini_robot 特有测试必须放：

```text
robot-package/mini_robot/tests/
```

不得放入：

```text
tests/mearm
tests/mini_robot
```

这种长期机器人专属目录。

建议：

```text
robot-package/mini_robot/tests/
├── fk-cases.json
├── ik-cases.json
├── joint-cases.json
├── workspace-cases.json
├── capability-cases.json
└── sim2sim-cases.json
```

具体格式优先复用 Core Test Runner 当前支持的 Case Format。

如果当前框架没有统一格式：

> 先分析现有 Core Validation 机制，再设计最小扩展。

不要为了 mini_robot 复制一套 Test Runner。

---

# 12. Validation

mini_robot 必须能够由 Core Validation Framework 自动执行：

```text
Package Validation
        ↓
Manifest Validation
        ↓
Asset Validation
        ↓
Joint Validation
        ↓
Capability Validation
        ↓
FK Validation
        ↓
IK Validation
        ↓
Workspace Validation
        ↓
MuJoCo Validation
        ↓
Sim2Sim Validation
```

输出：

```text
PASS / FAIL
```

以及：

```text
robot_id
package_version
test_count
passed
failed
duration
```

---

# 13. Working Robot 验证

这是 mini_robot 最重要的测试之一。

执行：

```text
update.bat
```

或当前框架规定的更新命令。

指定：

```text
robot = mini_robot
```

系统应该自动：

```text
robot-package/mini_robot
        ↓
Working Robot
        ↓
working-robot/package/
```

最终：

```text
working-robot/
├── manifest.yaml
├── package/
├── generated/
├── dist/
├── backend/
├── assets/
├── tests/
└── build/
```

必须只有：

```text
mini_robot
```

不能出现：

```text
working-robot/mearm-v1
working-robot/mini_robot
```

这种多机器人并存结构。

---

# 14. Working Robot 稳定路径验证

验证：

```text
Backend
Frontend
MuJoCo Runtime
Test Runner
```

是否都能够只依赖：

```text
working-robot/
```

而不是：

```text
robot-package/mearm-v1/
robot-package/mini_robot/
```

运行时不允许出现：

```text
if robot == "mearm-v1"
```

或：

```text
if robot == "mini_robot"
```

这种路径特判。

---

# 15. MeArm → mini_robot → MeArm

这是本任务的核心验收。

依次执行：

### Test A

```text
选择 mearm-v1
↓
update
↓
build
↓
test
↓
run
```

记录：

```text
MeArm Golden Baseline
```

---

### Test B

```text
选择 mini_robot
↓
update
↓
build
↓
test
↓
run
```

确认：

```text
working-robot = mini_robot
```

并且：

```text
MeArm 不被修改
```

---

### Test C

再次：

```text
选择 mearm-v1
↓
update
↓
build
↓
test
↓
run
```

确认：

```text
working-robot = mearm-v1
```

并且：

```text
MeArm 行为与 Golden Baseline 完全一致
```

---

# 16. 状态隔离测试

特别测试以下状态是否泄漏：

```text
FK
IK
Joint State
Workspace
MuJoCo
Frontend
Backend
Capability
Robot Manifest
```

例如：

```text
mini_robot
J1 = 30°
```

切换：

```text
MeArm
```

后：

```text
MeArm 不允许出现 mini_robot 的 Joint State。
```

反向同样成立。

---

# 17. Frontend 验证

Frontend 不允许写：

```text
if robot === "mearm-v1"
```

或者：

```text
if robot === "mini_robot"
```

Frontend 应该读取：

```text
Robot Manifest
+
Capability
+
Joint Metadata
```

然后动态生成：

```text
Robot Information
Joint Controls
Joint Limits
Gripper
3D Model
```

例如：

```text
mini_robot

DOF: 3 + gripper

base_yaw
shoulder
elbow
gripper
```

而不是写死 MeArm 的：

```text
S9
S7
S8
S6
```

---

# 18. Backend 验证

Backend 必须能够：

```text
load working-robot
        ↓
read manifest
        ↓
load robot implementation
        ↓
start runtime
```

Backend 不应该知道：

```text
mini_robot 的具体 FK 公式
```

也不应该知道：

```text
MeArm 的四连杆结构
```

Backend 只调用：

```text
Core Robot Interface
```

---

# 19. Package Completeness Check

Core 必须能够检查：

```text
manifest.yaml
model
physics
kinematics
capability
tests
```

是否完整。

故意制造一个错误 Package：

```text
mini_robot_invalid
```

例如删除：

```text
physics/mini_robot.xml
```

执行：

```text
update
```

必须：

```text
FAIL
```

并明确指出：

```text
Missing required asset:
physics/mini_robot.xml
```

同时：

> 不得破坏当前有效的 `working-robot`。

---

# 20. Build Failure Safety

测试：

```text
mini_robot
```

故意制造一个编译错误。

执行：

```text
update
```

要求：

```text
update FAIL
```

但是之前有效的：

```text
working-robot
```

必须继续可运行。

禁止出现：

```text
working-robot 被半成品覆盖
```

推荐：

```text
working-robot.tmp
        ↓
build
        ↓
test
        ↓
validation
        ↓
PASS
        ↓
atomic replace
        ↓
working-robot
```

---

# 21. Version / Hash 更新测试

修改：

```text
mini_robot/manifest.yaml
```

版本：

```text
1.0.0
→
1.0.1
```

执行：

```text
update
```

必须检测到：

```text
package changed
```

重新部署。

然后恢复：

```text
1.0.0
```

再次执行：

```text
update
```

验证更新机制正常。

如果当前框架采用 content hash：

```text
package hash
```

则优先使用现有机制。

不要重复设计第二套更新机制。

---

# 22. Core 与 Robot Package 边界测试

检查：

```text
robot-package/mini_robot
```

是否包含不应该包含的共享代码。

禁止复制：

```text
TestRunner
RobotLoader
Runtime
MuJoCo Runtime
Build System
Frontend Framework
Generic Kinematics Interface
```

这些必须由：

```text
core/
```

提供。

mini_robot 只提供：

```text
Robot-specific implementation/data
```

---

# 23. 反向依赖检查

必须保证：

```text
Core
  ↓
Robot Package Interface
  ↓
Robot Package
```

而不是：

```text
Core
  ↓
MeArm implementation
  ↓
mini_robot
```

特别检查：

```text
core/** → robot-package/mearm-v1/**
```

必须为：

```text
0 dependency
```

同样：

```text
core/** → robot-package/mini_robot/**
```

必须为：

```text
0 dependency
```

Core 不得 import 任意具体 Robot Package。

---

# 24. mini_robot 的设计目的

不要追求：

```text
复杂模型
高精度动力学
真实硬件
复杂视觉
AI
```

mini_robot 的价值是验证：

```text
Robot Package
        +
Core
        +
Working Robot
        +
Update
        +
Validation
```

是否真正解耦。

因此：

> 简单是优点，而不是缺点。

---

# 25. 验收标准

必须全部满足。

## A. Package

```text
[ ] mini_robot 独立存在
[ ] manifest 正确
[ ] model 独立
[ ] physics 独立
[ ] FK 独立
[ ] IK 独立
[ ] capability 独立
[ ] tests 独立
```

## B. Core

```text
[ ] 不增加 mini_robot 特判
[ ] 不增加 MeArm 特判
[ ] 共享 Test Runner
[ ] 共享 Runtime
[ ] 共享 Loader
[ ] 共享 Build/Update Mechanism
```

## C. Working Robot

```text
[ ] 只能存在一个当前 Robot
[ ] 可以部署 mini_robot
[ ] backend 使用稳定路径
[ ] frontend build 正常
[ ] generated 正常
```

## D. Kinematics

```text
[ ] FK PASS
[ ] IK PASS
[ ] FK(IK(X)) PASS
[ ] Joint limits PASS
[ ] Workspace PASS
```

## E. Simulation

```text
[ ] MuJoCo 加载成功
[ ] Joint State 正确
[ ] Tool Position 正确
[ ] FK ↔ MuJoCo PASS
[ ] 3D ↔ FK PASS
```

## F. Robot Switching

```text
[ ] MeArm → mini_robot PASS
[ ] mini_robot → MeArm PASS
[ ] 无状态泄漏
[ ] MeArm Golden Baseline 未改变
```

## G. Failure Safety

```text
[ ] Package 缺文件时 update FAIL
[ ] Build FAIL 不破坏旧 Working Robot
[ ] Test FAIL 不破坏旧 Working Robot
[ ] Validation FAIL 不破坏旧 Working Robot
```

---

# 26. 最重要的最终验收

完成后必须证明：

```text
                 ┌──────────────┐
                 │     Core     │
                 │              │
                 │ Loader       │
                 │ Runtime      │
                 │ Validation   │
                 │ Build        │
                 │ Kinematics   │
                 │ Simulation   │
                 └──────┬───────┘
                        │
              ┌─────────┴─────────┐
              │                   │
       ┌──────▼──────┐     ┌──────▼──────┐
       │ MeArm-V1    │     │ mini_robot  │
       │ RobotPackage│     │ RobotPackage│
       └──────┬──────┘     └──────┬──────┘
              │                   │
              └─────────┬─────────┘
                        ↓
                ┌───────────────┐
                │ Working Robot │
                │               │
                │  one active   │
                │    robot      │
                └───────────────┘
```

最终必须达到：

> **增加 mini_robot 时，不修改 MeArm Robot Package。**

并且：

> **Core 不知道 mini_robot 的具体实现。**

同时：

> **删除 mini_robot 后，MeArm 仍然可以完整运行。**

---

# 27. 最终报告

任务完成后输出：

## 1. 架构变化

说明：

```text
Core
Robot Package
Working Robot
```

分别负责什么。

## 2. mini_robot 文件清单

列出：

```text
manifest
model
physics
kinematics
control
capability
tests
```

## 3. 修改文件

分别列出：

```text
新增
修改
删除
```

特别列出：

```text
MeArm 文件是否修改
Core 文件是否修改
```

## 4. 测试结果

输出：

```text
Package Validation
FK
IK
Joint
Workspace
3D
MuJoCo
Sim2Sim
Frontend
Backend
Update
Robot Switching
Failure Safety
```

## 5. Golden Baseline

明确说明：

```text
MeArm Golden Baseline:
PASS / FAIL
```

并给出与改造前的差异。

## 6. 架构问题

主动指出：

```text
哪些地方仍然存在机器人耦合
哪些地方仍然存在 MeArm 特判
哪些接口还不够通用
哪些地方未来增加 SO-ARM101 仍可能遇到问题
```

不要为了让任务显示 PASS 而隐藏问题。

---

# 28. 执行要求

严格按照以下顺序执行：

```text
Phase 1
只读分析当前 Core / Robot Package / Working Robot 架构
        ↓
Phase 2
确认 Robot Package Specification
        ↓
Phase 3
设计 mini_robot
        ↓
Phase 4
实现 mini_robot Package
        ↓
Phase 5
Package Validation
        ↓
Phase 6
部署 Working Robot
        ↓
Phase 7
Frontend / Backend / 3D
        ↓
Phase 8
FK / IK / MuJoCo / Sim2Sim
        ↓
Phase 9
MeArm → mini_robot → MeArm
        ↓
Phase 10
故障安全测试
        ↓
Phase 11
完整回归测试
        ↓
Phase 12
输出架构审查报告
```

## 禁止事项

禁止：

```text
1. 修改 MeArm Golden Baseline 逻辑来适配 mini_robot
2. 在 Core 中增加 mini_robot 特判
3. 在 Core 中增加 MeArm 特判
4. 复制 Core 到 mini_robot
5. 复制 MeArm FK/IK 作为 mini_robot FK/IK
6. 创建第二套 Test Runner
7. 创建第二套 Manifest 机制
8. 让 Backend 直接引用 robot-package/<robot>
9. 让 Working Robot 同时保存多个机器人
10. 为了测试通过而降低原有 MeArm 验收标准
```

如果发现当前架构无法支持 mini_robot：

> **优先指出架构缺陷并最小化修复 Core 的通用机制，而不是在 mini_robot 中加入临时特判。**
