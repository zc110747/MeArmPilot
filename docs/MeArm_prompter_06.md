# MeArmPilot Robot Package + Working Robot 架构级重构

## 0. 任务性质

这是一次 **MeArmPilot 架构级重构**，不是普通功能开发。

当前 MeArmPilot 已经完成并冻结了 MeArm-V1 的 Golden Baseline：

```text
MeArm-V1
├── 3D
├── FK
├── IK
├── Joint
├── Workspace
├── MJCF / MuJoCo
├── Sim2Sim
├── Backend
├── Frontend
└── Robot-specific Tests
```

当前直接让 Agent 增加 SO-ARM101 时，发现现有实现存在明显机器人耦合：

* MeArm FK/IK 与工程结构耦合
* MeArm 3D 与运行逻辑耦合
* MuJoCo 模型与 MeArm 逻辑耦合
* Backend 与具体机器人路径/实现存在耦合
* Robot-specific tests 与通用测试混合
* 新机器人接入容易导致 Agent 修改 MeArm-V1
* Agent 无法稳定在不破坏 MeArm 的情况下生成 SO-ARM101

因此本任务的核心目标不是“把 SO-ARM101 加进去”。

而是：

> **建立真正的 Robot Package + Working Robot 架构，使机器人特有实现与 MeArmPilot Core 解耦，同时保留所有真正共享的框架代码。**

最终达到：

```text
                         MeArmPilot
                            │
              ┌─────────────┴─────────────┐
              │                           │
          Shared Core               Robot Packages
              │                           │
      ┌───────┼────────┐          ┌───────┴───────┐
      │       │        │          ↓               ↓
    Loader  Runtime  Validator  MeArm-V1       SO-ARM101
      │       │        │          │               │
      └───────┴────────┘          │               │
              │                  FK/IK           FK/IK
              │                  MJCF            MJCF
              │                  3D              3D
              │                  Tests           Tests
              │                    │               │
              └────────────┬───────┴───────────────┘
                           ↓
                     Working Robot
                           ↓
                    Build / Test / Run
```

---

# 1. 第一原则：MeArm-V1 Golden Baseline 冻结

这是整个任务最高优先级。

MeArm-V1 当前已经验证完成，必须视为：

> **不可破坏的 Golden Baseline。**

不得为了实现新架构而重新设计：

* MeArm FK
* MeArm IK
* MeArm Joint
* MeArm Workspace
* MeArm 3D
* MeArm MJCF
* MeArm MuJoCo
* MeArm Servo Mapping
* MeArm Calibration
* MeArm Control
* MeArm Protocol
* MeArm 测试数据
* MeArm 数学模型

禁止：

```text
为了统一 API
→ 修改 MeArm FK

为了支持 SO-ARM101
→ 修改 MeArm IK

为了适应 Package
→ 修改 MeArm MJCF

为了抽象
→ 重写 MeArm 算法
```

如果现有 MeArm 实现无法直接满足新接口：

> 优先增加 Adapter / Wrapper / Loader，而不是修改 MeArm 算法。

---

# 2. 第二原则：不要走“每个机器人一套代码”的极端

Robot Package 不等于：

```text
每个机器人复制一套 ArmPilot。
```

这是本任务必须避免的错误。

正确原则：

> **共享机制放 Core，机器人差异放 Robot Package。**

例如：

### Core 共享

```text
RobotLoader
RobotRegistry
RobotRuntime
KinematicsEngine
NumericalIK
MuJoCoRuntime
Sim2Sim
PackageValidator
TestRunner
BuildSystem
Vision API
Common Protocol Infrastructure
```

### Robot Package 专属

```text
MeArm FK
MeArm IK
MeArm MJCF
MeArm Mesh
MeArm Servo Mapping
MeArm Calibration
MeArm Capability
MeArm Test Cases

SO101 FK
SO101 MJCF
SO101 Mesh
SO101 Calibration
SO101 Test Cases
```

---

# 3. 文件归属判断规则

遇到任何文件时必须先判断：

## 3.1 换机器人后仍然不需要修改

放：

```text
core/
```

例如：

```text
RobotLoader
RobotRegistry
RobotRuntime
PackageValidator
TestRunner
MuJoCoRuntime
Sim2Sim
BuildSystem
WebSocket infrastructure
common API
```

---

## 3.2 换机器人后必须修改

放：

```text
robot-package/<robot>/
```

例如：

```text
FK
IK
MJCF
3D model
mesh
joint definition
joint limits
workspace
servo mapping
calibration
robot capability
robot-specific tests
```

---

## 3.3 当前 Robot 的构建/运行产物

放：

```text
working-robot/
```

例如：

```text
当前 package
dist
generated files
当前 tests
runtime assets
build metadata
```

---

# 4. 目标目录结构

最终目标：

```text
MeArmPilot/
│
├── core/
│   │
│   ├── robot/
│   │   ├── Robot.ts
│   │   ├── RobotRegistry.ts
│   │   ├── RobotLoader.ts
│   │   ├── RobotRuntime.ts
│   │   └── RobotCapability.ts
│   │
│   ├── kinematics/
│   │   ├── KinematicsEngine.ts
│   │   ├── IKSolver.ts
│   │   ├── NumericalIK.ts
│   │   ├── Jacobian.ts
│   │   └── ...
│   │
│   ├── simulation/
│   │   ├── MujocoRuntime.*
│   │   ├── Sim2Sim.*
│   │   └── ...
│   │
│   ├── validation/
│   │   ├── PackageValidator.*
│   │   ├── TestRunner.*
│   │   ├── CaseLoader.*
│   │   └── Report.*
│   │
│   ├── build/
│   │   ├── RobotBuilder.*
│   │   └── ...
│   │
│   ├── runtime/
│   │   └── ...
│   │
│   └── vision/
│       ├── Camera.*
│       ├── Observation.*
│       └── ...
│
├── robot-package/
│   │
│   ├── mearm-v1/
│   │   ├── manifest.yaml
│   │   ├── model/
│   │   ├── physics/
│   │   ├── kinematics/
│   │   ├── control/
│   │   ├── capability/
│   │   ├── calibration/
│   │   └── tests/
│   │
│   └── so-arm101/
│       ├── manifest.yaml
│       ├── model/
│       ├── physics/
│       ├── kinematics/
│       ├── control/
│       ├── capability/
│       ├── calibration/
│       └── tests/
│
├── working-robot/
│   ├── manifest.yaml
│   ├── package/
│   ├── generated/
│   ├── dist/
│   ├── backend/
│   ├── assets/
│   ├── tests/
│   └── build/
│
├── frontend/
├── backend/
├── tests/
└── update.bat
```

注意：

上述目录只是目标架构。

**必须先分析当前仓库，不允许机械地移动目录。**

如果当前工程有更合理的目录名称或模块边界，可以采用等价结构，但必须保持以下逻辑：

```text
Core
Robot Package
Working Robot
```

三个概念必须明确存在。

---

# 5. Robot Package 定义

Robot Package 表示：

> **一个机器人自身完整的、可独立验证的定义与实现。**

包含：

```text
Identity
Model
Physics
Kinematics
Control
Capability
Calibration
Validation
```

例如：

```text
robot-package/mearm-v1/
```

必须包含 MeArm 自身：

```text
3D
MJCF
FK
IK
Joint
Workspace
Control
Calibration
Tests
```

---

# 6. Robot Package Manifest

每个 Robot Package 必须有：

```text
manifest.yaml
```

至少描述：

```yaml
id: mearm-v1
name: MeArm V1
version: 1.0.0

package:
  format: 1

robot:
  type: robotic_arm
  dof: 4
  actuator_count: 4

model:
  scene: physics/scene.xml
  mjcf: physics/mearm.xml

kinematics:
  fk:
    type: custom
    entry: kinematics/fk.*
  ik:
    type: custom
    entry: kinematics/ik.*

capabilities:
  position: true
  orientation: false
  gripper: true
  simulation: true
  hardware: true
```

实际字段必须根据当前工程情况设计。

不要为了示例字段而破坏已有实现。

---

# 7. FK / IK 架构

这是本次重构最重要的部分之一。

必须做到：

```text
Core 提供：
    Kinematics Interface / Engine

Robot Package 提供：
    Robot-specific FK / IK
```

即：

```text
core/kinematics/
        │
        ↓
KinematicsEngine
        │
        ├──────────────┐
        ↓              ↓
MeArm FK/IK        SO101 FK/IK
Package            Package
```

不要实现成：

```text
core/
└── mearm-fk.ts
```

然后增加：

```text
so101-fk.ts
```

这会重新把机器人实现塞回 Core。

---

# 8. 不强行统一 IK 算法

不同机械臂可能采用：

```text
Analytical IK
Numerical IK
Damped Least Squares
Optimization
External Solver
```

因此 Package 必须能够声明自己的实现。

例如：

```yaml
kinematics:
  fk:
    type: custom
    entry: kinematics/fk.ts

  ik:
    type: custom
    entry: kinematics/ik.ts
```

或者：

```yaml
kinematics:
  ik:
    type: numerical
```

如果机器人当前没有可靠 IK：

```yaml
capabilities:
  ik: false
```

不要伪造 IK。

---

# 9. Robot-specific Tests

这是本次架构中必须特别处理的一部分。

当前机器人测试：

```text
fk-cases.json
ik-cases.json
joint-cases.json
workspace-cases.json
```

属于 MeArm-V1。

因此必须迁移到：

```text
robot-package/mearm-v1/tests/
```

而不是：

```text
tests/mearm/
```

或者：

```text
core/tests/mearm/
```

---

# 10. 测试执行机制仍然必须共享

不能因为 tests 属于 Robot Package，就给每个 Robot Package 复制 TestRunner。

错误：

```text
robot-package/mearm-v1/tests/TestRunner.*
robot-package/so-arm101/tests/TestRunner.*
```

正确：

```text
core/validation/TestRunner.*
```

然后：

```text
Core TestRunner
       ↓
读取当前 Robot Package
       ↓
读取 robot-package/<robot>/tests
       ↓
执行 Robot-specific tests
```

即：

> **测试数据和测试内容属于 Robot；测试执行框架属于 Core。**

---

# 11. Working Robot

建立：

```text
working-robot/
```

定义为：

> **当前 Robot Package 的运行时工作实例。**

不是新的 Robot Package。

不是第二套源码。

不是复制 Core。

---

# 12. Working Robot 目录

推荐：

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

其中：

### package

当前 Robot Package 的工作副本。

### generated

构建或适配过程中产生的机器人特定生成文件。

### dist

当前机器人前端最终构建产物。

### backend

当前 Robot Runtime 需要的 Python 文件/模块。

### assets

当前机器人运行所需资源。

### tests

当前 Robot Package 的测试副本。

### build

构建中间产物。

---

# 13. Working Robot 不允许复制 Core

禁止：

```text
working-robot/core/
working-robot/runtime/
working-robot/validation/
```

然后复制大量共享代码。

Core 永远位于：

```text
core/
```

Working Robot 只是：

```text
Package + Generated + Build + Runtime Artifacts
```

---

# 14. Working Robot 当前只允许存在一个 Robot

不要：

```text
working-robot/
├── mearm-v1/
└── so-arm101/
```

当前设计：

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

它代表：

> 当前正在工作的机器人。

例如：

```yaml
id: mearm-v1
```

切换 SO-ARM101 后：

```yaml
id: so-arm101
```

Working Robot 整体更新。

---

# 15. Robot Package → Working Robot

安装流程：

```text
robot-package/<robot>
        ↓
Package Validation
        ↓
Copy
        ↓
working-robot/package
        ↓
Build
        ↓
working-robot/dist
        ↓
Generate runtime files
        ↓
working-robot/backend
        ↓
Copy Robot Tests
        ↓
working-robot/tests
        ↓
Validate
```

---

# 16. Backend 路径必须稳定

Backend Runtime 不允许直接依赖：

```text
robot-package/<robot>
```

例如禁止：

```text
backend
  ↓
../../robot-package/so-arm101/backend/main.py
```

必须：

```text
Backend
   ↓
working-robot/backend/
```

例如：

```text
working-robot/backend/main.py
```

内部使用相对路径定位：

```python
BASE_DIR = Path(__file__).resolve().parent.parent
```

具体实现根据当前工程语言和目录决定。

核心要求：

> **Backend 只认 Working Robot。**

这样切换 Robot 后，Backend 的运行路径不发生变化。

---

# 17. Frontend Build

Robot Package 如果包含机器人特定前端代码：

```text
robot-package/<robot>/frontend/
```

构建：

```text
Robot Package Frontend
        ↓
Build
        ↓
working-robot/dist/
```

Frontend Runtime 不应该直接使用：

```text
robot-package/<robot>/dist
```

最终运行服务统一从：

```text
working-robot/dist
```

提供。

---

# 18. Working Robot 更新检测

任务启动时必须自动检查。

至少检查：

```text
1. working-robot 是否存在
2. manifest 是否存在
3. Robot ID 是否一致
4. Package version 是否一致
5. Package content hash 是否一致
6. package 是否完整
7. dist 是否存在
8. backend 是否存在
9. tests 是否存在
10. build metadata 是否有效
```

---

# 19. Hash 机制

不要只比较 version。

建议：

```yaml
source:
  id: mearm-v1
  version: 1.0.0
  hash: xxx
```

Working Robot：

```yaml
installed:
  id: mearm-v1
  version: 1.0.0
  hash: xxx
```

比较：

```text
Robot ID
+
Version
+
Content Hash
```

只要任意一项不同：

```text
Working Robot = stale
```

需要重新构建。

Hash 的具体算法可以使用项目已有机制；如果没有，选择稳定、简单、跨平台的内容 hash。

必须忽略不应参与 hash 的构建目录，例如：

```text
dist
build
generated
```

具体规则需要结合实际工程定义。

---

# 20. update.bat

项目根目录增加：

```text
update.bat
```

负责：

```text
检查当前 Robot
        ↓
检查 Robot Package
        ↓
检查 Working Robot
        ↓
比较 version/hash
        ↓
需要更新？
   ┌────┴────┐
   No        Yes
   │          │
   ↓          ↓
返回        Update
              ↓
           Build
              ↓
            Test
              ↓
           Validate
              ↓
            Install
```

重复执行：

```text
update.bat
```

如果没有变化：

```text
No update required
```

不得无意义重新构建。

---

# 21. 更新必须支持原子替换

禁止：

```text
删除 working-robot
        ↓
重新 Build
        ↓
Build 失败
```

导致整个运行环境损坏。

应该：

```text
working-robot/
       │
       │ 当前有效
       ↓
temporary-working-robot/
       │
       ↓
Copy
Build
Test
Validate
       │
       ↓
Success
       │
       ↓
替换 Working Robot
```

如果失败：

```text
temporary-working-robot
```

直接清理。

原有 Working Robot 保持不变。

---

# 22. Task Startup

任务启动时：

```text
Start
 ↓
Load current Robot configuration
 ↓
Load Robot Package
 ↓
Check Working Robot
```

如果一致：

```text
Start Runtime
```

如果不一致：

```text
Update Working Robot
 ↓
Build
 ↓
Robot Tests
 ↓
Validation
 ↓
Start Runtime
```

如果任何一步失败：

```text
禁止启动任务
```

并输出明确错误。

---

# 23. Robot Registry

Core 提供：

```text
RobotRegistry
```

用于：

```text
list
load
validate
select
```

例如：

```text
mearm-v1
so-arm101
```

但 Registry 不包含：

```text
MeArm FK
SO101 FK
```

Registry 只知道：

```text
Robot ID
Package path
Manifest
Capability
```

---

# 24. Robot Loader

Core 提供：

```text
RobotLoader
```

输入：

```text
robot ID
```

加载：

```text
robot-package/<robot>/manifest.yaml
```

解析：

```text
Model
Kinematics
Physics
Control
Capability
Tests
```

---

# 25. Robot Runtime

Core 提供统一 Runtime：

```text
RobotRuntime
```

负责：

```text
load
initialize
build
validate
start
stop
state
command
```

但不包含：

```text
MeArm-specific code
SO101-specific code
```

---

# 26. Robot Capability

Capability 属于 Robot Package。

例如：

```yaml
capabilities:
  position: true
  orientation: false
  gripper: true
  ik: true
  simulation: true
  hardware: true
```

不要假设所有机器人：

```text
6DOF
XYZ + Orientation
```

MeArm：

```text
4DOF
```

SO-ARM101：

```text
5DOF arm + gripper actuator/joint
```

具体能力必须由各自 Package 声明。

---

# 27. MeArm-V1 迁移策略

这是第一阶段。

**暂时不要实现 SO-ARM101。**

先：

```text
MeArm-V1
 ↓
Golden Baseline
 ↓
robot-package/mearm-v1
 ↓
Working Robot
 ↓
Build
 ↓
Test
 ↓
Sim2Sim
```

---

# 28. MeArm 迁移时必须先建立文件映射表

不要直接移动。

先分析：

```text
当前文件
 ↓
Robot-specific?
 ↓
Core shared?
 ↓
Generated?
 ↓
Test?
```

输出：

```text
File
Old Location
New Location
Category
Reason
```

例如：

```text
fk.ts
→ robot-package/mearm-v1/kinematics/fk.ts
→ Robot-specific

TestRunner.ts
→ core/validation/TestRunner.ts
→ Shared

fk-cases.json
→ robot-package/mearm-v1/tests/fk-cases.json
→ Robot-specific
```

只有分析完成后再开始迁移。

---

# 29. MeArm Tests 必须迁移

已有：

```text
fk-cases.json
ik-cases.json
joint-cases.json
workspace-cases.json
```

全部归入：

```text
robot-package/mearm-v1/tests/
```

迁移后：

```text
Core TestRunner
        ↓
MeArm Package
        ↓
tests/*
```

测试结果必须保持一致。

---

# 30. MeArm Golden Regression

迁移前记录 Baseline。

至少记录：

```text
TypeScript
Vitest
Go
go vet
Browser E2E
MuJoCo pytest
FK random tests
FK ↔ Three.js
FK ↔ IK
FK ↔ MuJoCo
Sim2Sim
```

当前已经存在的测试数量和结果必须作为迁移前基线。

迁移后重新运行。

要求：

```text
Before == After
```

不是：

```text
大概能跑
```

---

# 31. 不允许为了新架构降低测试标准

例如：

```text
原来 318/318
```

不能因为迁移困难改成：

```text
300/300
```

然后认为通过。

不能删除失败 case 来让测试通过。

不能修改测试 expected value 来适应错误实现。

---

# 32. MeArm Runtime 验证

必须证明：

```text
robot: mearm-v1
```

能够：

```text
Package Load
 ↓
Working Robot Install
 ↓
Build
 ↓
Robot Tests
 ↓
3D
 ↓
FK
 ↓
IK
 ↓
MuJoCo
 ↓
Sim2Sim
 ↓
Backend
 ↓
Frontend
```

完整工作。

---

# 33. SO-ARM101 第二阶段

只有 MeArm 完整跑通后才开始。

目标：

```text
Agent
 ↓
生成 SO-ARM101 Robot Package
 ↓
robot-package/so-arm101/
 ↓
Package Validator
 ↓
Working Robot
 ↓
Build
 ↓
Tests
 ↓
3D
 ↓
FK
 ↓
MuJoCo
 ↓
Sim2Sim
```

---

# 34. SO-ARM101 不允许修改 MeArm

Agent 生成 SO-ARM101 时：

允许：

```text
robot-package/so-arm101/**
```

允许修改：

```text
core/
```

但仅限真正的通用框架问题。

禁止：

```text
robot-package/mearm-v1/**
```

任何修改。

如果 Agent 认为必须修改 MeArm：

> 停止并报告原因，不允许自动修改。

---

# 35. SO-ARM101 测试独立

SO-ARM101 建立：

```text
robot-package/so-arm101/tests/
```

只放 SO-ARM101 自身验证数据。

不得复制：

```text
MeArm fk-cases
MeArm ik-cases
MeArm workspace cases
```

来伪造测试。

如果 SO-ARM101 某项功能尚未实现：

```yaml
capabilities:
  ik: false
```

然后明确标记：

```text
IK not implemented
```

而不是写一个未经验证的 IK。

---

# 36. SO-ARM101 官方模型

SO-ARM101 优先使用官方：

```text
URDF
MJCF
Mesh
Calibration
```

不要根据图片重新猜机械结构。

如果官方模型与 MeArmPilot Package 存在接口差异：

```text
Official Model
       ↓
Adapter
       ↓
Robot Package
```

而不是修改 MeArm。

---

# 37. Multi-Robot Switching

完成两个 Package 后必须验证：

```text
MeArm
 ↓
SO-ARM101
 ↓
MeArm
 ↓
SO-ARM101
```

只改变：

```yaml
robot:
  model: xxx
```

然后：

```text
Working Robot Update
 ↓
Build
 ↓
Test
 ↓
Runtime
```

不能：

```text
修改 FK
修改 IK
修改 3D
修改 Backend
修改 MuJoCo
```

---

# 38. 防止 Robot-specific Branch 污染 Core

检查整个 Core：

禁止出现大量：

```text
if robot == "mearm-v1"
```

或者：

```text
if robot == "so-arm101"
```

特别是禁止：

```text
Core FK:
    if MeArm → ...
    if SO101 → ...
```

正确：

```text
Core
 ↓
Robot Package
 ↓
Robot-specific implementation
```

---

# 39. Adapter 是允许的

如果 Robot Package 与 Core 接口之间存在差异：

```text
Core Interface
       ↑
     Adapter
       ↑
Robot implementation
```

这是允许的。

Adapter 的作用：

> **隔离差异。**

而不是：

> **把 Robot-specific logic 重新塞进 Core。**

---

# 40. 未来 Object / Scene / Vision 必须预留边界

本次不要大规模实现 Object Package。

但是 Core 架构不能阻止未来增加：

```text
Object Package
Scene Package
Camera
Observation
Vision Agent
```

未来：

```text
Robot Package
+
Object Package
+
Scene
+
Camera
```

形成：

```text
Observation
```

再交给 Agent。

当前不要因为这个未来需求过度设计。

---

# 41. 未来 Agent 的正确工作方式

最终 Agent 不应该：

```text
修改 MeArmPilot
 ↓
加入某机械臂
```

而应该：

```text
输入：
Robot Description / CAD / URDF / MJCF
        ↓
Agent / Robot Compiler
        ↓
生成：
robot-package/<robot>
        ↓
Validator
        ↓
Working Robot
        ↓
Build
        ↓
Test
        ↓
Runtime
```

这样 Robot Package 才真正成为：

> **Agent 生成机器人的标准输出格式。**

---

# 42. 开发执行规则

本次必须严格分阶段。

禁止一次性大规模修改整个仓库。

---

## Phase 0：只读分析

不修改代码。

输出：

```text
1. 当前目录结构
2. MeArm-specific 文件
3. Shared Core 文件
4. Generated 文件
5. Robot-specific tests
6. Shared tests
7. 当前 Build 流程
8. 当前 Frontend 流程
9. 当前 Backend 流程
10. 当前 MuJoCo 流程
11. 当前 FK/IK 调用链
12. 当前 3D 调用链
13. 当前配置加载流程
14. 所有 MeArm 耦合点
```

重点回答：

> **如果现在直接复制 MeArm 到 Robot Package，哪些东西不应该复制？**

---

# 43. Phase 1：建立 Core 边界

只建立最小通用接口：

```text
Robot
RobotRegistry
RobotLoader
RobotRuntime
KinematicsEngine
PackageValidator
TestRunner
```

不要重写 MeArm。

---

# 44. Phase 2：建立 MeArm Robot Package

迁移：

```text
3D
MJCF
FK
IK
Joint
Workspace
Control
Calibration
Tests
Assets
```

到：

```text
robot-package/mearm-v1/
```

保持行为不变。

---

# 45. Phase 3：建立 Working Robot

实现：

```text
Install
Update
Validate
Build
Test
Runtime
```

建立：

```text
working-robot/
```

---

# 46. Phase 4：MeArm 完整回归

运行完整测试。

必须证明：

```text
旧 MeArm
      =
新 MeArm Robot Package
      =
Working Robot MeArm
```

在行为层面一致。

---

# 47. Phase 5：update.bat

实现：

```text
update.bat
```

测试：

### Case A：首次启动

```text
working-robot 不存在
```

自动创建。

### Case B：没有变化

```text
Package hash unchanged
```

不重新 Build。

### Case C：Package 修改

```text
hash changed
```

自动 Build。

### Case D：Robot 切换

```text
MeArm → SO101
```

更新 Working Robot。

### Case E：Build 失败

旧 Working Robot 保持可用。

### Case F：Tests 失败

禁止安装新的 Working Robot。

---

# 48. Phase 6：SO-ARM101

此阶段才开始。

使用：

```text
robot-package/so-arm101/
```

完成：

```text
Model
3D
Joint
FK
MJCF
MuJoCo
Tests
Capability
```

根据实际情况决定 IK。

---

# 49. Phase 7：多机器人回归

执行：

```text
MeArm
SO101
MeArm
SO101
```

验证：

```text
Package isolation
Working Robot replacement
Frontend replacement
Backend path
3D
FK
MuJoCo
Tests
```

---

# 50. 必须测试“污染问题”

这是本次重构的重要验收。

测试：

```text
加载 MeArm
 ↓
运行
 ↓
切换 SO101
 ↓
运行
 ↓
切换 MeArm
 ↓
运行
```

检查：

```text
MeArm FK 是否改变
MeArm IK 是否改变
MeArm 3D 是否改变
MeArm MJCF 是否改变
MeArm Tests 是否改变
```

要求：

```text
No Cross Robot State Leakage
```

---

# 51. 构建验证

必须至少执行项目实际使用的：

```text
Frontend build
Backend build/check
TypeScript
Vitest
Go test
go vet
Python tests
MuJoCo tests
Browser E2E
```

具体命令以仓库当前实际配置为准。

不要凭空创造不存在的命令。

---

# 52. 最终验收标准

必须全部满足。

### Architecture

```text
Core
Robot Package
Working Robot
```

边界明确。

### MeArm

```text
MeArm Golden Baseline
```

不被破坏。

### Shared Code

不会因为增加第二机器人而复制。

### Robot Code

MeArm / SO101-specific implementation 位于各自 Package。

### Tests

Robot-specific tests 跟随 Robot Package。

### TestRunner

测试执行机制共享。

### Working Robot

只代表当前机器人。

### Backend

只依赖 Working Robot Runtime 路径。

### Frontend

运行 `working-robot/dist`。

### Update

能够自动检测：

```text
Robot ID
Version
Hash
```

### Build

更新后自动重新构建。

### Failure Safety

Build/Test 失败不会破坏当前有效 Working Robot。

### Multi-Robot

MeArm ↔ SO101 可以反复切换。

### Isolation

SO101 不修改 MeArm。

### Core

Core 不包含 MeArm/SO101 特殊实现。

---

# 53. 最终架构目标

完成后，必须能够清晰解释：

```text
                   MeArmPilot Core
                         │
       ┌─────────────────┼─────────────────┐
       ↓                 ↓                 ↓
    Registry           Runtime          Validator
       │                 │                 │
       └─────────────────┼─────────────────┘
                         ↓
                  Robot Package
                  ↙           ↘
             MeArm-V1       SO-ARM101
                │               │
          FK / IK / 3D      FK / 3D / MJCF
          MJCF / Tests      Tests / Capability
                │               │
                └───────┬───────┘
                        ↓
                 Working Robot
                        │
             ┌──────────┼──────────┐
             ↓          ↓          ↓
            dist      backend     tests
             │          │          │
             └──────────┼──────────┘
                        ↓
                     Runtime
```

---

# 54. 最重要的架构判断

整个任务始终遵循：

```text
共享“机制”
        ↓
Core

机器人“差异”
        ↓
Robot Package

当前“运行实例”
        ↓
Working Robot
```

因此：

```text
Core ≠ MeArm
Core ≠ SO101

Robot Package ≠ 一套完整 MeArmPilot

Working Robot ≠ 第二套源码
```

最终：

```text
1 个 MeArmPilot Core
+
N 个 Robot Package
+
1 个 Working Robot
```

而不是：

```text
N 个机器人
+
N 套 Core
```

---

# 55. 本任务的最终目标

最终希望达到：

```text
                    Robot Description
                           │
                           ↓
                  Robot Package
                           │
                    Package Validator
                           │
                           ↓
                    Working Robot
                           │
                 ┌─────────┴─────────┐
                 ↓                   ↓
              Frontend             Backend
                 │                   │
                 └─────────┬─────────┘
                           ↓
                    MeArmPilot Runtime
                           │
                           ↓
                    Simulation / Robot
```

未来进一步：

```text
STEP / URDF / MJCF / CAD
          ↓
    Robot Package Builder
          ↓
    robot-package/<robot>
          ↓
      Working Robot
          ↓
       MeArmPilot
```

最终使 Agent 的工作边界从：

> **“修改 MeArmPilot 源码以适配一个机器人”**

转变为：

> **“生成一个符合 Robot Package Specification 的机器人。”**

这才是本次重构真正需要建立的架构能力。

---

# 56. 执行纪律

每个 Phase 完成后必须：

```text
修改
 ↓
Build
 ↓
Test
 ↓
验证
 ↓
记录
```

并报告：

```text
Changed
Tests
Results
Remaining Issues
Next Step
```

如果某个阶段失败：

> 停止进入下一阶段。

不要通过降低测试标准、删除测试、修改 Golden Baseline 或修改 MeArm 算法来“让测试通过”。

如果发现架构设计存在冲突：

> 优先保护 MeArm-V1 Golden Baseline，并报告冲突点。

不要自行扩大任务范围。
