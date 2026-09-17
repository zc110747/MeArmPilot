# MeArmPilot：MeArm-V1 基线冻结 + MeArm-3D 最小架构抽象 + Sim2Sim 回归

## 0. 任务定位

你正在维护 MeArmPilot 项目。

当前项目已经完成并能够测试：

* MeArm 3D Web 模型
* FK
* IK
* Web 控制
* MuJoCo 仿真
* Sim2Sim / 仿真链路
* 现有 Robot Model / robot.yaml
* Joint limits
* Teaching / Playback 等已有能力

当前阶段**不是开发新功能**。

本任务只有四个目标：

```text
① 冻结当前 MeArm 模型
        ↓
② 建立 MeArm-V1 Baseline
        ↓
③ 在 MeArm-3D 内进行最小程度架构抽象
        ↓
④ 使用 Sim2Sim 完成全部回归验证
```

当前没有真实机械臂硬件，因此：

> 本阶段禁止把“真机测试”作为验收条件，所有验证必须基于现有 MeArm 3D + MuJoCo + Sim2Sim 完成。

---

# 1. 总原则

## 1.1 当前 MeArm 是唯一基准

当前已经能够运行的 MeArm 3D / FK / IK / MuJoCo 模型是项目的：

```text
MeArm-V1
```

它不是 demo。

它不是临时模型。

它不是未来被替换的旧代码。

它是：

> MeArmPilot 第一个正式 Robot Model，也是后续所有通用化开发的 Reference Robot / Golden Baseline。

---

## 1.2 严禁重新设计 MeArm

不要因为要进行通用化，就重新设计：

* MeArm 几何结构
* FK 数学模型
* IK 数学模型
* Joint 定义
* Joint limit
* MuJoCo 结构
* 四连杆关系
* Gripper 结构
* 现有坐标系
* 当前 robot.yaml 的实际含义

除非现有代码存在明确 Bug，并且能够通过现有测试证明。

原则：

```text
先保留
↓
再抽象
↓
最后扩展
```

而不是：

```text
重新设计
↓
套一个通用框架
↓
希望结果一样
```

---

# 2. 第一阶段：冻结当前 MeArm 模型

## 2.1 首先完整审查当前 MeArm-3D

不要立即修改代码。

先分析当前：

```text
MeArm-3D/
├── frontend
├── backend
├── robot.yaml
├── physics.yaml
├── FK
├── IK
├── MuJoCo
├── WebSocket
├── teaching
├── tests
└── assets
```

明确：

1. 当前 Robot Model 从哪里读取
2. 当前 FK 从哪里读取机器人参数
3. 当前 IK 从哪里读取机器人参数
4. Three.js 使用哪些机器人参数
5. MuJoCo 使用哪些机器人参数
6. MJCF 是如何生成的
7. Sim2Sim 当前测试入口在哪里
8. 当前测试覆盖哪些功能
9. 哪些代码是 MeArm 专属
10. 哪些代码已经具备通用性

先形成一份分析文档：

```text
docs/architecture/mearm-v1-baseline-analysis.md
```

---

# 3. MeArm-V1 正式冻结

创建明确的 Robot Model 标识：

```text
MeArm-V1
```

建议增加：

```yaml
robot:
  id: mearm
  model: MeArm-V1
  version: 1.0.0
```

具体字段必须根据现有 robot.yaml 结构适配。

不要为了版本号大规模修改 YAML。

---

# 4. 建立 MeArm-V1 Baseline

Baseline 必须描述：

## 4.1 Robot Model

记录：

* link 数量
* joint 数量
* joint name
* joint type
* joint axis
* joint limits
* servo mapping
* geometry parameters
* gripper 参数
* 四连杆关系

---

## 4.2 Kinematics Baseline

固定当前：

### FK

记录：

```text
joint state
    ↓
FK
    ↓
XYZ / pose
```

建立固定测试集合。

至少包括：

* 零位
* 各 joint limit
* 中间姿态
* 随机合法姿态
* 边界姿态

---

## 4.3 IK Baseline

当前 MeArm IK 不允许因为通用化而重写。

保留当前：

* reachable
* OUT_OF_WORKSPACE
* JOINT_LIMIT
* elbow-up
* elbow-down
* nearest solution

建立固定 IK 测试集合：

```text
XYZ target
    ↓
IK
    ↓
joint solution
    ↓
FK
    ↓
XYZ
    ↓
compare target
```

记录：

```text
position error
joint-limit status
success/failure
solution type
```

---

# 5. 建立 Golden Test Dataset

创建：

```text
tests/baseline/mearm-v1/
```

至少包含：

```text
fk_cases.json
ik_cases.json
joint_cases.json
workspace_cases.json
```

如果当前测试框架已有对应数据格式，优先复用。

不要为了 Baseline 重复建设测试框架。

---

# 6. Baseline 必须可重复

禁止使用：

```text
每次随机生成一组测试
```

作为唯一验收方式。

可以保留随机测试，但必须增加固定 seed：

```text
seed = fixed
```

目标：

> 任何开发者在任何时间重新运行 MeArm-V1 Baseline，都应该得到可比较的结果。

---

# 7. 记录当前真实结果

执行当前项目完整测试。

至少记录：

```text
TypeScript
Go
Vitest
E2E
MuJoCo
FK
IK
MJCF
Sim2Sim
```

不要自行修改测试阈值。

把当前通过结果记录到：

```text
docs/architecture/mearm-v1-baseline.md
```

格式类似：

```text
MeArm-V1 Baseline

Frontend:
PASS

Backend:
PASS

FK:
PASS

IK:
PASS

MJCF:
PASS

MuJoCo:
PASS

Sim2Sim:
PASS
```

具体数字必须来自实际运行结果，禁止编造。

---

# 8. 第二阶段：MeArm-3D 最小架构抽象

## 核心要求

这一步不是“通用机器人系统开发”。

只允许做：

> 把现有 MeArm 实现放到清晰的接口后面。

---

# 9. RobotDefinition

在不破坏当前 robot.yaml 的情况下，建立最小 Robot Definition 概念。

推荐结构：

```text
RobotDefinition
├── metadata
├── links
├── joints
├── actuators
├── kinematics
├── limits
└── simulation
```

但不要一次设计大量未来字段。

只抽取当前 MeArm 已经真实存在的信息。

---

# 10. KinematicsEngine

建立统一接口。

建议概念：

```ts
interface KinematicsEngine {
    forward(joints): Pose;
    inverse(target, options?): IKResult;
}
```

可以根据现有代码调整实际接口。

重点：

> 接口统一，算法不统一。

当前 MeArm：

```text
MeArmKinematics
    ├── FK
    └── existing MeArm IK
```

不要创建假的：

```text
GenericIK
```

不要为了支持 6DOF 而现在重写 IK。

---

# 11. IKResult

不要继续让上层代码依赖 MeArm 专属的 IK 返回格式。

建立统一结果概念，例如：

```ts
interface IKResult {
    success: boolean;
    joints: number[];
    positionError: number;
    orientationError?: number;
    constraintsSatisfied: boolean;
    solutionType: string;
    error?: string;
}
```

实际字段根据当前代码风格调整。

对于 MeArm：

```text
orientationError = undefined/null
```

是允许的。

因为：

> MeArm-V1 不需要伪装成 6D Pose IK。

---

# 12. 不要把 DOF 写死到通用接口

禁止：

```ts
function solveIK(x, y, z, roll, pitch, yaw)
```

作为通用 API。

因为：

```text
4DOF
5DOF
6DOF
```

任务空间能力不同。

当前 MeArm 可以继续使用：

```text
position target
```

未来 6DOF 再增加 orientation constraint。

---

# 13. Robot Runtime

如果当前已经存在：

```text
sim
serial
mujoco
mock
```

等 abstraction，优先复用。

不要重新建立第二套 Transport / Device / Runtime 系统。

目标只是让：

```text
Robot Model
      ↓
Runtime
```

之间职责更加清晰。

---

# 14. MuJoCo 不得重新建模

当前 MuJoCo 模型是 MeArm-V1 的一部分。

不要重新手工制作 MJCF。

继续保持：

```text
MeArm-V1 Robot Definition
        +
Physics Definition
        ↓
MJCF Generator
        ↓
MuJoCo
```

如果当前已经是这样，就只进行必要的接口整理。

禁止为了抽象而修改物理参数。

---

# 15. Three.js 不得改变视觉结果

抽象后：

```text
RobotDefinition
      ↓
Three.js
```

但是：

> MeArm 的外观、比例、关节运动、坐标系必须保持不变。

不能因为重构导致：

* 模型比例变化
* Joint axis 变化
* 零位变化
* 运动方向变化
* Gripper 行为变化

---

# 16. 第三阶段：Sim2Sim 回归

当前没有真实硬件。

因此建立：

```text
MeArm-V1
   ↓
3D FK
   ↓
MuJoCo
   ↓
MuJoCo state
   ↓
再通过 FK / 独立计算验证
```

核心目标：

> 验证架构抽象前后的机器人行为完全一致。

---

# 17. Sim2Sim 回归项目必须覆盖

## 17.1 Joint → FK

测试：

```text
joint
 ↓
FK
 ↓
XYZ
```

比较抽象前后的结果。

---

## 17.2 IK → FK

测试：

```text
XYZ
 ↓
IK
 ↓
joint
 ↓
FK
 ↓
XYZ
```

要求：

```text
error <= 当前项目已有阈值
```

不要随意放宽误差。

---

## 17.3 Three.js ↔ FK

随机生成合法 joint：

```text
joint
 ↓
FK
 ↓
Three.js pose
```

验证：

```text
Three.js transform
≈
FK result
```

必须继续满足当前已有精度。

---

## 17.4 FK ↔ MuJoCo

同一个 joint state：

```text
q
├── FK
└── MuJoCo
```

比较：

```text
end-effector position
joint position
link pose
```

允许使用当前已经存在的数值误差阈值。

---

## 17.5 IK ↔ MuJoCo

```text
target XYZ
      ↓
    IK
      ↓
joint q
      ↓
  MuJoCo
      ↓
end-effector
      ↓
target
```

验证最终位置误差。

---

# 18. 随机 Sim2Sim 测试

增加固定 seed 的随机测试。

建议至少：

```text
100
500
1000
```

个合法 joint pose。

每一个：

```text
random q
 ↓
FK
 ↓
Three.js
 ↓
MuJoCo
```

检查一致性。

同时：

```text
random reachable XYZ
 ↓
IK
 ↓
FK
 ↓
MuJoCo
```

检查闭环。

---

# 19. Regression Golden Rule

这是本任务最重要的验收原则：

> **抽象之后，MeArm-V1 的行为必须与抽象之前一致。**

不是：

```text
新架构自己测试通过
```

而是：

```text
旧基线
    VS
新架构

结果一致
```

---

# 20. 禁止行为

本任务明确禁止：

### 禁止 1

增加 CAD Import。

### 禁止 2

增加 URDF Import。

### 禁止 3

增加新机械臂。

### 禁止 4

重新设计 IK。

### 禁止 5

增加 5DOF / 6DOF solver。

### 禁止 6

增加 LLM。

### 禁止 7

增加 Voice。

### 禁止 8

增加 Vision。

### 禁止 9

增加 Gesture。

### 禁止 10

修改 MuJoCo 物理参数来“适应测试”。

### 禁止 11

为了让测试通过而降低测试精度。

### 禁止 12

删除原有测试。

### 禁止 13

大规模重命名导致历史代码无法追踪。

---

# 21. 如果发现现有代码问题

不要顺手修一堆。

分为：

```text
A. 与本次抽象直接相关
B. 与本次抽象无关
```

只有 A 类允许处理。

B 类问题记录：

```text
docs/architecture/mearm-v1-followups.md
```

以后单独处理。

---

# 22. Commit / 阶段要求

建议分成至少三个逻辑阶段。

## Commit 1

```text
freeze: establish MeArm-V1 baseline
```

内容：

* Baseline
* 固定测试数据
* 文档
* 当前测试结果

不得改变功能。

---

## Commit 2

```text
refactor: introduce minimal robot model interfaces
```

内容：

* RobotDefinition
* KinematicsEngine
* IKResult
* 最小 Runtime 抽象

不得改变 MeArm 行为。

---

## Commit 3

```text
test: add MeArm-V1 sim2sim regression
```

内容：

* FK regression
* IK regression
* Three.js/FK regression
* MuJoCo regression
* Sim2Sim regression

---

# 23. 最终目录建议

不要机械照抄，根据当前项目实际结构调整。

目标概念：

```text
MeArm-3D/
│
├── robot/
│   ├── RobotDefinition
│   ├── KinematicsEngine
│   ├── IKResult
│   └── mearm/
│       └── MeArmKinematics
│
├── simulation/
│   └── mujoco/
│
├── baseline/
│   └── mearm-v1/
│
├── tests/
│   ├── unit/
│   ├── integration/
│   └── sim2sim/
│
└── docs/
    └── architecture/
        ├── mearm-v1-baseline.md
        ├── mearm-v1-baseline-analysis.md
        └── mearm-v1-followups.md
```

不要为了达到这个目录结构而移动大量现有文件。

**代码职责优先，目录形式其次。**

---

# 24. 最终验收标准

任务完成必须同时满足：

## A. MeArm-V1 冻结

```text
PASS
```

存在明确：

```text
MeArm-V1
```

Robot Model。

---

## B. Baseline

```text
PASS
```

存在固定测试数据和固定 seed。

---

## C. 最小架构抽象

```text
PASS
```

至少存在：

```text
RobotDefinition
KinematicsEngine
IKResult
```

或者与当前项目架构等价的最小接口。

---

## D. MeArm IK 没有被重写

```text
PASS
```

现有 MeArm IK 算法继续工作。

---

## E. FK

```text
PASS
```

抽象前后结果一致。

---

## F. IK

```text
PASS
```

抽象前后结果一致。

---

## G. Three.js

```text
PASS
```

模型姿态、比例、运动方向保持一致。

---

## H. MuJoCo

```text
PASS
```

现有 MuJoCo 模型继续工作。

---

## I. Sim2Sim

```text
PASS
```

至少覆盖：

```text
Joint → FK
Joint → Three.js
Joint → MuJoCo
XYZ → IK → FK
XYZ → IK → MuJoCo
```

---

## J. 全量回归

运行当前项目全部已有测试。

要求：

```text
新测试全部 PASS
旧测试全部 PASS
TypeScript PASS
Go PASS
Vitest PASS
E2E PASS
MuJoCo PASS
Sim2Sim PASS
```

具体测试数量以实际项目当前结果为准。

---

# 25. 最终输出报告

完成后不要只告诉我：

```text
Done
```

必须输出：

## 1. 当前 MeArm-V1 结构

说明：

```text
哪些内容被冻结
哪些文件作为基线
```

## 2. 架构变化

用：

```text
Before
↓
After
```

说明最小抽象在哪里。

## 3. 未修改内容

明确列出：

```text
FK
IK
MuJoCo
robot.yaml
geometry
...
```

哪些保持原样。

## 4. 测试结果

提供实际：

```text
测试项
数量
PASS
FAIL
误差
```

## 5. Sim2Sim 结果

至少报告：

```text
FK max error
IK max error
Three.js max error
MuJoCo max error
```

具体数值必须来自实际测试。

## 6. 风险

如果发现任何：

```text
数值差异
坐标系差异
IK差异
MuJoCo差异
```

必须明确报告。

禁止用“基本一致”掩盖数值问题。

---

# 26. 最重要的执行原则

整个任务遵循：

```text
已有 MeArm
      ↓
冻结
      ↓
建立黄金基线
      ↓
最小抽象
      ↓
重新加载同一个 MeArm-V1
      ↓
Sim2Sim
      ↓
与旧基线比较
      ↓
全部一致
      ↓
任务完成
```

不要把本任务理解为：

> “把 MeArm-3D 改造成通用机器人平台。”

正确理解是：

> **“保护已经成功的 MeArm，并在它外面增加一层足够小的抽象，为未来通用机器人平台留下接口。”**

如果通用化设计与当前 MeArm-V1 发生冲突：

> **优先保护 MeArm-V1。**

如果发现某个抽象需要修改 MeArm 数学模型：

> **停止该抽象，不要修改 MeArm。**

如果未来需要支持 5DOF / 6DOF：

> **通过新增 Robot Profile + 新 Kinematics Solver 实现，而不是修改 MeArm-V1。**
