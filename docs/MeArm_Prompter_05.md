# MeArmPilot-ARM101 模型转换 + 后端配置驱动模型切换 + Sim2Sim 验证

## 一、任务目标

当前 MeArmPilot 已经完成：

1. MeArm-V1 基线冻结
2. MeArm-3D 最小 Robot 架构抽象
3. MeArm-V1 Sim2Sim 回归
4. MeArm-V1 当前验证数据保持稳定

下一阶段暂时不实现 CAD / STEP 导入，也不实现网页端模型选择。

本阶段只做一个事情：

> 引入第二个与 MeArm-V1 结构明显不同的机器人：SO-ARM101，用于验证 MeArmPilot 的多机器人模型抽象是否真正成立。

模型选择暂时只允许通过后端配置文件修改。

最终通过：

```text
配置文件
  ↓
Robot Model Loader
  ↓
RobotDefinition
  ↓
3D Model
  ↓
FK
  ↓
MuJoCo
  ↓
Sim2Sim
```

后续再增加网页端模型选择功能。

---

# 二、外部参考资料

使用以下公开资料作为 SO-ARM101 的模型来源，不允许凭外观自行重新建模。

## 1. TheRobotStudio SO-ARM100 / SO-101

仓库：

https://github.com/TheRobotStudio/SO-ARM100

SO-101 仿真说明：

https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/README.md

官方 URDF：

https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf

官方 MuJoCo MJCF：

https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.xml

必须优先使用：

```text
so101_new_calib.urdf
so101_new_calib.xml
```

官方说明指出：

* 模型由 Onshape CAD 通过 onshape-to-robot 生成
* 提供 URDF
* 提供 MuJoCo MJCF
* `so101_new_calib.xml` 是新的 calibration 版本
* `so101_old_calib.xml` 是旧 calibration 版本
* 当前任务优先使用 new calibration

## 2. Hugging Face LeRobot

https://huggingface.co/docs/lerobot/so101

只作为 SO-101 关节、执行器和机器人定义的辅助参考。

不要把 LeRobot 的完整控制框架直接搬进 ArmPilot。

---

# 三、重要原则

## 1. MeArm-V1 是 Golden Baseline

绝对禁止为了接入 SO-ARM101 而修改：

```text
MeArm-V1 FK
MeArm-V1 IK
MeArm-V1 joint limits
MeArm-V1 MuJoCo 参数
MeArm-V1 case 数据
MeArm-V1 Three.js 当前行为
MeArm-V1 既有测试
```

如果新的抽象与 MeArm-V1 当前实现冲突：

> 优先停止抽象，而不是修改 MeArm-V1。

---

# 四、当前 MeArm-V1 数据结构必须保持兼容

目前 MeArm-V1 的验证数据已经存在：

```text
fk-cases.json
ik-cases.json
joint-cases.json
workspace-cases.json
```

不要为了 SO-ARM101 强行把它们重构成其他格式。

这些文件属于：

> Robot-specific regression / validation data

而不是通用 RobotDefinition 本身。

SO-ARM101 可以建立自己的 cases。

例如：

```text
SO-ARM101/
├── fk-cases.json
├── joint-cases.json
└── ...
```

但是：

禁止为了“文件结构统一”而伪造：

```text
ik-cases.json
workspace-cases.json
```

如果当前阶段没有可靠的 SO-101 IK / workspace 数据：

```text
暂不提供
```

而不是生成假的数据。

---

# 五、推荐的模型目录

根据当前项目实际目录结构选择最小改动方案。

目标逻辑结构：

```text
RobotModels/
├── MeArm-V1/
│   ├── fk-cases.json
│   ├── ik-cases.json
│   ├── joint-cases.json
│   └── workspace-cases.json
│
└── SO-ARM101/
    ├── robot metadata
    ├── fk-cases.json
    ├── joint-cases.json
    ├── so101_new_calib.urdf
    ├── so101_new_calib.xml
    └── assets/
```

如果当前仓库已有更合适的目录结构：

> 不要为了这个任务进行大规模目录重构。

优先复用现有架构。

---

# 六、模型选择必须由后端配置控制

增加一个明确的机器人模型配置项。

例如：

```yaml
robot:
  model: mearm-v1
```

切换 SO-ARM101：

```yaml
robot:
  model: so-arm101
```

具体配置文件名称、位置和格式根据当前项目已有配置体系决定。

不要新建第二套互相冲突的配置系统。

---

# 七、配置的职责

配置文件只负责：

```text
选择当前 Robot Model
```

不要把 SO-ARM101 的所有参数复制进主配置文件。

错误：

```yaml
robot:
  model: so-arm101
  shoulder_limit: ...
  elbow_limit: ...
  wrist_limit: ...
  mesh: ...
  mass: ...
```

正确方向：

```yaml
robot:
  model: so-arm101
```

然后：

```text
RobotRegistry
    ↓
RobotModel
    ├── metadata
    ├── kinematics
    ├── cases
    ├── 3D assets
    └── MuJoCo model
```

机器人自身拥有自己的定义。

---

# 八、建立 Robot Registry / Loader

如果当前架构已经存在类似机制，优先扩展，而不是重复创建。

逻辑上需要：

```text
RobotRegistry
    ├── mearm-v1
    └── so-arm101
```

根据：

```text
robot.model
```

加载对应 RobotDefinition。

要求：

```text
load("mearm-v1")
load("so-arm101")
```

返回统一的 RobotDefinition / RobotModel 接口。

禁止在业务代码中大量出现：

```text
if robot == "mearm-v1"
if robot == "so-arm101"
```

机器人差异应该封装在 Robot Model 自身。

---

# 九、SO-ARM101 模型转换策略

不要重新制作 SO-ARM101 几何模型。

优先直接采用官方：

```text
so101_new_calib.urdf
so101_new_calib.xml
assets/*
```

需要解决的是：

```text
官方模型
    ↓
MeArmPilot Robot Model
    ↓
MeArmPilot 3D
    ↓
MeArmPilot MuJoCo
```

而不是：

```text
SO-101 外观
    ↓
自己重新猜尺寸
    ↓
自己重新猜 joint
    ↓
自己重新猜惯量
```

---

# 十、URDF / MJCF 的角色

当前阶段允许同时保留：

```text
SO-ARM101
├── URDF
└── MJCF
```

其中：

### URDF

主要用于：

* 机器人结构参考
* link / joint 定义
* joint axis
* joint limits
* visual / collision
* inertial
* FK 结构验证

### MJCF

主要用于：

* MuJoCo 仿真
* physics
* actuator
* collision
* dynamics
* simulation regression

不要要求 URDF 和 MJCF 在当前阶段由 MeArmPilot 自动互转。

---

# 十一、SO-ARM101 的关节结构

根据官方模型建立 RobotDefinition。

至少识别：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

注意：

SO-ARM101 是：

```text
5 个主要机械臂运动关节
+
1 个 gripper
```

不要简单因为存在 6 个执行电机就把它当作“完整 6D pose 机械臂”。

当前阶段主要验证：

```text
5DOF arm
+
gripper
```

---

# 十二、不要现在实现 SO-ARM101 专用复杂 IK

当前目标不是开发 SO-101 IK。

第一阶段只要求：

```text
Joint
 ↓
FK
 ↓
End Effector Pose
```

以及：

```text
Joint
 ↓
MuJoCo
 ↓
End Effector Pose
```

IK 暂时可以：

```text
SO-ARM101 IK = unavailable / not implemented
```

如果当前统一接口要求存在 IK：

必须支持明确的 capability：

```text
supportsIK: false
```

或者使用当前项目已有的 capability 表达方式。

禁止：

> 为了让测试通过而复制 MeArm 的 IK。

禁止：

> 为了让 API 看起来完整而伪造 SO-101 IK。

---

# 十三、SO-ARM101 FK

需要建立 SO-101 的 FK 实现或适配层。

优先方案：

```text
RobotDefinition
    ↓
SO101Kinematics
    ↓
FK
```

FK 必须依据官方 URDF / MJCF 的：

```text
joint origin
joint axis
link transform
```

进行计算。

不要依据图片估算。

不要依据视觉模型矩阵直接反推 FK。

Three.js renderer 不得成为 FK 的唯一实现。

---

# 十四、FK 验证

至少建立：

```text
joint-cases.json
fk-cases.json
```

测试：

```text
Joint
 ↓
MeArmPilot FK
 ↓
End Effector Pose A

Joint
 ↓
MuJoCo
 ↓
End Effector Pose B
```

比较：

```text
position_error
orientation_error
```

如果当前阶段只可靠验证 position：

可以明确：

```text
position_error
```

作为主要指标。

不要伪造 orientation accuracy。

---

# 十五、3D 模型接入

Three.js 必须根据当前 RobotDefinition 加载对应模型。

模型切换：

```text
robot.model = mearm-v1
```

显示 MeArm-V1。

切换：

```text
robot.model = so-arm101
```

显示 SO-ARM101。

必须保证：

```text
MeArm mesh
    不影响
SO-101 mesh

SO-101 mesh
    不影响
MeArm mesh
```

禁止通过复制大量 MeArm renderer 代码创建 SO-101 专用页面。

---

# 十六、MuJoCo 接入

根据 RobotDefinition 加载：

```text
MeArm-V1 → 当前 MeArm MJCF
SO-ARM101 → so101_new_calib.xml
```

不得修改官方 SO-101 物理参数，仅为了让视觉结果更接近而随意调整。

如果发现：

```text
MuJoCo
vs
FK
```

不一致：

优先调查：

```text
coordinate frame
joint axis
joint origin
zero position
joint sign
model conversion
```

而不是直接修改物理参数。

---

# 十七、配置切换测试

必须验证：

### Case A

```yaml
robot:
  model: mearm-v1
```

启动：

```text
MeArm-V1
```

### Case B

```yaml
robot:
  model: so-arm101
```

启动：

```text
SO-ARM101
```

两个配置分别执行完整测试。

---

# 十八、模型切换后必须检查

### MeArm-V1

```text
DOF = 当前基线
joint names 正确
joint limits 正确
3D 正确
FK 正确
MuJoCo 正确
Sim2Sim 正确
原有 cases 全部通过
```

### SO-ARM101

```text
joint count 正确
joint names 正确
joint limits 正确
3D 正确
FK 正确
MuJoCo 正确
Sim2Sim 正确
```

---

# 十九、Sim2Sim 最小回归矩阵

必须建立统一测试框架：

```text
                  MeArm-V1       SO-ARM101
------------------------------------------------
Model Load           ✓               ✓
Joint Definition     ✓               ✓
Joint Limits         ✓               ✓
3D Model             ✓               ✓
FK                   ✓               ✓
MuJoCo               ✓               ✓
FK ↔ MuJoCo          ✓               ✓
Sim2Sim              ✓               ✓
IK                   ✓               -
Workspace             ✓               -
```

其中：

```text
-
```

代表：

> 当前阶段明确未实现，不算失败。

---

# 二十、最重要的跨机器人测试

同一个测试框架：

```text
runSim2Sim(robot)
```

分别执行：

```text
runSim2Sim("mearm-v1")
runSim2Sim("so-arm101")
```

而不是：

```text
runMeArmSim2Sim()
runSO101Sim2Sim()
```

必须证明：

> Sim2Sim 测试框架本身与具体机器人解耦。

---

# 二十一、模型切换压力测试

连续：

```text
MeArm
→ SO101
→ MeArm
→ SO101
...
```

至少执行多次模型加载 / 测试。

检查：

```text
□ joint state 没有串模型
□ RobotDefinition 没有串
□ FK solver 没有串
□ 3D mesh 没有残留
□ MuJoCo model 没有残留
□ joint limits 没有串
□ cases 没有串
□ UI 没有残留
```

---

# 二十二、禁止事项

本任务明确禁止：

```text
❌ CAD / STEP 导入
❌ CAD 自动 joint 推断
❌ 网页机器人选择
❌ LLM
❌ Vision
❌ Voice
❌ Gesture
❌ RL
❌ Dataset training
❌ SO-101 真机控制
❌ 修改 MeArm-V1 baseline
❌ 重写 MeArm FK
❌ 重写 MeArm IK
❌ 为 SO-101 伪造 IK
❌ 为 SO-101 伪造 workspace
❌ 为了测试通过修改官方物理参数
❌ 删除原有测试
❌ 修改原有测试预期结果来适应新架构
❌ 大规模目录重构
```

---

# 二十三、执行步骤

## Phase 0：只读分析

先不要修改代码。

检查：

```text
当前 MeArm-3D Robot abstraction
当前 config
当前 RobotDefinition
当前 FK
当前 IK
当前 MuJoCo loader
当前 Three.js model loader
当前 cases loader
当前 Sim2Sim tests
```

输出：

```text
1. 当前模型加载路径
2. 当前 MeArm-V1 数据路径
3. 当前抽象接口
4. 最小 SO-101 接入点
5. 需要修改的文件列表
```

等待确认后再实施。

---

## Phase 1：引入 SO-ARM101 外部模型

下载/复制官方：

```text
so101_new_calib.urdf
so101_new_calib.xml
assets/*
```

保留来源说明：

```text
source:
TheRobotStudio/SO-ARM100

model:
SO101

variant:
so101_new_calib
```

不要修改官方模型内容。

如果因为路径、资源加载等原因必须做适配：

必须记录：

```text
original
modified
reason
```

---

## Phase 2：建立 SO-ARM101 RobotDefinition

实现最小：

```text
metadata
joints
limits
kinematics
visual model
MuJoCo model
capabilities
```

其中：

```text
supportsFK = true
supportsIK = false   # 当前阶段如果没有可靠实现
supportsWorkspace = false / unavailable
```

使用当前项目已有能力表达方式。

---

## Phase 3：配置模型选择

增加：

```yaml
robot:
  model: mearm-v1
```

支持：

```yaml
robot:
  model: so-arm101
```

启动时根据配置加载对应 RobotDefinition。

---

## Phase 4：3D 模型切换

验证：

```text
mearm-v1
```

可以正常显示。

然后改配置：

```text
so-arm101
```

可以正常显示 SO-101。

不能修改 MeArm 原有视觉行为。

---

## Phase 5：SO-101 FK

完成：

```text
joint values
    ↓
SO101Kinematics
    ↓
end effector pose
```

建立固定 FK cases。

---

## Phase 6：SO-101 MuJoCo

加载：

```text
so101_new_calib.xml
```

验证：

```text
joint state
    ↓
MuJoCo
    ↓
end effector pose
```

与 MeArmPilot FK 对比。

---

## Phase 7：Sim2Sim

统一执行：

```text
Joint
 ↓
FK
 ↓
EE Pose A

Joint
 ↓
MuJoCo
 ↓
EE Pose B

A ↔ B
```

输出：

```text
position_error
orientation_error
pass/fail
```

不要编造误差阈值。

根据当前 MeArm 测试体系和数值精度确定合理阈值，并说明依据。

---

## Phase 8：模型切换回归

依次：

```text
MeArm-V1
SO-ARM101
MeArm-V1
SO-ARM101
```

运行完整 regression。

确认：

```text
MeArm-V1 regression == 原 baseline
```

---

# 二十四、最终验收标准

## A. MeArm-V1

必须：

```text
原有测试全部通过
原有 FK/IK 数据不变
原有 Sim2Sim 数据不变
原有行为不变
```

## B. SO-ARM101

必须：

```text
官方模型成功加载
3D 模型正确
joint definition 正确
joint limits 正确
FK 正确
MuJoCo 正确
FK ↔ MuJoCo Sim2Sim 通过
```

## C. Model Switching

必须：

```text
只修改配置中的 model
即可选择机器人
```

例如：

```yaml
robot:
  model: mearm-v1
```

和：

```yaml
robot:
  model: so-arm101
```

无需修改业务代码。

## D. 架构

必须证明：

```text
同一套 Robot API
        ↓
MeArm-V1
SO-ARM101
```

都能够工作。

不得通过：

```text
if robot == ...
```

在上层大量堆积机器人特判。

---

# 二十五、最终报告

完成后输出：

## 1. 当前 Robot Architecture

```text
config
 ↓
RobotRegistry
 ↓
RobotDefinition
 ├── MeArm-V1
 └── SO-ARM101
```

## 2. SO-ARM101 模型来源

记录：

```text
repository
URDF
MJCF
asset
calibration variant
license/source information
```

## 3. 转换/适配过程

明确：

```text
哪些内容直接使用官方模型
哪些内容由 MeArmPilot 适配
为什么需要适配
```

## 4. Sim2Sim 结果

分别报告：

```text
MeArm-V1
SO-ARM101
```

实际测试数量、误差、失败数量。

禁止编造数据。

## 5. MeArm-V1 Regression

必须明确：

```text
Before
After
```

确认基线没有变化。

## 6. 当前能力边界

明确列出：

```text
SO-101 FK：支持
SO-101 MuJoCo：支持
SO-101 Sim2Sim：支持
SO-101 IK：暂未实现 / 当前状态
SO-101 workspace：暂未实现 / 当前状态
网页模型切换：暂未实现
CAD 导入：暂未实现
```

---

# 最核心的验收问题

最终不要回答：

> “SO-ARM101 做出来了吗？”

而要回答：

> **“在完全不修改 MeArm-V1 核心模型的情况下，ArmPilot 是否已经能够通过配置加载第二个完全不同的机器人，并使用同一套上层模型接口完成 3D、FK、MuJoCo 和 Sim2Sim？”**

如果答案是 YES：

本阶段成功。

如果答案是 NO：

不要继续增加功能，先解决模型抽象问题。
