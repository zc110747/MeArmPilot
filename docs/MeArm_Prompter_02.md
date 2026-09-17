# MeArmPilot 机械臂 3D 建模开发提示词

## 1. 项目目标

开发一个基于 Web 的机械臂 3D 虚拟建模与运动仿真模块。

当前阶段只关注：

1. 机械臂 3D 模型建立
2. 机械臂各部件参数化
3. 机械臂关节层级关系建立
4. 机械臂关节旋转控制
5. 机械臂末端位置计算
6. 虚拟机械臂与真实机械臂的关节角度一一对应
7. 为后续机械臂控制提供清晰、稳定的 API

当前阶段明确禁止加入：

* AI 学习
* 强化学习
* 模型训练
* 手势识别
* 摄像头识别
* 自动轨迹学习
* 机器学习数据集
* AI Agent 决策逻辑

这些功能未来单独增加。

当前目标是：

> 先把“虚拟机械臂 = 真实机械臂的数字孪生基础模型”做好。

---

# 2. 机械臂对象

目标机械臂为四舵机结构的 meArm 风格机械臂。

机械臂包含：

1. Base / 底座旋转
2. Shoulder / 下臂关节
3. Elbow / 上臂关节
4. Gripper / 夹爪

机械结构可以参考 meArm 的经典结构，但不要简单加载一个黑盒模型。

必须建立自己的参数化机械臂模型。

模型需要能够根据参数修改：

* 底座尺寸
* 底座高度
* 下臂长度
* 上臂长度
* 末端连杆长度
* 夹爪尺寸
* 各关节位置
* 各关节旋转轴
* 各关节角度限制

修改参数后，3D 模型应该自动重新构建或更新。

---

# 3. 整体技术要求

采用 Web 3D 技术实现。

推荐：

* Three.js
* TypeScript
* Vite

如果项目已经存在前端框架，则优先复用现有框架。

不要为了建模引入过重的 3D 引擎。

模型核心必须基于：

* THREE.Object3D
* THREE.Group
* THREE.Mesh
* THREE.Geometry / BufferGeometry
* Quaternion / Euler
* Vector3

建立机械臂层级结构。

---

# 4. 建模风格

## 4.1 整体视觉风格

采用：

> 工程化、简洁、现代、低多边形工业机械臂风格。

不要做成：

* 写实工业机器人
* 卡通玩具
* 科幻机器人
* 复杂工业设备
* 高精度影视级模型

重点是：

> 清晰表达机械结构和运动关系。

---

# 5. 模型视觉要求

机械臂整体：

* 结构清晰
* 边缘适当倒角
* 表面使用简单 PBR 材质
* 不使用复杂纹理
* 不依赖贴图才能正确显示
* 不使用高面数模型
* 保证 WebGL 实时运行

推荐视觉：

* 主体使用工程塑料/金属质感
* 关节轴使用金属材质
* 螺丝等小部件适量表现
* 夹爪结构必须明显
* 不需要真实螺纹
* 不需要内部齿轮
* 不需要内部电机建模

---

# 6. 模型结构

必须严格建立以下层级。

```text
Robot
│
├── Base
│   └── BaseRotation
│       └── Shoulder
│           └── UpperArm
│               └── Elbow
│                   └── ForeArm
│                       └── Wrist
│                           └── Gripper
│                               ├── LeftFinger
│                               └── RightFinger
```

实际实现可以根据 meArm 机械结构调整，但必须满足：

> 父关节旋转会带动所有子结构一起运动。

例如：

```text
BaseRotation
    ↓
整个机械臂左右旋转

Shoulder
    ↓
上臂、前臂、夹爪整体运动

Elbow
    ↓
前臂、夹爪整体运动

Gripper
    ↓
只控制夹爪开合
```

禁止通过分别修改每个 Mesh 的世界坐标来模拟机械臂运动。

必须使用：

> Scene Graph / Parent-Child Hierarchy

实现真实的机械结构关系。

---

# 7. 坐标系

必须首先定义统一坐标系。

推荐：

```text
X = 左右
Y = 上下
Z = 前后
```

机械臂初始状态：

```text
Base 位于：

X = 0
Y = 0
Z = 0
```

机械臂默认朝：

```text
+Z
```

方向伸展。

Y 为世界竖直方向。

所有尺寸统一：

> mm

但 Three.js 内部可以：

```text
1 unit = 1 mm
```

或者：

```text
1 unit = 1 cm
```

必须在项目中统一，不允许不同模块使用不同单位。

推荐：

```text
1 unit = 1 mm
```

---

# 8. 参数系统

不要把尺寸直接写死在建模代码中。

建立：

```ts
ArmModelConfig
```

例如：

```ts
interface ArmModelConfig {
    baseWidth: number;
    baseDepth: number;
    baseHeight: number;

    shoulderHeight: number;

    upperArmLength: number;
    foreArmLength: number;

    endEffectorLength: number;

    gripperWidth: number;
    gripperLength: number;

    jointRadius: number;

    baseAngleMin: number;
    baseAngleMax: number;

    shoulderAngleMin: number;
    shoulderAngleMax: number;

    elbowAngleMin: number;
    elbowAngleMax: number;

    gripperAngleMin: number;
    gripperAngleMax: number;
}
```

提供一个默认配置。

例如：

```ts
const defaultArmConfig = {
    baseWidth: ...,
    baseDepth: ...,
    baseHeight: ...,

    shoulderHeight: ...,

    upperArmLength: ...,
    foreArmLength: ...,

    endEffectorLength: ...,

    gripperWidth: ...,
    gripperLength: ...,

    jointRadius: ...,

    baseAngleMin: ...,
    baseAngleMax: ...,

    shoulderAngleMin: ...,
    shoulderAngleMax: ...,

    elbowAngleMin: ...,
    elbowAngleMax: ...,

    gripperAngleMin: ...,
    gripperAngleMax: ...
};
```

具体默认尺寸如果无法从真实机械臂获得，不允许伪装成真实尺寸。

必须明确标记：

```text
DEFAULT / 示例尺寸
```

后续可以通过配置修改。

---

# 9. 参数化建模原则

所有主要机械部件都必须由参数生成。

例如：

```text
createBase(config)
createShoulder(config)
createUpperArm(config)
createForeArm(config)
createGripper(config)
```

最终：

```ts
createRobot(config)
```

统一生成机械臂。

禁止：

```ts
// 大量写死坐标
mesh.position.set(23, 14, 76);
mesh.position.set(27, 15, 91);
mesh.position.set(...);
```

这种方式只能用于固定的小型装饰件。

主要机械尺寸必须来自 config。

---

# 10. 关节模型

建立统一关节对象：

```ts
interface Joint {
    name: string;

    minAngle: number;
    maxAngle: number;

    angle: number;

    axis: THREE.Vector3;

    object: THREE.Object3D;
}
```

至少包括：

```text
base
shoulder
elbow
gripper
```

---

# 11. 关节旋转轴

必须根据真实机械结构定义。

例如：

### Base

绕 Y 轴旋转：

```text
axis = (0, 1, 0)
```

### Shoulder

根据实际模型结构确定旋转轴。

### Elbow

根据实际模型结构确定旋转轴。

### Gripper

夹爪左右手指围绕自身转轴运动。

禁止仅凭视觉随意设置旋转轴。

必须在代码中显式定义：

```ts
joint.axis
```

并在开发过程中验证。

---

# 12. 关节零位

必须定义机械臂标准零位：

```text
base = 0°
shoulder = 0°
elbow = 0°
gripper = 0°
```

零位必须具有明确物理意义。

建议：

> 机械臂自然展开、朝向 +Z 的状态作为默认零位。

必须在代码中明确：

```ts
ZERO_POSITION
```

不要依赖模型加载后的随机姿态。

---

# 13. 角度系统

内部统一：

```text
degree
```

因为真实舵机最终使用角度。

但是 Three.js 内部旋转计算可以转换成：

```text
radian
```

必须统一封装：

```ts
degToRad()
radToDeg()
```

禁止在业务代码中大量出现：

```ts
angle * Math.PI / 180
```

---

# 14. 角度限制

每个关节必须具备：

```text
minAngle
maxAngle
```

任何输入必须经过：

```ts
clampAngle()
```

例如：

```ts
function setJointAngle(
    jointName: string,
    angle: number
) {
    const joint = joints[jointName];

    angle = clamp(
        angle,
        joint.minAngle,
        joint.maxAngle
    );

    joint.angle = angle;

    updateJointTransform(joint);
}
```

不能允许 UI 或外部 API 直接把关节设置到无限角度。

---

# 15. 虚拟机械臂 API

必须提供统一控制接口。

例如：

```ts
arm.setJointAngle("base", 30);
arm.setJointAngle("shoulder", 45);
arm.setJointAngle("elbow", 60);
arm.setJointAngle("gripper", 20);
```

同时支持：

```ts
arm.setJointAngles({
    base: 30,
    shoulder: 45,
    elbow: 60,
    gripper: 20
});
```

获取：

```ts
arm.getJointAngles();
```

返回：

```ts
{
    base: 30,
    shoulder: 45,
    elbow: 60,
    gripper: 20
}
```

这是后续连接真实机械臂最重要的接口之一。

---

# 16. 虚拟机械臂与真实机械臂的一一对应原则

必须保证：

```text
Virtual Base Angle
        ↓
Real Base Servo

Virtual Shoulder Angle
        ↓
Real Shoulder Servo

Virtual Elbow Angle
        ↓
Real Elbow Servo

Virtual Gripper Angle
        ↓
Real Gripper Servo
```

但是必须预留：

> 虚拟角度 → 实际舵机角度映射层。

例如：

```ts
interface ServoMapping {
    virtualMin: number;
    virtualMax: number;

    servoMin: number;
    servoMax: number;

    reversed: boolean;
    offset: number;
}
```

因为：

```text
虚拟模型 0°
```

不一定等于：

```text
真实舵机 0°
```

因此必须避免把：

```text
模型角度
```

和：

```text
舵机 PWM / Servo Angle
```

直接硬编码绑定。

---

# 17. 正运动学

必须提供：

```ts
arm.getEndEffectorPosition()
```

返回：

```ts
{
    x: number;
    y: number;
    z: number;
}
```

该位置必须来自：

> 3D Scene Graph 的实际世界坐标。

例如：

```ts
const position = new THREE.Vector3();

endEffector.getWorldPosition(position);
```

同时提供：

```ts
arm.getEndEffectorTransform()
```

返回末端：

```text
position
rotation
quaternion
```

这样后续可以用于 IK。

---

# 18. 不要过早实现 IK

当前阶段：

> IK 不是建模核心。

可以预留接口：

```ts
arm.setTargetPosition(x, y, z);
```

但当前版本不要为了 IK 复杂化模型系统。

优先保证：

```text
Joint Angle
    ↓
3D Model
    ↓
End Effector Position
```

完全正确。

后续再增加：

```text
Target Position
    ↓
IK
    ↓
Joint Angles
```

---

# 19. 旋转控制方式

推荐使用：

```text
Quaternion
```

或者经过统一封装的：

```text
Euler
```

但不能同时在不同模块混乱使用。

推荐：

```text
外部 API
    ↓
角度 degree
    ↓
Joint Controller
    ↓
Quaternion
    ↓
Three.js
```

这样未来可以避免欧拉角万向节锁等问题。

---

# 20. 机械结构可视化

为了调试，必须增加：

### 坐标轴

显示：

```text
X
Y
Z
```

### 关节轴

每个关节显示：

```text
rotation axis
```

例如使用：

```ts
THREE.AxesHelper
```

### 关节中心

可以显示小球：

```text
Joint Origin
```

### 末端坐标系

在夹爪末端显示：

```text
End Effector Frame
```

包含：

```text
X
Y
Z
```

这些调试元素必须可以关闭。

例如：

```ts
arm.setDebugVisible(true);
arm.setDebugVisible(false);
```

---

# 21. 地面和参考环境

3D 场景至少包含：

```text
Ground
Grid
Coordinate Axis
Robot
Camera
Light
```

推荐：

```text
GridHelper
AxesHelper
```

用于确认机械臂空间运动是否正确。

---

# 22. 摄像机

默认摄像机应能够看到完整机械臂。

支持：

* OrbitControls
* 缩放
* 旋转
* 平移

提供：

```text
Front
Side
Top
Isometric
```

四种观察方式。

---

# 23. 光照

采用简单实时光照。

建议：

```text
Ambient / Hemisphere Light
Directional Light
```

不要依赖复杂 HDRI。

模型即使离线也必须保持正常可见。

---

# 24. 材质

采用简单材质。

优先：

```text
MeshStandardMaterial
```

不要在第一版加入：

* 复杂贴图
* 法线贴图
* 环境反射贴图
* PBR 材质库

目标是：

> 几何结构优先，视觉效果其次。

---

# 25. 模型性能

第一版必须保证：

* 浏览器直接运行
* 不依赖服务器渲染
* 不使用高面数模型
* 不使用大型纹理
* 不出现明显掉帧

机械臂主体应该保持低复杂度。

目标：

```text
60 FPS
```

普通桌面浏览器下应流畅操作。

---

# 26. UI

当前 UI 只用于：

> 验证 3D 模型和关节控制。

至少提供：

```text
Base
    [slider]  -180° ~ 180°

Shoulder
    [slider]

Elbow
    [slider]

Gripper
    [slider]
```

实时显示：

```text
Base: XX°
Shoulder: XX°
Elbow: XX°
Gripper: XX°
```

同时显示：

```text
End Effector

X: XXX mm
Y: XXX mm
Z: XXX mm
```

---

# 27. 参数编辑

提供一个：

```text
Model Parameters
```

区域。

至少能够修改：

```text
Base Width
Base Height
Upper Arm Length
Fore Arm Length
Gripper Width
Gripper Length
```

修改后：

```text
重新生成模型
```

或者：

```text
实时更新模型
```

两者均可。

优先选择实现简单、稳定的方式。

---

# 28. 模型重建原则

如果尺寸参数发生变化：

```text
旧模型
    ↓
dispose
    ↓
重新生成
    ↓
重新绑定 Joint
    ↓
恢复当前关节角度
```

必须避免：

* GPU Geometry 泄漏
* Material 泄漏
* Event Listener 泄漏
* 重复添加模型
* 多次初始化后出现多个机械臂

---

# 29. 项目代码结构

建议：

```text
src/
├── main.ts
│
├── arm/
│   ├── ArmModel.ts
│   ├── ArmConfig.ts
│   ├── Joint.ts
│   ├── JointController.ts
│   ├── ServoMapping.ts
│   │
│   ├── geometry/
│   │   ├── Base.ts
│   │   ├── Shoulder.ts
│   │   ├── UpperArm.ts
│   │   ├── ForeArm.ts
│   │   └── Gripper.ts
│   │
│   └── kinematics/
│       └── ForwardKinematics.ts
│
├── scene/
│   ├── SceneManager.ts
│   ├── CameraManager.ts
│   ├── LightManager.ts
│   └── DebugHelper.ts
│
├── ui/
│   ├── JointPanel.ts
│   └── ParameterPanel.ts
│
└── types/
    └── arm.ts
```

如果已有项目结构，不需要机械照搬。

核心原则是：

> 建模、关节、场景、UI 分离。

---

# 30. 禁止事项

开发过程中禁止：

1. 把整个机械臂做成一个 Mesh
2. 通过修改 Mesh 世界坐标实现关节运动
3. 把所有坐标硬编码
4. 把真实舵机 PWM 直接写入 3D 模型
5. 使用 AI 自动生成不可控的复杂模型
6. 使用复杂外部 CAD 模型作为第一版核心模型
7. 为视觉效果牺牲机械结构正确性
8. 先做 UI 再补模型层级
9. 在没有确认坐标系的情况下开始写运动代码
10. 在没有确认零位和旋转轴的情况下开始做 IK

---

# 31. 开发执行顺序

必须严格按照以下顺序执行。

## Step 1：分析项目

检查现有：

```text
package.json
src/
vite.config.*
tsconfig.*
```

确认当前技术栈。

不要立即修改代码。

---

## Step 2：建立坐标系

先定义：

```text
X = 左右
Y = 上下
Z = 前后
```

定义：

```text
1 unit = 1 mm
```

并记录在：

```text
docs/coordinate-system.md
```

---

## Step 3：建立参数系统

实现：

```text
ArmModelConfig
```

建立默认尺寸。

所有默认尺寸必须注明：

```text
Reference / Example
```

如果不是实际测量值，不得声称是真实 meArm 尺寸。

---

## Step 4：建立最小机械结构

首先只实现：

```text
Base
UpperArm
ForeArm
Gripper
```

不要加入复杂装饰。

确认：

```text
Base → Shoulder → Elbow → Gripper
```

父子层级正确。

---

## Step 5：建立关节轴

分别定义：

```text
Base Axis
Shoulder Axis
Elbow Axis
Gripper Axis
```

使用：

```text
AxesHelper
```

进行人工检查。

---

## Step 6：实现角度控制

实现：

```ts
setJointAngle()
setJointAngles()
getJointAngle()
getJointAngles()
```

完成：

```text
Slider
 ↓
Joint Angle
 ↓
3D Model
```

---

## Step 7：实现正运动学输出

实现：

```ts
getEndEffectorPosition()
```

确保：

```text
模型真实末端位置
=
API 输出位置
```

---

## Step 8：增加调试模式

加入：

```text
Joint Origin
Joint Axis
World Axis
End Effector Axis
```

可以开关。

---

## Step 9：增加参数编辑

实现机械尺寸修改。

验证：

```text
修改长度
↓
模型改变
↓
关节层级没有破坏
↓
角度控制仍然有效
```

---

## Step 10：建立 Servo Mapping

仅建立接口：

```ts
virtualAngle
→
servoAngle
```

当前阶段不连接真实串口。

---

## Step 11：优化模型

最后再处理：

* 倒角
* 材质
* 光照
* 螺丝
* 细节
* UI 美化

绝不能反过来。

---

# 32. 必须执行的建模验证

完成模型后必须主动进行以下实验。

## Test 1：Base Rotation

设置：

```text
Base = -90°
Base = 0°
Base = +90°
```

确认：

> 整个机械臂绕正确的竖直轴旋转。

---

## Test 2：Shoulder

固定：

```text
Base = 0°
Elbow = 0°
```

改变：

```text
Shoulder
```

确认：

> 上臂及其所有子结构正确运动。

---

## Test 3：Elbow

固定：

```text
Base
Shoulder
```

改变：

```text
Elbow
```

确认：

> 只有肘部以下结构跟随运动。

---

## Test 4：Gripper

改变：

```text
Gripper
```

确认：

> 只有夹爪发生开合。

---

## Test 5：父子关系测试

设置：

```text
Base = 45°
Shoulder = 30°
Elbow = 50°
```

确认：

> 所有关节组合运动没有出现脱节。

---

## Test 6：极限角度

分别设置：

```text
minAngle
maxAngle
```

确认：

> 模型不会超过机械限制。

---

## Test 7：非法角度

输入：

```text
-9999°
9999°
NaN
Infinity
```

必须：

```text
安全处理
```

不能导致模型崩溃。

---

## Test 8：末端位置

随机改变三个关节。

记录：

```text
3D 模型末端世界坐标
```

与：

```text
getEndEffectorPosition()
```

比较。

误差必须接近：

```text
0
```

允许浮点计算误差。

---

## Test 9：模型尺寸变化

修改：

```text
UpperArmLength
ForeArmLength
```

确认：

```text
模型长度改变
关节关系正确
末端位置同步改变
```

---

## Test 10：重复初始化

连续执行：

```ts
createRobot()
createRobot()
createRobot()
```

确认：

> 场景中只有一个机械臂。

不存在：

* 重复 Mesh
* 重复 Event
* GPU Resource Leak

---

# 33. 最终验收标准

只有同时满足以下条件，模型才算完成。

## A. 几何结构

* [ ] 机械臂结构完整
* [ ] Base 正确
* [ ] Shoulder 正确
* [ ] Upper Arm 正确
* [ ] Elbow 正确
* [ ] Fore Arm 正确
* [ ] Gripper 正确

---

## B. 层级结构

* [ ] 使用 Object3D / Group 建立父子关系
* [ ] Base 控制整个机械臂
* [ ] Shoulder 控制其下级结构
* [ ] Elbow 控制前臂及末端
* [ ] Gripper 独立控制

---

## C. 参数化

* [ ] 主要尺寸没有硬编码
* [ ] 有统一 Config
* [ ] 修改长度可以改变模型
* [ ] 修改尺寸不会破坏关节

---

## D. 运动

* [ ] Base 旋转正确
* [ ] Shoulder 旋转正确
* [ ] Elbow 旋转正确
* [ ] Gripper 开合正确
* [ ] 关节限制有效
* [ ] 非法输入不会崩溃

---

## E. 坐标

* [ ] 世界坐标定义明确
* [ ] 单位统一
* [ ] 关节坐标系明确
* [ ] 末端坐标系明确
* [ ] 旋转轴明确

---

## F. API

必须存在：

```ts
setJointAngle()
setJointAngles()
getJointAngle()
getJointAngles()
getEndEffectorPosition()
getEndEffectorTransform()
```

---

## G. 调试

必须支持：

```text
显示/隐藏：

World Axis
Joint Axis
Joint Origin
End Effector Axis
Grid
```

---

## H. 性能

桌面浏览器环境：

```text
正常操作 ≥ 60 FPS
```

不出现明显：

* 卡顿
* 内存持续增长
* WebGL Resource Leak

---

# 34. 最终交付物

开发完成后必须输出：

```text
1. 可运行 Web 3D 机械臂
2. 参数化模型代码
3. 关节控制代码
4. Forward Kinematics
5. Servo Mapping Interface
6. Debug 模式
7. 参数编辑 UI
8. 坐标系文档
9. 模型结构文档
10. 验收测试结果
```

同时生成：

```text
docs/
├── coordinate-system.md
├── model-structure.md
├── joint-definition.md
├── api.md
└── validation.md
```

---

# 35. 开发者执行要求

你不是在制作一个“看起来像机械臂”的网页模型。

你的目标是：

> 建立一个能够和现实四舵机机械臂进行关节级一一映射的数字机械臂模型。

因此开发优先级必须严格按照：

```text
机械结构正确
        ↓
坐标系正确
        ↓
关节层级正确
        ↓
旋转轴正确
        ↓
角度控制正确
        ↓
末端坐标正确
        ↓
参数化
        ↓
真实舵机映射接口
        ↓
视觉优化
```

如果视觉效果和机械结构正确性发生冲突：

> 永远优先机械结构正确性。

如果模型尺寸无法确定：

> 不要猜测为真实尺寸。

应使用：

```text
可配置参数
+
明确标记的默认值
```

如果发现 meArm 的真实机械结构与当前假设不一致：

> 暂停继续扩展模型，先修正机械结构、坐标系、关节轴和零位定义。

最终验收的核心不是：

> “模型看起来像不像 meArm”。

而是：

> “给定一组真实舵机角度，虚拟机械臂是否能够按照相同的关节关系、旋转方向、旋转轴和角度限制运动。”

达到这一标准后，再进入后续：

```text
虚拟机械臂
      ↕
Servo Mapping
      ↕
真实机械臂
```

以及之后的：

```text
IK
摄像头
手势
动作控制
AI
```

这些全部属于后续阶段，不得在本阶段提前耦合。
