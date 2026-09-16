# 末端目标「边界值」是怎么算出来的

> **本文只描述现状**，不含任何修改建议。所有数值取自 `robot-package/mearm-v1/model/robot.yaml`（唯一真值）
> 与代码实测，非手抄估计值。
>
> 本文回答两个问题：
> ① **XYZ 数字输入**（含 Move 按钮、XYZ ± Jog）时，"超出范围"是怎么判的？
> ② **场景里拖动蓝色把手**时，"限制范围"是怎么算的？

---

## 0. 一句话结论（先看这个）

**Web 端有两套判据，但它们都在同一个函数 `solveIk()` 里，且判完之后的处理是同一种：拒绝 + 保留目标 + 变红。**

| 路径 | 有无「范围钳制」 | 边界在哪判 | 越界后行为 |
|---|---|---|---|
| **XYZ 数字输入** | ❌ 无。输入框没有 `min`/`max`，输入什么就送什么 | `solveIk()` 双重判据 | `target` 保留、`commandJoints` **逐位不变**、面板报 `reason@joint` |
| **鼠标拖动** | ❌ 无。交点原样送进 `moveTo()` | 同上（**完全共用**） | 同上 + 把手球变红 + 画分离虚线 |

**关键认知：这两条路径在"限制范围"这件事上没有任何独立逻辑** —— 它们都把候选点丢给同一个
`solveIk()`，由它来判。区别只在**输入点是怎么产生的**：

- 数字输入：用户敲的十进制字面量（可带任意舍入误差）
- 拖动：射线 ∩ 冻结平面的**真实交点**（数学上精确，但**几何上必然落在边界之外**）

这就是"拖动到极限，记录 XYZ，回 HOME 再手输，却被拒"的**结构性原因** —— 见 §5。

---

## 1. 判据的唯一来源：`solveIk()`

文件：`robot-package/mearm-v1/kinematics/ik.ts`

调用链：

```
TargetControl.commit()   ─┐
TargetControl.jog()      ─┤
DragHandle.continueDrag()─┘
        ↓
robotStore.moveTo(xyz)                 ← frontend/src/store/robotStore.ts
        ↓
solveIk(model, xyz, {prefer:'nearest', near, seed})   ← robot-package/.../ik.ts
        ↓
  ┌─────────────────────────────────────────────┐
  │ 层① 几何球壳（reach）   → OUT_OF_WORKSPACE   │
  │ 层② 关节限位（limits）  → JOINT_LIMIT @ 关节 │
  │ 层③ 输入框约束          → 无                 │
  └─────────────────────────────────────────────┘
```

### 1.1 层①：几何球壳判据（`OUT_OF_WORKSPACE`）

先看代码原文（`ik.ts` §②）：

```ts
const { dr, dz } = wristSagittal(geometry, target);
const d = Math.hypot(dr, dz);

const [reachMin, reachMax] = geometry.reach;          // = [|l1−l2|, l1+l2]
if (d > reachMax + EPS_MM || d < reachMin - EPS_MM) {
  return { reason: 'OUT_OF_WORKSPACE', candidates: [], ... };
}
```

**"扣偏移"这一步是判据成立的前提**（`wristSagittal()`）：

```ts
export function wristSagittal(geometry, target) {
  const [x, y, z] = target;
  const r = Math.hypot(x, y);
  return {
    dr: r - geometry.pivotR - geometry.toolOffset[0],   // 径向
    dz: z - geometry.pivotZ - geometry.toolOffset[1],   // 竖直
  };
}
```

换算成人类可读的公式：

```
r  = hypot(x, y)                       目标点的水平半径
R  = r − pivotR − 40                   腕枢轴的水平半径（扣掉 40mm 的爪水平前伸）
Z  = z − pivotZ − 0                    腕枢轴的高度
d  = hypot(R, Z)                       腕枢轴 ↦ 肩枢轴 的距离
可达 ⟺  |l1 − l2| ≤ d ≤ l1 + l2
```

**本机实际数值**（从 `robot.yaml` 求导，非手写）：

| 量 | 值 | 来源 |
|---|---|---|
| `pivotZ` | 60.0 mm | 肩枢轴高度（= `column_link.length`） |
| `pivotR` | 0.0 mm | 肩枢轴在偏航轴上 |
| `l1` | 80.0 mm | `upper_arm_link.length` |
| `l2` | 80.0 mm | `forearm_link.length`（**不含**腕→TCP 那 40mm） |
| `toolOffset` | `[40, 0]` | 腕枢轴 → TCP，纯水平 +40mm |
| **`reach`** | **`[0, 160]`** | `[|80−80|, 80+80]` |

⚠️ **`reachMin = 0` 是"两根 80 等长"的副产物**，不是"能伸到肩枢轴里面"。
真实的不可达内圈由**关节限位**（层②）裁掉，见 §5.2。

⚠️ **球壳判据不知道关节限位**。它只回答"这两根杆能不能摆出这个距离"，
回答不了"摆到那个角度时 shoulder/elbow 是否还在 108.44°..141.86° 之间"。
所以**几何可达 ≠ 真机能到**。这是本机边界问题的主要来源。

⚠️ `EPS_MM = 1e-9`。判"越界"和判"在界上"的分界是**纳米级**的 —— 没有安全余量。

### 1.2 层②：关节限位判据（`JOINT_LIMIT`）

几何壳过了之后，逐个关节判限位。代码（`ik.ts`）：

```ts
function limitViolation(joint: Joint, value: number): number {
  return Math.max(0, joint.limits.min - value, value - joint.limits.max);
}
```

即 `violation = 越界的度数`，在界内或正好在界上时为 `0`。

**(a) base（偏航）单独判，与支解无关**

```ts
const azimuthRaw = azimuthIndeterminate ? 0 : radToDeg(Math.atan2(y, x));
const baseViolation = azimuthIndeterminate ? 0 : limitViolation(base, azimuthUsed);
```

- `azimuthRaw = atan2(y, x)`：偏航角**只由 XY 方向决定，与 Z 无关**
- `r < 1e-9`（目标落在偏航轴上）时方位角数学不定 ⇒ 取 `near`/`homePose` 再**钳位**
- base 限位 = `[-60, 60]` ⇒ **方位角超出 ±60° 的点，无论多远都直接判 `JOINT_LIMIT @ base`**
  （注意：这条是"角度"限制，不是"距离"限制 —— 球壳判据完全覆盖不到它）

**(b) shoulder / elbow 逐支判**

```ts
const vS = limitViolation(shoulder, ths);
const vE = limitViolation(elbow, the);
const violation = Math.max(vS, vE);
const violatedJoint = violation <= EPS_DEG ? undefined : vE >= vS ? elbow.id : shoulder.id;
...
feasible: violation <= EPS_DEG,     // ⚠️ EPS_DEG = 1e-9
```

⚠️ **可行判据是零容差的**：`violation <= 1e-9` 度才算可行。
等价于要求 **目标点必须能由 FK 逐位精确产出**（因为 IK 的解一定精确落在限位上，
而用户给的点带舍入 ⇒ 解会溢出限位千分之几度）。

**(c) 选支：几何可达但两支都越界**

```ts
const feasible = candidates.filter((c) => c.feasible && baseViolation <= EPS_DEG);
if (feasible.length === 0) {
  ...
  const best = candidates.reduce((a, b) => (a.violation <= b.violation ? a : b));
  // 报越界最轻的那支，message 里写明"仍差 X°"
}
```

**两支解**（`elbow-up` / `elbow-down`）的含义：

| 支 | `α = θe − θs` | 几何含义 |
|---|---|---|
| `elbow-up` | `α > 0` | 肘部高于「肩→目标」连线，**真机 HOME 所在支** |
| `elbow-down` | `α < 0` | 肘部低于该连线（小臂向回折） |

⚠️ 因为本机 `elbow` 存**绝对角**，两支解的 `shoulder` / `elbow` 数值会呈现**互换式镜像**
（不是简单取负）。判越界时必须两支都算，不能只算一支。

### 1.3 层③：输入框约束 —— **没有**

`frontend/src/components/RobotControl/TargetControl.tsx` 原文：

```tsx
<input
  type="number"
  step={1}
  value={draft[index]}
  ...
/>
```

**没有 `min`、没有 `max`、没有 `step` 以外的任何限制**。
`commit()` 只做一件事：

```ts
const parsed = draft.map((s) => Number(s)) as [number, number, number];
if (parsed.some((v) => !Number.isFinite(v))) return;   // 只挡 NaN / Infinity
editingRef.current = false;
moveToXyz(parsed);                                      // 原样送进去
```

⇒ 手输路径**完全依赖 `solveIk()` 的拒绝**来划边界，前端不做任何预判。

### 1.4 越界之后会发生什么（`robotStore.moveTo`）

```ts
} else {
  // ⚠️ 目标越界时关节**逐位不变**（Phase 6 已拍板）：保留 target 让用户看到"差多远"，
  //    但绝不静默钳位 —— 钳位会让工作空间边界从界面上消失，也无法保证钳位点满足关节限位。
  set({
    target: [xyz[0], xyz[1], xyz[2]],        // ← 目标照常写入（"我想去哪"）
    ikStatus: { ok: false, reason, joint, message },
  });
}
```

| 量 | 越界时 | 说明 |
|---|---|---|
| `target` | **被更新** | 记住"我想去哪"，界面能看到差多少 |
| `commandJoints` | **逐位不变** | "实际去哪"完全不动，机械臂不抖 |
| `endEffector` | 不变 | `commandJoints` 的 FK 结果 |
| `ikStatus` | `{ok:false, reason, joint, message}` | 面板红字 |
| 日志 | 一条 `console.info` | `[target] REASON @ joint: message` |

面板显示（`TargetControl.tsx`）：

```
OK   · elbow-up · 残差 1.5e-13 mm · 方位 29.5°          ← 成功
JOINT_LIMIT @ elbow                                       ← 失败：原因 + 关节
JOINT elbow 141.862 (limit 108.4414852068..141.8582211436)：几何可达但两支解都越界，最接近的一支（elbow-up）仍差 0.004°
```

---

## 2. 拖动路径：限制范围是怎么算的

**结论：拖动路径没有任何"限制范围"的计算 —— 它把交点原样交出去，靠 store 拒绝 + 视觉反馈。**

### 2.1 三步链路

**(a) `pointerdown`：吸附起点 + 冻结平面**（`DragHandle.tsx`）

```ts
const beginDrag = (event) => {
  state.resetTarget();                                   // ← 把手吸附到真实 TCP
  const anchor = useRobotStore.getState().endEffector.position;
  camera.getWorldDirection(cameraDir);
  modeRef.current = state.dragPlane;
  planeRef.current = makePlane(state.dragPlane, anchor, [cameraDir.x, cameraDir.y, cameraDir.z]);
  ...
};
```

⚠️ `resetTarget()` 是**关键**：它把 `target` 设为当前 TCP，避免"从幽灵位置起跳"。
在 `dragPlane.ts` 头部有明确理由说明平面为什么必须**冻结**：

> 若每帧按当前 TCP 重建平面，会形成自反馈环：
> 目标点变 → 关节变 → TCP 变 → 平面又跟着变 → 手势"飘走"且永远收敛不到鼠标处。

**(b) `pointermove`：射线 ∩ 平面 → 原样送 store**

```ts
const continueDrag = (event) => {
  ...
  const point = dragTarget(modeRef.current, plane, [ray.origin...], [ray.direction...]);
  if (!point) return;                       // ← 唯一的分支：射线失败就保持上一目标
  useRobotStore.getState().moveTo(point);   // ← 没有任何范围钳制
};
```

⭐ **这里就是答案**：`moveTo(point)` 的 `point` 是**射线与冻结平面的真实交点**，
未经过任何裁剪、钳位、球壳检测。**越界与否完全由 `solveIk()` 事后判定。**

**(c) `useFrame`：事后的视觉反馈**

拖动时唯一的"看到边界"的渠道，是每帧同步的把手球外观：

```ts
const COLOR_OUT_OF_REACH = '#e5534b';
useFrame(() => {
  // 把手位置 = store.target（越界时仍会动到越界点）
  // 颜色 = state.ikStatus?.ok ?? true  ⇒ 失败时变红
  // 若 target 与 TCP 分离距离 > SEPARATION_EPS_MM(0.4mm)，画虚线连接
});
```

| 反馈 | 判据 | 含义 |
|---|---|---|
| 把手球变红 | `ikStatus?.ok === false` | 当前目标解不出来 |
| 虚线连接 | `dist(target, TCP) > 0.4mm` | 目标没到，"还差这么远" |
| 把手停住 | `dragTarget()` 返回 `null` | 射线平行于平面，或交点在相机后方 |

⇒ **把手会一路跟着鼠标跑出工作空间，然后变红** —— 这是设计意图（"看到差多远"），
不是 bug。但它也意味着**拖动时没有任何"软墙"手感**。

### 2.2 平面模式（决定哪个自由度被锁）

`frontend/src/robot/interaction/dragPlane.ts`：

```ts
export function planeNormalFor(mode, cameraDir): Vec3 {
  switch (mode) {
    case 'xy':     return [0, 0, 1];                                  // 法线 = Z
    case 'xz':     return [0, 1, 0];                                  // 法线 = Y
    case 'camera': return normalizeVec3([-cameraDir[0], -cameraDir[1], -cameraDir[2]]) ?? [0, 0, 1];
  }
}
```

| 模式 | 平面 | 被锁死的轴 | 说明 |
|---|---|---|---|
| `xy`（默认） | 水平面（法线 = 世界 Z） | **Z 锁死在锚点高度** | 只改 X / Y |
| `xz` | 矢状面（法线 = 世界 Y） | **Y 锁死在锚点** | 只改 X / Z |
| `camera` | 朝向相机的平面 | 无（XYZ 全自由） | 法线 = 视线反向 |

锁死是**精确相等**，不是"浮点近似"：

```ts
export function snapToMode(mode, point, anchor): Vec3 {
  switch (mode) {
    case 'xy': return [point[0], point[1], anchor[2]];      // ← 取回 anchor[2]
    case 'xz': return [point[0], anchor[1], point[2]];      // ← 取回 anchor[1]
    case 'camera': return [point[0], point[1], point[2]];
  }
}
```

⚠️ **对本机的一个直接后果**：默认模式 `xy` 锁死 Z。
如果锚点（拖动起点的 TCP）的 Z 已经贴在工作空间边缘，那么**整条 XY 拖动都注定越界** ——
因为 Z 被钉住了，而 `dz = z − pivotZ` 是个常量，球壳判据退化成"水平圆环"
（`hypot(R, dz)` 落在 `[0, 160]`）。这时怎么拖都出不来。

### 2.3 射线求交的两个失败条件

```ts
export function intersectRayPlane(origin, dir, plane): Vec3 | null {
  const denom = dot(plane.normal, dir);
  if (Math.abs(denom) < EPS_PARALLEL) return null;              // ① 平行：|n·d| < 1e-9
  const t = dot(plane.normal, sub(plane.point, origin)) / denom;
  if (!(t >= 0)) return null;                                   // ② 背向：交点在相机后方
  return addScaled(origin, dir, t);
}
```

- ① **平行**：射线与平面无交点（含"整条射线都在平面内"）
- ② **背向**：`t < 0`，交点在相机后方 —— 鼠标"看向"平面之外

两种情况都返回 `null`，调用方 `if (!point) return;` **保持上一目标不动**。
⚠️ 注意这里是**保持**，不是"钳到边界"。

---

## 3. 两条路径的对照

```
                      数字输入                     拖动
                  ┌──────────────┐          ┌──────────────────┐
输入点来源         │ 用户敲的字符串 │          │ 射线 ∩ 冻结平面   │
                  │ （带舍入误差） │          │ （数学上精确）    │
                  └──────┬───────┘          └────────┬─────────┘
                         │                           │
                         │  step={1}，无 min/max      │  无钳制，原样
                         │                           │
                         └───────────┬───────────────┘
                                     ▼
                          robotStore.moveTo(xyz)
                                     │
                     ┌───────────────┴───────────────┐
                     ▼                               ▼
            solverKind !== 'analytic'          solveIk(model, xyz, ...)
                     │                               │
                     ▼                    ┌──────────┴──────────┐
              NO_SOLVER                   ▼                     ▼
              （保留 target）       层① 球壳 d∈[0,160]?    层② base / shoulder / elbow
                                   否 → OUT_OF_WORKSPACE    限位（容差 1e-9）
                                                           否 → JOINT_LIMIT @ joint
                                     │                     │
                                     └──────────┬──────────┘
                                                ▼
                                   target 保留 / commandJoints 逐位不变
                                   ikStatus = {ok:false, ...}
                                                │
                          ┌─────────────────────┴─────────────────────┐
                          ▼                                           ▼
                   面板红字 reason@joint                    把手球变红 + 虚线分离
```

**两者共用完全相同的判据**，没有任何一条路径有独立的边界公式。

---

## 4. 真机实测可达范围（对照用）

用 `core/tools/ws_scan_mearm_v1.py`（走独立参考实现 `fkref.py`，
**不是**调前端 bridge 再算一次）在 J1/J2/J3 上做 1° 网格扫描，共 **234498** 个合法位形，
统计 TCP 落点：

| 轴 | 实测范围（mm） | 与纯几何判据的差 |
|---|---|---|
| X | `[40.686, 177.091]` | — |
| Y | `[-153.365, 153.365]` | 由 base `±60°` 直接决定 |
| Z | `[48.965, 114.693]` | — |
| **R（水平半径）** | **`[81.373, 177.091]`** | 几何判据给 `D ∈ [0, 160]` ⇒ `R ∈ [40, 200]` |

**差距归因**（都是**限位裁剪**，不是模型缺陷）：

| 端 | 几何判据 | 实测 | 差 |
|---|---|---|---|
| 内圈 | `R_min = 40` | `R_min = 81.373` | **+41.373 mm** 被限位裁掉 |
| 外圈 | `R_max = 200` | `R_max = 177.091` | **−22.909 mm** 被限位裁掉 |

⇒ **球壳（层①）只是必要条件，关节限位（层②）才是真正的边界。**
单靠球壳判据描述边界，内圈会虚报 41mm、外圈会虚报 23mm。

### 4.1 端点量化（5% 判据复核）

限位端点本身**不可精确到达**（整数度量化后会落到限位外，被 `ERR JOINT` 拒）。
复核脚本 `core/tools/ik_limit_gap_pct.py` 输出 **16 项判定 · 正常 14 · 超 5.0% 的 2**：

| 关节 | 端点 | round 到 | 量化偏差率 | 判定 |
|---|---|---|---|---|
| shoulder | 下限 `-6.093683` | `-6` | **1.5374 %** | ✅ 正常 |
| shoulder | 上限 `49.454929` | `49` | **0.9199 %** | ✅ 正常 |
| elbow | 下限 `108.441485` | `108` | **0.4071 %** | ✅ 正常 |
| elbow | 上限 `141.858221` | `142` | **0.0999 %** | ✅ 正常 |
| FK→IK 自反性 | 5 组姿态 × 2 关节 | — | **0.0000 %** | ✅ 全部正常 |

⚠️ 上面 5 组自反性 = `home` / `shoulder_min` / `shoulder_max` / `elbow_min` / `elbow_max`，
**FK 精确产出的点全部能逐位解回** ⇒ 计算链自洽。

⚠️ **那 2 项"超限"是我自己的指标设计错误**，不是模型缺陷。
第 ② 组拿 `R_max = 200` 当分母去比 `R_min` 的差值（`41.373 / 200 = 20.69%`），
**分母选错了** —— 内圈差值应该用 `R_min` 或"该轴量程"当分母。
这两种差距的真实来源是**关节限位裁剪**，属预期行为（见 §4 归因表）。

**安全整数区间**（round 到整数度后仍落在限位内）：

```
base     ∈ [-60, 60]
shoulder ∈ [-6, 49]        ← 由 -6.093683 / 49.454929 向内外取整
elbow    ∈ [109, 141]      ← 由 108.441485 / 141.858221 向内外取整
```

⚠️ `elbow` 的两端**round 到整数后都会掉出限位** ——
`108 < 108.4415`、`142 > 141.8582`。所以**只能取 109..141**，不能用 `round()` 得的 108 / 142。

---

## 5. 为什么"拖到极限 → 记录 XYZ → 手输"会被拒

这是用户实际遇到的场景，根因有**两层**，且互相独立。

### 5.1 第一层：滑杆读数与手输之间存在舍入差

| 来源 | 值 |
|---|---|
| 滑杆拖到极限时的 FK 精确 TCP | `(130.5020, 73.8345, 49.5107)` |
| 显示给用户 / 用户手输的 | `(130.4, 73.8, 49.5)` |
| **差** | **0.108 mm** |

界面只显示 **1 位小数**（`toDraft` 用 `toFixed(1)`），用户也只能按 1 位精度回输。
但 `solveIk()` 的可行容差是 **1e-9 度 / 1e-9 mm**。

⇒ 解出的 `elbow = 141.862303°`，限位上限 `141.8582211436°`，
**越界 0.0038°**（舵机侧换算后 0.0090°）⇒ 判 `JOINT_LIMIT @ elbow`。

**量级对比**：输入偏差 `0.108 mm` 是判据容差 `1e-9 mm` 的 **1.08 亿倍**。
这不是"精度不够"，是**两套量级差了八个数量级**。
### 5.2 第二层：极限点本身就在限位表面上，向任何方向都只会更糟

用户拖到极限时，`shoulder` 或 `elbow` **恰好**贴在限位上（例如 `elbow = 141.8582211436`）。
这个点是**由 FK 精确产出**的，所以它能通过 1e-9 判据。但：

- 舍入 → 偏离表面 → 溢出 ⇒ 拒
- 想"往内缩一点" → 但界面上没有任何"内缩"机制，用户只能靠试

而且**球壳判据帮不上忙**：极限点的 `d` 深在 `[0, 160]` 内部，
层① 一定放行，问题全部暴露在层②。

### 5.3 用户报告的两个具体点，实测定位

**(a) `(97, 157, 49)` —— 该点本身不可能到达**

```
r = hypot(97, 157) = 184.548 mm
真机扫描实测最大 r = 177.091 mm                    ← 见 §4，1° 网格扫描结果
差 = 7.457 mm                                       ← 超出真机物理可达上限
另外：由该点换算出腕枢轴距离 d = hypot(184.548−40, 49−60) = 144.974 mm
      —— d 落在 [0,160] 内 ⇒ **层① 球壳会放行**，被拒是因为层② 关节限位
```

⇒ 这个点在**真机上物理不可达**（超出扫描出的 r 上限 7.457mm）。用户记录这个值时，
界面显示的应该是"滑杆推到了极限、但 TCP 没跟上"的**幽灵位置**。

**(b) `(130.4, 73.8, 49.5)`（截图） —— 差 0.0038°**

```
r = hypot(130.4, 73.8) = 149.835 mm
R = 149.835 − 0 − 40    = 109.835 mm
Z = 49.5   − 60 − 0     = −10.5  mm
d = hypot(109.835, 10.5) = 110.336 mm       ← 深在 [0, 160] 内，层① 放行
⇒ 解出 elbow = 141.862303°
   限位 max  = 141.8582211436°
   越界       = 0.0038°                      ← 层② 拒绝
```

**对照：同一个点用 FK 精确值喂进去**

```
FK 精确 TCP = (130.5020, 73.8345, 49.5107)
r = 149.9410 mm
R = 109.9410 mm
Z = −10.4893 mm
d = 110.4403 mm                             ← 与上面只差 0.104mm
⇒ 通过（这个点正是 FK 精确产出，必然能解回原角）
```

两个点的 `d` 只差 **0.104 mm**，判定却相反 —— **判据分界线就在这 0.1mm 量级上**。

**根因链**：
```
界面 1 位小数舍入（0.108mm）
   +  球壳判据不含限位信息（层① 必然放行）
   +  容差 1e-9 度（零余量）
   ⇒ 极限点回输必然被拒
```

### 5.4 一个反直觉的确认：Web 端计算链**没有 bug**

把 FK 的**精确值**（不带舍入）喂回 IK：

| 输入 | IK 结果 |
|---|---|
| FK 精确 TCP `(130.5020, 73.8345, 49.5107)` | 完美解回原关节角，**偏差 0.0000%** |

⇒ 计算链（FK / IK / 标定 / 限位）本身是自洽的。
问题**纯粹**出在"1 位小数显示"与"1e-9 度判据"这两个量级的不匹配上。

---

## 6. 判据的"三层语义"速查

`solveIk()` 会返回三种失败之一，请勿混为一谈：

| `reason` | 触发条件（代码位置） | 物理含义 | 用户该做什么 |
|---|---|---|---|
| `OUT_OF_WORKSPACE` | 层① `d > reachMax+1e-9 \|\| d < reachMin−1e-9` | **杆摆不出这个距离** | 换个近点的目标 |
| `JOINT_LIMIT` | 层② `violation > 1e-9`（含 base / shoulder / elbow） | 杆能摆出来，但**真机做不出那个角** | 挪到限位内部 |
| `NO_SOLVER` | `capability.solverKind !== 'analytic'` | **这台机器人压根没有逆解器**，求解未被尝试 | 改用 Joint 面板直接驱动 |

⚠️ `NO_SOLVER` 与另外两条**性质不同**：另两条是"试过、算出来了、不行"，
这条是"**没算**"。判据是**数据**（`capability.solverKind`），不是 `if robot === …`。
目前只有 SO-ARM101 返回它。

---

## 7. 相关文件索引

| 主题 | 文件 |
|---|---|
| IK 判据本体（**唯一**） | `robot-package/mearm-v1/kinematics/ik.ts` |
| 真值（限位 / 杆长 / TCP 偏移） | `robot-package/mearm-v1/model/robot.yaml` |
| XYZ 输入 + Jog + IK 状态显示 | `frontend/src/components/RobotControl/TargetControl.tsx` |
| 拖动把手（起点吸附 / 冻结平面 / 事后反馈） | `frontend/src/components/RobotScene/DragHandle.tsx` |
| 平面数学（射线求交 / 吸附 / 模式） | `frontend/src/robot/interaction/dragPlane.ts` |
| store（`moveTo` / `target` / `ikStatus`） | `frontend/src/store/robotStore.ts` |
| Phase 6 冻结语义的守护断言 | `frontend/tests/acceptance/drag-tracking.test.ts`（9 条）<br>`frontend/tests/unit/robotStore.target.test.ts`（4 条） |
| 工作空间扫描工具 | `core/tools/ws_scan_mearm_v1.py` → `core/tools/out/ws_mearm_v1.json` |
| 复现用户报告现象 | `core/tools/repro_web_limit.py` |
| 限位几何求导 + 端点量化 | `core/tools/ik_limit_diag.py` |
| FK→IK 自反性诊断 | `core/tools/ik_reflexivity_diag.py` |
| 差距对照 / 5% 判据复核 | `core/tools/ik_limit_gap.py`、`core/tools/ik_limit_gap_pct.py` |

⚠️ `ik.ts` 被 **sim2sim**（经 `kinematics-bridge.mjs` 直接调用）与 **sim2real** 共同使用，
是本项目里**最不能随便改**的文件之一。
