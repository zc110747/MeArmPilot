# 分析：三条约束的可行性与实现方案

> 状态：**分析完成，待实施**
> 触发需求：
> 1. 硬误差保证在 2% 以内（限位可调）
> 2. 几何球壳的内径可调（选项在「末端目标」中支持修改，**最小 0.1**，其它方式不能变化）
> 3. 不影响任何算法，仅影响参数

---

## 一、先把三条约束翻译成可执行判据

### 约束 1「硬误差 ≤ 2%（限位可调）」

**"硬误差"指什么？** 唯一有物理意义的解释：**接受一个目标点后，机械臂实际到达的位置
与用户指定的位置之差**。

这个差从哪来？——**关节限位钳位**。求解后 `completeState()` 与 `clipJointState()`
都会把关节角裁到 `[limits.min, limits.max]`。若解出的角越界 `T` 度，被裁回来后
末端就偏离 `≈ L·sin(T)`。

所以「硬误差 ≤ 2%」= **允许判据放宽到 `T` 度，但 `T` 必须小到钳位误差仍 ≤ 2%**。

**反推 `T`（实算，非估计）**：

| reach 口径 | 2% 对应位移 | L=80 | L=120 | L=160（最坏力臂） |
|---|---|---|---|---|
| 177.091（实测最大 R） | 3.542 mm | 2.537° | 1.691° | **1.268°** |
| 150.200（另一口径） | 3.004 mm | 2.152° | 1.434° | **1.076°** |

⇒ **取 `T = 1.0°` 是唯一同时满足两侧的整数选择**：

| L | 钳位误差上限 | 占 177.091 |
|---|---|---|
| 80 | 1.396 mm | 0.788 % |
| 120 | 2.094 mm | 1.183 % |
| **160** | **2.793 mm** | **1.577 %** ✅ |

⚠️ `T = 2.0°` 时最坏 **3.154 %——超了**，所以 2.0 不能用。

**★ 关键认知转变（这条最重要）**

我原先的直觉是"把限位往内收紧就能解决舍入问题"。**这是错的**。
收紧限位只是**把悬崖搬了个位置**：用户读到的新极限点仍会因 1 位小数舍入而
解出「限位 + 0.004°」，仍被 `EPS_DEG = 1e-9` 拒掉。

**真正起作用的是「判据容差 `T`」**，不是限位数值本身。两者是**两件独立的事**：

| 手段 | 作用 | 是否解决舍入被拒 |
|---|---|---|
| 收紧限位（限位可调） | 把物理可达边界往内挪 | ❌ 只搬悬崖，不解决 |
| 放宽判据容差 `T` | 允许"差一点点"的点通过 | ✅ 真正解决 |

⇒ 约束 1 的落点应是：**新增一个判据容差参数 `T`（默认 1.0°）**，
而「限位可调」是它的**配套**（把边界挪到安全侧，留出余量）。

### 约束 2「几何球壳内径可调，最小 0.1，其它方式不能变化」

当前球壳是 `reach = [|l1 − l2|, l1 + l2] = [0, 160]`，**由连杆长度自动求导**。

"内径可调"= 把 `reachMin` 从"求导值"改为"可覆写参数"，下界 **0.1**（不能是 0）。

⚠️ 「**其它方式不能变化**」这条约束的含义：**内径只能由这个 UI 选项改，
不能因为改了别的（杆长、限位、标定）而被动变化**。所以它必须是一个
**独立的覆写值**，而不是"从别处派生出来的数"。

⚠️ 内径下限取 **0.1** 而不是 0 的物理理由：`reachMin = 0` 时
`cosAlpha = (0 − l1² − l2²)/(2·l1·l2)` 在 `l1 = l2` 时恰好等于 −1，
`acos(−1) = 180°` 是**退化位形**（两杆完全对折重合），
在 `clamp1()` 的边界上，且 FK 无法稳定产出该点。留 0.1mm 避开这个奇点。

### 约束 3「不影响任何算法，仅影响参数」

这条决定了**改哪些文件、不能改哪些文件**。

| 层 | 归属 | 能否改 |
|---|---|---|
| `ik.ts`（求解公式、`assertPlanar2R`、`ikGeometry` 求导逻辑） | **算法** | ❌ **一个字都不能动** |
| `fk.ts` / `transform.ts`（正运动学） | **算法** | ❌ 不能动 |
| `RobotModel.ts` 校验规则 | **契约** | ⚠️ 谨慎 |
| `robot.yaml` 的数值 | **参数** | ✅ 可调 |
| `robotStore` 的参数管理 | **参数** | ✅ 可加 |
| `TargetControl.tsx` 的 UI | **参数入口** | ✅ 可加 |

⚠️ **`ik.ts` 不能改** —— 它被 **sim2sim**（经 `kinematics-bridge.mjs` 直接调用）
与 **sim2real** 共用。改它 = 同时改三条链，违反约束 3。

**但 `reach` 与可行判据都在 `ik.ts` 里** —— 这是本次实现的核心矛盾。

---

## 二、核心矛盾与解法

### 矛盾

`ik.ts` 的 `ikGeometry()` 把 `reach` **算死**：

```ts
reach: [Math.abs(l1 - l2), l1 + l2],      // ← 已经写死了
```

`geometryCache` 又是 `WeakMap<RobotModel, IkGeometry>` —— **按模型对象缓存**。
所以"改 reach 内径"必须让**缓存失效**，否则改了不生效。

### 解法：改「模型对象」，不改「算法代码」

三条约束的公共解是：**在 `RobotModel` 上加一层"参数覆写"，让 `ik.ts` 读到不同的输入，
而它自己的代码一行不改。**

具体做法：

```
robot.yaml（真值，不动）
      ↓  读取
RobotModel（pristine，不动）
      ↓  applyParameterOverrides(model, overrides)   ← 新增，纯参数变换
RobotModel（patched，含覆写后的 limits / reach）
      ↓  喂给
solveIk()（ik.ts，一字不改）  →  reach 与限位都用 patched 的值
```

⚠️ **`ik.ts` 的 `geometryCache` 是 `WeakMap<RobotModel, IkGeometry>`** ⇒
只要传入的是**新的模型对象**，缓存天然失效，**不需要改 `ik.ts`**。这是本方案成立的关键。

⚠️ 但要注意：若每次 `moveTo` 都造新对象，`WeakMap` 会**每帧都 miss** ⇒
必须**把覆写后的模型缓存住**（按 overrides 的序列化值做 key），而不是每调用一次新建。

---

## 三、就绪检查：有没有别的坑

### 坑 1：`ACTUATOR_REACH` 校验会拦下放宽的限位 ⚠️

`RobotModel.ts:441` 强制「关节限位 → 舵机空间」必须落在舵机硬限位内。

实算 `elbow`（`servo_8`：`reverse=true, scale=2.39401, offset=359.61`，硬限位 `20..100`）：

| | 值 |
|---|---|
| 限位 `108.441485` → 舵机 | **100.00**（= 硬限位 max） |
| 限位 `141.858221` → 舵机 | **20.00**（= 硬限位 min） |
| 可放宽的余量 | **0.000000°** |

⇒ **`elbow` 限位一个度都放不宽**，它已经**顶死在舵机硬限位上**。
`shoulder` 同理（`80..160` → 关节恰好 `-6.093683..49.454929`）。

**结论：约束 1 的"限位可调"只能是「向内收紧」，不能「向外放宽」。**
这与"留安全余量"的意图一致，不冲突 —— 但必须在 UI 上明确，否则用户会试图放宽然后失败。

### 坑 2：收紧限位会让 HOME 位失效 ⚠️

`RobotModel.ts:498` 校验 `homePose` 各关节必须落在限位内。
`homePose.elbow = 112.6186` ⇒ 若把下限收到 `> 112.6186`，**模型直接判非法**。

⇒ 收紧幅度必须对 HOME 留余量，或在覆写后**跳过该校验**
（因为覆写是"运行期参数"而非"配置真值"，不该让真值校验失败）。

**决策**：覆写层**不做模型级校验**（`validate: false` 语义），
只做**覆写值自身的范围校验**（例如 `min < max`、`reachMin ≥ 0.1`）。
理由：真值校验的对象是 `robot.yaml`，而覆写是**会话内的显示/安全参数**，
两者不该互相污染。

### 坑 3：Phase 6 冻结语义有测试守护 ⚠️

`drag-tracking.test.ts`（9 条）+ `robotStore.target.test.ts`（4 条）共 13 条断言
守护「**越界即拒绝、关节逐位不变**」。

若引入容差 `T`，行为会从"越界即拒"变成"越界 ≤ T 则接受（且钳位）" ⇒
**这 13 条断言会失败**。

**这不是缺陷，是语义变更。** 必须：
- 默认 `T` 的取值要**让原行为在未配置时保持不变**（即默认 `T = 0`，等价于现状）
- 只有用户在 UI 上显式调整后，才启用容差
- ⇒ **13 条断言在默认配置下继续通过**，另加新断言覆盖"启用容差"的行为

⚠️ 这与约束 3「不影响任何算法」一致：**默认不改变行为 = 不影响算法**。

### 坑 4：Core → Robot Package 边界白名单 ⚠️

`corePackageBoundary.test.ts` 用白名单校验全部 Core→包的 import，
现白名单只有 `robotStore.ts: ['solveIk', 'IkBranch', 'IkPreference', 'IkReason', 'IkResult']`。

⇒ 若新增的参数模块 import 了包内符号，**必须同步登记白名单**，否则契约测试失败。

### 坑 5：真值冻结（D55/D71/D80）⚠️

`robot.yaml` 的语义核心被哈希冻结。**本方案不动 `robot.yaml`** ⇒ 不触发连锁清单。
这是刻意的：约束 3 要求"仅影响参数"，而改 `robot.yaml` 属"改真值"，会牵连
产物重生成 + 黄金数据重采集。

---

## 四、实施方案

### 参数定义（新增，不改任何算法）

```ts
export interface TargetGuardParams {
  /** 判据容差（degree）。默认 0 = 与引入本特性前逐位一致。上限 1.0（= 硬误差 1.577%）。 */
  jointToleranceDeg: number;
  /** 球壳内径覆写下限（mm）。默认 0.1，允许 [0.1, 外径)。 */
  reachMinMm: number;
  /** 是否启用球壳内径覆写；false 时用求导值（= 现状） */
  reachMinOverridden: boolean;
}
```

### 落点（哪些文件）

| 文件 | 改动性质 | 说明 |
|---|---|---|
| `frontend/src/robot/model/parameterOverrides.ts` | **新增** | 纯函数：`applyOverrides(model, params) → RobotModel`。只改 `joints[].limits` 与（经注入）`reach` |
| `frontend/src/robot/index.ts` | 加一行 export | 不改任何既有导出 |
| `frontend/src/store/robotStore.ts` | **加** params 状态 + 取值函数 | 不改 `moveTo` 的既有分支逻辑 |
| `frontend/src/components/RobotControl/TargetControl.tsx` | **加** UI 区块 | 不改既有输入/拖动逻辑 |
| `frontend/tests/unit/corePackageBoundary.test.ts` | 按需登记白名单 | 契约 |
| **`robot-package/mearm-v1/kinematics/ik.ts`** | **❌ 不动** | 算法 |

### ⚠️ 关于 `reach` 的注入点（待定，二选一）

`reach` 是 `ikGeometry()` 内部算的，**外部无法直接覆写**。两个选项：

**选项 A：把内径覆写"翻译"成杆长覆写**
`reach = [|l1 − l2|, l1 + l2]` ⇒ 给定目标内径 `r_min`，反解 `l2' = l1 − r_min`
（要求 `l1 > r_min`）。这样**完全不碰 `ik.ts`**，但会**改变 `l2`**，
而 `l2` 同时决定 `reachMax` 与 IK 几何 ⇒ **外径也会跟着变**，不符合"只有内径可调"。

**选项 B：给 `ik.ts` 的 `ikGeometry` 加一个可选覆写入口**
需要改 `ik.ts` ⇒ **违反约束 3**。

**选项 C（推荐）：内径覆写只作用于判据，不作用于求解**
即新增一层**前置检查**（在 `moveTo` 里、调 `solveIk` 之前）：
```
若 d < reachMinOverride ⇒ 直接判 OUT_OF_WORKSPACE（不调 solveIk）
```
`reachMinOverride ≥ 0.1` 时该检查**比 `ik.ts` 的原判更严**，
所以**原判据的行为被完全包含**，`ik.ts` 一字不改，
且"只有内径变化、外径不动"**精确成立**。

**✅ 选 C。** 它是唯一同时满足约束 2（只有内径变）与约束 3（不改算法）的方案。

⚠️ 选项 C 的代价：内径检查在 `ik.ts` **之外**，所以：
- `solveIk` 被**直接调用**的地方（sim2sim bridge）**看不到**这个覆写 ⇒ 符合"不影响 sim2sim" ✅
- 前端 UI 路径会看到 ⇒ 符合"在末端目标选项支持修改" ✅

这**正是约束所要求的隔离**。

---

## 五、验收判据（实施后必须全绿）

| # | 判据 | 命令 |
|---|---|---|
| 1 | 默认配置下 `pytest` / vitest 基线**不变** | `pytest -q` = 198；vitest = 401 |
| 2 | 13 条 Phase 6 冻结断言**仍通过**（默认 T=0） | `vitest run tests/acceptance/drag-tracking.test.ts tests/unit/robotStore.target.test.ts` |
| 3 | 启用 T=1.0° 后，钳位误差实测 ≤ 2% | 新增断言（见下） |
| 4 | `reachMin` 下限被拒为 < 0.1 | 新增断言 |
| 5 | `ik.ts` 的 git diff **为空** | `git diff --stat robot-package/mearm-v1/kinematics/ik.ts` = 空 |
| 6 | 包边界白名单通过 | `vitest run tests/unit/corePackageBoundary.test.ts` |

**判据 3 的实测方法**：构造一组"极限点 + 1 位小数舍入"的输入，
断言 `residual`（或 `target` 与 `endEffector` 的差）≤ 2% × 177.091 = 3.542mm。

---

## 六、待用户确认的两个决策点

1. **`T` 的默认值**：`0`（= 完全不影响现状，需手动开启）vs `1.0`（= 直接生效）。
   我倾向 **`0`**，因为约束 3 说"不影响算法"，默认 0 才是字面意义上的"不影响"。

2. **球壳内径覆写的语义**：我按**选项 C**（只收紧内径判据、不改求解）实现。
   若用户本意是"真的要改 2R 求解的可达壳"，那就会改到 `l2` 或改 `ik.ts` ——
   与约束 3 冲突，需要重新确认。

---

## 七、实施结果（已落地并全绿）

> 决策点已确认：**① 只许向内收紧** · **② 选项 C** · **③ `T` 默认 `0`**。
> 以下为实际落地内容与实测数据。

### 7.1 落地文件

| 文件 | 性质 | 说明 |
|---|---|---|
| `frontend/src/robot/model/parameterOverrides.ts` | **新增→重写** | 参数层：类型 / 默认值 / 校验 / 外径覆写透传（**已删除关节限位收紧全套**） |
| `frontend/src/store/robotStore.ts` | 修改 | `targetGuard` 状态 + `setTargetGuard`（**含实时重解**）/ `resetTargetGuard` + `moveTo` 三处前置检查 |
| `frontend/src/components/RobotControl/TargetControl.tsx` | 修改 | 可折叠「安全参数」面板（判据容差 / 内径覆写 / **外径覆写**） |
| `frontend/src/styles.css` | 修改 | `.guard-*` 样式，沿用既有变量，无装饰色 |
| `frontend/src/robot/index.ts` | 修改 | 加一行 `export * from './model/parameterOverrides'` |
| `frontend/tests/unit/targetGuardParams.test.ts` | **新增→重写** | **32 条**验收断言（需求 ①②③④） |
| `robot-package/mearm-v1/kinematics/ik.ts` | **本轮 opt-in 修改** | 新增可选参 `reachMaxMm?`；**缺省时逐位一致**（`git diff` = 47+/5−） |
| `robot-package/mearm-v1/model/robot.yaml` | **未动** | 真值零污染（D55/D71 冻结不受影响） |
| `backend/` · `config/` | **未动** | 已核实**不存在**可调限位参数（见 §7.6） |

### 7.2 `moveTo` 的两处注入点

```
moveTo(xyz)
  │
  ├─ 能力门（NO_SOLVER）           ← 用**原始模型**
  │
  ├─ 参数层①：球壳内径覆写（前置检查）
  │     guard.reachMinOverridden && wristDistance(current.model, xyz) < guard.reachMinMm − 1e-9
  │       ⇒ 直接返回 OUT_OF_WORKSPACE（**不进 solveIk**）
  │     ★ 因 reachMinMm ≥ 0.1 > 原 reachMin = 0 ⇒ 恒比原判据更严
  │       ⇒ 原判据被完全包含，**外径精确不动**
  │
  ├─ 参数层①b：球壳外径覆写（前置检查，与 ① 共用一次 wristDistance）
  │     guard.reachMaxOverridden && wristDistance(current.model, xyz) > guard.reachMaxMm + 1e-9
  │       ⇒ 直接返回 OUT_OF_WORKSPACE（给出"外径覆写生效中"的可读原因）
  │     ★ 这只是**为了好错误信息**；真正让外径参与求解的是下面的 reachMaxMm 透传
  │
  ├─ solveIk(current.model, xyz, { …, reachMaxMm: reachMaxOverrideFor(guard) })
  │     ⚠️ 外径覆写（可选）**直接进 `ik.ts` 参与 `cosAlpha` 求解**，
  │        缺省 undefined ⇒ 用求导外径，与原行为逐位一致
  │
  ├─ success ⇒ clipJointState + set（原路径不变）
  │
  └─ 参数层②：判据容差（T > 0 时才进）
        pickWithinTolerance(result.candidates, T)
          ⇒ 取 violation ≤ T 的解，回算 residual 与 azimuth
        ★ T = 0 时整块不进 ⇒ 默认路径零逻辑变化
```

### 7.3 实测验收（2026-09-16，含外径覆写轮次）

| # | 判据 | 结果 |
|---|---|---|
| 1 | `tsc --noEmit` | **0 错误** |
| 2 | `vitest run`（全量） | **33 文件 · 433 passed** |
| 3 | `pytest -q`（仓库根） | **198 passed** — 与基线一致 |
| 4 | `test_baseline_frozen.py` | **11 passed** |
| 5 | `tests/sim2sim`（基线矩阵） | **13 passed** |
| 6 | `core/tests`（包契约） | 全绿 |
| 7 | `run_sim2sim.py` | FK 前端↔参考 ≤7.110e-14 mm · IK 25/28 · 闭环 ≤1.025e-13 mm，**与基线一致** |
| 8 | `drag-tracking.test.ts`（Phase 6 冻结） | **11 passed** |
| 9 | `robotStore.target.test.ts`（Phase 6 冻结） | **13 passed** |
| 10 | `corePackageBoundary.test.ts`（白名单） | **5 passed** — 未扩边界 |
| 11 | `git diff robot-package/` | 仅 `ik.ts`（47+/5−），`robot.yaml` 零改动 |
| 12 | `vite build` | 成功（CSS 7.77 kB / JS 1358.95 kB） |
| 13 | `go test ./...` | 全绿 |
| 14 | `grep -r 'tighten' frontend/src` | **零命中** — 限位调节彻底移除 |

### 7.6 为什么**没有**关节限位调节（实测论证）

"限位可调"这个方向在本机上**不成立**，不是没做，是做了也无效：

- 关节限位映射到舵机空间后必须落在**舵机硬限位**内（`ACTUATOR_REACH` 校验）。
- 实测本机余量 **0.000000°** —— 现有 `shoulder ∈ [-6.093683, 49.454929]`、
  `elbow ∈ [108.4414852068, 141.8582211436]` 已经**顶满**舵机行程。
- ⇒ 限位**一个度都放不宽**。"调节"只剩"向内收紧"，而向内收紧能做的事
  已由**球壳内径覆写**更直接地覆盖（一个标量 vs 逐关节两个端点）。
- 因此本轮按需求整段移除 `jointTightenDeg` / `tightenLimits()` /
  `applyJointTightening()` / `modelWithTightening()` 等，`ik.ts` 也不再接收收紧模型。
- 后端侧经核实**本就没有**这类可调参数（`backend/config.yaml` 与
  `internal/cfg/cfg.go` 明确"运行配置里绝不允许出现关节限位或标定数值"），
  `grep -rn 'reach\|jointLimit' backend/ config/` 只命中 `protocol.CodeJointLimit`
  （错误码常量，非可调参数）⇒ **无需改动**。

### 7.7 ★ 外径覆写为什么必须开在 `ik.ts` 里面

需求要求外径覆写"**参与计算**"，而外径参与的地方是 2R 的 `cosAlpha`：

```
d² = l1² + l2² − 2·l1·l2·cos(π − θ_elbow)
```

判据处的 `if (d > reachMax)` 只决定"能不能解"，`cosAlpha` 才决定"解出来是多少"。
**在 `ik.ts` 外面拦一道**只能实现"拒绝更早"，不能实现"按新臂展求解"。

所以本轮对 `ik.ts` 做了一次**刻意、最小、向后兼容的 opt-in 改动**：

```ts
export interface IkOptions { /* … */ reachMaxMm?: number }
```

- `reachMaxMm === undefined` ⇒ `effectiveReachMax = geometry.reach[1]`，
  **一个字节的语义都没变** ⇒ sim2sim / sim2real / 2000 组闭环验收逐位一致
  （已实测：`抽象层残差差 0.000e+0`）。
- 传值时 `d ≤ reachMaxMm` 的**解与不覆写完全相同**（同一个 `cosAlpha` 公式）；
  差异只在 `d ∈ (reachMaxMm, 160]` 由"可达"变 `OUT_OF_WORKSPACE`。

### 7.8 ★ 外径覆写的**有效区间**远小于 UI 允许的 1~160

这是本轮实测出、必须钉住的事实（`targetGuardParams.test.ts` 已有断言守护）：

| 量 | 值 | 来源 |
|---|---|---|
| 几何球壳（`ikGeometry().reach`） | `[1.42e-14, 160]` | `l1 = l2 = 80`，`\|l1−l2\| = 0` |
| **合法限位域内实际腕枢轴距离 `d`** | **`[44.1665, 139.2662]`** | 401×401 网格 + FK 双层复核 |
| 极值点（最大） | `shoulder = max, elbow = min` → `d = 139.2662` | 两杆近乎伸直 |
| 极值点（最小） | `shoulder = min, elbow = max` → `d = 44.1665` | 折得最狠 |

⇒ **外径覆写只有落在 `(44.1665, 139.2662]` 内才真正改变可达性**；
填 150 / 160 等于没覆写。UI 因此把 `max` 显示为 `min(160, 求导外径)` 并附注实测区间。

⚠️ 本文件初版曾把"外圈点"写成 `shoulder = 0, elbow = 141` 的点，
但它的 `d` 只有 **19.5mm**（`shoulder = 0` 让手臂折回），离外径十万八千里 ⇒
用它验证"外径收紧拦住外圈"会假性通过。正解是 `shoulder = max, elbow = min`。

⚠️ 另一个坑：求 `radial` 时**不能**用 `wrist.parentLink` 的长度 ——
`tcp.joint = 'tool'` 的 `parentLink` 是 **`forearm_link`(80)** 而非 `tool_link`(40)，
算出的 `d` 会偏小（94.15 vs 真值 131.96）。正确口径是
`radial = geometry.toolOffset[0] = model.tcp.offset` 的径向分量（= 40）。

### 7.9 ★★ 事故：前置检查的 `d` 与 `ik.ts` 的 `d` 曾经不是一个量

**用户报告**：「为什么本身可运行范围，我把外径覆写从160改到60，反而OUT_OF_SPACE了」

**根因**：`robotStore.ts` 的 `wristDistance()` 自己重算 `d`，其中
`radial = linkLengthOf(model, wrist.parentLink)` —— `wrist` 是 `model.tcp.joint`（`'tool'`），
它的 `parentLink` 是 **`forearm_link`(80)**，而 `ik.ts` 的口径是 `toolOffset[0] = 40`。

| 来源 | `radial` | `d`（同一点） |
|---|---|---|
| `ik.ts`（正确） | `toolOffset[0]` = 40 | **111.542** |
| `wristDistance()`（错） | `forearm_link` = 80 | **92.159** |

差 **19.38mm**，**不报错**。错误 `d` 系统性偏小 19~40mm，于是：

- **内圈一大片实际不可达的点被误报为可达** ⇒ 用户看到的"可运行范围"是虚的
- 那个点在外径 160 下**本来就失败**（`JOINT_LIMIT @ elbow`），降到 60 只是换了原因
  ⇒ **报错是"碰巧对"的**：判定结果对，依据的数是错的
- 两层判据口径不一致 ⇒ 拒绝理由随机（有时前置检查、有时 `ik.ts`）

**修复**：删除自研几何，一律转发 `ik.ts` 的同一份真值：

```ts
function wristDistance(model: RobotModel, target: Vec3): number {
  const geometry = ikGeometry(model);
  const { dr, dz } = wristSagittal(geometry, target);
  return Math.hypot(dr, dz);
}
function reachShellOf(model: RobotModel): readonly [number, number] {
  return ikGeometry(model).reach;
}
```

`reachShellOf()` 原本也从连杆表自己算（本机数值恰好一致，但**不是同一份真值**：
`ikGeometry()` 是从 FK 采样反推），一并改为转发。
代价是扩了两条 `Core→包` 的 import（`ikGeometry` / `wristSagittal`），
**已登记进 `corePackageBoundary.test.ts` 的 `ALLOWED` 白名单**（该白名单双向断言，
多取少取都会失败）。

**回归断言**：`targetGuardParams.test.ts` 新增
`★★ 前置检查的 d 必须与 ik.ts 同口径` —— 用极小外径强制前置检查触发，
从错误消息抠出 store 算的 `d` 与独立复算比对，**15 个采样点**全覆盖。

**★ 教训**：本项目铁律「真值只有一份」不是洁癖 —— 这个 bug 正是以
**轻微（19mm）、随机（哪层先拒不定）、不报错**的方式咬人的。
凡"为了少 import 而自己重算真值"的写法都是温床；`ik.ts` 里专门有
`requireConstantToolOffset()` 用三个姿态交叉验证该常量，恰恰说明它本就容易算错。

### 7.4 容差与硬误差（需求①的核心数字）

最坏力臂 `L = l1 + l2 = 160mm`，钳位误差 `≈ L·sin(T)`，基准取实测最大可达半径 `177.091mm`：

| `T` | 最坏位移 | 占基准 | 判定 |
|---|---|---|---|
| `0.5°` | 1.396 mm | **0.789%** | ✅ |
| `1.0°` | 2.793 mm | **1.577%** | ✅ ← **上限，取此值** |
| `2.0°` | 5.585 mm | **3.154%** | ❌ 超标 |

⇒ `JOINT_TOLERANCE_MAX_DEG = 1.0`，UI 输入框 `max = 1`。

### 7.5 ★ 一个认知修正（实施中发现的）

**"限位可调"解决不了"手输被拒"。**

原因：本机 `elbow` 限位 `[108.4414852068, 141.8582211436]` 恰好等于
`servo_8` 硬限位 `20..100` 经标定（`reverse=true, scale=2.39401, offset=359.61`）
反算的值 ⇒ **放宽余量 = 0.000000°**。`shoulder` 同理（`servo_7` 硬限位 `80..160`
→ 关节 `[-6.093683, 49.454929]`）。任何放宽都会让 `ACTUATOR_REACH` 校验失败。

反过来，**收紧**也只是把悬崖搬个位置 —— 新极限点回输仍会被 `EPS_DEG = 1e-9` 拒。
真正起作用的是**判据容差 `T`**。所以：

- 「限位可调」= 可选的**保守化**手段（给怕撞限位的人用），**不是**修复手段；
- 「判据容差」= 真正修复"极限值回输超范围"的手段。

UI 上因此对每个关节显示"已顶死舵机硬限位，无可放宽余量"的提示，避免误以为能放大。

### 7.6 测试设计上的一个坑（供后续参考）

`atElbowMax()` 那类"把关节推到限位上限、再按 1 位小数舍入"的构造
**不保证**产生舍入误差 —— `base = 0, shoulder = 0` 时该点可能恰好落在
0.1mm 整格上，默认参数就能接受，测试前提不成立（初版就是这么失败的）。

改为**穷举实际可达的极限姿态**、取其中"精确角贴限位但舍入后回输被拒"的点，
才拿到稳定可复现的用例（用户截图那组正是此类：回输后 `elbow` 解出
`141.862303 > 141.8582211436`）。

