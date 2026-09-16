/**
 * IK（逆运动学）—— 末端 **XYZ 位置** → `base / shoulder / elbow` 三个定位关节角。
 *
 * ## 为什么可以解析求解
 *
 * 本机是「1 个绕 Z 的偏航关节 + 1 组在竖直平面内摆动的 2R 连杆」，
 * 因此可以精确解耦成两步（几何关系见 `docs/coordinate-system.md` §3.1）：
 *
 * ```
 *   J1 = atan2(y, x)          ← 偏航，与高度无关，把三维问题降成矢状面内的二维
 *   r  = hypot(x, y)          ← 矢状面内的水平半径
 *
 *   ┌─ ① 先扣掉「腕枢轴 → TCP」的常量偏移 ────────────────────────┐
 *   │  r_w = r − pivotR − offset.径向                            │
 *   │  z_w = z − pivotZ − offset.竖直                            │
 *   └────────────────────────────────────────────────────────────┘
 *   ┌─ ② 矢状面内 2R（肩枢轴 → 肘枢轴 → **腕枢轴**）──────────────┐
 *   │  L1 = 肩枢轴 → 肘枢轴         （大臂）                      │
 *   │  L2 = 肘枢轴 → **腕枢轴**     （小臂）  ⚠️ 不是「肘 → TCP」 │
 *   │  dr = r_w − 枢轴水平半径,  dz = z_w − 枢轴高度              │
 *   │  D  = hypot(dr, dz)                                        │
 *   │  α  = ±acos((D² − L1² − L2²) / (2·L1·L2))   ← 相对肘角      │
 *   │  φ  = atan2(dr, dz)                          ← 0 = 天顶     │
 *   │  θs = φ − atan2(L2·sinα, L1 + L2·cosα)                     │
 *   │  θe = θs + α                                               │
 *   └────────────────────────────────────────────────────────────┘
 * ```
 *
 * **所有几何量（L1 / L2 / 枢轴 / 腕→TCP 偏移）都在运行时从 RobotModel 求导，
 * 且逐个跨姿态交叉验证，不写死常数** —— 改 `config/robot.yaml` 的 `length` 即改机构，
 * 本文件无需改动；改错则**显式报错**而不是静默解偏。
 *
 * ## ⚠️ 本机最特殊的两点
 *
 * ### ① `elbow` 是**绝对角**
 *
 * 真机小臂由独立舵机经**平行四连杆**驱动，其绝对倾角与肩角解耦
 * （实测：肩转 52.91° 时小臂绝对角只漂 10.08°，见 `docs/hardware-measurement.md`）。
 * 因此 `JointState.elbow` 存的是**离开天顶的绝对倾角**，而不是相对大臂的夹角。
 *
 * 反直觉的结论：**IK 解出的 `θe` 就是最终要写进 JointState 的值，不需要再叠加肩角**。
 * 串联网里的局部旋转由 FK 的 `effectiveJointAngle()` 负责（`relative = θe + (−1)·θs`）。
 * 若按常见 meArm 写法"再叠加一次肩角"，机构会立刻错位 —— 见 `docs/decisions.md` D18。
 *
 * ### ② 腕（`tool`）是**被动关节**，爪被连杆锁成水平
 *
 * 实测（2026-09-13 受控实验：定机位 S8 扫描 + 爪指轴线画回原图复核）：小臂绝对倾角
 * 变 29.24° 时爪指的画面倾角只变 7.31° ⇒ 折角反向补偿 ⇒ **爪近似恒水平**。
 * 配置里 `tool` 是 `passive`、`coupling{gain:-1}` 到 `elbow`、锁定值 90°。
 *
 * 这对 IK 的**直接后果**：`elbow → TCP` **不再是**一条固定长的直线段
 * （它随 θe 在 109~120mm 之间变，实测），所以那一整段不能再进 2R。
 * 真正进 2R 的是「肘枢轴 → 腕枢轴」= `forearm_link.length`；
 * 而「腕枢轴 → TCP」退化为一个**常量矢量**（本机 = 径向 +40mm，纯水平），
 * 在解算前从目标点里减掉即可。
 *
 * 这个"偏移是常量"的前提由 `requireConstantToolOffset()` 在**三个不同姿态**上数值验证 ——
 * 谁要是哪天把腕改回刚性固连，那一步会当场抛 `IkModelError`，而不是让所有解
 * 系统性偏掉 40mm（这种错**不会自己暴露**：残差自查同样是偏的）。
 *
 * ## 约束
 *
 * - IK **只解定位三关节**，绝不碰夹爪，更不知道舵机 / PWM / 标定
 *   （`coordinate-system.md` §4 明文规定：标定绝不进入 IK）。
 * - 被动关节（`tool`）**不参与解算**：它没有输入，角度由 coupling 派生。
 * - 夹爪角若传入 `seed` 则原样透传，否则取 `homePose`，保证返回值可直接喂给 FK 闭环验收。
 */
import type { Joint } from '@robot/model/Joint';
import type { JointState, Vec3 } from '@robot/model/Pose';
import { degToRad, radToDeg } from '@robot/model/Pose';
import type { RobotModel } from '@robot/model/RobotModel';
import {
  homeJointState,
  jointById,
  jointByRole,
  jointDepth,
  movableJoints,
} from '@robot/model/RobotModel';
import { endEffectorPosition, jointMatrices } from '@robot/kinematics/fk';
import { mat4GetPosition } from '@robot/kinematics/transform';

/** 求解失败的原因 */
export type IkReason =
  /** 目标点超出连杆总长能触及的球壳范围（几何不可达） */
  | 'OUT_OF_WORKSPACE'
  /** 几何可达，但所需关节角超出该关节限位（真机做不出来） */
  | 'JOINT_LIMIT'
  /**
   * **本机器人没有逆解器** —— 求解根本没有被尝试。
   *
   * ⚠️ 它与上面两条是**不同性质的失败**：那两条是"试过、算出来了、不行"，
   * 这一条是"没算"。调用方据它区分「目标不可达」和「这台机器人压根不支持 IK」，
   * 否则用户会以为是工作空间问题而反复调整目标点。
   *
   * 判据是 `KinematicsEngine.capability.solverKind`（数据），不是 `if robot === …`（逻辑）。
   * 目前只有 SO-ARM101 会返回它（`solverKind: 'none'`）。
   */
  | 'NO_SOLVER';

/**
 * 解的分支。
 *
 * 平面 2R 有两组解，由相对肘角 `α = θe − θs` 的符号区分：
 * - `elbow-up`：`α > 0`，肘部高于「肩→目标」连线 —— **真机 HOME 位所在支**
 * - `elbow-down`：`α < 0`，肘部低于该连线（小臂向回折）
 */
export type IkBranch = 'elbow-up' | 'elbow-down';

/** 多解选取策略 */
export type IkPreference = IkBranch | 'nearest';

/** 单个候选解（含被限位否决的那些，便于调试与可视化） */
export interface IkCandidate {
  branch: IkBranch;
  /** `α = θe − θs`（degree） */
  relativeAngle: number;
  /** 相对肘角绝对值（degree） */
  alphaAbsolute: number;
  /** 该支解（base/shoulder/elbow 三元组，degree） */
  joints: JointState;
  /** 是否全部落在限位内 */
  feasible: boolean;
  /** 不可行时，越界最严重的关节 id */
  violatedJoint?: string;
  /** 越界量（degree，0 表示刚好在界上） */
  violation: number;
}

export interface IkOptions {
  /**
   * 多解选取策略，默认 `'nearest'`（需要 `near`；缺失时自动退化为 `'elbow-up'`）。
   *
   * `nearest` 是为「鼠标拖动末端」准备的：末端在工作空间内边界附近移动时，
   * 两支解会互相翻越，固定选支会让机械臂突然翻肘；按「离当前位姿最近」选支可消除跳变。
   */
  prefer?: IkPreference;
  /** 当前关节状态；`prefer: 'nearest'` 用它做判据，也用作未解关节的取值来源 */
  near?: JointState;
  /** 未参与解算的关节（夹爪）取值来源；缺省取 `homePose` */
  seed?: JointState;
  /** 残差阈值（mm）；超出即视为实现缺陷而非目标不可达，默认 1e-9 */
  tolerance?: number;
  /**
   * 球壳**外径**覆写（mm）。缺省 `undefined` ⇒ 用 `ikGeometry()` 从杆长求导出的
   * `reach[1]`（= `l1 + l2`，本机 160）—— **与原行为逐位一致**。
   *
   * ## 为什么这个口子必须开在 `ik.ts` 里面
   *
   * 球壳外径与外径内径不同：外径参与 `cosAlpha = (d² − l1² − l2²)/(2·l1·l2)`
   * 的求解。若只在 `solveIk` 之外拦一道 `d > reachMaxMm`，那么用户把外径
   * 设成 100mm 时，2R 仍按 160mm 的臂展算出的角度 —— 那是"另一台机器人"的
   * 姿态，不是"缩到 100mm 臂展"的姿态。⇒ 必须真正替换掉求解用的外径。
   *
   * ## 为什么这个改动是安全的（不破坏 sim2sim / 既有验收）
   *
   * · 缺省 `undefined` ⇒ 下面的代码路径**一个字节都没变**，`reach[1]` 仍是
   *   求导值 ⇒ 所有既有调用（sim2sim bridge / sim2real / 2000 组闭环验收）
   *   逐位一致。
   * · 传入值时，只要目标落在覆写外径**以内**（`d ≤ reachMaxMm`），
   *   `cosAlpha` 与不覆写时**完全相同**（同一个公式、同一组 l1/l2/d），
   *   只是 `clamp1` 多一次无操作的上界收窄 ⇒ 解与残差逐位相同。
   * · 差异**只在判据边缘**：`d ∈ (reachMaxMm, 原 reachMax]` 由"可达"变为
   *   `OUT_OF_WORKSPACE`。这正是"外径覆写参与计算"的字面含义。
   *
   * ⚠️ 覆写值不在此处校验（`ik.ts` 不认识"参数"这个概念）。合法性由调用方
   * 保证：见 `frontend/src/robot/model/parameterOverrides.ts` 的
   * `REACH_MAX_FLOOR_MM` / `REACH_MAX_CEILING_MM`。
   */
  reachMaxMm?: number;
}

export interface IkSuccess {
  success: true;
  /** 完整 JointState（含夹爪），可直接喂给 `forwardKinematics` */
  joints: JointState;
  branch: IkBranch;
  /** 解算后用 FK 自查的末端位置残差（mm） */
  residual: number;
  /** 实际采用的底座方位角（degree）；`r ≈ 0` 时该值由 `near` / 限位决定 */
  azimuth: number;
  /** `α = θe − θs`（degree） */
  relativeAngle: number;
}

export interface IkFailure {
  success: false;
  reason: IkReason;
  /** `JOINT_LIMIT` 时指出越界的关节 id */
  joint?: string;
  /** 人类可读说明，格式对齐 `docs/serial-v1.md` 的 `ERR JOINT ...` */
  message: string;
  /** 已算出的候选（`OUT_OF_WORKSPACE` 时为空数组） */
  candidates: IkCandidate[];
}

export type IkResult = IkSuccess | IkFailure;

/**
 * 矢状面 2R 的几何量 —— 全部由模型求导并跨姿态验证。
 *
 * 求导方式：在**多个合法姿态**上跑 FK（不是只跑一次"全零位形"），
 * 因为本机的被动腕让"全零位形"不再是一条竖直直线：
 * 被动关节没有输入、值恒为锁定角（90°），所以零位下爪就已经是水平的。
 */
export interface IkGeometry {
  baseId: string;
  shoulderId: string;
  elbowId: string;
  /** TCP 参考关节（本机 = 被动腕 `tool`）；其**坐标系原点**即 2R 子链的末端「腕枢轴」 */
  wristId: string;
  /** 肩枢轴高度（mm，底座平面之上） */
  pivotZ: number;
  /** 肩枢轴水平半径（mm，位于偏航轴上，正常为 0） */
  pivotR: number;
  /** 大臂等效长（肩枢轴 → 肘枢轴，mm） */
  l1: number;
  /** 小臂等效长（肘枢轴 → **腕枢轴**，mm）。⚠️ **不含**腕→TCP 那一段 */
  l2: number;
  /**
   * 腕枢轴 → TCP 的**常量**矢状面偏移（mm）：`[径向, 竖直]`。
   *
   * 本机被动腕把爪锁成水平 ⇒ `[40, 0]`。解算前必须从目标点里减掉它。
   */
  toolOffset: readonly [number, number];
  /**
   * 2R 子链关于肩枢轴的可达距离壳 `[|l1−l2|, l1+l2]`（mm）。
   *
   * ⚠️ 它约束的是**减去 `toolOffset` 之后**的腕目标点，不是原始目标点。
   * 判"是否越界"请用 `wristSagittal()` 换算，别直接拿原始点算距离。
   */
  reach: readonly [number, number];
}

/** 模型形状与平面 2R 假设不符时抛出（属配置错误，不是目标不可达） */
export class IkModelError extends Error {
  constructor(message: string) {
    super(`[ik] ${message}`);
    this.name = 'IkModelError';
  }
}

const EPS_DEG = 1e-9;
const EPS_MM = 1e-9;

/**
 * 跨姿态一致性容差（mm）。
 *
 * 纯旋转 / 平移运算下，同一几何量在不同姿态的求导值应逐位相同（实测差 ~1e-13）；
 * 这里留到 1e-6mm —— 仍是"微米级的千分之一"，任何真实的机构错误
 * （腕没锁住、轴不共面、offset 写错维度）都会冲到毫米以上。
 */
const PROBE_TOL_MM = 1e-6;

/** 采样姿态数（≥3：两个端点 + 一个中间点，避免"只在边界恰好相等"的巧合） */
const PROBE_COUNT_MIN = 3;

function axisCloseTo(axis: Vec3, expected: readonly [number, number, number], tol = 1e-6): boolean {
  return (
    Math.abs(axis[0] - expected[0]) < tol &&
    Math.abs(axis[1] - expected[1]) < tol &&
    Math.abs(axis[2] - expected[2]) < tol
  );
}

function requireRoleJoint(model: RobotModel, role: 'base' | 'shoulder' | 'elbow'): Joint {
  const joint = jointByRole(model, role);
  if (!joint) throw new IkModelError(`模型缺少 role=${role} 的关节，无法做位置 IK`);
  return joint;
}

/**
 * 校验「解析式 2R」的前提在模型里成立。
 *
 * 解析解假定：底座绕 Z 偏航、肩与肘共用一根平行于 Y 的俯仰轴、且两关节无固定朝向偏转。
 * 一旦有人改了 `joints[].axis` / `origin.rotation`，解析式会**静默失效**（不报错但解错），
 * 所以这里主动拦下来，把"改配置后的隐形错误"变成一条明确报错。
 */
function assertPlanar2R(base: Joint, shoulder: Joint, elbow: Joint): void {
  if (base.type !== 'revolute' || shoulder.type !== 'revolute' || elbow.type !== 'revolute') {
    throw new IkModelError('base / shoulder / elbow 必须都是 revolute 关节');
  }
  if (!axisCloseTo(base.axis, [0, 0, 1])) {
    throw new IkModelError(
      `base.axis = [${base.axis.join(', ')}]，解析式 IK 要求 [0, 0, 1]（绕 Z 偏航）。` +
        '若确需改轴，请同步改写 ik.ts 的解算方式。',
    );
  }
  if (!axisCloseTo(shoulder.axis, [0, 1, 0]) || !axisCloseTo(elbow.axis, [0, 1, 0])) {
    throw new IkModelError(
      `shoulder.axis / elbow.axis 必须同为 [0, 1, 0]（共面俯仰轴），` +
        `当前为 [${shoulder.axis.join(', ')}] / [${elbow.axis.join(', ')}]`,
    );
  }
  for (const joint of [base, shoulder, elbow]) {
    const [rx, ry, rz] = joint.origin.rotation;
    if (Math.abs(rx) > 1e-6 || Math.abs(ry) > 1e-6 || Math.abs(rz) > 1e-6) {
      throw new IkModelError(
        `${joint.id}.origin.rotation = [${rx}, ${ry}, ${rz}]，解析式 IK 要求无固定朝向偏转（[0, 0, 0]）`,
      );
    }
  }
}

/**
 * 校验 TCP 参考关节确实是一个「可被锁定成常量偏移的腕」。
 *
 * 反面例子（也是本文件存在的理由）：`tool` 若还是 `fixed`，爪就刚性固连在小臂上，
 * 「腕 → TCP」会随 θe 一起转 —— 那时把 `elbow → TCP` 当成一根定长杆，
 * IK 会给出一个**看起来很正常、却系统性偏掉几十毫米**的解。
 */
function assertPlanarWrist(model: RobotModel, elbow: Joint, wrist: Joint): void {
  if (wrist.type === 'fixed') {
    throw new IkModelError(
      `tcp.joint="${wrist.id}" 是固定关节 ⇒ TCP 随小臂刚性转动，「腕→TCP」不是常量偏移，` +
        '矢状面 2R 不成立。若爪确实随小臂转，请把 tcp.joint 指到小臂末端的关节上，' +
        '并让 IK 把该段计入 L2；若爪被连杆锁住（本机情形），' +
        '请把该关节标为 passive 并声明 coupling。',
    );
  }
  if (!axisCloseTo(wrist.axis, [0, 1, 0])) {
    throw new IkModelError(
      `tcp.joint="${wrist.id}" 的 axis = [${wrist.axis.join(', ')}]，解析式 IK 要求 [0, 1, 0]`,
    );
  }
  const [rx, ry, rz] = wrist.origin.rotation;
  if (Math.abs(rx) > 1e-6 || Math.abs(ry) > 1e-6 || Math.abs(rz) > 1e-6) {
    throw new IkModelError(
      `tcp.joint="${wrist.id}".origin.rotation = [${rx}, ${ry}, ${rz}]，` +
        '解析式 IK 要求无固定朝向偏转（[0, 0, 0]）',
    );
  }
  const depthElbow = jointDepth(model, elbow.id);
  const depthWrist = jointDepth(model, wrist.id);
  if (!(depthWrist > depthElbow)) {
    throw new IkModelError(
      `tcp.joint="${wrist.id}"（链上深度 ${depthWrist}）必须排在 elbow（深度 ${depthElbow}）之后`,
    );
  }
}

/** 一个采样姿态下的四个枢轴世界坐标（mm） */
interface SagittalProbe {
  pShoulder: Vec3;
  pElbow: Vec3;
  /** 腕枢轴 = TCP 参考关节的坐标系原点 */
  pWrist: Vec3;
  pTcp: Vec3;
}

/**
 * 取一个「每条关节都显式落值」的基准状态。
 *
 * ⚠️ 被动关节没有输入，值恒为锁定角（= `limits.min`）—— 这里**显式写出来**，
 * 而不是依赖 FK 的缺省回退：那是隐式约定，改一个字就会静默错位。
 */
function neutralJointState(model: RobotModel, baseId: string): JointState {
  const state: JointState = {};
  for (const joint of model.joints) {
    if (joint.type === 'fixed') continue;
    state[joint.id] = joint.limits.min;
  }
  // 偏航归零：使肩枢轴落在 XZ 平面内，"径向"就是 +X
  state[baseId] = 0;
  return state;
}

/**
 * 在**多个合法姿态**上求导矢状面几何量。
 *
 * 姿态取「肩 / 肘限位的两端点 + 中点」，全部落在真机限位内（合法位形）。
 * 单点求导无法区分"常量"与"恰好在该点相等"，所以至少要 3 个点。
 */
function sagittalProbes(
  model: RobotModel,
  base: Joint,
  shoulder: Joint,
  elbow: Joint,
  wristId: string,
): SagittalProbe[] {
  const pairs: ReadonlyArray<readonly [number, number]> = [
    [shoulder.limits.min, elbow.limits.min],
    [shoulder.limits.max, elbow.limits.max],
    [
      (shoulder.limits.min + shoulder.limits.max) / 2,
      (elbow.limits.min + elbow.limits.max) / 2,
    ],
  ];
  if (pairs.length < PROBE_COUNT_MIN) {
    throw new IkModelError(`采样姿态只有 ${pairs.length} 个，不足以验证"常量"（需要 ≥3）`);
  }

  return pairs.map(([ts, te], i) => {
    const state = neutralJointState(model, base.id);
    state[shoulder.id] = ts;
    state[elbow.id] = te;

    const matrices = jointMatrices(model, state);
    const mShoulder = matrices.get(shoulder.id);
    const mElbow = matrices.get(elbow.id);
    const mWrist = matrices.get(wristId);
    if (!mShoulder || !mElbow || !mWrist) {
      throw new IkModelError(
        `FK 未产出 shoulder / elbow / ${wristId} 的坐标系（采样姿态 #${i}），无法求导几何量`,
      );
    }
    return {
      pShoulder: mat4GetPosition(mShoulder),
      pElbow: mat4GetPosition(mElbow),
      pWrist: mat4GetPosition(mWrist),
      pTcp: endEffectorPosition(model, state),
    };
  });
}

function assertCloseMm(got: number, want: number, label: string): void {
  if (Math.abs(got - want) > PROBE_TOL_MM) {
    throw new IkModelError(
      `${label} 不一致：${got} vs ${want}（差 ${(got - want).toExponential(3)}mm）。` +
        '「矢状面 2R」要求这些量只由 links[].length 决定，与姿态无关。',
    );
  }
}

/**
 * 求「腕枢轴 → TCP」的**常量**矢状面偏移 `[径向, 竖直]`（mm）。
 *
 * ⚠️ 这不是"读一个配置值"，而是**跨姿态的数值交叉验证**：
 * 同一个偏移必须在全部采样姿态下逐位一致，且横向分量为 0。
 * 任何一条不满足都直接抛错 —— 因为那种情况下 2R 的解会系统性偏掉
 * 最多 `|offset|` 那么多（本机 40mm），而且**不会自己暴露**：FK 自查同样是偏的。
 */
function requireConstantToolOffset(probes: readonly SagittalProbe[]): readonly [number, number] {
  const radialOf = (p: SagittalProbe): number => p.pTcp[0] - p.pWrist[0];
  const verticalOf = (p: SagittalProbe): number => p.pTcp[2] - p.pWrist[2];
  const lateralOf = (p: SagittalProbe): number => p.pTcp[1] - p.pWrist[1];

  const ref = probes[0]!;
  const refOffset: readonly [number, number] = [radialOf(ref), verticalOf(ref)];

  for (let i = 0; i < probes.length; i += 1) {
    const p = probes[i]!;
    const lateral = lateralOf(p);
    if (Math.abs(lateral) > PROBE_TOL_MM) {
      throw new IkModelError(
        `腕枢轴 → TCP 的偏移有 ${lateral.toFixed(4)}mm 落在矢状面之外（采样姿态 #${i}）：` +
          '2R 只能解矢状面内的目标。请检查 tcp.offset 与 tcp.joint 的 axis。',
      );
    }
    const radial = radialOf(p);
    const vertical = verticalOf(p);
    if (
      Math.abs(radial - refOffset[0]) > PROBE_TOL_MM ||
      Math.abs(vertical - refOffset[1]) > PROBE_TOL_MM
    ) {
      throw new IkModelError(
        `腕枢轴 → TCP 的偏移随姿态变化（采样姿态 #${i}：[${radial.toFixed(4)}, ${vertical.toFixed(4)}]，` +
          `#0：[${refOffset[0].toFixed(4)}, ${refOffset[1].toFixed(4)}]）⇒ 平面 2R 前提不成立。` +
          '通常是 tcp.joint 指向的腕关节没有被连杆锁住：' +
          // ⚠️ 这条提示会随生产包发出去，**不许写死路径** —— Phase 2 把真值搬进了
          //    `robot-package/<id>/model/robot.yaml`，写死的 `config/robot.yaml`
          //    会让读到这条错误的人去找一个不存在的文件。
          '本机应由 passive 关节 + coupling 表达（见该机器人 robot.yaml 的 joints.tool）。',
      );
    }
  }
  return refOffset;
}

/** 从模型求导矢状面 2R 的几何量（带缓存） */
const geometryCache = new WeakMap<RobotModel, IkGeometry>();

export function ikGeometry(model: RobotModel): IkGeometry {
  const cached = geometryCache.get(model);
  if (cached) return cached;

  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');
  const wrist = jointById(model, model.tcp.joint);
  if (!wrist) {
    throw new IkModelError(`tcp.joint="${model.tcp.joint}" 不存在，无法求导几何量`);
  }
  assertPlanar2R(base, shoulder, elbow);
  assertPlanarWrist(model, elbow, wrist);

  const probes = sagittalProbes(model, base, shoulder, elbow, wrist.id);
  const first = probes[0]!;

  const pivotZ = first.pShoulder[2];
  const pivotR = Math.hypot(first.pShoulder[0], first.pShoulder[1]);
  const l1 = dist(first.pShoulder, first.pElbow);
  const l2 = dist(first.pElbow, first.pWrist);

  if (!(l1 > EPS_MM) || !(l2 > EPS_MM)) {
    throw new IkModelError(
      `求导出的连杆长度非法（l1=${l1}, l2=${l2}）；请检查 links[].length`,
    );
  }

  // 杆长与枢轴高度必须与姿态无关 —— 这就是"矢状面 2R"这个前提本身
  for (let i = 0; i < probes.length; i += 1) {
    const p = probes[i]!;
    assertCloseMm(dist(p.pShoulder, p.pElbow), l1, `l1（采样姿态 #${i} vs #0）`);
    assertCloseMm(dist(p.pElbow, p.pWrist), l2, `l2（采样姿态 #${i} vs #0）`);
    assertCloseMm(p.pShoulder[2], pivotZ, `肩枢轴高度（采样姿态 #${i} vs #0）`);
  }

  const toolOffset = requireConstantToolOffset(probes);

  const geometry: IkGeometry = {
    baseId: base.id,
    shoulderId: shoulder.id,
    elbowId: elbow.id,
    wristId: wrist.id,
    pivotZ,
    pivotR,
    l1,
    l2,
    toolOffset,
    reach: [Math.abs(l1 - l2), l1 + l2],
  };
  geometryCache.set(model, geometry);
  return geometry;
}

function dist(a: Vec3, b: Vec3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/**
 * 目标点 → 矢状面内**腕枢轴**相对肩枢轴的偏移 `[dr, dz]`（mm）。
 *
 * 这是「先扣偏移再做 2R」的**单点**换算。IK 内部与验收程序都走它，
 * 少扣一次就会让整条解系统性偏 `toolOffset` 那么多（本机 40mm）而且不报错。
 */
export function wristSagittal(geometry: IkGeometry, target: Vec3): { dr: number; dz: number } {
  const [x, y, z] = target;
  const r = Math.hypot(x, y);
  return {
    dr: r - geometry.pivotR - geometry.toolOffset[0],
    dz: z - geometry.pivotZ - geometry.toolOffset[1],
  };
}

function limitViolation(joint: Joint, value: number): number {
  return Math.max(0, joint.limits.min - value, value - joint.limits.max);
}

/**
 * 求解全部候选解（不做多解选取）。
 *
 * 返回的候选**包含被限位否决的支**，便于 UI/调试展示「另一支差多少度」；
 * 调用方若只想拿可行解，过滤 `feasible === true` 即可。
 */
export function solveIkCandidates(
  model: RobotModel,
  target: Vec3,
  reachMaxMm?: number,
): {
  reason?: IkReason;
  candidates: IkCandidate[];
  azimuth: number;
  azimuthIndeterminate: boolean;
} {
  const geometry = ikGeometry(model);
  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');
  const { l1, l2 } = geometry;

  const [x, y] = target;
  const r = Math.hypot(x, y);

  // ---- ① 偏航 -------------------------------------------------------------
  // r ≈ 0 时目标落在偏航轴上，方位角数学上不定（无唯一解），交由上层按 nearest / 限位决定。
  const azimuthIndeterminate = r < 1e-9;
  const azimuthRaw = azimuthIndeterminate ? 0 : radToDeg(Math.atan2(y, x));

  // ---- ② 腕枢轴目标点：先扣掉「腕 → TCP」的常量偏移 -------------------------
  // ⚠️ 这一步是「爪被锁成水平」的直接后果：2R 解的是**腕**，不是 TCP。
  //    漏掉它 = 所有解系统性偏 toolOffset（本机 40mm）。
  const { dr, dz } = wristSagittal(geometry, target);
  const d = Math.hypot(dr, dz);

  // ⚠️ 外径覆写（`opts.reachMaxMm`）：缺省时 `effectiveReachMax` **恒等于**求导外径
  //    ⇒ 与原行为逐位一致。传入合法值（≤ 求导外径）时它替换掉求解判据的外径，
  //    于是 2R 的 `cosAlpha` 按新臂展求解 —— 这是"外径覆写真正参与计算"的落点。
  //    详见 `IkOptions.reachMaxMm` 的说明。
  const reachMin = geometry.reach[0];
  const reachMax = reachMaxMm ?? geometry.reach[1];
  if (d > reachMax + EPS_MM || d < reachMin - EPS_MM) {
    return {
      reason: 'OUT_OF_WORKSPACE',
      candidates: [],
      azimuth: azimuthRaw,
      azimuthIndeterminate,
    };
  }

  const clampedD = Math.min(Math.max(d, reachMin), reachMax);
  const cosAlpha = clamp1((clampedD * clampedD - l1 * l1 - l2 * l2) / (2 * l1 * l2));
  const alphaDeg = radToDeg(Math.acos(cosAlpha));
  const phiDeg = radToDeg(Math.atan2(dr, dz));

  const candidates: IkCandidate[] = [];
  for (const sign of [1, -1] as const) {
    const relativeAngle = sign * alphaDeg;
    const ths = phiDeg - radToDeg(Math.atan2(l2 * Math.sin(degToRad(relativeAngle)), l1 + l2 * Math.cos(degToRad(relativeAngle))));
    // ⚠️ elbow 存绝对角 ⇒ 直接 = ths + 相对角，不要再叠加肩角（见文件头说明）
    const the = ths + relativeAngle;

    const joints: JointState = { [base.id]: azimuthRaw, [shoulder.id]: ths, [elbow.id]: the };
    const vS = limitViolation(shoulder, ths);
    const vE = limitViolation(elbow, the);
    const violation = Math.max(vS, vE);
    const violatedJoint = violation <= EPS_DEG ? undefined : vE >= vS ? elbow.id : shoulder.id;

    candidates.push({
      branch: sign > 0 ? 'elbow-up' : 'elbow-down',
      relativeAngle,
      alphaAbsolute: alphaDeg,
      joints,
      feasible: violation <= EPS_DEG,
      ...(violatedJoint ? { violatedJoint } : {}),
      violation,
    });
  }
  return { candidates, azimuth: azimuthRaw, azimuthIndeterminate };
}

function clamp1(v: number): number {
  return v < -1 ? -1 : v > 1 ? 1 : v;
}

/** 把候选补成完整 JointState（夹爪等未解关节来自 seed / homePose） */
function completeState(model: RobotModel, partial: JointState, opts: IkOptions): JointState {
  const source = opts.seed ?? opts.near;
  const home = homeJointState(model);
  const out: JointState = {};
  for (const joint of movableJoints(model)) {
    const value = partial[joint.id] ?? source?.[joint.id] ?? home[joint.id] ?? joint.limits.min;
    out[joint.id] = clampToLimits(joint, value);
  }
  return out;
}

function clampToLimits(joint: Joint, value: number): number {
  return value < joint.limits.min ? joint.limits.min : value > joint.limits.max ? joint.limits.max : value;
}

/** 位置目标，供日志 / 错误信息使用 */
function fmt(p: Vec3): string {
  return `(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)})`;
}

/**
 * 逆运动学：末端位置 → 定位三关节角（`base / shoulder / elbow`）。
 *
 * ```
 *   const r = solveIk(model, [120, 0, 90]);
 *   if (r.success) apply(r.joints);      // r.joints 含夹爪，可直接喂 FK
 *   else console.warn(r.reason, r.message);
 * ```
 */
export function solveIk(model: RobotModel, target: Vec3, opts: IkOptions = {}): IkResult {
  const geometry = ikGeometry(model);
  const base = requireRoleJoint(model, 'base');
  const shoulder = requireRoleJoint(model, 'shoulder');
  const elbow = requireRoleJoint(model, 'elbow');

  const { candidates, reason, azimuth, azimuthIndeterminate } = solveIkCandidates(
    model,
    target,
    opts.reachMaxMm,
  );

  if (reason === 'OUT_OF_WORKSPACE') {
    const reachMin = geometry.reach[0];
    const reachMax = opts.reachMaxMm ?? geometry.reach[1];
    const { dr, dz } = wristSagittal(geometry, target);
    const d = Math.hypot(dr, dz);
    return {
      success: false,
      reason: 'OUT_OF_WORKSPACE',
      message:
        `目标 ${fmt(target)} 超出工作空间：腕枢轴到肩枢轴的距离 ${d.toFixed(3)}mm，` +
        `可达范围 ${reachMin.toFixed(3)}..${reachMax.toFixed(3)}mm ` +
        `（L1=${geometry.l1.toFixed(3)} + L2=${geometry.l2.toFixed(3)}；` +
        `已扣除腕→TCP 偏移 [${geometry.toolOffset[0].toFixed(3)}, ${geometry.toolOffset[1].toFixed(3)}]` +
        (opts.reachMaxMm === undefined ? '）' : '；外径覆写生效中）'),
      candidates: [],
    };
  }

  // ---- ③ 偏航限位（与支解无关，先判）-----------------------------------------
  let azimuthUsed = azimuth;
  if (azimuthIndeterminate) {
    // 目标在偏航轴上：方位角不定，取当前值（就近）或 home，再钳位
    const fallback = opts.near?.[base.id] ?? model.homePose[base.id] ?? 0;
    azimuthUsed = clampToLimits(base, fallback);
  }
  const baseViolation = azimuthIndeterminate ? 0 : limitViolation(base, azimuthUsed);

  // ---- ④ 选支 ---------------------------------------------------------------
  const preference: IkPreference = opts.prefer ?? 'nearest';
  const effectivePreference: IkPreference = preference === 'nearest' && !opts.near ? 'elbow-up' : preference;

  const feasible = candidates.filter((c) => c.feasible && baseViolation <= EPS_DEG);
  if (feasible.length === 0) {
    if (baseViolation > EPS_DEG) {
      return {
        success: false,
        reason: 'JOINT_LIMIT',
        joint: base.id,
        message:
          `JOINT ${base.id} ${azimuthUsed.toFixed(3)} ` +
          `(limit ${base.limits.min}..${base.limits.max})：目标方位角超出底座可达范围`,
        candidates,
      };
    }
    // 几何可达但两支解都超限 → 报越界最轻的那支
    const best = candidates.reduce((a, b) => (a.violation <= b.violation ? a : b));
    const joint = best.violatedJoint === elbow.id ? elbow : shoulder;
    const value = best.violatedJoint === elbow.id ? best.joints[elbow.id]! : best.joints[shoulder.id]!;
    return {
      success: false,
      reason: 'JOINT_LIMIT',
      joint: joint.id,
      message:
        `JOINT ${joint.id} ${value.toFixed(3)} ` +
        `(limit ${joint.limits.min}..${joint.limits.max})：` +
        `几何可达但两支解都越界，最接近的一支（${best.branch}）仍差 ${best.violation.toFixed(3)}°`,
      candidates,
    };
  }

  const chosen = selectCandidate(model, feasible, effectivePreference, opts);

  // ---- ⑤ 收敛：合成完整状态并用 FK 自查残差 -----------------------------------
  const joints = completeState(model, chosen.joints, opts);
  joints[base.id] = azimuthUsed;

  const achieved = endEffectorPosition(model, joints);
  const residual = dist(achieved, target);
  const tolerance = opts.tolerance ?? 1e-9;
  if (!(residual <= Math.max(tolerance, 1e-6))) {
    // 走到这里说明解算或模型有问题（例如连杆共线退化），必须显式暴露而不是静默返回
    throw new IkModelError(
      `解算自检失败：目标 ${fmt(target)}，FK 实得 ${fmt(achieved)}，残差 ${residual}mm ` +
        `（容差 ${tolerance}mm）—— 这是实现缺陷，请检查模型与 ik.ts 的一致性`,
    );
  }

  return {
    success: true,
    joints,
    branch: chosen.branch,
    residual,
    azimuth: azimuthUsed,
    relativeAngle: chosen.relativeAngle,
  };
}

function selectCandidate(
  model: RobotModel,
  feasible: IkCandidate[],
  preference: IkPreference,
  opts: IkOptions,
): IkCandidate {
  if (preference === 'nearest') {
    const near = opts.near!;
    return feasible.reduce((a, b) =>
      squaredDistanceTo(model, b.joints, near) < squaredDistanceTo(model, a.joints, near) ? b : a,
    );
  }
  const wanted = feasible.find((c) => c.branch === preference);
  if (wanted) return wanted;
  // 指定支不可行 → 退到可行解里的 elbow-up（真机 HOME 所在支），绝不静默返回不可行解
  return feasible.find((c) => c.branch === 'elbow-up') ?? feasible[0]!;
}

function squaredDistanceTo(model: RobotModel, a: JointState, b: JointState): number {
  let sum = 0;
  for (const joint of movableJoints(model)) {
    const av = a[joint.id];
    const bv = b[joint.id];
    if (av === undefined || bv === undefined) continue;
    const d = av - bv;
    sum += d * d;
  }
  return sum;
}

/** 全部候选解 + 选取结果，供调试面板 / 可视化使用 */
export function solveIkAll(
  model: RobotModel,
  target: Vec3,
  opts: IkOptions = {},
): { result: IkResult; candidates: IkCandidate[]; azimuthIndeterminate: boolean } {
  const probe = solveIkCandidates(model, target);
  const result = solveIk(model, target, opts);
  return {
    result,
    candidates: probe.candidates,
    azimuthIndeterminate: probe.azimuthIndeterminate,
  };
}
