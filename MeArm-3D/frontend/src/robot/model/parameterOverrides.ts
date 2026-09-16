/**
 * 末端目标「参数覆写」层 —— **只改参数，不改算法**。
 *
 * ## 为什么需要这一层
 *
 * `ik.ts`（解析式 2R 逆解）同时被三条链共用：
 *   · 前端 UI（sim web）
 *   · sim2sim（经 `kinematics-bridge.mjs` 直接调用 `solveIk`）
 *   · sim2real
 *
 * 而 `reach`（球壳）与可行判据（`EPS_DEG = 1e-9`）**都写在 `ik.ts` 内部**，
 * 外部无法覆写。若直接改它，三条链会同时变化 —— 这不是"调参数"，是"改算法"。
 *
 * ⇒ 本层的定位：**在 `ik.ts` 的调用侧做参数变换 / 前置判据**，
 *   让它自己的代码一行不动。
 *
 * ## 三项独立参数
 *
 * ### ① 判据容差 `jointToleranceDeg`（默认 0）
 *
 * `ik.ts` 的可行判据是 `violation <= EPS_DEG`（`EPS_DEG = 1e-9`），
 * 等价于"目标点必须由 FK 精确产出"。而界面上 XYZ 只显示 1 位小数，
 * 手输必然带舍入 ⇒ 极限点回输会被拒（实测差 0.0038°，见
 * `docs/workspace-boundary.md`）。本参数把判据放宽到 `violation <= T`，
 * 越界 ≤ T 的解视为可行，随后被 `clipJointState` 钳到限位上。
 *
 * ⚠️ 上限 `1.0°` 由「硬误差 ≤ 2%」反推（推导见 `jointToleranceDeg` 的注释）。
 *
 * ### ② 球壳内径 `reachMinMm`（默认不覆写，覆写下限 0.1mm）
 *
 * `ik.ts` 里 `reach = [|l1−l2|, l1+l2]`，本机 = `[0, 160]`。
 * 内径**只能收紧**（`reachMinMm ≥ 0.1 > 原内径 0`）⇒ 前置检查恒比原判据更严，
 * 原判据被完全包含，`ik.ts` 一字不改；**外径精确不动**。
 *
 * ### ③ 球壳外径 `reachMaxMm`（默认不覆写，覆写区间 `[1, 160]`）
 *
 * ⚠️ 与外径不同，**外径覆写必须真正参与解算** —— 因为 2R 的两支解
 * （`cosAlpha = (d² − l1² − l2²)/(2·l1·l2)`）本身就依赖杆长。
 * 只在前面拦一道 `d > reachMaxMm` 是不够的：用户选 100mm 外径时，
 * 期望的是"这台机器人此刻只有 100mm 的臂展"，而不是"能算到 160 但前面挡着"。
 *
 * 做法：把覆写值作为 `solveIk` 的 `opts.reachMaxMm` 传进去，
 * 由它替换 `ik.ts` 从 `ikGeometry()` 求导出的 `reach[1]`。
 *
 * ★ **这个改法是"零行为变更"的**：不覆写时 `opts.reachMaxMm === undefined`
 *   ⇒ `ik.ts` 走原分支，逐位一致。覆写时，目标在覆写外径**以内**的成立路径
 *   与不覆写**完全一致**（同一组 `cosAlpha` 公式，`d ≤ reachMaxMm ≤ 160`
 *   时 `cosAlpha` 落在同一个合法域内）。差异**只在判据边缘**：
 *   `d ∈ (reachMaxMm, 原 reachMax]` 由"可达"变为 `OUT_OF_WORKSPACE` ——
 *   这正是"参与计算"的含义。
 *
 * ⚠️ 覆写值**同时**在本层做一次前置检查：因为 `ik.ts` 在 `d > reachMax` 时
 * 返回的是 `OUT_OF_WORKSPACE` 且**不带 candidate**，容差层就没有候选可用。
 * 前置检查保证报错信息里带上"是覆写后的外径"这一关键上下文。
 *
 * ### 隔离
 *
 * 上述三项都在 `ik.ts` **之外**（或在 `opts` 里显式传入）⇒
 * `solveIk` 被直接调用的地方（sim2sim bridge）**看不到**任何覆写 ——
 * 这正是需求「不影响 sim2sim / sim2real」所要求的隔离。
 *
 * ## 内径下限为什么是 0.1 而不是 0
 *
 * `reachMin = 0` 时（本机 `l1 = l2 = 80`），
 * `cosAlpha = (0 − l1² − l2²)/(2·l1·l2) = −1` ⇒ `acos(−1) = 180°`，
 * 是「两杆完全对折重合」的**退化位形**，正好压在 `clamp1()` 的边界上，
 * 且 FK 无法稳定产出该点。留 0.1mm 避开这个奇点。
 *
 * ## 为什么**没有**"关节限位调节"
 *
 * ⚠️ 实测（2026-09-16）：本机 `elbow` 限位 `[108.441485, 141.858221]`
 * 恰等于舵机 `servo_8` 硬限位 `20..100` 经标定反算的值，**余量 0.000000°**；
 * `shoulder` 同理（`servo_7` 硬限位 `80..160` → `[-6.093683, 49.454929]`）。
 *
 * ⇒ 限位**一个度都放不宽**（放宽会让 `RobotModel` 的 `ACTUATOR_REACH` 校验失败）；
 *   而向内收紧只是把"悬崖"搬个位置 —— 新极限点回输仍会被 `EPS_DEG = 1e-9` 拒，
 *   真正解决问题的是①的判据容差。
 *
 * ⇒ 该功能已按需求**整段移除**（含 UI 与全部辅助函数），不留半成品。
 */

/**
 * 末端目标的安全参数（运行期可调，**不是**配置真值）。
 *
 * ⚠️ 默认值的取向：**与引入本特性前的行为逐位一致**。
 * 这是"不影响任何算法"的字面含义 —— 不配置就等于没这个特性。
 */
export interface TargetGuardParams {
  /**
   * 判据容差（degree）。默认 `0` ⇒ 与引入本特性前**逐位一致**。
   *
   * `ik.ts` 的可行判据是 `violation <= EPS_DEG`（`EPS_DEG = 1e-9`），
   * 等价于要求"目标点必须由 FK 精确产出"。而界面上 XYZ 只显示 1 位小数
   * （`toFixed(1)`），手输必然带舍入 ⇒ 极限点回输会被拒。
   *
   * 本参数把判据放宽到 `violation <= T`：越界 ≤ T 的解**视为可行**，
   * 随后由 `clipJointState` **钳到限位上**。
   *
   * ## 上限为什么是 1.0°
   *
   * 钳位后末端偏差 `≈ L·sin(T)`，`L` 最大取 160mm（= `l1 + l2`，最坏力臂）。
   * 以实测最大可达半径 177.091mm 为基准，2% = 3.542mm：
   *
   * | T | L=80 | L=120 | L=160 |
   * |---|---|---|---|
   * | 0.5° | 0.394% | 0.592% | **0.789%** |
   * | 1.0° | 0.788% | 1.183% | **1.577%** ✅ |
   * | 2.0° | 1.577% | 2.365% | **3.154%** ❌ 超标 |
   *
   * ⇒ `T = 1.0°` 是满足"硬误差 ≤ 2%"的最大整数选择，故设为上限。
   */
  jointToleranceDeg: number;
  /** 是否启用球壳内径覆写。`false` ⇒ 用 `ik.ts` 求导出的内径（= 现状） */
  reachMinOverridden: boolean;
  /**
   * 球壳内径覆写值（mm）。合法区间 `[0.1, 外径)`。
   *
   * ⚠️ 下限 `0.1` 是硬约束（避开 `l1 = l2` 时 `acos(−1) = 180°` 的退化位形），
   * 见文件头。
   */
  reachMinMm: number;
  /** 是否启用球壳外径覆写。`false` ⇒ 用 `ik.ts` 求导出的外径 `l1 + l2`（= 现状） */
  reachMaxOverridden: boolean;
  /**
   * 球壳外径覆写值（mm）。合法区间 `[1, 160]`。
   *
   * ⚠️ 与内径不同，外径覆写**真正参与解算**（经 `solveIk` 的 `opts.reachMaxMm`），
   * 见文件头。它会**同时**限制内径（`reachMinMm` 必须小于它）。
   */
  reachMaxMm: number;
}

/** 内径覆写的硬下限（见文件头：避开退化位形） */
export const REACH_MIN_FLOOR_MM = 0.1;

/** 外径覆写的硬下限（1mm —— 再小 2R 就只剩一个点，且与内径下限 0.1 留出可分辨区间） */
export const REACH_MAX_FLOOR_MM = 1;

/**
 * 外径覆写的硬上限（mm）= `ik.ts` 求导出的几何外径 `l1 + l2`（本机 = 160）。
 *
 * ⚠️ 为什么**不能**大于它：外径的真值来自杆长，放宽它意味着
 * `d > l1 + l2` 的目标会被判"在壳内"，而 2R 在那里**无解**
 * （`cosAlpha ∉ [−1, 1]`）⇒ 会退化成 `NO_SOLUTION` 而不是明确的
 * `OUT_OF_WORKSPACE`，报错信息反而更难懂。故此处与求导值取齐。
 */
export const REACH_MAX_CEILING_MM = 160;

/** 判据容差上限（= 硬误差 1.577% ≤ 2%，见 `jointToleranceDeg` 的推导） */
export const JOINT_TOLERANCE_MAX_DEG = 1.0;

/**
 * 实测最大可达半径（mm）—— 2% 硬误差的**基准**。
 *
 * 来源：`core/tools/ws_scan_mearm_v1.py`（234498 点 / 1° 网格）的扫描结果。
 * 它同时是验收断言（`targetGuardParams.test.ts`）与界面提示
 * （`worstClampPercent`）的分母 —— 两处必须同源，否则界面显示的数字
 * 与测试守护的数字会漂移。
 */
export const MEASURED_MAX_RADIUS_MM = 177.091;

/**
 * 默认参数 —— **刻意让每一项都退化为"无覆写"**：
 * 容差 0、内径不覆写、外径不覆写。于是不配置本特性时，`moveTo` 的行为与
 * 引入前逐位相同，Phase 6 的 13 条冻结断言（越界即拒绝、关节逐位不变）
 * 继续通过。
 */
export const DEFAULT_TARGET_GUARD_PARAMS: TargetGuardParams = {
  jointToleranceDeg: 0,
  reachMinOverridden: false,
  reachMinMm: REACH_MIN_FLOOR_MM,
  reachMaxOverridden: false,
  reachMaxMm: REACH_MAX_CEILING_MM,
};

/** 参数非法时抛出（属调用方误用，不是目标不可达） */
export class TargetGuardParamError extends Error {
  constructor(message: string) {
    super(`[targetGuard] ${message}`);
    this.name = 'TargetGuardParamError';
  }
}

/**
 * 校验并规范化参数。
 *
 * ⚠️ 这里**只校验覆写值自身**（范围 / 有限性），**不做模型级校验**。
 * 理由：模型级校验（`validateRobotModel`）的对象是 `robot.yaml` 那份**配置真值**；
 * 而本层是**会话内的安全参数**。让运行期参数去触发真值校验，会把一个
 * 可恢复的输入错误升级成致命错误。
 *
 * @param geometry.reach `ik.ts` 求导出的球壳 `[内径, 外径]`，用于判
 *                       "内径 < 外径"与给出默认外径。仅作校验，不参与解算。
 */
export function normalizeTargetGuardParams(
  raw: Partial<TargetGuardParams> | undefined,
  geometry: { reach: readonly [number, number] },
): TargetGuardParams {
  const merged = { ...DEFAULT_TARGET_GUARD_PARAMS, ...(raw ?? {}) };
  const {
    jointToleranceDeg,
    reachMinMm,
    reachMinOverridden,
    reachMaxMm,
    reachMaxOverridden,
  } = merged;

  if (!Number.isFinite(jointToleranceDeg) || jointToleranceDeg < 0) {
    throw new TargetGuardParamError(
      `jointToleranceDeg=${jointToleranceDeg} 非法：必须为 ≥ 0 的有限数值`,
    );
  }
  if (jointToleranceDeg > JOINT_TOLERANCE_MAX_DEG) {
    throw new TargetGuardParamError(
      `jointToleranceDeg=${jointToleranceDeg} 超出上限 ${JOINT_TOLERANCE_MAX_DEG}°：` +
        `再大会让钳位误差超过 2%（T=2.0° 时最坏 3.154%）`,
    );
  }

  // 求导外径（= l1 + l2，本机 160）。覆写值的上限与它取齐，理由见常量注释。
  const derivedMax = geometry.reach[1];
  const maxCeiling = Number.isFinite(derivedMax)
    ? Math.min(REACH_MAX_CEILING_MM, derivedMax)
    : REACH_MAX_CEILING_MM;

  if (!Number.isFinite(reachMaxMm)) {
    throw new TargetGuardParamError(`reachMaxMm=${reachMaxMm} 非法：必须为有限数值`);
  }
  if (reachMaxMm < REACH_MAX_FLOOR_MM) {
    throw new TargetGuardParamError(
      `reachMaxMm=${reachMaxMm} 低于硬下限 ${REACH_MAX_FLOOR_MM}mm：` +
        '再小 2R 的解空间就退化成一个点',
    );
  }
  if (reachMaxMm > maxCeiling) {
    throw new TargetGuardParamError(
      `reachMaxMm=${reachMaxMm} 超出上限 ${maxCeiling.toFixed(3)}mm：` +
        '外径不能大于杆长决定的几何外径 l1 + l2（超出部分 2R 无解）',
    );
  }

  if (!Number.isFinite(reachMinMm)) {
    throw new TargetGuardParamError(`reachMinMm=${reachMinMm} 非法：必须为有限数值`);
  }
  if (reachMinMm < REACH_MIN_FLOOR_MM) {
    throw new TargetGuardParamError(
      `reachMinMm=${reachMinMm} 低于硬下限 ${REACH_MIN_FLOOR_MM}mm：` +
        '0 会让 l1 = l2 时的解落到 acos(−1) = 180° 的退化位形上',
    );
  }
  // ⚠️ 内径的上界取决于**生效的**外径：覆写外径时用它，否则用求导值。
  const effectiveMax = reachMaxOverridden ? reachMaxMm : derivedMax;
  if (reachMinMm >= effectiveMax) {
    throw new TargetGuardParamError(
      `reachMinMm=${reachMinMm} 必须小于生效外径 ${effectiveMax.toFixed(3)}mm，否则工作空间为空`,
    );
  }

  return {
    jointToleranceDeg,
    reachMinOverridden: Boolean(reachMinOverridden),
    reachMinMm,
    reachMaxOverridden: Boolean(reachMaxOverridden),
    reachMaxMm,
  };
}

/**
 * 覆写是否**真正改变**了解算（用于跳过无谓的分支 / 让日志只在生效时记录）。
 *
 * 覆写值恰好等于求导值（例如外径填 160）时，结果与不覆写**逐位相同**
 * ⇒ 报"覆写生效"会是误导。这里给出精确判据。
 */
export function reachOverrideChangesSolve(
  params: TargetGuardParams,
  geometry: { reach: readonly [number, number] },
): boolean {
  if (params.reachMinOverridden && params.reachMinMm > geometry.reach[0]) return true;
  if (params.reachMaxOverridden && params.reachMaxMm < geometry.reach[1]) return true;
  return false;
}

/**
 * 把 `TargetGuardParams` 里的外径覆写转成 `solveIk` 的 `opts.reachMaxMm`
 * （不覆写时返回 `undefined` ⇒ `ik.ts` 走原分支，逐位一致）。
 *
 * ⚠️ 单独抽成函数是为了让"传什么给 `ik.ts`"这件事只有**一处**定义 ——
 * 否则 UI 显示、前置检查、解算三处容易各自漂移。
 */
export function reachMaxOverrideFor(params: TargetGuardParams): number | undefined {
  return params.reachMaxOverridden ? params.reachMaxMm : undefined;
}
