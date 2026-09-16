/**
 * 末端目标「安全参数」验收 —— 需求判据。
 *
 * 对应需求：
 *   ① 判据容差：硬误差保证在 2% 以内
 *   ② 几何球壳内径可调（最小 0.1，其它方式不能变化）
 *   ③ 几何球壳外径可调（1~160mm，启用后**实时变化并参与计算**）
 *   ④ 不影响任何算法，仅影响参数
 *
 * ⚠️ 本文件的**最重要**的一组断言是「默认参数下行为逐位不变」——
 * 它是需求④的字面判据，也是"不配置就等于没这个特性"的机器证明。
 *
 * ⚠️ 「关节限位调节」已按需求整段移除，本文件不再覆盖它。
 */
import { beforeEach, describe, expect, it } from 'vitest';
import {
  endEffectorPosition,
  endEffectorPose,
  homeJointState,
  jointByRole,
  loadRobotModel,
  movableJoints,
  type JointState,
  type Vec3,
} from '../../src/robot';
import {
  DEFAULT_TARGET_GUARD_PARAMS,
  JOINT_TOLERANCE_MAX_DEG,
  MEASURED_MAX_RADIUS_MM,
  REACH_MAX_CEILING_MM,
  REACH_MAX_FLOOR_MM,
  REACH_MIN_FLOOR_MM,
  normalizeTargetGuardParams,
} from '../../src/robot/model/parameterOverrides';
import { targetGapMm, useRobotStore } from '../../src/store/robotStore';

const model = loadRobotModel('mearm-v1');
const home = homeJointState(model);
const store = () => useRobotStore.getState();

/** 实测最大可达半径（`core/tools/ws_scan_mearm_v1.py`，234498 点 / 1° 网格） */
const MEASURED_MAX_R_MM = 177.091;

/** 求导球壳（`[|l1−l2|, l1+l2]`，本机 `[0, 160]`） */
const SHELL: readonly [number, number] = [0, 160];

/** 由一个合法关节状态反推目标点，保证目标一定可达 */
function reachable(patch: Partial<JointState>): Vec3 {
  const p = endEffectorPosition(model, { ...home, ...patch } as JointState);
  return [p[0], p[1], p[2]];
}

/**
 * 极限点：**腕枢轴距离 `d` 最大**的可达点。
 *
 * ⚠️ 这个锚点必须实测，不能靠"把某个关节推到限位"想当然：
 *   · 几何球壳（`ik.ts` 的 `reach`）是 `[0, 160]`
 *   · 但**关节限位把它裁到 `[44.1665, 139.2662]`**（401×401 网格实测）
 *   ⇒ 外径覆写只有落在 `(44.1665, 139.2662]` 内才会真正改变可达性；
 *     填 150 / 160 这种值等于没覆写。
 *
 * 极值点：
 *   · `shoulder=max, elbow=min` → `d = 139.2662`  ← **最大**（两杆近乎伸直）
 *   · `shoulder=min, elbow=max` → `d = 44.1665`   ← **最小**（折得最狠）
 *   · 注意 `shoulder=0, elbow=max` 只有 `d = 52.278`，**不是**极限。
 */
function atOuterShell(): Vec3 {
  const shoulder = jointByRole(model, 'shoulder')!;
  const elbow = jointByRole(model, 'elbow')!;
  return reachable({
    base: 0,
    [shoulder.id]: shoulder.limits.max,
    [elbow.id]: elbow.limits.min,
  });
}

/** 实测：合法限位域内腕枢轴距离的极值（解析式 401×401 网格 + FK 双层复核） */
const D_MIN_MM = 44.1665;
const D_MAX_MM = 139.2662;

/**
 * 由 TCP 目标点反算「腕枢轴到肩枢轴的距离 `d`」（mm）——
 * 与 `ik.ts` 的 `wristSagittal()` 同一套换算。
 *
 * ⚠️ **实测踩过的坑**：第一版把 `radial` 取成 `wrist.parentLink` 的长度，
 * 但 `tcp.joint = 'tool'` 的 `parentLink` 是 **`forearm_link`（80）**，
 * 不是 `tool_link`（40）⇒ 算出的 `d` 偏小（94.15 vs 真值 131.96），
 * 于是"外径 100 应可达"的断言假性通过/失败。
 *
 * `ik.ts` 的正确口径是：
 *   · `pivotZ`  = 肩枢轴在 FK 里的**实际 Z**（实测 60）
 *   · `radial`  = `toolOffset[0]`，即 **TCP 偏移的径向分量**（实测 40）
 *
 * 这里显式用 `model.tcp.offset` 求径向分量，与 `ik.ts` 的
 * `requireConstantToolOffset()` 同一语义，**不再依赖 parentLink 猜测**。
 */
function wristDistanceOf(target: Vec3): number {
  const shoulder = jointByRole(model, 'shoulder')!;

  // pivotZ：沿 parent 链累加到 shoulder 所辖连杆为止（本机 column(60) + base(0) = 60）
  let pivotZ = 0;
  {
    let cursor = model.links.find((l) => l.id === shoulder.parentLink);
    while (cursor) {
      pivotZ += cursor.length;
      cursor = cursor.parent ? model.links.find((l) => l.id === cursor!.parent) : undefined;
    }
  }
  // 实测锚点：必须等于 ikGeometry().pivotZ（60），否则本函数与 ik.ts 口径不一致
  expect(pivotZ).toBeCloseTo(60, 9);

  // radial = TCP 偏移的径向分量（本机 [0,0,40] ⇒ 本机是沿 +Z 表达、经被动腕转成径向 40）
  const offset = model.tcp.offset as unknown as [number, number, number];
  const radial = Math.abs(offset[2]);
  expect(radial).toBeCloseTo(40, 9);

  const r = Math.hypot(target[0], target[1]);
  return Math.hypot(r - radial, target[2] - pivotZ);
}

/** 界面精度：1 位小数（用户手输与截图记录的实际精度） */
function toTenths(p: Vec3): Vec3 {
  const r = (v: number) => Math.round(v * 10) / 10;
  return [r(p[0]), r(p[1]), r(p[2])];
}

/**
 * ★ 找一个**确定带舍入误差**的极限点。
 *
 * 为什么不能直接用 `atElbowMax()`：`base = 0, shoulder = 0` 时该点的三个分量
 * 可能恰好落在 0.1mm 的整格上（或舍入后仍落在限位内侧），默认参数就能接受，
 * 无法复现用户报的「极限值回输被拒」。
 *
 * 这里穷举实际可达的极限姿态，取其中"精确角贴限位、但舍入后的回输被拒"的点。
 */
function findRoundingRejectedTarget(): { xyz: Vec3; deviationMm: number } | null {
  const base = jointByRole(model, 'base')!;
  const shoulder = jointByRole(model, 'shoulder')!;
  const elbow = jointByRole(model, 'elbow')!;

  const baseVals = [base.limits.min, -30, 0, 30, base.limits.max];
  const shoulderVals = [shoulder.limits.min, shoulder.limits.max];
  const elbowVals = [elbow.limits.min, elbow.limits.max];

  for (const b of baseVals) {
    for (const s of shoulderVals) {
      for (const e of elbowVals) {
        const exact = reachable({ [base.id]: b, [shoulder.id]: s, [elbow.id]: e });
        const rounded = toTenths(exact);
        const deviationMm = Math.hypot(
          rounded[0] - exact[0],
          rounded[1] - exact[1],
          rounded[2] - exact[2],
        );
        if (deviationMm <= 0) continue;

        // ⚠️ 走 store 而不是直接 import `solveIk`：`ik.ts` **刻意不从 `@robot` 出口
        //    再导出**（Core 里不得出现型号名，见 `src/robot/index.ts` 注释）。
        store().resetTargetGuard();
        store().goHome();
        if (!store().moveTo(rounded).success) return { xyz: rounded, deviationMm };
      }
    }
  }
  return null;
}

const roundingCase = findRoundingRejectedTarget();

beforeEach(() => {
  store().resetTargetGuard();
  store().goHome();
});

// ---------------------------------------------------------------------------
// 需求④：默认参数下行为逐位不变
// ---------------------------------------------------------------------------

describe('需求④ · 默认参数 = 不影响任何算法', () => {
  it('默认参数全部退化为「无覆写」', () => {
    expect(DEFAULT_TARGET_GUARD_PARAMS.jointToleranceDeg).toBe(0);
    expect(DEFAULT_TARGET_GUARD_PARAMS.reachMinOverridden).toBe(false);
    expect(DEFAULT_TARGET_GUARD_PARAMS.reachMaxOverridden).toBe(false);
  });

  it('store 初始参数 = 默认值', () => {
    expect(store().targetGuard).toEqual(DEFAULT_TARGET_GUARD_PARAMS);
    expect(store().targetGuardError).toBeNull();
  });

  it('容差为 0 时，越界目标仍被拒绝（Phase 6 冻结语义不受影响）', () => {
    const before: JointState = { ...store().commandJoints };
    const result = store().moveTo([600, 0, 60]);
    expect(result.success).toBe(false);
    expect(store().ikStatus?.ok).toBe(false);
    for (const joint of movableJoints(model)) {
      expect(store().commandJoints[joint.id]).toBe(before[joint.id]);
    }
  });

  it('全部不覆写时，同一目标解出的关节稳定', () => {
    const xyz = reachable({ base: 20, shoulder: 20, elbow: 130 });
    const first = store().moveTo(xyz);
    expect(first.success).toBe(true);
    const jointsA = { ...store().commandJoints };

    store().setTargetGuard({
      jointToleranceDeg: 0,
      reachMinOverridden: false,
      reachMaxOverridden: false,
    });
    store().moveTo(xyz);
    for (const joint of movableJoints(model)) {
      expect(store().commandJoints[joint.id]).toBe(jointsA[joint.id]);
    }
  });
});

// ---------------------------------------------------------------------------
// 需求②：球壳内径可调（最小 0.1）
// ---------------------------------------------------------------------------

describe('需求② · 球壳内径覆写', () => {
  it('下限硬约束：reachMinMm < 0.1 被拒', () => {
    expect(() =>
      normalizeTargetGuardParams({ reachMinMm: 0.05, reachMinOverridden: true }, {
        reach: SHELL,
      }),
    ).toThrowError();
    // 恰好 0.1 合法
    expect(() =>
      normalizeTargetGuardParams({ reachMinMm: 0.1, reachMinOverridden: true }, {
        reach: SHELL,
      }),
    ).not.toThrow();
    // 0（退化位形）必须被拒
    expect(() =>
      normalizeTargetGuardParams({ reachMinMm: 0, reachMinOverridden: true }, {
        reach: SHELL,
      }),
    ).toThrowError();
  });

  it('reachMinMm ≥ 生效外径被拒（工作空间会变空）', () => {
    expect(() =>
      normalizeTargetGuardParams({ reachMinMm: 160, reachMinOverridden: true }, {
        reach: SHELL,
      }),
    ).toThrowError();
  });

  it('store 拒绝非法内径并保留旧值、给出原因', () => {
    store().setTargetGuard({ reachMinOverridden: true, reachMinMm: 80 });
    const okState = store().targetGuard;

    const accepted = store().setTargetGuard({ reachMinMm: 0.01 });
    expect(accepted).toBe(false);
    expect(store().targetGuard).toEqual(okState); // 旧值保留
    expect(store().targetGuardError).toBeTruthy();
  });

  it('覆写生效后：内径以内的目标被判 OUT_OF_WORKSPACE', () => {
    const inner = reachable({ base: 0, shoulder: 45, elbow: 120 });
    // 先确认该点在不覆写时是**可达**的（否则证明不了覆写在起作用）
    expect(store().moveTo(inner).success).toBe(true);

    store().setTargetGuard({ reachMinOverridden: true, reachMinMm: 159 });
    expect(store().targetGuardError).toBeNull();

    const result = store().moveTo(inner);
    expect(result.success).toBe(false);
    if (!result.success) expect(result.reason).toBe('OUT_OF_WORKSPACE');
  });

  it('覆写只收紧内径、外径精确不变', () => {
    const outer = reachable({ base: 0, shoulder: 0, elbow: 141 });
    store().setTargetGuard({ reachMinOverridden: true, reachMinMm: 10 });
    expect(store().moveTo(outer).success).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// 需求③：球壳外径可调（1~160mm，实时变化并参与计算）
// ---------------------------------------------------------------------------

describe('需求③ · 球壳外径覆写', () => {
  it('范围硬约束：< 1 与 > 160 均被拒，端点合法', () => {
    // 下限
    expect(() =>
      normalizeTargetGuardParams({ reachMaxMm: 0.5, reachMaxOverridden: true }, {
        reach: SHELL,
      }),
    ).toThrowError();
    expect(() =>
      normalizeTargetGuardParams({ reachMaxMm: REACH_MAX_FLOOR_MM, reachMaxOverridden: true }, {
        reach: SHELL,
      }),
    ).not.toThrow();

    // 上限
    expect(() =>
      normalizeTargetGuardParams({ reachMaxMm: 161, reachMaxOverridden: true }, {
        reach: SHELL,
      }),
    ).toThrowError();
    expect(() =>
      normalizeTargetGuardParams({ reachMaxMm: REACH_MAX_CEILING_MM, reachMaxOverridden: true }, {
        reach: SHELL,
      }),
    ).not.toThrow();
  });

  it('REACH_MAX_FLOOR_MM / CEILING 常量与需求一致（1 ~ 160）', () => {
    expect(REACH_MAX_FLOOR_MM).toBe(1);
    expect(REACH_MAX_CEILING_MM).toBe(160);
  });

  it('store 拒绝越界外径并保留旧值、给出原因', () => {
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 120 });
    const okState = store().targetGuard;

    expect(store().setTargetGuard({ reachMaxMm: 0.5 })).toBe(false);
    expect(store().targetGuard).toEqual(okState);
    expect(store().targetGuardError).toBeTruthy();

    expect(store().setTargetGuard({ reachMaxMm: 999 })).toBe(false);
    expect(store().targetGuard).toEqual(okState);
  });

  it('★ 参与计算：外径收紧后，原本可达的外圈目标变为 OUT_OF_WORKSPACE', () => {
    // 取腕枢轴距离最大的可达点（`d ≈ 139.27mm`），默认下必然成功
    const outer = atOuterShell();
    expect(store().moveTo(outer).success).toBe(true);

    // 把外径收到 80mm ⇒ 该点（d ≈ 139.27，**远在 80 之外**）应被判越界。
    // ⚠️ 80 这个值必须**明显小于** `D_MAX_MM`，否则这条断言会因为
    //    "覆写值恰好够用"而假性通过 —— 这正是本文件初版用 `d≈19.5` 的
    //    伪外圈点（`shoulder=0, elbow=141`）踩过的坑。
    expect(D_MAX_MM).toBeGreaterThan(80);
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 80 });
    expect(store().targetGuardError).toBeNull();

    const before: JointState = { ...store().commandJoints };
    const result = store().moveTo(outer);
    expect(result.success).toBe(false);
    if (!result.success) expect(result.reason).toBe('OUT_OF_WORKSPACE');

    // 越界时关节逐位不变（Phase 6 冻结语义）
    for (const joint of movableJoints(model)) {
      expect(store().commandJoints[joint.id]).toBe(before[joint.id]);
    }
  });

  it('★ 参与计算：外径以内确实变可达（解算仍成立且残差合格）', () => {
    const shoulder = jointByRole(model, 'shoulder')!;
    const elbow = jointByRole(model, 'elbow')!;
    // 采一个 d 落在 (D_MIN, D_MAX) 内部的点。`shoulder = 0.5·max` 时两杆
    // 有明显折角，d 明显小于上界（实测 d ≈ 112）。
    const inner = reachable({
      base: 0,
      [shoulder.id]: shoulder.limits.max * 0.5,
      [elbow.id]: elbow.limits.min,
    });
    const dInner = wristDistanceOf(inner);
    // 先钉住采点确实在区间内部 —— 否则下面"覆写 130 仍可达"就没有信息量
    expect(dInner).toBeGreaterThan(D_MIN_MM);
    expect(dInner).toBeLessThan(D_MAX_MM);

    // 覆写外径取 130：**大于实测 d ⇒ 不干涉**（这是"覆写参与计算但不误伤"的关键）
    const reachMaxMm = 130;
    expect(dInner).toBeLessThan(reachMaxMm);

    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm });
    expect(store().targetGuardError).toBeNull();

    const result = store().moveTo(inner);
    expect(
      result.success,
      `d=${dInner.toFixed(3)} 应可达（覆写外径 ${reachMaxMm}）；实得 ${
        result.success ? 'ok' : `${result.reason} @ ${result.joint ?? '-'}`
      }`,
    ).toBe(true);

    // 残差（FK 自查）必须仍然合格 —— 证明是"真解出来了"，不是硬塞
    const residual = store().ikStatus?.residual ?? Number.POSITIVE_INFINITY;
    expect(residual).toBeLessThan(1e-6);
  });

  it('★ 覆写外径只在 d 超出时才拒绝：同一点在 130 可达、在 100 被拒（边界随参数移动）', () => {
    const shoulder = jointByRole(model, 'shoulder')!;
    const elbow = jointByRole(model, 'elbow')!;
    const pt = reachable({
      base: 0,
      [shoulder.id]: shoulder.limits.max * 0.5,
      [elbow.id]: elbow.limits.min,
    });
    const d = wristDistanceOf(pt);
    expect(d).toBeGreaterThan(100);
    expect(d).toBeLessThan(130);

    // 外径 130 > d ⇒ 可达
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 130 });
    expect(store().moveTo(pt).success).toBe(true);

    // 外径 100 < d ⇒ 被拒，且**关节逐位不变**（同一点，只改参数）
    const before: JointState = { ...store().commandJoints };
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 100 });
    const rejected = store().moveTo(pt);
    expect(rejected.success).toBe(false);
    if (!rejected.success) expect(rejected.reason).toBe('OUT_OF_WORKSPACE');
    for (const joint of movableJoints(model)) {
      expect(store().commandJoints[joint.id]).toBe(before[joint.id]);
    }
  });

  it('★ 外径覆写不改变限位：越界原因是 OUT_OF_WORKSPACE 而非 JOINT_LIMIT', () => {
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 80 });
    const outer = atOuterShell();
    const result = store().moveTo(outer);
    expect(result.success).toBe(false);
    // 关键：外径覆写只影响"几何可达性"，不碰关节限位
    if (!result.success) expect(result.reason).toBe('OUT_OF_WORKSPACE');
  });

  it('★ 实时生效：参数一变，当前 target 的判定立即跟随（无需重新点 Move）', () => {
    const outer = atOuterShell();
    store().moveTo(outer);
    expect(store().ikStatus?.ok).toBe(true);

    // 只改参数，不调 moveTo —— setTargetGuard 内部会用当前 target 重解
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 80 });

    expect(store().ikStatus?.ok).toBe(false);
    expect(store().ikStatus?.reason).toBe('OUT_OF_WORKSPACE');
  });

  it('覆写值等于求导外径时，结果与不覆写逐位一致（无谓覆写不产生差异）', () => {
    const xyz = reachable({ base: 15, shoulder: 20, elbow: 130 });

    store().moveTo(xyz);
    const jointsA = { ...store().commandJoints };

    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: REACH_MAX_CEILING_MM });
    store().moveTo(xyz);
    for (const joint of movableJoints(model)) {
      expect(store().commandJoints[joint.id]).toBe(jointsA[joint.id]);
    }
  });

  it('★★ 前置检查的 d 必须与 ik.ts 同口径：store 报的 d == 独立复算的 d（全采样点）', () => {
    // 这一条是**事故回归**：store 曾自己重算 `d`，把 `radial` 取成
    // `wrist.parentLink`（= forearm_link 80）而不是 `toolOffset[0]`（= 40）
    // ⇒ 同一个目标点算出两个 d（92.159 vs 111.542，差 19.4mm）且不报错。
    // 症状是"改小外径后本该可达的点被判越界"（错误信息来源随机）。
    //
    // 本断言的做法：用一个**故意很小的外径**让 store 的前置检查必然触发，
    // 从错误消息里抠出它算的 d，与 helper 的独立复算比对。
    const shoulder = jointByRole(model, 'shoulder')!;
    const elbow = jointByRole(model, 'elbow')!;
    const base = jointByRole(model, 'base')!;

    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: REACH_MAX_FLOOR_MM });

    let checked = 0;
    for (const sRatio of [0, 0.25, 0.5, 0.75, 1]) {
      for (const eRatio of [0, 0.5, 1]) {
        const pt = reachable({
          [base.id]: 20,
          [shoulder.id]: shoulder.limits.min + (shoulder.limits.max - shoulder.limits.min) * sRatio,
          [elbow.id]: elbow.limits.min + (elbow.limits.max - elbow.limits.min) * eRatio,
        });
        store().moveTo(pt);
        const message = store().ikStatus?.message ?? '';
        const m = /距离 ([\d.]+)mm/.exec(message);
        expect(m, `前置检查未触发，无法核对 d：${message}`).not.toBeNull();

        const dStore = Number(m![1]);
        const dRef = wristDistanceOf(pt);
        // 同一个量，两处口径必须一致（1e-6 容差；消息里只打印 3 位小数，故放宽到 1e-3）
        expect(
          Math.abs(dStore - dRef),
          `d 口径不一致 @ sRatio=${sRatio} eRatio=${eRatio}：store=${dStore} 复算=${dRef}`,
        ).toBeLessThan(1e-3);
        checked += 1;
      }
    }
    expect(checked).toBe(15);
  });

  it('内径上界随生效外径联动：外径 50 时内径 80 被拒', () => {
    store().setTargetGuard({ reachMaxOverridden: true, reachMaxMm: 50 });
    expect(store().targetGuardError).toBeNull();

    const accepted = store().setTargetGuard({ reachMinOverridden: true, reachMinMm: 80 });
    expect(accepted).toBe(false);
    expect(store().targetGuardError).toBeTruthy();
  });

  it('★ 实测锚点：合法限位域内腕枢轴距离为 [44.1665, 139.2662]mm', () => {
    // 这两个数决定了"外径覆写能起多大作用"：几何球壳是 [0,160]，
    // 但关节限位把它裁到 [44.1665, 139.2662] ⇒ 外径覆写只有落在这个区间内
    // 才会**真正改变**可达性；填 150 / 160 等于没覆写。
    // 这个断言把该事实钉住，避免后续有人用错锚点写测试（本文件初版就踩过：
    // 曾把"外圈点"写成 shoulder=0/elbow=141 的 d≈19.5 点，离外径十万八千里）。
    const outer = atOuterShell();
    const dOuter = wristDistanceOf(outer);
    expect(dOuter).toBeGreaterThan(D_MAX_MM - 0.01);
    expect(dOuter).toBeLessThan(D_MAX_MM + 0.01);

    const shoulder = jointByRole(model, 'shoulder')!;
    const elbow = jointByRole(model, 'elbow')!;
    const folded = reachable({
      base: 0,
      [shoulder.id]: shoulder.limits.min,
      [elbow.id]: elbow.limits.max,
    });
    const dFolded = wristDistanceOf(folded);
    expect(dFolded).toBeGreaterThan(D_MIN_MM - 0.01);
    expect(dFolded).toBeLessThan(D_MIN_MM + 0.01);

    // 两者都在几何球壳 [0,160] 之内 —— 说明是限位在裁剪，不是几何
    expect(dFolded).toBeGreaterThan(0);
    expect(dOuter).toBeLessThan(160);

    // ⚠️ 推论：外径覆写的**有效区间**远小于 UI 允许的 1~160
    expect(D_MAX_MM).toBeLessThan(160);
  });
});

// ---------------------------------------------------------------------------
// 需求①：硬误差 ≤ 2%（判据容差）
// ---------------------------------------------------------------------------

describe('需求① · 容差与硬误差', () => {
  it('容差上限为 1.0°（2.0° 会让最坏钳位误差 3.154% 超标）', () => {
    expect(JOINT_TOLERANCE_MAX_DEG).toBe(1.0);
    expect(() =>
      normalizeTargetGuardParams({ jointToleranceDeg: 1.0 }, { reach: SHELL }),
    ).not.toThrow();
    expect(() =>
      normalizeTargetGuardParams({ jointToleranceDeg: 1.5 }, { reach: SHELL }),
    ).toThrowError();
    expect(() =>
      normalizeTargetGuardParams({ jointToleranceDeg: -0.1 }, { reach: SHELL }),
    ).toThrowError();
  });

  it('store 拒绝超限容差并保留旧值', () => {
    const before = store().targetGuard;
    expect(store().setTargetGuard({ jointToleranceDeg: 2 })).toBe(false);
    expect(store().targetGuard).toEqual(before);
    expect(store().targetGuardError).toBeTruthy();
  });

  it('★ 存在这样的极限点：默认被拒、启用容差后被接受，且钳位误差 ≤ 2%', () => {
    expect(roundingCase).not.toBeNull();
    const { xyz, deviationMm } = roundingCase!;
    expect(deviationMm).toBeGreaterThan(0);
    expect(deviationMm).toBeLessThan(0.5);

    const before = store().moveTo(xyz);
    expect(before.success).toBe(false);

    store().setTargetGuard({ jointToleranceDeg: JOINT_TOLERANCE_MAX_DEG });
    const after = store().moveTo(xyz);
    expect(after.success).toBe(true);

    const gap = targetGapMm(store().target, store().commandJoints, model);
    const pct = (gap / MEASURED_MAX_R_MM) * 100;
    expect(pct).toBeLessThanOrEqual(2.0);
  });

  it('★ 容差 1.0° 的钳位误差上界满足 2%（解析式复核）', () => {
    const reachMax = 160; // 最坏力臂 = l1 + l2
    const displacement = reachMax * Math.sin((JOINT_TOLERANCE_MAX_DEG * Math.PI) / 180);
    const pct = (displacement / MEASURED_MAX_RADIUS_MM) * 100;
    expect(pct).toBeLessThan(2.0);
    expect(pct).toBeGreaterThan(1.4); // 1.577%，确认上界是"紧"的而非虚设
  });

  it('界面显示用的基准常量与验收断言同源', () => {
    expect(MEASURED_MAX_RADIUS_MM).toBe(MEASURED_MAX_R_MM);
  });

  it('容差生效时所有下发关节仍严格落在限位内', () => {
    store().setTargetGuard({ jointToleranceDeg: JOINT_TOLERANCE_MAX_DEG });
    store().moveTo(toTenths(atOuterShell()));
    for (const joint of movableJoints(model)) {
      const value = store().commandJoints[joint.id]!;
      expect(value).toBeGreaterThanOrEqual(joint.limits.min - 1e-9);
      expect(value).toBeLessThanOrEqual(joint.limits.max + 1e-9);
    }
  });
});

// ---------------------------------------------------------------------------
// 复位
// ---------------------------------------------------------------------------

describe('参数复位', () => {
  it('resetTargetGuard 清空全部覆写与错误', () => {
    store().setTargetGuard({
      jointToleranceDeg: 0.5,
      reachMinOverridden: true,
      reachMinMm: 20,
      reachMaxOverridden: true,
      reachMaxMm: 100,
    });
    store().setTargetGuard({ jointToleranceDeg: 99 }); // 非法，留下错误
    expect(store().targetGuardError).toBeTruthy();

    store().resetTargetGuard();
    expect(store().targetGuard).toEqual(DEFAULT_TARGET_GUARD_PARAMS);
    expect(store().targetGuardError).toBeNull();
  });
});

// ---------------------------------------------------------------------------
// 隔离：不污染 sim2sim / 其它路径
// ---------------------------------------------------------------------------

describe('隔离 · 覆写不外溢', () => {
  it('内径下限常量与文档一致（避开退化位形）', () => {
    expect(REACH_MIN_FLOOR_MM).toBe(0.1);
  });

  it('覆写不影响 FK（末端位姿只由关节角决定）', () => {
    const before = endEffectorPose(model, home).position;
    store().setTargetGuard({
      reachMinOverridden: true,
      reachMinMm: 100,
      reachMaxOverridden: true,
      reachMaxMm: 100,
    });
    const after = endEffectorPose(model, home).position;
    expect(after).toEqual(before);
  });

  it('覆写不改变模型本身（真值零污染）', () => {
    const before = {
      joints: model.joints.map((j) => ({ ...j.limits })),
      links: model.links.map((l) => l.length),
    };
    store().setTargetGuard({
      reachMaxOverridden: true,
      reachMaxMm: 100,
      jointToleranceDeg: 0.5,
    });
    store().moveTo([100, 0, 90]);
    expect(model.joints.map((j) => ({ ...j.limits }))).toEqual(before.joints);
    expect(model.links.map((l) => l.length)).toEqual(before.links);
  });

  it('★ 关节限位调节已移除：参数对象里不存在 jointTightenDeg', () => {
    expect('jointTightenDeg' in store().targetGuard).toBe(false);
    expect(Object.keys(DEFAULT_TARGET_GUARD_PARAMS).sort()).toEqual([
      'jointToleranceDeg',
      'reachMaxMm',
      'reachMaxOverridden',
      'reachMinMm',
      'reachMinOverridden',
    ]);
  });

  it('★ 关节限位数值在覆写前后完全不变（不存在任何限位覆写通道）', () => {
    const before = model.joints.map((j) => ({ ...j.limits }));
    store().setTargetGuard({
      reachMaxOverridden: true,
      reachMaxMm: 80,
      jointToleranceDeg: 1,
    });
    store().moveTo([80, 0, 90]);
    expect(model.joints.map((j) => ({ ...j.limits }))).toEqual(before);
  });
});
