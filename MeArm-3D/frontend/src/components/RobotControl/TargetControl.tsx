/**
 * 末端目标控制（Phase 6）：XYZ 直输 + 六向 Jog + 拖动平面选择 + IK 状态。
 *
 * 组件内**不存任何关节副本**；一切经 `store.moveTo()` 落到 RobotState。
 * XYZ 输入框是个"草稿"：正在编辑时不被拖动结果覆盖，避免手被打断。
 *
 * ## 安全参数（可折叠区）
 *
 * 三项**只影响参数、不影响算法**的量：
 *   · 判据容差 T（默认 0）——解决"拖到极限 → 记录 XYZ → 手输被拒"
 *   · 球壳内径覆写（默认关，最小 0.1mm）
 *   · 球壳外径覆写（默认关，1~160mm）——**真正参与解算**
 * 全部落进 `store.targetGuard`，由 `moveTo` 读取。详见
 * `robot/model/parameterOverrides.ts`。
 *
 * ⚠️ 「关节限位调节」已整段移除（实测本机限位余量 0.000000°，只能向内收紧，
 *    而向内收紧解决不了"手输被拒"—— 那是判据容差的事）。理由见参数层文件头。
 */
import { useEffect, useRef, useState } from 'react';
import { DRAG_PLANE_MODES, dragPlaneLabel, type DragPlaneMode, type RobotModel } from '@robot/index';
import {
  JOINT_TOLERANCE_MAX_DEG,
  MEASURED_MAX_RADIUS_MM,
  REACH_MAX_CEILING_MM,
  REACH_MAX_FLOOR_MM,
  REACH_MIN_FLOOR_MM,
} from '@robot/model/parameterOverrides';
import { useRobotStore } from '@/store/robotStore';

const JOG_STEP_MM = 10;

type Draft = [string, string, string];

function toDraft(v: [number, number, number]): Draft {
  return [v[0].toFixed(1), v[1].toFixed(1), v[2].toFixed(1)];
}

export function TargetControl() {
  const target = useRobotStore((s) => s.target);
  const moveTo = useRobotStore((s) => s.moveTo);
  const resetTarget = useRobotStore((s) => s.resetTarget);
  const ikStatus = useRobotStore((s) => s.ikStatus);
  const dragPlane = useRobotStore((s) => s.dragPlane);
  const setDragPlane = useRobotStore((s) => s.setDragPlane);
  const dragging = useRobotStore((s) => s.dragging);
  const targetGuard = useRobotStore((s) => s.targetGuard);
  const targetGuardError = useRobotStore((s) => s.targetGuardError);
  const setTargetGuard = useRobotStore((s) => s.setTargetGuard);
  const resetTargetGuard = useRobotStore((s) => s.resetTargetGuard);
  const model = useRobotStore((s) => s.model);

  const [draft, setDraft] = useState<Draft>(() => toDraft(target));
  const [guardOpen, setGuardOpen] = useState(false);
  const editingRef = useRef(false);

  // 拖动 / Jog / HOME 改变了 target 时刷新输入框；但用户正在输入时不打断
  useEffect(() => {
    if (editingRef.current) return;
    setDraft(toDraft(target));
  }, [target]);

  const moveToXyz = (xyz: [number, number, number]) => {
    const result = moveTo(xyz);
    if (!result.success) {
      console.info(`[target] ${result.reason}${result.joint ? ` @ ${result.joint}` : ''}: ${result.message}`);
    }
  };

  const commit = () => {
    const parsed = draft.map((s) => Number(s)) as [number, number, number];
    if (parsed.some((v) => !Number.isFinite(v))) return;
    editingRef.current = false;
    moveToXyz(parsed);
  };

  const jog = (axis: 0 | 1 | 2, delta: number) => {
    const next: [number, number, number] = [target[0], target[1], target[2]];
    next[axis] = Math.round((next[axis] + delta) * 10) / 10;
    moveToXyz(next);
  };

  // ---- 安全参数区的展示量（全部从模型现取，不写死） ----
  //
  // 只用于**显示与输入范围约束**，不参与任何解算 —— 解算的几何量始终由
  // `ik.ts` 的 `ikGeometry()` 求导。
  const shell = reachShell(model);
  // 外径上限取「几何外径」与常量上限的较小者（超出 2R 无解，见参数层注释）
  const maxCeiling = Math.min(REACH_MAX_CEILING_MM, shell[1]);
  // 内径的上界取决于**生效的**外径（覆写时用它）
  const effectiveMax = targetGuard.reachMaxOverridden ? targetGuard.reachMaxMm : shell[1];

  return (
    <div className="card">
      <h2>末端目标 · Target XYZ</h2>

      <div className="target-row">
        {(['X', 'Y', 'Z'] as const).map((axis, index) => (
          <label className="target-field" key={axis}>
            <span className="name">{axis}</span>
            <input
              type="number"
              step={1}
              value={draft[index]}
              onFocus={() => {
                editingRef.current = true;
              }}
              onBlur={() => {
                editingRef.current = false;
              }}
              onChange={(event) => {
                const next = [...draft] as Draft;
                next[index] = event.target.value;
                setDraft(next);
              }}
              onKeyDown={(event) => {
                if (event.key === 'Enter') commit();
              }}
            />
          </label>
        ))}
      </div>

      <div className="btn-row">
        <button type="button" onClick={commit}>
          Move
        </button>
        <button type="button" onClick={resetTarget}>
          Use TCP
        </button>
      </div>

      <div className="jog-grid">
        <button type="button" disabled={dragging} onClick={() => jog(0, -JOG_STEP_MM)}>
          X −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(0, JOG_STEP_MM)}>
          X +
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(1, -JOG_STEP_MM)}>
          Y −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(1, JOG_STEP_MM)}>
          Y +
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(2, -JOG_STEP_MM)}>
          Z −
        </button>
        <button type="button" disabled={dragging} onClick={() => jog(2, JOG_STEP_MM)}>
          Z +
        </button>
      </div>

      <div className="kv">
        <span>拖动平面</span>
        <select
          value={dragPlane}
          onChange={(event) => setDragPlane(event.target.value as DragPlaneMode)}
        >
          {DRAG_PLANE_MODES.map((mode) => (
            <option value={mode} key={mode}>
              {dragPlaneLabel(mode)}
            </option>
          ))}
        </select>
      </div>

      {ikStatus === null ? (
        <div className="ik-status dim">未指定目标（滑杆 / HOME / ZERO 驱动）</div>
      ) : ikStatus.ok ? (
        <div className="ik-status ok">
          OK · {ikStatus.branch} · 残差 {ikStatus.residual?.toExponential(1)} mm · 方位{' '}
          {ikStatus.azimuth?.toFixed(1)}°
        </div>
      ) : (
        <>
          <div className="ik-status bad">
            {ikStatus.reason}
            {ikStatus.joint ? ` @ ${ikStatus.joint}` : ''}
          </div>
          <div className="ik-status dim">{ikStatus.message}</div>
        </>
      )}

      <div className="dim" style={{ marginTop: 6 }}>
        {dragging
          ? `拖动中…平面已冻结为「${dragPlaneLabel(dragPlane)}」`
          : '在场景中拖动蓝色半透明球即可移动末端'}
      </div>

      {/* ---- 安全参数（只影响参数，不影响算法） ---- */}
      <div className="guard-head">
        <button
          type="button"
          className="guard-toggle"
          onClick={() => setGuardOpen((v) => !v)}
          aria-expanded={guardOpen}
        >
          {guardOpen ? '▾' : '▸'} 安全参数
        </button>
        <span className="dim">
          {guardSummary(targetGuard, shell)}
        </span>
      </div>

      {guardOpen && (
        <div className="guard-body">
          {/* ① 判据容差 */}
          <div className="guard-row">
            <span className="guard-label">判据容差</span>
            <input
              type="number"
              step={0.1}
              min={0}
              max={JOINT_TOLERANCE_MAX_DEG}
              value={targetGuard.jointToleranceDeg}
              onChange={(e) =>
                setTargetGuard({ jointToleranceDeg: Number(e.target.value) })
              }
            />
            <span className="guard-unit">°</span>
          </div>
          <div className="dim guard-note">
            0 = 与原始判据一致（越界即拒绝）。放宽后越界 ≤ T 的解会被接受并钳位，
            末端偏差 ≈ L·sin(T)；T = {JOINT_TOLERANCE_MAX_DEG}° 时最坏{' '}
            {worstClampPercent(JOINT_TOLERANCE_MAX_DEG, shell[1]).toFixed(2)}%（上限 2%）。
          </div>

          {/* ② 球壳内径覆写 */}
          <div className="guard-row">
            <label className="guard-check">
              <input
                type="checkbox"
                checked={targetGuard.reachMinOverridden}
                onChange={(e) => setTargetGuard({ reachMinOverridden: e.target.checked })}
              />
              <span>球壳内径覆写</span>
            </label>
            <input
              type="number"
              step={1}
              min={REACH_MIN_FLOOR_MM}
              max={Math.max(REACH_MIN_FLOOR_MM, effectiveMax - 0.1)}
              disabled={!targetGuard.reachMinOverridden}
              value={targetGuard.reachMinMm}
              onChange={(e) => setTargetGuard({ reachMinMm: Number(e.target.value) })}
            />
            <span className="guard-unit">mm</span>
          </div>
          <div className="dim guard-note">
            求导内径 {shell[0].toFixed(3)} / 外径 {shell[1].toFixed(3)} mm。
            最小 {REACH_MIN_FLOOR_MM}（0 会落到两杆对折的退化位形）。
            {targetGuard.reachMinOverridden
              ? ` 生效中：内径收紧到 ${targetGuard.reachMinMm}mm，外径不变。`
              : ' 未启用：使用求导值。'}
          </div>

          {/* ③ 球壳外径覆写（真正参与解算） */}
          <div className="guard-row">
            <label className="guard-check">
              <input
                type="checkbox"
                checked={targetGuard.reachMaxOverridden}
                onChange={(e) => setTargetGuard({ reachMaxOverridden: e.target.checked })}
              />
              <span>球壳外径覆写</span>
            </label>
            <input
              type="number"
              step={1}
              min={REACH_MAX_FLOOR_MM}
              max={maxCeiling}
              disabled={!targetGuard.reachMaxOverridden}
              value={targetGuard.reachMaxMm}
              onChange={(e) => setTargetGuard({ reachMaxMm: Number(e.target.value) })}
            />
            <span className="guard-unit">mm</span>
          </div>
          <div className="dim guard-note">
            范围 {REACH_MAX_FLOOR_MM} ~ {maxCeiling.toFixed(0)} mm（求导外径{' '}
            {shell[1].toFixed(3)}）。启用后**参与解算**：2R 按这个臂展求解，
            超出它的目标直接判 OUT_OF_WORKSPACE；范围内与原行为逐位一致。
            {targetGuard.reachMaxOverridden
              ? ` 生效中：当前臂展 ${targetGuard.reachMaxMm.toFixed(0)}mm。`
              : ' 未启用：使用求导值。'}
          </div>

          <div className="btn-row">
            <button type="button" onClick={resetTargetGuard}>
              复位参数
            </button>
          </div>

          {targetGuardError && (
            <div className="ik-status bad" style={{ marginTop: 6 }}>
              {targetGuardError}
            </div>
          )}
        </div>
      )}
    </div>
  );
}

// ---------------------------------------------------------------------------
// 展示用的小工具（只读，不参与解算）
// ---------------------------------------------------------------------------

/** 球壳 `[内径, 外径]`（mm）——与 `ik.ts` 同一算法，仅用于显示 / 输入约束 */
function reachShell(model: RobotModel): [number, number] {
  const shoulder = model.joints.find((j) => j.role === 'shoulder');
  const elbow = model.joints.find((j) => j.role === 'elbow');
  if (!shoulder || !elbow) return [0, Number.POSITIVE_INFINITY];
  const l1 = model.links.find((l) => l.id === shoulder.childLink)?.length ?? 0;
  const l2 = model.links.find((l) => l.id === elbow.childLink)?.length ?? 0;
  return [Math.abs(l1 - l2), l1 + l2];
}

/** 容差 T 对应的最坏钳位误差（百分比）——只用于显示 */
function worstClampPercent(toleranceDeg: number, reachMax: number): number {
  if (!Number.isFinite(toleranceDeg) || toleranceDeg <= 0) return 0;
  if (!Number.isFinite(reachMax) || reachMax <= 0) return 0;
  // ⚠️ 最坏力臂 = l1 + l2 = 球壳**外径**（腕枢轴到 TCP 的 40mm 已含在 dr/dz 里，
  //    不额外加）。**不能**把 reachMax 既当力臂又当分母 —— 那样会自相消掉，
  //    对外径 50 的机器人给出 0.49%（正确值仍是 1.577%）。
  const momentArm = reachMax;
  const displacement = momentArm * Math.sin((toleranceDeg * Math.PI) / 180);
  // 基准取实测最大可达半径（`ws_scan_mearm_v1.py`），与验收断言同源
  return (displacement / MEASURED_MAX_RADIUS_MM) * 100;
}

/** 折叠状态下的参数摘要 */
function guardSummary(
  params: {
    jointToleranceDeg: number;
    reachMinOverridden: boolean;
    reachMinMm: number;
    reachMaxOverridden: boolean;
    reachMaxMm: number;
  },
  shell: [number, number],
): string {
  const parts: string[] = [];
  if (params.jointToleranceDeg > 0) parts.push(`容差 ${params.jointToleranceDeg}°`);
  if (params.reachMinOverridden) parts.push(`内径 ${params.reachMinMm}mm`);
  if (params.reachMaxOverridden) parts.push(`外径 ${params.reachMaxMm}mm`);
  if (parts.length === 0) return `默认（内径 ${shell[0].toFixed(1)}mm，容差 0）`;
  return parts.join(' · ');
}
