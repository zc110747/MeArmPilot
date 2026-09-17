/**
 * RobotState —— 机器人唯一状态载体（spec §十七）。
 *
 * 任何机器人状态变化（虚拟操作 / 真实反馈 / 命令回显）都必须经过本结构，
 * 它是虚拟机械臂与真实机械臂之间同步的**唯一交换格式**。
 *
 * ```
 *   Virtual Robot  ←→  RobotState  ←→  Real Robot
 * ```
 *
 * `source` 用于打破「虚拟 → 真实 → 虚拟」的无限回环（spec §十九）：
 *   - `virtual`：用户在 3D 场景 / 滑杆上操作产生的状态
 *   - `command`：命令下发链路产生的状态（已发往真实机械臂，等待回执）
 *   - `real`   ：真实机械臂回传的状态（权威值，用于反向同步虚拟机械臂）
 */
import type { ControlSource, JointState, Pose } from './Pose';

/**
 * 这帧状态**由谁引起**（对应后端 `protocol.OriginCommand` / `OriginDevice` /
 * `OriginExternal`）。
 *
 * 为什么需要它：设备侧会**自主变化** —— 硬件摇杆、红外遥控、面板手拧，
 * 都不经过本机命令。前端要不要让命令侧跟着走，只取决于这一点。
 *
 * ⚠️ 绝不要改成"比较 Actual 与 Command 就跟随"：拖动时设备还在斜坡上，
 * Actual 必然落后于 Command，那种判据会把命令侧一路拉回半路位置
 * （`命令 → 状态 → 命令` 回环，spec §十九 明令禁止）。
 *
 * ⚠️ 名字不能叫 `JointOrigin`：`model/Joint.ts` 已经导出同名类型
 * （那是关节的局部坐标系原点），`robot/index.ts` 的 `export *` 会撞名。
 */
export type StateOrigin = 'command' | 'device' | 'external';

export interface RobotState {
  joints: JointState;
  endEffector: Pose;
  timestamp: number;
  source: ControlSource;
  /**
   * 缺省按 `'command'` 处理（老后端不发这个字段，行为与引入前一致：
   * 只更新 Actual，不让命令侧跟随）。
   */
  origin?: StateOrigin;
}

export function createRobotState(
  joints: JointState,
  endEffector: Pose,
  source: ControlSource,
  timestamp: number = Date.now(),
  origin?: StateOrigin,
): RobotState {
  return { joints: { ...joints }, endEffector, timestamp, source, origin };
}

export function cloneRobotState(state: RobotState): RobotState {
  return {
    joints: { ...state.joints },
    endEffector: {
      position: [...state.endEffector.position],
      rotation: [...state.endEffector.rotation],
    },
    timestamp: state.timestamp,
    source: state.source,
    origin: state.origin,
  };
}

/** 两组关节角是否一致（每轴容差 eps 度） */
export function jointStatesEqual(a: JointState, b: JointState, eps = 1e-6): boolean {
  const keys = new Set([...Object.keys(a), ...Object.keys(b)]);
  for (const key of keys) {
    if (Math.abs((a[key] ?? 0) - (b[key] ?? 0)) > eps) return false;
  }
  return true;
}

/**
 * 逐关节误差（spec §三十四）：`Error = Actual − Command`。
 * 无位置反馈的舵机（MG90S 无回读）下 actual ≡ command，误差恒为 0；
 * 一旦固件具备回读能力，本函数无需改动即可显示真实误差。
 */
export function jointStateError(command: JointState, actual: JointState): JointState {
  const out: JointState = {};
  for (const key of Object.keys(command)) {
    out[key] = (actual[key] ?? command[key] ?? 0) - (command[key] ?? 0);
  }
  return out;
}

/** 末端位置误差（mm） */
export function positionError(a: Pose, b: Pose): number {
  return Math.hypot(
    a.position[0] - b.position[0],
    a.position[1] - b.position[1],
    a.position[2] - b.position[2],
  );
}
