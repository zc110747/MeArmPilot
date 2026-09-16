/**
 * 全局唯一状态仓库（Zustand）。
 *
 * 设计约束（spec §十七 / §十九）：
 *   - 虚拟机械臂与真实机械臂共享**同一个 RobotState**；
 *   - 所有状态变化必须经 `RobotState`，不允许组件各自持有局部关节角副本；
 *   - `mode`（simulation / real）与 `controlSource` 分离：前者决定"要不要发给真实机械臂"，
 *     后者决定"当前姿态是谁驱动的"，用于打破虚拟↔真实回环；
 *   - `commandJoints` 与 `actualJoints` 严格区分（spec §二十六 / §三十四）：
 *       command = 我们要求机械臂去的角度
 *       actual  = 机械臂实际处在的角度（Phase 11 起由真实反馈驱动；仿真下等于 command）
 *
 * Phase 6 新增「末端目标」概念，三者的关系：
 *   - `target`        = 我想去哪（XYZ 输入或鼠标拖动指定；**越界也照常保留**）
 *   - `commandJoints` = 实际去哪（恒在限位内；目标越界时**逐位不变**）
 *   - `endEffector`   = commandJoints 的 FK 结果，它与 `target` 的差即"还差多远"
 *
 *   `target` 只在 `moveTo()` 时由用户显式指定。滑杆 / HOME / ZERO 属于**非目标驱动**，
 *   它们会让 `target` 自动跟随新 TCP —— 否则幽灵标记会停在旧位置，误报"未到位"。
 *
 * Phase 7 新增「传输驱动」概念，`actualJoints` 的写入者因此分两种：
 *   - `transportDriven === false`（未连接）：`actual ≡ command`，仿真下立即跟随（Phase 1–6 行为）
 *   - `transportDriven === true` （已连接）：`actualJoints` **只由 Transport 回推写入**，
 *     命令路径不再碰它 —— 否则 Mock 的滞后回推会被下一次命令立刻覆盖，误差永远显示 0
 *
 *   注意 3D 场景渲染的一直是 `commandJoints`（虚拟臂 = 命令的即时预测），
 *   `actualJoints` 只进状态面板，两者分离才能看出"真机跟上了没有"。
 */
import { create } from 'zustand';
import {
  appendFrame,
  DEFAULT_TARGET_GUARD_PARAMS,
  defaultRobotId,
  emptyTrack,
  endEffectorPose,
  homeJointState,
  jointByRole,
  jointIds,
  loadRobot,
  loadRobotModel,
  movableJoints,
  normalizeTargetGuardParams,
  reachMaxOverrideFor,
  TargetGuardParamError,
  zeroJointState,
  type AppendFrameOptions,
  type DragPlaneMode,
  type JointState,
  type RobotModel,
  type TargetGuardParams,
  type TeachTrack,
  type Transform,
  type TransportStats,
  type Vec3,
} from '@robot/index';
// ⚠️ **登记的层次倒置（Phase 2 遗留 · 待 Phase 6/7 裁决）**
//
// 下面这几个是 **MeArm 解析解**的原生符号（`solveIk` 与它的四种原生类型）。
// 严格按分层，Core 不该认识它们 —— 上层应当只走 `RobotRegistry.loadRobot(id)
// .kinematics.inverse()`（统一 `IKResult` 形状）。
//
// 之所以**这一轮先不动**：`ikStatus` 现在暴露的是 MeArm 原生诊断
// （`candidates` / `azimuth` / `relativeAngle` / `joint`），而统一 `IKResult`
// 刻意不带这些字段。改成走引擎会**减少**界面能显示的信息，属于行为变更 ——
// 与「MeArm 冻结优先 / 抽象前后逐位一致」冲突，故按 spec 的要求
// **报告冲突而不是自行扩大范围**。
//
// 已登记的处置路径（Phase 6/7）：给 `IKResult` 加一个可选的 `diagnostics` 袋子，
// 由包内适配器填充，Core 只透传不解释；届时这条 import 才能真正删掉。
//
// ★★ `ikGeometry` / `wristSagittal` 是**后来加的**，理由更强：安全参数的
//    前置检查必须与 `solveIk` 用**同一个 `d`**。自己重算过一次，结果偏了
//    19.4mm（见 `wristDistance()` 的注释）—— 这正是本项目「真值只有一份」
//    铁律要防的事。宁可扩这条边，也不要第二个口径。
//
// 这条边**被一条测试盯着**：`frontend/tests/unit/corePackageBoundary.test.ts`
// 会列出全部"Core → 包"的 import，白名单外的一律失败 —— 于是它不会悄悄繁殖。
import {
  ikGeometry,
  solveIk,
  wristSagittal,
  type IkBranch,
  type IkPreference,
  type IkReason,
  type IkResult,
} from '../../../robot-package/mearm-v1/kinematics/ik';

export type RobotMode = 'simulation' | 'real';
export type RobotControlSource = 'virtual' | 'real' | 'command';

export interface LogEntry {
  id: number;
  time: string;
  // 'err' 是本地告警（例如 mode=real 准入校验失败、安全门拦截命令）——
  // 它不来自链路，所以不能复用 'in'/'out'；用独立 kind 才能在日志里一眼分辨。
  kind: 'in' | 'out' | 'sys' | 'err' | 'warn';
  text: string;
}

/** 最近一次末端目标求解的结果 */
export interface TargetStatus {
  ok: boolean;
  /** 失败原因（`ok === true` 时无） */
  reason?: IkReason;
  /** `JOINT_LIMIT` 时越界的关节 id */
  joint?: string;
  /** 人类可读说明，格式对齐 `docs/serial-v1.md` 的 `ERR JOINT ...` */
  message?: string;
  /** 成功时的解支 */
  branch?: IkBranch;
  /** 成功时的 FK 自查残差（mm） */
  residual?: number;
  /** 成功时的底座方位角（degree） */
  azimuth?: number;
}

export type ToggleKey =
  | 'showJointAxes'
  | 'showJointOrigins'
  | 'showWorldAxes'
  | 'showRobotAxes'
  | 'showActualGhost'
  | 'showTcp';

interface RobotStore {
  /**
   * 当前活动机器人 id（= `config/robots.yaml` 的 key，如 `mearm-v1` / `so-arm101`）。
   *
   * ★ 它**不是** `model.name` / `model.id`：那两者是**模型自己**的字段
   * （MeArm 的 `robot.id` 是 `mearm`），而选择器的 key 是 `mearm-v1` ——
   * 二者恰好不同名，所以绝不能互相反查（见 `resolve_robot_entry_by_config`）。
   *
   * 网页端**没有**模型选择器（spec：只通过配置文件选模型）。这里的初值来自
   * `config/robots.yaml → default`；运行期只有 `setRobot()` 能改它（受守卫）。
   */
  robotId: string;
  model: RobotModel;
  /** 命令关节角（我们要求去的角度） */
  commandJoints: JointState;
  /** 实际关节角（机械臂真实所处角度） */
  actualJoints: JointState;
  /** 命令末端位姿（FK of commandJoints） */
  endEffector: Transform;
  /** 实际末端位姿（FK of actualJoints） */
  actualEndEffector: Transform;

  /** 末端目标点（"我想去哪"，mm）—— 越界时依然保留，便于用户看到差多远 */
  target: Vec3;
  /** 最近一次目标求解结果；滑杆等非目标驱动会清空 */
  ikStatus: TargetStatus | null;
  /** 拖动平面模式（拖动开始时被冻结进 DragHandle 的局部状态） */
  dragPlane: DragPlaneMode;
  /** 是否正在拖动末端（拖动期间需禁用轨道旋转，否则会边转相机边拖） */
  dragging: boolean;

  /**
   * 末端目标「安全参数」（运行期可调，**不是**配置真值）。
   *
   * 三项都**只影响参数、不影响算法**（见 `robot/model/parameterOverrides.ts`）：
   *   · `jointToleranceDeg` 判据容差（默认 0 = 与引入前逐位一致）
   *   · `reachMinOverridden` / `reachMinMm` 球壳内径覆写（默认不覆写）
   *
   * ⚠️ 默认值刻意全部退化为"无覆写" ⇒ 不配置时 `moveTo` 的行为与引入本特性前
   *    逐位相同，Phase 6 的 13 条冻结断言（越界即拒绝、关节逐位不变）继续通过。
   */
  targetGuard: TargetGuardParams;
  /** 末端目标参数被拒绝时的原因（null = 当前参数全部合法） */
  targetGuardError: string | null;

  mode: RobotMode;
  controlSource: RobotControlSource;
  connection: 'connected' | 'disconnected';
  connectionLabel: string;
  /** 已连接的传输类型（`mock` / `websocket`）；null = 未接入 */
  transportKind: string | null;
  /**
   * `actualJoints` 是否由 Transport 回推驱动。
   * 为 true 时命令路径**不再**写 `actualJoints`（见文件头 Phase 7 说明）。
   */
  transportDriven: boolean;
  /** 传输统计（延迟 / 丢帧 / 累计帧数），由 `transportBridge` 定时回写 */
  transportStats: TransportStats | null;

  showJointAxes: boolean;
  showJointOrigins: boolean;
  showWorldAxes: boolean;
  showRobotAxes: boolean;
  /**
   * 是否显示「实际臂」幽灵（Phase 12）。
   *
   * 主臂跟随 `commandJoints`（意图），幽灵跟随 `actualJoints`（现状）。
   * 未被遮挡时露出的部分就是**滞后量** —— 这是"虚拟臂到底跟没跟上真机"最直接的
   * 视觉证据，比读数表更快。收敛时两者重合、幽灵被主臂挡住，等于自动"消失"。
   */
  showActualGhost: boolean;
  showTcp: boolean;

  cameraResetToken: number;
  /** 运行期 FK ↔ Three.js 一致性误差（mm），由场景实时回写（Phase 3 验收的运行态证据） */
  alignmentErrorMm: number | null;
  log: LogEntry[];

  /**
   * 示教轨迹（Phase 13）。
   *
   * 放 store 而不是组件局部状态，理由与 `commandJoints` 相同（本文件头部 §十七）：
   * 它是**机器人相关的状态**，且 e2e 探针要能读到末帧真值来做"回放终点 == 录制终点"
   * 的判定。若留在 `TeachPanel` 内部，探针就只能靠读 DOM 里的 1 位小数文本 —— 
   * 那点精度做不了逐值断言。
   */
  teachTrack: TeachTrack;
  /** 是否正在录制（录 `commandJoints` 的变化） */
  teachRecording: boolean;

  setJoint(jointId: string, angleDeg: number): void;
  setCommandJoints(next: JointState): void;
  setActualJoints(next: JointState): void;
  /**
   * **接管握手**：首次连上链路时，把本地「命令起点」对齐到机器现状。
   *
   * 与 `setActualJoints` 的区别只有一处，但必须分清：`setActualJoints` 是
   * **稳态回推**（只写 actual，绝不碰 command —— 写了就是 `命令→状态→命令` 回环）；
   * 本动作是**一次性握手**，它同时写 command 与 actual，因为此刻**本地根本还没有命令**。
   *
   * 为什么必须有这一次握手 —— 三个后果，第三个最严重：
   *   ① 首次加载画面上会多出一棵半透明的"实际臂幽灵"，与主臂错开 ⇒ 看起来像渲染重影；
   *   ② 关节面板显示的是页面假设的 HOME，而机器其实在别处 ⇒ UI 在说假话；
   *   ③ ★ **首次下发会是一记"跳变"**：命令侧仍是假设的 HOME，用户只是动了 1° 滑杆，
   *      下发的却是 `HOME + 1°` 整组角 ⇒ 真机从"它现在的位置"直接扑向 HOME。
   *      （真值只有 `config/robot.yaml` 一份，但"机器**现在**在哪"只有链路知道。）
   *
   * 只被 `transportBridge` 在**首次连接后的第一帧回推**上调用一次；
   * 重连走的是既有的"补发当前命令"路径（那时用户已经有意图了，不能反过来被覆盖）。
   */
  attachToActual(next: JointState): void;
  goHome(): void;
  goZero(): void;
  /** 求解末端目标并驱动关节；返回 IK 原始结果供调用方分支 */
  moveTo(xyz: Vec3, opts?: { prefer?: IkPreference }): IkResult;
  /** 把目标重置为当前 TCP（即"取消目标"） */
  resetTarget(): void;
  setDragPlane(mode: DragPlaneMode): void;
  setDragging(value: boolean): void;
  /**
   * 设置末端目标安全参数（部分更新）。
   *
   * ⚠️ 参数非法时**拒绝写入**并把原因放进 `targetGuardError`，绝不静默接受 ——
   * 一个越界的容差会让"硬误差 ≤ 2%"这个承诺失效，而界面上看不出任何异常。
   *
   * @returns 是否被接受
   */
  setTargetGuard(patch: Partial<TargetGuardParams>): boolean;
  /** 把末端目标安全参数复位为默认（= 无覆写，行为与引入前逐位一致） */
  resetTargetGuard(): void;
  /**
   * 由 `transportBridge` 调用：登记 / 注销传输连接。
   * `kind === null` 表示断开，`transportDriven` 随之复位。
   */
  setConnection(
    kind: string | null,
    status: 'connected' | 'disconnected',
    label: string,
  ): void;
  setTransportStats(stats: TransportStats | null): void;
  setMode(mode: RobotMode): void;
  setControlSource(source: RobotControlSource): void;
  setToggle(key: ToggleKey, value: boolean): void;
  resetCamera(): void;
  setAlignmentError(value: number): void;
  pushLog(kind: LogEntry['kind'], text: string): void;
  clearLog(): void;

  /** 追加一帧到示教轨迹（`nowMs` 缺省取 `Date.now()`，测试可注入以保持确定性） */
  appendTeachFrame(joints: JointState, nowMs?: number, options?: AppendFrameOptions): void;
  setTeachTrack(track: TeachTrack): void;
  setTeachRecording(value: boolean): void;
  clearTeachTrack(): void;

  /**
   * 切换活动机器人（重载 `model` 并整体复位依赖模型的派生状态）。
   *
   * 守卫：**已接入传输**（`transportDriven`）时**拒绝** —— 模型决定限位与标定，
   * 在驱动真机的当口换模型等于把"能发什么"悄悄换掉。要换请先断开。
   */
  setRobot(robotId: string): SetRobotResult;
}

/** `setRobot` 的结果 —— 拒绝时带上原因，调用方（UI / 测试 / hello 校验）可据此分支 */
export interface SetRobotResult {
  ok: boolean;
  /** 切换后的活动 id（被拒绝时 = 切换前的 id） */
  robotId: string;
  /** 拒绝原因（`ok: true` 时为 undefined） */
  reason?: string;
}

/**
 * 活动机器人 id 的初值。
 *
 * ★ 它来自 **配置文件**（`config/robots.yaml → default`），**不是** UI 开关 ——
 * 这与 spec「只通过后端配置文件选择模型」一致：网页端只**跟随**配置。
 * 后端（Go / Python）默认也读同一个 `default`，所以三端在默认情况下天然一致；
 * 若某一端被显式覆盖（`backend/config.yaml → robot.model_id`），
 * `hello` 在线互检会把不一致**报出来**（见 `transportBridge`）。
 */
const INITIAL_ROBOT_ID = defaultRobotId();

/**
 * 模块级初值 —— **只用于构造 store 的初始 state**。
 *
 * ⚠️ 运行期一律读 `useRobotStore.getState().model` / `get().model`，
 * 不要再把这个常量当"当前模型"用：`setRobot()` 之后它就已经过时了，
 * 而"读了一个过时的模型"在本项目里等于**限位/标定整体错位**，且不会报错。
 */
const initialModel = loadRobotModel(INITIAL_ROBOT_ID);

let logSeq = 0;
/** 已经为哪些机器人记过"没有逆解器"的日志（避免拖动时刷屏） */
const warnedNoSolver = new Set<string>();

// ---------------------------------------------------------------------------
// 末端目标安全参数 —— 辅助（只读计算，不碰算法）
// ---------------------------------------------------------------------------

/** 与 `ik.ts` 的 `EPS_MM` 同量级；用于"是否真的越过内径"的判定 */
const EPS_MM_LOCAL = 1e-9;

/**
 * 目标点 → 腕枢轴到肩枢轴的距离 `d`（mm）。
 *
 * ★★ **一律转发 `ik.ts` 的 `ikGeometry()` + `wristSagittal()`，禁止自己拼几何。**
 *
 * 为什么必须转发（**这是一个已经发生过的事故**）：
 *   本函数最初自己按"`pivotZ` = 沿 parent 链累加、`radial` = TCP 参考关节所辖连杆长"
 *   重算。但 `model.tcp.joint === 'tool'` 时 `wrist.parentLink` 是
 *   **`forearm_link`（80）**，不是 `tool_link`（40）—— 而 `ik.ts` 的
 *   `requireConstantToolOffset()` 用的口径是 `toolOffset[0] = 40`。
 *   于是前置检查报 `d = 92.159` 而 `solveIk` 内部算的是 `d = 111.542`，
 *   **同一个目标点两个 `d`，相差 19.4mm 且不报错** ——
 *   症状是"改小外径后本该可达的点被判越界"（错误信息来源随机：有时是这层，
 *   有时是 `ik.ts` 那层）。违反铁律「同一个量只能有一套口径」。
 *
 * 转发是安全的：`ikGeometry()` 会跨三个姿态交叉验证几何假设，前提不成立时
 * 抛 `IkModelError` 而不是静默算偏；`wristSagittal()` 就是 `solveIk` 自己用的那个。
 *
 * ⚠️ 这条 import 会**扩 Core→包的边**，已在 `corePackageBoundary.test.ts`
 * 的白名单里登记（与既有的 `solveIk` 同一条 import 语句）。
 */
function wristDistance(model: RobotModel, target: Vec3): number {
  const geometry = ikGeometry(model);
  const { dr, dz } = wristSagittal(geometry, target);
  return Math.hypot(dr, dz);
}

/**
 * 球壳 `[内径, 外径]`（mm）—— **直接转发 `ik.ts` 的 `ikGeometry().reach`**。
 *
 * 原实现自己按 `[|l1 − l2|, l1 + l2]` 重算（`l1`/`l2` 取 `shoulder.childLink` /
 * `elbow.childLink` 的长度）。本机两法数值恰好一致（都是 `[0, 160]`），但
 * **它和 `ik.ts` 并不是同一份真值** —— `ikGeometry()` 是从 FK 采样反推
 * （`first.pShoulder` / `requireConstantToolOffset()`），本函数是从连杆表读。
 * 一旦哪天模型改成带偏移的连杆，两法就会分叉且不报错。
 * 按铁律「同一个量只能有一套口径」，这里也改为转发。
 *
 * ⚠️ 用途仅限于**参数校验**（判 `reachMinMm < 外径`），不参与任何解算。
 */
function reachShellOf(model: RobotModel): readonly [number, number] {
  const { reach } = ikGeometry(model);
  return reach;
}

/** 关节状态里第一个 role=base 的关节 id（用于取方位角）；退化取第一个键 */
function baseIdOrFirst(model: RobotModel): string {
  return jointByRole(model, 'base')?.id ?? model.joints[0]?.id ?? '';
}

function fmtVec(p: Vec3): string {
  return `(${p[0].toFixed(3)}, ${p[1].toFixed(3)}, ${p[2].toFixed(3)})`;
}

/**
 * 在容差 `toleranceDeg` 内挑一个"最接近可行"的候选解。
 *
 * 判据与 `ik.ts` 的 `feasible` 同构（`violation <= tolerance`），但**只在
 * 调用方显式开启容差时**才走这条路径 —— 默认 0 时根本不会调用本函数，
 * 于是 Phase 6 的冻结语义（越界即拒绝）不受任何影响。
 *
 * @returns 取 `violation` 最小的那支；全部超出容差时返回 `null`
 */
function pickWithinTolerance<C extends { violation: number; feasible: boolean }>(
  candidates: readonly C[],
  toleranceDeg: number,
): C | null {
  let best: C | null = null;
  for (const candidate of candidates) {
    if (candidate.violation > toleranceDeg) continue;
    if (best === null || candidate.violation < best.violation) best = candidate;
  }
  return best;
}

function makeLogEntry(kind: LogEntry['kind'], text: string): LogEntry {
  logSeq += 1;
  const now = new Date();
  const time = `${now.toTimeString().slice(0, 8)}.${String(now.getMilliseconds()).padStart(3, '0')}`;
  return { id: logSeq, time, kind, text };
}

/** 示教轨迹默认名（带时间戳，导出文件名才有意义） */
function defaultTeachName(): string {
  const d = new Date();
  const p = (n: number) => String(n).padStart(2, '0');
  return `teach-${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

/**
 * 按**给定模型**的关节限位裁剪关节角。
 *
 * ★ `model` 是显式参数而非常量：切换机器人后限位整体不同，
 * 用旧模型的限位去裁新模型的关节会**静默**产出越界角
 * （然后被后端如实拒绝，表现为"命令没反应"）。
 */function clipJointState(model: RobotModel, partial: Partial<JointState>): JointState {
  const out: JointState = {};
  for (const id of jointIds(model)) {
    const joint = model.joints.find((j) => j.id === id);
    const fallback = joint ? joint.limits.min : 0;
    const raw = partial[id] ?? fallback;
    out[id] = joint ? Math.min(joint.limits.max, Math.max(joint.limits.min, raw)) : raw;
  }
  return out;
}

/**
 * 非目标驱动的关节变化（滑杆 / HOME / ZERO）统一出口。
 * 与 `moveTo` 的区别只有两点：清空 `ikStatus`、让 `target` 跟随新 TCP。
 *
 * `transportDriven` 为 true 时**不写** `actualJoints` —— 那时它是 Transport 回推的领地，
 * 命令路径若也去写，就会把回推的滞后值立刻覆盖掉，误差显示永远为 0。
 */
function deriveVirtual(
  model: RobotModel,
  joints: JointState,
  syncTarget: boolean,
  transportDriven: boolean,
): Partial<RobotStore> {
  const pose = endEffectorPose(model, joints);
  const patch: Partial<RobotStore> = {
    commandJoints: joints,
    endEffector: pose,
    controlSource: 'virtual',
  };
  if (!transportDriven) {
    // 未接入传输：仿真下实际值立即跟随命令（Phase 1–6 行为）
    patch.actualJoints = joints;
    patch.actualEndEffector = pose;
  }
  if (syncTarget) {
    patch.target = [pose.position[0], pose.position[1], pose.position[2]];
    patch.ikStatus = null;
  }
  return patch;
}

export const useRobotStore = create<RobotStore>((set, get) => {
  const initial = homeJointState(initialModel);
  const initialPose = endEffectorPose(initialModel, initial);
  return {
    robotId: INITIAL_ROBOT_ID,
    model: initialModel,
    commandJoints: initial,
    actualJoints: initial,
    endEffector: initialPose,
    actualEndEffector: initialPose,

    // 初始把目标放在 HOME 的 TCP 上：幽灵标记与 TCP 重合，场景里不显眼
    target: [initialPose.position[0], initialPose.position[1], initialPose.position[2]],
    ikStatus: null,
    dragPlane: 'xy',
    dragging: false,

    // 末端目标安全参数：默认全部退化为"无覆写"⇒ 不配置时行为与引入前逐位一致
    targetGuard: { ...DEFAULT_TARGET_GUARD_PARAMS },
    targetGuardError: null,

    // 默认 Simulation：防止网页一打开就直接控制真实机械臂（spec §三十一）
    mode: 'simulation',
    controlSource: 'virtual',
    connection: 'disconnected',
    connectionLabel: '未接入（可在 Connection 面板连接 MockTransport）',
    transportKind: null,
    transportDriven: false,
    transportStats: null,

    showJointAxes: true,
    showJointOrigins: false,
    // 世界轴默认**关**：它是挂在世界原点的三根参考线（Z 蓝轴竖直向上，长得像「机械臂中轴」），
    // 机械臂一动就会显出「它不跟着动」。几何上它本就该固定在世界系，问题出在默认显示它。
    // 开关保留（调试坐标系时仍可打开），只是不再默认占用画面。
    showWorldAxes: false,
    showRobotAxes: false,
    showActualGhost: true,
    showTcp: true,
    cameraResetToken: 0,
    alignmentErrorMm: null,
    log: [
      makeLogEntry(
        'sys',
        `RobotModel 载入：${initialModel.name}（${initialModel.links.length} 连杆 / ${initialModel.joints.length} 关节 / ${initialModel.actuators.length} 舵机）` +
          ` · 活动机器人 id = ${INITIAL_ROBOT_ID}`,
      ),
    ],

    // 空示教轨迹（Phase 13）。名字带时间戳，导出文件才有意义
    teachTrack: emptyTrack(defaultTeachName()),
    teachRecording: false,

    setJoint(jointId, angleDeg) {
      const m = get().model;
      const next = clipJointState(m, { ...get().commandJoints, [jointId]: angleDeg });
      set(deriveVirtual(m, next, true, get().transportDriven));
    },

    setCommandJoints(next) {
      const m = get().model;
      set(deriveVirtual(m, clipJointState(m, next), true, get().transportDriven));
    },

    setActualJoints(next) {
      const m = get().model;
      const clipped = clipJointState(m, next);
      set({
        actualJoints: clipped,
        actualEndEffector: endEffectorPose(m, clipped),
        controlSource: 'real',
      });
    },

    attachToActual(next) {
      const m = get().model;
      const clipped = clipJointState(m, next);
      const pose = endEffectorPose(m, clipped);

      // 记下"机器实际离页面的假设有多远"：这个数就是用户此前看到的那只幽灵的
      // 偏移量。写进日志而不是只留在画面上 —— 幽灵消失之后，这条记录是唯一
      // 能事后回答"刚才到底是机器不在 HOME，还是渲染坏了"的东西。
      const previous = get().commandJoints;
      let worstDeg = 0;
      let worstJoint = '';
      for (const id of jointIds(m)) {
        const delta = Math.abs((clipped[id] ?? 0) - (previous[id] ?? 0));
        if (delta > worstDeg) {
          worstDeg = delta;
          worstJoint = id;
        }
      }

      set({
        commandJoints: clipped,
        endEffector: pose,
        // 与 setActualJoints 同一套裁剪与 FK，保证 command 与 actual **逐位相同**
        actualJoints: clipped,
        actualEndEffector: pose,
        // 目标同步到"机器现在的位置"，否则拖动把手会停在页面假设那里
        target: [pose.position[0], pose.position[1], pose.position[2]],
        ikStatus: null,
        controlSource: 'real',
      });

      get().pushLog(
        'sys',
        worstDeg < 1e-9
          ? '接管：机器现状与页面初始位姿一致（无偏差）'
          : `接管：以机器现状为命令起点 —— 与页面初始位姿最大相差 ${worstDeg.toFixed(3)}°（${worstJoint}）`,
      );
    },

    goHome() {
      const m = get().model;
      const home = clipJointState(m, homeJointState(m));
      set(deriveVirtual(m, home, true, get().transportDriven));
      get().pushLog('sys', `HOME 位姿 ${JSON.stringify(home)}`);
    },

    goZero() {
      // 关节空间原点：各关节 0°。小臂（绝对角）的真机可达区间是 108.44..141.86°，
      // 0° 不可达，故 clipJointState 会把它钳到最竖直的可达角 —— 结果是真机约束，不是 bug。
      const m = get().model;
      const zero = clipJointState(m, zeroJointState(m));
      set(deriveVirtual(m, zero, true, get().transportDriven));
      get().pushLog('sys', `零位：各关节 0°（限位钳位后 ${JSON.stringify(zero)}）`);
    },

    moveTo(xyz, opts = {}) {
      const current = get();
      // ⚠️ 能力门与后续的 `clipJointState` 都读**原始模型**（限位与能力是"这台机器人
      //    是什么"，不该被会话参数改写）；只有喂给 `solveIk` 的那份才施加限位收紧。
      const m = current.model;

      // ---- 能力门：本机器人有没有逆解器？ ----
      //
      // ⚠️ 这一层**不能省**：下面的 `solveIk` 是 MeArm 的**解析解**（矢状面 2R），
      // 对着一台 6 铰链的 SO-101 去跑它，得到的是"把一台机器的几何套在另一台上"
      // 的结果 —— 它会**成功返回**一组关节角（`success: true`），而那组角在 SO-101
      // 上毫无意义。这正是 spec 说的"伪造 IK"：不是报错，是静默给出错误的关节角。
      //
      // 判据取自 `RobotRegistry` 的能力**声明**（数据），而不是在这里写
      // `if (robotId === 'so-arm101')`（逻辑）—— 将来 SO-101 加了数值 IK，
      // 只需要改它自己的 `capability`，本函数一个字都不用动。
      const solverKind = loadRobot(current.robotId).kinematics.capability.solverKind;
      if (solverKind !== 'analytic') {
        const message =
          `${m.name} 没有逆解器（solverKind="${solverKind}"），目标点未被求解。` +
          `这不是"目标不可达"——请改用关节角直接驱动（Joint 面板）`;
        // 与"目标越界"同一取向：**保留 target** 让用户看到"我想去哪"，
        // 并在面板上显示原因。绝不静默什么都不做。
        set({
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: { ok: false, reason: 'NO_SOLVER', message },
        });
        // 拖动时 `moveTo` 会被高频调用 ⇒ 只在**首次**为该机器人记一条日志，
        // 否则日志面板会被同一句话刷满（与 `warnNotImplementedOnce` 同一考量）。
        if (!warnedNoSolver.has(current.robotId)) {
          warnedNoSolver.add(current.robotId);
          current.pushLog('err', message);
        }
        return {
          success: false,
          reason: 'NO_SOLVER',
          message,
          candidates: [],
        };
      }

      // `prefer: 'nearest'` + `near: 当前命令角` —— 拖动经过工作空间内边界时不翻支；
      // `seed: 当前命令角` 让夹爪等未参与解算的关节保持原值，返回值可直接喂 FK 闭环。
      const guard = current.targetGuard;

      // ---- 参数层①：球壳内径 / 外径覆写（独立的前置检查） ----
      //
      // ⚠️ 为什么在这里而不是改 `ik.ts`：`reach` 是在 `ik.ts` 的 `ikGeometry()`
      //    内部由杆长算出来的（本机 `[0, 160]`），外部无法覆写；而 `ik.ts` 同时被
      //    sim2sim（bridge 直接调 `solveIk`）与 sim2real 共用，改它是"改算法"。
      //
      // 内径：`reachMinMm ≥ 0.1` 恒大于 `ik.ts` 的原内径 `|l1 − l2| = 0`
      //       （本机 l1 = l2 = 80）⇒ 这道检查**只会更严**，原判据被完全包含。
      //
      // 外径：⚠️ 与内径不同，外径**必须真正参与解算**（经 `opts.reachMaxMm` 传给
      //       `solveIk`，见下方）。这里的前置检查**不是**判据本体，而是为了
      //       给出更好的报错信息：`ik.ts` 在 `d > reachMax` 时返 `OUT_OF_WORKSPACE`
      //       且**不带 candidate**，容差层就没有候选可用，错误信息里也不会有
      //       "是覆写后的外径"这个关键上下文。
      //
      // ⚠️ `dWrist` 只需在任一覆写生效时计算（两处判据共用同一个值）。
      //
      // ⚠️ 副作用（刻意保留）：`solveIk` 被**直接调用**的地方（sim2sim bridge）
      //    既看不到这道检查、也不会传 `opts.reachMaxMm` ⇒ 覆写不会外溢，
      //    符合需求「不影响 sim2sim / sim2real」。
      //
      // ⚠️ 两项覆写都默认 false ⇒ 整个块不进，行为与引入本特性前逐位相同。
      const reachOverridden = guard.reachMinOverridden || guard.reachMaxOverridden;
      const dWrist = reachOverridden ? wristDistance(current.model, xyz) : 0;

      if (reachOverridden && guard.reachMinOverridden && dWrist < guard.reachMinMm - EPS_MM_LOCAL) {
        const message =
          `目标 ${fmtVec(xyz)} 超出工作空间：腕枢轴到肩枢轴的距离 ${dWrist.toFixed(3)}mm ` +
          `小于设定的球壳内径 ${guard.reachMinMm.toFixed(3)}mm（内径覆写生效中）`;
        set({
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: { ok: false, reason: 'OUT_OF_WORKSPACE', message },
        });
        return { success: false, reason: 'OUT_OF_WORKSPACE', message, candidates: [] };
      }

      if (reachOverridden && guard.reachMaxOverridden && dWrist > guard.reachMaxMm + EPS_MM_LOCAL) {
        const message =
          `目标 ${fmtVec(xyz)} 超出工作空间：腕枢轴到肩枢轴的距离 ${dWrist.toFixed(3)}mm ` +
          `大于设定的球壳外径 ${guard.reachMaxMm.toFixed(3)}mm（外径覆写生效中）`;
        set({
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: { ok: false, reason: 'OUT_OF_WORKSPACE', message },
        });
        return { success: false, reason: 'OUT_OF_WORKSPACE', message, candidates: [] };
      }

      // ⚠️ `reachMaxOverrideFor` 不覆写时返回 `undefined` ⇒ `ik.ts` 走原分支
      //    （用 `ikGeometry()` 求导出的 `reach[1]`），逐位一致。
      //    覆写时它替换掉那个外径，于是 2R 的 `cosAlpha` 按新外径求解 ——
      //    这就是"外径覆写真正参与计算"的落点。
      const result = solveIk(current.model, xyz, {
        prefer: opts.prefer ?? 'nearest',
        near: current.commandJoints,
        seed: current.commandJoints,
        reachMaxMm: reachMaxOverrideFor(guard),
      });

      if (result.success) {
        const joints = clipJointState(current.model, result.joints);
        set({
          ...deriveVirtual(current.model, joints, false, current.transportDriven),
          target: [xyz[0], xyz[1], xyz[2]],
          ikStatus: {
            ok: true,
            branch: result.branch,
            residual: result.residual,
            azimuth: result.azimuth,
          },
        });
        return result;
      }

      // ---- 参数层②：判据容差（默认 0 ⇒ 完全走原有分支，行为逐位不变） --------
      //
      // `ik.ts` 的可行判据是 `violation <= 1e-9`，而界面 XYZ 只显示 1 位小数
      // ⇒ 极限点回输必然被拒（实测差 0.0038°）。容差 T 让"差一点点"的解通过，
      // 随后被 `clipJointState` **钳到限位上**，末端偏差 `≈ L·sin(T)`。
      //
      // ⚠️ T 的上限 1.0° 由"硬误差 ≤ 2%"反推（见 `parameterOverrides.ts`）。
      // ⚠️ 默认 T = 0 ⇒ 下面的分支不进，13 条 Phase 6 冻结断言继续通过。
      // ⚠️ 只在 `!result.success` 时才有意义：成功路径已经返回了。
      if (guard.jointToleranceDeg > 0) {
        const relaxed = pickWithinTolerance(result.candidates, guard.jointToleranceDeg);
        if (relaxed) {
          // ⚠️ 钳位是**有意为之**：解出的角可能越界 ≤ T，必须裁回限位才敢下发。
          //    这正是"硬误差"的来源，也是 T 必须 ≤ 1.0° 的原因。
          const joints = clipJointState(current.model, relaxed.joints);
          const achieved = endEffectorPose(current.model, joints).position;
          const residual = Math.hypot(
            achieved[0] - xyz[0],
            achieved[1] - xyz[1],
            achieved[2] - xyz[2],
          );
          const baseId = baseIdOrFirst(current.model);
          const azimuth = joints[baseId] ?? 0;
          set({
            ...deriveVirtual(current.model, joints, false, current.transportDriven),
            target: [xyz[0], xyz[1], xyz[2]],
            ikStatus: { ok: true, branch: relaxed.branch, residual, azimuth },
          });
          return {
            success: true,
            joints,
            branch: relaxed.branch,
            residual,
            azimuth,
            relativeAngle: relaxed.relativeAngle,
          };
        }
      }

      // ⚠️ 目标越界时关节**逐位不变**（Phase 6 已拍板）：保留 target 让用户看到"差多远"，
      //    但绝不静默钳位 —— 钳位会让工作空间边界从界面上消失，也无法保证钳位点满足关节限位。
      set({
        target: [xyz[0], xyz[1], xyz[2]],
        ikStatus: {
          ok: false,
          reason: result.reason,
          joint: result.joint,
          message: result.message,
        },
      });
      return result;
    },

    resetTarget() {
      const pose = get().endEffector.position;
      set({
        target: [pose[0], pose[1], pose[2]],
        ikStatus: null,
      });
    },

    setDragPlane(mode) {
      set({ dragPlane: mode });
    },

    setDragging(value) {
      set({ dragging: value });
    },

    setTargetGuard(patch) {
      const current = get();
      const candidate: TargetGuardParams = { ...current.targetGuard, ...patch };
      try {
        // ⚠️ 校验需要"求导球壳"来判 `reachMinMm < 生效外径` 与
        //    `reachMaxMm ≤ 几何外径`。两者都由连杆长度决定（本机 `l1 + l2 = 160`），
        //    与覆写无关 ⇒ 不需要在这里跑 IK 几何求导，直接用同一来源的链长算。
        const normalized = normalizeTargetGuardParams(candidate, {
          reach: reachShellOf(current.model),
        });
        set({ targetGuard: normalized, targetGuardError: null });
        // ⚠️ 覆写改动必须**立刻**影响视口里的工作空间提示（把手球颜色 / 虚线），
        //    但 `moveTo` 只在"下一个目标"时才跑。所以这里主动把**当前 target**
        //    重解一次：参数一变，判定与显示就同步。
        //    · 默认参数下 `target` 恰好是可达点时，重解结果与原状态一致 ⇒ 无副作用。
        //    · 用 `resetTargetGuard` 复位时不需要（它本就是"回到默认"，且
        //      重解会让"参数复位"意外改动关节）—— 只在本 setter 里做。
        const [tx, ty, tz] = current.target;
        get().moveTo([tx, ty, tz]);
        return true;
      } catch (error) {
        // ⚠️ 拒绝写入而**保留旧值**：一个越界的容差会让"硬误差 ≤ 2%"的承诺失效，
        //    而界面上看不出任何异常。宁可原样不动 + 明说原因。
        const message =
          error instanceof TargetGuardParamError ? error.message : String(error);
        set({ targetGuardError: message });
        return false;
      }
    },

    resetTargetGuard() {
      set({ targetGuard: { ...DEFAULT_TARGET_GUARD_PARAMS }, targetGuardError: null });
    },

    setConnection(kind, status, label) {
      const driven = kind !== null && status === 'connected';
      const patch: Partial<RobotStore> = {
        transportKind: kind,
        transportDriven: driven,
        connection: status,
        connectionLabel: label,
      };
      if (!driven) {
        // 断开后回到「仿真立即跟随」：把 actual 拉回 command 并清掉统计。
        // 不这么做的话，最后那次回推的滞后值会永久挂在状态面板上显示一个假误差
        // —— 因为已经没有任何 transport 再去驱动它收敛了。
        const joints = get().commandJoints;
        patch.transportStats = null;
        patch.actualJoints = joints;
        patch.actualEndEffector = endEffectorPose(get().model, joints);
        patch.controlSource = 'virtual';
      }
      set(patch);
    },

    setTransportStats(stats) {
      set({ transportStats: stats });
    },

    setMode(mode) {
      // ⚠️ 这不是一个"纯 UI 开关"，也不是"只发告警的意图"。
      //
      // 规范（本文件头部 §十七）要求 `mode` **决定"要不要发给真实机械臂"**。
      // 这个 getter 经历过两轮修正，每一轮都留下了教训：
      //
      //   v1（Phase 6）只写 `set({ mode })` —— 点 Real Robot 命令仍走当前
      //     transport，真机不动而 UI 显示"真实机械臂"（静默失败，见 D41）。
      //
      //   v2（Phase 9 / D41）加了准入校验，但 `set({ mode })` 仍留在校验**之前**
      //     ⇒ 校验只是"发一条 err 日志"，mode 照样变成 real。用户看到的仍是
      //     「Real 模式」高亮的按钮 + 一行小字说末端是 sim（见 D43）。
      //
      //   v3（现在）**校验通过才改状态**：不具备驱动真机的条件时，mode 保持
      //     simulation，按钮回弹到 Simulation。所见即所是。
      //
      // 「校验失败就拒绝」的代价是：用户点按钮可能"没反应"。所以每条拒绝
      // 都必须 pushLog('err') 说明**为什么**和**怎么修** —— 拒绝不是目的，
      // 让用户知道当前到底在驱动谁才是。
      if (mode !== 'real') {
        set({ mode });
        get().pushLog('sys', '切换到 Simulation（命令不再下发给真实机械臂）');
        return;
      }

      // ---- mode === 'real' 的准入校验：全部通过才真正切换 ----
      const st = get();
      const stats = st.transportStats;
      const device = stats && 'device' in stats ? (stats.device as string | null) : null;

      if (st.transportKind === null || !st.transportDriven) {
        // 没有连接：命令无处可去。这是"点了 Real Robot 但真机不动"的第一大原因。
        get().pushLog(
          'err',
          'Real Robot 未启用：当前**未连接**任何传输，保持 Simulation。' +
            '请先在 Connection 面板连接后端（真机需用 config.serial.yaml 启动 armpilot-backend）',
        );
        return;
      }

      if (st.transportKind === 'mock') {
        get().pushLog(
          'err',
          'Real Robot 未启用：当前连接的是 **MockTransport**（浏览器内仿真，不碰硬件），' +
            '保持 Simulation。请在 Connection 面板切到 WebSocket 并连接真机后端',
        );
        return;
      }

      if (device === null) {
        // 已连上 WebSocket，但 hello 尚未到达 ⇒ **末端还未知**。
        //
        // 早期版本在这里"乐观放行"，理由是"宁可少拦"。实践证明这个理由在
        // **UI 切换**这个场景下是错的：`--real` 自动切换常常抢在 hello 之前，
        // 于是切换"成功"了，可末端其实是 sim —— 用户就看到了本次报障的现象。
        // 现在改为**拒绝并说明**：让调用方（自动连接）等 hello 后再来。
        get().pushLog(
          'err',
          'Real Robot 未启用：尚未收到后端 hello，**链路末端未知**，为安全起见保持 Simulation。' +
            '请稍候重试（连接建立后约 1 秒内会到达）',
        );
        return;
      }

      if (device !== 'serial') {
        // 后端连上了，但它自己也没接真机（device=sim 表示用的是内置假固件）。
        get().pushLog(
          'err',
          `Real Robot 未启用：后端链路末端是「${device}」而非 serial，保持 Simulation。` +
            '请用 config.serial.yaml 启动后端（并把机械臂接到配置的串口）',
        );
        return;
      }

      // ---- 全部通过：真正切换 ----
      set({ mode: 'real' });
      get().pushLog(
        'sys',
        `切换到 Real Robot：命令将下发给真实机械臂（链路末端 serial）`,
      );
    },

    setRobot(robotId) {
      const st = get();

      if (robotId === st.robotId) {
        // 幂等：重复切到同一台只回结果，不动任何状态。
        // 这一条不只是省事 —— P8 的切换压力回归会把它调上万次，
        // 若每次都重算/重写状态，压测本身就成了噪声源。
        return { ok: true, robotId };
      }

      // ---- 守卫：已接入传输时拒绝 ----
      //
      // 与 `setMode` 同一套哲学（本文件头部 §十七）：**校验通过才改状态**，
      // 拒绝时给出一条说明"为什么"和"怎么修"的日志 —— 而不是改了状态再抱怨。
      if (st.transportDriven) {
        const reason =
          `机器人切换被拒绝：当前已接入传输（${st.transportKind ?? '未知'}），` +
          `模型决定限位与标定，在驱动链路上换模型会把"能发什么"悄悄换掉。` +
          `请先在 Connection 面板断开，再切到 ${robotId}`;
        st.pushLog('err', reason);
        return { ok: false, robotId: st.robotId, reason };
      }

      let next: RobotModel;
      try {
        next = loadRobotModel(robotId);
      } catch (error) {
        // 未知 id / yaml 损坏：如实报错并**保持原模型**。
        // 绝不回退到缺省 —— 那会把"配置写错一个字"变成"静默加载了另一台机器人"。
        const reason = `机器人切换失败：${(error as Error).message}`;
        st.pushLog('err', reason);
        return { ok: false, robotId: st.robotId, reason };
      }

      const home = clipJointState(next, homeJointState(next));
      const pose = endEffectorPose(next, home);

      set({
        robotId,
        model: next,
        commandJoints: home,
        actualJoints: home,
        endEffector: pose,
        actualEndEffector: pose,
        // 目标 / IK 结果 / 对齐误差 / 示教轨迹全部是**旧模型关节空间**里的量，
        // 新模型下它们没有任何意义 ⇒ 一律复位。
        // （示教轨迹是用户数据，所以下面单独记一条日志，不能悄悄丢。）
        target: [pose.position[0], pose.position[1], pose.position[2]],
        ikStatus: null,
        alignmentErrorMm: null,
        teachTrack: emptyTrack(st.teachTrack.name),
        teachRecording: false,
      });

      const droppedFrames = st.teachTrack.frames.length;
      st.pushLog(
        'sys',
        `机器人已切换：${next.name}（id=${robotId}）· ${next.links.length} 连杆 / ` +
          `${next.joints.length} 关节 / ${next.actuators.length} 舵机 · 姿态复位到 HOME`,
      );
      if (droppedFrames > 0) {
        // 示教轨迹是**用户数据**，不能悄悄丢 —— 它记录的是旧模型的关节空间，
        // 换模型后回放会驱动一组语义完全不同的关节。
        st.pushLog(
          'warn',
          `示教轨迹已清空（原有 ${droppedFrames} 帧）：它记录的是 ${st.model.name} 的关节空间，` +
            `在 ${next.name} 上回放会驱动语义不同的关节。`,
        );
      }
      return { ok: true, robotId };
    },

    setControlSource(source) {
      set({ controlSource: source });
    },

    setToggle(key, value) {
      set({ [key]: value } as unknown as Partial<RobotStore>);
    },

    resetCamera() {
      set((state) => ({ cameraResetToken: state.cameraResetToken + 1 }));
    },

    setAlignmentError(value) {
      set({ alignmentErrorMm: value });
    },

    pushLog(kind, text) {
      set((state) => ({ log: [...state.log.slice(-199), makeLogEntry(kind, text)] }));
    },

    clearLog() {
      set({ log: [] });
    },

    appendTeachFrame(joints, nowMs, options) {
      // 复用纯函数做节流 / 去抖 / 上限判定；这里只负责把它接进 store。
      // 跳过时 `appendFrame` 返回**同一个引用** ⇒ set 之后 React 直接 bail out，
      // 不会因为"每帧都新对象"而让订阅者重渲染。
      set((state) => ({ teachTrack: appendFrame(state.teachTrack, joints, nowMs ?? Date.now(), options) }));
    },

    setTeachTrack(track) {
      set({ teachTrack: track });
    },

    setTeachRecording(value) {
      set({ teachRecording: value });
    },

    clearTeachTrack() {
      set((state) => ({
        teachTrack: emptyTrack(state.teachTrack.name),
        teachRecording: false,
      }));
    },
  };
});

// ---------------------------------------------------------------------------
// 派生选择器（供组件使用，避免在组件里重复写业务规则）
//
// ★ 以下函数的 `model` 都是**可选参数**，缺省取当前活动模型。
//   这样既保住了既有调用点（`jointLabel(joint.id)`）的形态，
//   又允许在多机器人语境下显式传入 —— 也让测试能在**不碰全局 store**的情况下
//   对指定模型做断言。
// ---------------------------------------------------------------------------

/**
 * 当前活动模型。
 *
 * ⚠️ 这是"读一次当时的值"，**不是** React 订阅。组件里若要随模型变化重渲染，
 * 必须订阅 `useRobotStore((s) => s.model)`（`RobotArm` / `ActualGhostArm` 就是这么做的）。
 */
export function activeRobotModel(): RobotModel {
  return useRobotStore.getState().model;
}

/** 关节显示名：J1/J2/J3 + Gripper（无 role 的关节回落到模型自己的名字） */
export function jointLabel(jointId: string, model: RobotModel = activeRobotModel()): string {
  const joint = model.joints.find((j) => j.id === jointId);
  if (!joint) return jointId;
  switch (joint.role) {
    case 'base':
      return 'J1 Base';
    case 'shoulder':
      return 'J2 Shoulder';
    case 'elbow':
      return 'J3 Elbow';
    case 'gripper':
      return 'Gripper';
    default:
      return joint.name;
  }
}

/**
 * 定位关节（不含夹爪）—— IK 只解这三个（spec §十三）。
 *
 * ⚠️ 这是 MeArm 的语义（三自由度定位）。SO-101 有 5 个可动关节，
 * 它的 IK 是 `NOT_IMPLEMENTED`，本函数对它的返回值**没有**"IK 输入"这层含义，
 * 仅供 UI 分组使用（见 `SoArm101Kinematics.capability`）。
 */
export function positioningJointIds(model: RobotModel = activeRobotModel()): string[] {
  return movableJoints(model)
    .filter((joint) => joint.role !== 'gripper')
    .map((joint) => joint.id);
}

export function gripperJointId(model: RobotModel = activeRobotModel()): string | null {
  return jointByRole(model, 'gripper')?.id ?? null;
}

/** 目标点与当前 TCP 的距离（mm）—— 0 表示已到位 */
export function targetGapMm(
  target: Vec3,
  joints: JointState,
  model: RobotModel = activeRobotModel(),
): number {
  const tcp = endEffectorPose(model, joints).position;
  return Math.hypot(target[0] - tcp[0], target[1] - tcp[1], target[2] - tcp[2]);
}
