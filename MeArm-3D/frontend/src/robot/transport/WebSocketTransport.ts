/**
 * WebSocketTransport —— 浏览器 ↔ Go 后端的关节级传输（spec §二十一，Phase 8）。
 *
 * ```
 *   RobotState ─→ RobotTransport ─→ MockTransport        （Phase 7：纯虚拟闭环）
 *                              └─→ WebSocketTransport    （Phase 8：本文件）
 * ```
 *
 * 它只做四件事，每件都有必须做对的细节：
 *
 * 1. **编解码**：`joint_command` 下行、`joint_state` / `error` 上行。坏消息一律
 *    **丢弃并报错**，绝不把 `undefined` 关节角写进状态 —— 一条脏数据会把 Actual 变成 NaN。
 *
 * 2. **心跳（两层，缺一不可）**
 *    - 应用层：我们发 `{"type":"ping"}`、等 `pong`，超时判链路死。
 *      （浏览器 WebSocket API 不暴露传输层 ping，所以只能自己做。）
 *    - 传输层：服务端发 RFC6455 ping，浏览器自动回 pong，由服务端看门狗判死。
 *
 * 3. **重连退避**：指数退避（base → 2× → 4× …，封顶 max）。显式 `disconnect()`
 *    **不**触发重连 —— 否则用户点了"断开"却看到它自己连回来。
 *
 * 4. **模型一致性校验**：`hello` 里带后端读到的限位/标定真值，与本地 `RobotModel` 比对。
 *    不一致立刻报警 —— 这是"标定表只有一份"这条铁律唯一能被在线检验的地方。
 *
 * 5. **链路精度即误差下限**：`JR` 线只能表达 0.1°（`docs/serial-v1.md` §4）。
 *    跟踪误差必须在**线上精度**上定义，否则量化残差会伪装成"永不收敛的误差"。
 *
 * ⚠️ 与 MockTransport 一样，回推的状态**只描述"机械臂现在在哪"**，
 * 绝不回写命令，否则形成 `命令→状态→命令` 无限回环（spec §十九）。
 */
import { endEffectorPose } from '../kinematics/fk';
import type { JointState } from '../model/Pose';
import type { RobotModel } from '../model/RobotModel';
import { createRobotState, type RobotState } from '../model/RobotState';
import type {
  RobotTransport,
  TransportStats,
  TransportStatus,
  TransportStatusDetail,
  Unsubscribe,
} from './RobotTransport';
import { SOCKET_OPEN, webSocketFactory, type SocketFactory, type SocketLike } from './socket';
import { realTimer, type TimerLike } from './timer';
import {
  ORIGIN_DEVICE,
  ORIGIN_EXTERNAL,
  SERVER_DEVICE_STATUS,
  SERVER_ERROR,
  SERVER_HELLO,
  SERVER_JOINT_STATE,
  SERVER_PONG,
  decodeServer,
  describeModelMismatch,
  encodeJointCommand,
  encodePing,
  encodeStatusRequest,
  quantizeForWire,
  quantizeViaServo,
  type BackendModelInfo,
  type ServerEnvelope,
} from './wsProtocol';

/** 关节角"已到位"阈值（度）—— sim：模型内部精确收敛，故取极小值 */
const ARRIVED_EPS_DEG = 1e-6;

/**
 * 真机（`device === 'serial'`）的"已到位"阈值（度）。
 *
 * 真机残差有两个来源：
 *   ① 固件舵机角是**整数** —— 关节侧最大 0.347°（S7 scale 1.44018）。
 *      这一项已经被 `quantizeViaServo` 在比较基准里**精确抵消**了，
 *      不属于"未到位"，所以这里不再为它留余量。
 *   ② 后端把 `STATE` 格式化成 2 位小数 —— 残差 ≤0.005°。
 *
 * 取 0.02°：只为吸收 ②，比 ① 小 17 倍以上。也就是说它
 * **不会掩盖任何物理量化误差**，仅避免"两边其实是同一个位置、
 * 只因打印精度不同而被判成还没到位"。
 */
const ARRIVED_EPS_SERIAL_DEG = 0.02;

export interface WebSocketTransportOptions {
  /** 后端地址，如 `ws://localhost:8090/ws/joint` */
  url: string;
  /** 本地模型（用于 FK 与 hello 一致性校验）；必传 */
  model: RobotModel;
  /** 可注入的 socket 工厂（测试用假 socket） */
  socketFactory?: SocketFactory;
  /** 可注入时钟（测试用虚拟时钟）；所有定时器与时间戳都走它 */
  timer?: TimerLike;
  /** 应用层心跳间隔（ms） */
  heartbeatIntervalMs?: number;
  /** 心跳超时（ms）：发出 ping 后多久没等到 pong 就判死 */
  heartbeatTimeoutMs?: number;
  /** 连接超时（ms）：握手迟迟不完成 */
  connectTimeoutMs?: number;
  /** 重连退避基数（ms） */
  reconnectBaseMs?: number;
  /** 重连退避上限（ms） */
  reconnectMaxMs?: number;
  /** 最大重连次数；默认不限 */
  maxReconnectAttempts?: number;
}

export const DEFAULT_WS_OPTIONS = {
  heartbeatIntervalMs: 10_000,
  heartbeatTimeoutMs: 30_000,
  connectTimeoutMs: 5_000,
  reconnectBaseMs: 500,
  reconnectMaxMs: 8_000,
  maxReconnectAttempts: Number.POSITIVE_INFINITY,
} as const;

export interface WebSocketTransportStats extends TransportStats {
  reconnects: number;
  rttMs: number | null;
  /** 非 null 表示前后端模型真值不一致（限位/标定/关节顺序） */
  modelMismatch: string | null;
  /** 后端上报的链路末端类型（sim / serial） */
  device: string | null;
}

function describeError(err: unknown): string {
  if (err instanceof Error) return err.message;
  return String(err);
}

export class WebSocketTransport implements RobotTransport {
  readonly kind = 'websocket';

  private readonly url: string;
  private readonly model: RobotModel;
  private readonly factory: SocketFactory;
  private readonly timer: TimerLike;
  private readonly heartbeatIntervalMs: number;
  private readonly heartbeatTimeoutMs: number;
  private readonly connectTimeoutMs: number;
  private readonly reconnectBaseMs: number;
  private readonly reconnectMaxMs: number;
  private readonly maxReconnectAttempts: number;

  private connection: TransportStatus = 'disconnected';
  private socket: SocketLike | null = null;

  /** 显式断开标记：为 true 时不重连（否则用户点"断开"会被立刻连回来） */
  private manualClose = false;
  private attempt = 0;
  private reconnects = 0;

  private heartbeatHandle: number | null = null;
  private connectTimeoutHandle: number | null = null;
  private reconnectHandle: number | null = null;
  /** 已发出 ping 的时刻；null = 无在途 ping */
  private pingSentAt: number | null = null;
  private rttMs: number | null = null;

  private seq = 0;
  private counters = { sent: 0, received: 0, dropped: 0, rejected: 0 };
  private lastCommand: JointState = {};
  private lastState: JointState = {};

  private modelInfo: BackendModelInfo | null = null;
  private modelMismatch: string | null = null;
  private device: string | null = null;

  private stateListeners = new Set<(state: RobotState) => void>();
  private statusListeners = new Set<(detail: TransportStatusDetail) => void>();

  constructor(options: WebSocketTransportOptions) {
    this.url = options.url;
    this.model = options.model;
    this.factory = options.socketFactory ?? webSocketFactory;
    this.timer = options.timer ?? realTimer;
    this.heartbeatIntervalMs = options.heartbeatIntervalMs ?? DEFAULT_WS_OPTIONS.heartbeatIntervalMs;
    this.heartbeatTimeoutMs = options.heartbeatTimeoutMs ?? DEFAULT_WS_OPTIONS.heartbeatTimeoutMs;
    this.connectTimeoutMs = options.connectTimeoutMs ?? DEFAULT_WS_OPTIONS.connectTimeoutMs;
    this.reconnectBaseMs = options.reconnectBaseMs ?? DEFAULT_WS_OPTIONS.reconnectBaseMs;
    this.reconnectMaxMs = options.reconnectMaxMs ?? DEFAULT_WS_OPTIONS.reconnectMaxMs;
    this.maxReconnectAttempts =
      options.maxReconnectAttempts ?? DEFAULT_WS_OPTIONS.maxReconnectAttempts;

    const home = { ...this.model.homePose };
    this.lastCommand = { ...home };
    this.lastState = { ...home };
  }

  // -------------------------------------------------------------------------
  // RobotTransport
  // -------------------------------------------------------------------------

  async connect(): Promise<void> {
    if (this.connection === 'connected' || this.connection === 'connecting') return;
    this.manualClose = false;
    this.attempt = 0;
    this.openSocket();
  }

  async disconnect(): Promise<void> {
    this.manualClose = true;
    this.clearReconnect();
    this.clearConnectTimeout();
    this.stopHeartbeat();
    this.pingSentAt = null;

    const socket = this.socket;
    this.socket = null;
    if (socket !== null) {
      try {
        socket.close(1000, 'client disconnect');
      } catch {
        // 已关闭的 socket 重复 close 是安全的
      }
    }
    // ⚠️ 显式复位，不依赖 onclose 事件：事件可能已被摘掉或根本不会触发
    //    （Phase 7 的 dispose 顺序 bug 就是踩了这个）
    this.connection = 'disconnected';
    this.emitStatus('disconnected');
  }

  async sendJointState(state: JointState): Promise<void> {
    if (this.connection !== 'connected' || this.socket === null) {
      this.emitStatus('error', '未连接：命令被丢弃');
      return;
    }
    this.seq += 1;
    this.lastCommand = { ...this.lastCommand, ...state };
    this.counters.sent += 1;
    this.trySend(encodeJointCommand(state, { seq: this.seq, timestamp: this.timer.now() }));
  }

  onState(callback: (state: RobotState) => void): Unsubscribe {
    this.stateListeners.add(callback);
    return () => this.stateListeners.delete(callback);
  }

  onStatus(callback: (detail: TransportStatusDetail) => void): Unsubscribe {
    this.statusListeners.add(callback);
    return () => this.statusListeners.delete(callback);
  }

  status(): TransportStatus {
    return this.connection;
  }

  // -------------------------------------------------------------------------
  // 诊断 / 统计
  // -------------------------------------------------------------------------

  stats(): WebSocketTransportStats {
    const lagDeg = this.lagDeg();
    return {
      kind: this.kind,
      ...this.counters,
      moving: lagDeg > this.arrivedEps(),
      lagDeg,
      reconnects: this.reconnects,
      rttMs: this.rttMs,
      modelMismatch: this.modelMismatch,
      device: this.device,
    };
  }

  /** 后端 hello 里的模型真值（未收到时为 null） */
  backendModelInfo(): BackendModelInfo | null {
    return this.modelInfo;
  }

  /** 发送序号（诊断用） */
  lastSeq(): number {
    return this.seq;
  }

  // -------------------------------------------------------------------------
  // 连接生命周期
  // -------------------------------------------------------------------------

  private openSocket(): void {
    this.clearReconnect();
    this.connection = 'connecting';
    this.emitStatus('connecting');

    let socket: SocketLike;
    try {
      socket = this.factory(this.url);
    } catch (err) {
      this.emitStatus('error', `创建连接失败：${describeError(err)}`);
      this.scheduleReconnect();
      return;
    }
    this.socket = socket;
    socket.onopen = () => this.handleOpen(socket);
    socket.onmessage = (data) => this.handleMessage(data);
    socket.onclose = (code, reason) => this.handleClose(socket, code, reason);
    // onerror 之后浏览器必发 onclose，这里不重复处理（避免双重重连）
    socket.onerror = () => undefined;

    this.connectTimeoutHandle = this.timer.setTimeout(() => {
      this.connectTimeoutHandle = null;
      if (this.socket === socket && this.connection !== 'connected') {
        this.emitStatus('error', `连接超时（${this.connectTimeoutMs}ms）`);
        this.abortSocket(socket, 'connect timeout');
      }
    }, this.connectTimeoutMs);
  }

  private handleOpen(socket: SocketLike): void {
    if (this.socket !== socket) return; // 过期回调（已被替换/断开）
    this.clearConnectTimeout();
    this.attempt = 0;
    this.connection = 'connected';
    this.emitStatus('connected');
    this.startHeartbeat();
    // 主动拉一次 hello + 状态：不依赖"服务端接入即推送"的时序
    this.trySend(encodeStatusRequest(this.timer.now()));
  }

  private handleClose(socket: SocketLike, code: number, reason: string): void {
    if (this.socket !== socket) return; // 已被显式断开或已重连
    this.socket = null;
    this.clearConnectTimeout();
    this.stopHeartbeat();
    this.pingSentAt = null;

    if (this.manualClose) {
      this.connection = 'disconnected';
      this.emitStatus('disconnected');
      return;
    }
    const detail = reason ? `code=${code} ${reason}` : `code=${code}`;
    this.emitStatus('error', `连接断开（${detail}），准备重连`);
    this.scheduleReconnect();
  }

  private scheduleReconnect(): void {
    if (this.manualClose) return;
    if (this.attempt >= this.maxReconnectAttempts) {
      this.connection = 'error';
      this.emitStatus(
        'error',
        `重连次数已达上限（${this.maxReconnectAttempts}），停止重连`,
      );
      return;
    }
    const delay = Math.min(this.reconnectBaseMs * 2 ** this.attempt, this.reconnectMaxMs);
    this.attempt += 1;
    this.reconnects += 1;
    this.connection = 'connecting';
    this.reconnectHandle = this.timer.setTimeout(() => {
      this.reconnectHandle = null;
      this.openSocket();
    }, delay);
  }

  /** 主动放弃当前 socket（心跳超时 / 连接超时）。幂等：close 触发的 onclose 会被忽略 */
  private abortSocket(socket: SocketLike, why: string): void {
    try {
      socket.close(4000, why);
    } catch {
      // 忽略：假 socket 或已关闭的连接
    }
    this.handleClose(socket, 4000, why);
  }

  // -------------------------------------------------------------------------
  // 消息处理
  // -------------------------------------------------------------------------

  private handleMessage(raw: string): void {
    const env = decodeServer(raw);
    if (env === null) {
      // 坏消息必须丢弃并报错：放行会让 Actual 变成 NaN 或静默错位
      this.emitStatus('error', '收到无法解析的下行消息（已丢弃）');
      return;
    }
    switch (env.type) {
      case SERVER_HELLO:
        this.handleHello(env);
        break;
      case SERVER_JOINT_STATE:
        this.handleJointState(env);
        break;
      case SERVER_ERROR:
        if (env.code === 'JOINT_LIMIT') this.counters.rejected += 1;
        // error 是**事件**不是连接终态：后端拒绝一条命令，链路仍然活着
        this.emitStatus('error', env.message ?? env.code ?? '后端返回未知错误');
        break;
      case SERVER_PONG:
        this.handlePong();
        break;
      case SERVER_DEVICE_STATUS:
        this.device = env.device ?? this.device;
        if (env.connected === false) {
          this.emitStatus('error', `后端链路末端不可用：${env.message ?? '未知原因'}`);
        }
        break;
      default:
        // 未知 type：向后兼容，忽略但不报错
        break;
    }
  }

  private handleHello(env: ServerEnvelope): void {
    const model = env.model;
    if (!model) return;
    this.modelInfo = model;
    this.device = env.device ?? this.device;
    const mismatch = describeModelMismatch(this.model, model);
    this.modelMismatch = mismatch;
    if (mismatch !== null) {
      this.emitStatus('error', `模型/标定不一致：${mismatch}`);
    }
  }

  private handleJointState(env: ServerEnvelope): void {
    const joints = env.joints;
    if (!joints || typeof joints !== 'object') return;
    this.counters.received += 1;
    this.lastState = { ...joints };
    // ★ 来源只认**明确标注**的那两种：`device`（设备侧自主变化）与
    //   `external`（另一台上位机经外部入口下命令）。其余（含未知取值）一律按
    //   command 处理 —— 宁可少跟随，也不要因为一个拼错的字符串把命令侧交给别人拖着走：
    //   跟随的代价是命令侧被改写，判错方向比不跟随危险得多。
    //
    //   ⚠️ 这一层是**白名单**：后端新增来源时，若只改了 `transportBridge` 而漏了这里，
    //      表现就是"后端标了、页面却没跟随"，而两端各自看代码都像是对的。
    const origin =
      env.origin === ORIGIN_DEVICE
        ? ('device' as const)
        : env.origin === ORIGIN_EXTERNAL
          ? ('external' as const)
          : undefined;
    const state = createRobotState(
      joints,
      endEffectorPose(this.model, joints),
      'real',
      env.timestamp ?? this.timer.now(),
      origin,
    );
    for (const listener of this.stateListeners) listener(state);
  }

  private handlePong(): void {
    if (this.pingSentAt === null) return;
    this.rttMs = this.timer.now() - this.pingSentAt;
    this.pingSentAt = null;
  }

  // -------------------------------------------------------------------------
  // 心跳
  // -------------------------------------------------------------------------

  private startHeartbeat(): void {
    this.stopHeartbeat();
    this.pingSentAt = null;
    this.heartbeatHandle = this.timer.setInterval(() => this.heartbeat(), this.heartbeatIntervalMs);
  }

  private heartbeat(): void {
    if (this.connection !== 'connected' || this.socket === null) return;
    const now = this.timer.now();
    if (this.pingSentAt !== null && now - this.pingSentAt > this.heartbeatTimeoutMs) {
      this.emitStatus(
        'error',
        `心跳超时（${this.heartbeatTimeoutMs}ms 未收到 pong），判定链路已死`,
      );
      const socket = this.socket;
      this.abortSocket(socket, 'heartbeat timeout');
      return;
    }
    // 上一次 ping 仍在途时不重复发：否则超时判定会被自己不断推后
    if (this.pingSentAt === null) {
      this.pingSentAt = now;
      this.trySend(encodePing(now));
    }
  }

  private stopHeartbeat(): void {
    if (this.heartbeatHandle === null) return;
    this.timer.clearInterval(this.heartbeatHandle);
    this.heartbeatHandle = null;
  }

  private clearConnectTimeout(): void {
    if (this.connectTimeoutHandle === null) return;
    this.timer.clearTimeout(this.connectTimeoutHandle);
    this.connectTimeoutHandle = null;
  }

  private clearReconnect(): void {
    if (this.reconnectHandle === null) return;
    this.timer.clearTimeout(this.reconnectHandle);
    this.reconnectHandle = null;
  }

  // -------------------------------------------------------------------------
  // 其它
  // -------------------------------------------------------------------------

  private trySend(payload: string): void {
    const socket = this.socket;
    if (socket === null) return;
    if (socket.readyState !== SOCKET_OPEN) {
      this.counters.dropped += 1;
      return;
    }
    try {
      socket.send(payload);
    } catch (err) {
      this.emitStatus('error', `发送失败：${describeError(err)}`);
    }
  }

  /**
   * 最近一次命令与最近一次实际状态的最大关节差（度）。
   *
   * ⚠️ 命令侧必须先用**链路末端实际能表达的精度**归整再比对，
   * 否则量化残差会被当成"永远收敛不了的跟踪误差"。
   * 两种末端的瓶颈不同，必须分别处理：
   *
   *   sim    瓶颈是 `JR` 文本（保留 1 位小数）⇒ `quantizeForWire`（0.1°）
   *          实测 `elbow` 内部命令 `112.6185771989` 与线上 `112.6` 恒差
   *          `0.0186°`，面板永久显示 `0.02°`、`moving` 永不归零。
   *
   *   serial 瓶颈是**固件舵机角为整数**（`parse_u8`）⇒ `quantizeViaServo`。
   *          同样不处理的话残差是 `0.347°`（S7），比 sim 那次大一个量级。
   *
   * 详见 `wsProtocol.ts` 的 `WIRE_JOINT_STEP_DEG` / `quantizeViaServo`。
   */
  private lagDeg(): number {
    const commanded =
      this.device === 'serial'
        ? quantizeViaServo(this.model, this.lastCommand)
        : quantizeForWire(this.lastCommand);
    const ids = new Set([...Object.keys(commanded), ...Object.keys(this.lastState)]);
    let worst = 0;
    for (const id of ids) {
      const d = Math.abs((this.lastState[id] ?? 0) - (commanded[id] ?? 0));
      if (d > worst) worst = d;
    }
    return worst;
  }

  /** "已到位"阈值：真机与 sim 的链路精度不同，阈值必须分开（理由见常量注释）。 */
  private arrivedEps(): number {
    return this.device === 'serial' ? ARRIVED_EPS_SERIAL_DEG : ARRIVED_EPS_DEG;
  }

  private emitStatus(status: TransportStatus, reason?: string): void {
    const detail: TransportStatusDetail = {
      status,
      timestamp: this.timer.now(),
      ...(reason ? { reason } : {}),
    };
    for (const listener of this.statusListeners) listener(detail);
  }
}
