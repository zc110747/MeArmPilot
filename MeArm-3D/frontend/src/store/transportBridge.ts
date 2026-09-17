/**
 * transportBridge —— 把 Zustand store 与 `RobotTransport` 接起来（Phase 7 起）。
 *
 * Phase 8 起它**不再绑定 MockTransport**：任何实现 `RobotTransport` 的传输
 * （Mock / WebSocket / Phase 9 的 Serial）都能挂上来。它承担四件事，
 * 每件都有一个**必须做对**的细节：
 *
 * 1. **命令下发 + 尾沿节流**
 *    所有改 `commandJoints` 的路径（滑杆 / HOME / ZERO / 拖动 moveTo）统一在这里
 *    汇流。拖动时每帧都会变，若逐帧下发，60fps 就是每秒 60 条命令 ——
 *    Phase 9 串口（115200、ACK 门控）下必然打爆。故取 **尾沿合并（trailing edge）**：
 *    间隔内只保留最新一帧，松手时由 trailing 定时器补发最后一帧，不会停在半路。
 *
 * 2. **回推落地（回环打破）**
 *    `onState` 回调**只写 `actualJoints`**，绝不回写 `commandJoints`。
 *    若两边互相写，就形成 `命令 → 状态 → 命令` 的无限回环（spec §十九）。
 *
 *    ⚠️ 唯一的例外是**首次接管的那一帧**：那时本地还没有任何命令，"回写"不构成
 *    回环（一次性、且被 `suppressCommandSend` 拦住不下发），但对齐之后
 *    ① 画面上不会多出一棵错开的半透明幽灵、② 面板显示的是机器真实姿态、
 *    ③ 首次下发是"从机器现在的位置"出发而不是扑向页面假设的 HOME。
 *    详见 `robotStore.attachToActual`。
 *
 * 3. **状态登记 + 统计刷新**
 *    `onStatus` 映射到 store 的 connection；`poll()` 每 `STATS_POLL_MS` 拉一次统计，
 *    **脏检查后**才写 store（避免收敛后每 200ms 无谓触发 React 重渲染）。
 *    发送/接收日志按 `LOG_FLUSH_MS` 合并成一条，避免拖动时刷屏。
 *
 * 4. **重连后补齐命令**（Phase 8）
 *    断线期间用户可能已经拖到别处。重连成功时把当前 `commandJoints` 补发一次，
 *    否则虚拟臂与实际臂会永久错开 —— 而 UI 上看不出任何异常。
 */
import {
  MockTransport,
  WebSocketTransport,
  realTimer,
  type JointState,
  type MockTransportStats,
  type MockTransportTuning,
  type RobotState,
  type RobotTransport,
  type TransportStats,
  type TransportStatusDetail,
  type TimerLike,
  type WebSocketTransportOptions,
} from '@robot/index';
import { useRobotStore } from './robotStore';

/** 命令最小发送间隔（ms）：约 30Hz，兼顾跟手与链路压力 */
export const MIN_SEND_INTERVAL_MS = 33;
/** 统计刷新周期（ms） */
export const STATS_POLL_MS = 200;
/** 发送/接收日志的合并周期（ms） */
export const LOG_FLUSH_MS = 500;
/** 未接入传输时的统一文案 */
export const NO_TRANSPORT_LABEL = '未接入（可在 Connection 面板连接）';

export interface TransportBridgeOptions {
  /**
   * 显式注入传输实现。缺省时建立 `MockTransport`（保持 Phase 7 行为不变）。
   * 测试可借此注入假传输。
   */
  transport?: RobotTransport;
  /** 可注入时钟；与传输实现共用同一个，测试才能确定性推进 */
  timer?: TimerLike;
  /** Mock 调参初始值 */
  tuning?: Partial<MockTransportTuning>;
  /** 可复现的随机源（丢帧） */
  random?: () => number;
  /** 仿真 tick 周期（ms） */
  tickMs?: number;
  /** 命令最小发送间隔覆盖值 */
  minimumSendIntervalMs?: number;
}

/** 状态面板上的一行描述（按传输类型给出最有信息量的那几项） */
export function describeTransportStats(stats: TransportStats): string {
  switch (stats.kind) {
    case 'mock': {
      const speed =
        Number.isFinite(stats.maxSpeedDegPerSec ?? 0) && (stats.maxSpeedDegPerSec ?? 0) > 0
          ? `${stats.maxSpeedDegPerSec}°/s`
          : '瞬时到位';
      const drop = ((stats.dropRate ?? 0) * 100).toFixed(0);
      return `MockTransport · ${speed} · ${stats.latencyMs ?? 0}ms 延迟 · 丢帧 ${drop}%`;
    }
    case 'websocket': {
      const rtt = stats.rttMs === null || stats.rttMs === undefined ? '—' : `${stats.rttMs}ms`;
      return `WebSocket · 心跳 RTT ${rtt} · 重连 ${stats.reconnects ?? 0} 次`;
    }
    default:
      return stats.kind;
  }
}

export class TransportBridge {
  private readonly timer: TimerLike;
  private readonly transport: RobotTransport;
  /** 非 null 表示当前挂的是 MockTransport（调参面板据此启用） */
  private readonly mock: MockTransport | null;
  private readonly minSendIntervalMs: number;

  /** 节流状态 */
  private pendingJoints: JointState | null = null;
  private lastSendAt = Number.NEGATIVE_INFINITY;
  private trailingHandle: number | null = null;

  private pollHandle: number | null = null;
  private unsubscribeStore: (() => void) | null = null;
  private unsubscribeState: (() => void) | null = null;
  private unsubscribeStatus: (() => void) | null = null;

  /** 待合并进日志的计数 */
  private txSince = 0;
  private rxSince = 0;
  private lastTxLogAt = Number.NEGATIVE_INFINITY;
  private lastRxLogAt = Number.NEGATIVE_INFINITY;

  private lastStatsKey = '';
  private disposed = false;
  /** 是否已经成功连接过一次（用于区分"首次连接"与"重连后补发命令"） */
  private hasConnectedOnce = false;
  /**
   * 首次接管握手待办（见 `handleState`）。
   *
   * 由 `handleStatus('connected')` 在**首次**连接时置起，被**第一帧回推**消费掉。
   * 它存在的理由：`actualJoints` 是"机器现在在哪"的唯一来源，而页面在连上之前
   * 只能假设 HOME。两者不一致时，页面会画出一棵错开的半透明幽灵、面板显示假姿态，
   * 并且**首次下发会把真机整组扑向 HOME**（见 `robotStore.attachToActual` 注释）。
   */
  private attachPending = false;
  /**
   * 本次连接建立后，用户是否**已经下达过命令**。
   *
   * 为什么需要它：握手要等"第一帧回推"，而回推何时到达不由页面决定 ——
   * 若后端（或 Mock 的 tick）慢一拍，用户完全可能先在滑杆上动了手。
   * 那时**用户的意图必须优先**：接管只是"本地参照还没有值"时的兜底，
   * 一旦有人表达了意图，再去对齐就会把刚下达的命令悄悄擦掉。
   *
   * 判定点与"命令变化 → 下发"是同一条订阅：能被当成意图下发的，就是意图。
   */
  private commandAuthoredSinceConnect = false;
  /**
   * 接管握手期间抑制一次「命令变化 → 下发」。
   *
   * `attachToActual()` 会写 `commandJoints`（把它对齐到机器现状），而上面那条
   * store 订阅把"命令变化"一律当作用户意图去下发。**接管不是意图** ——
   * 它只是把本地参照挪到机器此刻的位置，绝不能因此驱动机械臂。
   * 该标志只在 `handleState` 的握手分支里短暂置起（zustand 的订阅是同步回调，
   * try/finally 足以精确覆盖那一次 `set`）。
   */
  private suppressCommandSend = false;
  /** 被安全门拦下的下发次数（mode=simulation 却连着真机链路） */
  private blockedSince = 0;
  /**
   * 是否已就"设备侧自主变化"记过日志。
   *
   * 摇杆持续拨动时每帧都会走 `followDevice`，逐帧记日志会把面板刷满；
   * 而"为什么要跟随"这个信息只需要出现一次。连接复位时清回 false。
   */
  private deviceOriginLogged = false;

  /**
   * 是否已就"**外部入口**在驱动"记过日志（同一台上位机经 TCP 网关下命令）。
   * 与 `deviceOriginLogged` 同理只记一次 —— 摇杆持续推动时每帧都会走跟随。
   */
  private externalOriginLogged = false;

  constructor(options: TransportBridgeOptions = {}) {
    this.timer = options.timer ?? realTimer;
    this.minSendIntervalMs = options.minimumSendIntervalMs ?? MIN_SEND_INTERVAL_MS;

    if (options.transport) {
      this.transport = options.transport;
      this.mock = options.transport instanceof MockTransport ? options.transport : null;
    } else {
      const state = useRobotStore.getState();
      const mock = new MockTransport({
        model: state.model,
        timer: this.timer,
        // 初始实际位置与当前命令一致：连接不应让机械臂"跳"一下
        initialJoints: state.commandJoints,
        ...(options.tickMs !== undefined ? { tickMs: options.tickMs } : {}),
        ...(options.random !== undefined ? { random: options.random } : {}),
        ...(options.tuning ?? {}),
      });
      this.mock = mock;
      this.transport = mock;
    }
  }

  /** 当前传输类型（mock / websocket…） */
  kind(): string {
    return this.transport.kind;
  }

  /**
   * 当前链路是否**真的连着物理硬件**。
   *
   * 判据两条同时成立：
   *   1. 传输是 WebSocket（浏览器内 Mock 永远不是真机）
   *   2. 后端 hello/stats 上报的 `device === 'serial'`
   *
   * `device` 未知（尚未收到 hello）时按 **false** 处理 —— 宁可少拦，
   * 也不要因为"还没握手"就把正常仿真误判成真机而拒绝下发。
   */
  private isRealHardwareLink(): boolean {
    if (this.transport.kind !== 'websocket') return false;
    const stats = this.transport.stats?.();
    const device = stats !== undefined && stats !== null && 'device' in stats ? stats.device : null;
    return device === 'serial';
  }

  async connect(): Promise<void> {
    // 先挂监听再 connect：否则 connect 期间发出的 status 事件会丢
    this.unsubscribeStatus = this.transport.onStatus((detail) => this.handleStatus(detail));
    this.unsubscribeState = this.transport.onState((state) => this.handleState(state));

    await this.transport.connect();

    // 命令变化 → 节流下发。比较的是 `commandJoints` 的对象引用，
    // 而 store 每次改命令都会生成新对象，所以不会漏事件。
    this.unsubscribeStore = useRobotStore.subscribe((state, prev) => {
      // 接管握手写的是"本地参照"，不是用户意图 —— 见 `suppressCommandSend`
      if (this.suppressCommandSend) return;
      if (state.commandJoints !== prev.commandJoints) {
        // 走到这里就是用户意图（滑杆 / HOME / ZERO / 拖动）：接管握手必须让位
        this.commandAuthoredSinceConnect = true;
        this.enqueue(state.commandJoints);
      }
    });

    this.pollHandle = this.timer.setInterval(() => this.poll(), STATS_POLL_MS);
  }

  async dispose(): Promise<void> {
    if (this.disposed) return;
    this.disposed = true;

    if (this.trailingHandle !== null) {
      this.timer.clearTimeout(this.trailingHandle);
      this.trailingHandle = null;
    }
    if (this.pollHandle !== null) {
      this.timer.clearInterval(this.pollHandle);
      this.pollHandle = null;
    }
    this.unsubscribeStore?.();
    this.unsubscribeStore = null;
    this.unsubscribeState?.();
    this.unsubscribeState = null;
    this.unsubscribeStatus?.();
    this.unsubscribeStatus = null;

    await this.transport.disconnect();
    this.pendingJoints = null;
    this.hasConnectedOnce = false;
    this.attachPending = false;
    this.commandAuthoredSinceConnect = false;
    this.suppressCommandSend = false;
    this.blockedSince = 0;
    this.deviceOriginLogged = false;
    this.externalOriginLogged = false;

    // ⚠️ 这里必须**显式**复位，不能指望 transport.disconnect() 发出的 disconnected 事件：
    //    上面已经退订了 status 监听，事件根本没人接。早先的写法就是踩了这个坑 ——
    //    断开后状态灯仍显示 Connected、Actual 还挂着最后那个滞后值。
    this.store().setConnection(null, 'disconnected', NO_TRANSPORT_LABEL);
  }

  /** 运行期调参（面板滑杆）；非 Mock 传输下是 no-op */
  tune(patch: Partial<MockTransportTuning>): void {
    if (this.mock === null) return;
    this.mock.tune(patch);
    this.poll();
  }

  /** Mock 专有统计；非 Mock 返回 null（供调参面板回填滑杆） */
  mockStats(): MockTransportStats | null {
    return this.mock?.stats() ?? null;
  }

  /** 通用统计 */
  stats(): TransportStats | null {
    return this.transport.stats?.() ?? null;
  }

  // -------------------------------------------------------------------------
  // 内部
  // -------------------------------------------------------------------------

  private store() {
    return useRobotStore.getState();
  }

  /** 尾沿合并：间隔内只保留最新一帧 */
  private enqueue(joints: JointState): void {
    this.pendingJoints = joints;
    const now = this.timer.now();
    const elapsed = now - this.lastSendAt;

    if (elapsed >= this.minSendIntervalMs) {
      this.flush(now);
      return;
    }
    if (this.trailingHandle === null) {
      this.trailingHandle = this.timer.setTimeout(() => {
        this.trailingHandle = null;
        this.flush(this.timer.now());
      }, this.minSendIntervalMs - elapsed);
    }
  }

  private flush(now: number): void {
    const joints = this.pendingJoints;
    if (joints === null || this.disposed) return;
    this.pendingJoints = null;
    this.lastSendAt = now;

    // ---- 安全门（mode ↔ transport 联动）----
    //
    // `mode === 'simulation'` 表示用户明确要求"只在仿真里动，别碰真机"。
    // 此时若连的是**真机链路**（websocket + serial），必须**拒绝下发** ——
    // 否则"切回 Simulation"就成了纯装饰，真机照样被驱动（危险且不可预期）。
    //
    // 注意只拦"真机链路"：mock 与 device=sim 的后端本就是仿真，照常放行，
    // 否则会把 Phase 7/8 的既有仿真闭环一起打死。
    if (this.store().mode === 'simulation' && this.isRealHardwareLink()) {
      this.blockedSince += 1;
      return;
    }

    this.txSince += 1;
    void this.transport.sendJointState(joints);
  }

  /**
   * 回推落地。
   *
   * 稳态：**只写 `actualJoints`** —— 回写 command 即无限回环。
   * 例外有二，都必须显式抑制下发：
   *   ① **首次接管的第一帧**走 `attachToActual`（见字段 `attachPending` 注释）；
   *   ② 后端标注 `origin === 'device'` 的帧走 `followDevice`
   *      （设备侧自主变化，见 `handleState`）。
   */
  private handleState(state: RobotState): void {
    this.rxSince += 1;
    const store = this.store();
    if (this.attachPending) {
      // 只有"还没有任何意图"时才对齐。用户已经动过手 ⇒ 命令优先，
      // 放弃握手（此后按稳态语义，让机器去追命令）。
      const handshake = !this.commandAuthoredSinceConnect;
      this.attachPending = false;
      if (handshake) {
        this.suppressCommandSend = true;
        try {
          store.attachToActual(state.joints);
        } finally {
          this.suppressCommandSend = false;
        }
        return;
      }
    }

    // ★ 设备侧自主变化（硬件摇杆 / 红外遥控 / 面板手拧 / 调试直控）：
    //   命令侧必须跟着走，否则滑杆、主臂、目标点停在旧值上 —— 而画面看起来
    //   一切正常（这正是"上位机状态没跟随"这类缺陷最难被发现的地方）。
    //
    //   判据是**后端标注的 origin**，不是"比较 Actual 与 Command"：
    //   拖动时设备还在斜坡上，Actual 必然落后于 Command，那种判据会把命令侧
    //   一路拉回半路位置（spec §十九 明令禁止的回环）。
    if (state.origin === 'device') {
      this.followDevice(state);
      return;
    }

    // ★ **外部入口**在驱动（另一台上位机经 TCP JSON 网关下命令，见
    //   backend/internal/protocol.OriginExternal）：处理方式与设备侧一致 ——
    //   命令侧必须跟随。若不跟随，画面会变成"只有半透明的实际臂在动、主臂不动"，
    //   而且命令行（滑杆 / 目标点）停在旧值：用户下一次动本页任何一个控件，
    //   会把**整组旧指令**下发（真机上就是机械臂突然跳回旧位姿）。
    //
    //   同样必须抑制回发，否则变成 命令 → 状态 → 命令 回环
    //   （对方还在驱动，本页又把旧值推回去，两侧对着拽）。
    if (state.origin === 'external') {
      this.followExternal(state);
      return;
    }

    store.setActualJoints(state.joints);
  }

  /**
   * 把命令侧对齐到设备报来的现状（外部驱动）。
   *
   * ⚠️ 必须抑制下发：对齐会写 `commandJoints`，而上面那条 store 订阅把
   * "命令变化"一律当作用户意图。不抑制的话，每次摇杆上报都会触发一条命令下发 ——
   * 那既是 `命令 → 状态 → 命令` 回环，真机侧还会表现为
   * "设备自己动 → 上位机又把它推回去"的对抗。
   */
  private followDevice(state: RobotState): void {
    this.alignCommandSide(state);
    // 只记一次：摇杆持续拨动时每帧一行会把日志面板刷满，
    // 而"为什么会跟随"这个信息只需要出现一次。
    if (!this.deviceOriginLogged) {
      this.deviceOriginLogged = true;
      this.store().pushLog(
        'in',
        '设备侧自主变化（摇杆 / 红外 / 手拧）—— 命令侧已跟随，未回发命令',
      );
    }
  }

  /**
   * 外部入口（另一台上位机经 TCP JSON 网关）在下命令时的跟随。
   *
   * 日志文案必须与 `followDevice` 分开：来源是**外部命令**，不是设备的自主变化 ——
   * 让日志说出"摇杆/红外/手拧"会是对现场的误报，而这个面板正是排障时最先看的东西。
   */
  private followExternal(state: RobotState): void {
    this.alignCommandSide(state);
    if (!this.externalOriginLogged) {
      this.externalOriginLogged = true;
      this.store().pushLog(
        'in',
        '外部入口在驱动（另一台上位机 / TCP 网关）—— 命令侧已跟随，未回发命令',
      );
    }
  }

  /**
   * 把命令侧对齐到"别人报来的现状"（设备自主变化 / 外部入口命令共用）。
   *
   * ⚠️ 必须抑制下发：对齐会写 `commandJoints`，而上面那条 store 订阅把
   * "命令变化"一律当作用户意图。不抑制的话，每次外部上报都会触发一条命令下发 ——
   * 那既是 `命令 → 状态 → 命令` 回环，真机侧还会表现为
   * "别人动 → 本页又把它推回去"的对抗（两台上位机对着拽）。
   */
  private alignCommandSide(state: RobotState): void {
    this.suppressCommandSend = true;
    try {
      this.store().followDevice(state.joints);
    } finally {
      this.suppressCommandSend = false;
    }
  }

  private handleStatus(detail: TransportStatusDetail): void {
    const store = this.store();
    switch (detail.status) {
      case 'connecting':
        store.setConnection(
          this.transport.kind,
          'disconnected',
          `连接中…（${this.transport.kind}）`,
        );
        break;
      case 'connected': {
        store.setConnection(this.transport.kind, 'connected', this.describeCurrent());
        this.commandAuthoredSinceConnect = false;
        this.deviceOriginLogged = false;
        this.externalOriginLogged = false;
        if (this.hasConnectedOnce) {
          // ⚠️ 只在**重连**时补发：断线期间用户可能已改过命令，
          //    不补发会让虚拟臂与实际臂永久错开而 UI 看不出异常。
          this.enqueue(store.commandJoints);
        } else {
          // 首次连接：既不补发命令（保持 Phase 7 时序契约），也不任由页面
          // 假设位姿挂在那里 —— 等第一帧回推，用它把本地命令对齐到机器现状。
          this.attachPending = true;
        }
        this.hasConnectedOnce = true;
        break;
      }
      case 'disconnected':
        store.setConnection(null, 'disconnected', NO_TRANSPORT_LABEL);
        break;
      case 'error':
        // error 是**事件**而非连接终态：固件回 ERR 一行，链路仍然活着
        store.pushLog('in', detail.reason ?? '传输错误');
        break;
      default:
        break;
    }
  }

  private describeCurrent(): string {
    const stats = this.transport.stats?.();
    if (!stats) return `已连接（${this.transport.kind}）`;
    return describeTransportStats(stats);
  }

  /** 周期刷新统计与合并日志（脏检查 → 收敛后不再重渲染） */
  private poll(): void {
    if (this.disposed) return;
    const stats = this.transport.stats?.();
    if (!stats) {
      this.flushLogs();
      return;
    }
    const key = [
      stats.sent,
      stats.received,
      stats.dropped,
      stats.rejected,
      stats.moving ? 1 : 0,
      stats.lagDeg.toFixed(4),
      stats.maxSpeedDegPerSec ?? '',
      stats.latencyMs ?? '',
      stats.dropRate ?? '',
      stats.enforceLimits === undefined ? '' : stats.enforceLimits ? 1 : 0,
      stats.reconnects ?? '',
      stats.rttMs ?? '',
    ].join('|');

    if (key !== this.lastStatsKey) {
      this.lastStatsKey = key;
      const store = this.store();
      store.setTransportStats(stats);
      if (store.connection === 'connected') {
        store.setConnection(this.transport.kind, 'connected', describeTransportStats(stats));
      }
    }

    this.flushLogs();
  }

  private flushLogs(): void {
    const now = this.timer.now();
    const store = this.store();

    if (this.txSince > 0 && now - this.lastTxLogAt >= LOG_FLUSH_MS) {
      this.lastTxLogAt = now;
      store.pushLog('out', `joint_command ×${this.txSince} → ${this.transport.kind}`);
      this.txSince = 0;
    }
    if (this.rxSince > 0 && now - this.lastRxLogAt >= LOG_FLUSH_MS) {
      this.lastRxLogAt = now;
      store.pushLog('in', `joint_state ×${this.rxSince} ← ${this.transport.kind}`);
      this.rxSince = 0;
    }
    // 安全门拦下的下发必须**可见**：否则用户会看到"滑杆动了但机械臂没动"，
    // 却没有任何线索说明为什么 —— 正是本次要修的那类静默失败。
    if (this.blockedSince > 0) {
      store.pushLog(
        'err',
        `已拦截 ${this.blockedSince} 条命令：当前为 Simulation 模式，` +
          '不下发给真机（切到 Real Robot 才会下发）',
      );
      this.blockedSince = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// 模块级单例 —— UI 只跟这几个函数打交道
// ---------------------------------------------------------------------------

let activeBridge: TransportBridge | null = null;

/** 连接 MockTransport（幂等：先释放旧的） */
export async function connectMockTransport(
  options: TransportBridgeOptions = {},
): Promise<TransportBridge> {
  await disconnectTransport();
  const bridge = new TransportBridge(options);
  activeBridge = bridge;
  await bridge.connect();
  return bridge;
}

/**
 * 连接后端关节级 WebSocket。
 *
 * `model` 由 store 提供（与前端渲染同一个 `RobotModel`），用于 FK 与 hello 一致性校验。
 */
export async function connectWebSocketTransport(
  options: Omit<WebSocketTransportOptions, 'model'> & { model?: WebSocketTransportOptions['model'] },
): Promise<TransportBridge> {
  await disconnectTransport();
  const state = useRobotStore.getState();
  const transport = new WebSocketTransport({ ...options, model: options.model ?? state.model });
  const bridge = new TransportBridge({ transport });
  activeBridge = bridge;
  await bridge.connect();
  return bridge;
}

/** 断开并释放当前传输 */
export async function disconnectTransport(): Promise<void> {
  const bridge = activeBridge;
  if (bridge === null) return;
  activeBridge = null;
  await bridge.dispose();
}

export function activeTransportBridge(): TransportBridge | null {
  return activeBridge;
}

/** 运行期调参（未连接或非 Mock 时是 no-op） */
export function tuneTransport(patch: Partial<MockTransportTuning>): void {
  activeBridge?.tune(patch);
}
