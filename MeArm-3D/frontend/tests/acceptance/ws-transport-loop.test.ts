/**
 * WebSocket 传输闭环验收（Phase 8）。
 *
 * 与 `webSocketTransport.test.ts`（只测传输自身）的分工：
 *   本文件证明的是「**接线正确**」—— 把 WebSocketTransport 挂到 transportBridge 上之后：
 *     1. 命令节流生效（拖动不会把链路打爆）
 *     2. 稳态回推只写 actualJoints（回环打破）
 *     3. **重连后自动补发当前命令**（否则虚拟臂与实际臂永久错开）
 *     4. 首次连接不补发（不改变 Phase 7 的时序契约）
 *     5. **首次接管**：第一帧回推是一次显式握手，把本地命令对齐到机器现状 ——
 *        它既不下发命令、也不构成回环（一次性），但它是"首次加载画面上不会多出
 *        一棵错位的半透明幽灵"的唯一保证，见 `robotStore.attachToActual`。
 *
 * 用 FakeSocket + FakeTimer：无真实网络、无真实等待。
 */
import { afterEach, beforeEach, describe, expect, it } from 'vitest';
import {
  WebSocketTransport,
  endEffectorPose,
  homeJointState,
  loadRobotModel,
} from '@robot/index';
import { TransportBridge } from '@/store/transportBridge';
import { useRobotStore } from '@/store/robotStore';
import { FakeSocketFactory } from '../helpers/fakeSocket';
import { FakeTimer } from '../helpers/fakeTimer';
import { backendInfoFromLocal } from '../helpers/backendModel';

const model = loadRobotModel('mearm-v1');
const home = homeJointState(model);
const URL = 'ws://test.local:8090/ws/joint';

function resetStore(): void {
  const pose = endEffectorPose(model, home);
  useRobotStore.setState({
    commandJoints: { ...home },
    actualJoints: { ...home },
    endEffector: pose,
    actualEndEffector: pose,
    target: [pose.position[0], pose.position[1], pose.position[2]],
    ikStatus: null,
    dragging: false,
    mode: 'simulation',
    controlSource: 'virtual',
    connection: 'disconnected',
    connectionLabel: '未接入',
    transportKind: null,
    transportDriven: false,
    transportStats: null,
    log: [],
  });
}

interface Harness {
  timer: FakeTimer;
  factory: FakeSocketFactory;
  transport: WebSocketTransport;
  bridge: TransportBridge;
  frames(type: string): Array<Record<string, unknown>>;
}

let active: Harness | null = null;

async function connect(): Promise<Harness> {
  const timer = new FakeTimer(1000);
  const factory = new FakeSocketFactory();
  const transport = new WebSocketTransport({
    url: URL,
    model,
    socketFactory: factory.create,
    timer,
    heartbeatIntervalMs: 10_000,
    heartbeatTimeoutMs: 30_000,
    reconnectBaseMs: 500,
  });
  const bridge = new TransportBridge({ transport, timer });
  await bridge.connect();
  active = {
    timer,
    factory,
    transport,
    bridge,
    frames: (type) => factory.last.framesOfType(type),
  };
  return active;
}

/** 连接并完成握手 */
async function connected(): Promise<Harness> {
  const h = await connect();
  h.factory.last.open();
  return h;
}

beforeEach(() => {
  resetStore();
});

afterEach(async () => {
  if (active) {
    await active.bridge.dispose();
    active = null;
  }
});

describe('WebSocket 接线 · 连接登记', () => {
  it('连接后 store 记 websocket 且进入 transportDriven', async () => {
    await connected();
    const s = useRobotStore.getState();
    expect(s.connection).toBe('connected');
    expect(s.transportKind).toBe('websocket');
    expect(s.transportDriven).toBe(true);
  });

  it('断开后复位：actual 拉回 command，统计清空', async () => {
    const h = await connected();
    // 第 1 帧被接管握手消费掉（命令对齐到机器现状 = 30），第 2 帧才是稳态滞后
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 30 },
    });
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 40 },
    });
    expect(useRobotStore.getState().actualJoints.shoulder).toBeCloseTo(40, 9);

    await h.bridge.dispose();
    active = null;
    const s = useRobotStore.getState();
    expect(s.connection).toBe('disconnected');
    expect(s.transportDriven).toBe(false);
    expect(s.transportStats).toBeNull();
    // 拉回的是**命令**（接管时对齐到的机器现状 30），而不是最后那次滞后值 40
    expect(s.actualJoints.shoulder).toBeCloseTo(30, 9);
    expect(s.actualJoints.shoulder).toBeCloseTo(s.commandJoints.shoulder, 9);
  });

  it('首次连接**不**补发命令（保持 Phase 7 时序契约）', async () => {
    const h = await connected();
    h.timer.advance(200);
    expect(h.frames('joint_command')).toHaveLength(0);
  });
});

describe('WebSocket 接线 · 节流', () => {
  it('400 次命令变化只下发个位数~十几帧（尾沿合并 33ms）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    for (let i = 0; i < 400; i += 1) {
      store.setJoint('shoulder', 10 + (i % 20));
      h.timer.advance(1); // 每 1ms 改一次，共 400ms
    }
    h.timer.advance(100); // 放掉 trailing

    const sent = h.frames('joint_command').length;
    expect(sent).toBeGreaterThan(0);
    expect(sent).toBeLessThan(60); // 400ms / 33ms ≈ 13 帧
    expect(sent).toBeLessThan(400);
  });

  it('最后一帧一定是"最新命令"（trailing 保证不会停在半路）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 20);
    h.timer.advance(1);
    store.setJoint('shoulder', 25);
    h.timer.advance(1);
    store.setJoint('shoulder', 33.3); // 最终值
    h.timer.advance(200);

    const cmds = h.frames('joint_command');
    const last = cmds[cmds.length - 1];
    expect((last.joints as Record<string, number>).shoulder).toBeCloseTo(33.3, 9);
  });
});

describe('WebSocket 接线 · 首次接管握手', () => {
  /**
   * 非 HOME 的「机器现状」。
   *
   * 取值与真实复现一致：把后端 sim 停在 `{shoulder:-5, elbow:110, gripper:0}`
   * （`tools/park_sim_pose.mjs`），再冷启动页面即可看到主臂在 HOME、幽灵在别处。
   */
  const parked = { ...home, shoulder: -5, elbow: 110, gripper: 0 };

  it('★ 首次加载不得出现"错位的实际臂幽灵"：命令对齐到机器现状', async () => {
    const h = await connected();
    const assumed = useRobotStore.getState().commandJoints;
    expect(assumed.shoulder).toBeCloseTo(home.shoulder, 12); // 连上之前，页面只能假设 HOME

    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: parked });

    const s = useRobotStore.getState();
    // 主臂跟 command、幽灵跟 actual —— 两者不重合，画面上就是两棵树（重影）
    for (const id of Object.keys(parked)) {
      expect(s.actualJoints[id], id).toBeCloseTo(s.commandJoints[id], 12);
    }
    expect(s.commandJoints).not.toBe(assumed); // 确实换成了机器现状
    expect(s.controlSource).toBe('real');

    // 目标同步到机器现状，否则拖动把手会停在页面假设的位置
    const tcp = endEffectorPose(model, parked).position;
    expect(s.target[0]).toBeCloseTo(tcp[0], 9);
    expect(s.target[2]).toBeCloseTo(tcp[2], 9);
  });

  it('握手只改本地参照，**不下发**任何命令（接管 ≠ 用户意图）', async () => {
    const h = await connected();
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: parked });
    h.timer.advance(1000); // 足够放掉任何 trailing
    expect(h.frames('joint_command')).toHaveLength(0);
  });

  it('用户先下达命令 ⇒ 接管让位（绝不擦掉用户意图）', async () => {
    const h = await connected();
    // 后端回推慢一拍时，用户完全可能先动滑杆
    useRobotStore.getState().setJoint('shoulder', 12);
    const authored = useRobotStore.getState().commandJoints;

    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: parked });

    const s = useRobotStore.getState();
    expect(s.commandJoints).toBe(authored); // toBe：引用级 —— 命令没被改写
    expect(s.commandJoints.shoulder).toBeCloseTo(12, 9);
    expect(s.actualJoints.shoulder).toBeCloseTo(parked.shoulder, 9); // 现状照常落地
  });

  it('握手只发生一次：之后的回推不再改动 commandJoints', async () => {
    const h = await connected();
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: parked });
    const adopted = useRobotStore.getState().commandJoints;
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 20 },
    });
    expect(useRobotStore.getState().commandJoints).toBe(adopted); // toBe：引用级
    expect(useRobotStore.getState().actualJoints.shoulder).toBeCloseTo(20, 9);
  });
});

describe('WebSocket 接线 · 回环打破（稳态）', () => {
  it('接管之后 joint_state 回推只写 actualJoints，commandJoints **引用不变**', async () => {
    const h = await connected();
    // 先消费掉接管握手，此后才是稳态 —— 顺带钉住"握手只发生一次"。
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 5 },
    });
    const before = useRobotStore.getState().commandJoints;
    for (let i = 1; i < 50; i += 1) {
      h.factory.last.deliver({
        version: 1,
        type: 'joint_state',
        joints: { ...home, shoulder: 5 + i * 0.5 },
      });
    }
    const after = useRobotStore.getState();
    expect(after.commandJoints).toBe(before); // toBe：引用级断言
    expect(after.actualJoints.shoulder).toBeCloseTo(5 + 49 * 0.5, 9);
    expect(after.controlSource).toBe('real');
  });

  it('回推不会反过来触发新的下发（无 命令→状态→命令 回环）', async () => {
    const h = await connected();
    for (let i = 0; i < 30; i += 1) {
      h.factory.last.deliver({
        version: 1,
        type: 'joint_state',
        joints: { ...home, shoulder: 10 + i },
      });
    }
    h.timer.advance(500);
    expect(h.frames('joint_command')).toHaveLength(0);
  });
});

describe('WebSocket 接线 · 重连补发', () => {
  it('断线重连成功后自动补发当前命令', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 30);
    h.timer.advance(100);
    const beforeDrop = h.frames('joint_command').length;
    expect(beforeDrop).toBeGreaterThan(0);

    // 网络掉线 → 退避 500ms → 重连 → 握手成功
    h.factory.last.drop(1006, 'network lost');
    h.timer.advance(500);
    expect(h.factory.count).toBe(2);
    h.factory.last.open();
    h.timer.advance(10);

    // 补发发生在新连接上，故按新 socket 计数（旧 socket 的帧不再增长）
    const after = h.factory.last.framesOfType('joint_command');
    expect(after).toHaveLength(1);
    expect((after[0].joints as Record<string, number>).shoulder).toBeCloseTo(30, 9);
    expect(h.transport.stats().reconnects).toBe(1);
  });

  it('断线期间的命令改动也会被补发（不是补发旧值）', async () => {
    const h = await connected();
    const store = useRobotStore.getState();
    store.setJoint('shoulder', 10);
    h.timer.advance(100);

    h.factory.last.drop(1006);
    // 断线期间用户继续拖动
    store.setJoint('shoulder', 42);
    h.timer.advance(500);
    h.factory.last.open();
    h.timer.advance(10);

    // 补发的必须是**当前**命令（42），而不是断线前那条（10）
    const cmds = h.factory.last.framesOfType('joint_command');
    expect(cmds).toHaveLength(1);
    expect((cmds[0].joints as Record<string, number>).shoulder).toBeCloseTo(42, 9);
  });
});

describe('WebSocket 接线 · 设备侧自主变化（origin=device）', () => {
  /** 先消费掉首次接管握手，此后才是稳态 */
  async function steady(): Promise<Harness> {
    const h = await connected();
    h.factory.last.deliver({ version: 1, type: 'joint_state', joints: { ...home } });
    return h;
  }

  it('★ 摇杆改动下位机 ⇒ 命令侧跟随（滑杆 / 主臂不再停在旧值）', async () => {
    const h = await steady();
    const before = useRobotStore.getState().commandJoints;
    const nudged = { ...home, shoulder: 42, elbow: 130 };

    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: nudged,
      origin: 'device',
    });

    const s = useRobotStore.getState();
    expect(s.commandJoints).not.toBe(before); // 确实改写了命令侧（这正是跟随）
    expect(s.commandJoints.shoulder).toBeCloseTo(42, 9);
    expect(s.commandJoints.elbow).toBeCloseTo(130, 9);
    // 外部驱动下"命令 = 现状"，两侧同步
    expect(s.actualJoints.shoulder).toBeCloseTo(42, 9);
    expect(s.controlSource).toBe('real');

    // 目标同步到设备现状，否则把手停在旧位置、面板误报"还差多远"
    const tcp = endEffectorPose(model, s.commandJoints).position;
    expect(s.target[0]).toBeCloseTo(tcp[0], 9);
    expect(s.target[2]).toBeCloseTo(tcp[2], 9);
  });

  it('★ 跟随**绝不**回发命令（无 设备 → 上位机 → 设备 的对抗）', async () => {
    const h = await steady();
    for (let i = 1; i <= 20; i += 1) {
      h.factory.last.deliver({
        version: 1,
        type: 'joint_state',
        joints: { ...home, shoulder: 10 + i },
        origin: 'device',
      });
    }
    h.timer.advance(500); // 足够放掉任何 trailing
    // 若不抑制下发，每次摇杆上报都会把设备刚做的动作推回去
    expect(h.frames('joint_command')).toHaveLength(0);
  });

  it('缺省 origin（命令回执）**不**跟随 —— 引用级不变', async () => {
    const h = await steady();
    const before = useRobotStore.getState().commandJoints;
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 33 },
    });
    const s = useRobotStore.getState();
    expect(s.commandJoints).toBe(before); // toBe：引用级
    expect(s.actualJoints.shoulder).toBeCloseTo(33, 9);
  });

  it('未知 origin 取值按命令处理（宁可少跟随，也不误判方向）', async () => {
    const h = await steady();
    const before = useRobotStore.getState().commandJoints;
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...home, shoulder: 33 },
      origin: 'DEVICE', // 大小写不同：不当成 device
    });
    expect(useRobotStore.getState().commandJoints).toBe(before);
  });

  it('值没变（差 < 0.01°）时不动引用，避免无谓重渲染', async () => {
    const h = await steady();
    const before = useRobotStore.getState().commandJoints;
    h.factory.last.deliver({
      version: 1,
      type: 'joint_state',
      joints: { ...before },
      origin: 'device',
    });
    expect(useRobotStore.getState().commandJoints).toBe(before);
  });
});

describe('WebSocket 接线 · 状态面板', () => {
  it('hello 后统计里带上 device 与模型一致性结论', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'hello',
      model: backendInfoFromLocal(model),
      device: 'sim',
    });
    h.timer.advance(300); // 等一次 poll

    const stats = useRobotStore.getState().transportStats;
    expect(stats).not.toBeNull();
    expect(stats?.kind).toBe('websocket');
    expect(stats?.rttMs ?? null).toBeNull();
    const wsStats = stats as { device?: string | null; modelMismatch?: string | null };
    expect(wsStats.device).toBe('sim');
    expect(wsStats.modelMismatch).toBeNull();
  });

  it('限位拒绝回执计入 rejected，连接仍保持', async () => {
    const h = await connected();
    h.factory.last.deliver({
      version: 1,
      type: 'error',
      code: 'JOINT_LIMIT',
      message: 'ERR JOINT elbow 95.00 (limit 108.44..141.86)',
    });
    h.timer.advance(300);
    const stats = useRobotStore.getState().transportStats;
    expect(stats?.rejected).toBe(1);
    expect(useRobotStore.getState().connection).toBe('connected');
  });
});
