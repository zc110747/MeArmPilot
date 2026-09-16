#!/usr/bin/env node
/**
 * verify_device_follow.mjs —— 「设备侧自主变化 → 上位机跟随」端到端验收
 *
 * 这条链路解决的是什么
 * --------------------
 * 机械臂在真机上可以被**上位机以外的**手段改动：硬件摇杆、红外遥控、面板手拧、
 * 调试串口直控。这时上位机如果不知道，界面就会停在一个**过时的位置**上继续
 * 显示（用户看到的是一台不存在的机器）。
 *
 * 所以设备侧一旦变化，必须异步上报：
 *
 *     JOY / SET（设备侧输入）
 *        │
 *        ▼
 *     位置推进 ──▶ `# SERVO S6=… S7=… S8=… S9=…`   ← 异步事件，不是某条命令的应答
 *        │                              （docs/serial-v1.md §3.1）
 *        ▼
 *     backend：ReplyServo → ServoAnglesToJoints → joint_state{origin:"device"}
 *        │
 *        ▼
 *     frontend：transportBridge 见 origin=device ⇒ store.followDevice()（命令侧跟随，
 *                且**抑制回发**，否则每一次摇杆上报都会反弹一条命令下来）
 *
 * 为什么必须分成两条上报路径
 * --------------------------
 *   JR 受理   → 位置推进 → `STATE …`      关节角，**命令引起**   ⇒ origin=command
 *   SET/JOY   → 位置推进 → `# SERVO …`    舵机角，**设备侧变化** ⇒ origin=device
 *
 * 混成一条会立刻出两个真实缺陷：
 *   ① 用 STATE 跑外部变化 ⇒ 界面跟随没问题，但**命令的语义被设备覆盖**：
 *      拖滑杆时设备还在斜坡上，摇杆角一路把滑杆"回拉"到半路。
 *   ② 用 `# SERVO` 的**形状**去猜来源 ⇒ 前端只能猜。猜"Actual ≠ Command 就跟随"
 *      同样会在拖动过程中把命令侧拉走；猜"一律跟随"吃掉命令语义。
 *   ⇒ 跟随与否只能由**显式标注**决定（`origin`），不能让前端猜。
 *
 * 低优先级（本脚本第 4/5 阶段专测）
 * --------------------------------
 *   自主上报的**触发优先级低于正常通讯的交互指令**：被挡下时**让位，但不丢弃**
 *   （保留脏标志，等命令了结后用**当时**的值补报 ⇒ 天然 latest-wins）。
 *   固件侧落点：`arm_report_tick()` 排在 `cmd_poll()` 之后，且只在 `uart_tx_used()==0`
 *   （TX 环全空）时才发。sim 侧落点：`jrBusy` 在途标志抑制 + 命令了结后 `flushReport()`。
 *   ⚠️ 闸门本身由 Go 单测证明，本脚本证明的是**端到端可观测后果** ——
 *      为什么不能指望脚本走到闸门，见 `phase4()` 的注释（窗口太窄）。
 *
 * 本脚本的最大价值：**仿真与真机跑的是同一份脚本**
 * ------------------------------------------------
 *   中午（硬件不在手边）： `node core/tools/verify_device_follow.mjs --config sim`
 *   晚上（插上机械臂）：   `node core/tools/verify_device_follow.mjs --config serial`
 *   差别只在 `--config`。判定项、容差、时序完全一致 —— 这样"仿真通过了"才对夜间
 *   真机跑有参考价值。`device_command` 直通的是**原始设备指令**，在 sim 与真机上
 *   走的是同一条 `WriteLine` 出口，不存在"仿真走了捷径"。
 *
 * 用法
 * ----
 *   node core/tools/verify_device_follow.mjs                  # 复用 8090 上的实例
 *   node core/tools/verify_device_follow.mjs --config sim     # 起一个仿真后端
 *   node core/tools/verify_device_follow.mjs --config serial  # 起一个真机后端（会驱动舵机！）
 *   node core/tools/verify_device_follow.mjs --dry-run        # 只打印计划
 *   node core/tools/verify_device_follow.mjs --only 3,4       # 只跑指定阶段
 *
 * 前置
 * ----
 *   * `backend/bin/armpilot-backend.exe` 已构建（`cd backend && go build -o bin/armpilot-backend.exe .`）
 *   * `--config serial` 时：机械臂接在 backend/config.serial.yaml 写的串口上
 *   * Node 18+（用内置全局 `WebSocket`，无第三方依赖）
 *
 * 退出码：0 = 全 PASS；1 = 有 FAIL；2 = 前置缺失或脚本自身崩溃。
 *
 * ⚠️ 真机上的"能看见"与"真到位"是两件事
 * --------------------------------------
 *   真机固件**无位置反馈**，`joint_state` 是**开环目标值**。本脚本全绿只证明
 *   「上报被生成、被解析、被标成 device、被广播、且被命令让位」——
 *   **不证明**机械臂物理上真的动了。物理证据只有相机（见 verify_serial_e2e.mjs）。
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, writeFileSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

// ---------------------------------------------------------------------------
// 路径 / 参数
// ---------------------------------------------------------------------------

const TOOLS_DIR = path.dirname(fileURLToPath(import.meta.url));
// ⚠️ 本脚本住在 core/tools/ ⇒ 仓库根要退**两层**。
//    只退一层会得到 <repo>/core：backend 找不到、输出目录写进 core/.workbuddy/。
//    （Grep 记录：verify_serial_e2e.mjs 在 Core 重构时踩过同一个坑。）
const ROOT = path.resolve(TOOLS_DIR, '..', '..');
const BACKEND_DIR = path.join(ROOT, 'backend');
const BACKEND_EXE = path.join(BACKEND_DIR, 'bin', 'armpilot-backend.exe');

const argv = process.argv.slice(2);
function opt(name, fallback) {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && argv[i + 1] && !argv[i + 1].startsWith('--') ? argv[i + 1] : fallback;
}
const flag = (name) => argv.includes(`--${name}`);

const A = {
  http: opt('http', process.env.BACKEND_HTTP ?? 'http://127.0.0.1:8090'),
  ws: opt('ws', process.env.BACKEND_WS ?? 'ws://127.0.0.1:8090/ws/joint'),
  // 配置文件：sim | serial | 任意路径 | null（= 只复用已有实例）
  config: opt('config', null),
  outDir: opt('out', path.join(ROOT, '.workbuddy', 'captures', `follow_${stamp()}`)),
  // 跟随幅度下限：一次 JOY 的步长是**舵机度**，经标定换算到关节度。取 0.5° 是为了
  // 只证明"确实动了"，不去假定 scale —— 假定 scale 就等于在验收脚本里重算真值。
  followTol: Number(opt('follow-tol', 0.5)),
  // 命令收敛容差：固件只吃**整数舵机度**，量化上限 0.347°(肩) / 0.209°(肘)。
  // 取 0.4 是"量化误差 + 一点余量"，再大就变成对真实偏差不敏感了。
  ackTol: Number(opt('ack-tol', 0.4)),
  settle: Number(opt('settle', 700)),
  /** 阶段 ④：命令受理后多久插入摇杆（默认 60ms = 已受理、仍在斜坡上）。 */
  inflightMs: Number(opt('inflight-ms', 60)),
  /** 阶段 ④：连续插入几次拨动。 */
  nudges: opt('nudges', '3'),
  dryRun: flag('dry-run'),
  keepBackend: flag('keep-backend'),
  only: opt('only', null),
};

function stamp() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}`;
}

// ---------------------------------------------------------------------------
// 结果收集（与 verify_serial_e2e.mjs 同风格）
// ---------------------------------------------------------------------------

const results = [];
function check(name, ok, detail = '') {
  results.push({ name, ok, detail });
  console.log(`[${ok ? 'PASS' : 'FAIL'}] ${name}${detail ? `  — ${detail}` : ''}`);
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const info = (msg) => console.log(`  · ${msg}`);
const warn = (msg) => console.log(`  ! ${msg}`);
const phase = (msg) => console.log(`\n── ${msg} ${'─'.repeat(Math.max(0, 54 - msg.length))}`);
const f3 = (n) => (Number.isFinite(n) ? n.toFixed(3) : 'NaN');

const T0 = Date.now();
const logLines = [];
function log(tag, msg) {
  const rel = ((Date.now() - T0) / 1000).toFixed(2).padStart(7);
  const line = `[${rel}s] ${tag.padEnd(9)} ${msg}`;
  logLines.push(line);
  console.log(line);
}
function flushLog(outDir) {
  try {
    mkdirSync(outDir, { recursive: true });
    writeFileSync(path.join(outDir, 'run.log'), logLines.join('\n') + '\n', 'utf8');
    return path.join(outDir, 'run.log');
  } catch {
    return null;
  }
}

// ---------------------------------------------------------------------------
// 后端进程
// ---------------------------------------------------------------------------

async function probeBackend(timeoutMs = 800) {
  const ac = new AbortController();
  const t = setTimeout(() => ac.abort(), timeoutMs);
  try {
    const res = await fetch(`${A.http}/healthz`, { signal: ac.signal });
    return res.ok ? await res.json() : null;
  } catch {
    return null;
  } finally {
    clearTimeout(t);
  }
}

let backendChild = null;

async function startBackend() {
  const existing = await probeBackend();
  if (existing) {
    info(`复用 ${A.http} 上的实例（device=${existing.device}）`);
    if (A.config) {
      const want = A.config === 'sim' ? 'sim' : A.config === 'serial' ? 'serial' : null;
      if (want && existing.device !== want) {
        throw new Error(
          `${A.http} 上的实例是 device=${existing.device}，但 --config ${A.config} 要求 ${want}。` +
            `先停掉它，或换一组端口。`,
        );
      }
    }
    return { external: true, device: existing.device };
  }
  if (!A.config) {
    throw new Error(
      `${A.http} 上没有实例，且未指定 --config。` +
        `用 --config sim 起一个仿真后端，或 --config serial 起一个真机后端。`,
    );
  }
  if (!existsSync(BACKEND_EXE)) {
    throw new Error(`未找到 ${BACKEND_EXE}；先在 backend/ 下 go build -o bin/armpilot-backend.exe .`);
  }
  const cfg = A.config === 'sim' ? 'config.yaml' : A.config === 'serial' ? 'config.serial.yaml' : A.config;
  const child = spawn(BACKEND_EXE, ['-c', cfg], { cwd: BACKEND_DIR, stdio: 'ignore' });
  backendChild = child;
  child.on('exit', (code) => info(`后端进程退出 code=${code}`));

  const deadline = Date.now() + 15000;
  while (Date.now() < deadline) {
    const h = await probeBackend(1000);
    if (h) {
      info(`后端就绪 device=${h.device} linked=${h.linked}（配置 ${cfg}）`);
      return { external: false, device: h.device };
    }
    await sleep(200);
  }
  throw new Error(`后端 15s 内未就绪（配置 ${cfg}；真机模式请检查串口号与接线）`);
}

/**
 * 真机模式下要等链路就绪。后端开门后会有一段**开机静默窗口 + 暖机探测**，
 * 这期间 `linked=false`，此时下发的指令会被 `DEVICE_UNAVAILABLE` 直接拒掉。
 * 没有这道门的话，第一条 JOY 的报错就会藏在后面的日志里没人注意。
 */
async function waitLinked(timeoutMs = 12000) {
  const h = await probeBackend(800);
  if (h?.device !== 'serial') return { needed: false, linked: true };
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const cur = await probeBackend(800);
    if (cur?.linked) return { needed: true, linked: true };
    log('ready', `linked=false，继续等（剩 ${((deadline - Date.now()) / 1000).toFixed(1)}s）`);
    await sleep(200);
  }
  return { needed: true, linked: false };
}

function stopBackend() {
  if (backendChild && !A.keepBackend) {
    try {
      backendChild.kill();
    } catch {
      /* 已退出 */
    }
  }
}

// ---------------------------------------------------------------------------
// WebSocket 链路（Node 内置全局 WebSocket）
// ---------------------------------------------------------------------------

class Link {
  constructor(ws) {
    this.ws = ws;
    this.hello = null;
    this.frames = []; // { at, joints, origin }
    this.errors = [];
    ws.addEventListener('message', (ev) => {
      let env;
      try {
        env = JSON.parse(typeof ev.data === 'string' ? ev.data : ev.data.toString());
      } catch {
        return;
      }
      if (env.type === 'joint_state' && env.joints) {
        // ⚠️ `origin` 缺省按 **command** 处理（后端 `omitempty`，字段可能不出现）。
        //    宁可少跟随，也不能把一帧来源不明的状态当成"设备侧变化"去拖命令侧。
        this.frames.push({ at: Date.now(), joints: env.joints, origin: env.origin ?? 'command' });
      } else if (env.type === 'hello') {
        this.hello = env;
      } else if (env.type === 'error') {
        this.errors.push(`${env.code}: ${env.message}`);
        console.log(`  ! 收到 error：${env.code}: ${env.message}`);
      }
    });
  }

  static async open(url) {
    return new Promise((resolve, reject) => {
      const ws = new WebSocket(url);
      const t = setTimeout(() => reject(new Error(`WebSocket 连接超时：${url}`)), 8000);
      ws.addEventListener('open', () => {
        clearTimeout(t);
        resolve(new Link(ws));
      });
      ws.addEventListener('error', (e) => {
        clearTimeout(t);
        reject(new Error(`WebSocket 连接失败：${url} (${e?.message ?? e?.type ?? 'error'})`));
      });
    });
  }

  send(obj) {
    this.ws.send(JSON.stringify({ version: 1, timestamp: Date.now(), ...obj }));
  }

  /** 直通一行**原始设备指令**（JOY / SET / STATUS），走的是真机同一条 WriteLine 出口。 */
  deviceCommand(line) {
    this.send({ type: 'device_command', line });
  }

  last() {
    return this.frames.length ? this.frames[this.frames.length - 1] : null;
  }

  since(t) {
    return this.frames.filter((f) => f.at >= t);
  }
  deviceSince(t) {
    return this.since(t).filter((f) => f.origin === 'device');
  }
  commandSince(t) {
    return this.since(t).filter((f) => f.origin !== 'device');
  }

  /** 等到 `pred(last)` 为真或超时。 */
  async until(pred, timeoutMs, stepMs = 30) {
    const deadline = Date.now() + timeoutMs;
    while (Date.now() < deadline) {
      const l = this.last();
      if (l && pred(l)) return true;
      await sleep(stepMs);
    }
    return false;
  }

  close() {
    try {
      this.ws.close();
    } catch {
      /* 已关闭 */
    }
  }
}

function maxDiff(a, b) {
  let m = 0;
  for (const k of Object.keys(b)) {
    if (typeof a?.[k] !== 'number') return Number.POSITIVE_INFINITY;
    m = Math.max(m, Math.abs(a[k] - b[k]));
  }
  return m;
}

// ---------------------------------------------------------------------------
// 阶段编排
// ---------------------------------------------------------------------------

const JOINT_IDS = ['base', 'shoulder', 'elbow', 'gripper'];
/** 摇杆 id → 关节名。7 = 肩（S7），与 docs/serial-v1.md §2 的通道映射一致。 */
const JOY_ID_SHOULDER = 7;

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

let home = null;
/** 肩关节限位（来自 hello.model.limits —— 唯一真值 robot.yaml 的转发）。 */
let shLim = null;
/** 本次选用的摇杆 raw 值与方向（+1 向上 / -1 向下）。 */
let nudgeRaw = 1023;
let nudgeDir = -1;
/** 一次 JOY 实际推动多少**关节度**（在阶段 ② 实测，不假定标定 scale）。 */
let stepPerJoy = 0;

/**
 * 选摇杆方向：**挑行程更大的一侧**。
 *
 * ⚠️ 这一条不是锦上添花，是必须的。真值：肩关节 HOME = 0.85°，
 *    限位 [-6.09°, 49.45°] —— 向下只有约 7° 的行程，向上有约 48.6°。
 *    如果脚本写死"向下拨"（raw=1023），第一次 JOY 之后就被关节限位钳住，
 *    后面几次全成了空动作，而脚本只会报"没收到多帧上报"——
 *    把**限位**造成的现象误报成**上报机制**的缺陷。
 *    （本次第一版就踩了这个坑：连拨 5 次只收到 1 帧。教训写在这里，
 *      免得下次有人去"修" `# SERVO` 上报。）
 *
 * 方向映射：id≠8 时 `positive = pastLo`（raw<200 为正方向）。
 */
function pickNudgeDirection(limits, homePose) {
  const row = (limits ?? []).find((r) => r.id === 'shoulder');
  if (!row) {
    warn('hello 未带 shoulder 限位，回退到 raw=0（向上）');
    return { raw: 0, dir: 1, row: null };
  }
  const up = row.max - homePose.shoulder;
  const down = homePose.shoulder - row.min;
  info(`肩关节行程：向上 ${f3(up)}° / 向下 ${f3(down)}°（限位 [${f3(row.min)}, ${f3(row.max)}]）`);
  return up >= down ? { raw: 0, dir: 1, row } : { raw: 1023, dir: -1, row };
}

/** 阶段 1：基线 —— hello 与开机快照。 */
async function phase1(link) {
  phase('① 基线：hello 与开机快照');
  const t0 = Date.now();
  while (!link.hello && Date.now() - t0 < 3000) await sleep(50);
  check('收到 hello（携带模型真值）', !!link.hello, `device=${link.hello?.device ?? '(未收到)'}`);
  if (link.hello?.model?.jointOrder) {
    const jo = link.hello.model.jointOrder.join(',');
    check('jointOrder = base,shoulder,elbow,gripper', jo === JOINT_IDS.join(','), jo);
  }

  const ok = await link.until((l) => l.origin === 'command', 3000);
  check('开机即收到状态快照', ok, ok ? '' : '3s 内未收到 joint_state');
  if (!ok) return false;

  // 快照的 origin 必须是 command。若把快照标成 device，前端会在**没有任何设备变化**
  // 的情况下触发一次 followDevice + 抑制下发 —— 表现为"一打开页面滑杆就自己跳一下"。
  const snap = link.last();
  check('快照 origin=command（不得误标 device）', snap.origin === 'command', `origin=${snap.origin}`);

  home = { ...snap.joints };
  const hp = link.hello?.model?.homePose;
  if (hp) {
    const d = maxDiff(home, hp);
    if (link.hello.device === 'sim') {
      check('开机位置 = homePose（±0.5°）', d <= 0.5, `max|Δ|=${f3(d)}°`);
    } else {
      // 真机上这条**可能合理地不成立**：开串口前的舵机中立位由 Uno 上电决定，
      // 且若在此之前有人拨过摇杆，目标值已经离开 HOME。所以只报信息，不判 FAIL ——
      // 把预期内的差异记成 FAIL 会训练出"忽略 FAIL"的习惯。
      info(`开机位置与 homePose 的差 max|Δ|=${f3(d)}°（真机模式下不判 FAIL，见脚本注释）`);
    }
  }
  info(`HOME = ${JSON.stringify(home)}`);

  const pick = pickNudgeDirection(link.hello?.model?.limits, home);
  nudgeRaw = pick.raw;
  nudgeDir = pick.dir;
  shLim = pick.row;
  info(`本次摇杆输入：JOY ${JOY_ID_SHOULDER} ${nudgeRaw}（方向 ${nudgeDir > 0 ? '向上' : '向下'}，行程更大的一侧）`);
  return true;
}

/** 阶段 2：设备侧自主变化必须异步上报，并标成 origin=device。 */
async function phase2(link) {
  phase('② 设备侧自主变化（JOY = 摇杆）必须上报为 origin=device');
  const t = Date.now();
  link.deviceCommand(`JOY ${JOY_ID_SHOULDER} ${nudgeRaw}`);

  const gotDevice = await link.until((l) => l.origin === 'device', 2000);
  check('收到 origin=device 的 joint_state', gotDevice,
    gotDevice ? '' : '2s 内未收到；检查 sim/固件是否把外部变化错走成 STATE 通道');

  await sleep(A.settle);
  const dev = link.deviceSince(t);
  const cmd = link.commandSince(t);
  check('设备变化期间没有混入 origin=command 的状态帧',
    cmd.length === 0,
    cmd.length ? `${cmd.length} 帧 origin=command（外部变化被错走成 STATE？）` : '0 帧');

  if (!dev.length) return false;
  const last = dev[dev.length - 1].joints;
  const delta = last.shoulder - home.shoulder;
  check(`命令侧可跟随：肩角离开 HOME（>${A.followTol}°）`, Math.abs(delta) > A.followTol,
    `Δ=${f3(delta)}°`);
  // 方向必须与摇杆方向一致 —— 这一条能抓到"摇杆方向映射写反"（真机上表现为
  // "往上拨反而往下走"，而幅度检查会照常通过）。
  check('移动方向与摇杆方向一致', Math.sign(delta) === nudgeDir || Math.abs(delta) < 1e-6,
    `摇杆 ${nudgeDir > 0 ? '向上' : '向下'}，肩角 Δ=${f3(delta)}°`);

  // 实测"一次 JOY 走多远"，供阶段 ③ 计算不撞限位的拨动次数。
  stepPerJoy = Math.abs(delta);
  if (stepPerJoy > 0) info(`实测步长：一次 JOY ⇒ ${f3(stepPerJoy)} 关节度`);
  else warn('一次 JOY 未产生可见位移，阶段 ③ 将退化为按固定次数拨动');

  // 未动的关节不得被"带跑"：controller 的 mergeState 对缺项要沿用上一帧，
  // 而不是补 0。补 0 会让另外三个关节每次都跳到 0（一眼可见但很容易漏测）。
  const moved = JOINT_IDS.filter((k) => Math.abs((last[k] ?? NaN) - home[k]) > 0.5);
  check('只有 shoulder 移动，其余关节未被带跑',
    moved.length === 1 && moved[0] === 'shoulder',
    `实际移动：${moved.join(',') || '(无)'}`);
  return true;
}

/** 阶段 3：上报是**事件流**（多次推进 ⇒ 多帧），不是命令式的一次性回执。 */
async function phase3(link) {
  phase('③ 上报是异步事件流（连拨多次 ⇒ 多帧，而非一次回执）');
  // 次数按**剩余行程**算，不写死：写死会在行程小的一侧撞限位，
  // 于是"上报是事件流"这条被判 FAIL，而真正的锅是限位。
  const headroom = shLim
    ? (nudgeDir > 0 ? shLim.max - home.shoulder : home.shoulder - shLim.min)
    : 20;
  const budget = headroom * 0.4; // 只用 40% 行程，给阶段 ④⑤ 留余量
  const count = stepPerJoy > 0 ? clamp(Math.floor(budget / stepPerJoy), 2, 8) : 4;
  info(`拨动 ${count} 次（预算 ${f3(budget)}° / 步长 ${f3(stepPerJoy)}°，留 60% 余量不撞限位）`);

  const t = Date.now();
  const seen = [];
  for (let i = 0; i < count; i++) {
    link.deviceCommand(`JOY ${JOY_ID_SHOULDER} ${nudgeRaw}`);
    await sleep(120);
    const j = link.last()?.joints;
    if (j) seen.push(f3(j.shoulder));
  }
  await sleep(A.settle);
  const dev = link.deviceSince(t);
  check('多帧 origin=device（≥2 帧）', dev.length >= 2,
    `${dev.length} 帧（每帧一次位置推进；单帧 = 上报被折叠成了回执）`);
  check('连拨期间无 error', link.errors.length === 0, link.errors.join('; ') || '0 条');
  const last = dev.length ? dev[dev.length - 1].joints : null;
  if (last) {
    const travel = last.shoulder - home.shoulder;
    info(`肩角现在 ${f3(last.shoulder)}°，累计离开 HOME ${f3(travel)}°`);
    // 再确认一次"没被限位钳住"：真的推进了足够多，说明次数选得合理。
    check('累计行程未被关节限位钳住', Math.abs(travel) >= Math.abs(stepPerJoy) * 1.5,
      `累计 ${f3(travel)}° vs 单次 ${f3(stepPerJoy)}°`);
  }
  return dev.length >= 2;
}

/**
 * 等命令侧（origin=command）的状态收敛到目标；返回最优 max|Δ|。
 *
 * ⚠️ 只看 origin=command 的帧。设备侧帧（`# SERVO`）**不是命令的回执**，
 *    把它算进来会让"命令有没有被接受"这件事失去判据 —— 这是本脚本第一版
 *    在并发阶段误判 FAIL 的原因：命令的斜坡被设备输入接手后改走 `# SERVO`，
 *    于是永远看不到收敛帧，被读成"命令没收敛"。
 *
 *    根因是 sim 的 `externalMotion` 是**单一全局标志**（两个来源的推进共用它）：
 *    命令在途时一旦来了设备输入，后续整个推进都算设备侧。
 *    这是 sim 的标注选择（设备对物理执行器有最后发言权），**不是上报机制缺陷**；
 *    但在真机上不会这样 —— 真机 `OK JR` 回的是**目标角**且立即回，
 *    所以命令侧状态的收敛由 `OK JR` 独立保证，与设备上报无关。
 */
async function waitConverge(link, target, tSince, timeoutMs = 6000) {
  let best = Number.POSITIVE_INFINITY;
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    for (const fr of link.commandSince(tSince)) best = Math.min(best, maxDiff(fr.joints, target));
    if (best <= A.ackTol) break;
    await sleep(40);
  }
  return best;
}

/** 行程中点 —— 离两端都远，不会被限位钳位，适合做"干净收敛"的目标。 */
function midShoulder() {
  if (!shLim) return home.shoulder + 10;
  return clamp((shLim.min + shLim.max) / 2, shLim.min + 5, shLim.max - 5);
}

/**
 * 挑一个**离当前位置足够远**的肩角目标（≥ MIN_TRAVEL）。
 *
 * ⚠️ 这条不是讲究，是验收前提。真值里"命令"是**有限角速度逼近**：
 *    目标与当前位置相同时**没有位移 ⇒ 没有位置推进 ⇒ 一帧状态都不会产生**，
 *    于是"等收敛"拿到的是 `NaN`，看起来像"命令没被受理"。
 *    本脚本第一版就在这里踩过：阶段 ⑦ 要把机器收回 HOME，而阶段 ⑥ 已经收敛到
 *    HOME 了 ⇒ 零位移 ⇒ max|Δ|=NaN ⇒ 误报 FAIL。
 *    同样的坑还有"贴着限位下发"（固件钳位后同样零位移）。
 */
const MIN_TRAVEL = 10;
function farShoulderTarget(cur) {
  // ⚠️ 候选值必须**夹进限位内**。写 `home.shoulder - 20` 这种"看起来安全"的常量
  //    会落到限位外 ⇒ 命令被同步拒掉（ERR JOINT）⇒ 同样是零状态帧。
  const lo = shLim ? shLim.min + 5 : home.shoulder - 20;
  const hi = shLim ? shLim.max - 5 : home.shoulder + 30;
  const cands = [midShoulder(), clamp(home.shoulder - 20, lo, hi), clamp(home.shoulder + 30, lo, hi)];
  for (const c of cands) if (Math.abs(c - cur) >= MIN_TRAVEL) return c;
  return cands[0];
}

/** 构造一条"只动肩"的关节命令（其余关节保持 HOME）。 */
function shoulderTarget(v) {
  return { base: home.base, shoulder: v, elbow: home.elbow, gripper: home.gripper };
}

/**
 * 阶段 4：**干净状态下**命令能收敛（先把命令通道本身钉成基线）。
 *
 * 为什么必须单独测一遍：并发阶段一旦失败，光看结果无法区分"并发导致"与
 * "命令通道本来就坏"—— 两种根因的修法完全不同。先钉住无干扰的基线，
 * 后面所有并发阶段的 FAIL 才能被归因。
 */
async function phase4(link) {
  phase('④ 基线：无干扰时命令收敛到目标');
  await sleep(300);
  const cur = link.last()?.joints ?? home;
  const target = shoulderTarget(farShoulderTarget(cur.shoulder));
  info(`当前位置肩 ${f3(cur.shoulder)}° → 目标 ${f3(target.shoulder)}°（位移 ${f3(Math.abs(target.shoulder - cur.shoulder))}°）`);
  const t = Date.now();
  link.send({ type: 'joint_command', joints: target });
  const best = await waitConverge(link, target, t);
  check(`干净命令收敛（≤${A.ackTol}°，量化上限 0.347°）`, best <= A.ackTol,
    `目标肩 ${f3(target.shoulder)}°，max|Δ|=${f3(best)}°${Number.isNaN(best) ? '（无状态帧 ⇒ 命令未受理，或目标与当前位置相同）' : ''}`);
  check('干净命令的状态帧 origin=command', link.commandSince(t).length > 0,
    `${link.commandSince(t).length} 帧`);
  return best <= A.ackTol;
}

/**
 * 阶段 5：**正向并发** —— 命令在途（仍在斜坡）时插入摇杆拨动。
 *
 * 每条断言只证一件事：
 *   a. 命令被受理（出现 origin=command 帧）⇒ ACK 门控没被设备上报顶掉；
 *   b. 全程无 error；
 *   c. 被让位掉的拨动**不丢**：命令平息后收到一帧 origin=device，其肩角 ≈
 *      命令目标 + 全部拨动（latest-wins ⇒ 一帧含全部，不积压中间帧）。
 *
 * ⚠️ 本阶段**故意不断言"命令收敛到目标"**：拨动是命令之后发生的真实改动，
 *    设备对物理执行器有最后发言权，位置本来就该是"目标 + 拨动"。
 *    在这里断言收敛会把**正确的语义**判成 FAIL。
 *    命令的收敛由阶段 ④（干净）与阶段 ⑥（反向抢占）负责。
 *
 * ⚠️ 关于注入时机的诚实说明（别把这段读成"已证明让位闸门被走到"）
 * ----------------------------------------------------------
 *   「让位」闸门本身（固件：`uart_tx_used()==0`；sim：`jrBusy`）**在本脚本里无法
 *   被可靠地走到**，原因是时序窗口太窄：
 *     * sim 的在途窗口 = `LatencyMs`（默认 15ms），且 `deliver()` 到期会用**绝对
 *       目标**覆盖舵机目标 ⇒ 窗口内插入的拨动会被整个丢掉。这是 sim 的建模选择
 *       （"延迟期内不回执、不生效"），**不是**上报机制的缺陷。
 *     * 真机上 TX 环大部分时间是空的（115200 下发几十字节很快），闸门也很少拦人。
 *   所以闸门由 **Go 单测**证明（`TestSimExternalReportYieldsToInflightJR` 直接用
 *   `jrBusy` 构造在途态；固件侧由 `arm_report_tick()` 的结构保证），
 *   本脚本证明的是它的**端到端可观测后果**（c）。两者互补，不可互相替代。
 *   默认注入点 `--inflight-ms 60` = 命令已受理、位置仍在斜坡上，对应真机上
 *   "机械臂还在动的时候又拨了摇杆"。
 */
async function phase5(link) {
  phase('⑤ 正向并发：命令在途插入摇杆 —— 命令优先，且被让位的拨动不丢');
  // 先把设备推离 HOME，制造"设备实际位置 ≠ 命令目标"的对抗局面。
  const t = Date.now();
  for (let i = 0; i < 3; i++) {
    link.deviceCommand(`JOY ${JOY_ID_SHOULDER} ${nudgeRaw}`);
    await sleep(90);
  }
  await sleep(300);
  const devicePos = link.last()?.joints ?? home;
  info(`设备侧实际位置（命令发出前）：肩 ${f3(devicePos.shoulder)}°`);

  // 目标取**关于 HOME 的镜像** —— 这样"命令被设备拖走"会被 max|Δ| 立刻抓到。
  // 若固定取一个值，当设备恰好也在它附近时两者同向，缺陷会被掩盖成 PASS
  //（验收脚本自己制造假绿，是这里最贵的错法）。
  let want = 2 * home.shoulder - devicePos.shoulder;
  if (shLim) want = clamp(want, shLim.min + 5, shLim.max - 5); // 贴限位会被钳位 ⇒ 判定失真
  if (Math.abs(want - devicePos.shoulder) < 5) want = midShoulder();
  const target = shoulderTarget(want);
  info(`命令目标：肩 ${f3(target.shoulder)}°（与设备位置相距 ${f3(Math.abs(target.shoulder - devicePos.shoulder))}°）`);

  const tCmd = Date.now();
  link.send({ type: 'joint_command', joints: target });

  await sleep(A.inflightMs);
  const tJoy = Date.now();
  const NUDGES = Number(A.nudges);
  for (let i = 0; i < NUDGES; i++) {
    link.deviceCommand(`JOY ${JOY_ID_SHOULDER} ${nudgeRaw}`);
    await sleep(30);
  }
  info(`命令受理后 ${A.inflightMs}ms 起，连续插入 ${NUDGES} 次摇杆拨动（同一链路争用）`);

  await sleep(A.settle);

  const cmdFrames = link.commandSince(tCmd);
  check('命令被受理（出现 origin=command 帧）—— ACK 没被设备上报顶掉',
    cmdFrames.length > 0, `${cmdFrames.length} 帧 origin=command`);
  check('全程无任何 error 消息', link.errors.length === 0, link.errors.join('; ') || '0 条');

  // c. 被让位掉的拨动不能丢：命令平息后应收到一帧 origin=device，
  //    其肩角 ≈ 命令目标 + 全部拨动。
  const won = await link.until((l) => l.origin === 'device' && l.at >= tJoy, 3000);
  check('命令之后收到 origin=device 补报', won, won ? '' : '3s 内未收到（让位被实现成了丢弃？）');
  if (won) {
    await sleep(A.settle);
    const dev = link.deviceSince(tJoy);
    const got = dev.length ? dev[dev.length - 1].joints.shoulder : NaN;
    // 期望 = 命令目标 + NUDGES 次拨动。步长用阶段 ② 的**实测**值，不用标定 scale
    // 反算 —— 在验收脚本里重算真值就是本项目明确禁止的"第二处口径"。
    const expectDev = target.shoulder + nudgeDir * stepPerJoy * NUDGES;
    const dv = Math.abs(got - expectDev);
    check('补报含**全部**被让位的拨动（latest-wins，无丢失）',
      Number.isFinite(got) && dv <= Math.abs(stepPerJoy) * 0.9,
      `期望 ≈${f3(expectDev)}°（目标 ${f3(target.shoulder)}° + ${NUDGES}×${f3(stepPerJoy)}°）实测 ${f3(got)}°，差 ${f3(dv)}°`);
  }
  return cmdFrames.length > 0 && won;
}

/**
 * 阶段 6：**反向抢占** —— 设备正在斜坡上时下发命令，命令必须夺回上报通道。
 *
 * 这一段就是用户最初报告的那个症状的复现：
 *   「拖滑杆的时候滑杆被回拉」—— 设备侧的变化把命令侧拖走了。
 * 判据两条缺一不可：
 *   * 命令侧收敛到命令目标（只查"没有后续 device 帧"会被"命令压根没受理"骗过）；
 *   * 收敛之后不得再有 origin=device 帧把位置拖回去（只查收敛会被"爬完坡又发一帧"
 *     骗过）。
 */
async function phase6(link) {
  phase('⑥ 反向抢占：设备在坡上时命令夺回报通道（滑杆不被回拉）');
  // 目标**不取 HOME**：一是 HOME 可能离当前位置太近（零位移 ⇒ NaN，见 farShoulderTarget
  // 的注释），二是给阶段 ⑦ 留出真实的"复位位移"，否则收尾那一步会变成空操作。
  const cur = link.last()?.joints.shoulder ?? home.shoulder;
  const tgt = farShoulderTarget(cur);
  const target = shoulderTarget(tgt);
  info(`命令目标：肩 ${f3(tgt)}°（当前位置 ${f3(cur)}°，位移 ${f3(Math.abs(tgt - cur))}°）`);

  // 先让设备连续爬坡（上报在流），**紧接着**下发命令 —— 命令要落在设备斜坡期间。
  const t = Date.now();
  for (let i = 0; i < 4; i++) {
    link.deviceCommand(`JOY ${JOY_ID_SHOULDER} ${nudgeRaw}`);
    await sleep(25);
  }
  const devFrames = link.deviceSince(t).length;
  check('设备侧确实在上报（制造出并发前提）', devFrames > 0,
    devFrames ? `${devFrames} 帧 origin=device` : '未收到设备帧 ⇒ 本阶段前提不成立，结论无意义');

  const tCmd = Date.now();
  link.send({ type: 'joint_command', joints: target });
  const best = await waitConverge(link, target, tCmd);
  check(`命令夺回报通道并收敛到命令目标（≤${A.ackTol}°）`, best <= A.ackTol,
    `max|Δ|=${f3(best)}°${Number.isNaN(best) ? '（无 origin=command 帧 ⇒ 上报通道没被夺回）' : ''}`);

  // 收敛后再观察一段：不得有 origin=device 帧把位置拖回去。
  const tQuiet = Date.now();
  await sleep(900);
  const late = link.deviceSince(tQuiet);
  const dragged = late.filter((f) => Math.abs(f.joints.shoulder - tgt) > A.followTol);
  check('收敛后无 origin=device 帧把位置拖走（无"回拉"）', dragged.length === 0,
    dragged.length
      ? `${dragged.length} 帧把肩角带到 ${f3(dragged[dragged.length - 1].joints.shoulder)}°`
      : '0 帧');
  return best <= A.ackTol && dragged.length === 0;
}

/** 阶段 7：收尾复位（用正常的关节命令回 HOME，别把机器留在歪着的状态）。 */
async function phase7(link) {
  phase('⑦ 收尾复位');
  await sleep(300);
  const cur = link.last()?.joints ?? null;
  // 已经在 HOME 就不要下发：目标 == 当前位置 ⇒ 零位移 ⇒ 零状态帧 ⇒ NaN。
  // 把"本来就到位了"读成"复位失败"，是验收脚本最常见的假 FAIL。
  if (cur && maxDiff(cur, home) <= A.ackTol) {
    check('收尾复位到 HOME（≤0.4°）', true,
      `已在 HOME（max|Δ|=${f3(maxDiff(cur, home))}°），无需下发`);
    return true;
  }
  const t = Date.now();
  link.send({ type: 'joint_command', joints: home });
  const best = await waitConverge(link, home, t, 5000);
  check(`收尾复位到 HOME（≤${A.ackTol}°）`, best <= A.ackTol,
    `max|Δ|=${f3(best)}°${Number.isNaN(best) ? '（无状态帧 ⇒ 复位命令未被受理）' : ''}`);
  return best <= A.ackTol;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

async function main() {
  console.log('==== ArmPilot 验收：设备侧自主变化 → 上位机跟随 ====');
  console.log(`# 工程根目录   ${ROOT}`);
  console.log(`# 输出目录     ${A.outDir}`);
  console.log(`# 配置         ${A.config ?? '(复用已有实例)'}`);
  console.log(`# 跟随容差     >${A.followTol}°   ·  命令收敛 ≤${A.ackTol}°`);

  if (A.dryRun) {
    console.log('\n# 计划（--dry-run，未连接任何东西）');
    console.log('  ① 基线：hello + 开机快照 origin=command（含读取肩关节限位以选摇杆方向）');
    console.log('  ② 单次 JOY → 收到 origin=device、肩角按摇杆方向移动、其余关节不被带跑');
    console.log('  ③ 按剩余行程连拨 → 多帧事件流、无 error、未被限位钳住');
    console.log('  ④ 无干扰命令收敛到目标（先钉住命令通道的基线）');
    console.log('  ⑤ 正向并发：命令在途插入摇杆 → 命令被受理、无 error、');
    console.log('     且命令平息后补报含全部拨动（latest-wins，无丢失）');
    console.log('  ⑥ 反向抢占：设备在坡上时发命令 → 收敛到命令目标、之后无"回拉"帧');
    console.log('  ⑦ 收尾复位到 HOME');
    return 0;
  }

  mkdirSync(A.outDir, { recursive: true });
  log('start', `工作目录 ${A.outDir}`);

  const phases = [phase1, phase2, phase3, phase4, phase5, phase6, phase7];
  const picked = A.only
    ? phases.filter((_, i) => A.only.split(',').some((t) => Number(t.trim()) === i + 1))
    : phases;
  // 阶段 7 是收尾复位，被显式排除时也照样保留 —— 不能把机械臂留在歪着的状态。
  if (!picked.includes(phase7)) picked.push(phase7);

  phase('启动后端');
  const backend = await startBackend();
  log('backend', `device=${backend.device} external=${backend.external}`);

  const ready = await waitLinked();
  if (ready.needed) {
    check('链路就绪（已过开机静默窗口 + 暖机）', ready.linked,
      ready.linked ? '' : '等 12s 仍 linked=false；检查串口号 / 接线 / 供电');
    if (!ready.linked) {
      if (!backend.external) stopBackend();
      return 1;
    }
  }

  const link = await Link.open(A.ws);
  try {
    for (const p of picked) {
      const ok = await p(link);
      if (ok === false && p !== phase7) {
        warn(`阶段 ${p.name} 未通过，但继续跑后续阶段（后面的失败模式可能不同）`);
      }
    }

    // ---- 汇总 ----
    const npass = results.filter((r) => r.ok).length;
    const nfail = results.length - npass;
    console.log(`\n==== 设备跟随验收 PASS ${npass} / FAIL ${nfail}（共 ${results.length} 项）====`);
    console.log('# 读结果的口径：');
    console.log('#   ① 全绿只证明「上报被生成 → 被解析成 ReplyServo → 被标 origin=device →');
    console.log('#      被广播 → 且被命令让位」。真机固件无位置反馈，**不证明机械臂物理到位**。');
    console.log('#   ② 物理证据只有相机（见 core/tools/verify_serial_e2e.mjs）。');
    if (link.hello?.device === 'serial') {
      console.log('#   ③ 真机额外人工项：在串口终端敲 `STATS`，确认 tx_drop 保持 0 ——');
      console.log('#      它是"上报让位没把 TX 环写爆"的唯一独立证据（收发对账不算证据）。');
    }

    const summaryFile = path.join(A.outDir, 'summary.json');
    writeFileSync(
      summaryFile,
      JSON.stringify(
        { when: new Date().toISOString(), options: A, device: link.hello?.device ?? null, home, results },
        null,
        2,
      ),
      'utf8',
    );
    console.log(`# 结果 -> ${path.relative(ROOT, summaryFile)}`);
    const runLog = flushLog(A.outDir);
    if (runLog) console.log(`# 日志 -> ${path.relative(ROOT, runLog)}`);
    return nfail ? 1 : 0;
  } finally {
    link.close();
    if (!backend.external) stopBackend();
  }
}

main()
  .then((code) => process.exit(code))
  .catch((err) => {
    // 走到这里说明是**脚本自身崩溃**（验收 FAIL 走 return 1）。
    console.error('\n[fatal] 脚本执行中断（这是脚本崩溃，不是验收失败）');
    console.error(err?.stack ?? String(err));
    const runLog = flushLog(A.outDir);
    if (runLog) {
      console.error(`# 已把中断前的逐步日志写入 -> ${path.relative(ROOT, runLog)}`);
      console.error('# 看这份日志最后几行即可知道崩在哪一步（✗ 标记的那条）。');
    }
    stopBackend();
    process.exit(2);
  });
