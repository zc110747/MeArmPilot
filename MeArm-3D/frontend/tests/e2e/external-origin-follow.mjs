#!/usr/bin/env node
/**
 * e2e：外部入口（TCP JSON v1）驱动时，主臂与「实际臂幽灵」是否**统一**。
 *
 * 复现的用户现象：arm-web 经 9100 驱动 Sim 时，页面上只有半透明的幽灵臂在动，
 * 不透明的主臂不动。根因是 TCP 入口借用 origin=command，而前端只对
 * origin=device 做「命令侧跟随」。
 *
 * 本脚本的判据（**全部走 dev 探针 `window.__armPilot`，不靠肉眼**）：
 *   1. 外部驱动后 `commandJoints` 必须变化    → 证明命令侧跟随真的发生
 *   2. 外部驱动后主臂末端 `tcp` 必须移动      → 排除"两者都没动"的假重合
 *   3. 收敛后 `|tcp - actualTcp|` 必须≈0     → 主臂与幽灵**统一**
 *   4. 交叉验证渲染侧：__armPilotFrames 末帧里
 *      非幽灵根与幽灵根的 tcp 世界坐标必须重合
 *
 * 前置：后端在 8090（`MeArm-3D/backend/bin/armpilot-backend.exe -c config.yaml`）、
 *      vite dev 在 5273。**不得**带 VITE_AUTO_* 环境变量（否则页面自动连、
 *      读数会被开局回推干扰）。
 */
import { spawn } from 'node:child_process';
import { existsSync, mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import net from 'node:net';

const TARGET_URL = process.argv[2] ?? 'http://127.0.0.1:5273/';
const DEBUG_PORT = Number(process.argv[3] ?? 9351);
const WS_URL = process.argv[4] ?? 'ws://127.0.0.1:8090/ws/joint';
const TCP_HOST = '127.0.0.1';
const TCP_PORT = 9100;

const EDGE_CANDIDATES = [
  process.env.EDGE_PATH,
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',
  'C:\\Program Files (x86)\\Google\\Chrome\\Application\\chrome.exe',
].filter(Boolean);

const CHROME_FLAGS = [
  '--headless=new',
  '--no-sandbox',
  '--disable-gpu',
  '--disable-dev-shm-usage',
  '--no-first-run',
  '--no-default-browser-check',
  '--use-gl=angle',
  '--use-angle=swiftshader',
  '--enable-unsafe-swiftshader',
  '--hide-scrollbars',
  '--window-size=1400,950',
];

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function findBrowser() {
  for (const c of EDGE_CANDIDATES) if (c && existsSync(c)) return c;
  return null;
}

async function waitForDebuggerEndpoint(port, timeoutMs = 25000) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    try {
      const res = await fetch(`http://127.0.0.1:${port}/json/list`);
      const targets = await res.json();
      const page = targets.find((t) => t.type === 'page' && t.webSocketDebuggerUrl);
      if (page) return page;
    } catch {
      /* 端口未就绪 */
    }
    await sleep(250);
  }
  throw new Error(`调试端口 ${port} 未就绪`);
}

class Cdp {
  constructor(socket) {
    this.socket = socket;
    this.nextId = 1;
    this.pending = new Map();
    this.consoleLines = [];
    socket.addEventListener('message', (event) => {
      const message = JSON.parse(event.data);
      if (message.method === 'Runtime.consoleAPICalled') {
        const text = (message.params.args ?? [])
          .map((a) => (a.value !== undefined ? String(a.value) : a.description ?? ''))
          .join(' ');
        this.consoleLines.push(text);
      }
      if (message.id && this.pending.has(message.id)) {
        const { resolve, reject } = this.pending.get(message.id);
        this.pending.delete(message.id);
        if (message.error) reject(new Error(JSON.stringify(message.error)));
        else resolve(message.result);
      }
    });
  }

  send(method, params = {}) {
    const id = this.nextId++;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.socket.send(JSON.stringify({ id, method, params }));
      setTimeout(() => {
        if (this.pending.has(id)) {
          this.pending.delete(id);
          reject(new Error(`CDP 超时: ${method}`));
        }
      }, 30000);
    });
  }

  async evaluate(expression) {
    const result = await this.send('Runtime.evaluate', {
      expression,
      returnByValue: true,
      awaitPromise: true,
    });
    if (result.exceptionDetails) throw new Error(`页面求值异常: ${JSON.stringify(result.exceptionDetails)}`);
    return result.result?.value;
  }
}

// ---- 页面操作片段（与 ui-smoke.mjs 同口径）----
const cardByTitle = (title) =>
  `Array.from(document.querySelectorAll('.card')).find(c => c.textContent.includes('${title}'))`;

const READY_PROBE = `(() => {
  const overlay = document.querySelector('.overlay');
  const canvas = document.querySelector('.viewport canvas');
  return Boolean(overlay && canvas) && document.querySelectorAll('.overlay .chip').length >= 3;
})()`;

const CLICK_WS_MODE = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const radio = card ? card.querySelector('input[type=radio][value="websocket"]') : null;
  if (!radio) return false;
  radio.click();
  return true;
})()`;

const setWsUrl = (url) => `(() => {
  const el = document.querySelector('[data-testid="ws-url"]');
  if (!el) return false;
  const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
  setter.call(el, ${JSON.stringify(url)});
  el.dispatchEvent(new Event('input', { bubbles: true }));
  return true;
})()`;

const CLICK_CONNECT_WS = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const btn = card ? Array.from(card.querySelectorAll('button')).find(b => b.textContent.trim() === 'Connect WS') : null;
  if (!btn) return false;
  btn.click();
  return true;
})()`;

const READ_CONNECTION = `(() => {
  const card = ${cardByTitle('连接 · Transport')};
  const badge = card ? card.querySelector('.badge') : null;
  return badge ? badge.textContent.replace(/\\s+/g,' ').trim() : null;
})()`;

const READ_STATE = `(() => {
  const p = window.__armPilot;
  return p ? p.state() : null;
})()`;

/** 渲染侧：末帧里每棵名字以 robot: 开头的根的 ghost / visible / tcp 世界坐标 */
const READ_LAST_FRAME = `(() => {
  const list = window.__armPilotFrames;
  if (!list || !list.length) return null;
  const last = list[list.length - 1];
  return { f: last.f, roots: last.roots, visibleRoots: last.visibleRoots, arms: last.arms };
})()`;

/**
 * UI 日志区全文。
 *
 * ⚠️ 必须读这里，**不能**去抓 CDP 的 `Runtime.consoleAPICalled`：
 * `transportBridge` 的来源提示走的是 `store().pushLog('in', …)`，落到页面
 * 右下角的日志面板（`.log`），console 里一个字都不会出现。
 * 实测踩过：用 console 抓 → 两个关键词都是 false，看起来像"文案不对"，
 * 其实只是抓错了通道。
 */
const READ_LOG = `(() => {
  const el = document.querySelector('.log');
  return el ? el.textContent.replace(/\\s+/g, ' ').trim() : null;
})()`;

const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** 从 node 侧经外部入口下一条舵机级命令（arm-web 走的就是这条路） */
function sendExternalServo(servo, angle) {
  return new Promise((resolve) => {
    const lines = [];
    let sock;
    try {
      sock = net.connect(TCP_PORT, TCP_HOST, () => {
        sock.write(JSON.stringify({ cmd: 'servo', servo, angle }) + '\n');
      });
    } catch (e) {
      resolve({ ok: false, error: e.message, lines });
      return;
    }
    let buf = '';
    sock.on('data', (d) => {
      buf += d.toString();
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i).trim();
        buf = buf.slice(i + 1);
        if (line) lines.push(line);
      }
    });
    sock.on('error', (e) => {
      lines.push('ERR ' + e.message);
      resolve({ ok: false, lines });
    });
    setTimeout(() => {
      sock.end();
      resolve({ ok: true, lines });
    }, 1200);
  });
}

const results = [];
function check(name, ok, detail) {
  results.push({ name, ok });
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  | ' + detail : ''}`);
}

async function main() {
  const browser = findBrowser();
  if (!browser) throw new Error('未找到 Edge/Chrome，请设置 EDGE_PATH');
  console.log(`浏览器: ${browser}`);
  console.log(`前端  : ${TARGET_URL}`);
  console.log(`后端  : ${WS_URL}\n`);

  const userDataDir = mkdtempSync(path.join(tmpdir(), 'armpilot-ext-origin-'));
  const child = spawn(
    browser,
    [...CHROME_FLAGS, `--user-data-dir=${userDataDir}`, `--remote-debugging-port=${DEBUG_PORT}`, TARGET_URL],
    { stdio: 'ignore', detached: false },
  );

  let socket;
  try {
    const page = await waitForDebuggerEndpoint(DEBUG_PORT);
    socket = new WebSocket(page.webSocketDebuggerUrl);
    await new Promise((resolve, reject) => {
      socket.addEventListener('open', resolve, { once: true });
      socket.addEventListener('error', () => reject(new Error('CDP WS 连接失败')), { once: true });
    });
    const cdp = new Cdp(socket);
    await cdp.send('Runtime.enable');
    await cdp.send('Page.enable');

    // ---- 1. 等页面渲染完成 ----
    let ready = false;
    for (let i = 0; i < 60 && !ready; i += 1) {
      try {
        ready = await cdp.evaluate(READY_PROBE);
      } catch {
        ready = false;
      }
      if (!ready) await sleep(500);
    }
    if (!ready) throw new Error('页面未在 30s 内渲染');
    await sleep(2500);

    const hasProbe = await cdp.evaluate('Boolean(window.__armPilot && window.__armPilotFrames)');
    check('dev 探针可用（window.__armPilot / __armPilotFrames）', hasProbe === true);
    if (!hasProbe) throw new Error('缺 dev 探针 —— vite 是否为 DEV 模式？');

    // ---- 2. 连真实后端 ----
    await cdp.evaluate(CLICK_WS_MODE);
    await cdp.evaluate(setWsUrl(WS_URL));
    await sleep(200);
    await cdp.evaluate(CLICK_CONNECT_WS);

    let connected = false;
    for (let i = 0; i < 40 && !connected; i += 1) {
      const badge = await cdp.evaluate(READ_CONNECTION);
      if (badge && /connect/i.test(badge) && !/disconnect/i.test(badge)) connected = true;
      if (!connected) await sleep(300);
      else console.log(`      连接徽标: ${badge}`);
    }
    check('页面已连上真实后端 WS', connected);
    if (!connected) throw new Error('未能连上后端，后续判据无意义');
    await sleep(2000); // 等 hello 落地 + 首帧稳定

    // ---- 3. 基线 ----
    const logBefore = (await cdp.evaluate(READ_LOG)) ?? '';
    const before = await cdp.evaluate(READ_STATE);
    const frameBefore = await cdp.evaluate(READ_LAST_FRAME);
    if (!before) throw new Error('读不到 __armPilot.state()');
    const gapBefore = dist(before.tcp, before.actualTcp);
    console.log(`\n[基线] commandJoints = ${JSON.stringify(before.commandJoints)}`);
    console.log(`[基线] tcp=${fmtV(before.tcp)}  actualTcp=${fmtV(before.actualTcp)}  |差|=${gapBefore.toFixed(3)} mm`);
    if (frameBefore) {
      console.log(`[基线] 末帧 f=${frameBefore.f} roots=${frameBefore.roots} visible=${frameBefore.visibleRoots}`);
      for (const a of frameBefore.arms) console.log(`         ghost=${a.ghost} visible=${a.visible} tcp=${fmtV(a.tcp)}`);
    }

    // ---- 4. 经外部入口驱动 ----
    // 反向驱动：base 与 servo#1 的关系是 servo = 90 + base（HOME base=0 ↔ servo 90）。
    // 若沿用固定角度，上一轮跑完残留的位姿会让本轮"驱动后位姿不变"⇒ 假 FAIL。
    const baseNow = before.commandJoints?.base ?? 0;
    const angle = baseNow > 0 ? 60 : 120; // 60→base −30°；120→base +30°
    const ext = await sendExternalServo(1, angle);
    console.log(`\n[外部入口] TCP servo 1 ${angle}（基线 base=${baseNow}）→ ${JSON.stringify(ext.lines.slice(0, 1))}`);
    check('外部入口命令被后端受理（ok:true）', ext.lines.some((l) => l.includes('"ok":true')), ext.lines[0]);

    await sleep(3000); // 等 sim 走完 + 前端收敛

    // ---- 5. 驱动后 ----
    const after = await cdp.evaluate(READ_STATE);
    const frameAfter = await cdp.evaluate(READ_LAST_FRAME);
    const gapAfter = dist(after.tcp, after.actualTcp);
    console.log(`\n[驱动后] commandJoints = ${JSON.stringify(after.commandJoints)}`);
    console.log(`[驱动后] tcp=${fmtV(after.tcp)}  actualTcp=${fmtV(after.actualTcp)}  |差|=${gapAfter.toFixed(3)} mm`);
    const moved = dist(before.tcp, after.tcp);
    console.log(`[驱动后] 主臂末端位移 = ${moved.toFixed(3)} mm`);

    // ---- 6. 判据 ----
    const jointsChanged =
      JSON.stringify(before.commandJoints) !== JSON.stringify(after.commandJoints);
    check(
      '① 命令侧跟随：外部驱动后 commandJoints 变化',
      jointsChanged,
      `before.base=${before.commandJoints?.base} after.base=${after.commandJoints?.base}`,
    );

    check('② 主臂真的动了（排除"两者都没动"的假重合）', moved > 5, `位移 ${moved.toFixed(2)} mm`);

    check('③ 主臂与幽灵统一：|tcp − actualTcp| ≈ 0', gapAfter < 1.0, `${gapAfter.toFixed(3)} mm（基线 ${gapBefore.toFixed(3)} mm）`);

    if (frameAfter) {
      console.log(`\n[驱动后] 末帧 f=${frameAfter.f} roots=${frameAfter.roots} visible=${frameAfter.visibleRoots}`);
      for (const a of frameAfter.arms) console.log(`         ghost=${a.ghost} visible=${a.visible} tcp=${fmtV(a.tcp)}`);
      const main = frameAfter.arms.find((a) => !a.ghost && a.tcp);
      const ghost = frameAfter.arms.find((a) => a.ghost && a.tcp);
      if (main && ghost) {
        const rgap = dist(main.tcp, ghost.tcp);
        check('④ 渲染侧交叉验证：主臂与幽灵 TCP 世界坐标重合', rgap < 1.0, `${rgap.toFixed(3)} mm`);
      } else {
        console.log('      （渲染侧缺 tcp 标记，跳过 ④）');
      }
    }

    // ---- 7. 日志文案 + 回发抑制（读 UI 日志区，不是 console）----
    const logAfter = (await cdp.evaluate(READ_LOG)) ?? '';
    const delta = logAfter.startsWith(logBefore) ? logAfter.slice(logBefore.length) : logAfter;
    console.log(`\n[日志增量] ${delta.slice(0, 400) || '(空)'}`);

    const saysExternal = /外部入口/.test(delta);
    const saysDevice = /设备侧自主变化/.test(delta);
    check('⑤ 日志区分来源：提"外部入口"且不误报"设备侧自主变化"', saysExternal && !saysDevice);

    /**
     * 回发抑制：外部驱动后本页**不得**把对齐后的 commandJoints 当作用户意图发回去。
     * 判据用日志区里 `joint_command` 的出现次数增量 —— 这是"命令→状态→命令"回环
     * 的防线，破了会在真机上表现为自激。
     */
    const countCmd = (s) => (s.match(/joint_command/g) ?? []).length;
    const outDelta = countCmd(logAfter) - countCmd(logBefore);
    check('⑥ 抑制回发：外部驱动未触发本页下发 joint_command', outDelta === 0, `增量 ${outDelta}`);

    const failed = results.filter((r) => !r.ok).length;
    console.log(`\n===== ${results.length - failed} PASS / ${failed} FAIL =====`);
    process.exitCode = failed === 0 ? 0 : 1;
  } finally {
    try {
      socket?.close();
    } catch {
      /* ignore */
    }
    try {
      child.kill();
    } catch {
      /* ignore */
    }
    await sleep(600);
    try {
      rmSync(userDataDir, { recursive: true, force: true });
    } catch {
      /* Windows 占用忽略 */
    }
  }
}

const fmtV = (v) => (v ? `[${v.map((n) => n.toFixed(1)).join(', ')}]` : 'null');

main().catch((error) => {
  console.error(`[external-origin-follow] 失败: ${error.message}`);
  process.exit(1);
});
