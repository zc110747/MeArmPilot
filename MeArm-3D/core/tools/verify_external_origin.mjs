#!/usr/bin/env node
/**
 * verify_external_origin.mjs —— 「外部入口驱动 → 来源标注正确」端到端验收（ADR D84）
 *
 * 这条链路解决的是什么
 * --------------------
 * `MeArm-RemoteControl`（`arm-web`）的 network 模式是**另一台上位机**：它经
 * TCP JSON v1（9100）给本后端下命令。对 MeArm-3D 自己的网页来说，这既不是
 * "我自己下的命令"，也不是"设备被手动拨动了" —— 它是**第三方**。
 *
 * 前端据此决定要不要「命令侧跟随」（写 commandJoints + 抑制回发），判据是状态帧里的
 * `origin` 字段。三条来源必须严格分离：
 *
 *     origin=command    本页自己的命令（JR 应答 / 回读）  → 不跟随
 *     origin=device     设备自主变化（摇杆 / 红外 / 手拧）→ 跟随 + 不回发
 *     origin=external   另一台上位机经 TCP 网关下命令     → 跟随 + 不回发（同 device）
 *
 * 为什么必须分段统计，而不是"看到 external 就算过"
 * -------------------------------------------------
 * 只验证"后端标了 external"是不够的，还要证明**来源随命令同行、不被污染**：
 *   · 若 `curOrigin` 没被后续本页命令刷新，本页操作会被当成外部驱动而**抑制回发**
 *     （用户拖滑杆，机器不动）—— 这正是第 C 段要抓的。
 *   · 若外部命令借用 `command`，对端页面只有半透明的"实际臂幽灵"在动、主臂停在旧值；
 *     更危险的是对端 commandJoints 停在旧值，用户下一次碰任何控件会把**整组旧指令**
 *     一次性下发（真机跳回旧位姿）。这正是第 B 段要抓的。
 *
 * 实测（2026-09-17，Sim 链路）:  A=command(2 帧) · B=external(6 帧) · C=command(8 帧)
 *
 * 配套
 * ----
 *   · 后端单测 internal/tcpserver/protocol_test.go::TestWriteCommands_CarryExternalOrigin
 *   · 后端单测 internal/controller/controller_test.go（external 状态帧 + 在途来源不丢）
 *   · 前端 e2e frontend/tests/e2e/external-origin-follow.mjs（浏览器侧：主臂与幽灵重合）
 *
 * 前置：后端在 8090（`backend/bin/armpilot-backend.exe -c config.yaml`）。
 * 零依赖（Node >= 22 自带 WebSocket），不需要浏览器、不需要前端 dev server。
 */
import net from 'node:net';

const WS_URL = process.env.WS_URL ?? 'ws://127.0.0.1:8090/ws/joint';
const TCP_HOST = process.env.TCP_HOST ?? '127.0.0.1';
const TCP_PORT = Number(process.env.TCP_PORT ?? 9100);

const phases = [];
let cur = null;
let model = null;
let homePose = null;

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const phase = (name) => {
  cur = { name, origins: [] };
  phases.push(cur);
  console.log(`\n--- 阶段 ${name} ---`);
};

const ws = new WebSocket(WS_URL);

ws.onopen = () => console.log('[WS] open', WS_URL);
ws.onerror = (e) => console.log('[WS] error', e.message || e);

ws.onmessage = (ev) => {
  let m;
  try {
    m = JSON.parse(ev.data);
  } catch {
    console.log('[WS] non-json:', String(ev.data).slice(0, 120));
    return;
  }
  if (m.type === 'hello') {
    model = m.model ?? null;
    homePose = model?.homePose ?? null;
    console.log('[hello] jointOrder =', JSON.stringify(model?.jointOrder));
    console.log('[hello] homePose   =', JSON.stringify(homePose));
    return;
  }
  if (m.type === 'joint_state') {
    const o = m.origin === undefined ? '(缺失)' : m.origin;
    if (cur) cur.origins.push(o);
    console.log(`  [${cur ? cur.name : '-'}] origin=${o}`);
    return;
  }
  if (m.type === 'error') {
    console.log('  [error]', m.code, m.message);
    return;
  }
  console.log('[WS]', m.type);
};

function sendJointCommand(joints) {
  if (!ws || ws.readyState !== 1) {
    console.log('  [send] WS 未就绪，跳过');
    return;
  }
  ws.send(JSON.stringify({ version: 1, type: 'joint_command', timestamp: Date.now(), joints }));
  console.log('  [send] joint_command', JSON.stringify(joints));
}

function sendServo(servo, angle) {
  return new Promise((resolve) => {
    const sock = net.connect(TCP_PORT, TCP_HOST, () => {
      console.log(`  [send] TCP servo id=${servo} angle=${angle}`);
      sock.write(JSON.stringify({ cmd: 'servo', servo, angle }) + '\n');
    });
    let buf = '';
    sock.on('data', (d) => {
      buf += d.toString();
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const line = buf.slice(0, i);
        buf = buf.slice(i + 1);
        if (line.trim()) console.log('  [tcp]', line.trim().slice(0, 160));
      }
    });
    sock.on('error', (e) => console.log('  [tcp] error', e.message));
    setTimeout(() => {
      sock.end();
      resolve();
    }, 1200);
  });
}

const bump = (pose, jointId, delta) => {
  const out = { ...pose };
  out[jointId] = Math.round((pose[jointId] + delta) * 10) / 10;
  return out;
};

async function main() {
  // 等 hello
  for (let i = 0; i < 40 && !homePose; i++) await sleep(50);
  if (!homePose) {
    console.log('!! 未收到 hello / homePose，终止');
    process.exit(1);
  }
  const j0 = model?.jointOrder?.[0] ?? 'base';

  await sleep(200);

  // --- A：本页命令 ---
  phase('A 本页 WS joint_command');
  sendJointCommand(bump(homePose, j0, 5));
  await sleep(900);

  // --- B：外部 TCP ---
  phase('B 外部 TCP servo');
  await sendServo(1, 120);
  await sleep(900);

  // --- C：本页命令（验证来源被刷回）---
  phase('C 本页 WS joint_command（再来一次）');
  sendJointCommand(bump(homePose, j0, -5));
  await sleep(900);

  // --- 汇总 ---
  console.log('\n===== 分段 origin 统计 =====');
  let bad = 0;
  const expect = {
    'A 本页 WS joint_command': 'command',
    'B 外部 TCP servo': 'external',
    'C 本页 WS joint_command（再来一次）': 'command',
  };
  for (const p of phases) {
    const tally = {};
    for (const o of p.origins) tally[o] = (tally[o] || 0) + 1;
    const want = expect[p.name];
    const got = Object.keys(tally);
    const ok = p.origins.length > 0 && got.length === 1 && got[0] === want;
    if (!ok) bad += 1;
    console.log(
      `${ok ? 'PASS' : 'FAIL'} ${p.name}  期望=${want} 实得=${JSON.stringify(tally)} 帧数=${p.origins.length}`,
    );
  }
  console.log(`\n结论: ${phases.length - bad} PASS / ${bad} FAIL`);
  ws.close();
  process.exit(bad === 0 ? 0 : 1);
}

main();
