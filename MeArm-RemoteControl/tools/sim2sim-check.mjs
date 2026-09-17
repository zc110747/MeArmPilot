// sim2sim-check.mjs — Sim2Sim 端到端验收（network 模式：网页 → arm-web → TCP → MeArm-3D Sim）。
//
// 零依赖：WebSocket 客户端是手写的（与 tools/e2e-sim.js 同款），
// 所以只要 node 就能跑，不需要 npm install。
//
// 用法（两个服务都要先起来）：
//   1) MeArm-3D：      MeArm-3D\start.bat              （SIM 模式；其 config.yaml 里 tcp.enabled: true）
//   2) RemoteControl： MeArm-RemoteControl\start.bat    （默认就是 network 模式）
//   3) 本脚本：       node tools/sim2sim-check.mjs [wsUrl]
//      默认 wsUrl = ws://127.0.0.1:9001/ws（端口取 MeArm-RemoteControl/config.yaml 的 web.port）
//
// 期望值**全部从配置派生**（工程铁律：测试期望值禁止硬编码）：
//   - 舵机编号/接线       → MeArm-RemoteControl/config.yaml 的 joystick.* 与 network.servo_ids
//   - 爪开合对应的舵机角   → MeArm-3D/config/robots.yaml → robot.yaml 的 actuators/limits
// 本文件里出现的角度数字只有两个：`90`（= HOME，同时也是四舵机行程的公共内点）
// 与 `179.5`（越界样本）—— 两者都不依赖任何标定值。
//
// ⚠️ 等待同步一律按**值**判定，不按"又收到几条消息"。摇杆停下来之后链路
//    是不发帧的（latest-wins + 只发脏轴），用"等新消息"会永远等不到。
//
// 退出码：0 = 全部 PASS；1 = 有 FAIL 或致命错误。

import http from "node:http";
import crypto from "node:crypto";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const RC_DIR = path.resolve(HERE, ".."); // MeArm-RemoteControl
const REPO = path.resolve(RC_DIR, ".."); // MeArmPilot（本 monorepo 根）
// ⚠️ robots.yaml 里 `config:` 的路径是以 **MeArm-3D 自己的仓库根**为基准解析的
//    （它注释里的「仓库相对路径」指的是那一层，三端都按那个根解析），
//    在本 monorepo 里就是 MeArm-3D/ —— 不是 MeArmPilot/。
const M3D = path.join(REPO, "MeArm-3D");

const WS_URL = process.argv[2] || "ws://127.0.0.1:9001/ws";

/** HOME：四舵机 90°。既是 mearm-v1 的开机位，也是四个舵机行程的公共内点。 */
const HOME_ANGLE = 90;

// ===========================================================================
// 从配置派生期望值（不做任何硬编码）
// ===========================================================================

const read = (p) => fs.readFileSync(p, "utf8");

// 读 MeArm-RemoteControl/config.yaml 里各平铺段。
// 只认 `段名:` 与紧随其后的 `  键: 值`，够用且不会被注释干扰。
function loadRemoteConfig() {
  const text = read(path.join(RC_DIR, "config.yaml"));
  const out = { joystick: {}, network: {}, web: {} };
  let section = "";
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.replace(/#.*$/, "");
    if (!line.trim()) continue;
    const sec = /^([a-z_]+):\s*$/.exec(line);
    if (sec) { section = sec[1]; continue; }
    const kv = /^\s+([a-z_0-9]+):\s*(.+?)\s*$/.exec(line);
    if (!kv || !out[section]) continue;
    out[section][kv[1]] = kv[2].replace(/^["']|["']$/g, "");
  }
  const raw = out.network.servo_ids;
  if (!raw) throw new Error("config.yaml 里没有 network.servo_ids");
  const servoIDs = raw.replace(/[[\]]/g, "").split(",").map((s) => parseInt(s.trim(), 10));
  if (servoIDs.some(Number.isNaN)) throw new Error(`network.servo_ids 解析失败: ${raw}`);
  const num = (v, d) => (v === undefined ? d : parseFloat(v));
  return {
    servoIDs,
    joystick: {
      lx: parseInt(out.joystick.lx_servo, 10),
      ly: parseInt(out.joystick.ly_servo, 10),
      rx: parseInt(out.joystick.rx_servo, 10),
      ry: parseInt(out.joystick.ry_servo, 10),
    },
    minSpeed: num(out.network.min_speed_deg_per_s, 12),
    maxSpeed: num(out.network.max_speed_deg_per_s, 90),
    webPort: parseInt(out.web.port, 10),
  };
}

// 读 MeArm-3D 的模型真值：robots.yaml 选路径 → robot.yaml 取关节/执行器。
// 只解析本验收需要的最小字段（关节的 role/limit，执行器的 jointId/offset/scale/reverse）。
function loadRobotModel() {
  const sel = read(path.join(M3D, "config", "robots.yaml"));
  const rel = /^\s*config:\s*(\S+)\s*$/m.exec(sel);
  if (!rel) throw new Error("robots.yaml 里找不到 config: 指向的 robot.yaml");
  const yml = read(path.join(M3D, rel[1]));

  const joints = [];
  const acts = [];
  let top = "";
  let joint = null;
  let act = null;
  for (const raw of yml.split(/\r?\n/)) {
    const line = raw.replace(/#.*$/, "").replace(/\s+$/, "");
    if (!line.trim()) continue;
    const t = /^([a-z_]+):/.exec(line);
    if (t) { top = t[1]; joint = null; act = null; continue; }

    const idm = /^ {2}- id:\s*(\S+)/.exec(line);
    if (idm) {
      if (top === "joints") { joint = { id: idm[1] }; joints.push(joint); act = null; }
      else if (top === "actuators") { act = { id: idm[1] }; acts.push(act); joint = null; }
      else { joint = null; act = null; }
      continue;
    }
    const kv = /^ {4}([a-zA-Z]+):\s*(.*)$/.exec(line);
    if (kv) {
      const k = kv[1], v = kv[2].trim();
      if (joint) {
        if (k === "role") joint.role = v;
        if (k === "limit") joint._lim = {};
      }
      if (act) {
        if (k === "jointId") act.jointId = v;
        if (k === "channel") act.channel = parseInt(v, 10);
        if (k === "offset") act.offset = parseFloat(v);
        if (k === "scale") act.scale = parseFloat(v);
        if (k === "reverse") act.reverse = v === "true";
        if (k === "limits") act._lim = {};
      }
      continue;
    }
    const sub = /^ {6}(min|max):\s*([-\d.eE]+)/.exec(line);
    if (sub) {
      const v = parseFloat(sub[2]);
      if (joint && joint._lim) joint._lim[sub[1]] = v;
      if (act && act._lim) act._lim[sub[1]] = v;
    }
  }
  return { joints, acts, source: rel[1] };
}

// JointToServo 的镜像实现（真值在 backend/internal/robot/robot.go）。
// 这里只用来算"期望值"；服务端算的是同一套，不一致就会被断言抓出来。
function jointToServo(a, theta) {
  return a.reverse ? -theta * a.scale + a.offset : theta * a.scale + a.offset;
}

// ===========================================================================
// 最小 WebSocket 客户端（零依赖）
// ===========================================================================
class WSClient {
  constructor(url) { this.url = url; this.buf = Buffer.alloc(0); this.onMsg = null; }
  connect() {
    return new Promise((resolve, reject) => {
      const u = new URL(this.url);
      const key = crypto.randomBytes(16).toString("base64");
      const req = http.request({
        hostname: u.hostname, port: u.port || 80, path: u.pathname,
        headers: {
          Connection: "Upgrade", Upgrade: "websocket",
          "Sec-WebSocket-Key": key, "Sec-WebSocket-Version": "13",
        },
      });
      req.on("upgrade", (res, socket, head) => {
        this.sock = socket;
        socket.on("data", (d) => this._onData(d));
        // ⚠️ `head` 是紧跟在握手响应之后、被 HTTP 解析器一并读走的那一段数据。
        //    服务端在握手后立刻连发 caps + link_status + state，它们常常与握手
        //    同在一个 TCP 段里 —— 不处理 head 就会静默丢掉最前面那几条
        //    （实测：caps 永远收不到）。必须在返回事件循环之前同步喂进去。
        if (head && head.length) this._onData(head);
        resolve(this);
      });
      req.on("error", reject);
      req.end();
    });
  }
  _onData(d) {
    this.buf = Buffer.concat([this.buf, d]);
    for (;;) {
      if (this.buf.length < 2) return;
      const b1 = this.buf[1];
      const opcode = this.buf[0] & 0x0f;
      let len = b1 & 0x7f, offset = 2;
      if (len === 126) { if (this.buf.length < 4) return; len = this.buf.readUInt16BE(2); offset = 4; }
      else if (len === 127) {
        if (this.buf.length < 10) return;
        len = this.buf.readUInt32BE(2) * 4294967296 + this.buf.readUInt32BE(6); offset = 10;
      }
      const masked = (b1 & 0x80) !== 0;
      if (masked) offset += 4;
      if (this.buf.length < offset + len) return;
      let payload = this.buf.subarray(offset, offset + len);
      if (masked) {
        const m = this.buf.subarray(offset - 4, offset);
        const o = Buffer.alloc(len);
        for (let i = 0; i < len; i++) o[i] = payload[i] ^ m[i & 3];
        payload = o;
      }
      this.buf = this.buf.subarray(offset + len);
      if (opcode === 0x8) { try { this.sock.end(); } catch { /* ignore */ } return; }
      if ((opcode === 0x1 || opcode === 0x2) && this.onMsg) this.onMsg(payload.toString("utf8"));
    }
  }
  send(obj) {
    const data = Buffer.from(JSON.stringify(obj), "utf8");
    const len = data.length;
    const mask = crypto.randomBytes(4);
    let header;
    if (len < 126) { header = Buffer.alloc(2); header[0] = 0x81; header[1] = 0x80 | len; }
    else if (len < 65536) {
      header = Buffer.alloc(4); header[0] = 0x81; header[1] = 0x80 | 126;
      header.writeUInt16BE(len, 2);
    } else {
      header = Buffer.alloc(10); header[0] = 0x81; header[1] = 0x80 | 127;
      header.writeUInt32BE(0, 2); header.writeUInt32BE(len, 6);
    }
    const masked = Buffer.alloc(len);
    for (let i = 0; i < len; i++) masked[i] = data[i] ^ mask[i & 3];
    this.sock.write(Buffer.concat([header, mask, masked]));
  }
}

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ===========================================================================
// 断言与报告
// ===========================================================================
const results = [];
function check(name, ok, detail) {
  results.push({ name, ok });
  console.log(`  ${ok ? "PASS" : "FAIL"}  ${name}${detail ? "  -- " + detail : ""}`);
}
function info(msg) { console.log(`        ${msg}`); }

// ===========================================================================
// 主流程
// ===========================================================================
const cfg = loadRemoteConfig();
const model = loadRobotModel();

// servo_ids 的**位次** = MeArm-3D 的 TCP 编号；值是 arm-device 的舵机 id。
// TCP 编号 → arm-device id → 网页 angles 字段名（s6..s9）。
const angleField = (armID) => `s${armID}`;

// 爪的开/合对应舵机角：端点取自 robot.yaml 的关节限位 + 执行器标定。
const gripJoint = model.joints.find((j) => j.role === "gripper");
const gripAct = model.acts.find((a) => a.jointId === gripJoint?.id);
if (!gripJoint || !gripAct) throw new Error(`robot.yaml 里找不到 gripper 关节/执行器（来源 ${model.source}）`);
const gripOpenServo = jointToServo(gripAct, gripJoint._lim.max); // 张开 = 关节限位 max
const gripCloseServo = jointToServo(gripAct, gripJoint._lim.min); // 闭合 = 关节限位 min

console.log("===== Sim2Sim 端到端验收（MeArm-RemoteControl --network--> MeArm-3D Sim）=====");
console.log(`[cfg] WS=${WS_URL}`);
console.log(`[cfg] 接线表 servo_ids=[${cfg.servoIDs}]（位次 = TCP 编号）`);
console.log(`[cfg] 摇杆 -> arm-device id: LX=${cfg.joystick.lx} LY=${cfg.joystick.ly} RX=${cfg.joystick.rx} RY=${cfg.joystick.ry}`);
console.log(`[cfg] 模型真值 ${model.source}：爪 张开=${gripOpenServo}° 闭合=${gripCloseServo}°`);
console.log("");

const ws = new WSClient(WS_URL);
const seen = []; // 收到的全部消息（已解析）
let rawCount = 0, jsonErrors = 0;

ws.onMsg = (text) => {
  rawCount++;
  let m;
  try { m = JSON.parse(text); } catch { jsonErrors++; return; }
  seen.push(m);
  if (m.t === "state") {
    console.log(`  [WS<-] state  angles=${JSON.stringify(m.angles)} tcp=${m.tcp?.map((v) => v.toFixed(1))} device=${m.device}`);
  } else if (m.t === "err") {
    console.log(`  [WS<-] err    ${m.msg}`);
  }
};

await ws.connect();
check("WebSocket 握手成功", true);

const states = () => seen.filter((m) => m.t === "state");
const latest = () => states()[states().length - 1] || null;
const countErrors = () => seen.filter((m) => m.t === "err").length;
// 摇杆轴：arm-device id -> angles 字段
const angOf = (state, armID) => state?.angles?.[angleField(armID)];

/** 等到"最新状态"满足条件（按**值**判定，见文件头）。返回该状态或 null。 */
async function waitState(pred, ms) {
  const t0 = Date.now();
  for (;;) {
    const s = latest();
    if (s && pred(s)) return s;
    if (Date.now() - t0 >= ms) return null;
    await sleep(30);
  }
}
/** 等到 caps / 某个类型的消息出现。 */
async function waitMsg(pred, ms) {
  const t0 = Date.now();
  for (;;) {
    for (let i = seen.length - 1; i >= 0; i--) if (pred(seen[i])) return seen[i];
    if (Date.now() - t0 >= ms) return null;
    await sleep(30);
  }
}
/** 连续发 n 帧摇杆（side= L/R）。 */
async function pushJoy(side, x, y, n, gapMs) {
  for (let i = 0; i < n; i++) { ws.send({ t: "joy", side, x, y }); await sleep(gapMs); }
}
/** 让四轴回到 HOME 并等回读一致。 */
async function goHome(ms = 3000) {
  for (let n = 1; n <= cfg.servoIDs.length; n++) { ws.send({ t: "servo", servo: n, angle: HOME_ANGLE }); await sleep(80); }
  return waitState((s) => cfg.servoIDs.every((id) => angOf(s, id) === HOME_ANGLE), ms);
}

// ---- 1. 能力声明：必须是 network 模式 -------------------------------------
const caps = await waitMsg((m) => m.t === "caps", 2000);
if (!caps) throw new Error("没有收到 caps 消息（网页与后端版本不匹配？）");
info(`caps = ${JSON.stringify(caps.caps)}`);
check("caps.mode == network（本页跑在 TCP 链路上）", caps.caps.mode === "network", `mode=${caps.caps.mode}`);
check("caps 声明支持 xyz / gripper / servo_direct / refresh_state",
  caps.caps.xyz && caps.caps.gripper && caps.caps.servo_direct && caps.caps.refresh_state);
check("caps.arm_device_cmds == false（网络模式无固件指令，按钮应置灰）",
  caps.caps.arm_device_cmds === false);
check("caps.servo_ids 与 config.yaml 的 network.servo_ids 一致",
  JSON.stringify(caps.caps.servo_ids) === JSON.stringify(cfg.servoIDs),
  `caps=[${caps.caps.servo_ids}] cfg=[${cfg.servoIDs}]`);

// ---- 2. 链路已连上 MeArm-3D ---------------------------------------------
const linkMsg = await waitMsg((m) => m.t === "link_status" && m.connected === true, 6000);
check("link_status: 已连上 MeArm-3D TCP", !!linkMsg, linkMsg ? "" : "6s 内未连上（MeArm-3D 起来了吗？）");

// ---- 3. 新页面立刻拿到状态（接入时主动拉一次）----------------------------
const first = await waitState((s) => !!s.angles, 3000);
check("收到首帧 state（基准同步 + 新页面自动拉取）", !!first);
check("state.device == sim（驱动的确实是仿真，不是真机）", first?.device === "sim", `device=${first?.device}`);
check("state 含 tcp 末端坐标（[x,y,z] mm）", Array.isArray(first?.tcp) && first.tcp.length === 3);

// ---- 4. 先回 HOME，消除后续测试对"当前位姿"的依赖 -------------------------
const sHome0 = await goHome();
check("复位到 HOME（四轴 90°）", !!sHome0, JSON.stringify(sHome0?.angles));

// ---- 5. 摇杆：左摇杆 X 正偏 → 底座(arm-device id 9) 角度增大 --------------
// 方向依据 config.yaml 的约定「推杆方向 = 舵机角度增大（+）」；
// 网络链路的换算见 internal/protocol.NetInvertFor（含固件方向差异的推导）。
const baseID = cfg.joystick.lx;
const elbowID = cfg.joystick.ly;
const baseBefore = angOf(latest(), baseID);
const elbowBefore = angOf(latest(), elbowID);
await pushJoy("L", 0.9, 0, 10, 50); // 约 0.5s
const sUp = await waitState((s) => angOf(s, baseID) >= baseBefore + 5, 2000);
const baseUp = angOf(sUp, baseID);
check(`左摇杆 X=+0.9 → 底座(S${baseID}) 角度增大`, baseUp > baseBefore,
  `${baseBefore} -> ${baseUp ?? "无变化"}`);
check(`未被推动的轴不动（左摇杆 Y -> S${elbowID}）`, angOf(latest(), elbowID) === elbowBefore,
  `${elbowBefore} -> ${angOf(latest(), elbowID)}`);
info(`0.5s 内变化 ${(baseUp ?? baseBefore) - baseBefore}°（配置区间 ${cfg.minSpeed}..${cfg.maxSpeed}°/s）`);
await pushJoy("L", 0, 0, 4, 60);

// ---- 6. 摇杆回中 → 停住（latest-wins + 只发脏轴）-------------------------
const sBeforeStop = latest();
const baseAtStop = angOf(sBeforeStop, baseID);
await sleep(400);
check("摇杆回中后角度停止变化", angOf(latest(), baseID) === baseAtStop,
  `${baseAtStop} -> ${angOf(latest(), baseID)}`);

// ---- 7. XYZ 相对位移（x/y/z 各正负一次）----------------------------------
// ⚠️ 本段**必须**先回 HOME，别依赖第 5 段摇杆留下的位姿：
//    摇杆的位移 = 按**真实 dt** 积分（前端频率不影响速度），所以"0.5s 推杆"
//    到底把底座(M9)转了多远是**跑一次一个值**；而本机的 y± 恰好要靠底座旋转实现，
//    底座一旦停在行程端点，y 方向就会被 `ERR JOINT base ... (limit ...)` 拒掉，
//    表现为"XYZ 某一条偶发 FAIL"（2026-09-17 实测，两次同脚本一次绿一次红）。
//    HOME 位姿下六向都可达，所以这里把前置条件**显式做出来**，而不是写在注释里。
const sHome7 = await goHome();
check("XYZ 段前置：回到 HOME（消除对摇杆段结束位姿的依赖）", !!sHome7,
  JSON.stringify(sHome7?.angles));
info(`XYZ 起始位姿 tcp=${latest().tcp.map((v) => v.toFixed(1)).join(",")} mm`);

const xyz = [
  { axis: "x", dir: "+", idx: 0, sign: 1 },
  { axis: "x", dir: "-", idx: 0, sign: -1 },
  { axis: "y", dir: "+", idx: 1, sign: 1 },
  { axis: "y", dir: "-", idx: 1, sign: -1 },
  { axis: "z", dir: "+", idx: 2, sign: 1 },
  { axis: "z", dir: "-", idx: 2, sign: -1 },
];
for (const c of xyz) {
  const from = latest().tcp;
  ws.send({ t: "xyz", axis: c.axis, direction: c.dir, step: 5 });
  const to = await waitState((s) => Math.sign(s.tcp[c.idx] - from[c.idx]) === c.sign && Math.abs(s.tcp[c.idx] - from[c.idx]) > 1, 2500);
  const d = to ? to.tcp[c.idx] - from[c.idx] : NaN;
  check(`XYZ ${c.axis}${c.dir} step=5 → state.tcp[${c.idx}] 变化 ${c.sign > 0 ? "增大" : "减小"}`,
    !!to, `${from[c.idx].toFixed(2)} -> ${to ? to.tcp[c.idx].toFixed(2) : "无变化"} (Δ${Number.isNaN(d) ? "-" : d.toFixed(2)}mm)`);
}

// ---- 8. 爪：open / close 落在 robot.yaml 派生的两端 -----------------------
// 夹取舵机的 arm-device id 固定为 6（MeArm-Device/core/joystick.c 的 J_ID 表
// 与 internal/protocol/validIDs 一致）；这里只检查它确实在接线表里。
if (!cfg.servoIDs.includes(6)) throw new Error("servo_ids 里没有夹取舵机(6)，无法测爪");
const gripID = 6;
ws.send({ t: "grip", action: "close" });
const sClose = await waitState((s) => Math.abs(angOf(s, gripID) - gripCloseServo) <= 1, 2500);
const closeGot = angOf(latest(), gripID);
ws.send({ t: "grip", action: "open" });
const sOpen2 = await waitState((s) => Math.abs(angOf(s, gripID) - gripOpenServo) <= 1, 2500);
const openGot = angOf(latest(), gripID);
check("爪 close → 舵机角落到 robot.yaml 的闭合端点", !!sClose,
  `期望 ${gripCloseServo}° 实得 ${closeGot}°`);
check("爪 open → 舵机角落到 robot.yaml 的张开端点", !!sOpen2,
  `期望 ${gripOpenServo}° 实得 ${openGot}°`);
check("爪 open/close 落在不同端点（方向与标定一致）", openGot !== closeGot,
  `open=${openGot}° close=${closeGot}°`);

// ---- 9. 四舵机直接控制 + 状态回读一致 -------------------------------------
await goHome();
let servoOK = true; const servoBad = [];
for (let n = 1; n <= cfg.servoIDs.length; n++) {
  const id = cfg.servoIDs[n - 1];
  ws.send({ t: "servo", servo: n, angle: 91 }); // 91 ≠ HOME，确认真的听话
  const sn = await waitState((s) => angOf(s, id) === 91, 2000);
  if (!sn) { servoOK = false; servoBad.push(`#${n}(S${id})=${angOf(latest(), id)}`); }
}
check(`四舵机直接控制 servo 1..${cfg.servoIDs.length}=91 → 状态回读逐一一致`, servoOK,
  servoBad.join(" ") || "四条均回读 91°");
await goHome();

// ---- 10. 越界被拒 + 本地目标回滚（不漂移）--------------------------------
// 179.5° 超出 mearm-v1 任一舵机的行程（robot.yaml actuators[].limits），
// 服务端以 out of range 拒绝；arm-web 必须① 上报错误 ② 把该轴目标回滚到
// 服务端确认值 —— 否则之后的摇杆增量会从 179.5 继续累加，每一帧都被拒，
// 表现为「推杆完全不动」，且不会有任何一处报错（这是最难查的那种故障）。
const errBefore = countErrors();
const preBase = angOf(latest(), baseID);
ws.send({ t: "servo", servo: 1, angle: 179.5 });
let errSeen = null;
for (let i = 0; i < 40 && !errSeen; i++) {
  await sleep(50);
  if (countErrors() > errBefore) errSeen = seen.filter((m) => m.t === "err").pop();
}
check("越界角被服务端拒绝并向网页上报 err", !!errSeen, errSeen ? errSeen.msg : "未收到 err");
await sleep(300);
check("越界角未被执行（角度停在服务端确认值）", angOf(latest(), baseID) === preBase,
  `${preBase} -> ${angOf(latest(), baseID)}`);

// 回滚后摇杆必须还能动 —— 这是"回滚是否生效"唯一的可观测判据：
// 若回滚失效，目标会停在 179.5，之后的每一帧都被拒，角度**永远不会**离开 preBase。
await pushJoy("L", -0.9, 0, 10, 50);
const movedBack = await waitState((s) => angOf(s, baseID) < preBase - 2, 2500);
check("被拒后摇杆仍能驱动该轴（目标已回滚，未卡在 179.5）", !!movedBack,
  `${preBase} -> ${angOf(latest(), baseID)}`);
await pushJoy("L", 0, 0, 4, 60);

// ---- 11. 并发冲刷后链路仍存活 --------------------------------------------
// 一次性灌入大量摇杆帧（不等待），验证单写泵串行化成立、JSON 不交叉、后端不崩。
// 判据 = 之后仍能收到**新的**合法 state，且紧随其后的离散命令仍生效。
const statesBeforeFlood = states().length;
for (let i = 0; i < 120; i++) {
  ws.send({ t: "joy", side: i % 2 ? "L" : "R", x: i % 3 ? 0.7 : -0.7, y: 0.2 });
}
ws.send({ t: "joy", side: "L", x: 0, y: 0 });
ws.send({ t: "joy", side: "R", x: 0, y: 0 });
await sleep(500);
const flooded = await waitMsg(() => states().length > statesBeforeFlood, 3000);
check("并发冲刷 120 帧后仍收到合法 state（JSON 未交叉、后端未崩）",
  !!flooded && jsonErrors === 0, `jsonErrors=${jsonErrors}，冲刷后新增 state ${states().length - statesBeforeFlood} 条`);
const stAlive = seen.filter((m) => m.t === "link_status").pop();
check("冲刷后链路仍为已连接", !stAlive || stAlive.connected === true);

// ---- 12. 收尾：四轴回 HOME 并回读一致 -------------------------------------
const sHomeEnd = await goHome();
check("收尾：四轴回到 90°（HOME）并回读一致", !!sHomeEnd, JSON.stringify(latest()?.angles));

// ---- 汇报 -----------------------------------------------------------------
const pass = results.filter((r) => r.ok).length;
const fail = results.length - pass;
console.log("");
console.log("===== 结果: " + (fail === 0 ? "PASS" : "FAIL") + `  (${pass} PASS / ${fail} FAIL，共 ${results.length} 项) =====`);
if (fail) {
  console.log("失败项：");
  for (const r of results.filter((x) => !x.ok)) console.log("  - " + r.name);
}
console.log(`（共收到 ${rawCount} 条 WS 消息，JSON 解析失败 ${jsonErrors} 条）`);
try { ws.sock.end(); } catch { /* ignore */ }
process.exit(fail === 0 ? 0 : 1);
