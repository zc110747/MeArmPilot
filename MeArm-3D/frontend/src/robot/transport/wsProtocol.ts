/**
 * wsProtocol —— 浏览器 ↔ Go 的 JSON 消息编解码（spec §二十一 / docs/serial-v1.md §5）。
 *
 * 本模块是**纯函数**，不认识 socket、不认识计时器，因此可以独立单测。
 *
 * 消息三分（spec §十九）在这里体现为三种 type：
 *   joint_command  下行：我们要机械臂去哪
 *   joint_state    上行：机械臂现在在哪（由后端从舵机角反算）
 *   error          上行：后端拒绝或链路异常（带固定错误码）
 */
import type { JointState } from '../model/Pose';
import type { RobotModel } from '../model/RobotModel';
import { actuatorsForJoint } from '../model/RobotModel';
import { jointToServo, servoToJoint } from '../calibration/calibration';

/** 与后端 `protocol.Version` 必须一致，不一致直接拒绝（避免字段静默错解） */
export const PROTOCOL_VERSION = 1;

// ---------------------------------------------------------------------------
// 链路精度 —— 决定「跟踪误差」的可分辨下限
// ---------------------------------------------------------------------------

/**
 * `JR` 线的关节角步进（度）：**0.1°**。
 *
 * 依据 `docs/serial-v1.md` §4：「关节角整帧（degree，浮点，保留 1 位小数）」。
 * 后端控制器把 `joint_command` 编码成 `JR` 时按这个精度量化 —— 也就是说
 * **链路本身无法表达比 0.1° 更细的命令**。
 *
 * ⚠️ 为什么这个常量必须存在：
 * 前端内部的 `commandJoints` 是全精度浮点（例如 HOME 位的 `elbow = 112.6185771989`），
 * 而回推的 `joint_state` 来自「线上值 112.6 → 舵机角 → 反算回关节角」。
 * 若拿**全精度命令**去和回推值比较，会得到一个 **永远收敛不到 0 的量化残差**
 * （实测恒为 `0.02°`，`moving` 因此永久为 true ⇒ 面板永远显示"正在逼近目标"）。
 * 那是个**假误差**：链路已经到位，只是双方用了不同的精度表述同一个位置。
 *
 * 所以跟踪误差的比较基准必须是**线上值**（见 `quantizeForWire`）。
 */
export const WIRE_JOINT_STEP_DEG = 0.1;

/**
 * 把关节角归整到线上精度（四舍五入到 0.1°）。
 *
 * 用于**比较**，不用于发送：JSON 通道仍传全精度（后端有权选更细的编码），
 * 但「误差」这个概念只能在链路能表达的精度上定义。
 *
 * ⚠️ 实现细节：必须写成 `Math.round(v * 10) / 10` 而不是 `Math.round(v / 0.1) * 0.1`。
 * 后者会乘上不精确的 `0.1`（0.1000000000000000055…），结果可能与
 * `JSON.parse("112.6")` 差 1 ulp，于是"误差为 0"变成"误差 1e-14"，
 * 让 `moving` 再次永远为真。除以 10 是正确舍入的，与解析字面量逐位相同。
 */
export function quantizeForWire(joints: JointState): JointState {
  const out: JointState = {};
  for (const [id, value] of Object.entries(joints)) {
    if (!Number.isFinite(value)) continue;
    out[id] = Math.round(value * 10) / 10;
  }
  return out;
}

/**
 * 把关节命令归整到**真机固件能表达的格点**（Phase 9）。
 *
 * 真机的瓶颈不在 JSON，而在**固件的舵机角是整数**：`MeArm-Device/core/cmd.c`
 * 用 `parse_u8` 收角度、`arm_set_angle` 按硬限位钳位，只吃整数度。
 * 所以命令经过「舵机空间取整 → 反算回关节角」之后那个值，
 * 才是设备侧真正会停的位置。
 *
 * ⚠️ 不这样做会重演 sim 那次的假误差，而且**大一个量级**：
 * S7（肩）的 `scale = 1.44018` ⇒ 0.5 舵机度的取整误差在关节侧是 **0.347°**，
 * 而 sim 那次只有 0.0186° —— 面板会永久停在一个假的 `0.35°` 跟踪误差上，
 * "正在逼近目标"熄灭不掉。
 *
 * 多舵机关节取平均，与后端 `protocol.ServoAnglesToJoints` 同口径。
 */
export function quantizeViaServo(model: RobotModel, joints: JointState): JointState {
  const out: JointState = {};
  for (const [id, value] of Object.entries(joints)) {
    if (!Number.isFinite(value)) continue;
    const acts = actuatorsForJoint(model, id);
    if (acts.length === 0) {
      out[id] = value; // 无执行器的关节（不该出现）原样透传
      continue;
    }
    let sum = 0;
    for (const a of acts) {
      // 与固件一致：先取整到整数舵机度，再反算回关节角
      sum += servoToJoint(a, Math.round(jointToServo(a, value)));
    }
    out[id] = sum / acts.length;
  }
  return out;
}

// ---------------------------------------------------------------------------
// 消息类型 / 错误码
// ---------------------------------------------------------------------------

export const CLIENT_JOINT_COMMAND = 'joint_command';
export const CLIENT_PING = 'ping';
export const CLIENT_STATUS_REQUEST = 'status_request';
/**
 * 调试直通：把一行**原始设备指令**（`JOY …` / `SET …`）发给链路末端。
 *
 * 用途是让"下位机被外部手段改动 → 界面跟随"这条链路**在仿真里就能验证**
 * （真机上对应硬件摇杆 / 红外遥控，仿真里对应这两条固件级命令）。
 *
 * ⚠️ 它绕过关节限位校验（固件按舵机硬限位自钳），只应被调试脚本 / 验收探针
 *    使用，**不要**接到用户界面上。
 */
export const CLIENT_DEVICE_COMMAND = 'device_command';

export const SERVER_HELLO = 'hello';
export const SERVER_JOINT_STATE = 'joint_state';
export const SERVER_ERROR = 'error';
export const SERVER_PONG = 'pong';
export const SERVER_DEVICE_STATUS = 'device_status';

/**
 * 关节状态的来源（字面量与后端 `protocol.OriginCommand` / `OriginDevice` 一致）。
 *
 * `device` 表示设备侧**自主变化**（硬件摇杆 / 红外遥控 / 面板手拧 / 调试直控）——
 * 界面必须让命令侧跟随，否则滑杆、主臂、目标点会停在旧值上，
 * 而画面上看不出任何异常。
 *
 * ⚠️ 缺省（字段不存在）按 `command` 处理：老后端不发它，行为与引入前一致。
 */
export const ORIGIN_COMMAND = 'command';
export const ORIGIN_DEVICE = 'device';

export const CODE_BAD_MESSAGE = 'BAD_MESSAGE';
export const CODE_VERSION = 'VERSION_MISMATCH';
export const CODE_JOINT_LIMIT = 'JOINT_LIMIT';
export const CODE_DEVICE_DOWN = 'DEVICE_UNAVAILABLE';
export const CODE_ACK_TIMEOUT = 'ACK_TIMEOUT';
export const CODE_INTERNAL = 'INTERNAL';

// ---------------------------------------------------------------------------
// 后端模型元数据（hello 的 payload）
// ---------------------------------------------------------------------------

export interface BackendLimitRow {
  id: string;
  role: string;
  min: number;
  max: number;
}

export interface BackendCalibrationRow {
  jointId: string;
  servoId: string;
  channel: number;
  offset: number;
  scale: number;
  reverse: boolean;
  servoLo: number;
  servoHi: number;
}

export interface BackendModelInfo {
  id: string;
  name: string;
  source: string;
  jointOrder: string[];
  limits: BackendLimitRow[];
  calibration: BackendCalibrationRow[];
  homePose: Record<string, number>;
}

// ---------------------------------------------------------------------------
// 信封
// ---------------------------------------------------------------------------

export interface ServerEnvelope {
  version: number;
  type: string;
  timestamp?: number;
  seq?: number;
  joints?: JointState;
  code?: string;
  message?: string;
  model?: BackendModelInfo;
  device?: string;
  connected?: boolean;
  /** 关节状态来源（`ORIGIN_COMMAND` / `ORIGIN_DEVICE`）；缺省按 command 处理 */
  origin?: string;
}

// ---------------------------------------------------------------------------
// 编码（浏览器 → 服务器）
// ---------------------------------------------------------------------------

/**
 * 编码关节角命令。
 *
 * `seq` 是本项目在 spec 字段之外加的可选扩展：拖动时命令高频变化，
 * 前端用单调递增序号便于排查乱序；后端容忍缺失（默认 0）。
 */
export function encodeJointCommand(
  joints: JointState,
  options: { seq?: number; timestamp?: number } = {},
): string {
  const envelope: Record<string, unknown> = {
    version: PROTOCOL_VERSION,
    type: CLIENT_JOINT_COMMAND,
    timestamp: options.timestamp ?? Date.now(),
    joints,
  };
  if (options.seq !== undefined) envelope.seq = options.seq;
  return JSON.stringify(envelope);
}

export function encodePing(timestamp: number = Date.now()): string {
  return JSON.stringify({ version: PROTOCOL_VERSION, type: CLIENT_PING, timestamp });
}

export function encodeStatusRequest(timestamp: number = Date.now()): string {
  return JSON.stringify({ version: PROTOCOL_VERSION, type: CLIENT_STATUS_REQUEST, timestamp });
}

/**
 * 编码一条**调试直通**指令（`JOY <id> <raw>` / `SET <id> <ang>`）。
 *
 * 见 `CLIENT_DEVICE_COMMAND`：它让仿真也能复现"摇杆把机械臂拧了 30°"，
 * 从而验证界面是否跟随 —— 否则这条链路只能靠手动拨硬件来验。
 */
export function encodeDeviceCommand(line: string, timestamp: number = Date.now()): string {
  return JSON.stringify({
    version: PROTOCOL_VERSION,
    type: CLIENT_DEVICE_COMMAND,
    timestamp,
    line,
  });
}

// ---------------------------------------------------------------------------
// 解码（服务器 → 浏览器）
// ---------------------------------------------------------------------------

/**
 * 解析一条上行消息。
 *
 * 返回 `null` 表示**这条消息不可用**（非法 JSON / 缺 type / 版本不匹配）——
 * 调用方应丢弃它而不是当成合法状态，否则一条坏消息会把 Actual 写成 NaN。
 */
export function decodeServer(raw: string): ServerEnvelope | null {
  let parsed: unknown;
  try {
    parsed = JSON.parse(raw);
  } catch {
    return null;
  }
  if (parsed === null || typeof parsed !== 'object') return null;
  const env = parsed as Partial<ServerEnvelope>;
  if (typeof env.type !== 'string' || env.type === '') return null;
  // 版本：缺省视为兼容（老后端），显式写了就必须一致
  if (env.version !== undefined && env.version !== PROTOCOL_VERSION) return null;
  return env as ServerEnvelope;
}

export function isJointState(env: ServerEnvelope): boolean {
  return env.type === SERVER_JOINT_STATE && env.joints !== undefined && env.joints !== null;
}

export function isError(env: ServerEnvelope): boolean {
  return env.type === SERVER_ERROR;
}

export function isHello(env: ServerEnvelope): boolean {
  return env.type === SERVER_HELLO && env.model !== undefined && env.model !== null;
}

export function isPong(env: ServerEnvelope): boolean {
  return env.type === SERVER_PONG;
}

// ---------------------------------------------------------------------------
// 模型一致性校验 —— "标定表只有一份"的在线检查
// ---------------------------------------------------------------------------

/** 逐项比对的容差（度）。后端算的是同一批浮点数，正常应逐位相同 */
const MODEL_EPS = 1e-6;

/**
 * 比对本地 RobotModel 与后端 hello 里的模型真值，不一致时返回人类可读描述。
 *
 * 检查三件事：
 *   1. 关节顺序（决定 JR 四元组位次，错位是最危险的）
 *   2. 关节限位（前后端限位不同 → 一侧放行另一侧拒绝）
 *   3. 标定通道映射（S7=肩 / S8=肘 —— 早期按固件命名推定是错的，必须能测出来）
 *
 * 一致时返回 `null`。
 */
export function describeModelMismatch(
  local: RobotModel,
  remote: BackendModelInfo,
): string | null {
  // 与后端 robot.JointOrder() 对齐：参与状态帧的只有**可控关节**（revolute）。
  // 被动关节（腕）没有舵机、不进 JointState，混进来会让顺序校验直接失败。
  const localOrder = local.joints.filter((j) => j.type === 'revolute').map((j) => j.id);
  if (localOrder.length !== remote.jointOrder.length) {
    return `关节数量不一致：本地 ${localOrder.length} 个，后端 ${remote.jointOrder.length} 个`;
  }
  for (let i = 0; i < localOrder.length; i += 1) {
    if (localOrder[i] !== remote.jointOrder[i]) {
      return `关节顺序不一致：第 ${i + 1} 位本地是 ${localOrder[i]}，后端是 ${remote.jointOrder[i]}（JR 位次会错位）`;
    }
  }

  for (const row of remote.limits) {
    const joint = local.joints.find((j) => j.id === row.id);
    if (!joint) {
      return `后端存在本地没有的关节：${row.id}`;
    }
    if (
      Math.abs(joint.limits.min - row.min) > MODEL_EPS ||
      Math.abs(joint.limits.max - row.max) > MODEL_EPS
    ) {
      return `关节 ${row.id} 限位不一致：本地 ${joint.limits.min}..${joint.limits.max}，后端 ${row.min}..${row.max}`;
    }
  }

  for (const row of remote.calibration) {
    const actuator = local.actuators.find((a) => a.jointId === row.jointId);
    if (!actuator) {
      return `后端标定表存在本地没有的执行器（关节 ${row.jointId}）`;
    }
    if (actuator.channel !== row.channel) {
      return `关节 ${row.jointId} 的舵机通道不一致：本地 S${actuator.channel}，后端 S${row.channel}`;
    }
    if (
      Math.abs(actuator.offset - row.offset) > MODEL_EPS ||
      Math.abs(actuator.scale - row.scale) > MODEL_EPS ||
      actuator.reverse !== row.reverse
    ) {
      return `关节 ${row.jointId} 标定参数不一致：本地 offset=${actuator.offset} scale=${actuator.scale} reverse=${actuator.reverse}，` +
        `后端 offset=${row.offset} scale=${row.scale} reverse=${row.reverse}`;
    }
  }

  return null;
}
