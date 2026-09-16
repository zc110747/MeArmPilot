/**
 * `src/robot` 统一出口。
 *
 * 分层（严格对应 spec §六）：
 *   definition/   RobotDefinition（机器人定义的最小统一入口）
 *   model/        RobotModel / Link / Joint / Actuator / Pose / RobotState / RobotCommand
 *                 + loadRobotModel（配置 → 模型）/ robotConfigRegistry（id → 配置）
 *   kinematics/   coordinate（坐标系转换层）/ transform（矩阵）/ fk / ik
 *                 + KinematicsEngine（统一调用面）/ IKResult（统一结果）
 *                 ⚠️ 各型号的实现（`ik.ts` / `engine.ts`）**住在各自的包**
 *                    `robot-package/<id>/kinematics/`，不在本目录。
 *   registry/     RobotRegistry（id → 定义 + 引擎的**唯一**分派表；
 *                 表由 `import.meta.glob` 从包内自动发现，不手写）
 *   calibration/  关节角 ↔ 舵机角 标定
 *   transport/    RobotTransport 抽象 + Mock / WebSocket 实现
 */
export * from './definition/RobotDefinition';

export * from './model/Pose';
export * from './model/Link';
export * from './model/Joint';
export * from './model/Actuator';
export * from './model/RobotModel';
export * from './model/RobotState';
export * from './model/RobotCommand';
export * from './model/configError';
export * from './model/robotIds';
export * from './model/robotConfigRegistry';
export * from './model/loadRobotModel';
export * from './model/linkFeedback';
// 末端目标「参数覆写」层：只改参数（关节限位收紧 / 球壳内径覆写 / 判据容差），
// 不碰任何算法。默认值刻意全部退化为"无覆写" ⇒ 不配置时行为与引入前逐位一致。
export * from './model/parameterOverrides';

export * from './kinematics/transform';
export * from './kinematics/coordinate';
export * from './kinematics/fk';
// ⚠️ `ik.ts`（MeArm 的解析解）与两台机器人的引擎实现**已搬进各自的包**
//    （Phase 2 步④）：`robot-package/<id>/kinematics/`。
//    它们**刻意不从本出口再导出** —— 一旦从 Core 的出口导出，Core 就有了
//    "MeArm 的解"这个概念，而 `Core 里不得出现任何型号名` 是本项目的铁律。
//    上层要解算请走 `RobotRegistry.loadRobot(id).kinematics`（统一 `IKResult` 形状）。
//
// ⚠️ 命名陷阱（搬走之后依然存在）：`IkResult`（包的 `ik.ts`，MeArm 原生判别联合）
//    与 `IKResult`（本目录，统一结果形状）**只差一个字母大小写**。
//    前者是算法实现细节、住在包里；后者是给上层用的稳定契约、住在 Core。
//    两者同时存在是刻意的（替换原生返回会破坏现有 2000 组闭环验收）。
export * from './kinematics/IKResult';
export * from './kinematics/KinematicsEngine';

export * from './registry/RobotRegistry';

export * from './interaction/dragPlane';

export * from './calibration/calibration';

export * from './transport/RobotTransport';
export * from './transport/timer';
export * from './transport/socket';
export * from './transport/wsProtocol';
export * from './transport/MockTransport';
export * from './transport/WebSocketTransport';

export * from './teach/teachTrack';
export * from './teach/TeachPlayer';
