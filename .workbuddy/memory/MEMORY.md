# ArmPilot 大工程 · 长期记忆

> 跨子项目、长期有效的约定。子项目细节在各子项目 `.workbuddy/memory/`；方法论放 skill，不放 memory。
> 保持简洁、去本机绝对路径、不写凭据。

## 一、工程构成与链路

| 子项目 | 角色 | 端口 |
|--------|------|------|
| `MeArm-Device/` | ATmega328P 裸机固件（C + avr-libc，**不用 Arduino 框架**） | 串口 |
| `MeArm-RemoteControl/` | Go + 内嵌 Web，**舵机级**摇杆；默认 network 模式（TCP Client） | 9001 Web/WS · 9002 TCP 透传（**仅 serial**） |
| `MeArm-3D/` | Go 关节级后端 + React/Three.js 孪生前端 | 8090 `/ws/joint` · 9100 TCP JSON Lines · 5273 vite |

链路（端口刻意错开）：孪生前端→8090；遥控台 serial→9002 透传；遥控台 network→**TCP Client 连 9100**
（经 MeArm-3D 落 sim/real，**不经 Uno**）。

## 二、铁律

1. **模型/标定/限位真值只有一份** = `MeArm-3D/robot-package/<id>/model/robot.yaml`
   （`config/robots.yaml` 只是选择器指针、无数值；物理量在同包 `physics/physics.yaml`）。
   禁止硬编码；测试期望值**必须从配置派生**。**包内路径以 MeArm-3D 仓库根为基准**，非 monorepo 根。
2. **前端默认停 MockTransport 且不自动连接**是刻意设计（防误驱真机）。自动连接只靠
   `start.bat` 注入 `VITE_AUTO_CONNECT=ws`（`--real` 时另加 `VITE_AUTO_REAL=1`）；`npm run dev` 不注入。
   不要改全局默认值。
3. **舵机命名 ≠ 运动学角色**（相机实测确证）。改机构参数走「先量后改」。
4. **建新目录前先探针** `git check-ignore -v <dir>/probe.txt`：根 `.gitignore` 是 STM32 黑名单
   （`Drivers`/`third_party`/`Debug`(含小写)/`Build`/`Release`/`obj`）。`.workbuddy/skills/` 实测安全。
5. **git push 由用户执行**，agent 只做本地 commit / diff / branch。
6. **跨链路复用方向配置必须先做语义换算**：serial 的 `invert_*` 含一层 arm-device 固件补偿
   （`core/joystick.c`），TCP 链路无固件层 ⇒ 直接复用会让两种模式**推杆方向正好相反**且都"看起来正常"。
   入口 = `internal/protocol.NetInvertFor`（8 轴保持、其余取反）。
7. **给"本页↔设备"链路加第三方入口时，入口必须在协议里标注身份（origin）。** 前端只对
   `origin==='device'` 做命令侧跟随（写 `commandJoints` + 抑制回发），其余只写 `actualJoints`
   ⇒ 借用 `command` 会表现为**幽灵动、主臂不动**（2026-09-17 实测 8/8 帧）；真正危险的是本页
   `commandJoints` 停在旧值，下次操作把**整组旧指令**下发 ⇒ 真机跳回旧位姿。
   已修：新增 `OriginExternal`（ADR D84）；**来源须随命令同行**（不能等状态帧再从 `inflight` 取）。
8. **别承诺"服务还留着给你手测"**：用户重跑 `start.bat` 会按端口 `taskkill` 掉 agent 的后台实例，
   且 `.bat` 用 `start` 起独立窗口 ⇒ 其运行期日志 agent 读不到。交代环境状态要**现场复核**
   （端口 + 进程 StartTime），别引用上次结论。详库 → `~/.workbuddy/playbook.md` §3.4。

## 三、验收节奏与基线

先出计划并确认 → 实现 → 零警告（tsc 0 error / 双构）→ 单测 → 生产构建 → e2e →
增量汇报 + 同步 README / ADR / memory。新模块动手前**必先出实现计划**。

基线（2026-09-17 实跑）：vitest **442**（33 文件）· go test **114**（6 包，`go vet` 干净）·
遥控台 go test **39**（`-race` 干净）· Sim2Sim **30** · `verify_external_origin.mjs` **3** ·
`tests/e2e/external-origin-follow.mjs` **9**。

## 四、跑 e2e 前置（最常复发的假 FAIL）

1. 清残留：`taskkill //F //IM armpilot-backend.exe` + `arm-web.exe`，再逐个清
   8090/9100/9001/5273 的 LISTENING PID。
2. **必须用不带 `VITE_AUTO_*` 的干净 vite dev** —— 否则页面自动连后端，读数类断言被回推干扰 ⇒ 假 FAIL。
3. **判遥控台模式看 9002 是否 LISTENING**（serial 监听、network 不监听；9001 两种都监听）。

## 五、skill 与文档落点

- skill 全清单由系统每会话注入，此处不重复；**入口 = `.workbuddy/skills/armpilot-workspace/SKILL.md`**
  （含真实路由表；5 个 `arm-*` 专项 skill 曾一度消失，**已于 2026-09-17 重建**，取证见其 §3.1）。
- ⚠️ 后端**没有** FK/IK；XYZ→关节角只能走「冻结基线 + 三条实现互证」（`simulation/mujoco/sim2sim.py`
  + `fkref.py`，入口 `core/tools/run_sim2sim.py`），**不要自证**。
- 设计决策 → `MeArm-3D/docs/decisions.md`（最新在前）；项目全貌/验收数据 → 各子项目 `README.md`；
  工作日志 → 各自 `.workbuddy/memory/YYYY-MM-DD.md`。
