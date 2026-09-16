# ArmPilot 大工程 · 长期记忆

> 本文件记录**跨子项目、长期有效**的约定与事实。
> 子项目内部细节放在各自的 `.workbuddy/memory/`；可复用的方法论放 `.workbuddy/skills/`。
> 维护约定：保持简洁、去本机绝对路径、不写凭据。

## 一、工程构成（2026-09-12）

| 子项目 | 角色 | 端口 |
|--------|------|------|
| `MeArm-Device/` | ATmega328P(Uno) 裸机固件，C + avr-libc，**不用 Arduino 框架** | 串口 |
| `MeArm-RemoteControl/` | Go + 内嵌 Web，**舵机级**摇杆服务 | **8080**(WS) / 9001(TCP) |
| `MeArm-3D/` | Go 关节级后端 + React/Three.js 数字孪生前端 | **8090**(`/ws/joint`) / 5273(vite dev) |

8080 与 8090 刻意错开，可同时运行。

## 二、铁律

1. **模型 / 标定 / 限位真值只有一份 = `MeArm-3D/config/robot.yaml`**。
   禁止硬编码尺寸/角度/限位；测试期望值必须**从配置派生**。
2. **前端默认停在 MockTransport 且不自动连接**是刻意设计（防误驱真机）。
   自动连接只能靠 `start.bat` 注入 `VITE_AUTO_CONNECT=ws`（+ `--real` 时 `VITE_AUTO_REAL=1`）。
   `npm run dev` 不注入 ⇒ 手动调试行为不变。不要改全局默认值。
3. **舵机命名 ≠ 运动学角色**（相机实测确证）。改机构参数走「先量后改」。
4. **建新目录前必须探针**：`git check-ignore -v <dir>/probe.txt`。
   根 `.gitignore` 是 STM32 工程黑名单（`Drivers`/`third_party`/`Debug`(含小写)/`Build`/`Release`/`obj`）。
   `.workbuddy/skills/` 已实测安全。
5. **git push 由用户自行执行**，agent 只做本地 commit。

## 三、验收节奏

先出计划并确认 → 实现 → 零警告（tsc 0 error / 双构）→ 单测 → 生产构建 → e2e →
增量汇报 + 同步 README / ADR / memory。任何新模块动手前**必先出实现计划**。

当前基线（2026-09-12）：vitest **233** 项 · e2e **52** 项 · 后端 go test **56** 项。

## 四、跑 e2e 前置（最常复发的假 FAIL）

1. 清残留：`taskkill //F //IM armpilot-backend.exe` + 逐个清 8090/5273 的 LISTENING PID
2. **必须用不带 `VITE_AUTO_*` env 的干净 vite dev** —— 否则页面自动连后端，
   读数类断言会被回推干扰，出现与代码无关的 FAIL。

## 五、skill 路由（用户级 `~/.workbuddy/skills/`）

| skill | 管什么 |
|-------|--------|
| `arm-mechanism-photogrammetry` | 相机实测反解机构参数（offset/scale/reverse/限位） |
| `arm-robot-serial` | 串口链路陷阱（Uno 静默窗口、首条指令丢弃、ACK 门控、非重叠 I/O） |
| `arm-ws-joint-link` | 关节级 WS 闭环（回执=目标角、latest-wins、量化下限、重连补发） |
| `arm-backend-new-ingress` | **新增入口**（TCP/API/MQTT…）只当 adapter：只调 `controller.Apply`、几何从 robot.yaml 推导、冻结基线当判据、`Close()` 先关活连接 |
| **`armpilot-workspace`**（本工程） | 导航 + 跨项目铁律 + 路由表（**入口**） |

四者各管一段、无重叠：机构（几何真值）→ 串口（字节链路）→ WS（语义链路）→ 新入口（adapter 分层）。
⚠️ 后端**没有** FK/IK；要 XYZ→关节角时按 `arm-backend-new-ingress` §4 走冻结基线，别自证。

## 六、文档落点

- 设计决策 → `MeArm-3D/docs/decisions.md`（ADR，最新在前）
- 项目全貌/验收数据 → 各子项目 `README.md`
- 工作日志 → 各自 `.workbuddy/memory/YYYY-MM-DD.md`
- 可复用方法论 → skill（不放 memory）
