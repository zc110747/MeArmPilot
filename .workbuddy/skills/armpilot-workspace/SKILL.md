---
name: armpilot-workspace
description: ArmPilot 大工程（MeArm-3D 数字孪生 / MeArm-Device AVR 固件 / MeArm-RemoteControl 摇杆服务）的**工程导航与跨项目铁律**。覆盖：三个子项目的分工与端口分配（9001/9002 摇杆 · 8090/9100 数字孪生）、单一真值源 robot-package/<id>/model/robot.yaml 的跨端共享方式、前端默认停在 Mock 与自动连接的 env 驱动约定、e2e 跨批次状态残留的必查清单（跑前清残留 + 用干净 vite dev）、建新目录前的 gitignore 探针、以及"该查哪个专项 skill"的**实际**路由表（含已缺失 skill 的警示）。当在 ArmPilot 下新建模块、跨子项目联调、改动配置真值、跑验收、或不确定某项知识沉淀在哪时**先读本文件**。
agent_created: true
---

# ArmPilot 大工程 · 导航与铁律

大工程 `ArmPilot/` 下有三个子项目。**本文件是入口**：先在这里定位"该改哪儿、该查哪个 skill"，
再进专项文档。专项 skill 的**权威正文不在本文件**（避免双重维护），只登记路由。

> 2026-09-17 校订：端口 / 真值源路径 / 路由表 / 基线数字均已按**磁盘与代码实况**更正。
> 旧版里的 `8080`、`config/robot.yaml`、以及三个已不存在的 `arm-*` skill 都是历史遗留。

## 0. 三个子项目的分工（别搞混端口）

| 子项目 | 角色 | 语言/栈 | 端口 | 通信 |
|--------|------|---------|------|------|
| `MeArm-Device/` | **下位机固件** | C + avr-libc（裸机，**不用 Arduino 框架**），ATmega328P @16MHz | 串口 | 文本协议，每条合法指令回 `OK ...` / `ERR ...`，事件以 `# ` 前缀 |
| `MeArm-RemoteControl/` | **舵机级**摇杆服务 | Go（标准库）+ 内嵌 Web | **9001**（Web/WS）+ **9002**（TCP 透传，**仅 serial 模式**） | 把固件串口暴露成 Web/TCP；**默认 network 模式** = TCP Client 直连 9100 |
| `MeArm-3D/` | **数字孪生 + 关节级后端** | Go（`armpilot/backend`）+ React/Three.js 前端 | **8090**（`/ws/joint`）+ **9100**（TCP JSON Lines）+ 5273（vite dev） | 关节级 WebSocket(JSON)；`wsserver → controller → device(sim \| serial)` |

> ⚠️ 端口**刻意错开**：孪生前端→8090；遥控台 serial→9002 透传；遥控台 network→**TCP Client 连 9100**
> （经 MeArm-3D 落到 sim/real，**不经 Uno**）。历史文档里的 **8080 已废弃**。新增服务前先确认端口不撞。
> 判"遥控台在哪个模式"看 **9002 是否 LISTENING**（serial 会监听、network 不监听；9001 两种都监听）。

## 1. ★ 铁律：模型 / 标定 / 限位真值只有一份

**唯一真值源 = `MeArm-3D/robot-package/<id>/model/robot.yaml`**（物理量在同包 `physics/physics.yaml`）。

- `MeArm-3D/config/robots.yaml` **只是选择器指针**，**不含任何数值**；后端启动时 `robot.Load()` 按它取包。
- **禁止**在任何地方硬编码连杆尺寸、角度 offset、限位、舵机映射。
- 测试的期望值必须**从配置派生**，不许写常数（写死会与真值漂移，且改动时不会报错）。
- ⚠️ 引用包内路径时以 **MeArm-3D 自己的仓库根**为基准，**不是** monorepo 根。
- 舵机命名 ≠ 运动学角色（已用相机实测确证）。改机构参数一律走「先量后改」。

## 2. 前端「默认不连硬件」是刻意设计，别擅自改默认值

`MeArm-3D/frontend` 的传输方式**默认停在 MockTransport**（浏览器内仿真），
且**不会自动连接**任何后端。理由：网页一打开就控制真实机械臂是危险的。

因此"一键启动后要接硬件"只能靠**环境变量注入**（`start.bat` 负责）：

| 变量 | 作用 |
|------|------|
| `VITE_AUTO_CONNECT=ws` | 页面挂载后自动连 WebSocket 后端（而非浏览器内 Mock） |
| `VITE_AUTO_REAL=1` | 连接成功后自动切 Real Robot（带准入校验） |

`npm run dev` **不注入** ⇒ 手动开发行为完全不变。**不要**把 websocket 改成全局默认值：
那会破坏手动调试、并让生产构建去连一个不存在的后端。

> ⚠️ 副作用（2026-09-17 实测）：`start.bat` 起的前端**带** `VITE_AUTO_CONNECT=ws` ⇒ 页面自动连接。
> **不要**在它上面跑读数类 e2e（会撞 §5 的假 FAIL），要重跑就自己另起干净实例。

## 3. ★ 路由表：想查什么，去哪个 skill

**下表只登记磁盘上真实存在的 skill**（2026-09-17 对 `~/.workbuddy/skills/` 与本仓 `.workbuddy/skills/`
做过全量 `ls` 核实）。已缺失的名字见 §3.1，**别再按旧文档去找它们**。

| skill | 位置 | 管什么 |
|-------|------|--------|
| **`armpilot-workspace`** | 本仓 `.workbuddy/skills/` | **本文件** —— 工程导航 + 跨项目铁律（入口） |
| `arm-tcp-client-link` | 用户级 | **客户端侧**新增 TCP Client 传输（保留原链路）：基准缺失先补只读查询、跨链路方向换算、被拒回滚可观测、单写泵、Sim2Sim 验收从配置派生；也覆盖"入口**能驱动** ≠ 对端界面显示对了"（origin 语义） |
| `robotics-multiphysics-vmodel-workflow` | 用户级 | 多物理场 / 多实现项目：真值只有一份、冻结基线 + **语义核心哈希**、**三条实现互证**（不是自证） |
| `duplicated-truth-forensics` | 用户级 | "同一个量在两处被独立实现、数值悄悄不一致"的取证套路（症状轻微偏差、随机拒答、不崩） |

### 3.1 ⚠️ 已缺失的历史路由目标（2026-09-17 核实，**别再引用**）

以下名字在旧文档 / 旧日志里被当作 skill 引用，但**磁盘上不存在**；本仓 git 历史里
`.workbuddy/skills/` 也只提交过 `armpilot-workspace`：

`arm-mechanism-photogrammetry` · `arm-robot-serial` · `arm-ws-joint-link` · `arm-backend-new-ingress` ·
`arm-mujoco-physics-sim`

相应知识**不再有单点汇总**，目前散落在：各子项目 `.workbuddy/memory/`、`MeArm-3D/docs/decisions.md`
（ADR）、`MeArm-3D/.workbuddy/analysis/`、以及代码注释里。**要恢复请当成一次显式任务做**
（别假设它们还在，也别照着旧路由表去 load）。

## 4. ★ 验收节奏（每个模块完成即走一遍）

1. **先出实现计划**并获确认（任何新模块动手前）
2. 实现 → **零警告**（前端 `tsc` 0 error；固件侧 Debug/Release 双构）
3. 单测：前端 vitest **442**（33 文件）· 后端 `go test ./...` **114** 个顶级用例（+ `go vet` 干净）·
   遥控台 `go test ./...` **39**（`-race` 干净）
4. 生产构建：`vite build`
5. 端到端：Sim2Sim `MeArm-RemoteControl/tools/sim2sim-check.mjs` **30** ·
   `MeArm-3D/core/tools/verify_external_origin.mjs` **3** ·
   `MeArm-3D/frontend/tests/e2e/external-origin-follow.mjs` **9** ·
   `MeArm-3D/frontend/tests/e2e/ui-smoke.mjs`（**需先起后端 + 前端**，且用 §5 的干净 vite dev）
6. 增量汇报 + 交付清单（✅ 收尾）+ 同步 README / ADR / memory

> ⚠️ **本机不能用 `npm run`**：与 `bash` / `npx` 同源，会撞 **WSL 拦截**
> （`PROGRAM BLOCKED BY SECURITY POLICY: wsl.exe`）。用 managed node 直调入口：
> `node node_modules/vitest/vitest.mjs run`、`node node_modules/typescript/bin/tsc -b --force`、
> `node node_modules/vite/bin/vite.js`。
>
> 基线数字**只在 `MeArm-3D/README.md` 与工程 `.workbuddy/memory/MEMORY.md` 各写一处**，
> 改测试数量时同步这两处，不要在多份文档里各留一个。

## 5. ⚠️ 跑 e2e 前必查：清掉残留进程与带 env 的 dev server

这是本项目**最常复发的假 FAIL 来源**（已记入 ADR D40/D41/D43 的"跨批次状态残留"章节）。

```bash
# 1) 清残留后端与 dev server（8090 / 9100 / 9001 / 5273）
taskkill //F //IM armpilot-backend.exe 2>/dev/null
taskkill //F //IM arm-web.exe 2>/dev/null
netstat -ano -p TCP | grep -E ":8090|:9100|:9001|:5273" | grep LISTENING   # 逐个 taskkill //F //PID <pid>

# 2) ★ 关键：必须用**干净的** vite dev（不带 VITE_AUTO_* env）跑 e2e
cd MeArm-3D/frontend && ./node_modules/.bin/vite --port 5273
```

> **为什么**：若 5273 上跑着之前注入过 `VITE_AUTO_CONNECT=ws` 的实例，页面会自动连后端，
> e2e 读到的表格/状态会被回推干扰 ⇒ 出现**与代码无关的 FAIL**。
> 判据：FAIL 项集中在"读数类"断言且换干净 dev 后即消失。
>
> 另注（本机 `grep` 不可信）：端口预检请用 PowerShell 的 `Get-NetTCPConnection -State Listen`
> 复核，`netstat | grep` 会**假通过**。
> `start.bat` **不杀已有实例**、且 listen 失败**不退出** ⇒ 启动前先确认同名 exe 只有一个。

**前端资源占用**（e2e 需真实浏览器渲染，勿与其它重任务并行）：
e2e 用 headless Edge 直连 CDP，零额外依赖（Node ≥22 自带 `fetch`/`WebSocket`）。

## 6. 沙箱/工具链环境坑（WorkBuddy 会话内）

| 现象 | 处理 |
|------|------|
| `npx <tool>` / `npm run` 触发 **WSL 黑名单**（`wsl.exe`） | 改用 `node node_modules/<pkg>/...` 直调入口 |
| 从 Bash 调 PowerShell 被安全策略拦截 | 用 PowerShell 工具；若其输出通道不回显，用 Bash 查 `netstat`/`tasklist` 复核 |
| `/tmp` 写入静默失败 | 日志/探针文件写到**项目根或 `.workbuddy/captures/`**（用完即删） |
| 沙箱每次工具调用**回收子进程** | "服务能否启动"的验证必须在**同一次调用内**起服务 + 发请求 |
| 用户重跑 `start.bat` 会**按端口 taskkill** | 别承诺"服务还留着给你手测"；现场复核端口 + 进程 **StartTime** |
| 运行过的 `.exe` 被锁 | 构建到新文件名，或申请沙箱放行后重建 |
| Git Bash 的 `sed -i` 剥 CRLF | 用 Python 二进制模式替换；提交前 `git diff --numstat` 自查 |

## 7. 新增子项目 / 新目录前：先探针

大工程根 `.gitignore` 是给 STM32 工程写的**黑名单**（`Drivers` / `third_party` / `Debug`(含小写
`debug`) / `Build`(含小写) / `Release` / `obj` 等）。被忽略的文件 `git add` 会**静默跳过**。

```bash
mkdir -p <新目录> && touch <新目录>/probe.txt
git check-ignore -v <新目录>/probe.txt   # 有输出 = 被屏蔽，换名
```

`.workbuddy/skills/` 已实测**未被屏蔽**（2026-09-12 探针确认）。

## 8. 文档落点约定

| 内容 | 落点 |
|------|------|
| 关键设计决策（"为什么这么做"） | `MeArm-3D/docs/decisions.md`（ADR，**最新在前**，代码注释引编号如 `D84`） |
| 项目全貌 / 验收数据 / 使用说明 | 各子项目 `README.md`（基线数字的**唯一**落点之一） |
| 跨会话工作日志 | 各自的 `.workbuddy/memory/YYYY-MM-DD.md`（append-only，最新在前） |
| 跨子项目铁律 / 基线 | 工程 `.workbuddy/memory/MEMORY.md`（**保持 ≤3000 字符**） |
| 可复用的方法论 | 提升为 skill（见 §3 路由表），**不放 memory** |
| 环境/工具语义坑的详库 | 用户级 `~/.workbuddy/playbook.md`（按 § 精确读，别整份读） |

## 9. 给"本页 ↔ 设备"链路加第三方入口：入口必须标注身份（origin）

2026-09-17 实测踩过，**这是本工程最容易复发的跨端缺陷**：

- 孪生前端画**两棵树**：主臂 = `commandJoints`（指令"要去哪"）、幽灵 = `actualJoints`（实际"现在在哪"）；
  `transportBridge` 只对 `origin === 'device'` 做**命令侧跟随**（写 `commandJoints` + 抑制回发）。
- 第三方入口（TCP JSON v1）若借用 `command` ⇒ 页面**幽灵动、主臂不动**；更危险的是本页
  `commandJoints` 停在旧值，用户下次动**任何**滑杆会把**整组旧指令**下发 ⇒ **真机跳回旧位姿**。
- 修法 = 新增 `OriginExternal`（ADR **D84**），前端把它与 `device` 同等对待（跟随 + 抑制回发）。
- ⚠️ 两个易漏点：① **来源必须随命令同行**（不能等状态帧到了再从 `inflight` 取，`OK JR` 一到就清空了）；
  ② **前端有两层 origin 闸门**（后端常量 + `WebSocketTransport` 白名单），只改一层 ⇒ "后端标了、页面不跟随"。

判据脚本：`MeArm-3D/core/tools/verify_external_origin.mjs`（后端侧分段）+
`MeArm-3D/frontend/tests/e2e/external-origin-follow.mjs`（浏览器侧）。
