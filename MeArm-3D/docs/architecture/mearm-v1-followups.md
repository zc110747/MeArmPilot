# MeArm-V1 基线阶段 · 无关问题登记（B 类）

> 本阶段（基线冻结 + 最小抽象 + Sim2Sim 回归）**只做这一件事**。
> 过程中发现的、与抽象无关的问题一律**只记录不修** —— 混进来会让"抽象前后行为一致"
> 这条判据失去归因能力（spec §22 的分阶段提交要求同源）。
>
> 每条都给出：**症状 → 根因 → 建议修法 → 为什么现在不修**。
> 完成日期：2026-09-14

---

## F1 · `MeArm-RemoteControl` 缺 `go.mod`，Go 构建必然失败

**症状**：在 `MeArm-RemoteControl/` 下 `go build` 报
`go: cannot find main module; see 'go help modules'`。

**根因**：父仓 `D:/user_project/git/ArmPilot/.gitignore` 里有一条从 Linux 内核模板
抄来的 `*.mod*`。它**必然匹配** `go.mod`（`*.mod` 也一样），于是该文件的
`go.mod` 从未入库。本仓 `backend/go.mod` 踩过同一个坑，已用 `!go.mod` 负向规则救回。

**建议修法**：在父仓 `.gitignore` 加 `!go.mod`（一行救两个仓），然后
`git add -f MeArm-RemoteControl/go.mod` 或等负向规则生效后正常 add。

**现在不修**：跨仓改动，且属另一个项目；本阶段任何非 MeArm-3D 的改动都会污染 diff。

**判别口诀**：见到 `cannot find main module`，先跑
`git check-ignore -v <path>/go.mod`，**不要**急着 `go mod init`（会把已存在的文件覆盖掉）。

---

## F2 · Windows Git Bash 的三种"静默失败"工具语义

三条都在本阶段真实踩到过，共同特征是**不报错、结果看起来正常**：

| 工具 | 现象 | 根因 | 正确写法 |
|---|---|---|---|
| `sort -u` | 返回**空** | 解析到 `C:\Windows\System32\sort.exe`，它不认 `-u` | `awk '!seen[$0]++'`，或 `/usr/bin/sort` |
| `grep -E "a\|b"` / `\b` | 预检**恒返回"干净"** | `\b` 紧跟 `)` 在本机 GNU grep 3.0 失效；**漏 `-E` 时 `(` `\|` `)` 是字面字符** | 字段级比较：`netstat -ano \| tr -d '\r' \| awk '$4=="LISTENING"{n=split($2,p,":"); if (p[n]==8091) print $5}'` |
| `taskkill //PID <n>` | 报错并**静默不杀** | MSYS 路径改写把 `//PID` 变成 `/PID` 的失败形态 | `taskkill -F -PID <n>`（单横线）或 `Stop-Process` |

**最危险的一条是 `sort -u`**：本阶段用它做"按 PID 去重后清理 e2e 进程"，
循环一次都没执行，而输出完全正常 —— 直到复查端口才发现 8091 还在监听。

**建议修法**：把端口/进程管理收敛到一个 `tools/e2e-ports.mjs`（Node 实现，无这些语义坑），
`netstat`／`taskkill` 只出现在那一个文件里。

**现在不修**：新增工具属无关改动；且本阶段的 e2e 已用正确写法跑通（84/84）。

---

## F3 · 关节锚点辅助函数有两份实现

**症状**：`tests/sim/test_fk.py::joint_origin_mm()` 与
`tests/sim/harness.py::mujoco_joint_origin_mm()` 是同一段逻辑的两份拷贝。

**根因**：`test_fk.py` 里的那份是先写的局部实现；本阶段为 Sim2Sim 回归把同一逻辑
加进了共享模块 `harness.py`，但**没有回头去改既有测试文件**。

**建议修法**：让 `test_fk.py` 改为 `from harness import mujoco_joint_origin_mm`，
删掉局部副本。两者行为逐字相同（可动关节读 `xanchor`、固定关节读子 body `xpos`）。

**现在不修**：spec 明确要求「不得修改原有测试」。虽然把局部 helper 换成共享实现
**不会削弱任何断言**，但在"零回归"的声明里，任何既有测试文件的 diff 都会让
"哪些行为变了"的归因多一层噪声。留到下一个纯重构窗口做。

---

## F4 · `docs/serial-v1.md §5.1` 文档漂移：`hello` 没有 `connected` 字段

**症状**：按协议文档用 `hello.connected` 判真机接入，永远拿不到值。

**根因**：实现里没有这个字段。判"链路末端是不是真机"只能看
`hello.device === 'serial'`。

**建议修法**：改文档（实现是对的）—— 删掉 `connected` 行，补一句
「判定真机链路以 `device === 'serial'` 为准」。

**现在不修**：属另一条轨（协议文档），改它要连带核对固件侧 §4 的描述。

---

## F5 · ~~`device.mujoco.python` 是本机绝对路径~~ ✅ **已修掉（2026-09-16，ADR D81）**

**原症状**：换一台机器后 `device.mode: mujoco` **必然起不来**（启动握手超时）。

**原根因**：`backend/config.yaml` 里写的是
`C:/Users/lx176/.workbuddy/binaries/python/envs/default/Scripts/python.exe` ——
一个**已不存在的用户目录**（"换机器即失效"的标本；本文档写的"已修正为当前机器"
只是把它换成了另一台机器的绝对路径，机制并未变）。

**已落地的修法**（合并了原"建议修法 ①③"）：字段**留空**，改由
`cfg.ResolveMujocoPython(configured)` 解析，优先级：

1. 显式配置值（非空时直接用）
2. 环境变量 `ARMPILOT_MUJOCO_PYTHON`（临时切换 / CI 用，不必改被跟踪的文件）
3. `<用户目录>/.workbuddy/binaries/python/envs/default/{Scripts/python.exe, bin/python}`
   —— **相对用户目录**，换用户名照样成立
4. PATH 上的 `python`

`config.yaml` 与新增的 `config.mujoco.yaml` 两处都**留空**，并注明"**不要**在这里写本机绝对路径"。

**验收**：`verify_device_follow.mjs --config mujoco` **22/22 PASS**（连跑 3 次稳定）·
`go vet` 干净 · `go test ./...` 全绿。

---

## F6 · 探针泄漏判据的**扫描范围**没写进契约文本

**症状**：`grep -rl "__armPilot" dist/` 得到 **1** 个命中，看起来像探针进了生产包。

**真相**：命中的是 `dist/assets/index-*.js.map` —— 源映射内联了**源码文本**
（含 `import.meta.env.DEV` 守卫那一行与相关注释）。发布的 JS 是干净的。

**既有契约**（`docs/decisions.md`）写的是：
`grep -o "__armPilot" dist/assets/*.js | wc -l` = **0** —— 即**只扫 `.js`，不含 `.map`**。

**建议修法**：把这条扫描命令连同"为什么排除 `.map`"写进 README 的验收表，
而不是只在 decisions 里出现。否则下一个人对着 1 命中要重新查一遍。

**现在不修**：README 未改动即已正确；这条属"文档补齐"，本阶段不动 README 验收段。

---

## F7 · e2e 项数随环境变化（84 / 88）的语义

**症状**：同一份代码，完整跑 88 项，复用既有后端实例时 84 项。

**根因**：复用实例（`device=serial` 或非本脚本拉起）时，脚本会
① 跳过「断线重连」子项（它需要能拉起/杀掉后端）；② 跳过 opt-in 的真机段。
两者走的是 `skip()` 而**不是** `check()` —— 是**语义上的不适用**，不是回归。

**建议修法**：汇总行同时打印 `PASS / FAIL / SKIP` 三个数，并在 SKIP 时列出原因，
让"84 与 88"在输出里就能自解释。

**现在不修**：属 e2e 脚本的报表格式；本阶段刻意用**自起干净实例**跑，
避开该分支（本次断线重连子项照跑，84 = 除 opt-in 真机段外的全部）。

---

## F8 · 隔离端口的 e2e 仍是"手写长命令"

**症状**：跑一次隔离 e2e 需要在同一次 shell 调用里写下：端口预检（字段级）→
起后端（`.workbuddy/e2e-sim.yaml`，8091）→ 起前端（5276，不注入 `VITE_AUTO_CONNECT`）→
等待就绪 → 跑 `ui-smoke.mjs` → 清理 → 复查残留。**顺序错一步就 `ERR_CONNECTION_REFUSED`**
（`(cmd &)` 起的进程只活到本次工具调用结束）。

**根因**：`tests/e2e/ui-smoke.mjs` 假设"服务已就绪"，而本项目**必须**用隔离端口
（8090/5273 常被用户正在驱动真机的实例占着，不能杀）。

**建议修法**：落一个 `tools/e2e-isolated.mjs` 把上面七步固化，
把 `.workbuddy/e2e-sim.yaml` 的派生逻辑也放进去（从 `backend/config.yaml` 改端口）。

**现在不修**：属"新增工具"，与本阶段判据无关。本阶段的手写版本已跑通并留在
会话记录里，随时可固化。

---

## F9 · 两个 "version" 同名（已缓解，值得留意）

`config/robot.yaml` 里同时存在：

```yaml
version: 1          # 顶层：yaml 格式版本（数字，schema）
robot:
  version: 1.0.0    # 模型语义版本（字符串，= MeArm-V1 的版本）
```

**缓解措施**：`RobotModel.ts` 的字段注释里已明确写清二者不是一回事
（`version: number` 是格式版本，`modelVersion: string` 是模型语义版本）。

**建议修法**：将来若再动 schema，把格式版本改名 `formatVersion`，
彻底消掉同名。

**现在不修**：改名会触及三端解析与既有测试，属大规模改名（spec 禁止 13）。
