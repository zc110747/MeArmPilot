# MeArm-3D · 项目长期记忆（索引）

> **本文件只是索引**：铁律 + 验收命令 + 去哪找细节。明细在 `playbook.md`（§1–§12），
> 决策理由在 `docs/decisions.md`（D1–D80），每日过程在 `YYYY-MM-DD.md`。
> ★ 注入阈值实测 ≈6.3k 字符，超出会被**静默截断**。**本文件必须 < 6.3k**，超了就往 playbook 搬。

## 一、铁律（读这一节就能不犯错）

- **真值只有一份**：`robot-package/<id>/model/robot.yaml`（运动学/标定/限位/零位）
  + `physics/physics.yaml`（**只放物理量，全 SI**）。**禁止硬编码尺寸/角度/限位**。
- **路径只在 manifest 声明、只由 `declared_path()` 解析**。新增路径字段必须登记。
  ★ 动机：搬迁时"自己拼路径的读者"漏改**不一定报错** —— 可能解析到一个**仍然存在的**同名文件。
- **真值 + 黄金数据都已冻结**（语义核心哈希，非整文件，D55/D71）。
  改真值 → 照抄 **D80 连锁清单**（`playbook.md` §13）：
  重生成产物 → `freeze_baseline --update` → 重采集黄金数据 → 修写死旧值的断言 → 同步文档。
  ⚠️ 改方向类标定（`reverse`）时，**逐处检查"越界钳位"断言的钳位端是否翻转**。
- ★★ **读复杂标准格式一律用参考实现，禁止自研解析器**。
  自研工具给出**否定性结论**（"文件坏了"/"数据缺"）时，**必须先用参考实现复核** ——
  否则会把**工具的能力边界**误报成**数据的缺陷**。（STEP 0/241 误判复盘：`playbook.md` §10）
- ★★ **诊断工具会自己误报**：脚本报"缺 221 条/全帧无 ACK"，固件计数器实测却是 240/240、`rx_drop=0`。
  **否定"链路丢字节"只能靠独立计数器，不能用收发对账**。（4 个陷阱：`playbook.md` §12）
- **STEP = 单姿态快照**：可作**外观**源，**禁止**作运动学真值源（`length` 唯一路径是标尺实拍）。
- **`mode`（simulation/real）不是 UI 开关**，取值须校验全通过才改（D41/D43）。
- **机器人相关状态一律进 store**，组件不持局部副本。
- **多机器人**：`config/robots.yaml` 只放指针 → `RobotRegistry` 唯一分派表（**禁止 `if robot ==`**）；
  SO-ARM101 资产**逐字节原样、禁止改**。
- **改真值后必重生成 MJCF/URDF**（皆为产物，`test_generated_mjcf_is_in_sync_with_config` 盯）。
- ★ **`ik.ts` 的"禁止改动"有唯一豁免口**：新增**可选参数且缺省语义不变**（opt-in）。
  范例：`IkOptions.reachMaxMm?`（球壳外径覆写）。判据：`undefined` ⇒ 逐位一致
  （必须用 sim2sim「抽象层残差差 0.000e+0」证明），传值 ⇒ 只影响 `cosAlpha` 的臂展。
  ⚠️ **要"参与计算"就必须改 `ik.ts`**：外层拦一道只能"拒绝更早"，不能"按新臂展求解"。
- ★★ **同一个量只能有一套口径** —— 宁可**扩 `Core→包` 的边**（登记进
  `corePackageBoundary.test.ts` 的 ALLOWED），也**不要为了"少 import"而自己重算真值**。
  实例：`robotStore.wristDistance()` 自己算腕枢轴距离 `d`，把 `radial` 取成
  `wrist.parentLink`（`tcp.joint='tool'` 的 parentLink 是 **`forearm_link` 80**，
  不是 `tool_link` 40）⇒ 与 `ik.ts` 的 `toolOffset[0]=40` 口径不符，
  同一点算出 **92.159 vs 111.542（差 19.4mm）且不报错**，
  症状是"改小外径后本该可达的点被判越界"、且**内圈被误报为可达**。
  ⇒ 现改为 `ikGeometry()` + `wristSagittal()` 转发。
- ⚠️ **在测试 helper 里发现的口径坑，必须立刻回头 grep 生产代码有没有同款。**
  （上面那个坑我上一轮先在测试 helper 里踩到并修了，却没查生产代码，于是它多活了一轮。）
- ⚠️ **几何量极易算错的证据**：`ik.ts` 专门有 `requireConstantToolOffset()`
  用**三个姿态交叉验证**那个偏移常量 —— 说明它本来就容易算错，更不该有第二处重算。

## 二、验收

**完整命令见 `playbook.md` §11**。实测基线（2026-09-15）：
`pytest -q` **198** · `test_baseline_frozen` **11** · `validate --all` ✓ ·
前端 tsc 0 / vitest **401** · `go test ./...` 全绿 ·
固件（AVR）FLASH **12848 B** / RAM **844 B**，零警告。

- ⚠️ **`pytest` 只在仓库根成立**：从 `frontend/` 跑会输出 `no tests collected` **且 exit 0**
  ⇒ 只看退出码的脚本会读成"通过"。**不要**给 pytest 传目录参数（传了就绕过 `testpaths`）。
- ⚠️ **限位端点不可精确到达**（整数度量化后可能落到限位外被拒 `ERR JOINT`）⇒ 取**限额内部**的值。
- ★ **真机验收容易自欺**：链路层全绿只证明"指令被固件接受"，**不证明物理到位**
  （真机无位置反馈，`joint_state` 是**开环目标值**）。物理证据只有相机。
- 环境硬约束、沙箱陷阱、`start.bat` 陈旧闸门 ⇒ `playbook.md` §5 · §9。

## 三、细节入口（按需读）

| 主题 | 去处 |
|---|---|
| §1 测试/探针纪律 · §2 机制与关节判定（轴/nq=5/绝对角/斜合法域） | `playbook.md` §1 · §2 |
| §3 外观·纹理轨（D56–D69） · §4 测量方法论与能力边界 | `playbook.md` §3 · §4 |
| §5 运行环境与 Windows/沙箱陷阱 · §6 协作约定与真机链路 | `playbook.md` §5 · §6 |
| §7 多机器人轨 · §8 Robot Package 重构边界与搬迁纪律 | `playbook.md` §7 · §8 · `docs/architecture/robot-package-phase*.md` |
| §9 `start.bat` + 真机验收两条证据链 · §11 验收七件套全文 | `playbook.md` §9 · §11 |
| §10 CAD/STEP 接入（OCCT 铁律/平行四连杆实证） | `playbook.md` §10 · `docs/STEP_KINEMATICS_VALIDATION.md` |
| §12 固件链路诊断（独立计数器 / 复位吞命令双窗口 / `STATS`） | `playbook.md` §12 |
| §13 改真值的连锁清单（D80 实践，照抄顺序） | `playbook.md` §13 |
| 冻结/基线/黄金数据决策理由 | `docs/decisions.md` D55 / D71–D73 |
| 多机器人统一验收 / 运行期切换 | ADR D77–D79 |
| 首帧"重影" / 幽灵臂渲染（D74） | ADR **D74** · `core/tools/park_sim_pose.mjs` |
| 末端目标「安全参数」（容差/内径/外径覆写 · 为何无限位调节 · 为何进 ik.ts） | **`docs/target-guard-analysis.md` §7** |
| 未修的无关问题 F1–F9 | `docs/architecture/mearm-v1-followups.md` |

## 四、skill 分层约定（`~/.workbuddy/skills/`）

- **`description` 是常驻注入**（每个 skill 每条会话都进上下文），**正文是按需加载**。
  ⇒ **description 只写「路由判据」，不写目录**。目标 ≤500 字符；写了目录就是反模式。
- **正文 < 11k 字符**；超了就把细节按节拆进 `references/*.md`，正文只留核心 + 指针表。
- **拆分纪律**：reference 只把标题降一级（`##`→`#`），**正文逐字节不变**；
  拆完必须跑两项校验 —— ① 代码块围栏数原文 == 新正文 + refs 之和；
  ② 原文每个**非标题行**都能在「新正文 ∪ refs」里找到；再查**引用双向**（无死链、无孤儿文件）。
- **description 必须是单行 plain scalar**。禁用 YAML 折叠式（`description: >`）—— 折叠块
  实际**保留换行**（`yaml.safe_load` 后 `"\n" in value`），而 description 是常驻注入，
  且**裸 `:` 后接空格会直接让 frontmatter 解析失败**（实例：`webgl-first-frame-forensics`
  的 `Triggers: ...` 未加引号 ⇒ `mapping values are not allowed here`，只在"改写成单行"时才暴露）。
  ⇒ 含 `: ` 或 `#` 的值一律**用引号包住**。
- ★ **计量 description 必须用真 YAML 解析器**（`yaml.safe_load`），**不能用正则取首行** ——
  正则会把折叠式的后续行漏掉，本会话实测因此把总量 **10145 少报成 8752（漏 1377）**，
  还误判 4 个 skill 为"description 仅 1 字符"。
- ★ **拆分后必须补一段 refs 索引**（正文末尾「本 skill 的参考文件」指针表）。
  本会话实测：拆完 22 个 refs 文件，其中 **14 个曾是"孤儿"**（孤立文件存在但正文零引用），
  纯靠"围栏守恒 + 零行丢失"两项校验**查不出来** —— 必须显式跑引用双向。
- ★ 本项目 5 个重 skill 曾按此拆过（2026-09-15）；**全库 31 个 skill 的归一化在 2026-09-16 完成**
  （30 个正文全部 < 11k，description 全部单行 ≤500）。注意：`~/.workbuddy/skills/` **不在 git 仓库内**，
  「同步」= 写进本记忆 + 落当日日志，**不是 commit**。

