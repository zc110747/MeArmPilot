## §1 测试与探针纪律

- **测试必须能红**：新加的守卫要么先构造一个会失败的场景验证它能红，要么等于没加。
- ★★ **`TransportStats.moving` 不能用来判"卡死"**（D44）：`moving = lagDeg > arrivedEps()`，
  它是 **lag 的同义重写** ⇒ `stalled` **永远不可能**出现。判据必须自己从**时间序列**得出：
  只有「命令已静止 ≥ `STALL_HOLD_MS`(700ms)」**且**「趋势不再下降（flat/growing）」才判 `stalled`；
  `unknown`（样本不足）/ `shrinking`（仍在收敛）一律 `tracking` ——
  **宁可提示"还在追"，也不误报异常**。
- **趋势判定用四分位中位数**，不用首末值/均值：回推序列混有掉帧与量化台阶，
  **个别异常帧就能把结论翻面**，而"误报卡死"会让人去查并不存在的问题。
- **恒假的检查比没有检查更危险**（D65）：`netstat | grep -E "..." | grep -E ":(5173|5273|8090)\b"`
  两个独立缺陷叠在一行（**漏 `-E`** ⇒ `(` `|` `)` 是字面字符 ⇒ 正则**恒不命中**；
  补 `-E` 后 **`\b` 紧跟 `)`** 在 GNU grep 3.0 下失效）⇒ 预检**恒返回"干净"**，
  把"我没查"伪装成"我查过了"。可靠写法：字段级 `awk '$4=="LISTENING"{...}'`。
- **复用真机后端会把「前提不成立」读成「代码回归」**（D65/D40/D43）：
  8090 上有上一轮遗留的 `device=serial` 后端时，"**允许**切换 Real Robot"才是**正确行为**。
  这是**环境差异**，不是回归 ⇒ 每次 e2e 前必须确认后端是**干净 sim**。
- **探针的两类静默失真**（D67）：
  ① **读错表** —— `.sidebar table.grid` 不止一张（`ConnectionControl` 的"指标/值"表排在
  `StatusPanel` 关节表**之前**），全局 `querySelectorAll` 会命中错的那张 ⇒ 必须**限定作用域**；
  ② **滞后读数拆成多次 CDP 往返** —— 慢环境里读到的是**已收敛值**，让"滞后瞬间"断言假 FAIL
  ⇒ 一次往返取齐所有量。
  ★ 另加 `start.bat` 的**残留进程对**问题：残留后端会让页面 Actual 由 WS 回推驱动。
- **e2e 必须隔离端口**（5276 + 8091），起服务与跑 e2e **必须在同一次工具调用里**
  （`(cmd &)` 后台进程只活到本次调用结束）。
- **`mode` 跨批次残留**：上一批点过 Real Robot 会带进下一批 ⇒ 每批开头显式复位。

## §2 机制与关节判定速查

> **状态管理铁律**：**机器人相关状态一律进 store，组件不持局部副本。**
> 包括 `teachTrack`（示教轨迹）这类看起来"只属于某个面板"的运行时状态 ——
> 组件持副本会导致 UI 与真机/仿真状态分叉，且切换机器人时不重置。

**关节轴**（D2，有意改写 spec 示例 yaml 的 axis 字段）：
`base = [0,0,1]`（绕竖直轴偏航）· `shoulder/elbow/tool = [0,1,0]`（XZ 平面内俯仰）·
`gripper = [1,0,0]`（爪沿 ±Y 分开）。
⚠️ spec §八 示例给的是 `base=[0,1,0]` / `shoulder,elbow=[0,0,1]`，与 §九「Z：上下」**矛盾** ——
按后者取值才得到竖直平面内的 2R 机构；照前者是**水平 SCARA**，与 meArm 完全不符。

**`nq = 5`**：`base / shoulder / elbow / tool / gripper`。其中 **`tool` 是 passive**
（`type: passive`，`limit.min === limit.max`）。⚠️ 它**不进入** JointState / UI 滑杆 / JR 协议
⇒ JR 仍是**四元组**，`docs/serial-v1.md` 与固件**一个字都不用改**（D70）。

**绝对角 vs 局部角**（最关键的一条）：
- `elbow` 存的是**绝对倾角**（θ=0 指天顶），真机由独立舵机 S8 经**平行四连杆**驱动
  ⇒ 与肩角**解耦**，用 `coupling: {joint: shoulder, gain: -1}` 表达（D16）。
  串联网里的**局部旋转** = `112.62 + (−1) × shoulder`。
- `tool` 是**被动关节**（D70）：爪的绝对倾角同样被连杆锁住（**近似恒水平**），
  `limit = 90..90` + `coupling{gain:-1}` 到 `elbow` ⇒ 爪绝对倾角 ≡ 90°，
  **与 J2/J3 都无关**。
- ⚠️ **`elbow` 的合法域是「斜的」**（D49）：它与 shoulder 耦合，而 **MuJoCo hinge `range`
  只能表达轴对齐的盒** ⇒ 没有任何单一 `range` 能表达它。
  - 局部角外接区间 = `[108.4415−49.4549, 141.8582+6.0937] = [58.9866, 147.9519]`
  - 取下界配肩角下界：`θe = 58.9866 + (−6.0937) = 52.89°` < 108.4415° ✗ **越界**
  - 求"内切"：下界 114.5352 > 上界 92.4033 ⇒ **空集**
  ⇒ **若只靠 range，MuJoCo 会照常执行 `(shoulder=−6°, elbow=53°)`**（它只看到局部角 59° 在盒内），
  而真机 S8 结构上**转不到**那个绝对角。
  **架构**：`robot.yaml` = **唯一限位真值**；`physics.yaml` 的 `range_padding_deg` 只承担
  **数值保护**（防止积分器发散），**不承担语义**。
- **`link.length` 是机构尺寸的唯一真值**（D1）：一根连杆沿**其近端关节坐标系**的 +Z 伸展
  `length`，末端即下一关节坐标系原点 ⇒ **改 `length` 即改机构尺寸**，无需同步改任何关节字段。
- ⚠️ **改 `length`/轴限位/耦合/标定 ⇒ 报错**（冻结基线语义核心哈希，D55）；
  改 `links[].geometry`/`details` ⇒ **放行**（纯外观）。

## §3 外观 / 纹理轨（D56–D69）· 速查

- ★★ **PIL 的 `Image.transform(QUAD)` 不能用于透视校正**（D56）—— 它引入 **+12~17px 的
  静默平移**（刻度线真值 `[185,370,554]` → `[197,387,567]`；而四角估计误差只有 `1.00px`）。
  用 `make_texture.py` 的自实现 `homography` 重采样（同四角同 round-trip ⇒ **精确命中**）。
- ★★ **近黑照片纹理「看不见」的根因是 8bit 量化，不是曝光**（D64）：
  照片按**白色桌面**曝光 ⇒ 黑亚克力落在 **1~3 码值**（`upper_arm_link` ≤3 占 **72.2%**、
  `forearm_link` **77.0%**），经 three 的 sRGB→线性（`sRGB(2) → 0.0006`）⇒ 渲染 `L≈0.08`。
  **七成以上像素在量化台阶上，提亮只是把台阶一起放大**（推到 L≈60 需 **+6.2 EV**，
  那时板面摊成 41/60/74 **三级平台**，比纯黑更假）。
  **解法是逐材质 `envMap`**，⚠️ **不要**改回 `scene.environment`（全局的，实测把底座蓝板点亮 ×3.3757；
  且 three 在 `envMap===null && scene.environment!==null` 时**用 scene.environmentIntensity
  覆盖 material.envMapIntensity** ⇒ 逐材质"关环境反射"在全局方案下**根本不生效**）。
  ⚠️ **真正能把板面细节送进数据的是拍摄端**（对板测光/补光重拍）。
- ★ **局部轴 ≠ 世界轴**（D57）：plate 的**大面法向 = `size` 里最小那一维**，但"局部薄轴"
  **不能**直接当"世界方向"写进指南 —— 曾有 2/6 张据此拍错。
  摆动平面 = X–Z（`base` 轴 = 世界 Z，`shoulder`/`elbow` 轴 = 世界 Y）⇒ **相机在 −Y 一侧**；
  ⚠️ `tool_link` 局部 Z → 世界 X。
  ⇒ **别背轴向，拿实物转一圈找面积最大的面。**
- ★ **采集判定「说不该说的话」比不量不说更贵**（D57）：产物是**给操作者的行动指令**，
  指令错了**不会报错** —— 人会照着错的指令调很久也调不好。
- **提高分辨率只有物理靠近一条路**（D58）：`base` 旋转是**零和**（远端 ×1.08 但板法向偏 20°
  ⇒ 投影缩短 `cos20°=0.94` ⇒ 净 1.015，还白搭透视）；肩/肘摆动**改变不了 `px/mm`**
  （摆动平面 X–Z **平行于像平面**）。
  ⚠️ **"自动分割出单块板"不可行**：板/立柱/底座/舵机/控制板**同一种黑**且**由螺栓物理相连**
  ⇒ **不存在"只含一块板"的连通域**（暗像素占整帧 25.7%，左下象限高达 65%）。
- **「带区间的搜索」必须显式检测最优解是否贴边**（D59）：最优落在区间端点 2% 以内
  ⇒ **该值不是测量结果，是区间的人为截断**（`s = 3.80` 就是这么来的）。
  **两个读数矛盾时，先查区间是不是设窄了，别急着改模型。**
- **纯几何推理走不通时改用「致动并观察」（motion-diff）**（D59）：动一个关节 → 拍前后两帧 → 差分
  ⇒ **差分区域 = 该关节及其下游零件**，不依赖任何相机/位姿假设。
  ⚠️ 但**给不出干净的单板四角** ⇒ 只做归属判定 + 粗定位。
- ⚠️⚠️ **`mujoco.Renderer(model, height, width)` —— 顺序是 (height, width)**（D61）：
  传反**不报错、不告警**，却让**所有目视结论都错** ⇒ **工具调用的参数语义必须先验证。**
- ★ **三态 `ok` / `reject` / `undecidable`；「判不了」≠「不合格」**（D61/D57）
  ⇒ 此时**禁止**输出"像结论"的数字。
- **孔洞掩膜**（D62）：实板**镂空** vs 模型**实心 box** ⇒ 把非板面区域填成**板面色**。
- **前端接入照片纹理按「面材质数组」**（D63）：一条被 `flipY` 掀翻的朝向推理 ——
  用**面材质数组**而不是翻转贴图。
- **夹爪按实拍照片反解平面轮廓**（D66）：`type: jaw` 一条连续闭合折线 +
  三条由几何自洽给出的派生关系。形体特征：根部**整圈方齿齿轮盘**（齿轮直径/爪长 = 246/359 = **0.69**，
  模型 24/34 = 0.71）· **两片爪齿轮互相啮合** · 内侧缘约 **9 个浅密锯齿** ·
  外侧缘出盘后**先收窄一次**（"脖子"约在 1/3 处）· 末端**斜切收尖**。
- **视口**（D68）：产品渲染式灰底 + 板件统一**近黑** + 世界轴**默认关**。
- **「空洞」补料**（D69）：小臂侧板到腕点缺 **12.5mm** 料 ⇒ 用**沿臂推导覆盖区间**定位 +
  用**无贴图 `box` 补料**填充（**不是**拉伸主板）。
  ⚠️ 必须追加到 `details` **末尾**：`load_plate_sizes` 用 `enumerate(details)` 的**下标**做别名
  （`forearm_brace` = index 3），插中间会把后续下标整体挤错。
环境纹理由程序化 RoomEnvironment + PMREM 生成，不依赖任何外部 HDR 资产（实测：只作用于 `plateTexture` 材质，底座蓝板 ×1.0000）。
- 几何铁律：plate 大面法向 = `size` 里**最小那一维**；摆动平面 = X–Z（`base` 轴 = 世界 Z，
  `shoulder`/`elbow` 轴 = 世界 Y）⇒ **相机在 −Y 一侧**。⚠️ **局部轴 ≠ 世界轴**（`tool_link` 局部 Z → 世界 X）
  ⇒ **别背轴向，拿实物转一圈找面积最大的面。**
- ⚠️⚠️ **`mujoco.Renderer(model, height, width)` —— 顺序是 (height, width)**（D61 一）。
  传反不报错、不告警，却让**所有目视结论都错** ⇒ **工具调用的参数语义必须先验证。**
- ★ **三态 `ok` / `reject` / `undecidable`；「判不了」≠「不合格」** ⇒ 此时**禁止**输出"像结论"的数字。
  ⚠️ **禁用 PIL `Image.transform(QUAD)`**（+12~17px **静默**平移），用 `make_texture.py` 的 `homography`。

## §4 测量方法论与能力边界（只留结论）

- **单批"最优" = 过拟合**：定 ROI / 阈值 / 骨架模型**必须跨 ≥2 批验证**。
- **"改模型让它对上"与"调参数让它对上"是同一类错误**：必须先证**测量可复现**
  （`tools/verify_calib_repro.py`：增益跨批极差 ≤5%），才有资格改 `robot.yaml`。**一次只改一条**，改完重跑判据。
- ★★ **带区间的搜索必须显式检测"最优解是否贴边"**（D59 ①）：最优落在区间端点 2% 以内
  ⇒ **该值不是测量结果，是区间的人为截断**。**两个读数互相矛盾时，先查区间是不是设窄了，别急着改模型。**
- ★★ **纯几何推理走不通时，改用「致动并观察」（motion-diff）**（D59 ④）：动一个关节 → 拍前后两帧 → 差分
  ⇒ **差分区域 = 该关节及其下游零件**，不依赖任何相机/位姿假设。⚠️ 但**给不出干净的单板四角** ⇒ 只做归属判定 + 粗定位。
- 量包围盒必须**目视复核**（线缆 / 桌沿会污染 bbox）；**手入镜 / 相机位移**是头号污染源。
  ★ **固定 ROI 只在同一机位下可比**（D64 七）：换机位必须重定 ROI 并**目视复核 ROI 在物体上**；
  量一个小面时不要用"掩膜 bbox 画矩形"（会把**确实该变**的邻件圈进来）⇒ 在**基线图算出的固定像素集合**上量。
- **MG90S 无位置回读**：`OK SET` / `STATUS` / 后端 `joint_state` **全都在说"我打算去哪"**，
  没有一条能证明"它实际上在哪"；唯一外部地面真值是**相机**（D34）。
  ★ **「无位置回读」≠「没动作」**：只说明**不能证明"到位"**，**不等于"没动"**。
  Phase 11 误差面板反映的是**链路时延与限位截断**，不是"物理臂到位没有"。
- 当前状态：A1（骨架双杆）/ A2（`coupling.gain` → −0.81）**均判定为不改**；
  唯一入口是 A3 台面重测（`docs/hardware-measurement.md` §7）。

## §5 运行环境与 Windows 陷阱

| 用途 | 位置 / 版本 |
|---|---|
| 前端 | `frontend/node_modules`（`npm install` 产物，**不入库**）· Node 22.22.2（managed）· vite 8 · vitest 5 · tsc 5.9 |
| 后端 | Go 1.27 → `backend/bin/armpilot-backend.exe`（`/backend/bin/` 已 gitignore）· module `armpilot/backend`，唯一外部依赖 `gopkg.in/yaml.v3 v3.0.1` |
| Python | `~/.workbuddy/binaries/python/envs/default` · py3.13 · numpy 2.5 · Pillow 12.3 · pyyaml 6.0 · **mujoco 3.13** · pytest 9.1 · pyserial 3.5 |

- ★ **`backend/go.mod` 曾根本不存在**：父仓 `D:/user_project/git/ArmPilot/.gitignore` 里那条 `*.mod*`
  （从 Linux 内核模板抄来）把 `go.mod` 一起吞了 ⇒ `go build` 报 `cannot find main module`。已加例外 `!go.mod`
  （注意 `*.mod` / `*.mod*` 都必然匹配 `go.mod`，只能靠负向规则救）。**`MeArm-RemoteControl` 是同一个坑**
  （其 `go.mod` 至今未入库，followups **F1**）⇒ 见 `cannot find main module`，先跑
  `git check-ignore -v <path>/go.mod`，别急着 `go mod init`。
- ★★ **裸名 `bash` 解析到 WSL 启动器，会被沙箱拦**（2026-09-14 实测）：`which -a bash` 首项是
  `/tmp/system32/bash` = `C:\Windows\System32\bash.exe` ⇒ `bash x.sh` 直接报
  `PROGRAM BLOCKED BY SECURITY POLICY … wsl.exe`，**且 stdout 被整段丢弃**（表现为"脚本什么都没输出"，
  极易误判成脚本自身的问题）。**跑脚本一律 `/usr/bin/bash x.sh` 或 `sh x.sh`。**
  ⚠️ 脚本 shebang 写 `#!/usr/bin/env bash` 也会踩同一个坑 ⇒ 写 `#!/usr/bin/bash`。
- ★★ **`./node_modules/.bin/vite` 同样会被拦**（npm 生成的 shim 先 `cygpath -w "$basedir"`，
  再 `exec node "D:\…\node_modules\.bin/../vite/bin/vite.js"`，反斜杠路径走歪）⇒
  **`node node_modules/vite/bin/vite.js …`**。`vitest` 的 shim 不受影响（同上位脚本实测可用）。
- ★★ **本机 `sort` 解析到 Windows `System32\sort.exe`**（不认 `-u`）⇒ `... | sort -u` **静默返回空**，
  表现为"按 PID 清理进程"的循环一个都没杀。去重用 `awk '!seen[$0]++'`，或走 `/usr/bin/sort`。
  **与 §1 的 `grep \b`、`taskkill //PID` 是同一类坑：工具语义没验证。**
- `backend/config.yaml → device.mujoco.python` 是**本机绝对路径**，换机器必改（followups **F5**）。
- ★★ **`git push` 在本沙箱"推送已生效，但退出非 0 / 或干脆不退出"**（2026-09-14 实测两次）：
  stderr 末尾是 `fatal: unable to write credential store: Permission denied` +
  `[sandbox] 命令被沙箱拦截 … C:\Users\lx176\.git-credentials (写 · 剥写)`，
  但**上一行已经打印了 `6981c77..d98f7ea  Develop -> Develop`** ⇒ **推送其实成功了**，
  挂掉的只是"把凭据回写缓存"这一步（另一次同场景表现为进程 6 分钟无输出被转后台，
  `GIT_TERMINAL_PROMPT=0` 拦不住）。
  ⇒ **判定推送是否成功只认 `git ls-remote origin refs/heads/<branch>` 与本地 HEAD 比对**，
  **不要看退出码**；非 0 时先读 stderr 最后一行，分辨"真失败"与"收尾被拦"。
  连通性自检：`curl -s -o /dev/null -w "%{http_code}" --max-time 8 https://github.com`（返回 200 即网络正常，
  curl 自身 exit 23 是沙箱写 `/dev/null` 被拦，**不是网络故障**）。
- `start.bat` 前置检查只依赖：`backend/bin/armpilot-backend.exe` · `backend/config.yaml` ·
  `config/robots.yaml`（**注意：只有 `robots.yaml`，旧的 `config/robot.yaml` 已随选择器化废弃**）·
  PATH 上的 `node` · `frontend/node_modules/.bin/vite.cmd`。
  ★ `[0/3]` 之前的**陈旧闸门**会在 `.go` 源比 exe 新时自动 `go build`；launch 后还会再探一次 `/healthz`。
  两者都存在的原因：**"启动了"和"真的在服务"是两个命题**（详见 §9）。
- ★★ **本机 `grep`（msys2 `GNU grep 3.0`）里 `\[` / `\]` 不等于字面方括号**（2026-09-14 实测）：
  `grep -c '\[0/3\]' start.bat` 返回 **59**（= 文件里含 `0`/`/`/`3` 的**行数**，说明 `\[…\]` 被当成
  **方括号表达式**），而该串真实出现 **1** 次；`grep -cF '[0/3]'` 与 Python `str.count()` 都返回 1。
  用 `\|` 把两个模式或起来更糟：`grep -c "robot-package\^)\|\[0/3\]"` 返回 **0**
  （而两个模式**都确实在文件里**）——这是本轮唯一一次把工具 bug 误读成"仓库被回退"的源头。
  ⇒ **数精确字符串一律 `grep -cF`，或直接 Python `str.count()`**；先拿"已知答案的探针文件"
  验一次工具语义（与 §1 的 `grep \b`、§5 的 `sort -u` 是同一类坑：**工具语义没验证就下结论**）。
  > 假警报的完整还原：`grep` 报 0 → 我读成"修复被 `checkout` 回退"，于是去查 reflog/index mtime。
  > 真相是：`git diff -- MeArm-3D/start.bat` **为空**（工作区 == 暂存区**内容一致**），
  > 313/343 字节差 = 343 行 × 1 字节 = **纯 CRLF 行尾**，属正常；
  > 工作区与暂存区都含 `robot-package^)` ×1、`[0/3]` ×1、`STALE` ×3、`healthz` ×5。
  > **教训：判"文件被回退"用 `git diff` + 逐字节哈希，不要用 `grep` 的计数。**

## §6 协作约定与真机链路

- `git push` **默认由用户自行执行**；**用户显式要求时可代推**（2026-09-14 起，用户明确下达过一次）。
  ⚠️ 推送的判定与坑见 §5「push 成功但不退出」。
- **破坏性操作先列清单确认**；建新目录先跑 `git check-ignore -v <path>/probe.txt` 探针。
- 每轮收尾：README 阶段表/§6 + `docs/decisions.md` ADR + memory **同步更新**。
- 提交按逻辑拆分（freeze / refactor / test 各自独立），不混在一起。
- **真机链路**：`backend/bin/armpilot-backend.exe -c config.serial.yaml` · **CH340 @115200 8N1** ·
  ★ **COM 号不是常量**，它会随 USB 枚举变：2026-09-14 实测 = **`COM18`**（此前 `COM16`，已成
  `Present=False` 的幽灵条目）。**唯一改动点 = `config.serial.yaml` 的 `device.serial.port`**，
  别信记忆/文档里的旧值 —— 先 `serial.tools.list_ports.comports()` 看一眼实际是谁。
  开机四舵机全 90°(= HOME)。调试入口 `core/tools/set_joints.mjs`（`--status`/`--home`/`name=value`），
  **别拿验收脚本当摇杆**。
  - ★ `hello` **没有** `connected` 字段（`docs/serial-v1.md` §5.1 属**文档漂移**，followups **F4**）
    ⇒ 判真机只能看 `hello.device === 'serial'`。
  - ★ `hello` 与 `joint_state` 同时到达且 `hello` 在前 ⇒ 发送点必须在收齐 `joint_state` 之后，
    否则"保持不变"的关节会 fallback 到 `homePose` = **把臂拉回 home**。`--settle` 的"保持不变"
    会让状态**跨实验累积** ⇒ 对比实验必须先 `--home`。
  - ★ 打开串口会拉低 DTR 复位 ATmega328P（舵机弹回 90°）⇒ 实测脚本必须在**单次连接**内完成
    `[set → 稳定 → 抓拍]`。

---

## §7 多机器人轨（SO-ARM101）· 速查（2026-09-14 起）

**通用层**（与具体机器人无关，MeArm / SO-101 共用）：

- 选择链：`config/robots.yaml`（**只**放 id/name/config，禁止放参数）
  → `model/robotConfigRegistry.ts`（`import.meta.glob` 构建期登记 yaml 原文）
  → `model/loadRobotModel(id?)`（按 id 缓存；未知 id **抛错不回退**）
  → `registry/RobotRegistry.ts`（**唯一**分派表：工厂表 + `assertRegistryCoverage()`）。
- 业务代码**禁止** `if robot === ...`；"谁有 IK"由 `kinematics.capability` **声明**（数据）。
- 叶子模块（防循环导入）：`model/configError.ts`（`RobotConfigError`，`source` 可传文件标签）、
  `model/robotIds.ts`（`MEARM_V1_ROBOT_ID` / `SO_ARM101_ROBOT_ID`）。
- `Actuator.unit`：`'deg'`（缺省，0..180 舵机行程）/ `'joint'`（关节空间，ctrlrange ≡ 关节 range）。
  `ACTUATOR_LIMIT_180` 只在**非**关节空间生效。
- `RotationConvention`：`'xyz'`（缺省，intrinsic，`Rx·Ry·Rz`）/ `'rpy'`（URDF `<origin rpy>`，
  fixed-axis，`Rz·Ry·Rx`；three.js 侧用 Euler order `'ZYX'`，**数值原样只换 order**）。
- mesh：`model/meshRegistry.ts` 只登记 **URL**（`?url` + eager）；渲染在
  `RobotScene/meshObject.ts`，两条守卫（无 DOM 不加载 / key 未登记则 warnOnce + 回退）。

**SO-ARM101 专属**：

- 官方资产 `assets/models/so-arm101/official/`（**逐字节原样，禁止改**）；配置在
  `config/robots/so-arm101/{robot.yaml,physics.yaml}`；生成器 `tools/gen_so_arm101_robot_yaml.py`。
- ★ **物理量真值 = 官方 MJCF** ⇒ `physics.yaml` **不复制任何数值**，只放
  真值声明 + 驱动参数 + 审计快照 + **官方未声明项**。
  `tools/inspect_so101_physics.py --check` 从 **MjModel** 读生效值（**不是读 XML 文本** ——
  XML 里 class 写的 `forcerange` 是 ±2.94，被 6 个 `<position>` 逐个覆盖成 ±3.35）。
- ★ **两条 90° / 精度陷阱**：TCP 帧朝向取 **MJCF**（URDF `rpy=[0,π,0]` vs MJCF `quat=Ry(π/2)`）；
  关节限位取 **MJCF 满精度**（URDF 截断到 6 位小数）。理由见 `SOURCE.md §4.2`。
- ★ **FK↔MuJoCo 残差**的根因是**官方两份文件自身的精度差**（URDF `rpy` 截断到 6 位有效数字
  `1.5708`≠π/2；MJCF 四元数归一化后恰好 90°），**不是换算错误**。
  实测冻结值：MeArm **7.7e-14 mm** / SO-101 **3.3e-3 mm**。
- **能力声明**：`{positioningDof: 5, supportsOrientation: false, solverKind: 'none'}`；
  `inverse()` → `ikFailure('NOT_IMPLEMENTED')`（`success:false` / `joints:{}` /
  `positionError:null`）。**禁止**抄 MeArm 的平面 2R 解或塞数值解 —— 见 spec「不伪造」。
- 官方**未声明**（别以为有）：无 floor/table、**无 `<contact><exclude>`**（相邻连杆会互相碰撞）、
  无关节速度上限、无独立标定段、质量来自 CAD 非称重。
- 执行器：官方 6 个 `<position>`、`gear=1` ⇒ **无 offset/scale/reverse 标定**（`unit: joint`）。

**P3'–P8 落地事实**（2026-09-14 完成，实测数据见 `README.md` §6）：

- **三端同源**：`config/robots.yaml` 由**前端**（`import.meta.glob`）、**Go**（`internal/robot.LoadSelector`
  /`LoadByID`）、**Python**（`robotcfg.py`）**各自解析同一份文件** —— 三端都有 `resolve_robot_entry_by_config()`。
- ★★ **选择器只放"指针"**：允许 `name` / `config` / `physics` / `simulation.mjcf` / `simulation.tcpSite`，
  顶层只允许 `version` / `default` / `robots`；**尺寸、限位、标定、物理量、TCP 偏移一律禁止**。
  用**白名单 schema 断言**守（Go `TestSelectorSchemaAllowsPointersOnly`），不是靠自觉。
- ★★ **注册表 id ≠ 模型 id**：选择器 key `mearm-v1`，而 `robot.yaml → robot.id` 是 `mearm`
  ⇒ **不能用 `robot.id` 反查注册表**；`SO_ARM101_ROBOT_ID = 'so-arm101'`（注意是 `so-arm101` 不是 `so-arm101-*`）。
- ★ **`physics.yaml` 两种形态，判据是"顶层有没有 `driver:` 段"**（刻意不加配置项）：
  `legacy`（MeArm，顶层即驱动参数）/ `driver`（SO-101，物理量真值在官方 MJCF）。
  Go 侧告警**必须按 `physics_kind` 分支** —— 对官方模型说"惯量是估算值"是**错的告警**，
  比没有告警更糟（`backend/main.go`）。
- ★★ **`max_velocity` 优先级 = `rad_per_s` 先**：MeArm 同时写了 `deg_per_s: 573` 与 `rad_per_s: 10`，
  573°/s 换算回来 = 10.000737 rad/s ⇒ **取 `rad_per_s` 才是改前行为**。写反会静默改变运动速度。
- ★★ **timestep 不一致是静默错误**：物理时间只由 `model.opt.timestep` 决定（MeArm `0.001` / SO-101 `0.002`）
  ⇒ `RobotSim` 构造时硬自检，`server.run()` 再对 `--phys-hz` 告警"以 MJCF 为准"。
- **能力门必须落在真正下发命令的层**（D78）：`store.moveTo` 在 `solverKind !== 'analytic'` 时
  **不调求解器**，返回 `NO_SOLVER`。它与 `OUT_OF_WORKSPACE` / `JOINT_LIMIT` **并列而不合并** ——
  前者是"**没算**"，后两者是"**试过不行**"。合并会让用户以为要去调目标点。
  拒绝时仍写入 `target`，UI 才能说清"我想去哪"和"为什么没动"。
- **统一 Sim2Sim 只有一份判据**（D77）：`run_sim2sim(robot_id)`（`simulation/mujoco/sim2sim.py`）
  + CLI `tools/run_sim2sim.py --all` + `sim2sim_matrix()`（默认取选择器**全部**机器人）。
  **绝不**为第二台机器人另写一套验收（两套标准各自都会绿）。
- ★★ **容差按机器人登记、必须写明理由、未登记一律 `raise`**（不给缺省值）：
  `FK_TOL_MM = {mearm-v1: 1e-6(实测 7.7e-14，留 7 个数量级), so-arm101: 5e-2(实测 3.3e-3，留 1 个数量级)}`。
  悄悄给个"够大"的缺省 = 把"没人想过它的精度来源"藏起来，而那正是放宽阈值以通过测试的开端。
- ★ **交叉一致性**：统一框架的快照（`run_sim2sim.py` 采）与旧黄金基线（`gen_mearm_v1_baseline.py` 采）
  在两批**独立采集**产物的**同名用例**上必须逐位相同 —— 否则抽象过程会把基准期望值悄悄改掉。
- ★ **同名关节会骗人**：两台机器人**都有** `gripper`（MeArm `0..90` vs SO-101 `-10..100`）
  ⇒ "键集合相等"**不是**模型一致的判据，只有**限位**能兜住。切换后的断言应写
  "MeArm 独有键 `not.toHaveProperty`" + 专节声明同名不同义。
- **活动机器人是 store 状态**（D79）：初值取选择器 `default`；`setRobot` 有守卫
  （`transportDriven` 时拒切并说明"请先断开"）+ 未知 id **拒绝不回退**；
  `hello` 只做**在线互检**、**不静默切换**。★ 它**不是** `model.id`（见上）。
  运行期一律用 `get().model`，**不要再引用模块级 `initialModel`**。
- **测试落点**：`tests/sim` · `tests/sim2sim` · `backend/internal/robot` `go test`
  · 前端 `vitest`（含 `tests/acceptance/robot-switch.test.ts` + `tests/sim2sim/so-arm101-baseline.test.ts`）。
  基线快照：`robot-package/{mearm-v1,so-arm101}/tests/cases/sim2sim.json`（`--freeze`，`--n-random 24` 冻结）。

## §8 Robot Package 重构（Core / Robot Package / Working Robot，2026-09-14 起）

**目标形态**：`core/`（机制，**不含任何型号名**）· `robot-package/<id>/`（一台机器人的全部：manifest +
`model/` + `physics/` + `tests/cases/` + `tools/` + 资产）· `working-robot/`（"现在轮到谁"，构建产物，gitignore）。
spec = `docs/MeArm_3D_Prompter_06.md`（56 节）；只读分析 = `docs/architecture/robot-package-phase0.md`。

**文档落点（2026-09-14 定，用户拍板）**：串口 / WS 协议基线 = **`MeArm-3D/docs/serial-v1.md`**
（原 `MeArm-3D/protocol/serial-v1.md`，`protocol/` 目录已撤）。phase0 映射表原拟的 `core/protocol/`
**未采用**，该行已就地标注实际落点。全仓 51 处引用 + 2 处仓库树 + 1 处映射表已同步
（唯一故意不改：父仓 `docs/MeArm_3D_Prompter_01.md` 的**历史设计快照树**）。

**搬迁纪律（不可跳）**：一次只搬一个子系统 → `Build → Test → 验证 → 记录`，**每步交付一次实测**。
判据按子系统选：
- **配置真值**：`freeze_baseline.py --update` 后**语义核心哈希逐位不变**（证明"只挪了位置，一个字节没动"）。
- **黄金数据**：`gen_mearm_v1_baseline.py --check` 四份 `逐位一致`。
- **包契约**：`core/python/robopkg/cli.py validate --all`（选择器 / manifest / 真值 / 包目录 四方对账）。

### 本轮（Phase 2 步① ②）踩到的坑 —— 全是"静默"类

- ★★ **「编辑成功」≠「改到盘上了」**：`tools/gen_so_arm101_robot_yaml.py` 的 `OUT_PATH` 上一轮改过、
  **实际没落盘**。它不报"我还在用旧路径"，而是报 `缺少 config\robots\so-arm101\robot.yaml`
  —— 读起来像"文件丢了"。⇒ **每处编辑后用独立命令核对磁盘**（`grep -n` / `--check` / `Read`），
  同文件多处编辑尤其要复核。
- ★★ **`bash` 的 `cat >> file << EOF` 在本沙箱会写坏文件**（2026-09-14 实测）：追加 45 行，
  结果**文件头部被覆盖**（标题 / §1 / §2 标题丢失、尾部混入半行残片），而字节数**恰好不变**
  （22156 → 22156），`wc`/`ls` 完全看不出来。**唯一可靠的发现方式是 `Read` + `grep -n "^#"` 看章节**。
  ⇒ **追加长文本一律用 Edit 工具（带锚点）或 Write（整文件重写），并在写完后立刻读回核对**。
  ⇒ 记忆文件受 git 跟踪（仓库根在父目录 `ArmPilot/`，故 `git show HEAD:MeArm-3D/.workbuddy/memory/playbook.md`）
  ⇒ **写坏可直接 `git checkout --` 恢复**（本次即如此救回）。
- ★★ **"漏改的读者"清单要用全仓 grep 建立，不能靠记忆**：本轮漏掉的是
  ① 前端 `loadRobotModel.ts` 的 `import robotYamlText from '@config/robot.yaml?raw'`
  （Vite 的 `?raw` **必须静态** ⇒ 它是唯一绕开 Registry 自己拼路径的读者；一处漏改 →
  **29 个 vitest 红 + 13 个 pytest 红**，vitest 报 `ENOENT`、ikbridge 报 `Cannot find module`）；
  ② `tools/{verify_pose,fit_pose,make_texture}.py` 的 `YAML` / `ROBOT_YAML` 常量；
  ③ `tests/sim/test_simulation.py`（`ROOT/"config"/"physics.yaml"`）；
  ④ `tests/sim/test_baseline_frozen.py` 的 `fb.BASELINE`（`freeze_baseline` 函数化后属性名变了）。
  ⇒ **扫描词**：`"config"` · `config/robot.yaml` · `config/physics.yaml` · `config/robots/` ·
  `tests/baseline` · `baseline-kinematics-physics`。
- ★ **报错文案决定定位速度**：`freeze_baseline.check()` 原先在路径变更时抛 `KeyError: '…/model/robot.yaml'`
  —— 读起来像"工具坏了"。改成可读的"**键已过期**"并直接给出修法（`--update`）后，一眼归因。
- ★ **不要把当时的目录布局写进断言**：`robotRegistry.test.ts` 的 `config.startsWith('config/')`
  问的是"路径在不在 `config/` 下"，而它**想**问的是"是不是三端都能解析的仓库相对路径"。
  搬迁时改它**不是放宽判据**，但必须**同时加强** —— 本轮补了"选择器声明的路径 ≡ 该包 manifest 的
  `model.config` 声明（resolve 后是同一个文件）"，把"两处声明各自漂移"也钉住。
- ★ **用户可见字符串里的路径同样不许写死**：`backend/main.go` 的物理量告警改为取
  `entry.PhysicsPath` / `entry.ConfigPath`；`ik.ts` 抛错文案里的 `config/robot.yaml` 是**进生产包**的
  —— 靠 `grep -ho "config/robot.yaml\|config/physics.yaml" dist/assets/*.js | wc -l`（须为 0）才发现。
- ★ **Core 里不该留下"像是真值目录"的名字**：`robotcfg.py` 的 `CONFIG_DIR` 已删（它现在只服务选择器，
  留着这个名字会让人以为真值还在仓库根 `config/` 下）。
- **`robopkg` 是包不是模块** ⇒ `python -m robopkg` 不可用；入口是
  `python core/python/robopkg/cli.py {list|show|validate|hash|selftest}`（退出码 0/1，不吞错误）。

### Phase 2 步③ ④ ⑤（2026-09-14 收口）—— 三个真 bug + 两条测试通道

**搬迁结果**：仓库根 `tools/` 消失，24 个工具就位 —— `core/tools/`（8：`freeze_baseline.py` /
`run_sim2sim.py` / `park_sim_pose.mjs` / `set_joints.mjs` / `ws_probe.mjs` / `first_load_probe.mjs` /
`lan_e2e_probe.mjs` / `verify_serial_e2e.mjs`）· `robot-package/mearm-v1/tools/`（14）·
`robot-package/so-arm101/tools/`（2）。前端：`ik.ts` / `MeArmKinematics.ts` → 包内 `kinematics/`，
`RobotRegistry` 改 `import.meta.glob` **自动发现**。测试：按**断言**拆（机制留 `tests/sim/`，期望值随包）。

**搬迁揭穿的四个真 bug（全是"静默"类）**：

| # | 症状 | 根因 | 修法 |
|---|---|---|---|
| 1 | `run_sim2sim.py --help` → `ModuleNotFoundError: No module named 'sim2sim'` | `Path(__file__).resolve().parent.parent` 搬到 `core/tools/` 后指向 `core/` | `_find_repo_root()` |
| 2 | `freeze_baseline.py` 实跑报"基线文件不存在：`config\baseline-…json`" | 同上 + `BASELINE`/`ROBOT_YAML`/`PHYSICS_YAML` 三条硬编码 | `_find_repo_root()` + 全改 `declared_path()` |
| 3 | `freeze_baseline.py` 是**半成品迁移**：`snapshot()` 的**键**已是包内路径、**读**的还是 `config/` ⇒ 路径二义 | 键与读取来自两个来源 | 统一为 `repo_relative(declared_path(...))` |
| 4 | `run_sim2sim.py --freeze` **静默假成功**：退出码 0，真值目录**一字节未变** | 默认目标 `PROJECT_ROOT/tests/baseline` 把旧目录**重新建出来** | 默认目标改 `declared_path(rid,"tests.cases")` + `rm -rf tests/baseline` |

★★ **`_find_repo_root()` = 「向上找标记」**：向上找**同时含 `core/` 与 `robot-package/`** 的那一层。
它**替代一切 `parents[N]` / `Path(__file__).parent.parent`** —— 后者的层数是**被搬迁改写的隐式契约**，
写错时**不报错**，只是解析到另一个目录（于是症状伪装成"文件丢了"）。

**两条测试通道 + 两条守卫（"测试存在" ≠ "测试被执行"）**：

- Python：`pytest.ini`（**仓库根**）的 `testpaths = core/tests / tests / robot-package`。
  ★ 刻意**不**设 `--import-mode=importlib`（现有测试依赖裸模块名导入 + prepend 模式）。
  ★★ **不要给 pytest 传目录参数** —— 传了就用 args **覆盖** testpaths，包内测试被**静默跳过**
  （旧记忆里的 `pytest tests/sim tests/sim2sim core/tests -q` 正是这种写法）。
- 前端：`vite.config.ts` 的 `test.include` 增 `'../robot-package/*/tests/**/*.test.ts'`。
  包在 `frontend/` **之外** ⇒ 包内 TS 测试的 `import 'vitest'` 向上走不到 node_modules，
  故 `tsconfig.app.json` 的 `paths` 显式加 `"vitest"` / `"vitest/*"`（否则 tsc 报 `Cannot find module 'vitest'`）。
- 守卫 A（通道）：`core/tests/test_package_contract.py::test_pytest_testpaths_covers_package_dirs`
  + `frontend/tests/unit/packageTestChannel.test.ts`（读 `vite.config.ts` 与各包 manifest 的 `tests.local`，
  断言目录存在 / 在自己包内 / 有 `*.test.ts` / 与 Core 测试不重名）。
- 守卫 B（边界）：`frontend/tests/unit/corePackageBoundary.test.ts` 扫 `src/**/*.{ts,tsx,vue}`
  （**先剥注释再匹配 `import`**），白名单**只有** `store/robotStore.ts` 的 IK 类型
  （`solveIk` / `IkBranch` / `IkPreference` / `IkReason` / `IkResult`），并做逐符号双向核对 +
  白名单腐烂检测 + 旧路径检测。★ `robotStore` ← 包内 `ik.ts` 的**层次倒置**留到 Phase 3，
  用 `IKResult.diagnostics` 袋子解。

★★ **`pytest` 只在仓库根成立**（2026-09-14 实测）：从 `frontend/` 跑 `pytest -q` 输出
**`no tests collected` 且 exit 0** —— 只看退出码的脚本会把它读成"通过"。跑验收前先确认 cwd。

**本轮新踩的本机沙箱坑（三条，都改工作方式）**：

- ★★ **同文件连续 / 并行 `Edit` 会丢写入**：⇒ 改完**立刻用独立 `grep -n` / `Read` 核对落盘**；
  一处以上修改优先写**一次性脚本**（对每条替换断言"命中次数 == 1"，零命中/多命中都报错）而非连续 Edit。
- ★★ **`Path.write_text()` 在 Windows 把 `\n` → `\r\n`**，而仓库是 **LF-only + `core.autocrlf=false`**
  ⇒ 批量改文件必须 `open(..., newline="")`，否则"1 行改动"里混进全文行尾变换
  （实测 `assets/textures/mearm/README.md` 1276 → 1303 字节、27 处 CRLF）。出锅后 `git checkout --` 回退重做。
- ★ **在途运行的 pytest 会用旧模块**：后台 pytest 在我改 `manifest.py` **之前**已 import 旧 `_unknown`
  白名单 ⇒ 拿新字段 `local` 报 10 failed。**不是缺陷** —— 改完代码必须重跑。

**守卫被反向验证过**（"能红"才算守卫）：往 `frontend/src/robot/` 插一个越界 import ⇒
`corePackageBoundary.test.ts` 报错并**点名** `__boundary_probe.ts`（随后删探针）；
临时去掉 `vite.config.ts` 的 `include` 项 ⇒ `packageTestChannel.test.ts` 变红并**给出修法**。

**三处重复真值被消除**（second source of truth）：`ORDER` 线序（`tests/sim/test_server.py` →
`dev.robot.joint_order()`）· 限位文案 `108.44..141.86`（改为**从真值派生** + `re.fullmatch` 钉格式）·
`sim2sim.json` 的 `generator`（`run_sim2sim.py` 写入目标改 `declared_path(rid,"tests.cases")`，
不再自带第二份生成器声明）。

**Phase 2 实测验收**：`pytest -q` **204 passed** · `vitest run` **33 文件 / 441 例** ·
`tsc -b --force` **0 error** · `vite build` 产物中 `__armPilot` / 旧真值路径命中 **0** ·
`go build/vet/test` **0** · `cli.py validate --all` **2 通过 / 0 问题** ·
`freeze_baseline.py` **与冻结基线一致** · `run_sim2sim.py --all` **全部机器人 FK 在登记容差内**。
文档：`docs/architecture/robot-package-phase2.md`（判据 / 真 bug / 纪律 / 验收命令 / 未做清单）。

**已登记未做（Phase 3+）**：资产搬迁（`assets/models/so-arm101/official/**`、`assets/textures/mearm/**`、
`3d-models/*.STEP`）· `IKResult.diagnostics` 袋子（解 `robotStore` → 包内 `ik.ts` 的层次倒置）·
C5（`role` 跨端语义分歧）· C6（`project_plates.py` 硬编码板件尺寸）· `test_ik.py` 残留的 MeArm 耦合
（目前是"把型号当数据键"的可接受形态）。

## §9 一键启动器（`start.bat`）与真机验收口径（2026-09-14 起）

### 9.1 `start.bat` 现在不止"起两个窗口"

| 阶段 | 做什么 | 为什么必须有 |
|---|---|---|
| `[0/3]` 陈旧闸门 | 任一 `backend/**/*.go` 比 `bin/armpilot-backend.exe` 新 ⇒ 自动 `go build`；**没有 `go` 就拒绝启动** | 旧二进制不报"我过期了"：它照常启动、毫秒内因配置错误死掉、窗口一闪而过。实测就这么发生的（exe 落后**整整一次重构**，仍在找已废弃的 `config/robot.yaml`） |
| `[1/3]` 端口预检 | 8090 / 5273 占用则列 PID 并**问**是否清理 | （原有） |
| `[2/3]` 启动 | 两个独立窗口 | （原有） |
| 启动后探活 | 主动请求 `/healthz`，答不上就在 **launcher 窗口里**报出来 | "窗口出现了" ≠ "服务起来了"。后端自己的报错在被回收的窗口里，用户在 launcher 里根本看不到 |

陈旧判据用 `%%~tT` 字符串比较（格式 `yyyy/MM/dd HH:mm` ⇒ 字典序 == 时间序）。
★ **别用 `dir /o-d` 多目标排序**（实测报"文件名、目录名或卷标语法不正确"），也别用 `forfiles /d`（只到"天"）。

### 9.2 `.bat` 里的括号：只有"块内裸括号"会炸

| 位置 | 安全? | 说明 |
|---|---|---|
| `if (...)` / `for ... (...)` **块内**的裸 `(` `)` | ❌ **炸** | cmd 的块读取器扫到第一个 `)` 就闭合块，余下文本成游离 token |
| 同一行**双引号内**的括号 | ✅ | 引号保护（实测） |
| **顶层**（不在任何块内）echo 里的括号 | ✅ | 不参与块配对（实测） |

症状：`此时不应有 .`（`. was unexpected at this time`）+ `RC=255`，脚本死在 preflight。
**定位手段（比二分快得多）**：把脚本复制成 `@echo on` 版本跑一遍，trace 会停在出错那一行。
（同理：块内 echo 里裸 `>` 会被当成**整块**的重定向，写文本要 `^>`。）

### 9.3 真机验收有**两条独立证据链**，不能混着读

| 证据链 | 能证明 | **不能**证明 |
|---|---|---|
| 链路层（`hello.device=serial` / `OK JR` 回执 / `joint_state` / `verify_serial_e2e.mjs --no-camera`） | 指令**确实送到了固件并被接受** | ❌ **物理到位** —— 真机无位置反馈，`joint_state` 是固件**内部目标值**（开环） |
| 相机（`verify_pose.py` 反解肩/肘绝对角） | **物理**在哪（唯一外部地面真值） | —— |

⇒ **链路层全绿 ≠ "真机验证通过"**。要判物理到位，必须让相机取景按 `docs/hardware-measurement.md`
的 **Phase 4.5** 就位（白分割板 + 画面内标尺 + 正交侧视 + **锁死曝光**）**且人员离场**。
台面没就位时，`verify_pose.py` 的**绝对误差含反解公共偏置**，不可当机械精度读 —— 只有
**重复性（同位姿两帧之差）**是干净的噪声底。取景里连臂都没有时，只能报"物理层未取到证据"，别硬算。

### 9.4 （已修）Core 重构后工具路径的 off-by-one

工具从 `<repo>/tools/` 搬进 `core/tools/` 时，`ROOT = resolve(TOOLS_DIR, '..')` **没多退一层** ⇒
`ROOT` 变成 `<repo>/core`。后果不是报错，而是**静默指错地方**：backend 找不到、输出写进
`core/.workbuddy/`、`verify_pose.py` 按旧目录找 ⇒ `[fatal]`。
**搬工具时同步检查**：`ROOT`/`REPO_DIR` 的层数 · 工具之间的相对引用 · 文档里写下的路径。
另一条同族纪律：型号专属工具**随包走**（`manifest.tests.tools` 是完整清单）⇒ Core 侧要
**由 `config/robots.yaml` 的 `default` 定位包**，不要写死 `mearm-v1`（写死 = 每接一台机器人回 Core 改一行）。
**这类修复的可测判据**：`node core/tools/verify_serial_e2e.mjs --dry-run`（不碰硬件，验路径 + 动作计划 + 限位）。

### 9.5 （2026-09-14 修复）gripper 真机「有概率不执行」

**根因（纯 Web UI 拖动即可触发，不需 IR/摇杆）**：后端 `execJR` 把 4 执行器按固件
`SET ≤3 对` 硬限制拆 2 条，**gripper 恒定在第二条**；原实现第一条 SET 的 ACK 超时/异常即
`return`，**第二条（gripper）根本不发**——前三个关节已动、gripper 没动。底层诱因是固件串口
64B RX 缓冲偶发静默丢字节（高频拖动 + 舵机负载下）。

**修复（提交 `caf402d`，6 文件 +168/−22）**：
① `serial.go` `execJR` 失败不再 `return`，`continue` 下发其余 SET（gripper 必发）；
② `awaitAck` 严格匹配 `OK <what>`，异步 `OK IR`/`OK IRSEQ` 不再被误当 SET 应答（B 缺陷），
   且只在应答行收集舵机角；
③ 固件 `arm_nudge` 基于 `target` 增量（斜坡中途 nudge 不再就地取消运动，A 缺陷）；
④ 固件 uart RX 缓冲 64→256（治本诱因）。新增 2 个回归测试（`device` 包全绿）。

> ⚠️ 修复当时设备不响应串口（独占打开成功、无进程占用、10×`STATUS` 静默），故真机端到端
> 最终复现未跑。恢复硬件后按 `.workbuddy/captures/gripper_noexec_hunt.py` 验证
> （直连串口、复刻 execJR 命令形状、读真实 `current`）。A/B 两缺陷触发源是 IR/摇杆，
> 与用户「从未碰遥控」不符，但主因（execJR 牺牲第二条 SET）是纯拖动场景即可触发。


## §10 CAD / STEP 数据接入（2026-09-15 起）

**结论**：STEP 可作**外观源**，**禁止**作运动学真值源。`robot.yaml` 一个数字都没改。

### 10.1 铁律：读复杂标准格式一律用参考实现

自研正则解析器读 `3d-structure/mearm3Dasm.STEP` 时误判「只有层级没有几何、0/241 可达」
并写成 `BLOCKED`。用户指出「不要自己写解析器，用完整开源项目」——**用户对**。
换 **OCCT**（`cadquery-ocp` 8.0.1.0.0，`STEPCAFControl_Reader` + `XCAFPrs_DocumentExplorer`）
读同一文件 ⇒ `nodes=111 leaves=103 with_geometry=111 servos=4`，**111/111 全有几何**。

- **铁律**：自研工具给出**否定性结论**（「文件坏了」/「数据缺」）时，
  **必须先用参考实现复核**，否则会把**工具的能力边界**误报成**数据的缺陷**。
- 自研解析器静默丢弃的三处（修完一处还会被下一处继续骗，**不要试图逐个打补丁**）：
  ① **复杂实例** `#1385 =( A(...) B(...) C() );`（多子类型并列、无单一类型名）；
  ② **嵌套括号引用** `FACE_OUTER_BOUND('NONE',(#101436),.T.)`（`refs()` 漏 list 就断链）；
  ③ **编码**（法语名 + 中文占位标签交替）。
  ★ 特别警示：修好 ① 后**实体计数已对齐（154708 / 0 缺失）但几何仍为 0** ——
  「计数对齐」**不等于**「解析正确」，不能拿它当验收判据。

### 10.2 唯一保留的工具

`core/tools/step_report.py`（OCCT）。输出装配树 + 命名件 + world AABB + 舵机输出轴：

```bash
PYTHONIOENCODING=utf-8 "$PY" core/tools/step_report.py \
  robot-package/mearm-v1/3d-structure/mearm3Dasm.STEP \
  --json .workbuddy/captures/step_report.json
# 预期: nodes=111 leaves=103 with_geometry=111 servos=4
```

❌ 已删除（产出过错误结论 / 已被吸收）：`parse_step_assembly.py`、`step_world_geometry.py`、
`step_occt_tree.py`、`step_joint_axes.py`。

### 10.3 OCP / pybind 绑定陷阱（写脚本必踩）

| 陷阱 | 正解 |
|---|---|
| `Bnd_Box.Get()` 报 `Unable to convert ... -> Bnd_Box::Limits` | 取 6 个 `CornerMin()/CornerMax()` 分轴访问 |
| `XCAFPrs_DocumentExplorer.Flags_s()` 不存在 | 用**模块级** `XCAFPrs_DocumentExplorerFlags_None` |
| `exp.IsCurrentLeafNode()` 不存在 | 用 `not node.IsAssembly` |
| `TopoDS.Face_s` | 用 `TopoDS.Face`（`Face_s` 是 OCP 旧命名） |
| 叶片标签名是 `NAUO*`（无名） | 真名在 **`node.RefLabel`**（`IsNull()` 才回落 `node.Label`） |
| `TDF_LabelSequence` 不在 `OCP.TDF` | 在 `OCP.collections.Sequence_TDF_Label` |
| 舵机输出轴与舵盘轮毂/让位孔混在一起 | 按半径阈值分离：轴 R=6.20 / 轮毂 R≈2.85 / M3 让位 R<0.8 ⇒ 取 R≥4.0 |

### 10.4 从 CAD 读关节轴：SG90 输出轴 = 关节旋转轴

在 4 个 `microservoSG90` 实体上枚举圆柱面（`BRepAdaptor_Surface` + `GeomAbs_Cylinder`），
每个舵机的输出轴都命中 **3 个共轴圆柱面**（R=6.20），一致性很高、非偶然面。

**实测（世界系，mm，Z-up）**：

| 编号 | 轴方向 `d` | 轴上一点 `p` | 角色（推定） |
|---|---|---|---|
| 0 | `[0,-1,0]` | `[-14.731, 25.400, 155.975]` | 肩 |
| 1 | `[0,0,1]` | `[  8.705,-13.165,  53.100]` | 底座（外侧） |
| 2 | `[0,0,1]` | `[ 13.693,  0.746,  53.100]` | 底座（内侧） |
| 3 | `[0,1,0]` | `[-89.986, 35.788, 232.587]` | 肘 或 夹取 |

**轴间关系**：0↔3 **平行（0.00°）、间距 107.39** ｜ 1↔2 **平行（0.00°）、间距 14.78** ｜
其余两两 **90°**。轴向**全部与 `robot.yaml` 吻合**（base `[0,0,1]`；shoulder/elbow/tool `[0,1,0]`）
⇒ **`KINEMATIC_STRUCTURE_CONFLICT = NO`**。

### 10.5 ★★ 肩轴 ∥ 肘轴 ⇒ 平行四连杆的 CAD 结构实证

轴 0（肩）与轴 3（肘）**严格平行**、间距 107.39 —— 这是**平行四连杆**的定义性几何签名。
⇒ `robot.yaml` 的 `elbow.coupling = {shoulder, gain: -1}` 此前只是 **2026-09-13 照片反解**的
经验拟合（拟合残差 ~19%，yaml 自陈「硬拟合 ≈ −0.81」），**现在升级为结构事实**。
（可选：在 yaml 注释补「已由 CAD 几何实证」；**注释不触发**冻结基线。）

### 10.6 ⚠️ 为什么 `length` **不可**由 STEP 反推（别浪费时间再试）

**已证明 STEP 姿态 ≠ 零点位**：yaml 零点位下 shoulder=0（大臂竖直）+ elbow=112.62
⇒ 肩肘两轴应差 **22.6°**；CAD 实测 **0.00°** ⇒ 建模时大臂与小臂**共线**。
因此 STEP 给的是「**该姿态下**的瞬时轴间距」，而需要的是「**零点位**沿各杆 +Z 的伸展」。
拿 107.39 写 `upper_arm_link.length` 会**立刻破坏 FK/IK + 冻结基线**。

- 「底板→肩轴 106 mm」同理**语义不同**：它含舵机体 29.8 + 转盘板(58×3×58) + 立柱板(90.7×3×145)，
  而 `column_link.length = 60` 是「转盘→肩的纯伸展」。
- spec §9 明令**禁止**用 mesh 包围盒当连杆长度（包围盒含端盖/横撑/倒角，系统性偏大）。
- **唯一正当路径**：**标尺实拍**（`docs/hardware-measurement.md` §7 Phase 4.5）。
  SCAD 没有、也无法替代这条路。

### 10.7 遗留 UNCERTAIN（Level 1–2）

| # | 待办 | 闭合方式 | 状态 |
|---|---|---|---|
| T1 | 关节 `+θ` 旋向符号（STEP 的轴是无向直线，不含零位信息） | 保持 yaml 现有符号（实拍反解） | 保持 |
| T2 | 4 段 `length` | 标尺实拍 | 待实拍 |
| ~~T3~~ | ~~轴 1 与 2 是两根平行竖轴、间距仅 14.78 mm 的身份~~ | **用户 2026-09-15 确认：固定件，不需考虑** | ✅ **CLOSED** |
| T4 | 夹爪铰轴销 R≈3.2（yaml `details` 写 `radius: 3.2`）被提取阈值 R≥4.0 漏掉 | 阈值降到 3.0 复扫 | 低优先 |

> ✅ **T3 已由用户裁定为固定件** ⇒ 底盘上那两根平行竖轴（间距 14.78 mm）**不是自由度**、
> 不进运动链、外观阶段可**如实照搬 CAD 形状**。5 关节拓扑确认无遗漏。


### 10.8 外观阶段的边界（Phase 1 待办，本次未执行）

| 允许改 | 禁止改（**动了就要停下问用户**） |
|---|---|
| `links[].geometry.*` / `links[].details.*` / `appearance.*` | `links[].length` / `joints[].*` / `actuators[].*` / `robot.homePose` / **`robot.tcp`** |

- 冻结基线判据是**语义核心哈希**（D55），白名单只含 `id/name/units/tcp/homePose` + 运动学/物理量
  ⇒ **`geometry` / `details` 不在其中**，改外观**不会**让 `test_baseline_frozen.py` 报红。
- ⚠️ 但 **`tcp` 在**白名单里 ⇒ 外观阶段**绝不要**顺手改 `robot.tcp`。
- ⚠️ **不得**按视觉外观重新划分 `links`（spec §14–17 + 用户明确要求）：
  显示分组**必须**沿用现有 6 个 link 边界，只换每个 link 内部的几何形状。

## §11 验收七件套（完整命令）

```bash
# ── 前端三件（在 frontend/ 下执行）──
cd frontend
./node_modules/.bin/tsc -b --force            # 0 error
./node_modules/.bin/vitest run                # 全绿（含 ../robot-package/*/tests/*.test.ts）
./node_modules/.bin/vite build                # dist/assets/*.js 中 __armPilot 命中 0（*.js.map 必然含，不算）
node tests/e2e/ui-smoke.mjs                   # 必须隔离端口
# ── Python 六件（★ 必须在**仓库根**执行）──
$PY -m pytest -q                              # ★ 走 pytest.ini testpaths：core/tests + tests + robot-package
$PY core/tools/run_sim2sim.py --all           # ★ 统一 Sim2Sim 矩阵（选择器全部机器人，D77）
$PY robot-package/mearm-v1/tools/gen_mearm_v1_baseline.py --check    # 黄金数据逐位复现（改运动学/物理必跑）
$PY robot-package/so-arm101/tools/gen_so_arm101_robot_yaml.py --check # SO-101 配置 ↔ 官方模型同步
$PY robot-package/so-arm101/tools/inspect_so101_physics.py --check   # SO-101 物理快照 ↔ 官方 MJCF（46 项）
$PY core/python/robopkg/cli.py validate --all # 选择器 / manifest / 真值 / 包目录 四方对账
```

**实测基线**（2026-09-15）：`pytest -q` = **198 passed** · `test_baseline_frozen.py` = **11 passed** ·
`cli.py validate --all` = 通过 · 前端 `vitest` = **401 passed** · `go test ./...` 全绿。

---

## §12 固件链路诊断与「工具自己误报」（2026-09-15）

**背景**：用户报「gripper 偶发不顺畅」。固件侧加了 `STATS` 命令（读 `rx_drop`/`tx_drop`），
真机实测 **240/240 收发、零缺失、rx_drop=0** ⇒ **没复现出缺陷**。

### 12.1 唯一可信判据：独立计数器，不是收发对账

| 判据 | 可信度 | 说明 |
|---|---|---|
| **固件 `STATS` 的 `rx_drop`** | ★★★ 一票否决 | 链路真丢字节 ⇒ 必然非 0。独立于上位机收发时序 |
| `STATUS` 回读 target | ★★★ | 证明"固件确实接受了目标"，不是只收到指令 |
| 脚本的"发送 vs 回执"对账 | ★ | **极易被工具自身的度量缺陷污染**（本次就是） |

⇒ **铁律**：要否定"链路丢字节"这一假设，**只能**靠 `rx_drop`；不能用脚本对账。

### 12.2 ★★ 诊断脚本的 4 个度量陷阱（写串口工具必查）

1. **`_read_line()` 只在"读到新串口字节"时才可能返回一行** —— 缓冲里还有 200 行，
   但串口暂无新字节 ⇒ 空转到超时返回 `None` ⇒ `drain()` 的空闲判据被假 `None` 触发、提前收工。
   **修法**：进循环前先查 `self._buf` 是否已含完整行，**有就直接返回、完全不碰串口**。
2. **声称 fire-and-forget 实则逐条 `wait_for`** —— 压力降到"发一条等一条"，
   且迟到的 ACK 被塞进 `others` 而报告**从不打印** ⇒ 静默吞掉。
3. **只填 `ack*_line` 不填 `ack*_ms`**，而 missing 统计读 `_ms is None` ⇒ **恒定**误报"全帧无 ACK"。
4. **固定等待窗口当就绪判据** —— 见 12.3。

> 与 §10「自研解析器」同源：**工具的局限会被表现成数据的性质**。
> 自研工具给出**否定性结论**（"缺 221 条"/"文件坏了"）时，**必须先用独立手段复核**。

### 12.3 板子复位后的「吞命令」窗口（Uno，可复用）

开串口 → 复位（DTR 断言）。实测序列：

```
try0: ''                                  ← 窗口①（横幅还没出来）
try1: '[meArm] ... ready' / 'STATUS ...'  ← 复位横幅（~1.6s 才到）
try2: ''                                  ← 窗口②（横幅之后又吞一条）
try3..: 'OK STATS rx_drop=0 tx_drop=0'    ← 之后稳定
```

★ **DTR 边沿并非每次都触发复位**（连续开关端口时尤其如此）⇒
**"是否看到横幅"不能当就绪判据**。唯一可靠定义是**能应答** ⇒ 用**重试探针**
（反复发一条轻量命令直到收到回执）。
**不是固件缺陷**：主循环非阻塞（`cmd_poll()` 每轮都跑），是复位后 UART 的稳定窗口。

### 12.4 固件侧的两条改动（仍有价值，但不修已证实的缺陷）

- `uart_putc()` 去掉 `while (tx_full())` 忙等 → **丢字节 + 计数**。
  理由：阻塞主循环会让它**停止读 RX**，反而可能丢**整条指令**且无痕迹。
- RX ISR 环满时**计数**（原为完全静默）。
- `TX_BUF_SZ` 128→256。占用：FLASH 12582→12848 B（39.8%）/ RAM 708→844 B（41.2%）。
- 新命令 `STATS [CLEAR]` → `OK STATS rx_drop=N tx_drop=N`（8 位**饱和**计数，读到 255 别当"恰好 255"）。

### 12.5 工具

`robot-package/mearm-v1/tools/probe_gripper_link.py`（4 处缺陷已修，文件头含完整复盘）·
`probe_gripper_ws.py`（WS 旁听真实前端）· `watch_gripper_live.py`（轮询 `/healthz`）。

**仍未复现用户症状**。下一步方向：① 用 `probe_gripper_ws.py` 旁听**真实前端**（而非脚本复刻）；
② 追问"不顺畅"的**物理表现**（卡顿/不到位/异响）；③ 考虑舵机**供电/电流**（堵转）这类非链路原因。

---

## §13 改真值的连锁清单（D80 实践，**照抄这个顺序**）

改 `robot.yaml` 的 `kinematics` / `actuators` 或 `physics.yaml` 的物理量时，
**必须按序**完成下面 5 步，漏一步就会被守卫抓到（或更糟：静默不一致）。

| # | 动作 | 判据 / 坑 |
|---|---|---|
| ① | 重生成产物：`tools/gen_model.py`（MJCF）+ `tools/gen_urdf.py`（URDF） | 产物过期 ⇒ `test_generated_mjcf_is_in_sync_with_config` 报红（**守卫正常工作**） |
| ② | `core/tools/freeze_baseline.py --update` | ★ **冻结基线的键 = 真值文件的仓库相对路径** ⇒ `--update` 后**哈希必须逐位不变**；变了说明路径动了 |
| ③ | 重采集黄金数据：`gen_mearm_v1_baseline.py` + `run_sim2sim.py --freeze` | 不重跑 ⇒ Sim2Sim 回归必红 |
| ④ | 修**写死旧值的断言**（前端 + Go 都有） | ★ **能引用 `limits.min/max` 就别写死端点**，否则每次修订都制造假红 |
| ⑤ | 同步文档（4 份：`serial-v1.md` / `coordinate-system.md` / `hardware-measurement.md` / `ARCHITECTURE_ANALYSIS.md`） | **文档↔配置一致性在 pytest 里盯** |

### ★★ 最容易踩的一条：方向类标定会翻转"钳位端"

改 `reverse`（或等价的方向翻转）时，**逐处检查"越界钳位"断言钳的是哪一端**。

D80 实录：`servo_6` 由 `θ+40` 改为 `-θ+140` ⇒ **θ 越大舵机角越小**。
`TestSerialClampedEchoSurvivesInOKJR` 原本用 `θ=200` 触发"钳到**上限**"，
新映射下 `θ=200` 会钳到**下限** ⇒ 必须改用 **`θ=-200`** 才保持原意。

⇒ 通则：**改方向后，把所有"越界用例"逐个重算一遍期望值**，
不要只看"还报不报错"（报错方向反了照样是绿的）。

### ★ 冻结基线的"语义核心"边界（D55）

判据是**语义核心哈希**，不是整文件：

- **放行**：改**外观**（`links[].geometry` / `details`）—— 换 STL 网格、改尺寸不报错
- **报错**：改**运动学**（`length` / 轴限位耦合 / `actuators`）或**物理量** ⇒ 逐字段列差异

### ★ `content_hash` 的语义陷阱

`manifest.model.generated_by` 描述的是**「生成 model.urdf 的生成器」**，
**不是** `model.config`（`robot.yaml` 永远是真值、永远进哈希）。

实测位置 `core/python/robopkg/content_hash.py`：
```python
add(manifest.model.config, is_generated=False, label="model.config")            # L178 ← 必须 False
add(manifest.model.urdf,   is_generated=manifest.model.generated_by is not None) # L180
add(manifest.model.physics, is_generated=False, label="model.physics")           # L181
```
即：**只有 `model.urdf` / `simulation.mjcf` 这类产物才 `is_generated=True`**；
`model.config`（robot.yaml）/ `model.physics` / 各 `entry` / `tests.cases` 一律 `False`。

### ★ 已放弃的路线（别再试，省得重复劳动）

| 路线 | 放弃理由（实测） |
|---|---|
| **CAD→URDF 直接接入** | `3d-structure/local_mu28fwc1_g8139u_urdf_stl/robot.urdf` **只有 1 link / 0 joint**、111 件未指派、且 **mm 尺寸配 m 原点的 1000× 单位错** ⇒ joint origin/rpy/axis/limit 全部**无输入可取** |
| **STEP 反推 `length`** | 已证 **STEP 姿态 ≠ 零点位**（yaml 零点位下肩肘应差 22.6°，CAD 实测 0.00°=共线）⇒ 唯一路径是**标尺实拍** |
| 自研 STEP 解析器 | 误判"0/241 可达"（见 §10 复盘）⇒ 一律用 OCCT |

### ★ 夹爪（S6）真机标定：`S6=40 张开 / S6=130 闭合`（ADR D80）

```yaml
# robot.yaml · actuators.servo_6
offset: 140
scale: 1
reverse: true        # servo = -θ + 140
# joints.gripper.limit = 10..100   （由舵机硬限位 40..130 反算）
```
三端点自洽：`θ=10 → S6=130`（闭合）/ `θ=50 → S6=90`（**HOME，固件开机位**）/ `θ=100 → S6=40`（张开）。

⚠️ **不能只翻 `reverse`**：`homePose.gripper=50` 必须反算出 S6=90，
而旧关节限位 `0..90` 本身是照**旧（错）方向**定的 ⇒ **限位必须一并由舵机硬限位反算**。
（只改 offset 的两种尝试都会撞车：`offset=130` ⇒ HOME 变 80；`offset=140` ⇒ 闭合端 140 超限。）

⚠️ **遗留**：真机方向仅来自**用户口述**，**无相机判据**（`verify_pose.py` 没有爪开合反解）。
要变成"可独立复核的读数"，需为夹爪设计近景拍摄 + 爪间距量化的标准流程。


## §14 状态回传的完整通路与缺口（2026-09-16）

**问题**：摇杆把机械臂调 30°，MeArm-3D 上位机状态会不会跟着变？

**一句话结论**：**「上位机下发」完整同步；「下位机自主变化」整条是断的。**

### 14.1 唯一的通路：`STATE <四个数值>`

`protocol.ParseReply` 是判决点。实测（把固件真实报文喂进 controller）：

| 报文 | Kind | 上位机 |
|---|---|---|
| `STATE 0.00 30.00 120.00 50.00` | `STATE` | ✅ 更新 |
| `OK JOY S6=90 S7=90 S8=90 S9=90` | `OTHER` | ❌ 无感 |
| `STATUS S6=90(H) S7=90(H) …` | `OTHER` | ❌ 无感 |
| `OK SET S9=120 S7=131` | `OTHER` | ❌ 无感 |
| `OK NUDGE S9=93` / `# IR RAW=…` | `OTHER` | ❌ 无感 |

**注意 `OK JR` 不等于状态**：它是**目标**舵机角（意图），只用于核对标定；
拿它当 Actual 回推会让误差恒为 0，把「滞后 → 收敛」整条语义抹掉。

### 14.2 那么 `STATE` 是谁发的？——**后端自己合成的**

`serial.go` 只在**自己发起**的三条路径上合成 `STATE`：

| 路径 | 触发 | 合成方式 |
|---|---|---|
| `execJR` | 后端下发 JR → 拆成 SET | 收齐 `OK SET` 的 `S<n>=` 后 `emitStateFromServo` |
| `execStatus` | 后端主动发 `STATUS` | 同上 |
| `execReset` | 后端发 `RESET` | 读第二行的 `STATUS` 后合成 |

`execute()` 的 `default` 分支（未识别动词）注释即结论：
**「回执原样转发但**不合成**任何关节级回执」**；`forward()` 只把被动收到的行记日志。

⇒ **没有外部动作时，链路一定静默**（实测静置 2s 新增 0 条）。

### 14.3 固件侧：根本没有 JR / STATE

`MeArm-Device/core/cmd.c` 只有舵机级动词：`SET` / `S<n>=` / `JOY` / `IR` / `RESET` /
`STATUS` / `STATS` / `AUTO` / `STOP` / `SEQ` / `JOYHW` / `IRHW` / `ADC` / `HELP`。
`docs/serial-v1.md` 开头就写着「**§4 的固件侧 `JR`/`STATE` 仍待实现**」。

摇杆（`joystick.c` 扫 ADC A0..A3）与红外改完舵机角**不回吐任何东西**。

### 14.4 后端无周期轮询

`STATUS` 只在 **warmup**（暖机）时发一次 —— 目的是把「Uno 被 DTR 复位后舵机弹回 90°」
这个真相拉回来（`serial.go` 注释：「不纠正的话 UI 会显示一个与现场完全不符的 Actual」）。
**之后不再轮询。**

### 14.5 sim 与 serial 的不对称（实测 sim 命令接受面）

```
JOY 9 900 / JOY 900 200 512 800   → ERR UNKNOWN      ← 真机摇杆命令 sim 不认
SET 9 120 / S9=120                → ERR UNKNOWN      ← 真机直控 sim 不认
STATUS                            → STATUS S9=90 …   ← 格式不被 ParseReply 识别
JR 0 30 120 50                    → OK JR + 一串 STATE ✅ 唯一可用
```

⇒ **想在 sim 上"模拟摇杆改下位机"，当前代码里没有入口**：sim 只实现关节级 `JR`。
另注：sim 的 `STATUS` 回执格式与真机**一致**，但 `serial.go` 有 `execStatus` 翻译层、
sim 没有 —— 这是两者的一处结构性不对称。

### 14.6 ★ 摇杆网关会抢串口

`MeArm-RemoteControl`（Web 9001 / TCP 转发）用 `serial.Open` **独占串口**，
backend 的 `device.mode=serial` 也要开同一个口 ⇒ **两者不能同时连**
（Windows 串口独占，它的错误信息里也写了"端口可能被其它程序占用"）。
所以若摇杆由它驱动，MeArm-3D 界面不是"不跟随"，而是**根本连不上设备**。

### 14.7 端到端 + 界面实测（Go 后端 sim + Vite + CDP）

- 新客户端接入即收 `hello` + 一条 `joint_state` ⇒ **接入即推当前状态**（好）
- A 客户端发命令，**B 客户端收到同一批 9 条** `joint_state` ⇒ **广播完整**
- 收敛轨迹 `shoulder 4.18 → 7.52 → … → 30.0000`（20ms tick / 240°/s，**非等值回显**）
- 外部 WS 客户端下发 A 组→B 组，读界面 DOM：`Actual` 四关节**全部跟随** ✅
  （`base -30→30`、`shoulder 10→45`、`elbow 130→115`、`gripper 30→90`）

⚠️ **但 Command 列不跟随**（它是本客户端的意图），于是界面立刻变
「异常：命令已静止但误差停在 60.00° —— 查卡死 / 失步 / 限位截断」、误差条 4 条全红。
**摇杆场景下这是持续误报** —— 真实原因是"别的入口动了机器"，
而 UI 没有"外部来源"这个概念。

### 14.8 要让「下位机自主变化」被感知，最短的三条路

1. **固件主动上报**（改方向）：协议 §4 的 `STATE` 已定义好，**通路已就绪** ——
   实测「假设固件发 `STATE …`」→ 上位机立刻更新。可做**变化触发**（`arm_nudge` /
   `ir_seq` 改角后 emit 一条），比周期上报省带宽（ATmega328P 只有 2KB RAM）。
2. **后端轮询**（改后端）：低频（如 2~5Hz）发 `STATUS`；但需同时**扩展 `ParseReply`**
   或让 `forward()` 对含 `S<n>=` 的行合成 STATE —— 否则回执仍然判 `OTHER`。
3. **前端区分来源**：给"外部变化"一个来源标记，避免在摇杆场景持续误报链路异常。

**本轮未改任何产品代码**，只做分析 + 实测；临时探针（Go 测试 2 个 / WS 客户端 /
CDP 脚本 / `tmp_probe` 包）已全部清理。
