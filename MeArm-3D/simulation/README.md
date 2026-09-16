# `simulation/mujoco/` · MuJoCo 物理仿真后端

> 把 ArmPilot 的末端从「几何 / 运动学回显」升级为**真实刚体动力学**，
> 但**不动 Web UI、不动协议、不动现有链路一行代码**。
>
> 快速命令：`python simulation/mujoco/run.py --demo`（Viewer）·
> `python robot-package/mearm-v1/tools/gen_model.py`（重新生成 MJCF）·
> `cd MeArm-3D && <python> -m pytest tests/sim -q`（106 项验收）

---

## 0. ★ 诚实声明（spec §37，先读这一段）

**当前模型是 Level 3→4（参数化物理仿真），不是 Level 5（真实机械臂标定模型）。**

`robot-package/mearm-v1/physics/physics.yaml` 里**没有任何一个数字来自对本台 meArm 的实测**：

| 标记 | 含义 | 举例 |
|------|------|------|
| `[公开]` | 开源 meArm 标称尺寸 / MG90S 舵机公开规格书 | `max_torque_nm: 0.1765`（1.8 kg·cm） |
| `[估算]` | 由 `robot.yaml` 的 `geometry.size` × 材料密度推算 | 各连杆质量与惯量 |
| `[经验]` | 控制增益类，取量级合理值待调 | `kp: 5.0` / `kv: 0.5` |

因此这个模型的**结果不可用于宣称"真机会这样动"**。它能支撑的结论只有：

- ✅ 「这套几何 + 这套惯量 + 这套增益下，动力学算出来是什么」
- ✅ 「运动学定义（FK/IK）与物理引擎对同一个模型的解释是否一致」
- ❌ 「真机在 0.3 s 内会转到 20°」（真机的摩擦、间隙、舵机死区、电池电压都没建模）

**"已标定"这件事被做成了机器可检查的**：`robot-package/mearm-v1/physics/physics.yaml` 的
`calibration.calibrated` 必须为 `false`，且七个可标定量段全为空
（`tests/sim/test_simulation.py::test_level_declaration_is_machine_checkable`）。
要把它改成 `true`，必须先有真实实验数据 —— 测试会拦住没有数据支撑的顺手改动。

---

## 1. 架构位置：第三个 `device.Device`

```
浏览器 ──WS(JSON)──▶ backend(wsserver) ──▶ controller ──▶ device.Device
                                                          ├── sim.go      内置假固件（纯运动学）
                                                          ├── serial.go   真串口（真机）
                                                          └── mujoco.go   ★ 本目录的 Python 服务
```

`device.Device` 只有 7 个方法（`Kind/Connected/UnavailableReason/WriteLine/Lines/OnStatus/Close`），
所以新增 MuJoCo **只是第三个实现** —— WebSocket 层、controller、协议、前端**全部零改动**。
前端看到 `device.kind == "mujoco"` 时按"仿真"放行（安全门只拦 `serial`），
`hello.simulation_mode` 会带上 `"mujoco"` 供 UI 区分（spec §25）。

**接入方式**：`backend/config.yaml` 把 `device.mode` 改成 `"mujoco"`。
Go 侧起一个 `python server.py` 子进程，用**与固件逐字节相同的文本协议**通信。

---

## 2. 文件清单

| 文件 | 职责 | 关键点 |
|------|------|--------|
| `../../robot-package/mearm-v1/tools/gen_model.py` | `robot.yaml` + `physics.yaml` → `mearm.xml` | **MJCF 是产物，禁止手工编辑**；重跑即覆盖。⚠️ 生成器随包走（Phase 2），**不在本目录** |
| `mearm.xml` | 生成的 MJCF（9.2 KB · **nq=5 njnt=5** nbody=8 ngeom=34 nu=4 neq=1 ntendon=1） | 真值来自 yaml，改硬件参数要重新生成。⚠️ **nq=5 ≠ 自由度 4**：被动腕 `tool` 是 hinge（有 qpos）但不是自由度 |
| `robotcfg.py` | 读 `robot.yaml` / `physics.yaml` | **不设默认值兜底，缺字段就报错** |
| `units.py` | 单位（mm↔m、deg↔rad）与**角度语义**转换 | `elbow` 绝对角 ↔ hinge 局部角只在这一个文件里换算 |
| `model.py` | `MeArmSim`：封装 `MjModel`/`MjData` | reset / step / settle / state / gravity_torque |
| `limits.py` | 关节限位校验（与 Go `Validate` 同语义） | 错误文案与固件 `ERR JOINT` 逐字一致 |
| `server.py` | 无头设备服务（stdio 上跑 arm-device 协议） | 启动握手 PING、命令先 drain 再判 stop |
| `run.py` | 独立 Viewer / 演示脚本（spec §27） | 统计行：FPS · sim time · 关节角 · TCP · 接触数 |
| `record.py` | JSONL / CSV 记录（spec §34） | 字段表来自 `physics.yaml`，不落数据库 |
| `calibrate.py` | 标定接口（spec §38） | `--show` / `--template` / `--apply` |
| `fkref.py` | **参考 FK（独立实现，仅用于验收）** | 直接读 `robot.yaml` 原始几何，不复用生成器 |
| `../../tests/sim/` | pytest 验收（106 项） | FK / IK / 碰撞 / 动力学 / 协议 / 系统级 |

---

## 3. 怎么跑

```bash
PY=~/.workbuddy/binaries/python/envs/default/Scripts/python.exe    # 需 mujoco / pyyaml / pytest

$PY robot-package/mearm-v1/tools/gen_model.py       # 重新生成 mearm.xml（改 yaml 后必跑）
$PY simulation/mujoco/run.py --demo                 # 打开 MuJoCo Viewer 跑 6 段演示
$PY simulation/mujoco/run.py --headless --demo --duration 6.5     # 无头，打印统计行
$PY simulation/mujoco/run.py --pose "0 20 130 50" --duration 5    # 指定目标位形
$PY simulation/mujoco/run.py --record               # 落 JSONL（默认关，需 physics.yaml 打开）

$PY simulation/mujoco/calibrate.py --show           # 看哪些量已标定 / 待标定
$PY simulation/mujoco/calibrate.py --template out.yaml   # 生成待填模板

cd MeArm-3D && $PY -m pytest tests/sim -q           # 106 项验收
```

接进 Web UI：

```bash
# backend/config.yaml → device.mode: "mujoco"
./backend/bin/armpilot-backend.exe                  # 或 go run ./backend
curl http://localhost:8090/healthz                  # {"device":"mujoco","linked":true,...}
# 前端连上后即得到物理仿真末端；hello.simulation_mode == "mujoco"
```

---

## 4. 判据纪律（血泪换来的，改本目录前必读）

这一节是本目录最有价值的部分 —— 每条都对应一次**看起来正常、实际是假的**结论。

### 4.1 MuJoCo 工具层

| # | 事实 | 后果 |
|---|------|------|
| 1 | **运行时改 `model.geom_pos` / `geom_size` 完全无效** | `mj_forward` 不重算静态 geom 的 `data.geom_xpos` 与 broadphase AABB。实测把工作台从 z=15mm 抬到 z=100mm，臂的稳态位置与接触对**一字不变**。⇒ 变体场景必须用 `mujoco.MjSpec` 在**加载期**改 XML（`MeArmSim(xml_text=...)`） |
| 2 | 但 `geom_contype` / `geom_conaffinity` **运行时改有效** | 碰撞过滤是逐对查询掩码，不走 AABB 缓存。⇒ 开关某个碰撞体可以用掩码 |
| 3 | `dist > 0` 的软接触**仍会施力** | 下压位形实测 `dist=+1.353mm`，但 `qfrc_constraint=[0,−0.23589,−0.12266,0]`、执行器满力矩 0.1765、禁用台面后 TCP 掉 14.6mm。⇒ **"接触表里有记录"和 `dist` 都不是证据** |
| 4 | `mj_geomDistance` 对 infinite plane **不可信** | 返回 +4.74mm 而臂在平面下方。判穿透要用 `data.contact.dist` 或对照实验 |
| 5 | `mjtDisableBit` 是 **enum，不能直接 `~`** | `TypeError`；必须先 `int(...)` 再取反 |
| 6 | **`passive` 关节有 `<joint>` 却不算自由度** | `tool` 是 hinge：`mj_name2id(mjOBJ_JOINT,"tool")` 返回有效 id，`nq` / `njnt` 都把它算进去，但它**不进** `joint_order()`。⇒ 凡"按关节顺序取值"的地方都必须走 `joint_order()` 重排（`dof_vector()` / `ctrl_vector()` / `constraint_torque()`）；直接拿 `joint_ids`（5 个）去 zip 状态帧（4 个）会把 gripper 读成 tool 的值 —— **量纲相同、不报错** |
| 6b | 锁**绝对角**只能用 tendon，`<equality><joint>` 是错的 | 串联网里某关节的"绝对转角" = 从根到它、且转轴与之共线的**全部局部角之和**，而 `qpos` 存的是**局部角**。`<equality><joint joint1="tool" joint2="elbow">` 只写了 `q_elbow`，少一项 `q_shoulder` ⇒ **残差恰为 `homePose.shoulder`**（0.85°），看着"差不多对"，其实整个姿态都偏了。正解：`<tendon><fixed>` 组 `q_shoulder + q_elbow + q_tool` + `<equality><tendon polycoef="1.5707… 0 0 0 0">` 钉在 90° |
| 7 | `np.allclose` 的 `atol` 会被 `rtol` 淹没 | 比较 1e2 量级的坐标时 `atol=1e-6` 的真实容差 ≈1e-3。**纯运动学量必须用 `np.array_equal`** |
| 8 | Windows `time.sleep` 粒度 1~2ms | 物理循环按 chunk（10ms）睡一次；逐步睡 1ms 会让仿真慢一个数量级，"实时"名存实亡 |

### 4.2 机构 / 控制层

| # | 事实 | 后果 |
|---|------|------|
| 9 | **`elbow` 的合法域是斜的** | 它存绝对倾角且与 `shoulder` 耦合（gain=−1），MuJoCo hinge 的 qpos 是**局部角**。外接区间 `[58.99, 147.95]` 严格大于合法域，**内切是空集**。⇒ **hinge range 只承担数值保护**，限位一致性必须由 Go controller + `limits.py` 保证 |
| 10 | 四对限位**恰好等价** | `S9 [30,150]` · `S7 [80,160]` · `S8 [20,100]` · `S6 [40,130]`，关节限位换算到舵机后无一越界 ⇒ 真配置下构造不出"关节入限但舵机出限" |
| 11 | **`elbow-down` 支恒不可行** | `θe = θs + α` 且 `elbow.limit_min (108.44°) > shoulder.limit_max (49.45°)` ⇒ α 恒 > 58.99° > 0。⇒ 只有 elbow-up 一支存在（不是"偶尔解不出来"） |
| 12 | **可达工作空间的内锥是空的** | `dr = off_r + l1·sin θs + l2·sin θe` 在合法域上的最小值在角点 `(θs_min, θe_max)`，实测 **80.92 mm > 0**（`off_r` = 腕枢轴→TCP 的常量径向偏移 = 40mm，爪锁水平的产物）。⇒ 真机**够不到自己的中轴线**；`ik.ts` 里「方位角不定」那段分支在本机上不可达 |
| 13 | 控制周期累加器**必须是实例状态** | 写成局部变量 ⇒ `step(1)` 连续调用时累加器每次从 0 开始、永远到不了 10ms 阈值 ⇒ **ctrl 永不更新 ⇒ 命令完全不生效**；而 `step(200)` 那次调用却一切正常。表现为"用 `settle()` 测就全挂、用 `step(200)` 测就全过" |
| 14 | `settle()` 判据必须"**持续**静止" | `reset()` 后 `qvel` 恒为 0，单步判据会立刻返回 ⇒ 所有"下命令然后 settle"的测试都读到**从未移动过的**初始位形 |
| 15 | 稳态误差必须与「(重力矩 **+ 约束矩**) ÷ kp」**自洽** | `qpos = ctrl − τ/kp`。实测肩 err 0.2875°（与预测残差 2.75 mrad，容差 3 mrad）。⚠️ `τ` 必须**同时**含重力项与约束项：被动腕的"绝对角锁定"是一条跨 `shoulder/elbow/tool` 的等式约束，物理上等价一根刚性连杆，会把力矩**传回被驱动的肩/肘**；漏掉它这条断言会在容差边缘反复横跳 |
| 16 | 惯量**双重单位换算**是静默的 | `physics.yaml` 已声明全 SI，生成器再做一次 `mm2m()` ⇒ 惯量掉到 1e-11，`%.6f` 后**显示成 "0"**。修法：不换算 + 格式化改用 `%g` |

### 4.3 协议层（跨语言）

| # | 事实 | 后果 |
|---|------|------|
| 17 | Go `ParseReply` **只认 3 种形状**：`ERR` / `STATE` / `OK JR` | `STATUS` / `OK PING` / `OK RESET` 全部落到 `ReplyOther`。⇒ **`STATUS` 不是状态通道，唯一的状态通道是 `STATE` 主动帧** |
| 18 | 握手必须是显式 PING → `OK PING` | 没有它，"解释器不对 / mujoco 没装 / XML 编译失败"会以"设备可用但永远没有回执"的形式出现，上层只看到 ACK 超时 —— 最难查的一种失败 |
| 19 | `pump_once` 必须**先 drain 再判 stop** | 写成"先判 stop"时，stdin 被管道喂完立刻 EOF ⇒ `_stop` 在队列被消费前就置位 ⇒ **丢命令** |

---

## 5. 验收数据（Phase 1–10，spec §36 / §42）

```
生成器        gen_model.py              9312 B · nq=5 nv=5 njnt=5 nbody=8 ngeom=34 nu=4 nexclude=5 neq=1 ntendon=1
模型总质量    Σ body_mass               0.1415 kg（2026-09-15：base/column 的 `[估算]` 质量按
                                         STEP 实测几何重算，0.2173 → 0.1415；见 physics.yaml §4）
HOME 位稳态   MuJoCo                     TCP=[115.03, 0, 109.22] mm · ncon=0（无伪接触）
重力测试     无驱动 3s（有/无重力对照）  Δ shoulder 32.095° / elbow 30.990°；Δ base = Δ gripper = 0.000°
                                        （被动腕把爪锁平后 gripper 铰轴变为世界竖直 ⇒ 与 base 一样重力矩恒为 0）
                                        关重力对照：max|Δ| = 1.574e-10°（阈值 1e-6）
稳态误差自洽 qpos == ctrl − τ/kp        肩 err 0.2875°（τ = 重力矩 + 约束矩）· 残差 2.75 mrad < 3 mrad 容差
下压位形      shoulder=49.4549 / elbow=141.8582
              TCP z=62.35 mm · ncon=1 · jaw↔table dist=+1.353 mm（无穿透）
              qfrc_constraint=[0, −0.23589, −0.12266, 0] · qfrc_actuator[肩]=0.1765（满力矩）
              禁用台面 ⇒ TCP z 由 62.35 掉到 47.73 mm
              （Δz = −14.62 mm = 台面顶 46 − 包络最低 32.73 + 软接触间隙 1.35）
自碰撞几何距  base↔column 0.000 / column↔upper_arm −0.001 / upper_arm↔forearm −10.438 /
              forearm↔tool −0.446 / tool↔jaw −5.000 mm（4 对重叠 ⇒ exclude 的必要性实测）
摩擦          单一组合 (1.0, 0.005, 0.0001)，全部来自 physics.yaml 的 contact.friction
接触稳定性    5s 漂移 0.0108°
FK 交叉验证   零位 [40, 0, 220] · HOME [115.0335, 0, 109.2236] · 120 随机 + 256 角点
              → max|Δ| = 7.105e-14 mm（阈值 1e-6）
IK 交叉验证   120 随机可达点 → 前端真实 ik.ts → MuJoCo 复算
              success = 120/120 (100%) · max = 9.948e-14 mm · mean = 3.426e-14 mm
              （目标点由 MuJoCo FK 生成 ⇒ 每一步都可独立复核）
前端 fk.ts    与 MuJoCo 逐点对撞            max = 8.527e-14 mm（376 个点，含 256 限位角点）
越限诊断      几何可达但限位不允许          IK 报 JOINT_LIMIT（不是 OUT_OF_WORKSPACE），最接近支差 55.000°
可达内锥      合法域内最小水平半径          80.9164 mm（角点 θs=−6.0937°, θe=141.8582°）
时间步分层    1kHz / 100Hz / 33.3ms        step(1)×10 ≡ step(10)（按位）；第 10 步限速目标走 1 格
渲染解耦      fps=5 vs fps=240             qpos 按位相同
复位可重复    reset × 2 + 热启动泄漏检查    qpos/qvel 按位相同
跨进程确定性  run.py --demo 跑两遍         12 行数值载荷逐字相同（seed=0）
记录          JSONL / CSV 回读             列与配置一致 · 数值等于当时状态 · sim_time 严格递增
快速运动      Test D 峰值角速度            4.9165 rad/s（限速目标 10 rad/s，0.49×）
 ─── pytest 151 passed（13 文件）· tests/sim2sim 9 passed · go test 76 passed · vitest 434 passed · tsc 0 error · vite build OK ───
```

误差门槛来源：spec §20 要求 FK < 1 mm（初期 < 5 mm）。实测是 **1e-13 mm** 量级，
比要求严 10 个数量级 —— 因此断言门槛收到 1e-6 mm，任何真实的几何或旋转约定错误
（intrinsic XYZ 写成 extrinsic、耦合项漏掉）都会立刻冲到毫米级。

---

## 6. 三层时间步（spec §23 / §24）

| 层 | 周期 | 配置项 | 实现位置 |
|----|------|--------|----------|
| 物理积分 | 1 ms | `timestep.physics` | `model.step()` → `mj_step` |
| 控制环 | 10 ms | `timestep.control` | `model.step()` 内部的 `_ctrl_accum` |
| 渲染 | 33.3 ms | `timestep.render` | `run.py` 里**只在到点时 `viewer.sync()`** |

**渲染帧率不能决定物理步长** —— `run_loop` 的批大小与步长都不依赖 `fps`，
这条由 `test_render_fps_does_not_change_physics` 钉住。

---

## 7. 能力边界与未实现项

**已实现**：刚体动力学 · 位置控制 · 舵机速率限制 · 关节摩擦/阻尼/armature ·
碰撞（臂↔地面 / 臂↔工作台 / 五对相邻连杆的自碰撞排除）· 确定性复位 · 数据记录 ·
独立 Viewer · 配置化参数 · 标定接口 · Sim2Real 预留接口（`Observation/Action/Reset/Step/State`
对应 `state()` / `set_target_joints()` / `reset()` / `step()`）。

**未实现（本阶段明确禁止，spec §30）**：
AI · LLM · Agent · 强化学习（PPO/SAC）· 自训练 · 视觉学习 · 数据集训练 · 策略训练 ·
自动数据采集。

**已知缺口**：
1. `physics.yaml` 的值是公开值/估算值 —— 见 §0。
2. 真机的舵机死区、齿轮间隙、电池电压漂移、线缆弹性**都没有建模**。
3. 自碰撞只对**五对相邻连杆**做了 exclude，其余连杆对靠几何距离自然过滤
   （实测相邻连杆确实互相重叠，所以那五对必须 exclude；非相邻对的间距都为正）。
4. `ik.ts` 的「方位角不定」分支在本机不可达（见 §4.2 #12）—— 分支本身没问题，
   但**不能当作已覆盖的路径**。

---

## 8. 相关文档

- `docs/ARCHITECTURE_ANALYSIS.md` —— Phase 1 架构勘察（自由度清点、五种角度对照、接入方案）
- `docs/decisions.md` **D48–D54** —— 本目录的关键决策
- `robot-package/mearm-v1/model/robot.yaml` —— 运动学真值（唯一来源；Phase 2 起随包走）
- `robot-package/mearm-v1/physics/physics.yaml` —— 物理参数（**只放物理量**，禁写限位与标定）
- `docs/model-structure.md` —— 显示几何与运动学层的边界
