# ArmPilot 对外控制协议（总索引）

> 本目录是 **ArmPilot / MeArm-3D 对外控制接口的唯一协议参考**。
> 面向**外部项目**（当前主要是 **visionflow** 的手势控制）实现客户端时使用。
>
> 内部设计理由、决策记录不在这里 —— 那些在 `docs/decisions.md`（ADR `D##`）与
> `docs/tcp-control-v1.md`（已并入本目录，见文末索引）。

---

## 1. 定位：端口扩展 + 协议转换

```
HTTP  /healthz ─┐
WebSocket      ─┼─▶ controller.Apply（关节角整帧）─▶ device ─▶ sim / serial / mujoco
Serial（上位机）─┤        ↑ 限位 · ACK 门控 · latest-wins 全在这里
TCP JSON  ←─────┘
```

TCP 层**只做两件事**：把一行 JSON 翻译成一次 `controller.Apply`，
再把执行后的状态读回来。它不持有 IK 算法、不持有软限位、不判断
`device.mode` —— 换链路（sim / serial / mujoco）本接口一个字都不用改。

因此：**真机上能做的，TCP 上都能做；真机上被挡下的，TCP 上也一样被挡下。**

---

## 2. 版本

| 版本 | 内容 | 文档 |
|------|------|------|
| **v1** | `move` / `gripper` / `servo` / `state` | [`tcp-v1.md`](./tcp-v1.md) |
| **v2** | 追加 `movexyz` / `moveto` / `home` / `caps`（**XYZ 矢量**） | [`tcp-xyz-v2.md`](./tcp-xyz-v2.md) |

- 服务端当前回报 `caps.protocol = 2`。
- v2 **只新增命令**，不改 v1 任何一条的语义、参数、错误文案与错误优先级
  （判据：`backend/internal/tcpserver/protocol_xyz_test.go`
  的 `TestMove_SingleAxisEqualsMoveXYZ` 逐位比对单轴 `move` 与 `movexyz`）。
- **v1 客户端无需任何改动**。未知 `cmd` 只是返回 `ok:false`，连接照常可用。
- 能力探测用 `caps`（[`tcp-xyz-v2.md` §4](./tcp-xyz-v2.md)），**不要**靠版本号猜。

---

## 3. 传输

| 项 | 值 |
|----|----|
| 协议 | TCP |
| 默认端口 | **9100**（刻意避开 8090=WS / 5273=vite / 8080 / 9001） |
| 开启 | `backend/config*.yaml` 的 `tcp.enabled: true`（默认 false），三份运行配置均已开启 |
| 帧格式 | **JSON Lines**：一条命令 = 一行 JSON + `\n`，一问一答（可流水线发送） |
| 编码 | UTF-8，无 BOM；行尾 `\n`（也接受 `\r\n`） |
| 单行上限 | `max_line_bytes`（默认 65536）。超限 ⇒ **断开该连接**，不影响其它连接 |
| 未知字段 | 忽略 |
| 未知 `cmd` | `ok:false` + `unknown cmd "x"`，**连接保持可用** |
| 鉴权 | **无**。假定运行在可信局域网，放到公网前必须加认证 |

> ⚠️ TCP 是**独立端口**，可与后端的 HTTP/WS 同时提供；但同一个 9100 只能被
> 一个后端实例占用。三条链路（sim / mujoco / serial）共用 8090 ⇒ 不能同时起，
> 但它们的 TCP 端口都是 9100，切链路不用改客户端。

---

## 4. 坐标系与单位

出处 `docs/coordinate-system.md` §1 / §3，`caps` 也会把这一组回传给客户端。

| 项 | 值 |
|----|----|
| 手性 | 右手系，**Z-up** |
| +X | 正前方 |
| +Y | 左 |
| +Z | 上 |
| 长度单位 | **mm** |
| 角度单位 | **degree** |
| 原点 | 肩枢轴，在底盘平面上 |
| 夹爪 | θ 增大 = **张开**（`open` = 限位 max，`close` = 限位 min） |

两条最容易错、且错了不会报错的机构事实：

1. **肘是绝对角**（`θe = θs + α`），不要再叠加肩角（ADR **D18**）。
2. **腕是被动关节**，爪被四连杆锁成水平 ⇒ "腕枢轴 → TCP" 是一段**恒定水平
   偏置** `toolR`（本机型 40mm）。IK 必须先扣掉它再解平面 2R ——
   **L2 ≠ 120**，机构是"两根 80 的杆 + 一段 40 的水平爪"。

---

## 5. 命令总表

| `cmd` | 版本 | 语义 | 参数 | 写/只读 |
|-------|------|------|------|---------|
| `move` | v1 | **单轴**相对位移 | `axis` `direction` `step` | 写 |
| `gripper` | v1 | 夹爪开合到限位端点 | `action` | 写 |
| `servo` | v1 | 直接给某个舵机绝对角 | `servo` `angle` | 写 |
| `state` | v1 | 读当前状态 | — | **只读** |
| `movexyz` | v2 | **三轴同时**相对位移 | `delta: [dx,dy,dz]` | 写 |
| `moveto` | v2 | **绝对**目标点 | `xyz: [x,y,z]` | 写 |
| `home` | v2 | 回 HOME 位姿 | — | 写 |
| `caps` | v2 | 读几何 / 限位 / 行程 / 可达包围盒 | — | **只读** |

---

## 6. 应答格式（各命令通用）

成功：

```json
{"ok":true,"cmd":"movexyz","state":{
  "joints":{"base":-1.43,"elbow":110.94,"gripper":50,"shoulder":3.84},
  "servo":[88.57,94.31,94.01,90],
  "tcp":[120.033,-3.0,111.224],
  "device":"sim"}}
```

失败：

```json
{"ok":false,"cmd":"movexyz","error":"OUT_OF_WORKSPACE: ..."}
```

| 字段 | 说明 |
|------|------|
| `ok` | 必出现 |
| `cmd` | 回显命令名（小写） |
| `error` | 仅失败时出现。**全部为文本，本协议不引入数字错误码** |
| `state` | 仅成功时出现 |
| `state.joints` | 关节角（degree），键 = `robot.yaml` 的关节 id |
| `state.servo` | 舵机角数组，按 `servo` 命令的编号 **1..N** 排列 |
| `state.tcp` | 末端位置 `[x,y,z]`（mm） |
| `state.device` | 链路末端 `sim` / `serial` / `mujoco` |
| `caps` | 仅 `caps` 命令出现 |

> ⚠️ `state` 返回的是**命令值**（最近一次被受理的目标），**不是设备过程值**。
> 要看设备实际走到哪了，读 WebSocket 的 `joint_state{origin:"device"}`。
> 这一点对做闭环的客户端很重要：命令刚下发就立刻读 `state`，读到的是目标。

---

## 7. 最小接入三步

```bash
# ① 起后端（任选一条链路）
cd backend && ./bin/armpilot-backend.exe -c config.yaml       # sim
cd backend && ./bin/armpilot-backend.exe -c config.mujoco.yaml # mujoco
cd backend && ./bin/armpilot-backend.exe -c config.serial.yaml # 真机（会真的动）

# ② 拿基准 + 能力（连上先做这两件事，别直接下发）
printf '{"cmd":"caps"}\n'  | nc 127.0.0.1 9100
printf '{"cmd":"state"}\n' | nc 127.0.0.1 9100

# ③ 下发
printf '{"cmd":"movexyz","delta":[5,-3,2]}\n' | nc 127.0.0.1 9100
```

**手势控制**的完整接入套路（坐标映射、钳位、节流、丢步处理、示例客户端）见
[`gesture-control.md`](./gesture-control.md)。

---

## 8. 安全与并发

| 项 | 做法 |
|----|------|
| 限位 | 关节限位走 `Model.Validate`；舵机行程走 `Actuator.Limits`。TCP 层**不持有任何角度常数** |
| 不绕过设备 | 命令一律经 `controller.ApplyFrom`，与 WebSocket 同一条路（ACK 门控 / latest-wins 一致） |
| 来源标注 | 全部写命令声明 `origin=external`（见 [`tcp-v1.md` §5.1](./tcp-v1.md)） |
| 多客户端 | 允许并发连接；`Handler` 内互斥量把「读当前 → 算目标 → 下发」做成原子操作 |
| 崩溃隔离 | 每连接独立 goroutine + `recover()`；单连接 panic 不拖垮后端 |
| 非法输入 | `Execute` 对任何输入都返回 `Result`，不返回 error、不 panic |

---

## 9. 验收

| 判据 | 命令 |
|------|------|
| 协议单测（含 v2 全部命令 + v1 未变） | `cd backend && go test ./internal/tcpserver/` |
| 端到端冒烟（**42 项**，含六向位移几何校验、XYZ 矢量、错误轰炸、超长行隔离） | `python core/tools/tcp_control_client.py` |
| 单条命令手测 | `python core/tools/tcp_control_client.py --cmd '{"cmd":"caps"}'` |

> ★ 上面的计数是**现跑现取**的。改代码或改文档前先重跑一遍 ——
> 「上次是绿的」不是证据（本工程已因此踩过两轮）。

---

## 10. 文件索引

| 文档 | 内容 |
|------|------|
| [`tcp-v1.md`](./tcp-v1.md) | v1 四条命令的完整参考、错误码表、`origin` 语义 |
| [`tcp-xyz-v2.md`](./tcp-xyz-v2.md) | v2 的 XYZ 矢量命令、`caps` 应答字段、兼容性说明 |
| [`gesture-control.md`](./gesture-control.md) | **visionflow 手势控制接入指南**（含可直接抄的客户端） |
| `../coordinate-system.md` | 坐标系与关节符号约定（协议里的坐标以此为准） |
| `../serial-v1.md` | 串口 + WebSocket 协议（**另一套**，用于浏览器前端，不是本目录的 TCP） |
| `../decisions.md` | 设计决策 ADR（D18 肘绝对角、D74 幽灵臂、D81 三条链路对等、D82/D83/D84 TCP） |

> 历史文件 `docs/tcp-control-v1.md` 已并入 [`tcp-v1.md`](./tcp-v1.md)，
> 原地只保留跳转指针。
