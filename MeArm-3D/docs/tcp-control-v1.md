# TCP JSON 控制接口 v1（已并入 `docs/protocol/`）

> ⚠️ **本文已迁移。** 协议文档统一整理到 [`docs/protocol/`](./protocol/README.md)：
>
> | 内容 | 新位置 |
> |------|--------|
> | 总索引（传输 / 坐标系 / 命令总表 / 验收） | [`docs/protocol/README.md`](./protocol/README.md) |
> | v1 四命令（move / gripper / servo / state）+ 错误码表 + `origin` 语义 | [`docs/protocol/tcp-v1.md`](./protocol/tcp-v1.md) |
> | v2 XYZ 矢量命令（movexyz / moveto / home / caps） | [`docs/protocol/tcp-xyz-v2.md`](./protocol/tcp-xyz-v2.md) |
> | 手势控制接入指南（visionflow） | [`docs/protocol/gesture-control.md`](./protocol/gesture-control.md) |
>
> 保留本文只是为了不打断既有链接。**不要再在这里改内容** —— 协议只有一份真值，
> 改请改 `docs/protocol/` 下对应文件。

## 为什么迁

`docs/protocol/` 是给**外部项目**（visionflow 的手势控制）看的完整协议参考，
TCP 只是其中一部分。散在 `docs/` 根下会出现两份互相漂移的协议描述 ——
本工程已经因此踩过（同一批验收计数曾漂移成四个版本）。
