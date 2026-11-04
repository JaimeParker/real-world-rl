<!-- English source: docs/tutorial/03-daemon.md -->

<a id="3-process-layout-and-optional-daemon"></a>

# 3. 进程布局与可选守护进程

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/03-daemon.md)

主线在 NUC 使用本地 `real_robot.robot` 映射和 `arm_backend: franky` 运行 recorder 或 HIL actor，由该进程直接打开机械臂、Hand、相机和 GELLO。**此路线不要启动守护进程。**GPU learner 单独运行也不需要机器人守护进程；actor 仍可留在 NUC 并上传训练数据。

只有 **actor 本身在另一机器**，用 `real_robot.remote` 替换 `real_robot.robot` 时才使用以下可选路线。

| 属性 | NUC 直接连接（主线） | 远程 actor（可选） |
|---|---|---|
| 单臂硬件进程 | 一个 recorder 或 actor | NUC 守护进程，加另一机器的 recorder/actor |
| 连接配置 | `real_robot.robot` 映射 | `real_robot.remote: NUC_ADDRESS:50051`，移除 `robot` |
| 键盘 | 接 NUC，目标终端键见 [00](00-overview.md#operator-keys) | 在 actor 机器读取；当前 `pynput` 需要桌面 |
| 动作 watchdog | 应用检查与硬件同进程 | 客户端停止动作后守护进程可停臂 |
| 外部停止 | 监督者杀死卡住的进程，随后固件处理控制通信丢失 | 存活的守护进程可独立于 actor 发 stop |

应用线程无法保证整个进程挂起或被杀时执行 `stop()`。监督者杀直接进程依赖固件的通信丢失响应，不是独立的应用 stop 命令。守护 watchdog 也不覆盖其自身或 NUC 挂起。两种布局都保留物理急停。

<a id="start-a-daemon-only-for-a-remote-actor"></a>

## 仅远程 actor 启动守护进程

先完成[真实 doctor 握手](01-install.md#5-configure-once-then-run-doctor)。本节所有命令在 **NUC、仓库根、`.venv` 激活**时执行，确保没有其他进程占设备。NUC 和 actor 均需 `robot_infra[grpc]`。

```bash
python -m robot_infra.server --config local/station.yaml --watchdog-s 5
```

`--watchdog-s` 默认 **0.5 秒**，示例显式 **5 秒**允许 reset 后首次推理较慢。间隔超限时守护进程将臂置 idle，下一动作失败。按现场选择，5 秒只是示例，不保证长暂停安全。

*应该看到：* preflight doctor 后打印服务地址和部件。保持终端打开；Ctrl-C 停守护进程。多臂 station 每臂启动一个独立端口进程，launcher 可从同一文件同时启动。

*否则：* 按 [01](01-install.md) 的 preflight 提示处理。recorder、actor 或守护进程占设备时不要对同设备跑 doctor，否则 claim 和握手冲突。

<a id="point-the-remote-actor-at-the-nuc"></a>

## 将远程 actor 指向 NUC

在 **actor 机器的仓库根、已安装的项目 venv** 中，将 `hil.yaml` 的 `real_robot.robot` 移除，`remote` 设为 NUC 可达地址及 station 端口（通常 50051）。仅配置了 `daemon.token` 才设 `real_robot.token`。相机需 `obs.image_size`，远程尺寸连接后才知道。动作模式、50 Hz 和观测契约均与录制一致。

在 actor 的项目 venv 安装 gRPC 客户端：

<!-- tutorial-test: skip -->
```bash
python -m pip install -e 'robot_infra[grpc]'
```

在 actor 机器、仓库根、项目 venv 中，无连接检查：

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

*应该看到：* 唯一连接源 `remote`、正确相机、动作模式和频率。实跑时客户端连接才触发硬件连接。

*否则：* 移除残留 `robot`。运行连接失败时查监听地址、网络/防火墙和 token。键盘事件属于 actor 事件源，只接 NUC 的键盘不会自动成为远程键盘。不要两个进程各自打开同一 GELLO，应使用守护进程的 teleop 部件。

下一步：[04 录制](04-record-demos.md)，并注意 [00](00-overview.md#known-limits) 的工作流和按键迁移限制。
