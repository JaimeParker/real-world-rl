<!-- English source: docs/remote-actor.md -->

<a id="remote-actor-over-the-robot-daemon"></a>

# 通过机器人守护进程运行远程 actor

[中文入口](../README.zh-CN.md) · [English](../remote-actor.md)

GPU 机器上的设备 RPC 和 RemoteRobot 见[机器人守护进程](daemon.md)。

主流程在 NUC 使用本地 `real_robot.robot` 映射及 `arm_backend: franky` 运行录制器或 HIL actor，直接打开机械臂、Hand、相机和 GELLO。**此路径不要启动守护进程。** GPU 机器运行 learner 本身不要求机器人守护进程：actor 可留在 NUC，向 learner 发送训练数据。

仅当 **actor 本身在另一台机器**运行时使用下方守护进程路径，以 `real_robot.remote` 替代 `real_robot.robot`。

| 属性 | NUC 直连（主流程） | 远程 actor（可选） |
|---|---|---|
| 单臂硬件进程 | 一个录制器或 actor 进程 | NUC 守护进程加另一机器的录制器或 actor |
| 连接配置 | `real_robot.robot` 映射 | `real_robot.remote: NUC_ADDRESS:50051`；省略 `robot` |
| 操作员键盘 | 接 NUC；目标终端键见 [00](tutorial/00-overview.md#operator-keys) | actor 机器读取；当前 `pynput` 路径需其桌面会话 |
| 动作看门狗 | 应用检查共享硬件进程 | 客户端停止发送动作时守护进程可停止机械臂 |
| 外部停止 | 监督器可终止卡住进程，再由固件处理控制通信丢失 | 存活守护进程可独立于 actor 发停止 |

整个进程挂起或被终止时，应用线程不能保证 `stop()`。监督器终止直连进程依赖固件通信丢失响应，并非独立应用停止命令。守护进程看门狗也不覆盖守护进程挂起或 NUC 故障。两种布局均需物理急停可用。

<a id="start-a-daemon-only-for-a-remote-actor"></a>

## 仅为远程 actor 启动守护进程

先完成[真实 doctor 握手](tutorial/03-franka-bringup.md)。本节全部命令在 **NUC** 的**仓库根目录**、**激活项目环境**后运行。确保无其他进程占用设备。远程路径需在 NUC 和 actor 均安装 `grpc` 扩展。

```bash
rwrl daemon --station local/station.yaml --watchdog-s 5
```

`--watchdog-s` 默认 **0.5 秒**。上方显式 **5 秒** 为 reset 后首次慢推理留时间；间隔超限时守护进程令机械臂空闲，下一动作失败。按工位选择超时；5 秒只是示例，不保证长暂停安全。

*应看到：* doctor 预检后打印服务地址及机器人部件。保持终端打开，Ctrl-C 停止守护进程。多臂站点每机器人独立守护进程、独立端口；启动器可从站点文件启动两者。

*否则：* 按 [03](tutorial/03-franka-bringup.md) 对应预检提示处理。录制器、actor、守护进程占设备时不要对同设备运行 doctor：占用声明和握手会冲突。doctor 将此进程列为 `BLOCKED`；只有持硬件连接的命令算占用者，离线 `rwrl` 命令如 `workspace show`、`teleop inspect-stop`、`archive inspect` 不阻塞。

启动后检查连接：在此 **NUC** 的**仓库根目录**打开另一终端，**激活项目环境**：

<!-- tutorial-test: skip -->
```bash
rwrl status
```

`local/station.yaml` 声明 `daemon:` 时应显示 `connection: daemon HOST:PORT`。主路径无守护进程声明且无录制器/actor 占臂时，同命令报告 `connection: direct franky`。声明但不可达的守护进程会报错；见 [rwrl 连接归属](cli.md#connection-ownership)。

<a id="point-the-remote-actor-at-the-nuc"></a>

## 将远程 actor 指向 NUC

在 **actor 机器**仓库根目录使用项目环境。在其 `hil.yaml` 移除 `real_robot.robot`，将 `real_robot.remote` 设为 NUC 可达地址和站点守护进程端口（通常 50051）。仅配置 `daemon.token` 时设置 `real_robot.token`。有相机时需提供 `obs.image_size`，因为远程尺寸连接后才知道。保持录制时相同动作模式、50 Hz 和观测契约。

在 actor 项目环境安装 gRPC 客户端：

<!-- tutorial-test: skip -->
```bash
python -m pip install -e '.[grpc]'
```

在 actor 机器仓库根目录、激活项目环境，不连接设备检查配置：

```bash
rwrl train hil_serl --config hil.yaml --teleop-device leader_cartesian --print-config
```

*应看到：* 恰好一个连接源 (`remote`)，正确的相机名、动作模式、控制频率。实际运行时，客户端连接触发守护进程连接硬件。

*否则：* 移除残留 `robot` 映射。运行时连接失败检查守护进程监听地址、网络/防火墙可达性及令牌。键盘事件属于 actor 事件源；只接 NUC 的键盘不会自动成为远程 actor 键盘。避免在两进程独立打开同一个 GELLO，应使用守护进程遥操作部件。

下一步：[08 录制示范](tutorial/08-record-demos.md)，留意 [00](tutorial/00-overview.md#known-limits) 描述的流程及按键迁移限制。
