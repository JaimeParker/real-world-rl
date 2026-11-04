<!-- English source: docs/tutorial/00-overview.md -->

<a id="0-overview-and-safety"></a>

# 0. 总览与安全

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/00-overview.md)

本教程从一台 NUC 和一台 Franka FR3 开始，先完成真实 doctor 握手，再准备使用 GELLO leader 的抓取放置实验。主线由 NUC 上**一个硬件进程**运行 recorder 或 HIL actor，通过 `real_robot.robot` 直接连接 franky。GPU learner 可以单独运行；只有 actor 在另一台机器上时才使用机器人守护进程，并改用 `real_robot.remote`。

控制契约为 **50 Hz 笛卡尔 GELLO**。示范、BC/ACT、HIL 和部署须保持动作模式、频率、相机、状态字段及夹爪契约一致。`runs/bc-pick-and-place`、`runs/hil-pick-and-place` 等名称标识不同实验；只有恢复同一次实验时才复用其身份和本地存储。

<a id="the-pipeline"></a>

## 流程

```text
New cell: power --> Control network --> Desk control + FCI --> install
                                                               |
Already commissioned: take Desk control --> confirm FCI --------+
                                                               v
                     station.yaml --> real doctor handshake
                                                               |
                       gello_cartesian.yaml (NUC, direct) <-----+
                            |                    |
                   record GELLO demos     remote actor needed?
                            |                    |
                       BC or ACT          optional daemon + remote:
                            |                    |
                            +-----> HIL actor + learner --> deploy
```

| 页面 | 要做的事 |
|---|---|
| [01 安装](01-install.md) | 上电、接线、持久网络、Desk 六项检查、安装和 doctor |
| [02 工作站](02-station.md) | 逐臂选择硬件、标定 GELLO、检查限位并准备直接连接的 `hil.yaml` |
| [03 进程布局](03-daemon.md) | NUC 单硬件进程；仅远程 actor 配置守护进程 |
| [04 录制示范](04-record-demos.md) | 在监督下使用 GELLO 或 SpaceMouse 录制 |
| [05 基础策略](05-base-policy.md) | 用兼容示范训练 BC 或 ACT |
| [06 人工介入](06-intervening.md) | 按键、有界跟随、夹爪锁存和记录数据 |
| [07 HIL 训练](07-hil-training.md) | HIL-SERL、残差 HIL-SERL 或 SiLRI 的 learner 与 actor |
| [08 部署](08-deploy.md) | 按保存的观测/动作契约运行 checkpoint |

<a id="names-used-in-the-commands"></a>

## 命令中的名称

| 名称 | 含义 |
|---|---|
| NUC / 机器人 PC | 连接 Control 口、相机和 GELLO；默认直接运行 recorder/actor |
| `ROBOT_PC` | NUC 可达地址，仅远程守护进程客户端需要 |
| `LEARNER_IP` | GPU learner 的可达地址，也可以是 NUC |
| actor 机器 | 默认 NUC；仅可选远程路线使用另一台机器 |

每条命令注明机器、工作目录和 venv。默认是 NUC、仓库根目录 `real-world-rl/`，安装后激活 `.venv`。同机服务使用 `127.0.0.1`；远程客户端必须使用可达地址。

<a id="action-mode"></a>

## 动作模式

主线使用 `action_mode: cartesian_delta`、`control_hz: 50` 和 `tcp_pose` 反馈。GELLO 关节读数必须经过笛卡尔映射 `leader_cartesian`，不能作为关节命令直接送给笛卡尔环境。先标定符号和偏移，再验证运动方向和尺度。50 Hz 是请求的应用更新率，不保证 SDK 伺服时序或实际 USB 采样吞吐量。

04–07 使用 `examples/presets/gello_cartesian.yaml`：仅状态观测、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`，采用本地 robot 映射。录制和硬件 actor 命令由 `robot_infra.supervise` 监督。关节空间示例属于其他模式，需要各自兼容的示范和基础策略。双臂先做[逐臂 Desk 与向导检查](02-station.md#multiple-arms)，再读 [bimanual.md](../bimanual.md)。

<a id="operator-keys"></a>

## 操作员按键

NUC 终端 `stdin` 和桌面 `keyboard` 事件源使用相同按键：

| 按键 | 作用 |
|---|---|
| `t` | GELLO 接管，锚定相对映射；不是切换键 |
| `h` | GELLO 交还；录制器保持实测位姿，HIL 返回策略控制 |
| `s` | 标记成功并结束回合 |
| `f` | 标记失败并结束回合 |
| `r` | 录制器丢弃并重录回合 |
| Esc | 无成功标签地结束当前回合 |
| `q` | 停止运行并关闭设备 |

SpaceMouse 用按键 0 接管/交还、按键 1 控制夹爪；终端 `t`/`h` 被忽略。只推动推杆不会接管，结果标签和退出仍走终端。见 [06](06-intervening.md)。终端 stdin 无需 evdev 权限；桌面键盘源需要 actor 的桌面会话。

<a id="safety"></a>

## 安全

保持工作区清空，物理急停触手可及。运动前按当前实臂和工位核对 `home_joints`、`joint_limits`、`safety_box`、`max_delta_per_step`。向导默认值只是起点，不是已经验证的 FR3 限位或 home 路径。直接连接和守护进程路线均由设备层执行限位，环境只能收紧。不可将记录的 home 复制给另一台臂。

直接连接的整个进程挂起时无法独立发 stop；监督进程可杀死它，此后依赖固件的通信丢失响应停机。独立且仍存活的守护进程能停止失去响应的 actor，但其 watchdog 不覆盖守护进程或 NUC 故障。见[进程比较](03-daemon.md)。

配置了 teleop 的机器人需要该设备才能连接。无人部署使用匹配且没有 `teleop` 的工作站；辅助部署保留操作设备及其契约。

<a id="known-limits"></a>

## 已知限制

- 实机已完成左臂 FCI 握手和单臂 GELLO 录制，以及独立右 Hand 检查；不代表完整双臂或在线 RL 验证。笛卡尔尺度、跟随滞后和整个进程失效时的 watchdog 限制仍需关注。
- 旧关节示范不能直接改标签冒充笛卡尔示范。物理录制上的 ACT 训练及完整在线训练/部署流程仍待验证。
- `task.name: none` 时环境奖励为零且不检测成功；须使用明确的操作员标签或奖励分类器。正奖励本身不等于自主成功。
- actor 上传与 learner 入库使用本地 SQLite。outbox、journal、checkpoint bundle 应放在本地存储；进程崩溃恢复与断电持久性有不同保证。见[真机实验](../real-world-experiments.md)和[已知问题](../known-issues.md)。

下一步：[01 从上电到 doctor](01-install.md)，或[已配置机器快捷流程](01-install.md#a-robot-someone-else-has-already-commissioned)。
