<!-- English source: docs/tutorial/00-overview.md -->

<a id="0-overview-and-safety"></a>

# 0. 总览与安全

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/00-overview.md)

本教程使用一台带 Franka Hand 的 Franka FR3、一台 NUC 和 GELLO 主臂，从空白 NUC 开始，逐步部署抓取放置策略。每一步都是一条 `rwrl` 命令。本页无需执行命令，介绍系统、机器分工、阅读顺序以及贯穿所有页面的安全规则。

<a id="the-system"></a>

## 系统

```text
                 Ethernet (Control port, not the Arm port)
  +-----------+  172.16.0.1  <------------->  172.16.0.2  +-----------------+
  | NUC       |                                            | Franka control  |
  | (robot    |                                            | box: Desk + FCI |
  |  host)    |  USB: cameras, GELLO, SpaceMouse           +--------+--------+
  |           |                                                     |
  | rwrl      |                                              Franka arm + Hand
  | recorder, |
  | HIL actor |  network (optional): replay data out, policy weights in
  +-----+-----+  <--------------------------------------->  +-----------------+
                                                            | GPU host        |
                                                            | rwrl train      |
                                                            | --role learner  |
                                                            +-----------------+
```

录制器和 HIL actor **在 NUC 上的同一个硬件进程中**运行，通过 franky (`real_robot.robot`) 直接打开机械臂、Hand、相机和 GELLO。learner 可以在 GPU 主机或 NUC 本机运行。仅当 actor 在另一台机器上运行时才需要机器人守护进程，见[通过机器人守护进程运行远程 actor](../remote-actor.md)。

控制契约为 **50 Hz 笛卡尔 GELLO** (`action_mode: cartesian_delta`)。示范、BC/ACT、HIL 和部署必须使用相同的动作模式、频率、相机、状态字段和夹爪契约。

<a id="roles-and-names"></a>

## 角色与名称

| 名称 | 含义 |
|---|---|
| NUC / 机器人主机 | 连接 Control 端口、相机和遥操作设备；运行所有硬件命令 |
| GPU 主机 | 运行 learner；也可以是 NUC |
| `ROBOT_PC` | NUC 的可达地址，仅远程守护进程客户端需要 |
| `LEARNER_IP` | learner 的可达地址；learner 在 NUC 上时为 `127.0.0.1` |

每条命令都注明机器、工作目录和环境。默认在 NUC 的仓库根目录 (`real-world-rl/`)，激活**项目环境**后运行：即[01 安装](01-install.md)创建的 venv 或 conda 环境（默认 `.venv`）。同机服务使用 `127.0.0.1`，远程客户端使用可达地址。

<a id="reading-order"></a>

## 阅读顺序

| 页面 | 操作 | 命令 |
|---|---|---|
| [01 安装](01-install.md) | 安装项目环境，无硬件检查 | `bash scripts/install.sh`, `rwrl doctor --mock` |
| [02 设置站点](02-setup.md) | 电源、网络和 Desk；写入 `local/station.yaml` | `rwrl discover`, `rwrl setup`, `rwrl workspace show` |
| [03 启动 Franka](03-franka-bringup.md) | 真实握手；阅读 doctor；读取状态 | `rwrl doctor`, `rwrl status`, `rwrl q`, `rwrl tcp`, `rwrl recover` |
| [04 Home、夹爪、相机](04-first-motion.md) | 首次运动；检查每个设备 | `rwrl home`, `rwrl gripper`, `rwrl camera`, `rwrl stop` |
| [05 遥操作：GELLO](05-teleop-gello.md) | 检查主臂 | `rwrl teleop read`, `rwrl teleop calibrate` |
| [06 遥操作：SpaceMouse](06-teleop-spacemouse.md) | 检查 SpaceMouse | `rwrl teleop diagnose`, `rwrl teleop profiles` |
| [07 遥操作：主臂/从臂](07-teleop-leader-follower.md) | 关节空间主臂 | `rwrl teleop read --with-follower` |
| [08 录制示范](08-record-demos.md) | 受监督的 GELLO 或 SpaceMouse 录制 | `rwrl supervise`, `rwrl record` |
| [09 基础策略](09-base-policy.md) | 从示范训练 BC 或 ACT | `rwrl train real_world_bc` |
| [10 HIL 训练](10-hil-training.md) | HIL-SERL、residual 或 SiLRI 的 learner 与 actor | `rwrl train` |
| [11 部署](11-deploy.md) | 按保存的契约运行检查点 | `rwrl deploy` |

录制和训练时，应在机器人旁打开[操作员页面](../intervening.md)。使用双臂时，结合 [bimanual.md](../bimanual.md) 的调整阅读本教程。

<a id="status-words"></a>

## 状态词

`rwrl doctor`、`rwrl setup` 和守护进程预检均以状态词报告每项检查。终端中状态词带颜色；管道输出或 `NO_COLOR=1` 时使用纯文本。

| 状态 | 含义 | 颜色 | 是否导致运行失败 |
|---|---|---|---|
| `PASS` | 已检查且正常 | 绿色 | 否 |
| `WARN` | 正常，但需阅读提示 | 黄色 | 否 |
| `SKIPPED` | 未检查；不能证明可用 | 灰色 | 否 |
| `FAIL` | 已检查且异常 | 红色 | 是 |
| `BLOCKED` | 无法检查或修复（设备占用、缺少权限） | 品红色 | 是 |
| `PENDING_ACTION` | 已执行修复，需你继续操作（重新登录、重新插拔） | 黄色 | 是 |

详细说明和检查列表见 [rwrl doctor](../doctor.md)。

<a id="safety"></a>

## 安全

- 机械臂通电时，保持工作区畅通，确保急停触手可及。
- 首次运动前，对照实际机械臂和工位核查站点的 `home_joints`、`joint_limits`、`safety_box` 和 `max_delta_per_step`（[02](02-setup.md#4-check-limits-and-home)）。向导默认值仅是起点，未经现场验证。切勿将 home 姿态复制给另一台机械臂。
- 运动命令 (`home go`, `gripper open/close/move`) 先打印目标，再询问 `proceed? [y/N]`；`--yes` 自动确认。确认前先阅读目标。
- `rwrl stop` 不经询问便停止机械臂。直连路径要求机械臂空闲：先结束录制器或 actor，或使用物理停止装置。
- 若整个直连进程挂起，没有独立看门狗；监督器可将其终止，再由固件处理连接丢失。守护进程看门狗能停止静默 actor，但不覆盖守护进程或 NUC 故障。
- doctor 中的 `BLOCKED` 表示另一个进程占用设备。doctor 不会停止、终止或接管那个进程，需自行结束。
- 站点配置遥操作设备后，该设备必须能连接。无人值守部署使用不含 `teleop` 的站点。

<a id="operator-keys"></a>
操作按键（接管、交还、成功、失败、重录、退出）见[操作员页面](../intervening.md)。

<a id="known-limits"></a>

## 已知限制

- 硬件调试覆盖单臂 FCI 握手和 GELLO 录制会话；完整双臂及在线 RL 运行、真实录制数据上的 ACT 训练和完整部署路径尚未验证。
- 关节示范不能重新标为笛卡尔示范。
- `task.name: none` 时环境返回奖励 0；成功来自操作员标签或奖励分类器。
- actor outbox、learner journal 和检查点包应保存在本地存储。

详见[已知问题](../known-issues.md)和[真实世界实验](../real-world-experiments.md)。

下一步：[01 安装](01-install.md)。
