<a id="0-overview-and-safety"></a>

# 0. 总览与安全

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/00-overview.md)

本教程从尚未配置的机器人 PC 开始，带你在一台 Franka 机械臂上部署抓取物体并放入箱子的策略。它使用一套经机器人守护进程读取的 GELLO 主臂，并用 actor 机器的键盘接收操作员按键。每一页都说明要运行什么、预期看到什么，以及结果不符时如何处理。

全文使用的任务是**抓取放置**：一个物体、一个箱子。运行目录以 `*-pick-and-place` 命名，例如 `runs/bc-pick-and-place`、`runs/hil-pick-and-place` 等。

<a id="the-pipeline"></a>

## 流程

```text
01 install --> 02 station.yaml + hil.yaml --> 03 daemon + doctor
    --> 04 record demos (GELLO + keyboard)       demos/pick-and-place.pkl
    --> 05 base policy: BC or ACT                runs/bc-pick-and-place, runs/act-pick-and-place
    --> 06 how to intervene (read before 04 and 07)
    --> 07 HIL training: hil_serl | residual_hil_serl | silri
    --> 08 deploy a checkpoint
```

| 页面 | 需要完成的工作 |
|---|---|
| [01 安装](01-install.md) | 核对硬件、网络与权限，并按机器角色安装依赖。 |
| [02 工作站](02-station.md) | 运行配置向导，完成 GELLO 标定，检查设备限制并创建 `hil.yaml`。 |
| [03 守护进程](03-daemon.md) | 发现设备、运行 doctor、启动机器人服务，并进行无硬件演练。 |
| [04 录制示范](04-record-demos.md) | 将成功的抓取放置示范录制到 pkl 文件。 |
| [05 基础策略](05-base-policy.md) | 使用示范训练 BC 或 ACT。 |
| [06 人工介入](06-intervening.md) | 操作员指南：按键、接管、记录内容与介入时机。 |
| [07 HIL 训练](07-hil-training.md) | 为 `hil_serl`、`residual_hil_serl`（BC 或 ACT 基础策略）和 `silri` 启动 learner 与 actor，并观察训练进展。 |
| [08 部署](08-deploy.md) | 在无 learner、无操作员的情况下执行 checkpoint。 |

<a id="names-used-in-the-commands"></a>

## 命令中的名称

| 名称 | 含义 |
|---|---|
| `ROBOT_PC` | 连接机械臂、相机和 GELLO 的机器，运行机器人守护进程。 |
| `LEARNER_IP` | 运行 learner 并训练 BC 和 ACT 的 GPU 机器，也可以与机器人 PC 共用。 |
| actor 机器 | 连接操作员键盘、运行 actor 和录制器的机器，通常也是机器人 PC。 |

单机运行时，将命令中的 `ROBOT_PC` 和 `LEARNER_IP` 都替换为 `127.0.0.1`。所有命令均从项目根目录（`real-world-rl/`）运行。

<a id="action-mode"></a>

## 动作模式

主流程在所有阶段使用 `action_mode: joint_delta`：录制示范、基础策略、每次 HIL 训练和部署必须使用相同的动作模式。learner 会拒绝约定附属文件与环境不符的示范 pkl，残差基础策略也必须在环境动作空间中输出动作。项目同时支持 `joint_abs`；录制和基础策略页面会用 **joint_abs** 提示说明差异。

双臂流程的逐步修改方式见 [bimanual.md](../bimanual.md#two-arms-the-tutorial-steps)。

<a id="safety"></a>

## 安全

将最初几次运行视为设备调试：

- 保持一只手能够操作急停，清空工作区，并在进入下一步前阅读当前步骤的输出。
- 首次运动前检查 `station.yaml` 中的 `safety_box`、`joint_limits` 和 `max_delta_per_step`（见 [02](02-station.md#check-the-safety-limits)）；调试时使用较低的 `max_delta_per_step`。守护进程会按这些限制裁剪每个伺服目标，环境侧的限制只能更严格。
- 配置了遥操作设备后，机器人连接时该设备必须在场；因此部署使用移除 `teleop:` 配置段的工作站 YAML（见 [08](08-deploy.md)）。

<a id="known-limits"></a>

## 已知限制

已知缺陷、临时处理方式与计划修复见[已知问题与待办工作](../known-issues.md)。

- **硬件侧目前仅通过 fake 设备验证。** robot_infra README 的[已知限制](../robot-infra.md#known-limitations-and-needs-hardware-verification)说明：“下面的内容都尚未在真实硬件上运行，fake 设备仅覆盖逻辑。”[gello.md](../gello.md#8-not-verified-yet)也说明：“尚未在物理 GELLO 或机器人上运行。”GELLO 的硬件数据（波特率、读取时间、延迟）、每台设备的偏移与方向，以及使能力矩后的行为都尚未验证。首次上机时需要调试。
- **使用录制 pkl 训练 ACT** 尚未在真实示范上验证。
- **`leader_cartesian`** 使用 Panda 正向运动学 DH 表，把 GELLO 转换成笛卡尔速度增量；该表尚未与真机对照验证（见 [teleop.md](../teleop.md#verify-the-dh-table-on-the-robot)）。本教程的流程使用关节空间。
- **`franka_real`** 是 SERL HTTP 桥接后端，仅支持单臂与笛卡尔动作；本教程使用 `real_robot`（robot_infra）。
- **奖励**：`real_robot` 环境返回奖励 0，且不会自行终止。奖励来自操作员的 `s` 键（`--reward-from-teleop-events`，见 [06](06-intervening.md#where-the-reward-comes-from)）或奖励分类器（见 [07](07-hil-training.md#optional-a-reward-classifier)）。
- **已配置的遥操作设备必须在场**：机器人 YAML 含 `teleop:` 配置段时，设备缺席会导致连接失败。
