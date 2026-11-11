<!-- English source: docs/tutorial/06-intervening.md -->

<a id="6-intervening-the-operator-page"></a>

# 6. 人工介入：操作员指南

[录制](04-record-demos.md)和 [HIL 训练](07-hil-training.md)时，在机器人旁参考本页。硬件进程在 NUC 上由 `robot_infra.supervise` 监督直接运行，使用 50 Hz、仅状态的笛卡尔预设。GELLO 使用 `--teleop-device leader_cartesian --teleop-event-device stdin`。SpaceMouse 使用 `--teleop-device spacemouse --teleop-event-device stdin`。

下面的 SpaceMouse 说明描述默认 `--spacemouse-mode tracker`。使用 `--spacemouse-mode serl` 时，先在工作站预设配置 xyz 和 RPY 安全盒：原始六轴范数 `> 0.001` 时接管，下一个空闲步返回策略（或录制器保持）。按住按钮 0 关闭、按钮 1 打开，按钮 0 优先，两者均未按下时夹爪中立值为 `0`。没有接管切换开关。终端 `t`/`h` 仍被忽略，标签/结束/退出有效。在 NUC 的仓库根目录激活项目环境，采用与录制和训练相同的模式及预设选择此模式。见[模式详情](../teleop.md#spacemouse)。

`stdin` 读取运行受监督命令的终端，包括 SSH；`keyboard` 通过 `pynput` 使用相同按键，需要桌面会话。

| 输入 | 作用 |
|---|---|
| `t` | GELLO：接管并锚定相对映射；重复按不会切换为关闭 |
| `h` | GELLO：交还；录制器保持该时刻实测位姿，HIL 将控制返回策略 |
| `s` | 成功并结束回合；录制器默认为奖励 1 的终止步；HIL 按键奖励需要 `--reward-from-teleop-events` |
| `f` | 失败；丢弃录制器尝试，或截断 HIL 回合 |
| `r` | 录制器丢弃并重录；HIL 没有重录动作 |
| Esc | 不带成功标签地结束回合（`human_episode_end`） |
| `q` | 结束当前回合，处理后停止录制器/actor |
| GELLO 扳机 | 接管锁存匹配后，捏合关闭（`-1`），释放打开（`+1`） |
| SpaceMouse 按钮 0 | 接管；再按交还；此路线只有这个按钮控制接管 |
| SpaceMouse 按钮 1 | 驱动时切换夹爪开/关；交还时忽略 |
| SpaceMouse 手柄 | 驱动时输入速度；仅运动不算接管 |

SpaceMouse 下终端 `t`/`h` 被忽略，`s`/`f`/`r`/Esc/`q` 仍有效。reset/回 home 期间的按键会丢弃。保持物理停止装置可触及。

反射停止后，检查工位，按需释放激活装置或在 Desk 清除故障。在 **NUC** 的**仓库根目录**、**项目环境激活**时，用 `rwrl recover` 请求错误恢复；`rwrl stop` 无需确认即可请求停臂。两者都遵循工作站连接选择。本直连流程须先结束录制器/actor 并释放连接；第二条 FCI 连接不能停止已被直连占用的机械臂。声明了活动 daemon 时，命令使用该 daemon。运行期间保持物理停止装置可触及。见 [rwrl](../cli.md#connection-ownership)。

<a id="the-sequence"></a>

## 操作顺序

<a id="gello"></a>

### GELLO

1. 等待 reset 及回 home/夹爪运动完成。
2. 舒适握持 leader 并按 **`t`**。当前 leader 位姿将相对映射锚定到 follower 命令；此笛卡尔路线无需逐关节匹配。
3. 移动 leader 引导工具，捏合/释放扳机控制夹爪。接管时若扳机位置与最后夹爪命令不一致，先匹配该状态；锁存机制避免立即意外张开。
4. 按 **`h`** 交还。录制器保持实测位姿，HIL 恢复策略。下次接管再按 `t` 建立新锚点。
5. 用 **`s`/`f`/`r`** 标记结果。录制器中应在介入时按 `s`，使成功终止步进入纯人工 pkl 导出。若已按 `h`，先用 `t` 重新锚定，再记录标签。
6. 用 **Esc** 无标签结束回合，或用 **`q`** 结束运行。

<a id="spacemouse"></a>

### SpaceMouse

1. 等待 reset。新接管前释放手柄和按钮。
2. 按**按钮 0** 接管。仅轻推手柄不会取得控制。
3. 偏转手柄输入工具速度；释放后停止向命令添加运动。机械臂仍可能继续移动至多一个领先上限以稳定。用**按钮 1** 切换夹爪；接管本身保持夹爪最后命令状态。
4. 再按**按钮 0** 交还；之后再按会开始新接管。键盘 `t` 和 `h` 不改变此设备的接管状态。
5. 用键盘 **`s`/`f`/`r`/Esc/`q`** 标记结果及退出。录制器中，在按钮 0 接管仍有效时标记成功。

选择 `episode_end_behavior: hold` 流程后，轨迹标签会冻结最后接受的目标并忽略手柄运动。按 n 请求新轨迹，再释放所有轴/按钮直到回中（默认最大轴绝对值 0.05，持续 0.2 秒）。此回中检查只在轨迹之间执行，不在每次接管时执行。达到回合配额后禁用 n，q 仍退出。同一轨迹内反复人工/策略交接照常进行。SERL 保留基于运动的接管阈值；回中阈值不替代它。

[评估](07-hil-training.md#evaluate)期间让策略持有动作控制权，以测量自主成功率。辅助成功计入 `task_success_rate`，主要 `success_rate` 则是 `autonomous_success_rate`。仅标签的 `s`/`f` 事件不算动作介入；失败标签否决同时出现的成功标签。

<a id="how-the-robot-follows-you"></a>

## 机器人如何跟随

笛卡尔输入源在 reset 后和每次交还后保存固定保持位姿。录制器保持策略共用该 tracker，因此保持命令指向固定位姿，而不是随实测位姿漂移的零 twist。HIL 交还给策略，不是要求冻结策略运动。

GELLO 接管时，输入源将 leader 当前位姿锚定到已有命令位姿。leader 位移变为请求平移和旋转。命令推进速度最高 **0.05 m/s** 和 **0.3 rad/s**，领先实测 TCP 至多 **0.05 m** 和 **0.3 rad**。多余领先量丢弃，不排队。leader 停止后，follower 仍可能向保留命令移动至多一个领先上限。调试时验证实际方向和尺度。

SpaceMouse 使用相同 tracker。手柄满幅偏转请求配置速度；松开后停止追加速度请求，但不会消除机械臂与现有命令之间受限的距离。

两次介入轮询间隔超过 **0.3 s** 时保持 follower 并记录延迟；下一次轮询重新锚定 GELLO，避免跨暂停跳变。这与环境 StepWatchdog 触发（停臂并要求 reset）及外部 supervisor 心跳升级相互独立。

请求位置在安全盒外时记录 **`WORKSPACE HOLD: <axes>`**，交还并保持实测位姿。清除 leader 请求，用 `h` 再 `t` 为 GELLO 建立新锚点；SpaceMouse 按按钮 0 再接管。不要仅为了清除保持状态而扩大工作区。

四个参数 `--teleop-lead-cap-m`、`--teleop-lead-cap-rad`、`--teleop-target-speed-m-s`、`--teleop-target-speed-rad-s` 进入 `experiment.teleop_control`。本教程录制、训练和部署中应保持这些值及所选遥操作设备一致。当前 HEAD 比较共享 tracker 块，而非物理设备身份；见 [04 的兼容性说明](04-record-demos.md#files-and-compatibility)。

<details>
<summary>其它模式：起始 arm 或 ramp（仅关节空间 leader_arm）</summary>

本节针对关节空间 leader 路线，不针对 `leader_cartesian` 或 SpaceMouse。`--teleop-onset` 决定 `t` 请求接管时的行为：

- `arm` 等待每个 leader 关节进入距 follower 的 `--teleop-arm-tolerance-rad`（0.1 rad）内；reset 会撤销激活。先保持 leader 对齐。
- 默认 `ramp` 立即开始。`joint_delta` 每步朝 leader 移动至多 `joint_scale_rad`；`joint_abs` 初始目标限制为每步 `--teleop-ramp-step-rad`（0.05）。

仅这些关节模式产生 `teleop_syncing`：`joint_delta` 初始追赶，以及 `joint_abs` 限速初始目标。实际动作和掩码保留在归档中；归档派生训练视图排除 syncing 监督。`--keep-syncing-steps` 可在录制器 pkl 导出中保留这些行。关节空间扳机接管仍需要操作员匹配夹爪状态；笛卡尔夹爪锁存机制不适用。

</details>

<a id="what-the-system-records"></a>

## 系统记录什么

介入包装器每步写入以下 `info` 键：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作；**仅介入时** |
| `policy_action` | 收到的原始策略提议动作 |
| `intervened` | 操作员动作已执行 |
| `intervention_onset` | 当前介入，本回合上一步未介入 |
| `human_episode_end` | 操作员回合结束键（Esc） |
| `teleop_events` | 触发的 `success`/`failure`/`rerecord`/`operator_quit` 事件；`teleop_syncing` 仅适用于关节空间 leader；**仅有事件时** |
| `human_action_mask` | 人实际控制的动作维度 |
| `override_mask` | 被覆盖维度，包括空闲保持补充动作 |
| `task_success`, `task_success_source` | 可用时的显式结果及来源；仅正奖励或终止标志不够 |

Esc 使 `step()` 返回 `truncated=True`（不会是 `terminated`）；`q` 还请求回合后关闭。成功键则可使该步终止。介入步保存的 `action` 始终为实际执行的人工动作。各方法的处理如下：

| 方法 | 实际动作与监督 | 回放或导出 |
|---|---|---|
| recorder | 保存实际执行动作；人工掩码选择监督维度 | 完整 SQLite 归档；成功人工 pkl 视图，默认排除关节空间 syncing |
| `hil_serl` | 从全部实际动作学习；人工且非 syncing 的行也进入示范回放 | 在线回放与不断增长的示范回放，按 `--demo-data-ratio` 混合 |
| `residual_hil_serl` | 人工动作为总执行动作；保留 `policy_action`、`base_actions`、`next_base_actions` | 残差 BC 取实际人工掩码与非零残差尺度的交集 |
| `silri` | 保留提议、实际掩码和前驱的 `intervention_onset` | 行为拟合和预训练监督人工维度；起始行不改标为专家 |

夹爪头（`hil_serl` 的 DQN、`--residual-variant hybrid`、`silri` 的离散头）将每个保存的夹爪动作标为最近的 `-1`/`0`/`+1`；`gripper_binarize_threshold: 0.5`（向导默认）使扳机恰好输出 `+1` 或 `-1`。详情见 [residual.md](../residual.md#human-intervention)、[silri.md](../silri.md#data-flow)、[teleop.md](../teleop.md#interventionwrapper)。

<a id="where-the-reward-comes-from"></a>

## 奖励来自哪里

教程默认 `task.name: none` 下，`real_robot` 返回奖励 0，没有任务成功检测器。需配置按键奖励或分类器。版本化任务预设可提供显式结果及 reset/终止规则；TCP 位置代理仍是独立指标。

- **录制器**：默认奖励来自按键（`--no-reward-from-teleop-events` 关闭）。`s` 将该步保存为奖励 1 的终止成功。
- **HIL 训练**（`hil_serl`、`residual_hil_serl`、`silri`）：没有 `--classifier-checkpoint` 时，向 actor 传 `--reward-from-teleop-events`（[07](07-hil-training.md) 的命令已包含）。两者均未设置时，actor 和 learner 会记录警告。

`s` 的规则：在录制器中**仍持有控制权时**按，因为只有介入步进入示范记录，交还后按 `s` 会使示范中所有奖励为 0。HIL 训练中，`s` 对人工或策略的任意步都有效：在物体落入容器那一步按。有分类器时取两个奖励的较大值（按键不会降低分类器奖励）。

<a id="when-to-intervene"></a>

## 何时介入

策略即将任务失败、碰撞或离开安全区域时接管；回到正确轨道后交还。正常执行时不要介入：actor 保存每一步，只有介入步进入示范缓冲区。初期通常介入较多，策略改善后逐渐减少。

两类接管及其教学作用：

- **恢复**：策略已出错（未抓到物体、漂向工作区外）。接管，将机器人带回可继续任务的状态，并在亲自完成任务前交还。这些行展示从策略自行到达、示范未覆盖的状态中恢复的方法。
- **纠正**：策略大体正确但细节有误（抓取偏心、过早释放物体）。只在该时刻接管，纠正后交还。关键步骤的短纠正为每个介入行提供最多信息。

每次接管应有目的且一致：`silri` 用动作训练行为模型，不一致处的分布变宽会放松约束；`residual_hil_serl` 中人工动作为总动作，BC 项将隐含残差裁剪到 `base +/- scale` 盒，因此远离基础策略的纠正只能教会残差“盒允许的范围”（critic 仍学习实际执行动作的价值）。无法恢复的回合用 Esc 结束。

<a id="take-over-selected-parts"></a>

## 接管指定部件

`--teleop-selected-parts arm.arm` 选择教程中的机械臂运动，`arm.gripper` 选择夹爪。双臂使用与 `arm_names` 一致的 `left.arm`、`right.arm`、`left.gripper` 或 `right.gripper`。空列表使用输入源可用的所有部件。默认 `--teleop-idle-behavior hold` 在接管期间保持未选部件；`policy` 让其继续由策略控制。只有 `human_action_mask` 维度提供专家监督；`override_mask` 也涵盖保持部件。录制和训练时采用相同部件选择及空闲行为，因为它们进入实验契约。见[部分接管](../real-world-experiments.md#part-takeover-and-imitation)。
