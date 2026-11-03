<a id="6-intervening-the-operator-page"></a>

# 6. 人工介入操作员指南

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/06-intervening.md)

本页供使用键盘和 GELLO 的操作员在录制示范（[04](04-record-demos.md)）和 HIL 训练（[07](07-hil-training.md)）时查阅。建议在机器人旁保持打开。

配置方式：GELLO 通过机器人守护进程读取（`--teleop-device leader_arm`）；按键来自 actor 机器的键盘（`--teleop-event-device keyboard`），由 `pynput` 监听器读取，因此需要该机器的桌面会话。

| 按键 | 效果 |
|---|---|
| **Space** | 切换人工介入：按一次接管，再按一次交还控制。 |
| **s** | 成功。录制器将回合标为成功并结束；启用 `--reward-from-teleop-events` 的 HIL 训练赋予奖励 1，将该步存为终止步骤并结束回合。 |
| **f** | 失败。录制器丢弃回合；启用 `--reward-from-teleop-events` 的 HIL 训练以截断方式结束回合，保持非终止状态。 |
| **r** | 重录。录制器丢弃当前回合并立即开始下一回合；对 HIL 训练无效果。 |
| **Esc** | 结束回合，以截断方式结束并保持非终止状态，设置 `info["human_episode_end"]`。 |
| 扳机 | 控制夹爪；捏过一半行程时闭合（`-1`），松开时打开（`+1`）。 |

reset 归位期间按下的键会被丢弃。

<a id="the-sequence"></a>

## 操作顺序

1. **Reset。** 环境重置，机械臂归位、夹爪打开，等待其静止。
2. **主臂靠近从臂。** 逐关节调整 GELLO，使其姿态接近从臂。
3. **Space。** 开始接管。使用 `--teleop-onset arm` 时，每个主臂关节与从臂的误差都小于 `--teleop-arm-tolerance-rad`（0.1 rad）后才接管；此前策略继续执行，actor 每秒打印一次关节误差。`ramp` 会立即接管。
4. **移动。** 操作 GELLO；`joint_delta` 中从臂以每关节每步最多 `joint_scale_rad`（0.05 rad）跟随，守护进程还会按 `max_delta_per_step` 裁剪所有目标。
5. **扳机。** 捏紧抓取，松开将物体放入箱子。
6. 机器人回到正确轨迹后，**再次按 Space** 交还控制（HIL 训练）。
7. 结果明确时按 **s / f / r**。录制时要在**仍然接管**的状态按 `s`；需要把达成任务的那一步记录为终止步骤时也应及时按下。
8. **Esc** 结束没有明确结果的回合；达到步数上限也会结束。

`--role eval`（[07](07-hil-training.md#evaluate)）中保持 Space 未接管状态，可测量自主回合。需要时仍能接管；辅助成功计入 `task_success_rate`，主要 `success_rate` 等于 `autonomous_success_rate`。采用按键奖励时，成功仍需按 `s` 标注。只按 `s` / `f` 标注结果不算动作介入；失败标签否决成功标签，同一步同时出现也按失败处理。

<a id="onset-arm-or-ramp"></a>

## 接管起始方式 `arm` 或 `ramp`

`--teleop-onset` 决定按 Space 时的行为：

- `arm` 用于 [04](04-record-demos.md) 录制：主从臂误差在 0.1 rad 内时切换才生效，每次 reset 解除就绪，就绪后保持至释放切换信号。接管本身不会导致追赶运动，但每次前需手工对齐。
- 默认 `ramp` 用于 [07](07-hil-training.md)：立即接管。`joint_delta` 中从臂每步以最多 `joint_scale_rad` 靠近主臂；`joint_abs` 初始目标按 `--teleop-ramp-step-rad`（0.05）限速。两种模式都用 `teleop_syncing` 标记追赶：`joint_delta` 标记直到主臂首次进入从臂一个 `joint_scale_rad` 范围内的初始步骤，`joint_abs` 标记各次 ramp 限速步骤。归档保留实际动作和真实人工掩码，训练投影排除同步监督。`joint_delta` 初次对齐后，即使之后操作者运动过快而再次饱和，也正常作为教学步骤。录制器可用 `--keep-syncing-steps` 将同步步骤作为普通监督写入 pkl，归档派生训练视图仍会排除。

希望每条示范从静止机器人开始时选择 `arm`；策略运动中需要快速接管时选择 `ramp`。两种方式都应在按 Space 前让主臂接近从臂。

<a id="what-the-system-records"></a>

## 系统记录什么

介入包装器在每步写入以下 `info` 字段：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作，**仅介入时存在**。 |
| `policy_action` | 原样保留策略提案。 |
| `intervened` | 是否实际执行人工动作。 |
| `intervention_onset` | 本步介入、同回合上一步未介入。 |
| `human_episode_end` | 操作员结束回合按键（Esc）。 |
| `teleop_events` | 触发的 `success` / `failure` / `rerecord`，主臂追赶时增加 `teleop_syncing`；**仅有事件时存在**。 |
| `human_action_mask` | 实际由人控制的动作维度。 |
| `override_mask` | 覆盖策略的维度，包含未选部件的保持补充。 |
| `task_success`、`task_success_source` | 显式结果及来源；正奖励或终止标记本身不足以判成功。 |

结束回合事件使 `step()` 返回 `truncated=True`，保持 `terminated` 为假。介入步骤保存的 `action` 始终是实际执行的人工动作。各方法的处理方式如下：

| 方法 | 执行动作与监督 | 回放或导出 |
|---|---|---|
| 录制器 | 保存实际执行动作，人工掩码选择监督维度。 | 完整 SQLite 归档；成功人工 pkl 视图默认排除同步步骤，`--keep-syncing-steps` 可保留。 |
| `hil_serl` | 使用全部实际动作学习，非同步人工数据也进入示范回放。 | 在线回放与持续增长的示范回放，按 `--demo-data-ratio` 混合。 |
| `residual_hil_serl` | 人工动作是实际总动作，保留 `policy_action`、`base_actions`、`next_base_actions`。 | 残差 BC 取真实人工掩码与非零残差缩放维度的交集。 |
| `silri` | 保留提案、真实掩码及介入前一步的 `intervention_onset`。 | 行为拟合和预训练只监督人工维度，起始标记数据不会改为专家标签。 |

夹爪头（`hil_serl` 的 DQN、`--residual-variant hybrid`、`silri` 的离散头）将保存的夹爪动作标记为 `-1` / `0` / `+1` 中最近的值。向导默认的 `gripper_binarize_threshold: 0.5` 让扳机恰好输出 `+1` 或 `-1`。细节见 [residual.md](../residual.md#human-intervention)、[silri.md](../silri.md#data-flow)、[teleop.md](../teleop.md#interventionwrapper)。

<a id="where-the-reward-comes-from"></a>

## 奖励来源

教程默认 `task.name: none` 时，`real_robot` 返回奖励 0，没有任务成功检测器。需要配置按键奖励或分类器。版本化任务可提供显式结果和 reset、终止规则；TCP 位置代理指标单独报告。

- **录制器**默认从按键生成奖励，使用 `--no-reward-from-teleop-events` 可关闭。按 `s` 将当前步存为奖励 1 的成功终止步骤。
- **HIL 训练**（`hil_serl`、`residual_hil_serl`、`silri`）未使用 `--classifier-checkpoint` 时，需要向 actor 传入 `--reward-from-teleop-events`；[07](07-hil-training.md) 的命令已经包含。两项都未设置时，actor 和 learner 会打印警告。

录制器中按 `s` 的规则是：**仍在接管时按下**。录制器只保存介入步骤，释放接管后再按 `s` 会让示范全程奖励为 0。HIL 训练中 `s` 对人工步骤和策略步骤都生效，应在物体落入箱子的那一步按下。使用分类器时取两个奖励的较大值，按键不会降低分类器奖励。

<a id="when-to-intervene"></a>

## 何时介入

策略即将任务失败、碰撞或离开安全区域时接管，恢复正确行为后交还控制。策略正常运行时保持自主执行：actor 保存每一步，只有人工介入步骤进入示范缓冲区。训练初期通常需要频繁介入，随后随策略改进逐渐减少。

两类接管及其教学作用：

- **恢复**：策略已经偏离，例如漏抓物体或漂出工作区。接管后将机器人带回可以继续任务的状态，在自己完成整个任务前交还控制。这些数据展示了策略自主到达、而原始示范未覆盖的状态的恢复路径。
- **纠正**：策略大致正确但细节有误，例如抓取位置偏心或过早松开。只在该时刻接管，纠正后交还。关键步骤的短暂修正能提高每条介入数据的信息量。

每次接管应有明确目的且动作一致。在 `silri` 中，人工动作训练行为模型，操作不一致会增大模型分散程度并放宽约束。在 `residual_hil_serl` 中，人工动作是总动作；BC 项会把隐含残差裁剪到 `base +/- scale` 范围，因此远离基础动作的纠正只能将残差教到边界，但 critic 仍会学习实际执行动作的价值。无法恢复的回合用 Esc 结束。


<a id="take-over-selected-parts"></a>

## 只接管指定部件

`--teleop-selected-parts arm.arm` 选择教程机器人的机械臂运动，`arm.gripper` 选择夹爪。双臂可用 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`，名称与 `arm_names` 一致。空列表采用输入源所有可用部件。默认 `--teleop-idle-behavior hold` 在接管时保持未选部件，`policy` 让其继续由策略控制。只有 `human_action_mask` 维度提供专家监督；`override_mask` 还包含保持部件。部件选择与空闲行为进入实验约定，录制和训练需一致，见[部分接管](../real-world-experiments.md#part-takeover-and-imitation)。
