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

使用 `--role eval` 时（[07](07-hil-training.md#evaluate)）保持策略自主执行，不按 Space。若奖励来自按键，任务成功时仍需按 `s`。

<a id="onset-arm-or-ramp"></a>

## 接管起始方式 `arm` 或 `ramp`

`--teleop-onset` 决定按 Space 时的行为：

- `arm` 用于 [04](04-record-demos.md) 的录制：主臂与从臂误差在 0.1 rad 内时切换才生效。每次环境 reset 都会解除就绪状态；就绪后持续保持到释放切换信号。接管本身不会引发额外运动，但每次接管前都需要手工对齐主从臂。
- `ramp` 是默认方式，用于 [07](07-hil-training.md)：立即接管。`joint_delta` 中从臂每步以最多 `joint_scale_rad` 向主臂靠近，直到对齐；过渡步骤也会存为人工介入。`joint_abs` 的初始目标按每步 `--teleop-ramp-step-rad`（0.05）限速，并标记 `teleop_syncing`。

希望每条示范从静止机器人开始时选择 `arm`；策略运动中需要快速接管时选择 `ramp`。两种方式都应在按 Space 前让主臂接近从臂。

<a id="what-the-system-records"></a>

## 系统记录什么

介入包装器在每步写入以下 `info` 字段：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作，**仅在介入时存在**。 |
| `policy_action` | 原样保存策略提出的动作。 |
| `intervened` | 是否执行了操作员动作。 |
| `intervention_onset` | 本步介入，而本回合上一步未介入。 |
| `human_episode_end` | 操作员结束回合的按键（Esc）。 |
| `teleop_events` | 触发的 `success` / `failure` / `rerecord` 事件；主臂在限速接管步骤还会添加 `teleop_syncing`。**仅有事件时存在**。 |

结束回合事件使 `step()` 返回 `truncated=True`，保持 `terminated` 为假。介入步骤保存的 `action` 始终是实际执行的人工动作。各方法的处理方式如下：

| 方法 | 保存的动作 | `intervened` 的用途 | 缓冲区 |
|---|---|---|---|
| 录制器 | 人工动作 | 仅录制介入步骤；ramp 同步步骤默认排除，`--keep-syncing-steps` 可保留。 | 示范 pkl。 |
| `hil_serl` | 实际执行动作，介入时为人工动作 | 仅用于路由，同时复制至示范缓冲区；标记本身不保存。 | 在线缓冲区保存所有数据；示范缓冲区包含介入数据和 `--demo-dataset-paths` 示范，按 `--demo-data-ratio` 混合。 |
| `residual_hil_serl` | 人工的**总动作**，以及 `policy_action`、`base_actions`、`next_base_actions` | 复制到示范缓冲区；`--residual-bc-weight` > 0 时，在介入数据上将单位残差拉向 `clip((a_human - a_base) / scale, -1, 1)`。 | 与 `hil_serl` 相同，两个缓冲区都保存 `intervened`。 |
| `silri` | 人工动作，以及 `policy_action`、`intervened`、`intervention_onset` | 介入数据训练人工行为模型和夹爪头；接管前的策略步骤标记为 `intervention_onset`。 | 在线缓冲区保存所有数据；示范缓冲区保存介入数据，使用 `--onset-to-demo-buffer` 时还复制起始标记数据，但它仍保持未介入状态。 |

夹爪头（`hil_serl` 的 DQN、`--residual-variant hybrid`、`silri` 的离散头）将保存的夹爪动作标记为 `-1` / `0` / `+1` 中最近的值。向导默认的 `gripper_binarize_threshold: 0.5` 让扳机恰好输出 `+1` 或 `-1`。细节见 [residual.md](../residual.md#human-intervention)、[silri.md](../silri.md#data-flow)、[teleop.md](../teleop.md#interventionwrapper)。

<a id="where-the-reward-comes-from"></a>

## 奖励来源

`real_robot` 环境返回奖励 0，且不会自行终止。没有奖励来源时，每条 transition 的奖励都为 0，智能体无法获知成功标准。

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
