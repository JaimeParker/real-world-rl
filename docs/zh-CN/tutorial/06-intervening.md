<!-- English source: docs/tutorial/06-intervening.md -->

<a id="6-intervening-the-operator-page"></a>

# 6. 人工介入：操作员指南

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/06-intervening.md)

[录制](04-record-demos.md)和 [HIL 训练](07-hil-training.md)时在机器人旁保留本页。硬件进程在 NUC 上直接连接，使用 50 Hz 仅状态笛卡尔预设，由 `robot_infra.supervise` 监督。GELLO 用 `--teleop-device leader_cartesian --teleop-event-device stdin`；SpaceMouse 用 `--teleop-device spacemouse --teleop-event-device stdin`。

下文的 SpaceMouse 操作说明针对默认的 `--spacemouse-mode tracker`。使用 `--spacemouse-mode serl` 时，先在工位预设中配置 xyz 和 RPY 安全盒：原始六轴范数 `> 0.001` 时接管，下一空闲步将控制权交还策略（或录制器的保持策略）。按住按钮 0 闭合、按钮 1 张开，按钮 0 优先；均未按下时夹爪为中性 `0`。此模式没有接管切换按钮。终端 `t`/`h` 仍被忽略；标签/结束/退出按键有效。在 NUC、仓库根目录、项目环境激活时选择该模式，并与录制、训练使用相同的模式和预设。见[模式细节](../teleop.md#spacemouse)。

`stdin` 从监督命令所在终端读取，也支持 SSH。`keyboard` 通过 `pynput` 使用同样按键，但需要桌面会话。

| 输入 | 作用 |
|---|---|
| `t` | GELLO 接管并锚定相对映射；反复按不会切换为交还 |
| `h` | GELLO 交还；录制器保持此刻实测姿态，HIL 交还策略 |
| `s` | 成功并结束；录制器默认奖励 1 且终止，HIL 按键奖励需 `--reward-from-teleop-events` |
| `f` | 失败；录制丢弃尝试，HIL 截断回合 |
| `r` | 录制器丢弃并重录，HIL 无重录动作 |
| Esc | 无成功标签结束回合，`human_episode_end` |
| `q` | 结束当前回合，处理后停止 recorder/actor |
| GELLO 扳机 | 接管锁存匹配后，握紧闭合 `-1`、释放张开 `+1` |
| SpaceMouse 按键 0 | 接管，再按交还；此路线只有它控制接管 |
| SpaceMouse 按键 1 | 驱动时切换夹爪开闭，交还时忽略 |
| SpaceMouse 推杆 | 接管中输入速度，单独推动不算接管 |

SpaceMouse 忽略终端 `t`/`h`，其他 `s`/`f`/`r`/Esc/`q` 有效。reset/home 中按键被丢弃。物理停止装置须可用。

反射停机后先检查工位，按需释放激活装置或在 Desk 清除故障。在 **NUC、仓库根目录、项目环境（project environment）激活**时，使用 `rwrl recover` 请求恢复错误；`rwrl stop` 无需确认即可请求停臂。两者均遵循工作站连接选择。本直连流程须先结束 recorder/actor 并释放连接，第二个 FCI 连接无法停止已被直接占用的臂；若声明了存活 daemon，命令通过该 daemon 执行。运行中保持物理停止装置可用。见 [rwrl](../cli.md#connection-ownership)。

<a id="the-sequence"></a>

## 操作顺序

<a id="gello"></a>

### GELLO

1. 等待 reset 及 home/夹爪运动结束。
2. 舒适握住 leader，按 **`t`**，以当前 leader 姿态锚到 follower 命令；笛卡尔路线无需逐关节对齐。
3. 移动 leader 引导工具，用扳机开闭夹爪。接管时若与最后夹爪命令不符，先匹配该状态，锁存会防止立即误开。
4. **`h`** 交还；录制固定保持实测姿态，HIL 恢复策略。下次 **`t`** 使用新锚点。
5. **`s`/`f`/`r`** 标结果。录制时须在接管中 `s`，成功终止步才进入人工 pkl。已 `h` 时先 `t` 再锚定后标记。
6. **Esc** 无标签结束，**`q`** 结束运行。

<a id="spacemouse"></a>

### SpaceMouse

1. 等 reset，开始新接管前释放推杆和按键。
2. **按键 0** 接管，仅碰推杆不接管。
3. 推杆控制速度；释放停止新增运动请求，臂仍可在一个领先上限内收敛。**按键 1** 切换夹爪，接管本身保留最后夹爪命令。
4. 再按 **0** 交还，下次再按进入新接管。键盘 `t`/`h` 不改变此设备接管状态。
5. 键盘 **`s`/`f`/`r`/Esc/`q`** 处理结果和退出；录制成功标签须在按钮 0 的接管仍有效时按。

[评估](07-hil-training.md#evaluate)时保持策略控制，以测自主成功。辅助成功计入 `task_success_rate`，主 `success_rate` 为 `autonomous_success_rate`。单纯 `s`/`f` 标签不算动作介入，同时失败标签否决成功。

<a id="how-the-robot-follows-you"></a>

## 机器人如何跟随

笛卡尔源在 reset 和每次交还后保留固定 hold pose。录制器 hold policy 共用 tracker，持续发回固定姿态的动作，不是会随实测漂移的零 twist。HIL 则交还策略，不意味着冻结策略运动。

GELLO 接管时将 leader 当前姿态锚到现有 command pose，其位移成为平移和旋转请求。command 最多 **0.05 m/s、0.3 rad/s** 推进，领先实测 TCP 最多 **0.05 m、0.3 rad**。多余领先丢弃、不排队。leader 停下后 follower 仍可向保留命令走一个领先上限。配置时验证物理方向和尺度。

SpaceMouse 共用 tracker；全幅推杆请求配置速度，松手停止新增速度请求，不会消除机械臂与现有命令间的有界距离。

介入轮询间隔超过 **0.3 s** 时保持 follower 并记录延迟，下一轮重新锚定 GELLO，避免跨暂停跳动。这与环境 StepWatchdog 超时停臂、需 reset，以及外部 supervisor 心跳升级不同。

请求位置越出安全盒时记录 **`WORKSPACE HOLD: <axes>`**，交还并保持实测姿态。清除请求后用 `h` 再 `t` 重新锚定 GELLO；SpaceMouse 再按 0。不要仅为清 HOLD 扩大工作区。

`--teleop-lead-cap-m`、`--teleop-lead-cap-rad`、`--teleop-target-speed-m-s`、`--teleop-target-speed-rad-s` 四值进入 `experiment.teleop_control`。本教程录制、训练、部署沿用它们和同一 teleop 设备。当前 HEAD 比较的是共享 tracker 块而非实体设备身份，见 [04 兼容性](04-record-demos.md#files-and-compatibility)。

<details>
<summary>其他模式：onset arm 或 ramp（仅关节空间 leader_arm）</summary>

此节只讨论关节 leader，不用于 `leader_cartesian` 或 SpaceMouse。`t` 请求接管时，`--teleop-onset` 决定：

- `arm` 等待每关节误差小于 `--teleop-arm-tolerance-rad`（0.1 rad）；reset 解除 armed，先对齐 leader。
- 默认 `ramp` 立即开始。`joint_delta` 每步最多 `joint_scale_rad` 向 leader 推进；`joint_abs` 初始目标受 `--teleop-ramp-step-rad`（0.05）限制。

只有这些关节模式发出 `teleop_syncing`：joint_delta 初始追赶，以及 joint_abs 初始限速目标。执行动作和 mask 均保留在归档，但归档派生视图排除 syncing 监督。`--keep-syncing-steps` 可在录制 pkl 保留这些行。关节 trigger 仍需手动匹配夹爪状态，不享有笛卡尔锁存保护。

</details>

<a id="what-the-system-records"></a>

## 系统记录什么

介入 wrapper 每步写以下 `info`：

| 键 | 值 |
|---|---|
| `intervene_action` | 实际执行的 `(1, A)` 动作，仅介入时存在 |
| `policy_action` | 策略原始提议 |
| `intervened` | 执行了操作员动作 |
| `intervention_onset` | 本步介入，上一步未介入 |
| `human_episode_end` | Esc 回合结束键 |
| `teleop_events` | 触发的 `success`/`failure`/`rerecord`/`operator_quit`；`teleop_syncing` 仅关节 leader；有事件才出现 |
| `human_action_mask` | 人实际控制的动作维度 |
| `override_mask` | 被覆盖维度，含 idle hold 补位 |
| `task_success`, `task_success_source` | 显式结果及来源；正奖励或 terminal 本身不足以判断 |

Esc 返回 `truncated=True` 而非 `terminated`；`q` 还请求回合后停止。成功键可使该步 terminal。介入时存储的 action 始终是人实际执行的动作，各方法用途如下：

| 方法 | 执行动作和监督 | 回放/导出 |
|---|---|---|
| recorder | 存执行动作，由 human mask 选监督维度 | 完整 SQLite，成功人工 pkl 默认排除关节 syncing |
| `hil_serl` | 学全部执行动作，人工非 syncing 行进入 demo replay | 在线与增长 demo 回放，按 `--demo-data-ratio` 混合 |
| `residual_hil_serl` | 人工动作是总执行动作，保留策略及前后 base | residual BC 取 human mask 与非零残差尺度交集 |
| `silri` | 保留提议、实际 mask 及前驱 onset | 行为拟合/预训练监督人工维度，不将 onset 行改标专家 |

夹爪头（HIL DQN、残差 hybrid、SiLRI 离散头）将动作标为最近的 `-1`/`0`/`+1`；向导默认 `gripper_binarize_threshold: 0.5` 使扳机严格输出 ±1。详见[残差](../residual.md#human-intervention)、[SiLRI](../silri.md#data-flow)、[wrapper](../teleop.md#interventionwrapper)。

<a id="where-the-reward-comes-from"></a>

## 奖励来自哪里

教程默认 `task.name: none`，环境返回零奖励且无成功检测。须配置按键或分类器；版本化任务可提供显式结果及复位/终止规则，TCP 位置代理指标仍独立。

- **录制器：**默认按键奖励，`--no-reward-from-teleop-events` 关闭；`s` 终止成功并奖励 1。
- **HIL：**无 `--classifier-checkpoint` 时传 `--reward-from-teleop-events`，如 [07](07-hil-training.md)。两者都无时 actor/learner 警告。

录制须在接管中 `s`，否则终止步不进人工示范，pkl 可能全零奖励；HIL 中任何控制方的步骤都可标成功，在物体落箱那步按键。并有分类器时取两者较大奖励，按键不会降低分类器奖励。

<a id="when-to-intervene"></a>

## 何时介入

策略将失败、碰撞或离开安全区时接管，恢复正常后交还；正常时不介入。actor 存每步，只有介入步进入 demo，起初介入多、随后应减少。

- **恢复：**策略失误后把机器人带回可继续任务的状态，在完成任务前交还。这些行展示策略自己到达、原始示范未覆盖的状态如何脱困。
- **修正：**策略基本正确但抓取偏心或过早释放时，短暂接管修正后交还，每个人工步骤信息更集中。

介入应有明确目的且一致。SiLRI 根据人工模型分散程度放松约束；残差方法的人动作是总动作，BC 只在 `base +/- scale` 范围内拉动残差，超出范围的纠正只能学到边界，critic 仍学习真实执行动作价值。不可恢复时用 Esc 结束。

<a id="take-over-selected-parts"></a>

## 接管指定部件

`--teleop-selected-parts arm.arm` 选教程的臂，`arm.gripper` 选夹爪。双臂用与 `arm_names` 一致的 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`。空列表使用可用全部部件。默认 `--teleop-idle-behavior hold` 保持未选部件，`policy` 留给策略。只有 human mask 维度提供专家监督，override mask 还包括保持维度。录制与训练应保持同样选择和 idle 行为，它们属于实验契约。见[部分接管](../real-world-experiments.md#part-takeover-and-imitation)。
