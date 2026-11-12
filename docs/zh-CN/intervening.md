<!-- English source: docs/intervening.md -->

<a id="operator-page-intervening"></a>

# 操作员页面：介入

[中文入口](../README.zh-CN.md) · [English](../intervening.md)

在机器人旁进行[录制](tutorial/08-record-demos.md)和 [HIL 训练](tutorial/10-hil-training.md)时使用本页。硬件进程直接在 NUC 上由 `rwrl supervise` 监督，使用 50 Hz 纯状态笛卡尔预设。GELLO 使用 `--teleop-device leader_cartesian --teleop-event-device stdin`。SpaceMouse 使用 `--teleop-device spacemouse --teleop-event-device stdin`。

下方 SpaceMouse 说明针对默认 `--spacemouse-mode tracker`。`--spacemouse-mode serl` 应先在站点预设配置 xyz 和 RPY 安全盒：原始六轴范数 `> 0.001` 接管，下一个空闲步返回策略（或录制器保持）。按住按钮 0 关闭、按钮 1 打开，按钮 0 优先，均未按下夹爪为中立 `0`。没有接管切换开关。仍忽略终端 `t`/`h`，标签/结束/退出可用。在 NUC 仓库根目录、激活项目环境，选择与录制训练相同模式和预设。见[模式详情](teleop.md#spacemouse)。

`stdin` 读取运行受监督命令的终端，包括 SSH；`keyboard` 通过 `pynput` 使用同样按键，需要桌面会话。

| 输入 | 效果 |
|---|---|
| `t` | GELLO：接管并锚定相对映射；重复按不会关闭 |
| `h` | GELLO：交还；录制器保持该时刻实测姿态，HIL 交回策略 |
| `s` | 成功并结束回合；录制器默认奖励 1、终止步；HIL 按键奖励需 `--reward-from-teleop-events` |
| `f` | 失败；丢弃录制尝试，或截断 HIL 回合 |
| `r` | 录制器丢弃重录；HIL 无重录动作 |
| Esc | 无成功标签结束回合 (`human_episode_end`) |
| `q` | 结束当前回合，处理后停止录制器/actor |
| GELLO 扳机 | 接管锁存匹配后，压下关闭 (`-1`)、释放打开 (`+1`) |
| SpaceMouse 按钮 0 | 接管，再按交还；此路径只有该按钮控制接管 |
| SpaceMouse 按钮 1 | 驱动时切换夹爪开闭；交还时忽略 |
| SpaceMouse 控制帽 | 驱动时为速度输入；单独移动不算接管 |

SpaceMouse 忽略终端 `t`/`h`；`s`/`f`/`r`/Esc/`q` 仍有效。reset/home 期间按键被丢弃。保持物理停止装置可及。

反射停止后，检查工位，按需释放激活装置或在 Desk 清错。在 **NUC** 的**仓库根目录**、**激活项目环境**，用 `rwrl recover` 请求错误恢复；`rwrl stop` 无确认请求停止。两者遵循站点连接选择。本直连流程须先结束录制器/actor，释放连接；第二 FCI 连接无法停止直连占用臂。声明且运行中的守护进程将供这些命令使用。运行期间保持物理停止装置可及。见 [rwrl](cli.md#connection-ownership)。

<a id="the-sequence"></a>

## 操作顺序

<a id="gello"></a>

### GELLO

1. 等待 reset 和 home/夹爪运动完成。
2. 舒适握持主臂，按 **`t`**。当前主臂姿态将相对映射锚定到从臂命令；笛卡尔路径无需逐关节匹配。
3. 移动主臂引导工具，压下/释放扳机操作夹爪。接管时若扳机与末次夹爪命令不符，先匹配状态；锁存避免立即意外打开。
4. 按 **`h`** 交还。录制器保持实测姿态；HIL 恢复策略。下一次按 `t` 用新锚点接管。
5. 用 **`s`/`f`/`r`** 标结果。录制器介入时按 `s`，成功终止步才进入仅人工 pkl。`h` 后先 `t` 重新锚定再记录标签。
6. **Esc** 无标签结束回合，**`q`** 结束运行。

<a id="spacemouse"></a>

### SpaceMouse

1. 等待 reset。新接管前释放控制帽和按钮。
2. 按**按钮 0** 接管。单独轻推控制帽不取得控制。
3. 偏转控制帽请求工具速度；释放停止向命令添加运动。机械臂仍可移动至多一个领先上限以收敛。**按钮 1** 切换夹爪；接管本身保留末次夹爪命令。
4. 再按**按钮 0** 交还；之后再按开始新接管。键盘 `t`、`h` 不改变此设备接管状态。
5. 键盘 **`s`/`f`/`r`/Esc/`q`** 标结果及退出。录制时在按钮 0 接管仍激活时标成功。

可选 `episode_end_behavior: hold` 流程中，轨迹标签冻结最后接受目标，忽略控制帽运动。按 n 请求新轨迹，再释放各轴/按钮至中立（默认轴最大绝对值 0.05，持续 0.2 秒）。门控仅用于轨迹之间，不用于每次接管。达到回合配额后 n 禁用，q 仍退出。同一轨迹内人工/策略可正常反复交还。SERL 保持运动接管阈值，中立阈值不替代它。

[评估](tutorial/10-hil-training.md#evaluate)时把动作控制留给策略，以测自主成功。辅助成功计入 `task_success_rate`，主要 `success_rate` 为 `autonomous_success_rate`。仅标签 `s`/`f` 不算动作介入；失败标签否决同时成功标签。

<a id="how-the-robot-follows-you"></a>

## 机器人如何跟随

笛卡尔源在 reset 和每次交还后保持固定姿态。录制器保持策略共享 tracker，保持时命令返回固定姿态，而非随实测姿态漂移的零 twist。HIL 交还给策略，不是冻结策略运动。

GELLO 接管时，源将主臂当前姿态锚定到现有命令姿态。主臂位移成为请求平移/旋转。命令最高以 **0.05 m/s**、**0.3 rad/s** 前进，领先实测 TCP 最多 **0.05 m**、**0.3 rad**。超额领先被丢弃，不排队。主臂停止后，从臂仍可向保留命令前进最多一个领先上限。调试时验证实际方向和尺度。

SpaceMouse 使用相同 tracker。控制帽最大偏转请求配置速度，松手停止继续请求速度，不移除机械臂与现有命令间的有界距离。

介入轮询间隔超过 **0.3 s** 时保持从臂并记录延迟；下一次轮询重新锚定 GELLO，避免跨暂停跳变。此 tracker 行为与停止机械臂且需 reset 的环境 StepWatchdog 触发、以及外部监督器心跳升级分别独立。

请求位置越出安全盒时记录 **`WORKSPACE HOLD: <axes>`**，交还并保持实测姿态。清除主臂请求，`h` 后 `t` 以新锚点恢复 GELLO；SpaceMouse 按按钮 0 重新接管。不要仅为解除 HOLD 扩大工作区。

四个 `--teleop-lead-cap-m`、`--teleop-lead-cap-rad`、`--teleop-target-speed-m-s`、`--teleop-target-speed-rad-s` 值进入 `experiment.teleop_control`。本教程录制、训练、部署保持这些值和选定设备一致。当前 HEAD 比较共享 tracker 块，而非物理设备身份；见 [08 兼容性说明](tutorial/08-record-demos.md#files-and-compatibility)。

<details>
<summary>其他模式：接管起始 arm 或 ramp（仅关节空间 leader_arm）</summary>

本节针对关节空间主臂路径，不适用 `leader_cartesian` 或 SpaceMouse。`--teleop-onset` 决定 `t` 请求接管时的行为：

- `arm` 等待各主臂关节距从臂不超过 `--teleop-arm-tolerance-rad`（0.1 rad）；reset 解除待接管。先保持主臂对齐。
- 默认 `ramp` 立即开始。`joint_delta` 每步向主臂前进至多 `joint_scale_rad`；`joint_abs` 初始目标每步限制为 `--teleop-ramp-step-rad`（0.05）。

只有这些关节模式输出 `teleop_syncing`：`joint_delta` 初始追赶、`joint_abs` 限速初始目标。执行动作和掩码保留在归档；归档派生训练视图排除同步监督。`--keep-syncing-steps` 可在录制器 pkl 保留这些行。关节空间扳机接管仍需操作员匹配夹爪状态；笛卡尔夹爪锁存不适用。

</details>

<a id="what-the-system-records"></a>

## 系统记录内容

介入包装器每步写入以下 `info` 键：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 执行的 `(1, A)` 动作；**仅介入时** |
| `policy_action` | 收到的策略提议动作 |
| `intervened` | 执行了操作员动作 |
| `intervention_onset` | 本步介入且本回合前一步未介入 |
| `human_episode_end` | 操作员回合结束键 (Esc) |
| `teleop_events` | 触发的 `success`/`failure`/`rerecord`/`operator_quit`；`teleop_syncing` 仅关节空间主臂；**仅有事件时** |
| `human_action_mask` | 人工实际控制的动作维度 |
| `override_mask` | 被覆盖维度，含空闲保持补充 |
| `task_success`, `task_success_source` | 可用时的显式结果及来源；仅正奖励或终止标志不足以判定 |

Esc 使 `step()` 返回 `truncated=True`（从不为 `terminated`）；`q` 还请求回合后关闭。成功键可使该步终止。介入步存储的 `action` 始终是实际执行的人工动作。各方法处理如下：

| 方法 | 执行动作与监督 | 回放或导出 |
|---|---|---|
| recorder | 存实际执行动作；人工掩码选择监督维度 | 完整 SQLite 归档；成功人工 pkl 视图，默认排除关节空间同步 |
| `hil_serl` | 学习全部执行动作；人工非同步行还填充示范回放 | 在线及增长示范回放，按 `--demo-data-ratio` 混合 |
| `residual_hil_serl` | 人工动作是总执行动作；保留 `policy_action`、`base_actions`、`next_base_actions` | 残差 BC 将实际人工掩码与非零残差尺度取交集 |
| `silri` | 保留提议、实际掩码及前驱的 `intervention_onset` | 行为拟合及预训练监督人工维度；起始行不重标为专家 |

夹爪头（`hil_serl` 的 DQN、`--residual-variant hybrid`、`silri` 离散头）将各存储夹爪动作标为最近的 `-1`/`0`/`+1`；`gripper_binarize_threshold: 0.5`（向导默认）使扳机恰给 `+1` 或 `-1`。详见 [residual.md](residual.md#human-intervention)、[silri.md](silri.md#data-flow)、[teleop.md](teleop.md#interventionwrapper)。

<a id="where-the-reward-comes-from"></a>

## 奖励来自哪里

教程默认 `task.name: none` 时，`real_robot` 返回奖励 0，无任务成功检测器。配置按键奖励或分类器。版本化任务预设可提供显式结果及 reset/终止规则；TCP 位置代理仍为独立指标。

- **录制器：** 默认奖励来自按键（`--no-reward-from-teleop-events` 关闭）。`s` 将该步保存为奖励 1 的终止成功。
- **HIL 训练** (`hil_serl`, `residual_hil_serl`, `silri`)：无 `--classifier-checkpoint` 时向 actor 传 `--reward-from-teleop-events`（[10](tutorial/10-hil-training.md) 已这样做）。两者均未设时 actor 和 learner 警告。

`s` 规则：录制器中在**仍持控制权时**按，因为仅介入步被录入；释放后按 `s` 会使示范处处奖励 0。HIL 中任一步（人工或策略）按 `s` 均有效：物体落入容器那步按。分类器和按键取较大奖励（按键不降低分类器奖励）。

<a id="when-to-intervene"></a>

## 何时介入

策略即将任务失败、碰撞或离开安全区时接管，回到正轨后释放。运行良好时不要介入：actor 存每步，只有介入步进入示范缓冲区。初期介入多，随策略改善减少。

两种接管及其教学作用：

- **恢复：** 策略已出错（漏抓、漂出工作区）。接管，将机器人带回任务可继续状态，在自己完成任务前释放。这些行展示策略自主到达而示范未访问状态的出路。
- **修正：** 策略大致正确但细节错（抓取偏心、过早释放）。只在该时刻接管、纠正、释放。关键步短修正使每个介入行提供最多信息。

每次接管保持目的明确、一致：`silri` 用动作训练行为模型，不一致处的分布宽度会放松约束；`residual_hil_serl` 中人工动作是总动作，BC 将隐含残差裁剪到 `base +/- scale` 盒，远离基础策略的修正只能教到盒允许处（critic 仍学习实际动作价值）。Esc 结束无法恢复的回合。


<a id="take-over-selected-parts"></a>

## 接管选定部件

`--teleop-selected-parts arm.arm` 选择本教程机械臂运动，`arm.gripper` 选夹爪。双臂用 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`，匹配 `arm_names`。空列表使用源可提供的全部部件。默认 `--teleop-idle-behavior hold` 在接管时保持未选部件；`policy` 留给策略。仅 `human_action_mask` 维度提供专家监督，`override_mask` 还涵盖保持部件。录制和训练保持部件选择及空闲行为相同，因为它们进入实验契约。见[部分接管](real-world-experiments.md#part-takeover-and-imitation)。
