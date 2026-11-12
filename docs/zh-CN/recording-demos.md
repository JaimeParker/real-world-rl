<!-- English source: docs/recording-demos.md -->

<a id="recording-demos-on-the-real-robot"></a>

# 在真实机器人上录制示范

`examples/record_real_demos.py` 将真机遥操作的完整回合归档到 SQLite，并将成功人工示范导出为 `--demo-dataset-paths` 接受的 pkl 格式（`hil_serl`、`residual_hil_serl`、`silri`、`real_world_bc`）。BC/HIL 加载器还接受 `.trajectory.sqlite3` 和 `.trajectory.pt`。ACT 轨迹训练器使用 pkl。实现位于 `real_world_rl/demos/record.py`。

录制器与 HIL actor 使用完全相同的环境构建方式（`build_hil_env(args, role="actor")`），因此接受与 `train_real_world.py hil_serl` 相同的环境、遥操作、观测参数和 `--config` 预设。请采用后续加载示范的运行参数：录制的观测和动作必须与其环境匹配（加载时检查 pkl 的契约附属文件）。[教程 04](tutorial/04-record-demos.md) 讲解首次录制，[教程 06](tutorial/06-intervening.md) 是操作员指南（按键、接管起始和记录内容）。

对于已调试 `real_robot` 工作站上的 SpaceMouse，受监督的 [`rwrl teleop run` 流程](spacemouse-commissioning.md#supervised-recording-sessions) 增加显式回合结束保持。标签封存轨迹；此后手柄运动不生效，直到 `n` 请求新回合且手柄稳定回中。`q` 释放控制，达到回合预算后也如此。仍支持同一轨迹内反复接管/交还，不触发此回中检查。检查后，录制和 actor 循环获取新观测；首个归档帧及策略输入均来自回中确认后，图像堆叠历史也用该新帧重新填充。Python 录制器默认仍为 `episode_end_behavior: auto`；直接使用时需显式选择 `hold`。

使用 [SpaceMouse 验收清单](testing/spacemouse-control-session.md) 及其记录模板，验证两种模式的方向、工作区、保持和正常停止。仅成功示范不能证明硬件通过验收或正常停止稳定。

<a id="commands"></a>

## 命令

GELLO 由机器人 daemon 提供，follower 为 `joint_delta`（教程各阶段的设置），按键使用键盘：

```bash
rwrl record --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --teleop-onset arm \
    --output-path demos/task_20.pkl --num-successes 20
```

`joint_abs` 数据集（用于 `joint_abs` 流程）需增加 `--real_robot.action_mode joint_abs --robot-teleop-action-mode joint_abs`，并在预设中提供 `joint_limits`。

`hil.yaml` 是训练预设 [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml) 的副本；命令行不能表达数值对列表，因此 `joint_limits` 必须来自预设。不是录制器字段的预设键（算法超参数）会被跳过。

其它运动设备使用方式相同：本机 GELLO 用 `--teleop-device gello` 和 `--teleop-options-yaml gello.yaml`；GELLO 驱动 `cartesian_delta` 臂（`real_robot` 或 `franka_real`）用 `--teleop-device leader_cartesian`（[`examples/presets/gello_cartesian.yaml`](../../examples/presets/gello_cartesian.yaml) 是单臂 50 Hz 预设，tracker 参数 `--teleop-lead-cap-m`、`--teleop-lead-cap-rad`、`--teleop-target-speed-m-s`、`--teleop-target-speed-rad-s` 见 [teleop.md](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)）；SpaceMouse 驱动 `cartesian_delta` 臂用 `--teleop-device spacemouse --teleop-event-device stdin`。`leader_cartesian` 或 `spacemouse` 录制的示范在实验契约（`teleop_control`）中包含 tracker，因此部署时若 `--teleop-device` 不带 tracker，会拒绝使用这些示范训练的策略。GELLO 设置见 [gello.md](gello.md)，设备说明见 [teleop.md](teleop.md)。

SpaceMouse 参数（详情见 [teleop.md](teleop.md#spacemouse)）：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--teleop-device spacemouse` | | 本机 SpaceMouse；按钮 0 接管/交还，按钮 1 切换夹爪（从最后命令开始），手柄输入速度 |
| `--teleop-event-device stdin`（或 `keyboard`） | | 结果键 `s`/`f`/`r`、Esc、`q`；`t`/`h` 被忽略（按钮 0 才是切换键） |
| `--teleop-target-speed-m-s`, `--teleop-target-speed-rad-s` | 0.05, 0.3 | 手柄满幅偏转时的速度 |
| `--teleop-lead-cap-m`, `--teleop-lead-cap-rad` | 0.05, 0.3 | 命令领先机械臂的上限；释放手柄后在此范围内停止 |
| `--real-robot.xyz-scale-m`, `--real-robot.rot-scale-rad` | 0.02, 0.1 | 必须至少等于领先上限：传入 0.05 和 0.3 |
| `--teleop-options-yaml` 中的 `device_index`, `deadzone`, `invert_axes` | 0, 0.05, `[]` | SpaceMouse 序号、每轴静止噪声死区（满幅比例）、要翻转的输出轴（0–5） |

`--teleop-event-device keyboard`（本机桌面会话）和 `--teleop-event-device stdin`（运行录制器的终端，也支持 SSH）的操作键：`t` 接管，`h` 交还，`s` 成功，`f` 失败，`r` 重录，Esc 结束回合，`q` 结束回合并在处理后停止录制器。

双臂（`--env-backend real_robot_bimanual`）：`--teleop-device bimanual --teleop-arm-device leader_arm`（或 `gello`、`robot`）每臂读取一个 leader，录制器逐臂处理。保持动作属于各臂自身（`joint_abs` 中为各 follower 关节在自身限位下的值），回合开始检查逐臂比较 leader 与 follower 并指出差距最大的臂，`--max-episode-steps` 替换 `--real_robot_bimanual.max_episode_steps`。任一操作员介入都会记录该步；另一臂保持不动。命令见 [bimanual.md](bimanual.md#teleop-and-demo-recording)。

然后用示范训练：

```bash
rwrl train hil_serl --role learner --config hil.yaml \
    --demo-dataset-paths demos/task_20.pkl ...
```

<a id="recorder-flags"></a>

## 录制器参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--output-path` | 必填 | 示范 pkl，必须以 `.pkl` 结尾且尚不存在；旁边写入 `<stem>.contract.json` |
| `--num-successes` | 20 | 保留指定数量的回合后停止 |
| `--max-episodes` | 未设置 | 完成指定次数尝试后停止，包括失败、重录和超时；该上限与 `--num-successes` 中先满足者结束录制 |
| `--max-episode-steps` | 未设置 | 回合步数上限；设置时替换 `--real_robot.max_episode_steps`、`--real_robot_bimanual.max_episode_steps` 或 `--franka_real.max_episode_steps` |
| `--start-tolerance-rad` | 0.8 | `joint_abs`：任一关节的 leader 与 follower 差值超过此值时拒绝开始回合 |
| `--keep-syncing-steps` | 关闭 | 将 leader 追赶行作为普通监督步纳入 pkl；SQLite 保留 syncing 标记 |
| `--export-trajectory-pt` | 关闭 | 额外将完整归档导出为可移植 `.trajectory.pt`；SQLite 仍是恢复来源 |
| `--reward-from-teleop-events` | 开启（仅录制器；训练默认关闭） | 成功键将该步保存为终止成功（奖励 `--teleop-success-reward`，1.0；`done` 为 True）并结束回合；`--no-reward-from-teleop-events` 保留环境自身奖励（`real_robot` 为 0），因此保留的成功以奖励 0、`done` 为 False 保存 |

`--control-hz` 像对 actor 一样控制循环频率。不支持 `--fwbw`。

<a id="what-happens-in-an-episode"></a>

## 回合内的处理流程

1. 环境 reset（机器人回 home，夹爪打开）。
2. 仅 `joint_abs`：录制器读取一次 leader，与 follower 的 `joint_pos` 比较。任一关节差值超过 `--start-tolerance-rad` 时停止并给出逐关节误差：先将 leader 对齐 follower 位姿再重启。低于容差时由起始策略（`--teleop-onset`）处理接管：`ramp` 每步以 `--teleop-ramp-step-rad` 将 follower 移向 leader，`arm` 等待 leader 进入 `--teleop-arm-tolerance-rad`。读取通过未激活接管开关的运动设备进行，因此起始策略不会隐藏差距，也不会消费事件设备按钮。
3. 每步录制器发送保持动作；操作员介入时，介入包装器改为执行操作员动作。保持动作将机器人留在原处：`joint_delta`（以及双臂 `cartesian_delta`）为零机械臂动作，`joint_abs` 为 follower 当前关节位置，单臂 `cartesian_delta` 则为返回固定保持位姿的动作（reset 后及每次交还时采样；`leader_cartesian` 使用输入源自身 tracker，见 [teleop.md](teleop.md#mapping)），避免机械臂漂移。距保持位姿超过一步（`--real-robot.xyz-scale-m`/`--real-robot.rot-scale-rad`）时发送饱和动作，逐步返回。夹爪维度保持最后执行的夹爪动作（reset 后为打开），每臂一个：夹爪是最后 `num_arms` 个动作维度（动作契约布局）。`run_recorder(args, hold_action_fn_factory=...)` 可注入其它保持策略（以构建后的环境调用）。
4. 每步都归档。pkl 视图选择介入步；两种关节模式的 leader 追赶均带 `teleop_syncing`，除非设置 `--keep-syncing-steps`，否则排除。归档派生训练视图仍排除 syncing。人工掩码选择专家维度，空闲保持补充动作没有专家标签。
5. 环境终止或截断时回合结束：操作员的回合结束按钮（键盘 Esc）截断，步数上限截断，成功键终止（默认 `--reward-from-teleop-events`），奖励分类器终止。仍在介入时按成功键：仅介入步进入示范记录，因此终止步也必须是介入步。

回合结果来自操作员事件（任意事件路径：键盘、终端、GELLO evdev 设备；SpaceMouse 按钮不产生这些事件）：

| 事件 | 作用 |
| --- | --- |
| `success`（`s`）或显式环境/分类器成功标签 | 回合计为成功，除非失败标签否决；仅正奖励不是成功标签 |
| `failure`（`f`） | 丢弃回合，即使先前已有成功标签 |
| `rerecord`（`r`） | 立即丢弃回合并开始下一回合 |

只保留至少有一个已记录步的成功回合；每个完成回合打印其结果。Ctrl-C 停止会话，仍保存此前保留的回合；`q` 键（`operator_quit`）结束当前回合，照常判定结果，然后以同样方式停止会话。

`joint_delta` 跳过开始检查：leader 读取每步在一个 `joint_scale_rad` 处饱和，无法测量更大的差距；相同饱和限制已经约束 follower 向 leader 运动的速度。

<a id="output"></a>

## 输出

pkl 包含成功人工转移字典，具有 `obs`、`next_obs`、实际执行的 `action`、`reward`、`done`（`terminated`）、可选 `grasp_penalty`、策略提议及人工/覆盖掩码。版本化记录还保留回合/步身份和训练片段边界，因此过滤不能虚构相邻关系或将终止奖励转移到更早的人工步。`<stem>.contract.json` 记录动作及实验兼容性，应与数据一起保留。参与比较的实验字段覆盖任务、控制、观测布局、预处理和遥操作部件选择/空闲行为。奖励来源和工作站所属 reset 事实属于溯源。

`<stem>.trajectory.sqlite3` 是完整持久归档，包含失败、重录、syncing 和中断尝试。只有 pkl 视图会丢弃不成功回合。没有保留回合时不写 pkl。在 pkl 路径仍未存在时重开已有归档，会将未完成回合标为中断，并为后续尝试分配新 ID。`--export-trajectory-pt` 还写入 `<stem>.trajectory.pt`。持久性从 append 返回时开始，使用 SQLite WAL `synchronous=NORMAL`；操作系统崩溃或断电可能丢失最新提交。完整恢复、掩码和训练视图规则见[真实机器人实验](real-world-experiments.md)。

<a id="single-attempt-commissioning"></a>

## 单次尝试上机验收

使用 `--max-episodes 1` 在一次尝试后结束，即使未保留成功示范。录制器在扫描归档或导出 pkl/PT 前关闭设备控制。成功、失败、重录、超时和中断均保留完整 SQLite 归档；只有成功人工示范产生 pkl。重启命令会开始新的 reset，包括回 home 和打开夹爪。每次重启前核实工作站与工作区。默认没有尝试上限，会继续到满足 `--num-successes`。

`rwrl archive inspect <archive>`（离线，无需工作站或机器人）只读访问归档；对于带 `tcp_pose` 的单臂状态，按 human_action_mask 片段报告保持漂移（也报告前 `--drift-window-s` 内的漂移）、残差、接管跳变（毫米），以及每个 `--stop-time-s` 后的稳定时间；`--measurements PATH` 将这些结果写为测试页的 `measurements.csv` 行，`--csv PATH` 导出逐步数据。带 `--stop-time-s` 时，所有停止都已稳定则退出 0，有一次未稳定退出 3，某个停止时间没有覆盖它的 TCP 回合则退出 4（[cli.md](cli.md#commands)）。`python examples/inspect_teleop_archive.py` 是其别名。

英文主版本测试页、记录 CSV 约定和证据要求见[硬件测试清单索引](testing/README.md)。硬件结果需在现场填写。
