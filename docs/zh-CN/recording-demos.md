<a id="recording-demos-on-the-real-robot"></a>

# 在真实机器人上录制示范

[中文入口](../README.zh-CN.md) · [English](../recording-demos.md)

`examples/record_real_demos.py` 把真机遥操作示范录制为 `--demo-dataset-paths` 可加载的 pkl 格式，供 `hil_serl`、`residual_hil_serl`、`silri` 和 `real_world_bc` 使用。逻辑位于 `real_world_rl/demos/record.py`。

录制器与 HIL actor 使用完全相同的环境构建方式，即 `build_hil_env(args, role="actor")`，因此接受与 `train_real_world.py hil_serl` 相同的环境、遥操作、观测参数和 `--config` 预设。请采用后续加载示范的训练配置：录制的观测和动作必须与其环境匹配，加载时会检查 pkl 的约定附属文件。[教程 04](tutorial/04-record-demos.md) 讲解首次录制，[教程 06](tutorial/06-intervening.md) 说明按键、接管起始方式和记录内容。

<a id="commands"></a>

## 命令

由机器人守护进程提供 GELLO 服务，从臂使用教程全流程采用的 `joint_delta`，按键来自键盘：

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --teleop-onset arm \
    --output-path demos/task_20.pkl --num-successes 20
```

为 `joint_abs` 流程录制 `joint_abs` 数据时，增加 `--real_robot.action_mode joint_abs --robot-teleop-action-mode joint_abs`，并在预设中提供 `joint_limits`。

`hil.yaml` 由训练预设 [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml) 复制而来。`joint_limits` 必须在预设中配置，因为命令行无法表达成对列表。预设中不属于录制器字段的键，例如算法超参数，会被跳过。

其他运动输入设备采用相同方式：本机 GELLO 使用 `--teleop-device gello` 并配合 `--teleop-options-yaml gello.yaml`；笛卡尔环境使用 `--teleop-device spacemouse`；`franka_real` 使用 `--teleop-device leader_cartesian`。GELLO 设置见 [gello.md](gello.md)，设备说明见 [teleop.md](teleop.md)。

双臂环境使用 `--env-backend real_robot_bimanual`，配合 `--teleop-device bimanual --teleop-arm-device leader_arm`，也可选择 `gello` 或 `robot`。每臂读取一套主臂，录制器逐臂处理。保持动作按每臂独立计算，`joint_abs` 使用各从臂自身关节与限制；回合起始检查逐一比较主从臂，并指出误差最大的臂。`--max-episode-steps` 覆盖 `--real_robot_bimanual.max_episode_steps`。任一操作员介入时记录该步，另一臂保持静止。命令见 [bimanual.md](bimanual.md#teleop-and-demo-recording)。

随后使用示范训练：

```bash
python examples/train_real_world.py hil_serl --role learner --config hil.yaml \
    --demo-dataset-paths demos/task_20.pkl ...
```

<a id="recorder-flags"></a>

## 录制器参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `--output-path` | 必填 | 示范 pkl，必须以 `.pkl` 结尾且尚不存在；同时在旁边写入 `<stem>.contract.json`。 |
| `--num-successes` | 20 | 保留指定数量的成功回合后停止。 |
| `--max-episode-steps` | 未设置 | 回合步数上限；设置后覆盖 `--real_robot.max_episode_steps`、`--real_robot_bimanual.max_episode_steps` 或 `--franka_real.max_episode_steps`。 |
| `--start-tolerance-rad` | 0.8 | `joint_abs` 中任一主从臂关节误差超过该值时，拒绝开始回合。 |
| `--keep-syncing-steps` | 关闭 | 同时录制 ramp 接管初期的同步步骤，详见下文。 |
| `--reward-from-teleop-events` | 开启，仅录制器如此；训练默认关闭 | 成功键将该步存为成功终止步骤并结束回合；奖励为 `--teleop-success-reward`，默认 1.0，`done` 为 True。`--no-reward-from-teleop-events` 保留环境自身奖励，`real_robot` 为 0，因此保留的成功示范奖励为 0、`done` 为 False。 |

`--control-hz` 像控制 actor 一样控制录制循环频率。暂不支持 `--fwbw`。

<a id="what-happens-in-an-episode"></a>

## 回合内的处理流程

1. 环境 reset，机器人归位、夹爪打开。
2. 仅 `joint_abs`：读取一次主臂并与从臂的 `joint_pos` 比较。任一关节误差超过 `--start-tolerance-rad` 时，打印逐关节误差并停止；对齐主从臂后重启。误差在容差内时，由 `--teleop-onset` 处理接管：`ramp` 以每步 `--teleop-ramp-step-rad` 向主臂靠近，`arm` 等待误差小于 `--teleop-arm-tolerance-rad`。读取时通过运动设备且释放切换信号，因此接管策略不会掩盖误差，也不会消耗事件设备的按键。
3. 录制器每步发送保持动作；介入时包装器改为执行操作员动作。保持动作使机器人停留原处：`joint_delta` 与 `cartesian_delta` 的机械臂动作为零，`joint_abs` 使用从臂当前关节位置，夹爪保持最后执行的动作，reset 后为打开。每臂各有一个夹爪，夹爪占据最后 `num_arms` 个动作维度，遵循动作约定布局。
4. 仅录制介入步骤。ramp 起始限速步骤标记为 `teleop_syncing`，默认排除，`--keep-syncing-steps` 可保留。
5. 环境终止或截断时回合结束：操作员结束键（键盘 Esc）和步数上限产生截断；默认开启 `--reward-from-teleop-events` 时，成功键产生终止；奖励分类器也可产生终止。成功键应在仍介入时按下，因为只有介入步骤被保存，成功终止步也必须包含在其中。

回合结果来自操作员事件，各事件路径均适用，包括键盘、SpaceMouse 按钮、GELLO 的 evdev 设备：

| 事件 | 效果 |
| --- | --- |
| `success`（`s`）或正奖励 | 回合计为成功，默认在当前步结束。 |
| `failure`（`f`） | 丢弃回合，即使之前已标记成功。 |
| `rerecord`（`r`） | 立即丢弃当前回合并开始下一回合。 |

只保留至少录制一步的成功回合，每个已结束回合都会打印结果。Ctrl-C 停止录制，仍会保存此前保留的回合。

`joint_delta` 跳过起始检查：主臂读数按每步一个 `joint_scale_rad` 饱和，无法测量更大的间距；该饱和也已限制从臂靠近主臂的速度。

<a id="output"></a>

## 输出

pkl 是 transition 字典的列表，与 hil_serl learner 示范快照格式相同：`obs` 和 `next_obs` 保持环境返回值，`action` 为实际执行动作（`info["intervene_action"]`，形状 `(1, A)`），还包括 `reward`、`done`（即 `terminated`）；启用 `--use-grasp-penalty` 时还保存 `grasp_penalty`。附属文件 `<stem>.contract.json` 保存动作模式、维度、关节限制、缩放和状态字段。加载到不同约定的环境时会抛出 `ValueError`，指出不一致的键。
