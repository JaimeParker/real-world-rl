<a id="recording-demos-on-the-real-robot"></a>

# 在真实机器人上录制示范

[中文入口](../README.zh-CN.md) · [English](../recording-demos.md)

`examples/record_real_demos.py` 将真机遥操作的完整回合归档到 SQLite，并将成功人工示范导出为 `--demo-dataset-paths` 接受的 pkl 格式，供 `hil_serl`、`residual_hil_serl`、`silri`、`real_world_bc` 使用。BC/HIL 加载器还接受 `.trajectory.sqlite3` 和 `.trajectory.pt`。ACT 轨迹训练器使用 pkl。实现位于 `real_world_rl/demos/record.py`。

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
| `--output-path` | 必填 | 示范 pkl，必须以 `.pkl` 结尾且尚不存在；旁边写入 `<stem>.contract.json`。 |
| `--num-successes` | 20 | 保留指定数量的回合后停止。 |
| `--max-episodes` | 未设置 | 完成指定次数尝试后停止，失败、重录、超时均计入；该上限与 `--num-successes` 中先满足者结束录制。 |
| `--max-episode-steps` | 未设置 | 回合步数上限；设置后替换 `--real_robot.max_episode_steps`、`--real_robot_bimanual.max_episode_steps` 或 `--franka_real.max_episode_steps`。 |
| `--start-tolerance-rad` | 0.8 | `joint_abs` 下，任何关节的主从差距超过该值就拒绝开始回合。 |
| `--keep-syncing-steps` | 关闭 | 将主臂追赶记录作为普通监督步骤写入 pkl；SQLite 保留同步标记。 |
| `--export-trajectory-pt` | 关闭 | 额外将完整归档导出为可移植 `.trajectory.pt`；SQLite 仍为恢复来源。 |
| `--reward-from-teleop-events` | 开启，仅 recorder，训练默认关闭 | 成功键将当前步骤存为终止成功，奖励 `--teleop-success-reward`（1.0）、`done` 为 True，并结束回合；`--no-reward-from-teleop-events` 保留环境奖励（`real_robot` 为 0），因此保留的成功记录奖励为 0、`done` 为 False。 |

`--control-hz` 像控制 actor 一样控制录制循环频率。暂不支持 `--fwbw`。

<a id="what-happens-in-an-episode"></a>

## 回合内的处理流程

1. 环境 reset，机器人回到 home、夹爪打开。
2. 仅 `joint_abs`：录制器读取一次主臂，与从臂 `joint_pos` 比较。任何关节差距超过 `--start-tolerance-rad` 时，打印逐关节误差并停止；手动对齐后重启。容差内由 `--teleop-onset` 决定接管：`ramp` 每步按 `--teleop-ramp-step-rad` 追赶；`arm` 等待差距进入 `--teleop-arm-tolerance-rad`。读取经过运动设备且接管开关释放，因此不会被 onset 逻辑掩盖差距，也不消耗事件设备按钮。
3. 录制器每步发送保持动作；操作员介入时，介入包装器改为执行人工动作。保持动作让机器人停在当前位置：`joint_delta` 和 `cartesian_delta` 的机械臂动作为零，`joint_abs` 使用从臂当前关节位置，夹爪保持上次执行动作，reset 后为打开。每臂各有一项，夹爪位于动作最后 `num_arms` 维，遵守动作布局约定。
4. 所有步骤均归档。pkl 视图选择介入步骤；两种关节模式的主臂追赶均带 `teleop_syncing`，默认排除，设置 `--keep-syncing-steps` 可保留。由归档生成的训练视图仍排除同步步骤。人工掩码选择专家维度，空闲保持补充动作没有专家标签。
5. 环境终止或截断时回合结束：操作员结束按钮（键盘 Esc）和步数上限造成截断；成功键在默认 `--reward-from-teleop-events` 下造成终止，奖励分类器也可终止回合。在仍介入时按成功键，pkl 只记录介入步骤，终止步骤也需包含在内。

回合结果来自操作员事件，各事件路径均适用，包括键盘、SpaceMouse 按钮、GELLO 的 evdev 设备：

| 事件 | 效果 |
| --- | --- |
| `success`（`s`）或环境/分类器的显式成功标签 | 未被失败否决时计为成功；单独正奖励不能作为成功标签。 |
| `failure`（`f`） | 丢弃回合，即使先前已标成功。 |
| `rerecord`（`r`） | 立即丢弃回合，开始下一个。 |

只保留至少录制一步的成功回合，每个已结束回合都会打印结果。Ctrl-C 停止录制，仍会保存此前保留的回合。

`joint_delta` 跳过起始检查：主臂读数按每步一个 `joint_scale_rad` 饱和，无法测量更大的间距；该饱和也已限制从臂靠近主臂的速度。

<a id="output"></a>

## 输出

pkl 包含成功人工 transition 字典：`obs`、`next_obs`、实际执行 `action`、`reward`、`done`（`terminated`）、可选 `grasp_penalty`、策略提议以及人工/覆盖掩码。版本化记录还保留回合/步骤身份及训练片段边界，筛选不会虚构相邻关系，也不会把终止奖励移到更早的人工步骤。`<stem>.contract.json` 记录动作与实验兼容性，需与数据一起保留。实验比较字段涵盖任务、控制、观测布局、预处理、遥操作部件选择和空闲行为；奖励来源及工作站 reset 事实属于来源元数据。

`<stem>.trajectory.sqlite3` 是完整持久归档，包含失败、重录、同步和中断尝试。只有 pkl 视图删除不成功回合。没有保留数据时不写 pkl。在 pkl 路径仍未存在时重开已有归档，会将未完成回合标为中断，并为后续尝试分配新 ID。`--export-trajectory-pt` 另写 `<stem>.trajectory.pt`。持久性从 append 返回起生效，使用 SQLite WAL `synchronous=NORMAL`；操作系统崩溃或断电可能丢失最新提交。完整恢复、掩码和训练视图规则见[真机实验](real-world-experiments.md)。

<a id="single-attempt-commissioning"></a>

## 单次尝试上机验收

使用 `--max-episodes 1` 可在一次尝试后结束，即使没有保留成功示范。录制器在扫描归档和导出 pkl/PT 前关闭设备控制。成功、失败、重录、超时和中断均保留完整 SQLite 归档；只有成功人工示范生成 pkl。再次启动命令会执行新一轮 reset，包含回 home 和开夹爪，每次重启前核实工作站与工作区。未设置尝试次数上限时，默认持续录制到 `--num-successes` 满足。

完整的[双臂 Franka 与 GELLO 单机接管测试](testing/bimanual-franka-gello-smoke.md)包含安装、NUC 配置迁移、设备侧独立按键、操作步骤和 CSV 记录模板。独立 actor 服务器的网络联调另列后续案例。
