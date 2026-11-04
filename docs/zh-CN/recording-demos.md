<!-- English source: docs/recording-demos.md -->

<a id="recording-demos-on-the-real-robot"></a>

# 在真实机器人上录制示范

[中文入口](../README.zh-CN.md) · [English](../recording-demos.md)

`examples/record_real_demos.py` 将真机遥操作的完整回合归档到 SQLite，并将成功人工示范导出为 `--demo-dataset-paths` 接受的 pkl 格式，供 `hil_serl`、`residual_hil_serl`、`silri`、`real_world_bc` 使用。BC/HIL 加载器还接受 `.trajectory.sqlite3` 和 `.trajectory.pt`。ACT 轨迹训练器使用 pkl。实现位于 `real_world_rl/demos/record.py`。

录制器与 HIL actor 使用完全相同的环境构建方式，即 `build_hil_env(args, role="actor")`，因此接受与 `train_real_world.py hil_serl` 相同的环境、遥操作、观测参数和 `--config` 预设。请采用后续加载示范的训练配置：录制的观测和动作必须与其环境匹配，加载时会检查 pkl 的约定附属文件。[教程 04](tutorial/04-record-demos.md) 讲解首次录制，[教程 06](tutorial/06-intervening.md) 说明按键、接管起始方式和记录内容。

<a id="commands"></a>

## 命令

GELLO 由机器人守护进程提供，follower 为 joint_delta（教程各阶段的配置），按键走键盘：

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --teleop-onset arm \
    --output-path demos/task_20.pkl --num-successes 20
```

joint_abs 数据用于独立 joint_abs 流水线，增加对应环境和 robot-teleop-action-mode 参数，预设需 joint_limits。`hil.yaml` 是[单臂预设](../../examples/presets/hil_single_arm.yaml)副本；列表对不能由 CLI 表达，因此限位来自 YAML。非 recorder 字段（算法超参数）跳过。

其他运动设备同理：本机 GELLO 用 `--teleop-device gello --teleop-options-yaml gello.yaml`；real_robot 或 franka_real 笛卡尔臂用 leader_cartesian，[50 Hz 单臂预设](../../examples/presets/gello_cartesian.yaml)及四个 tracker 参数见 [teleop](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)。SpaceMouse 用 `--teleop-device spacemouse --teleop-event-device stdin`。两种 tracker 输入源均将 teleop_control 写入契约，无 tracker 的部署设备会拒绝使用其训练策略。设备和 GELLO 设置见 [teleop](teleop.md)、[gello](gello.md)。

SpaceMouse 参数，详见 [SpaceMouse](teleop.md#spacemouse)：

| 参数 | 默认 | 含义 |
|---|---|---|
| `--teleop-device spacemouse` | | 本机设备，0 接管/交还，1 从最后命令切换夹爪，推杆为速度 |
| `--teleop-event-device stdin` / `keyboard` | | s/f/r、Esc、q；t/h 忽略，0 才是切换键 |
| `--teleop-target-speed-m-s`, `--teleop-target-speed-rad-s` | 0.05, 0.3 | 推杆全幅速度 |
| `--teleop-lead-cap-m`, `--teleop-lead-cap-rad` | 0.05, 0.3 | 命令领先实测上限，释放后在此距离内停止 |
| `--real-robot.xyz-scale-m`, `--real-robot.rot-scale-rad` | 0.02, 0.1 | 至少为领先上限，须传 0.05、0.3 |
| `--teleop-options-yaml`：`device_index`, `deadzone`, `invert_axes` | 0, 0.05, `[]` | 设备序号、各轴静止噪声死区（满幅比例）、翻转输出轴 0–5 |

keyboard（本机桌面）与 stdin（命令终端，含 SSH）的操作键：t 接管，h 交还，s 成功，f 失败，r 重录，Esc 结束回合，q 结束并在处理后停 recorder。

双臂用 real_robot_bimanual 和 bimanual teleop，每臂选 leader_arm/gello/robot。保持动作按各臂独立计算，joint_abs 用自身限位；初始逐臂比较并指出差距最大的臂。max-episode-steps 覆盖 bimanual 对应字段。任一臂介入记录该步，另一臂保持。命令见[双臂](bimanual.md#teleop-and-demo-recording)。

再用示范训练：

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

1. reset，机器人回 home、夹爪打开。
2. 仅 joint_abs：读 leader 与 follower joint_pos，任一差距超过 start-tolerance-rad 则报逐关节误差并停；对齐重启。容差内 onset ramp 按 ramp-step-rad 跟随，arm 等待 arm-tolerance-rad。读取时开关释放，不隐藏差距、不消耗事件按钮。
3. 每步发 hold，人工介入时 wrapper 改执行人工。joint_delta 和双臂 cartesian_delta 为零臂动作，joint_abs 为实测关节，单臂 cartesian_delta 返回固定 hold（reset 与交还采样，leader_cartesian 共用源 tracker，见[映射](teleop.md#mapping)），避免漂移。距 hold 超一步尺度时动作饱和、逐步回位。夹爪保留最后执行命令，reset 后开；各臂独立，位于动作末尾 num_arms 维。`run_recorder(args, hold_action_fn_factory=...)` 可注入另一 hold policy，接收构建后的环境。
4. 每步归档，pkl 选介入步。两关节模式初始追赶标 teleop_syncing，默认排除，keep-syncing-steps 可保留；归档训练视图仍排除。human mask 选专家维度，idle hold 补位无专家标签。
5. Esc、步数上限 truncated，成功键在默认按键奖励下 terminated，分类器亦可终止。成功键须在介入中按，终止步才进人工 pkl。

事件来自键盘、终端、GELLO evdev；SpaceMouse 按钮不产生结果事件：

| 事件 | 作用 |
|---|---|
| success（s）或环境/分类器显式成功 | 未被 failure 否决才成功，正奖励不是标签 |
| failure（f） | 丢弃，即使先前 success |
| rerecord（r） | 立即丢弃并开始下一回合 |

只保留至少一步的成功回合，打印每回合结果。Ctrl-C 保存已保留回合并停；q（operator_quit）按正常规则判断当前回合后同样停止。

joint_delta 跳过开始差距检查，因为 leader 读数已饱和到每步 joint_scale_rad，不能测更大差距，同时该饱和限制跟随速度。

<a id="output"></a>

## 输出

pkl 包含成功人工 transition 字典：`obs`、`next_obs`、实际执行 `action`、`reward`、`done`（`terminated`）、可选 `grasp_penalty`、策略提议以及人工/覆盖掩码。版本化记录还保留回合/步骤身份及训练片段边界，筛选不会虚构相邻关系，也不会把终止奖励移到更早的人工步骤。`<stem>.contract.json` 记录动作与实验兼容性，需与数据一起保留。实验比较字段涵盖任务、控制、观测布局、预处理、遥操作部件选择和空闲行为；奖励来源及工作站 reset 事实属于来源元数据。

`<stem>.trajectory.sqlite3` 是完整持久归档，包含失败、重录、同步和中断尝试。只有 pkl 视图删除不成功回合。没有保留数据时不写 pkl。在 pkl 路径仍未存在时重开已有归档，会将未完成回合标为中断，并为后续尝试分配新 ID。`--export-trajectory-pt` 另写 `<stem>.trajectory.pt`。持久性从 append 返回起生效，使用 SQLite WAL `synchronous=NORMAL`；操作系统崩溃或断电可能丢失最新提交。完整恢复、掩码和训练视图规则见[真机实验](real-world-experiments.md)。

<a id="single-attempt-commissioning"></a>

## 单次尝试上机验收

使用 `--max-episodes 1` 可在一次尝试后结束，即使没有保留成功示范。录制器在扫描归档和导出 pkl/PT 前关闭设备控制。成功、失败、重录、超时和中断均保留完整 SQLite 归档；只有成功人工示范生成 pkl。再次启动命令会执行新一轮 reset，包含回 home 和开夹爪，每次重启前核实工作站与工作区。未设置尝试次数上限时，默认持续录制到 `--num-successes` 满足。

`examples/inspect_teleop_archive.py <archive>` 只读归档；对带 `tcp_pose` 的单臂状态，按 human_action_mask 分段报告保持漂移（含前 `--drift-window-s` 秒的窗口值）、残余位移和接管跳动（mm），以及每个 `--stop-time-s` 之后的稳定时间；`--measurements PATH` 把它们写成测试页 `measurements.csv` 的行。

参见[真机测试页索引（英文主版本）](../testing/README.md)，查看英文主测试页、记录 CSV 约定与证据要求。真机结果在现场填写。
