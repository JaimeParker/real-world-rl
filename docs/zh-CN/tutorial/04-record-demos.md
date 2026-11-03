<a id="4-record-demos"></a>

# 4. 录制示范

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/04-record-demos.md)

`examples/record_real_demos.py` 采用与 HIL actor 完全相同的方式构建环境，只把人工介入步骤记录到示范 pkl，供 BC、ACT 和 HIL learner 加载。先读 [06 人工介入](06-intervening.md)，它是本步骤与 [07](07-hil-training.md) 共用的操作员指南。

先启动守护进程（[03](03-daemon.md)），再在 actor 机器上运行：

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard"
python examples/record_real_demos.py $ENV_FLAGS --teleop-onset arm \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

- `--teleop-device leader_arm` 读取由守护进程作为机器人 `teleop:` 部件提供的 GELLO。`--robot-teleop-action-mode` 在远程机器人连接前声明该部件；对主臂只需指定关节空间，实际输出由环境 `action_mode` 决定。
- `--teleop-event-device keyboard` 提供人工介入切换与回合按键。
- `--teleop-onset arm` 让切换信号在主臂与从臂距离小于 `--teleop-arm-tolerance-rad`（0.1）时才生效，使每条示范从静止且对齐的机器人开始，见 [06](06-intervening.md#onset-arm-or-ramp)。
- 录制器默认从按键生成奖励；仅录制器默认打开 `reward_from_teleop_events`，可用 `--no-reward-from-teleop-events` 关闭。按 `s` 会将当前步骤保存为成功终止步骤，奖励为 1、`done` 为 True，并结束回合。
- `--max-episode-steps N` 为本次录制覆盖预设的 `max_episode_steps`；10 Hz 下 200 步等于 20 秒。

<a id="an-episode"></a>

## 一个回合

1. 环境 reset，机械臂归位，夹爪打开。
2. 将 GELLO 移到从臂姿态并按 **Space**。每个关节与从臂的差值都在 0.1 rad 内时开始接管。
3. 移动 GELLO 接近物体，捏扳机闭合夹爪，将物体移至箱子，再松开扳机。
4. 在仍然接管时按 **s**，回合以成功结束，机械臂归位。

**f** 丢弃当前回合，**r** 丢弃并立即开始下一回合，**Esc** 以未成功结束并丢弃。只保留成功且至少有一个已录制步骤的回合；Ctrl-C 停止录制并保存此前保留的数据。按 `s` 时要保持接管：录制器只保存介入步骤，因此交还控制后再按 `s` 虽然会结束回合，终止步骤却不会进入示范，保留的数据将全程奖励为 0。

*预期结果*：每个已结束回合打印一行，最后打印保存结果。下方数字仅为示例：

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
...
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

同时得到两个文件：`demos/pick-and-place.pkl`，以及记录动作模式、维度、关节限制、缩放和状态字段的约定附属文件 `demos/pick-and-place.contract.json`。后续加载器会同时读取两者；环境不匹配时抛出 `ValueError` 并指出不同的键。`--output-path` 必须指向新的 `.pkl` 文件。各次示范应改变物体初始姿态。

*结果不符时：*

- 任务完成前回合就结束：增大 `hil.yaml` 中的 `max_episode_steps`，或传入 `--max-episode-steps`。
- Space 没有效果：主从臂关节误差超过 0.1 rad。actor 每秒记录一次各关节误差，将主臂移至从臂姿态。
- GELLO 完全读不到数据：查看守护进程的 `feedback:teleop` doctor 检查，或录制器抛出的 `DeviceFault`。
- `--keep-syncing-steps` 和 `--start-tolerance-rad` 详见 [recording-demos.md](../recording-demos.md#recorder-flags)。

> **joint_abs。** 使用预设副本 `hil_abs.yaml`，以 `joint_abs` 模式另录一套数据；其中设置 `action_mode: joint_abs` 和 `joint_limits`，见 [02](02-station.md#the-env-preset-hilyaml)。这套示范只加载到 `joint_abs` 运行中。
>
> ```bash
> python examples/record_real_demos.py --config hil_abs.yaml \
>     --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \
>     --output-path demos/pick-and-place-abs.pkl --num-successes 20
> ```
>
> 每个回合开始前，录制器读取一次主臂。任一关节与从臂的误差超过 `--start-tolerance-rad`（0.8）时，程序打印各关节误差并停止；对齐后重新启动。默认的 `ramp` 接管方式让从臂以每步 `--teleop-ramp-step-rad`（0.05）靠近主臂；除非启用 `--keep-syncing-steps`，这些步骤不会写入示范。`joint_delta` 跳过起始检查，因为主臂读数已经按每步一个 `joint_scale_rad` 饱和。

更多细节见 [recording-demos.md](../recording-demos.md)。

下一步：[05 基础策略](05-base-policy.md)。
