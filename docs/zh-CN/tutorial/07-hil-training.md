<a id="7-hil-training"></a>

# 7. HIL 训练

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/07-hil-training.md)

三个方法共用 actor、learner、eval 角色、遥操作参数、示范 pkl 和同步协议。每次运行一种方法，并分别设置 `--exp-name`：

| 方法 | 增加的能力 | 概念说明 |
|---|---|---|
| `hil_serl` | RLPD 加人工介入、持续增长的示范缓冲区与 DQN 夹爪头。 | [README](../project-readme.md#running-on-hardware)、[hil-serl-roadmap.md](../hil-serl-roadmap.md) |
| `residual_hil_serl` | 在冻结基础策略上学习有界残差，基础策略来自 [05](05-base-policy.md) 的 BC 或 ACT。 | [residual.md](../residual.md) |
| `silri` | 将 actor 拉向从人工介入中学习的行为模型。 | [silri.md](../silri.md) |

进程位置：守护进程运行在机器人 PC 上（[03](03-daemon.md)，`--watchdog-s 5`），learner 在 GPU 机器上，actor 在 actor 机器上。learner 绑定 `--sync-host`，actor 连接该地址，因此两者都填写 learner 地址。先启动 learner。首次启动 actor 前阅读 [06 人工介入](06-intervening.md)。

```bash
ENV_FLAGS="--config hil.yaml --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 100000"
```

- `--reward-from-teleop-events`：按 `s` 生成奖励 1 并终止回合，`f` 以失败结束。训练默认关闭此选项；它和 `--classifier-checkpoint` 都未设置时，所有 transition 奖励为 0，actor 与 learner 会警告，见 [06](06-intervening.md#where-the-reward-comes-from)。
- `--checkpoint-freq` 指定每隔多少次 learner 更新保存 `checkpoint_<update>.pt`。默认 0，仅在 learner 停止时保存 `final.pt`。
- `--buffer-size` 是回放容量：`hil_serl` 和 `residual_hil_serl` 默认 200000，`silri` 默认 30000，存放在 `--buffer-device cuda`。多台 128 × 128 相机时应按显存调整，或改用 `--buffer-device cpu`。
- learner 忽略遥操作参数，不会打开设备。

<a id="hil_serl"></a>

## hil_serl

```bash
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

learner 将示范加载到示范缓冲区，等待收到 `--learning-starts` 条在线 transition 后开始训练，默认 4000，预加载示范不计入。之后每 `--publish-freq` 次更新发布一次权重，默认 100。首批权重到达前 actor 使用未训练策略，需要准备接管。

<a id="residual_hil_serl-bc-base"></a>

## residual_hil_serl 使用 BC 基础策略

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

<a id="residual_hil_serl-act-base"></a>

## residual_hil_serl 使用 ACT 基础策略

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

actor 执行 `clamp(base + scale * residual, -1, 1)`，人工介入会覆盖总动作。基础策略必须来自同一套 `hil.yaml`，观测与动作空间一致，见 [05 的约定](05-base-policy.md#the-act-checkpoint-contract)。基础策略描述需要引号，因为 `?` 是通配符、`&` 表示后台运行。只有 actor 和 eval 构建基础策略；learner 使用 actor 保存的基础动作训练。采用 ACT 时，actor 在每个介入步骤之后重新规划动作块，见[重新同步](05-base-policy.md#resync-after-an-intervention)。

- `--residual-variant hybrid` 仅对机械臂维度加残差，夹爪使用 `hil_serl` 的 DQN 头；`continuous` 对全部维度建模，并保留基础策略的夹爪。
- `--residual-action-scale` 限制残差幅度，此处为每步归一化动作的 0.1。
- `--critic-only-updates 500`：前 500 次 learner 更新只训练 critic。
- `--residual-bc-weight 0.1`：在介入数据上将残差拉向人工纠正。

遥操作录制数据没有基础动作，因此这里没有通过 `--demo-dataset-paths` 传入示范 pkl。默认情况下 learner 会拒绝该数据；使用 `--allow-base-less-demos` 会按零基础动作上的残差学习这些数据，见 [residual.md](../residual.md#human-intervention)。

<a id="silri"></a>

## silri

```bash
python examples/train_real_world.py silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
python examples/train_real_world.py silri --role actor $COMMON --exp-name silri-pick-and-place
```

SiLRI learner 首次更新前会用示范缓冲区预训练人工行为模型，`--bc-pretrain-steps` 默认 500；期间阻塞数据接收。之后随人工介入数据到达定期刷新模型，见 [silri.md](../silri.md#data-flow)。

<a id="stopping"></a>

## 停止

先用 Ctrl-C 停止 actor，机械臂保持当前位置，再停止 learner；learner 退出时写入 `runs/<exp-name>/checkpoints/final.pt`。eval、deploy 和残差基础策略提供器读取 `runs/<exp-name>/config.json`。

<a id="evaluate"></a>

## 评估

在机器人上执行确定性回合，无 learner、无数据采集。成功定义为正常终止的回合，由启用 `--reward-from-teleop-events` 后的 `s` 键或分类器判定。先停止 actor，评估时保持策略自主执行，见 [06](06-intervening.md#the-sequence)：

```bash
python examples/train_real_world.py hil_serl --role eval $ENV_FLAGS --exp-name hil-pick-and-place \
    --load-checkpoint runs/hil-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC" \
    --load-checkpoint runs/residual-bc-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT" \
    --load-checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python examples/train_real_world.py silri --role eval $ENV_FLAGS --exp-name silri-pick-and-place \
    --load-checkpoint runs/silri-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

定期 checkpoint `checkpoint_<update>.pt` 的评估方式相同。*预期结果*：每回合输出 `[eval] episode=... success=... return=...`，最后输出 `[eval] success_rate=...`。

<a id="what-to-watch"></a>

## 观察哪些指标

learner 将日志写入 `runs/`，类型由 `--log-type` 指定，默认 TensorBoard：

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

**所有指标的横轴都是 learner 已收到的 transition 数量**。learner 按 `--log-freq` 边界记录，默认每 1000 条 transition，10 Hz 下约 100 秒；actor 每回合提交一组 `actor/*` 标量。

| 指标 | 方法 | 观察要点 |
|---|---|---|
| `actor/return`、`actor/success` | 全部 | 成功率应上升；成功表示回合正常终止，由 `s` 标记。 |
| `actor/intervention_rate`、`actor/intervened_steps`、`actor/intervention_count` | 全部 | 应随策略改进逐渐下降。 |
| `actor/length`、`actor/human_episode_end` | 全部 | 观察用 Esc 结束的回合。 |
| `actor/safety_clips`、`actor/safety_clips_total` | 全部 | 守护进程裁剪的伺服命令数；持续增加表示策略频繁请求越界目标。 |
| `actor/rtt_ms_mean/max`、`actor/step_time_ms_mean/max`、`actor/param_refresh_ms_mean/max`、`actor/dropped_transitions` | 全部 | 时间开销与网络健康状况。 |
| `learner/received`、`learner/update`、`learner/buffer_size`、`learner/train_time_s` | 全部 | 数据接收与训练进度。 |
| `learner/demo_buffer_size`、`learner/intervened_received` | 全部 | 示范缓冲区仅在人工介入时增长。 |
| `losses/critic_loss`、`losses/qf_loss`、`losses/actor_loss`、`losses/alpha_loss`、`entropy/alpha`、`entropy/target_entropy`、`q/predicted` | `hil_serl`、`residual_hil_serl` | SAC 风格损失；`q/predicted` 应随回报变化。 |
| `losses/actor/residual_bc`、`losses/actor/intervened_fraction` | `residual_hil_serl` | BC 项与批次介入数据占比，仅在只更新 critic 的阶段结束后出现。 |
| `losses/critic/loss`、`losses/actor/loss`、`losses/actor/q`、`losses/actor/distance`、`losses/lambda/loss`、`losses/lambda/mean`、`losses/behavior/sigma_sum`、`losses/batch/intervened_fraction`、`losses/discrete/ce` | `silri` | 比较 `losses/actor/distance` 和 `losses/behavior/sigma_sum` 加 margin，观察策略受人工行为约束的位置。 |
| `behavior/pretrained`、`behavior/pretrain/*`、`behavior/refresh/*`、`behavior/refresh_count` | `silri` | 行为模型预训练与刷新。 |

磁盘上还会保存：`runs/<exp-name>/checkpoints/` 中每隔 `--checkpoint-freq` 次更新生成的 `checkpoint_<update>.pt`，以及旁边用于崩溃恢复的快照。`buffer/transitions_<n>.pkl` 包含全部 transition，`demo_buffer/transitions_<n>.pkl` 仅包含介入步骤；每收到 `--buffer-period` 条 transition 保存一对，默认 1000。以相同 `--exp-name` 重启 learner 会重新加载这些快照。

<a id="optional-a-reward-classifier"></a>

## 可选的奖励分类器

腕部图像分类器可单独提供奖励，也可与 `s` 键共同使用。采集与训练两步都尚未在真实数据上验证。保持守护进程运行，照常用 GELLO 操作并采集标注帧：物体进入箱子后按 `s`，在成功区域内移动；离开成功区域时按 `f`。`s` 会锁存成功标签，直到按 `f` 或回合结束：

```bash
python -m real_world_rl.reward.collect_data --env-backend real_robot --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard \
    --output-dir classifier_data --exp-name pick-and-place --successes-needed 200
python -m rl_garden.models.reward.success.train --success_paths 'classifier_data/*success*.pkl' \
    --failure_paths 'classifier_data/*failure*.pkl' --output_dir runs/classifier-pick-and-place
```

训练器的冻结 ResNet 需要预训练权重，通过 `--pretrained_weights` 指定，见 `rl-garden/rl_garden/encoders/resnet.py`。之后在 `ENV_FLAGS` 中增加 `--classifier-checkpoint runs/classifier-pick-and-place/success_classifier.pt --classifier-image-keys rgb_wrist`。`--classifier-threshold` 默认 0.5；同时使用两种奖励来源时取较大值。

<a id="if-not"></a>

## 结果不符时

- reset 后第一步出现 `DeviceFault: the daemon watchdog stopped the arm ...`：守护进程的 `--watchdog-s` 太小。
- `Unknown env backend 'real_robot'`：没有安装 `real-world-rl` 软件包。
- `TransportError: robot daemon unreachable at <addr>`：守护进程未运行，或 `hil.yaml` 中的 `remote` 有误。
- actor 因遥操作的 `DeviceFault` 停止：GELLO 读取失败，读取故障会向上传播。
- `ValueError: ... recorded action contract does not match the env's`：示范 pkl 使用了不同的预设录制。
- `--base-provider is required`：`residual_hil_serl` 的每个角色都需要该参数，包括 learner。

下一步：[08 部署](08-deploy.md)。
