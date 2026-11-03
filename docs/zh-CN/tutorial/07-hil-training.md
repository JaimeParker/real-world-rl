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
    --checkpoint-freq 1000 --buffer-size 20000 --demo-buffer-size 20000"
```

- `--reward-from-teleop-events`：`s` 提供奖励 1 并终止回合，`f` 以失败结束。训练默认关闭；它与 `--classifier-checkpoint` 都未设置时，所有 transition 奖励为 0，actor 和 learner 会警告，见 [06](06-intervening.md#where-the-reward-comes-from)。
- `--checkpoint-freq` 默认 0，仅在结束时保存；设为正数后，每隔相应 learner 更新数写入 `checkpoint_<update>.pt` 与 checkpoint bundle。
- `--buffer-size` 是回放容量，`hil_serl`、`residual_hil_serl` 默认 200000，`silri` 默认 30000，通常位于 `--buffer-device cuda`。多台 128 × 128 相机时应按显存调整，或使用 `--buffer-device cpu`。每次 bundle 会把两个回放缓冲区按完整容量复制到主机内存和磁盘，因此 `--buffer-size` 与 `--demo-buffer-size` 都应按 GPU、主机内存和磁盘配置。10 Hz 下 20000 条约为 33 分钟。每次定期保存都会产生新一代 bundle，应将 manifest 引用的 generation 与 journal 一起保留。
- learner 忽略遥操作参数，不会打开设备。

<a id="hil_serl"></a>

## hil_serl

```bash
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 在 reset/step 前等待首个 learner 快照，最多 `--sync-startup-timeout`，默认 30 秒。首个快照仍可能是未训练策略；非残差 `joint_abs` 尚无实测姿态启动保持，所以本教程采用 `joint_delta`。learner 预加载示范，等待 `--learning-starts` 条在线 transition，默认 4000，示范不计入，然后训练。它会发布首个就绪策略，之后每 `--publish-freq` 次更新发布快照，默认 100。快照 session/version 可识别 learner 重启后的更新。学习初期需准备介入。

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

actor 执行 `clamp(base + scale * residual, -1, 1)`，默认人工覆盖总动作；按部件接管采用 [06](06-intervening.md#take-over-selected-parts) 的掩码。基础策略必须使用相同 `hil.yaml`，观测和动作空间一致，见 [05 的约定](05-base-policy.md#the-act-checkpoint-contract)。基础描述需加引号，`?` 会触发通配符，`&` 会后台运行。仅 actor 和 eval 构建基础策略，learner 用 actor 保存的基础动作训练。ACT 基础策略在每次介入后重新规划动作块，见[重新同步](05-base-policy.md#resync-after-an-intervention)。

- `--residual-variant hybrid` 仅对机械臂维度加残差，夹爪使用 `hil_serl` 的 DQN 头；`continuous` 对全部维度建模，默认夹爪残差缩放为零，保留基础策略的夹爪。
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

SiLRI learner 首次更新前用示范缓冲区预训练人工行为模型，`--bc-pretrain-steps` 默认 500。期间 actor 数据仍进入 learner journal，随后再应用到训练回放；之后随介入数据刷新行为模型，见 [silri.md](../silri.md#data-flow)。

<a id="stopping"></a>

## 停止

先使当前回合在安全状态结束，保持 learner 运行并 Ctrl-C 停止 actor。清理先释放环境和提供器，再尝试在 `--sync-drain-timeout` 内排空上传，默认 30 秒。超时未获确认的数据留在本地 outbox，重启后续传；强制终止仍依赖守护进程 watchdog。随后停止 learner，正常停止会保存 `runs/<exp-name>/checkpoints/final.pt` 与 checkpoint bundle，包含模型、优化器、两个回放缓冲区和 journal 位置，经 `bundle_manifest.json` 发布。eval、deploy 和基础策略提供器读取 `runs/<exp-name>/config.json`。actor 未送达的数据留在 `runs/<exp-name>/actor/` 下，使用同一 outbox 路径与身份重启后发送。该默认路径假设未覆盖 `--run-id`、`--actor-outbox-path`；并发 actor 各需独立 `--actor-id` 和 outbox。

<a id="evaluate"></a>

## 评估

评估在真机执行确定性回合，无 learner、无数据采集。成功需要显式成功标签，例如启用 `--reward-from-teleop-events` 后的 `s` 或分类器；整回合没有动作接管才算自主成功。先停止 actor，评估策略自主能力时保持未接管，见 [06](06-intervening.md#the-sequence)：

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

定期保存的 `checkpoint_<update>.pt` 以相同方式评估。*预期结果*：逐回合输出 `[eval] episode=... task_success=... autonomous_success=... return=...`，最终 `[eval] ...` 汇总 `autonomous_success_rate`、`task_success_rate`、介入率和 `mean_return`。自主成功率的分母为全部已完成回合，分子为没有接管的成功回合。

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
| `actor/return`、`actor/terminated`、`actor/task_success` | 全部 | 每回合回报、终止状态与显式成功标签；`terminated` 可来自 `s` 或分类器等任意终止步骤。 |
| `actor/autonomous_success_rate`、`actor/task_success_rate`、`actor/episode_intervention_rate` | 全部 | actor 启动后的累计值；已完成回合中的自主成功率应上升、介入率应下降。 |
| `actor/intervention_rate`、`actor/intervened_steps`、`actor/intervention_count` | 全部 | 随策略改进应下降。 |
| `actor/length`、`actor/human_episode_end` | 全部 | 用 Esc 结束的回合。 |
| `actor/safety_clips`、`actor/safety_clips_total` | 全部 | 守护进程裁剪的伺服命令数，增长表示持续请求越界目标。 |
| `actor/rtt_ms_mean/max`、`actor/step_time_ms_mean/max`、`actor/param_refresh_ms_mean/max` | 全部 | 时间开销。 |
| `actor/outbox_depth`、`actor/upload_retries` | 全部 | 磁盘待发送条数和失败尝试；积压增长表示上传待处理，重试保留 ID，容量耗尽或磁盘失败会停止采集。 |
| `learner/received`、`learner/update`、`learner/buffer_size`、`learner/train_time_s` | 全部 | 接收与训练进度。 |
| `learner/demo_buffer_size`、`learner/intervened_received` | 全部 | 示范缓冲区从非同步、有效人工监督数据增长。 |
| `losses/critic_loss`、`losses/qf_loss`、`losses/actor_loss`、`losses/alpha_loss`、`entropy/alpha`、`entropy/target_entropy`、`q/predicted` | `hil_serl`、`residual_hil_serl` | SAC 风格损失，`q/predicted` 应随回报变化。 |
| `losses/actor/residual_bc`、`losses/actor/intervened_fraction` | `residual_hil_serl` | BC 项和批次介入占比，在只更新 critic 阶段结束后观察。 |
| `losses/critic/loss`、`losses/actor/loss`、`losses/actor/q`、`losses/actor/distance`、`losses/lambda/loss`、`losses/lambda/mean`、`losses/behavior/sigma_sum`、`losses/batch/intervened_fraction`、`losses/discrete/ce` | `silri` | 比较 `losses/actor/distance` 与 `losses/behavior/sigma_sum` 加 margin，观察人工约束。 |
| `behavior/pretrained`、`behavior/pretrain/*`、`behavior/refresh/*`、`behavior/refresh_count` | `silri` | 行为模型预训练和刷新。 |

磁盘上还包含 actor 的 `outbox.sqlite3`、learner 的 `checkpoints/journal.sqlite3`、指向完整 `bundle_<update>_<id>/` 的 `bundle_manifest.json`，以及旧式 `checkpoint_<update>.pt`。bundle 保存模型、优化器、两个回放缓冲区和 journal 游标。旧 `buffer/`、`demo_buffer/` pkl 快照保留为回退来源，不会叠加加载到权威 bundle 上。

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

<a id="resume-an-interrupted-run"></a>

## 恢复中断的运行

将 learner checkpoint 目录、journal（包括 SQLite WAL/SHM）和 bundle 各代文件一起保留。先按相同预设、实验名、初始示范路径及缓冲区容量重启 learner。前述 `hil_serl` 运行示例如下：

```bash
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl --load-checkpoint \
    runs/hil-pick-and-place/checkpoints/bundle_manifest.json --checkpoint-dir \
    runs/hil-pick-and-place/checkpoints
python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 复用原 outbox，已确认 ID 只接收一次。learner 或网络中断时，只要 outbox 仍有容量便按退避策略重试。残差和 SiLRI 使用各方法原参数以及匹配的 manifest、目录。仅提供 manifest 不会迁移 journal；bundle 总会恢复完整回放与游标一致性单元，不受 `--load-replay-buffer` 开关影响。新运行仅初始化权重时使用 generation 的独立 `agent.pt`。独立实验使用新的 `--exp-name` / `--run-id` 或独立 outbox。

SQLite WAL 采用 `synchronous=NORMAL`，保证覆盖进程崩溃；操作系统崩溃或断电可能回滚最近提交。更新异常或 journal 应用中断会保留最后完整 bundle，恢复时重放 journal 尾部。完整路径、FWBW 方向隔离和旧格式恢复见[真机实验](../real-world-experiments.md)。

下一步：[08 部署](08-deploy.md)。
