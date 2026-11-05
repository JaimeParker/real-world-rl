<!-- English source: docs/tutorial/07-hil-training.md -->

<a id="7-hil-training"></a>

# 7. HIL 训练

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/07-hil-training.md)

三种方法共用 actor、learner、eval 角色、teleop 参数、示范 pkl 和同步协议。每次运行一种，各用独立 `--exp-name`：

| 方法 | 增加的机制 | 概念 |
|---|---|---|
| `hil_serl` | 人工介入 RLPD、增长示范缓冲、DQN 夹爪头 | [README](../project-readme.md#running-on-hardware)、[路线图](../hil-serl-roadmap.md) |
| `residual_hil_serl` | 在冻结 BC/ACT 基础上叠加有界残差 | [残差](../residual.md) |
| `silri` | 用学习的人类行为模型牵引 actor | [SiLRI](../silri.md) |

actor 是 **NUC 上唯一硬件进程**，直接连接 franky。learner 在 GPU 机器或同一 NUC，先启动。首次硬件运行前读 [06](06-intervening.md)。

全部命令在各机器的**仓库根、项目环境（project environment）激活**时运行。**learner 和 NUC 两个 shell**都设置下列变量；示例 `127.0.0.1` 是同机 learner，独立 GPU 时两边改为它的可达 IP。两份 checkout 的预设和实验输入一致。

```bash
LEARNER_IP=127.0.0.1
ENV_FLAGS="--config examples/presets/gello_cartesian.yaml --teleop-device leader_cartesian \
    --teleop-event-device stdin --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host $LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 20000 --demo-buffer-size 20000"
```

NUC 按 [04](04-record-demos.md#prepare-the-nuc) 配置 `gello_leader.yaml` 等现场值。预设仅状态、cartesian_delta、50 Hz，尺度 0.05 m / 0.3 rad。保持录制设备及四个 tracker 上限/速度。sidecar 的 `experiment.teleop_control` 记录版本和值，更改或缺失会拒绝。HEAD 比较 tracker 身份而非实体设备；同设备是教程操作要求，见[兼容性](04-record-demos.md#files-and-compatibility)。

使用 SpaceMouse 示范时，两边 `ENV_FLAGS` 改为 `--teleop-device spacemouse`、`--teleop-options-yaml None`，使用对应数据路径，四值不变。按键 0 接管，stdin 提供结果与退出键。只推杆不会接管。

以下硬件命令均包在 supervisor 中，心跳静默默认 **2.0 s**。显式 `--startup-s 60` 为首次 reset 心跳前的加载和初始快照等待留时间，须按本机实际确认；默认启动宽限 30 s。当前预设**环境 StepWatchdog 关闭**（未设 `real_robot.watchdog_s`），不同于 supervisor。[04](04-record-demos.md) 录制显式启用 0.3 秒 StepWatchdog；[实验指南](../real-world-experiments.md#step-watchdog-in-process)未指定 actor 值。

- `--reward-from-teleop-events` 令 `s` 给奖励 1 并终止，`f` 结束失败回合。训练默认关闭；也无分类器时所有奖励为零并警告，见 [06](06-intervening.md#where-the-reward-comes-from)。
- `--checkpoint-freq` 默认 0，仅最终保存；示例每 1000 次 learner 更新写 `checkpoint_<update>.pt` 和 bundle。
- `--buffer-size` 为 replay 容量，HIL/残差默认 200000、SiLRI 默认 30000，位于 `--buffer-device cuda`。多相机按显存预算或改 CPU。每个 bundle 将两缓冲完整容量复制到主存和磁盘；online/demo 容量都要核对。20000 行在 50 Hz 约 6 分 40 秒。每次保存创建新代，保留 manifest 引用的代与 journal。
- learner 不打开 teleop 硬件，但仍用 tracker 设置验证实验契约。

<a id="hil_serl"></a>

## hil_serl

```bash
# Learner machine: repository root, 项目环境激活.
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
# NUC: repository root, 项目环境激活.
python -m robot_infra.supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 在 reset/step 前等待初始快照，`--sync-startup-timeout` 默认 30 秒；快照仍可能是未训练策略。笛卡尔 teleop 不保证自主动作安全，随时准备 `t` 接管及物理停止。learner 预载示范，再等 `--learning-starts` 在线步（默认 4000，示范不计），然后训练。首个 ready 策略及之后每 `--publish-freq`（默认 100）次更新发布快照，session/version 跨重启标识版本。学习中准备介入。

<a id="residual_hil_serl-bc-base"></a>

## residual_hil_serl：BC 基础

在两边项目环境 shell 设置 `RESIDUAL`、`BASE_BC`；先在 learner 机器运行 learner，再从 NUC 仓库根运行监督 actor。

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
# NUC: repository root, 项目环境激活.
python -m robot_infra.supervise --startup-s 60 --log runs/residual-bc-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

<a id="residual_hil_serl-act-base"></a>

## residual_hil_serl：ACT 基础

同样两边设置 `RESIDUAL`、`BASE_ACT`，先 learner 后 NUC actor，均在各自仓库根和项目环境。

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
# NUC: repository root, 项目环境激活.
python -m robot_infra.supervise --startup-s 60 --log runs/residual-act-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

actor 执行 `clamp(base + scale * residual, -1, 1)`。人工默认覆盖总动作，部分接管用 [06 mask](06-intervening.md#take-over-selected-parts)。基础须来自同一笛卡尔预设和 tracker 契约、同一观测/动作空间，见 [05 契约](05-base-policy.md#the-act-checkpoint-contract)。spec 须加引号防止 `?` glob 和 `&` 后台。只有 actor/eval 构建 base，learner 用 actor 存的 base actions。ACT 每个介入步后重规划块，见[重新同步](05-base-policy.md#resync-after-an-intervention)。

- `hybrid` 对臂加残差、夹爪用 HIL DQN；`continuous` 表示全维，默认夹爪残差为零，保留 base 夹爪。
- `--residual-action-scale 0.1` 限制每步归一化残差。
- `--critic-only-updates 500` 前 500 次只训练 critic。
- `--residual-bc-weight 0.1` 在人工行上将残差拉向纠正。

纯 teleop 示范无 base actions，因此这里不传 `--demo-dataset-paths`。除非加 `--allow-base-less-demos`，否则拒绝；该开关将示范当零 base 残差学习，见[残差](../residual.md#human-intervention)。

<a id="silri"></a>

## silri

```bash
# Learner machine: repository root, 项目环境激活.
python examples/train_real_world.py silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
# NUC: repository root, 项目环境激活.
python -m robot_infra.supervise --startup-s 60 --log runs/silri-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py silri --role actor $COMMON --exp-name silri-pick-and-place
```

首次更新前，SiLRI 在 demo buffer 预训人类行为模型，`--bc-pretrain-steps` 默认 500；期间 actor 行仍进入 journal，之后消费。后续随介入刷新模型，见 [SiLRI](../silri.md#data-flow)。

<a id="stopping"></a>

## 停止

在安全状态结束回合，learner 仍运行时用 `q` 停 NUC actor，supervisor 终端 Ctrl-C 也请求清理。释放 env/provider 后尝试排空上传，`--sync-drain-timeout` 默认 30 秒。未确认行留在 outbox 下次发送。清理卡住时 supervisor 升级信号，最终杀进程断开 libfranka，依赖固件 Reflex。清故障重启前核对工位与日志。

随后停 learner；正常退出写 `runs/<exp-name>/checkpoints/final.pt` 和含模型、优化器、两缓冲、journal 位置的 bundle，经 `bundle_manifest.json` 发布。eval、deploy、base provider 读取 `config.json`。未送出行保存在 `runs/<exp-name>/actor/`，同路径同身份重启后继续发。此默认路径假设未覆盖 `--run-id`、`--actor-outbox-path`；并行 actor 各需独立 `--actor-id` 和 outbox。

<a id="evaluate"></a>

## 评估

机器人上确定性回合，无 learner 和数据采集。成功需要 `s`（启用按键奖励）或分类器的显式标签；整回合无接管才自主。先停 actor，勿介入，见 [06](06-intervening.md#the-sequence)。在 NUC、仓库根、项目环境激活，先定义上述 `ENV_FLAGS` 和对应 base 变量再执行各命令：

```bash
python -m robot_infra.supervise --startup-s 60 --log runs/hil-pick-and-place/eval-supervise.jsonl -- \
    python examples/train_real_world.py hil_serl --role eval $ENV_FLAGS --exp-name hil-pick-and-place \
    --load-checkpoint runs/hil-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python -m robot_infra.supervise --startup-s 60 --log runs/residual-bc-pick-and-place/eval-supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC" \
    --load-checkpoint runs/residual-bc-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python -m robot_infra.supervise --startup-s 60 --log runs/residual-act-pick-and-place/eval-supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT" \
    --load-checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
python -m robot_infra.supervise --startup-s 60 --log runs/silri-pick-and-place/eval-supervise.jsonl -- \
    python examples/train_real_world.py silri --role eval $ENV_FLAGS --exp-name silri-pick-and-place \
    --load-checkpoint runs/silri-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

自主评估保持 GELLO `h` 或 SpaceMouse 按钮 0 接管关闭；`s`/`f` 标结果，`q` 退出。接管即辅助。周期 checkpoint 同样可评估。*应该看到：* 每回合 `[eval] episode=... task_success=... autonomous_success=... return=...`，最终汇总自主成功率（完成回合中无接管成功）、任务成功率、介入率、平均回报。

<a id="what-to-watch"></a>

## 观察指标

在 learner 机器、仓库根、项目环境中查看 `--log-type`（默认 TensorBoard）写入 `runs/` 的日志：

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

**所有指标横轴为 learner 已接收 transition 数**，不是时间或更新次数。learner 按 `--log-freq` 边界记录（默认 1000，50 Hz 约 20 秒）；actor 每回合发布一组 `actor/*`。

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

磁盘还包括 actor 的 `outbox.sqlite3`、learner 的 `checkpoints/journal.sqlite3`、指向完整 `bundle_<update>_<id>/` 的 manifest，以及旧 `checkpoint_<update>.pt`。bundle 含模型/优化器、两缓冲和 journal 游标。旧 buffer/demo_buffer pkl 仅回退来源，不能在权威 bundle 上再次叠加加载。

<a id="optional-a-reward-classifier"></a>

## 可选：奖励分类器

主预设无 RGB，不可用腕部图像分类器。独立图像实验应先配置相机和图像观测，再录制、训练 base 和 HIL，整条图像契约一致。帧采集与分类器训练含终端标签见[真机实验](../real-world-experiments.md)、[teleop](../teleop.md#recording-demos-on-the-real-robot)。物理数据采集仍待验证；按键和分类器同时存在时取较大奖励。

<a id="if-not"></a>

## 失败排查

- 未知 real_robot 后端：在 NUC actor 所用解释器安装本包。
- 缺 leader 配置或读失败：查 `gello_leader.yaml`、已标定 leader-only YAML 和串口权限；读失败会停 actor，不会静默取消介入。
- `WORKSPACE HOLD`：按 [06](06-intervening.md#how-the-robot-follows-you) 恢复。
- `experiment.teleop_control` 差异：恢复录制设备、tracker 版本、上限和速度，也核对其他动作/观测差异。
- 启动前 supervisor 升级：查日志和启动宽限，不能与运行中不再心跳混为一谈。
- 缺 base-provider：每个残差角色都需要，包括 learner。

<a id="resume-an-interrupted-run"></a>

## 恢复中断实验

checkpoint、journal（含 WAL/SHM）和 bundle 各代一起保存。以相同预设、实验名、示范路径、容量先重启 learner。以上 HIL 示例：

```bash
# Learner machine: repository root, 项目环境激活.
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl --load-checkpoint \
    runs/hil-pick-and-place/checkpoints/bundle_manifest.json --checkpoint-dir \
    runs/hil-pick-and-place/checkpoints
# NUC: repository root, 项目环境激活.
python -m robot_infra.supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 复用 outbox，已确认 ID 只入库一次。learner/网络故障在容量内退避重试。其他方法用原参数及匹配 manifest/目录。仅移动 manifest 不会移动 journal；bundle 总是恢复完整 replay/cursor，不受 `--load-replay-buffer` 控制，代目录的 `agent.pt` 才是新运行的仅权重入口。独立实验用新 exp-name/run-id 或独立 outbox。

SQLite WAL、`synchronous=NORMAL` 保证进程崩溃恢复，OS 崩溃/断电可能回滚最新提交。journal 应用异常保留上一完整 bundle，恢复后重放尾部。路径、FWBW 方向隔离及旧格式恢复见[真机实验](../real-world-experiments.md)。

下一步：[08 部署](08-deploy.md)。
