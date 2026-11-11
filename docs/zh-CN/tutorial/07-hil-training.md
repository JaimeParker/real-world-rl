<!-- English source: docs/tutorial/07-hil-training.md -->

<a id="7-hil-training"></a>

# 7. HIL 训练

三种方法共用 actor、learner、eval 角色、遥操作参数、示范 pkl 和同步协议。每次运行一种，各自使用独立 `--exp-name`：

| 方法 | 新增能力 | 概念 |
|---|---|---|
| `hil_serl` | 带人工介入、增长示范缓冲区和 DQN 夹爪头的 RLPD | [README](../project-readme.md#running-on-hardware)、[hil-serl-roadmap.md](../hil-serl-roadmap.md) |
| `residual_hil_serl` | 冻结基础策略之上的有界残差（来自 [05](05-base-policy.md) 的 BC 或 ACT） | [residual.md](../residual.md) |
| `silri` | 被拉向已学习人工介入模型的 actor | [silri.md](../silri.md) |

actor 是 **NUC** 上唯一硬件进程，通过 franky 直连。learner 在 GPU 机器（或同一 NUC）运行，并先启动。启动硬件前阅读 [06 人工介入](06-intervening.md)。

使用已标定 SpaceMouse 工作站时，为录制、actor 和 learner 提供实际生效且匹配的任务坐标系（task frame）/工作区与标定档案（profile）设置。档案文件路径可在机器间不同，但数值变换必须一致；只改档案名不会改变数值契约。learner 不打开 HID 即解析这些设置。需要手动推进 actor 回合时，显式选择 `episode_end_behavior: hold`；它不会在介入之间插入回中检查。见 [SpaceMouse CLI 指南](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)和[物理验收检查](../testing/spacemouse-control-session.md)。

所有命令均在**仓库根目录**、对应机器的**项目环境激活**时运行。在 **learner 和 NUC 两边 shell** 设置以下变量。示例使用 `127.0.0.1` 表示 learner 位于同一机器；独立 GPU learner 需在两边 shell 将其替换为该机器可达 IP。确保两份源码检出使用相同预设和实验输入。

```bash
LEARNER_IP=127.0.0.1
ENV_FLAGS="--config examples/presets/gello_cartesian.yaml --teleop-device leader_cartesian \
    --teleop-event-device stdin --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host $LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 20000 --demo-buffer-size 20000"
```

在 NUC 使用工位专属的 [04 配置](04-record-demos.md#prepare-the-nuc)，包括 `gello_leader.yaml`。预设为仅状态 `cartesian_delta`、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`。保持录制设备及四个 tracker 上限/速度不变。示范附属文件的 `experiment.teleop_control` 记录 tracker 版本和值；更改或省略会被拒绝。当前 HEAD 比较 tracker 身份而非物理输入设备身份；教程将同一设备作为操作流程要求（[兼容性说明](04-record-demos.md#files-and-compatibility)）。

若改用 SpaceMouse 录制数据，在两边 shell 的 `ENV_FLAGS` 中设置 `--teleop-device spacemouse` 和 `--teleop-options-yaml None`，使用匹配示范路径，保持四个值不变。按钮 0 负责接管；stdin 提供 `s`/`f`/`r`/Esc/`q`。单纯移动手柄不会接管。

原生 SpaceMouse 介入需在 **NUC 和 learner 两边 shell** 的 `ENV_FLAGS` 添加 `--spacemouse-mode serl`，均在**仓库根目录**、各自**项目环境激活**时操作。使用有 xyz 和 RPY 安全盒的工作站预设，以及同模式录制的示范。模式解析在训练开始前保存 `real_robot.cartesian_semantics: serl`，无设备的 learner 也如此：原始六轴范数 `> 0.001` 时介入，释放手柄和按钮交还，按住按钮 0 关闭，按钮 1 打开，两者均未按下时发送夹爪中立零值。tracker 上限/速度不适用；参与比较的契约为 `spacemouse_serl_v1`。见[两种模式](../teleop.md#spacemouse)；硬件行为尚未验证。

下方每条硬件命令均由 `robot_infra.supervise` 包装。心跳静默阈值默认 **2.0 s**。`--startup-s 60` 为首次 reset 心跳前的策略初始加载和 actor 快照等待留出时间。检查此值是否超过本机启动耗时；默认启动宽限为 30 s。当前预设保持**环境 StepWatchdog 关闭**（未设置 `real_robot.watchdog_s`），它与 supervisor 独立。[04](04-record-demos.md) 录制显式启用 0.3 秒 StepWatchdog。当前[实验指南](../real-world-experiments.md#step-watchdog-in-process)未指定 actor 的 StepWatchdog 值。

- `--reward-from-teleop-events`：`s` 键产生奖励（1，终止），`f` 以失败结束回合。训练默认不启用；既无它也无 `--classifier-checkpoint` 时，每个转移奖励为 0，actor 和 learner 会警告（[06](06-intervening.md#where-the-reward-comes-from)）。
- `--checkpoint-freq`（默认 0，仅最终保存）每隔指定 learner 更新次数写入 `checkpoint_<update>.pt` 和检查点 bundle。
- `--buffer-size` 是回放容量（`hil_serl` 和 `residual_hil_serl` 默认 200000，`silri` 默认 30000），位于 `--buffer-device cuda`；多个 128 × 128 相机时按 GPU 显存调整，或用 `--buffer-device cpu`。每个检查点 bundle 都将两个回放缓冲区按完整容量复制到主机内存和磁盘。根据可用 GPU/主机内存及磁盘设置 `--buffer-size` 和 `--demo-buffer-size`；20000 行在 50 Hz 下约 6 分 40 秒。每次定期保存创建新一代。保留 manifest 引用的那一代及其 journal。
- learner 不打开遥操作设备，但仍使用 tracker 设置验证实验契约。

<a id="hil_serl"></a>

## hil_serl

```bash
# Learner machine: repository root, project environment active.
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
# NUC: repository root, project environment active.
python -m robot_infra.supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 在 reset/step 前等待 learner 初始快照，最多 `--sync-startup-timeout`（默认 30 秒）。该快照仍可能包含未训练策略；笛卡尔遥操作不会使其自主动作自动安全。随时准备按 `t` 接管，并在需要时使用物理停止装置。learner 预加载示范，等待 `--learning-starts` 个在线转移（默认 4000；预加载示范不计入）后训练。它发布首个就绪策略，之后每 `--publish-freq` 次更新发布快照（默认 100）。快照 session/version 标识 learner 重启前后的更新。策略学习时随时准备介入。

<a id="residual_hil_serl-bc-base"></a>

## residual_hil_serl：BC 基础

在两边 shell 激活项目环境并设置 `RESIDUAL` 和 `BASE_BC`；各自在仓库根目录，先在 learner 机器启动 learner，再在 NUC 启动受监督的 actor。

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
# NUC: repository root, project environment active.
python -m robot_infra.supervise --startup-s 60 --log runs/residual-bc-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

<a id="residual_hil_serl-act-base"></a>

## residual_hil_serl：ACT 基础

在两边 shell 激活项目环境并设置 `RESIDUAL` 和 `BASE_ACT`；各自在仓库根目录，先在 learner 机器启动 learner，再在 NUC 启动受监督的 actor。

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
python examples/train_real_world.py residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
# NUC: repository root, project environment active.
python -m robot_infra.supervise --startup-s 60 --log runs/residual-act-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

actor 执行 `clamp(base + scale * residual, -1, 1)`；默认人工介入覆盖总动作；部件选择接管使用 [06](06-intervening.md#take-over-selected-parts) 的掩码。基础策略必须来自同一笛卡尔预设和 tracker 契约（相同观测和动作空间，[05 的契约](05-base-policy.md#the-act-checkpoint-contract)）。基础策略规格加引号，因为 `?` 是 glob 字符，`&` 会将命令置于后台。只有 actor 和 eval 构建基础策略；learner 使用 actor 保存的基础动作训练。ACT 基础下，actor 每个介入步后重新规划动作块（[重新同步](05-base-policy.md#resync-after-an-intervention)）。

- `--residual-variant hybrid` 将残差用于机械臂维度，并给夹爪使用 `hil_serl` 的 DQN 头；`continuous` 表示所有维度，默认夹爪残差尺度为零，保留基础策略夹爪。
- `--residual-action-scale` 限制残差（每步归一化动作的 0.1）。
- `--critic-only-updates 500`：前 500 次 learner 更新仅训练 critic。
- `--residual-bc-weight 0.1` 在介入行上将残差拉向人工纠正。

遥操作录制不带基础动作，因此这里不通过 `--demo-dataset-paths` 传示范 pkl（learner 会拒绝，除非添加 `--allow-base-less-demos`，将这些行作为零基础上的残差学习，见 [residual.md](../residual.md#human-intervention)）。

<a id="silri"></a>

## silri

```bash
# Learner machine: repository root, project environment active.
python examples/train_real_world.py silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
# NUC: repository root, project environment active.
python -m robot_infra.supervise --startup-s 60 --log runs/silri-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py silri --role actor $COMMON --exp-name silri-pick-and-place
```

首次更新前，SiLRI learner 在示范缓冲区上预训练人工行为模型（`--bc-pretrain-steps`，默认 500）；期间 actor 行继续进入 learner journal，之后再消费。随后随人工介入到来刷新模型（[silri.md](../silri.md#data-flow)）。

<a id="stopping"></a>

## 停止

在安全状态结束回合，learner 仍运行时按 `q` 停止 NUC actor。在 supervisor 终端按 Ctrl-C 也请求清理。清理释放环境/provider，并尝试在 `--sync-drain-timeout`（默认 30 秒）内排空上传。超时将未确认行留在本地 outbox，供下次重启使用。清理停滞时 supervisor 升级信号；最终 kill 断开 libfranka 连接，依赖固件 Reflex 停止。清故障和重启前检查工位及 supervisor 日志。随后停止 learner：正常停止时，它在退出过程中写入 `runs/<exp-name>/checkpoints/final.pt` 和检查点 bundle（模型、优化器、两个回放缓冲区及 journal 位置，经 `bundle_manifest.json` 发布）。`runs/<exp-name>/config.json` 是 eval、deploy 和残差基础 provider 读取的配置。actor 停止前未送达的行保留在 `runs/<exp-name>/actor/` 下的 outbox，具有相同 outbox 路径和身份的 actor 再启动时发送。上述默认路径假设未覆盖 `--run-id` 和 `--actor-outbox-path`。每个并发 actor 需独立 `--actor-id` 和 outbox。

<a id="evaluate"></a>

## 评估

在机器人上执行确定性回合，无 learner、不采集数据；成功需显式成功标签（带 `--reward-from-teleop-events` 的 `s` 或分类器），只有整回合未接管才计为自主成功。先停 actor，不要介入（[06](06-intervening.md#the-sequence)）。在 NUC 的仓库根目录激活项目环境，先定义上述 `ENV_FLAGS` 和匹配基础变量，再运行各 eval 命令：

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

自主回合中保持 GELLO 已交还（`h`），或 SpaceMouse 按钮 0 接管关闭。`s`/`f` 标记结果；`q` 退出。一次接管会使回合变为辅助回合。定期 `checkpoint_<update>.pt` 用同样方式评估。*应看到*每回合的 `[eval] episode=... task_success=... autonomous_success=... return=...`，以及最终 `[eval] ...` 行，含 `autonomous_success_rate`（无接管成功数除以完成回合数）、`task_success_rate`、介入率和 `mean_return`。

<a id="what-to-watch"></a>

## 观察指标

在 learner 机器的仓库根目录激活项目环境，查看 `runs/` 下由 `--log-type` 指定的日志（默认 TensorBoard）：

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

**每个指标的横轴都是 learner 已收到的转移数量**，不是墙钟时间或更新次数。learner 指标在 `--log-freq` 边界记录（默认 1000 转移，50 Hz 下约 20 s）；actor 每回合提交一组 `actor/*` 标量。

| 指标 | 方法 | 观察内容 |
|---|---|---|
| `actor/return`, `actor/terminated`, `actor/task_success` | 全部 | 每回合：`terminated` 表示任意终止步（`s` 或分类器命中）；`task_success` 为存在时的显式成功标签 |
| `actor/autonomous_success_rate`, `actor/task_success_rate`, `actor/episode_intervention_rate` | 全部 | 从 actor 启动起累计：无接管成功数占完成回合比例应上升，介入率应下降 |
| `actor/intervention_rate`, `actor/intervened_steps`, `actor/intervention_count` | 全部 | 随策略改善应下降 |
| `actor/length`, `actor/human_episode_end` | 全部 | 用 Esc 结束的回合 |
| `actor/safety_clips`, `actor/safety_clips_total` | 全部 | daemon 裁剪的伺服命令；计数增长表示策略持续请求超限目标 |
| `actor/rtt_ms_mean/max`, `actor/step_time_ms_mean/max`, `actor/param_refresh_ms_mean/max` | 全部 | 时序 |
| `actor/outbox_depth`, `actor/upload_retries` | 全部 | actor 磁盘待发送行及上传失败次数；深度增长表示上传待处理；重试保留 ID，容量满或磁盘失败会停止采集 |
| `learner/received`, `learner/update`, `learner/buffer_size`, `learner/train_time_s` | 全部 | 接收与训练进度 |
| `learner/demo_buffer_size`, `learner/intervened_received` | 全部 | 示范缓冲区从人工监督、非 syncing 行增长 |
| `losses/critic_loss`, `losses/qf_loss`, `losses/actor_loss`, `losses/alpha_loss`, `entropy/alpha`, `entropy/target_entropy`, `q/predicted` | `hil_serl`, `residual_hil_serl` | SAC 风格损失；`q/predicted` 应跟踪回报 |
| `losses/actor/residual_bc`, `losses/actor/intervened_fraction` | `residual_hil_serl` | BC 项及批次中介入行占比（仅 critic 更新阶段之后） |
| `losses/critic/loss`, `losses/actor/loss`, `losses/actor/q`, `losses/actor/distance`, `losses/lambda/loss`, `losses/lambda/mean`, `losses/behavior/sigma_sum`, `losses/batch/intervened_fraction`, `losses/discrete/ce` | `silri` | `losses/actor/distance` 与 `losses/behavior/sigma_sum` + 裕量的关系：actor 在何处被约束到人工行为 |
| `behavior/pretrained`, `behavior/pretrain/*`, `behavior/refresh/*`, `behavior/refresh_count` | `silri` | 行为模型预训练和刷新 |

磁盘中还有 actor 的 `outbox.sqlite3`、learner 的 `checkpoints/journal.sqlite3`、指向一代完整 `bundle_<update>_<id>/` 的 `bundle_manifest.json`，以及旧式 `checkpoint_<update>.pt` 文件。bundle 包含模型/优化器、两个回放缓冲区和 journal 游标。旧式 `buffer/` 和 `demo_buffer/` pkl 快照仍作为回退来源；已有权威 bundle 时不会再叠加加载它们。

<a id="optional-a-reward-classifier"></a>

## 可选：奖励分类器

主要预设无 RGB 相机，不能使用腕部图像分类器。若单独做图像实验，先配置相机和图像观测，再录制示范、训练基础策略并运行 HIL。所有阶段保持图像契约一致。帧采集和分类器训练见[真实机器人实验](../real-world-experiments.md)和 [teleop.md](../teleop.md#recording-demos-on-the-real-robot)，包括终端结果标签。实体数据上的分类器采集仍待验证。按键和分类器同时存在时取较大奖励。

<a id="if-not"></a>

## 失败排查

- `Unknown env backend 'real_robot'`：在运行 NUC actor 的解释器中安装本包。
- 缺少 leader 配置或读取失败：检查 `gello_leader.yaml`、已标定的本地仅 leader YAML 及串口权限。遥操作读取失败会停止 actor，不会静默撤销介入。
- `WORKSPACE HOLD`：按 [06](06-intervening.md#how-the-robot-follows-you) 的恢复顺序操作。
- 差异指出 `experiment.teleop_control`：恢复录制时的设备、tracker 版本、领先上限和目标速度。也检查指出的其它动作/观测字段。
- supervisor 在启动完成前升级信号：检查日志与启动宽限；已运行循环停止发送心跳是另一类故障。
- `--base-provider is required`：每个残差角色都需要基础规格，包括 learner。

<a id="resume-an-interrupted-run"></a>

## 恢复中断实验

将 learner 检查点目录、journal（包括 SQLite WAL/SHM 文件）及 bundle 各代一起保留。先用相同预设、实验名、种子示范路径和容量重启 learner。对于上述 `hil_serl` 运行：

```bash
# Learner machine: repository root, project environment active.
python examples/train_real_world.py hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl --load-checkpoint \
    runs/hil-pick-and-place/checkpoints/bundle_manifest.json --checkpoint-dir \
    runs/hil-pick-and-place/checkpoints
# NUC: repository root, project environment active.
python -m robot_infra.supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    python examples/train_real_world.py hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

actor 复用现有 outbox；已确认 ID 只接纳一次。learner/网络故障时，只要 outbox 还有容量，就退避重试。残差或 SiLRI 使用原方法参数及匹配 manifest/目录。仅 manifest 不会迁移 journal。bundle 恢复始终恢复完整回放/游标单元，不受 `--load-replay-buffer` 影响；单代独立 `agent.pt` 是新运行仅加载权重的入口。独立实验使用新 `--exp-name`/`--run-id` 或独立 outbox。

SQLite 使用 WAL 和 `synchronous=NORMAL`：这些保证覆盖进程崩溃，操作系统崩溃或断电则可能回滚最新提交。journal 应用期间的更新异常或中断保留最后完整 bundle，因此恢复会重放 journal 尾部。完整路径、FWBW 方向隔离及旧式恢复见[真实机器人实验](../real-world-experiments.md)。

下一步：[08 部署](08-deploy.md)。
