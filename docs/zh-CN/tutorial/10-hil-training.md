<!-- English source: docs/tutorial/10-hil-training.md -->

<a id="10-hil-training"></a>

# 10. HIL 训练

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/10-hil-training.md)

**目标：** 启动 learner 和受监督机器人 actor，观察学习及介入指标，然后使用相同实验契约停止、评估和恢复。

三种方法共享 actor、learner、eval 角色，遥操作选项、示范 pkl 和同步协议。每次只运行一种，各用独立 `--exp-name`：

| 方法 | 增加的能力 | 概念 |
|---|---|---|
| `hil_serl` | 含人工介入、增长示范缓冲区及 DQN 夹爪头的 RLPD | [README](../project-readme.md#running-on-hardware), [hil-serl-roadmap.md](../hil-serl-roadmap.md) |
| `residual_hil_serl` | 冻结基础策略（[09](09-base-policy.md) 的 BC 或 ACT）上的有界残差 | [residual.md](../residual.md) |
| `silri` | 将 actor 拉向学习得到的人工介入模型 | [silri.md](../silri.md) |

<a id="before-you-start"></a>

## 开始前

- 完成 [08 录制示范](08-record-demos.md)；残差运行还需 [09 基础策略](09-base-policy.md)。阅读[操作员页面](../intervening.md)。
- actor 是 **NUC** 上唯一硬件进程，经 franky 直连。保持工作区畅通，物理停止装置可及，停止其他设备占用者。
- 在 **GPU 主机**（也可为 NUC）先启动 learner。命令使用各机**仓库根目录**和**项目环境**。
- 将完整已调试 `local/gello_cartesian.yaml`、所需主臂选项、示范 pkl/伴随文件复制到两机。learner 也用完整预设，而非 [09](09-base-policy.md) 的离线 BC 副本。基础检查点及元数据保留在下述路径。

actor 与机器人主机不同机时，按[通过机器人守护进程运行远程 actor](../remote-actor.md)配置守护进程、传输和连接。本页命令使用 NUC 直连。

使用已校准 SpaceMouse 站点时，为录制、actor、learner 提供匹配的有效任务坐标系/工作区和配置设置。配置文件路径可因机器不同，但数值变换须一致；仅改配置名称不改变数值契约。手动推进 actor 回合时显式选择 `episode_end_behavior: hold`；它不在介入之间插入中立门控。见 [SpaceMouse CLI 指南](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)及[实物验收检查](../testing/spacemouse-control-session.md)。

<a id="1-set-the-shared-flags"></a>

## 1. 设置共享选项

所有命令在**仓库根目录**、**激活本机项目环境**后运行。在 **learner 和 NUC 两个 shell** 设置下方变量。示例以 `127.0.0.1` 表示同机 learner；独立 GPU learner 须在两个 shell 均改为该机可达 IP。确保两份检出使用相同预设及实验输入。

```bash
LEARNER_IP=127.0.0.1
ENV_FLAGS="--config local/gello_cartesian.yaml --teleop-device leader_cartesian \
    --teleop-event-device stdin --reward-from-teleop-events"
COMMON="$ENV_FLAGS --sync-host $LEARNER_IP --sync-port 6000 --log-dir runs \
    --checkpoint-freq 1000 --buffer-size 20000 --demo-buffer-size 20000"
```

*应看到：* 两个 shell 定义变量（赋值不打印输出）。learner 远程时，两个 `--sync-host` 均须为其可达地址。

在 NUC 使用针对站点的 [08 设置](08-record-demos.md#before-you-start)，包括 `gello_leader.yaml`。预设为纯状态 `cartesian_delta`、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`。保留录制设备及四个 tracker 上限/速度。示范伴随文件的 `experiment.teleop_control` 记录 tracker 版本和值；修改或省略会被拒绝。当前 HEAD 比较 tracker 身份，不比较实际输入设备；教程将同设备作为操作流程规则（[兼容性说明](08-record-demos.md#files-and-compatibility)）。

使用 SpaceMouse 录制时，在两个 shell 的 `ENV_FLAGS` 设置 `--teleop-device spacemouse`、`--teleop-options-yaml None`，使用对应示范路径，保持四个值不变。按钮 0 接管，stdin 提供 `s`/`f`/`r`/Esc/`q`。仅移动控制帽不会接管。

原生 SpaceMouse 介入：在 **NUC 和 learner 两个 shell** 的**仓库根目录**、**激活各机项目环境**后，向 `ENV_FLAGS` 添加 `--spacemouse-mode serl`。使用含 xyz、RPY 安全盒的站点预设及同模式示范。模式解析在训练前保存 `real_robot.cartesian_semantics: serl`，无设备 learner 也一样：原始六轴范数 `> 0.001` 介入，释放控制帽和按钮交还，按住按钮 0 关闭、按钮 1 打开，均未按下发送中立夹爪零。tracker 上限/速度不适用；比较契约为 `spacemouse_serl_v1`。见[两种模式](../teleop.md#spacemouse)；硬件行为未验证。

下方硬件命令均由 `rwrl supervise` 包装。心跳静默阈值默认 **2.0 s**。`--startup-s 60` 为首次 reset 心跳前策略加载及 actor 等待快照留时间。确认它超过本机启动工作时长；默认启动宽限为 30 s。当前预设**关闭环境 StepWatchdog**（无 `real_robot.watchdog_s`），与监督器独立。[08](08-record-demos.md) 录制显式开启 0.3 秒 StepWatchdog。当前[实验指南](../real-world-experiments.md#step-watchdog-in-process)未指定 actor StepWatchdog 值。

- `--reward-from-teleop-events`：`s` 提供奖励（1，终止），`f` 以失败结束回合。训练默认不开启；未开启且无 `--classifier-checkpoint` 时，所有 transition 奖励为 0，actor 和 learner 会警告（[操作员页面](../intervening.md#where-the-reward-comes-from)）。
- `--checkpoint-freq`（默认 0：仅最终保存）每指定 learner 更新次数写入 `checkpoint_<update>.pt` 和检查点包。
- `--buffer-size` 为回放容量（`hil_serl`、`residual_hil_serl` 默认 200000，`silri` 默认 30000），存于 `--buffer-device cuda`；多台 128 x 128 相机时按显存调整或用 `--buffer-device cpu`。每个检查点包将两个回放缓冲区按完整容量复制到主机内存和磁盘。按 GPU/主机内存及磁盘调整 `--buffer-size`、`--demo-buffer-size`；50 Hz 下 20000 行约 6 分 40 秒。每次定期保存创建新一代。将 manifest 引用代与 journal 一起保留。
- `--teleop-device` 无默认值。actor 和 eval 必须显式或在预设中选择源。learner 不打开遥操作设备，但仍用 tracker 设置校验实验契约。

<a id="2-run-hilserl"></a>

<a id="2-run-hil_serl"></a>

## 2. 运行 hil_serl

在 GPU 主机的仓库根目录，激活项目环境，启动 learner：

```bash
rwrl train hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
```

*应看到：* learner 启动并等待 actor 数据。若退出，先检查指定路径或契约差异，再启动 actor。

在 NUC 的仓库根目录，激活项目环境，启动 actor：

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    rwrl train hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

*应看到：* actor 接收策略快照，reset 并开始回合。启动失败时，再次 reset 前先按[失败怎么办](#if-it-fails)处理。

actor 在 reset/step 前等待初始 learner 快照，最多 `--sync-startup-timeout`（默认 30 秒）。快照仍可能含未训练策略；笛卡尔遥操作不会使自主动作变安全。随时准备 `t` 接管，必要时使用物理停止。learner 预载示范，等待 `--learning-starts` 在线 transition（默认 4000；预载示范不计），再训练。发布首个就绪策略，此后每 `--publish-freq` 更新（默认 100）发布快照。快照 session/version 标识跨 learner 重启的更新。学习期间准备介入。

*应看到：* actor 数据到达时 `learner/received` 和缓冲区大小增长；开始学习后出现 `learner/update` 及 SAC 损失。每回合提供 `actor/task_success` 和介入指标（[下表](#8-watch-the-metrics)）。摄取停滞时检查同步地址及 actor outbox；硬件失败按[失败怎么办](#if-it-fails)处理。

<a id="residual_hil_serl-bc-base"></a>

<a id="3-alternative-residualhilserl-bc-base"></a>

<a id="3-alternative-residual_hil_serl-bc-base"></a>

## 3. 替代：residual_hil_serl，BC 基础策略

在两机 shell 激活项目环境并设置 `RESIDUAL`、`BASE_BC`；分别从仓库根目录先在 learner 机器运行 learner，再在 NUC 运行受监督 actor。

在两个 shell 的仓库根目录，激活项目环境，定义：

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
```

*应看到：* 无输出；变量可供下方命令使用。

在两个 shell 的仓库根目录，激活项目环境，定义：

```bash
BASE_BC='checkpoint:runs/bc-pick-and-place/checkpoints/final.pt?algo=real_world_bc'
```

*应看到：* 无输出；变量可供下方命令使用。

在 GPU 主机的仓库根目录，激活项目环境，启动 learner：

```bash
rwrl train residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

*应看到：* learner 启动并等待 actor 数据。若退出，先检查指定路径或契约差异，再启动 actor。

在 NUC 的仓库根目录，激活项目环境，启动 actor：

```bash
rwrl supervise --startup-s 60 --log runs/residual-bc-pick-and-place/supervise.jsonl -- \
    rwrl train residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC"
```

*应看到：* actor 接收策略快照，reset 并开始回合。启动失败时，再次 reset 前先按[失败怎么办](#if-it-fails)处理。

*应看到：* 摄取及 actor 回合指标，之后仅 critic 更新结束后出现 `losses/actor/residual_bc`、`losses/actor/intervened_fraction`。基础加载或契约校验失败时检查 [09 元数据](09-base-policy.md#keep-the-experiment-contract-with-the-base)及[失败怎么办](#if-it-fails)。

<a id="4-alternative-residualhilserl-act-base"></a>

<a id="4-alternative-residual_hil_serl-act-base"></a>

## 4. 替代：residual_hil_serl，ACT 基础策略

在两机 shell 激活项目环境并设置 `RESIDUAL`、`BASE_ACT`；分别从仓库根目录先在 learner 机器运行 learner，再在 NUC 运行受监督 actor。

在两个 shell 的仓库根目录，激活项目环境，定义：

```bash
RESIDUAL="--residual-variant hybrid --residual-action-scale 0.1 --residual-bc-weight 0.1 \
    --critic-only-updates 500"
```

*应看到：* 无输出；变量可供下方命令使用。

在两个 shell 的仓库根目录，激活项目环境，定义：

```bash
BASE_ACT='act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
```

*应看到：* 无输出；变量可供下方命令使用。

在 GPU 主机的仓库根目录，激活项目环境，启动 learner：

```bash
rwrl train residual_hil_serl --role learner $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

*应看到：* learner 启动并等待 actor 数据。若退出，先检查指定路径或契约差异，再启动 actor。

在 NUC 的仓库根目录，激活项目环境，启动 actor：

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/supervise.jsonl -- \
    rwrl train residual_hil_serl --role actor $COMMON $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT"
```

*应看到：* actor 接收策略快照，reset 并开始回合。启动失败时，再次 reset 前先按[失败怎么办](#if-it-fails)处理。

*应看到：* 与 BC 相同的残差指标，包括仅 critic 更新后的 `losses/actor/residual_bc`。ACT 无法加载时先检查保存的观测/动作宽度及基础路径，再看[失败怎么办](#if-it-fails)。

actor 执行 `clamp(base + scale * residual, -1, 1)`；人工介入默认覆盖总动作；选定部件接管使用[操作员页面](../intervening.md#take-over-selected-parts)描述的掩码。基础策略必须来自同一笛卡尔预设及 tracker 契约（同观测/动作空间，[09 契约](09-base-policy.md#the-act-checkpoint-contract)）。基础规格用引号包裹：`?` 是 glob 字符，`&` 会后台执行。仅 actor 和 eval 构建基础策略；learner 使用 actor 存储的基础动作训练。ACT 基础时，actor 每个介入步后重新规划块（[重新同步](09-base-policy.md#resync-after-an-intervention)）。

- `--residual-variant hybrid` 将残差用于机械臂维度，夹爪使用 `hil_serl` 的 DQN 头；`continuous` 表示全部维度，默认夹爪残差尺度为零，保留基础夹爪。
- `--residual-action-scale` 限制残差（每步归一化动作的 0.1）。
- `--critic-only-updates 500`：前 500 次 learner 更新仅训练 critic。
- `--residual-bc-weight 0.1` 在介入行将残差拉向人工修正。

遥操作录制不含基础动作，因此此处不传 `--demo-dataset-paths` 示范 pkl（learner 会拒绝，除非添加 `--allow-base-less-demos`，将这些行学为零基础上的残差；见 [residual.md](../residual.md#human-intervention)）。

<a id="silri"></a>

<a id="5-alternative-silri"></a>

## 5. 替代：silri

在 GPU 主机的仓库根目录，激活项目环境，启动 learner：

```bash
rwrl train silri --role learner $COMMON --exp-name silri-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl
```

*应看到：* learner 启动并等待 actor 数据。若退出，先检查指定路径或契约差异，再启动 actor。

在 NUC 的仓库根目录，激活项目环境，启动 actor：

```bash
rwrl supervise --startup-s 60 --log runs/silri-pick-and-place/supervise.jsonl -- \
    rwrl train silri --role actor $COMMON --exp-name silri-pick-and-place
```

*应看到：* actor 接收策略快照，reset 并开始回合。启动失败时，再次 reset 前先按[失败怎么办](#if-it-fails)处理。

SiLRI learner 在首次更新前用示范缓冲区预训练人工行为模型（`--bc-pretrain-steps`，默认 500）；期间 actor 行继续进入 learner journal，随后消费。之后随人工介入到达刷新模型（[silri.md](../silri.md#data-flow)）。

*应看到：* `behavior/pretrained`、`behavior/pretrain/*` 及后续 `behavior/refresh/*`，同时有下表摄取、回合、SiLRI 损失指标。缺失时先检查示范加载及接收行，再看[失败怎么办](#if-it-fails)。

<a id="6-stop-and-save"></a>

## 6. 停止并保存

安全结束回合后，保持 learner 运行，按 `q` 停止 NUC actor。监督器终端 Ctrl-C 也请求清理。清理释放环境/提供器，并在 `--sync-drain-timeout`（默认 30 秒）内尝试排空上传。超时将未确认行留在本地 outbox，供下次重启。清理卡住时监督器升级信号；最终强制终止打断 libfranka 连接，依赖固件 Reflex 停止。清除故障、重启前检查工位和监督器日志。再停止 learner：正常退出写入 `runs/<exp-name>/checkpoints/final.pt` 和检查点包（模型、优化器、两回放缓冲区及 journal 位置，经 `bundle_manifest.json` 发布）。`runs/<exp-name>/config.json` 供 eval、deploy、残差基础提供器读取。actor 未能送达的行留在 `runs/<exp-name>/actor/` 的 outbox；以同路径和身份重启 actor 后发送。上述默认路径假定未覆盖 `--run-id`、`--actor-outbox-path`。每个并发 actor 需独立 `--actor-id` 和 outbox。

*应看到：* actor 退出，learner 正常停止后有 `final.pt`、`config.json`、完整 bundle manifest。清理超时时保留 outbox、journal，按[恢复](#9-resume-an-interrupted-run)处理。

<a id="evaluate"></a>

<a id="7-evaluate"></a>

## 7. 评估

机器人上确定性回合，无 learner、不采集数据；成功须显式标签（`--reward-from-teleop-events` 的 `s` 或分类器），且全回合未接管才算自主成功。先停止 actor，不要介入（[操作员页面](../intervening.md#the-sequence)）。在 NUC 仓库根目录、激活项目环境并定义上述 `ENV_FLAGS` 及对应基础变量后，选择对应运行命令：

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/eval-supervise.jsonl -- \
    rwrl train hil_serl --role eval $ENV_FLAGS --exp-name hil-pick-and-place \
    --load-checkpoint runs/hil-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl supervise --startup-s 60 --log runs/residual-bc-pick-and-place/eval-supervise.jsonl -- \
    rwrl train residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-bc-pick-and-place --base-provider "$BASE_BC" \
    --load-checkpoint runs/residual-bc-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/eval-supervise.jsonl -- \
    rwrl train residual_hil_serl --role eval $ENV_FLAGS $RESIDUAL \
    --exp-name residual-act-pick-and-place --base-provider "$BASE_ACT" \
    --load-checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl supervise --startup-s 60 --log runs/silri-pick-and-place/eval-supervise.jsonl -- \
    rwrl train silri --role eval $ENV_FLAGS --exp-name silri-pick-and-place \
    --load-checkpoint runs/silri-pick-and-place/checkpoints/final.pt --eval-n-trajs 10
```

自主回合保持 GELLO 已交还（`h`），或 SpaceMouse 按钮 0 接管关闭。`s`/`f` 标结果，`q` 退出。接管使回合变为辅助。定期 `checkpoint_<update>.pt` 评估相同。*应看到* 每回合 `[eval] episode=... task_success=... autonomous_success=... return=...`，最终 `[eval] ...` 含 `autonomous_success_rate`（无接管成功数除以完成回合数）、`task_success_rate`、介入率和 `mean_return`。

<a id="what-to-watch"></a>

<a id="8-watch-the-metrics"></a>

## 8. 观察指标

在 learner 机器仓库根目录、激活项目环境后，查看 `runs/` 下 `--log-type`（默认 TensorBoard）日志：

<!-- tutorial-test: skip -->
```bash
tensorboard --logdir runs
```

*应看到：* 随行到达和更新运行出现下表指标。无事件时检查运行目录及 `--log-type`；设备错误见[失败怎么办](#if-it-fails)。

**所有指标横轴都是 learner 已接收 transition 数**，不是时间或更新数。learner 指标按 `--log-freq` 边界记录（默认 1000 transition，50 Hz 约 20 s）；actor 每回合提交一组 `actor/*` 标量。

| 指标 | 方法 | 关注内容 |
|---|---|---|
| `actor/return`, `actor/terminated`, `actor/task_success` | 全部 | 每回合：`terminated` 为任何终止步（`s` 或分类器命中）；`task_success` 为存在时的显式成功标签 |
| `actor/autonomous_success_rate`, `actor/task_success_rate`, `actor/episode_intervention_rate` | 全部 | actor 启动后累计：无接管成功占完成回合比例应上升，介入率应下降 |
| `actor/intervention_rate`, `actor/intervened_steps`, `actor/intervention_count` | 全部 | 随策略改善应下降 |
| `actor/length`, `actor/human_episode_end` | 全部 | 用 Esc 结束的回合 |
| `actor/safety_clips`, `actor/safety_clips_total` | 全部 | 守护进程裁剪的伺服命令；增长意味着策略持续请求越界目标 |
| `actor/rtt_ms_mean/max`, `actor/step_time_ms_mean/max`, `actor/param_refresh_ms_mean/max` | 全部 | 时序 |
| `actor/outbox_depth`, `actor/upload_retries` | 全部 | actor 磁盘待上传行及失败尝试；深度增长表示待上传；重试保留 ID，容量满或磁盘失败则停止采集 |
| `learner/received`, `learner/update`, `learner/buffer_size`, `learner/train_time_s` | 全部 | 摄取和训练进度 |
| `learner/demo_buffer_size`, `learner/intervened_received` | 全部 | 示范缓冲区由人工监督、非同步行增长 |
| `losses/critic_loss`, `losses/qf_loss`, `losses/actor_loss`, `losses/alpha_loss`, `entropy/alpha`, `entropy/target_entropy`, `q/predicted` | `hil_serl`, `residual_hil_serl` | SAC 风格损失；`q/predicted` 应跟随回报 |
| `losses/actor/residual_bc`, `losses/actor/intervened_fraction` | `residual_hil_serl` | BC 项及批次介入行比例（仅 critic 更新后） |
| `losses/critic/loss`, `losses/actor/loss`, `losses/actor/q`, `losses/actor/distance`, `losses/lambda/loss`, `losses/lambda/mean`, `losses/behavior/sigma_sum`, `losses/batch/intervened_fraction`, `losses/discrete/ce` | `silri` | 比较 `losses/actor/distance` 与 `losses/behavior/sigma_sum` + margin：actor 被约束于人工行为的位置 |
| `behavior/pretrained`, `behavior/pretrain/*`, `behavior/refresh/*`, `behavior/refresh_count` | `silri` | 行为模型预训练与刷新 |

磁盘还有 actor 的 `outbox.sqlite3`、learner 的 `checkpoints/journal.sqlite3`、指向完整 `bundle_<update>_<id>/` 代的 `bundle_manifest.json` 及旧式 `checkpoint_<update>.pt`。bundle 包含模型/优化器、两缓冲区及 journal 游标。旧式 `buffer/`、`demo_buffer/` pkl 快照作为回退源，不会在权威 bundle 上再次叠加加载。

<a id="optional-a-reward-classifier"></a>

## 可选：奖励分类器

主预设无 RGB 相机，无法用腕部图像分类器。独立图像实验应在录制示范、训练基础策略及 HIL 前配置相机与图像观测，全阶段保持图像契约一致。帧采集与分类器训练见[真实世界实验](../real-world-experiments.md)及 [teleop.md](../teleop.md#recording-demos-on-the-real-robot)，含终端结果标签。真实数据分类器采集尚待验证。按键和分类器同时使用时取较大奖励。

<a id="9-resume-an-interrupted-run"></a>

## 9. 恢复中断运行

将 learner 检查点目录、journal（含 SQLite WAL/SHM）及 bundle 各代一起保留。用相同预设、实验名、种子示范路径和容量先重启 learner。上述 `hil_serl` 运行：

在 GPU 主机的仓库根目录，激活项目环境，启动 learner：

```bash
rwrl train hil_serl --role learner $COMMON --exp-name hil-pick-and-place \
    --demo-dataset-paths demos/pick-and-place.pkl --load-checkpoint \
    runs/hil-pick-and-place/checkpoints/bundle_manifest.json --checkpoint-dir \
    runs/hil-pick-and-place/checkpoints
```

*应看到：* learner 启动并等待 actor 数据。若退出，先检查指定路径或契约差异，再启动 actor。

在 NUC 的仓库根目录，激活项目环境，启动 actor：

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/supervise.jsonl -- \
    rwrl train hil_serl --role actor $COMMON --exp-name hil-pick-and-place
```

*应看到：* actor 接收策略快照，reset 并开始回合。启动失败时，再次 reset 前先按[失败怎么办](#if-it-fails)处理。

actor 复用现有 outbox，已确认 ID 仅接纳一次。learner/网络故障时，在 outbox 有容量期间退避重试。残差或 SiLRI 使用原始参数及对应 manifest/目录。仅 manifest 不会迁移 journal。bundle 恢复总是恢复完整回放/游标单元，与 `--load-replay-buffer` 无关；独立代的 `agent.pt` 是新运行的仅权重入口。独立实验使用新 `--exp-name`/`--run-id` 或独立 outbox。

*应看到：* 恢复的 learner 摄取 actor 待处理 outbox 行并恢复更新。恢复失败时将完整 bundle 和 journal 保持在一起；不要合并独立运行。

SQLite 使用 WAL 和 `synchronous=NORMAL`：保证覆盖进程崩溃，系统崩溃或断电可能回滚最近提交。更新异常或 journal 应用中断保留最后完整 bundle，恢复时重放 journal 尾部。完整路径、FWBW 方向隔离及旧式恢复见[真实世界实验](../real-world-experiments.md)。

<a id="appendix-the-env-preset-and-observation-settings"></a>

## 附录：环境预设与观测设置

这些设置用于构建本地直连预设，定义任务/图像契约。录制前配置，各阶段保持一致。

<a id="the-env-preset-localgellocartesianyaml"></a>

<a id="the-env-preset-localgello_cartesianyaml"></a>

### 环境预设 (`local/gello_cartesian.yaml`)

使用 [04](04-first-motion.md#1-home)、[08](08-record-demos.md#1-prepare-the-local-preset-and-home) 从 `examples/presets/gello_cartesian.yaml` 创建的副本。内嵌 `arm_backend: franky` 的 `real_robot.robot`；`robot`、`remote` 必须且只能设一个。此直连路径不要启动守护进程。

- `robot` 是映射，不是 `station.yaml` 路径。复制站点单臂机器人映射，排除 `daemon` 块，同步硬件字段。
- 使用 `real_robot.action_mode: cartesian_delta`、`control_hz: 50`，`state_fields` 包含 `tcp_pose`。原始 GELLO 关节需 `leader_cartesian` 源驱动此模式。
- 纯状态教程保留 `obs.rgb: []`。图像实验应在新示范录制前配置相机名（如 `[wrist]`）和 `obs.image_size: [128, 128]`。
- 有 Hand 时用 `gripper_in_action: true`；仅当每阶段观测均需要时才添加 `gripper_pos`。无 Hand 用 `robot.gripper: null`、`gripper_in_action: false` 并省略 `gripper_pos`。

在任一主机仓库根目录，激活项目环境，不打开硬件检查：

```bash
rwrl train hil_serl --config local/gello_cartesian.yaml --teleop-device leader_cartesian --print-config
```

*应看到：* 解析配置含 `env_backend: real_robot`、`real_robot.robot` 映射、无远程地址、`action_mode: cartesian_delta`、`control_hz: 50`。否则检查 YAML 缩进和复制设置；解析配置不验证硬件。

在 NUC 仓库根目录，激活项目环境，无录制器或 actor 运行：

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml doctor
```

*应看到：* 复制硬件字段匹配时 `arm.preset` PASS。不同字段显示 `arm.preset:<field>` WARN 及两个值。阅读 WARN，修正副本再试。红色行见[失败怎么办](#if-it-fails)。

<a id="task-images-and-control-profiles"></a>

### 任务、图像与控制配置

教程预设将 `real_robot.task.name` 保持为 `none`：操作员或分类器提供成功标签。任务/reset、深度、逐相机预处理选项见[真实世界实验](../real-world-experiments.md#task-presets-and-observation-settings)。录制前设置，在 BC、HIL、部署中保持。`preprocess.<camera>.roi` 为像素 `[top, left, height, width]`，需实际 `source_resolution: [height, width]`。RGB 双线性缩放；深度为单通道米，最近邻缩放，无效值编码为零。修改这些设置会改变策略观测契约。

使用 `rwrl camera show`、`rwrl camera grab` 从真实帧读取实际分辨率并选 ROI（[04 Home、夹爪、相机](04-first-motion.md#3-cameras)）。

站点 YAML 支持可选逐臂 `control:` 阻抗配置；`real_robot.control` 可为实验覆盖。IDLE 时配置。覆盖只能收紧声明的扭矩变化率、误差裁剪边界，已安装 SDK 必须支持请求关键词。字段、有效设置及向导 `--control` 见[机械臂控制配置](../control-profiles.md)。向导保留现有配置，新站点用 `control: null`。根据实际工具和负载选增益。

使用 `real_robot.control_hz` 指定机器人频率。非默认全局 `--control-hz` 可填补仍为默认的机器人频率，但全局与环境非默认频率冲突会拒绝。录制、训练、部署按解析后的频率运行；它属于实验契约。

<a id="if-it-fails"></a>

## 失败怎么办

在 NUC 仓库根目录，激活项目环境，先结束 actor：

```bash
rwrl --station local/station.yaml doctor
```

从首个红色行和提示开始。见[网络与 FCI](../doctor.md#network-and-fci)、[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)、[SpaceMouse 与 HID](../doctor.md#spacemouse-and-hid)、[Python 环境与扩展](../doctor.md#python-environment-and-extras)或[设备占用](../doctor.md#busy-devices-blocked)。

- `Unknown env backend 'real_robot'`：在运行 NUC actor 的解释器中安装本包。
- 缺少主臂配置或读取失败：检查 `gello_leader.yaml`、已校准本地主臂专用 YAML 及串口权限。遥操作读取失败会停止 actor，不会静默移除介入。
- `WORKSPACE HOLD`：按[操作员页面](../intervening.md#how-the-robot-follows-you)恢复顺序处理。
- 差异指向 `experiment.teleop_control`：恢复录制时的设备、tracker 版本、领先上限及目标速度，同时检查其他指定动作/观测字段。
- 启动完成前监督器升级信号：检查日志和启动宽限；运行中的循环停止心跳属于另一故障。
- `--base-provider is required`：每个残差角色均需基础规格，含 learner。

下一步：[11 部署](11-deploy.md)。
