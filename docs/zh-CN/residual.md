<a id="residual-rl-with-human-intervention-residual_hil_serl"></a>

# 带人工介入的残差 RL（`residual_hil_serl`）

[中文入口](../README.zh-CN.md) · [English](../residual.md)

`residual_hil_serl` 在真机上基于冻结的基础策略（BC、较早的 RL checkpoint 或 ACT）学习有界修正，沿用 HIL-SERL 的 actor / learner 分离、示范缓冲区与人工介入。硬件配置、BC 和 ACT 基础策略，以及首次残差训练见教程 [05 基础策略](tutorial/05-base-policy.md) 和 [07 HIL 训练](tutorial/07-hil-training.md#residual_hil_serl-bc-base)；双臂修改见 [bimanual.md](bimanual.md#two-arms-the-tutorial-steps)。

<a id="method"></a>

## 方法

基础策略提出 `a_base`，即 `[-1, 1]` 范围的归一化环境动作。actor 网络将观测特征与 `a_base` 的连续部分拼接，输出 `[-1, 1]` 范围的单位残差 `u`。最终执行动作为：

```
a = clamp(a_base + scale * u, -1, 1)        (per dim)
```

critic 与评价普通环境动作一样评价组合动作 `a`。SAC actor 损失的 Q 项使用 `a`，熵项使用 `u` 的对数密度；组合变换的 `log(scale)` 雅可比项不加入，因此目标熵作用于单位残差。默认 `--target-entropy auto` 解析为 `-A_cont`，即残差 actor 维度的负值：`continuous` 使用完整动作维度，`hybrid` 使用机械臂维度，例如 7 维动作对应 -6；普通 SAC 的目标则是 `-A_env`。

默认开启 `--residual-zero-init`，均值头初始化为零，因此确定性残差恰为 0，初始策略与基础策略相同。对数标准差头的初始值与状态无关，由 `--residual-log-std-init` 决定，默认 `-3.0`，单位残差空间标准差约为 0.05。

learner 无需加载基础策略。actor 为每条 transition 保存 `obs` 和 `next_obs` 的基础动作；回合结束时后者是 reset 前观测。actor 损失以保存的 `base_actions` 为条件，TD 目标使用 `next_base_actions`。

<a id="variants"></a>

## 变体

| `--residual-variant` | 算法 | 残差作用范围 | 夹爪 |
|---|---|---|---|
| `continuous`（默认） | `ResidualRLPD` | 全部动作维度；机械臂缩放由 `--residual-action-scale` 决定，夹爪缩放由 `--residual-gripper-scale` 决定，默认 0。 | 基础夹爪动作加残差，残差默认零。 |
| `hybrid` | `ResidualRLPDHybrid` | 机械臂维度。 | 使用 `RLPDHybrid` 的 DQN 头，忽略基础策略的夹爪输出。 |

两种变体都将最后 `num_grippers` 个动作维度视为夹爪，各臂一个，排列在所有机械臂动作块之后；这遵循环境动作约定，单臂只有一个夹爪。所有 HIL 环境都包含夹爪，介入包装器要求 `gripper_in_action`。`--use-grasp-penalty` 需要 hybrid 变体。两种变体都保留持续增长的示范缓冲区，存入介入 transition 并按 `--demo-data-ratio` 混合。

<a id="base-action-providers"></a>

## 基础动作提供器

`--base-provider` 接受 `kind[:path][?key=value&key=value]` 描述字符串，保存于 `config.json` 和 checkpoint 元数据的 `base_provider`，使 eval 与 deploy 能重建相同提供器。

| 描述 | 提供器 |
|---|---|
| `zero` | `ZeroBaseProvider`：基础动作为 0，相当于动作受缩放限制的普通 RL，用于测试与消融。 |
| `checkpoint:<path>?algo=<name>[&num_grippers=<k>]` | `CheckpointBaseProvider`：使用 rl-garden checkpoint 的确定性动作。`algo` 可为 `real_world_bc` 或在线注册表中具有智能体构建器的名称，如 `rlpd`、`rlpd_hybrid` 等。若存在训练目录 `checkpoints/` 旁的 `config.json`，会一并读取。checkpoint 的观测与动作空间必须匹配环境。`num_grippers` 默认 1，仅用于 `algo=rlpd_hybrid`，表示多臂 checkpoint 的夹爪数量，其离散头覆盖最后 `num_grippers` 个动作维度。双臂 `hil_serl` 基础策略写为 `checkpoint:<path>?algo=rlpd_hybrid&num_grippers=2`；仅凭空间重建时无法推断夹爪数量，`real_world_bc` 基础策略无需该选项。 |
| `act:<path>[?temporal_agg=1&temporal_agg_k=0.01&state_dict_key=ema_agent]` | `ACTBaseProvider`：封装 rl-garden ACT checkpoint，按 checkpoint 统计量反标准化并裁剪到 `[-1, 1]`。读取 `obs["state"]`；回合结束调用 `reset()` 清空动作块和集成状态，默认在每个介入步骤后也会重置，详见下文。 |

选项用 `&` 连接，shell 中需给描述加引号，因为 `?` 会展开通配符、`&` 会转入后台：

```bash
--base-provider 'act:/data/act/policy_best.ckpt?temporal_agg=1&temporal_agg_k=0.02'
```

所有提供器均在无梯度模式下计算，返回 `[-1, 1]` 范围、形状为 `(B, A)` 的动作。仅 actor、eval 和 deploy 构建提供器，learner 无需构建。

`--residual-base-resync-on-intervention` 默认开启，可用 `--no-residual-base-resync-on-intervention` 关闭。它让 actor 在每个介入步骤之后、计算下一观测的基础动作前重置提供器。人工操作使机器人偏离原 ACT 动作块的规划轨迹；缺少重新同步时，开环提供器（`temporal_agg=0`）继续执行旧块直到下个边界，集成提供器（`temporal_agg=1`）继续平均接管前及接管中的预测块。因此保存的 `base_actions` / `next_base_actions`、介入数据上的 BC 目标和交还控制后的基础动作最多滞后一个动作块。重新同步后，介入后每个观测都使用从该观测重新规划的动作块首个动作。开始接管的那个步骤保留接管前计算的基础动作。`zero` 和 `checkpoint` 提供器无状态，`reset()` 不执行操作，因此不受该参数影响。

<a id="human-intervention"></a>

## 人工介入

操作员覆盖的是*总动作*。回放保存以下字段：

- `action`：实际执行动作，介入时为人工动作。即使超出 `base +/- scale` 范围也原样保存，critic 学习其价值。
- `policy_action`：策略提出的动作；同时保存 `base_actions`、`next_base_actions` 和 `intervened`。

与 hil_serl 相同，介入 transition 还会复制到示范缓冲区。

预先采集的示范（`--demo-dataset-paths`）视为人工动作。缺少 `intervened` / `policy_action` 的 transition 会补为 `intervened=True` 和 `policy_action=action`。learner 无法恢复基础动作，因此缺少 `base_actions` / `next_base_actions` 的示范 pkl，例如 hil_serl 快照和遥操作录制，会被拒绝，并抛出指明文件名的 `ValueError`。`--allow-base-less-demos` 会以零基础动作加载这些数据并警告一次，此时 critic 目标与 BC 项会把它们视为零基础上的残差；只有基础策略在这些状态本就接近 0 时才与当前运行匹配。残差运行自身 learner 写出的示范缓冲区包含基础动作。

可选 BC 项由 `--residual-bc-weight` 控制，默认 0。在介入数据上，将单位残差拉向人工动作隐含的残差：

```
L_bc = mean over intervened rows of  sum_d ( u_d - clip((a_human_d - a_base_d) / scale_d, -1, 1) )^2
```

对非零缩放维度求和，批次没有介入数据时会保护分母。在 `--log-freq` 边界，learner 除 SAC 损失（`losses/critic_loss`、`losses/actor_loss`、`entropy/alpha`、`q/predicted` 等）外，还将该项记录为 `losses/actor/residual_bc`，批次介入占比记录为 `losses/actor/intervened_fraction`。两项只在训练 actor 的更新中记录，因此 `--critic-only-updates` 阶段不会出现。actor 每回合的 `actor/intervention_rate` 是机器人执行步骤中的介入比例，见[教程 07](tutorial/07-hil-training.md#what-to-watch)。

<a id="warm-up"></a>

## 预热

- actor：首批 learner 权重到达前策略尚未训练，单位残差采用 `--residual-warmup-unit * U(-1, 1)`，默认 0，直接使用基础动作；夹爪也使用基础策略输出，因为 hybrid DQN 头同样尚未训练。
- learner：`--critic-only-updates N` 使前 `N` 次 `train()` 仅更新 critic，通过 `update_actor=False` 更新掩码控制；之后才更新 actor 和温度参数。计数会保存进 checkpoint。

<a id="commands"></a>

## 命令

完整的 BC / ACT 基础策略下 learner、actor、eval 命令、环境预设和遥操作参数见[教程 07](tutorial/07-hil-training.md#residual_hil_serl-bc-base)，部署见[教程 08](tutorial/08-deploy.md)。GELLO 由机器人守护进程提供、环境采用 `joint_delta` 时，最简形式如下；包括 learner 在内的每个角色都需要 `--base-provider`：

```bash
python examples/train_real_world.py residual_hil_serl --role actor --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard \
    --reward-from-teleop-events --base-provider 'checkpoint:runs/bc/checkpoints/final.pt?algo=real_world_bc'
```

eval（`--role eval --load-checkpoint ...`）在提供器动作上叠加确定性残差，成功由 `terminated` 判定。continuous 变体部署使用 `--algo residual_hil_serl`，hybrid 使用 `residual_hil_serl_hybrid`。两者都根据运行 `config.json` 中的类名重建，分别以算法类名称注册到 rl-garden 在线注册表；真机训练 CLI 统一使用 `residual_hil_serl`。提供器描述来自 checkpoint 元数据，可用 `--base-provider` 替换，例如基础 checkpoint 移动后。

<a id="limitations"></a>

## 限制

- 仅支持 `nstep` 为 1、内存回放、单个环境（一个机器人）；暂不支持 `--mmap-dir`、`--memory-efficient-buffer` 和 `--fwbw`。
- 暂无模拟器 `learn()` 路径，也不支持 `rl-garden-eval`：两者调用 `policy.predict` 时都不提供基础动作，原因记录在 `checkpoint_eval_unsupported_reason`。
- 缺少基础动作的示范 pkl 需要 `--allow-base-less-demos`，数据按零基础动作上的残差学习，见“人工介入”。
- checkpoint 提供器根据基础运行的 Args 重建智能体，基础 checkpoint 与残差运行的观测和动作空间必须相同。
