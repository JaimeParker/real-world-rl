<a id="hil-rl-silri"></a>

# HIL-RL（SiLRI）

[中文入口](../README.zh-CN.md) · [English](../silri.md)

`silri` 从可能次优的人工介入中训练真机策略，依据论文《Real-world Reinforcement Learning from Suboptimal Interventions》（HIL-RL，arXiv 2512.24288）及其参考实现 github.com/nuomizai/HIL-RL 的 `silri` 策略。它采用离策略 actor-critic：actor 被拉向学习得到的人工动作模型，约束强度取决于人工行为一致性。随状态变化的拉格朗日乘子施加距离约束，容许范围随人工模型分散程度增大。硬件配置与首次运行见[教程 10](tutorial/10-hil-training.md#silri)，双臂修改见 [bimanual.md](bimanual.md#two-arms-the-tutorial-steps)。

代码：`real_world_rl/policies/silri_policy.py`（`SiLRIPolicy`）、`real_world_rl/algorithms/silri.py`（`SiLRI`）、`real_world_rl/loops/silri.py`（`SilriActorLoop`、`SilriLearnerLoop`）、`real_world_rl/training/entrypoints/silri.py`（`SilriArgs`、`build_silri`、`run_silri`）。

<a id="method"></a>

## 方法

网络定义于 `SiLRIPolicy`，基于 rl-garden 的 `DDPGPolicy`：

| 模块 | 结构 | 更新方式 |
|---|---|---|
| actor `mu_pi(s)` | 独立编码器、MLP、连续动作维度上的 `tanh` 均值。 | `actor_optimizer` |
| `actor_target` | actor 头的 Polyak 副本，读取 actor 编码器。 | Polyak，`tau`。 |
| critic `Q_1, Q_2` | 独立编码器、连续维度上的双 Q 头。 | `q_optimizer` |
| `critic_target` | Q 头的 Polyak 副本，读取 critic 编码器。 | Polyak，`tau`。 |
| `behavior_model`（beta） | 独立编码器、MLP，`mu_b = tanh(.)`，`sigma_b = clamp(exp(.), std_min, std_max)`。 | `behavior_optimizer` |
| `lambda_net` | 独立编码器、输出层零初始化的 MLP，经 `softplus` 输出，初始为 `log 2`。 | `lambda_optimizer` |
| `discrete_head`（`discrete_gripper`） | 读取 actor 特征的 MLP，输出打开、保持、闭合三类 logits，编码为 -1/0/+1。 | `discrete_optimizer` |

执行动作：`a = clamp(mu_pi(s) + N(0, actor_noise_std^2), -1 + 1e-6, 1 - 1e-6)`；`--deterministic-actor` 关闭噪声。夹爪使用 `discrete_head` 的 argmax，附加在动作末尾。

实现中的损失如下：`a_cont` 是移除夹爪维度后的记录动作，`done` 仅等于 `terminated`，因此超时仍进行 bootstrap：

```
a'     = clamp(mu_target(s') + N(0, actor_noise_std^2), -1, 1)
y      = r + (1 - done) * gamma * min_k Q_target_k(s', a')
L_Q    = sum_k MSE(Q_k(s, a_cont), y)
d(s)   = || mu_pi(s) - mu_b(s) ||_2                          (unsquared; beta and lambda held fixed)
L_pi   = mean_s [ (-min_k Q_k(s, mu_pi(s)) + lambda(s) * d(s)) / (1 + lambda(s)) ]
L_lam  = -mean_s [ lambda(s) * (d(s) - sum_j sigma_b,j(s) - constraint_margin) ]
L_beta = -mean_{intervened} log N(a_cont; mu_b(s), diag(sigma_b(s)^2))
L_g    =  mean_{intervened} CE(gripper logits(s), gripper label)
L_bc   = -mean_{intervened} log N(a_cont; mu_pi(s), actor_noise_std^2 I)          (pretraining only)
```

对 `L_lam` 做梯度下降。当 actor 与人工均值的距离超过 `sum_j sigma_b,j(s) + constraint_margin` 时提高 `lambda(s)`，其他位置降低。`L_pi` 和 `L_g` 一起反向传播，actor 与夹爪优化器在同一次 backward 后更新；与参考实现一致，仅裁剪 actor 梯度。

`SiLRI.train(n)` 的调度：对 `n` 个梯度步骤中的每一步，先在新批次上做 `utd - 1` 次 critic 更新，每次之后对两个目标网络做 Polyak 更新；再取一个新批次，更新 critic、actor 和夹爪、乘子，随后更新两个目标网络。每批 `batch_size` 条，其中 `demo_data_ratio` 来自示范缓冲区。`global_update` 每个梯度步骤递增一次。`train` 本身不训练行为模型。

<a id="data-flow"></a>

## 数据流

```
actor (robot PC)                                    learner (GPU box)
SilriActorLoop                                      SilriLearnerLoop
  policy.predict + teleop wrapper  -- transitions -->  every row -> online buffer (buffer_size)
  holds each transition one step;                     intervened rows (+ onset rows with
  tags intervention_onset on the                      --onset-to-demo-buffer) -> demo buffer
  row before a takeover                               (demo_buffer_size)
                                   <-- policy params -- publish every publish_freq iterations
```

- 推送字段：`obs`、`next_obs`（显式提供，回合结束时为 reset 前观测）、`action`（实际执行动作，介入时为人工动作）、`reward`、`done = terminated`、`intervened`、`policy_action`（策略提议）和 `intervention_onset`。
- `intervention_onset` 标记同一回合中介入步骤前的最后一个策略 transition。回合结束时 actor 刷出暂存 transition；actor 停止时仍暂存的记录也会推送，且不标 onset。
- 两个缓冲区均为 `InterventionReplayBuffer`，存储 `intervened`、`policy_actions`、`intervention_onset`。路由到示范缓冲区的 onset 记录保留 `intervened=False`，因此不会训练行为模型或夹爪头。
- 收到 `learning_starts` 条 transition 且示范缓冲区有 `demo_learning_starts` 条记录后开始训练。
- 预训练：learner 进程首次 `train` 前，若 agent 尚未 `pretrained` 且示范缓冲区有数据，先在纯示范 batch 上训练 `bc_pretrain_steps` 步 `L_beta`，再训练同样步数 `L_bc + L_g`，然后执行 `actor_target <- actor`。`pretrained` 标志写入 checkpoint，恢复运行不会重复预训练。
- 行为刷新：示范缓冲区每增加 `behavior_refresh_every` 条数据，在训练线程下一次 `train` 前用纯示范 batch 执行 `behavior_refresh_steps` 步 `L_beta`。预训练重置新增计数。
- 预训练和刷新可能持有训练缓冲区锁。HTTP 接收独立提交到持久 journal，随后训练线程应用排队记录。actor 重试保留消息 ID，丢失 ACK 不会重复接收记录。
- 示范 pkl（`--demo-dataset-paths`）和崩溃恢复快照按 hil_serl 的方式加载。缺少 SiLRI 字段的 pkl 按介入、非 onset 记录加载，策略动作为记录动作。
- 每次 learner 迭代调用 `train(train_freq)`：`utd` 是每个梯度步骤内部的 critic 更新数，与 `LearnerLoop` 默认行为不同，不乘到梯度步骤数上。

<a id="hyperparameters"></a>

## 超参数

| 参数 | 默认值 | 参考实现 Franka 配置 | 含义 |
|---|---|---|---|
| `--batch-size` | 256 | 256，在线与示范各 128 | 每批条数。 |
| `--demo-data-ratio` | 0.5 | 0.5 | 每批示范比例。 |
| `--gamma` | 0.97 | 0.97 | 折扣因子。 |
| `--tau` | 0.005 | 0.005 / 0.005 | critic 与 actor 目标网络 Polyak 系数。 |
| `--utd` | 2 | 2 | 每梯度步骤的 critic 更新次数。 |
| `--actor-lr` / `--critic-lr` | 3e-4 / 3e-4 | 3e-4 / 3e-4 | actor 及夹爪头 / critic 学习率。 |
| `--behavior-lr` | 3e-4 | 3e-4（`actor_lr`） | 行为模型学习率。 |
| `--lambda-lr` | 3e-6 | 3e-6（`0.01 * critic_lr`） | 乘子学习率。 |
| `--grad-clip-norm` | 40.0 | 40.0 | 每次更新的梯度裁剪阈值。 |
| `--actor-noise-std` | 0.05，标准差 | 把 0.05 当方差，标准差为 0.224 | actor 与 TD 目标采样噪声。 |
| `--constraint-margin` | 0.2 | 0.2 | 容许距离偏置。 |
| `--std-min` / `--std-max` | 1e-5 / 2.0 | -5 / 2，方差边界 | 行为模型标准差裁剪范围。 |
| `--bc-pretrain-steps` | 500 | 500 | 预训练步数。 |
| `--behavior-refresh-steps` / `--behavior-refresh-every` | 50 / 50 | 50 / 50 | 刷新长度 / 周期，0 表示禁用。 |
| `--learning-starts` / `--demo-learning-starts` | 10 / 10 | 10 / 10 | 开始训练前的缓冲区条数。 |
| `--buffer-size` / `--demo-buffer-size` | 30000 / 10000 | 30000 / 10000 | 容量。 |
| `--onset-to-demo-buffer` | false | 会路由，见 S4 | 将起始标记数据复制至示范缓冲区。 |
| `--feature-dim` / `--hidden-dim` | 256 / 256 | MLP [256, 256] | actor/critic 主干与隐藏层宽度。 |
| `--behavior-hidden-dims` / `--lambda-hidden-dims` | 256 256 / 256 256 | [256, 256] | MLP 宽度，采用 Linear、LayerNorm、SiLU。 |
| `--discrete-gripper` / `--discrete-hidden-dim` | true / 256 | `num_discrete_actions` | 夹爪头。 |
| `--encoder.*` | plain_conv，随机平移 | 冻结 ResNet10，随机平移 | 图像编码器，每个网络一个。 |

<a id="deviations-from-the-reference"></a>

## 与参考实现的差异

| 编号 | 参考实现 | 本项目 | 原因 |
|---|---|---|---|
| S1 | actor 固定噪声 0.05 传入要求方差的位置，实际标准差为 0.224。 | `actor_noise_std=0.05` 作为标准差，与论文一致。 | 修正参考实现缺陷，遵循论文意图。 |
| S2 | 容许距离对行为模型方差求和：`sum_j v_b,j + 0.2`。 | 使用标准差 `sum_j sigma_b,j + constraint_margin`，margin 为 0.2。 | 与 S1 一致；未平方 L2 距离、求和加 margin 的结构遵循代码，论文使用 `kappa * sigma + c`。 |
| S3 | critic 目标读取在线 critic 编码器；`actor_target` 读取 actor 编码器。 | 相同。rl-garden 的 critic 目标是在在线 critic 编码器上滞后更新的 Q 头，`actor_target` 共享 actor 编码器。 | 遵循 rl-garden 目标网络约定，此处恰与参考实现一致。 |
| S4 | 介入前的 transition 以人工标记进入示范缓冲区，使行为模型与 BC 学习策略动作。 | 标记 `intervention_onset` 并留在在线缓冲区；`--onset-to-demo-buffer` 可复制至示范区，但仍保留策略数据身份。 | 起始标记数据实际来自策略。 |
| S5 | 预训练权重缓存在跨运行共享目录并复用。 | 每个运行独立预训练；恢复时保留 checkpoint 中的 `pretrained` 标记。 | 避免静默复用其他任务或种子的旧缓存。 |
| S6 | 每条数据的 `next_state` 是下一条已保存数据，采用 `optimize_memory`。 | 每条 transition 显式保存 `next_obs`。 | 避免介入片段与截断回合处的错误目标。 |
| S7 | actor 优化器由 `generator + list` 构建，会抛异常。 | 各网络使用普通 Adam；夹爪头以 `actor_lr` 使用独立 Adam，与 actor 在同次反向传播后更新。 | 参考实现无法构建优化器。 |
| S8 | 录制示范把动作与下一观测配对。 | 使用 actor 循环或相同 transition 格式的 pkl，保持正确对齐。 | |

其他差异：

- 乘子损失使用 actor 更新前、计算 actor 损失时的距离；参考实现在 actor 更新后重算。
- 夹爪头预测打开、保持、闭合三类，采用 rl-garden 的混合夹爪映射 -1/0/+1。参考实现为两类；本项目交叉熵在介入数据上求均值，没有介入数据时为零，参考实现此时会得到 NaN。
- 达到刷新条件时每次训练迭代只刷新一次，即使累计新增数据跨越多个 `behavior_refresh_every` 周期。
- MLP 使用 rl-garden 组件：行为模型与乘子使用带 LayerNorm 和 SiLU 的 `create_mlp`；actor、critic 使用 `DDPGPolicy` 主干与 ReLU 头。图像编码器由 `--encoder.*` 配置，采用 rl-garden 实现。
- critic 暂不使用 `torch.compile`。

<a id="cli"></a>

## 命令行

learner、actor、eval 的完整命令、环境预设和遥操作参数见[教程 10](tutorial/10-hil-training.md#silri)；部署见[教程 11](tutorial/11-deploy.md)。`--algo silri` 通过 rl-garden 在线注册表的 `silri` 条目重建智能体。GELLO 由机器人守护进程提供、环境采用 `joint_delta` 时的最简形式如下：

```bash
rwrl train silri --role learner --config hil.yaml --demo-dataset-paths 'demos/*.pkl'
rwrl train silri --role actor --config hil.yaml --teleop-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard --reward-from-teleop-events
```

评估（`--role eval --load-checkpoint ...`）使用确定性动作和显式任务结果。仍可接管；`autonomous_success_rate` 的分子排除动作辅助成功，分母保留全部已完成回合。

<a id="limitations"></a>

## 限制

- 仅支持真机循环：`SiLRI.learn` 会抛错，没有仿真训练路径。
- 拒绝 `--fwbw` 与 `--use-grasp-penalty`，目前没有免 reset 变体或离散 critic。
- learner 发布完整策略状态：actor 也会收到行为模型、乘子和 target，实际只需 actor 与夹爪头；筛选发布留待后续。
- 预训练默认 500 步，与刷新均会推迟回放应用；持久 HTTP journal 接收继续进行。
- 夹爪位于环境动作末尾，每臂一维，遵循动作布局约定。夹爪头在 `3 ** num_grippers` 类上联合分类，标签与 rl-garden 的 `encode_gripper_labels` 一致；单臂有一维，位于最后。


<a id="human-masks-and-recovery"></a>

## 人工掩码与恢复

行为拟合和 actor 预训练仅监督实际人工维度。连续 NLL 对每条受监督记录使用 `D / k * sum(mask * NLL)`，`D` 为完整连续动作宽度，`k` 为观测到的人工宽度。空记录不产生梯度，全掩码恢复原目标。联合夹爪头对已观测人工夹爪使用边缘概率，对未观测夹爪选择求和。在线行为先验距离和乘子约束仍涵盖完整连续动作。同步和空闲保持补充动作均排除在模仿之外。连续无夹爪配置使用 `--no-discrete-gripper`。

持久 outbox 保留 SiLRI 暂存的前驱记录；bundle 恢复两个回放缓冲区、接收游标、预训练状态和刷新计数。恢复的暂存记录会被刷出，不会虚构 onset 后继。路径以及进程崩溃、断电的持久边界见[真机实验](real-world-experiments.md)。
