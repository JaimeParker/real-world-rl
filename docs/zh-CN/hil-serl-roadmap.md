<a id="hil-serl-roadmap"></a>

# HIL-SERL 路线图

[中文入口](../README.zh-CN.md) · [English](../hil-serl-roadmap.md)

本文保留历史移植记录。当前操作请使用[教程](tutorial/00-overview.md)、[真机实验](real-world-experiments.md)和[已知问题](known-issues.md)。当前实现已包含示范录制器、主从臂与双臂支持、持久 outbox/journal、回放 bundle，以及显式的自主和辅助结果。下文条目描述保留迁移时期的行为，当前支持范围请查阅上述文档。

真机 RL 训练层（`rl_garden/real_world/`、`rl_garden/training/real_world/`、`rl_garden/envs/franka_real/`）拆入本仓库时，本页从主 `rl-garden` 仓库的 `docs/roadmaps/` 移至此处。下列路径早于拆分，尚未逐项更新。`robot_infra/...` 指现在本仓库 `robot_infra/` 下的设备层发行包，Python 包名为 `robot_infra`，从仓库根目录看路径是 `robot_infra/robot_infra/...`；它以前是同级 `rlgarden-robot-infra` 仓库。`3rd_party/hil-serl/` 指主 `rl-garden` 仓库保存的参考源码副本，本仓库不包含它。

本页编写后，真机层又增加 `TeleopSource` 注册表、共享 HIL 环境构建器、可重写的 actor 循环步骤、`InterventionReplayBuffer`，以及 `residual_hil_serl` 和 `silri` 方法，见“尚未迁移”末尾的“初次迁移后落地的工作”。下文描述条目当时状态的段落保留原意。

本页追踪初次落地 `hil_serl` 训练方法时尚未迁移的 HIL-SERL（`3rd_party/hil-serl/`）能力及原因，入口见 `rl_garden/training/real_world/hil_serl.py`。HIL-SERL 本身有三个基本独立脚本：`train_rlpd.py` 提供在线 RLPD、示范混合、人工介入和奖励分类器；`train_bc.py` 提供独立 BC；`train_hgdagger.py` 提供 BC 预训练及基于持续增长磁盘数据集的迭代纠正。本轮仅覆盖 `train_rlpd.py` 能力。

第 3、7 项现已落地并在条目中标记，它们是端到端真机人工介入训练的两个阻塞缺口：无法生成奖励分类器 checkpoint，以及长时间在线采集缺少崩溃恢复。第 8、9、10 项随后也已落地，其余在对应历史阶段仍待完成。

后续对唯一权威 HIL-SERL 实现 `3rd_party/hil-serl/examples/train_rlpd.py` 的源码比较，发现刚落地的第 7 项存在一个明确缺陷，并发现三项此前未记录的缺口，见“第 7 项更正”及第 8–10 项；当时第 11 项仍未完成。

第二轮更广泛比较将 `3rd_party/hil-serl/README.md` 的代码结构总览拆成 7 个叶类别：`examples`、`serl_launcher.{agents,data,vision,wrappers}`、`serl_robot_infra.{robot_servers,franka_env}`，逐项与当时 `rl-garden` 比较。该轮复核第 1–11 项，补充第 8 项更正以及第 2、5、9、10、11 项细节，又发现第 12 项正确性缺陷、第 13–20 项能力缺口，以及已实现但默认配置或行为不同的情况，后者单列于“与 HIL-SERL 真机默认配置的行为差异”。

<a id="item-7-correction---offline_dataset-silently-discarded-by-init_demo_buffer"></a>

### 第 7 项更正 `init_demo_buffer` 静默丢弃 `--offline_dataset`

`DemoInterventionMixin.init_demo_buffer()` 位于 `rl_garden/buffers/demo_intervention.py`，曾无条件用空缓冲区替换 `self.offline_replay_buffer`。`_run_learner` 位于 `rl_garden/training/real_world/hil_serl.py`，先调用 `build_rlpd_hybrid(args, ...)`，在指定时将 `args.offline_dataset` 加载至 `offline_replay_buffer`，再调用 `agent.init_demo_buffer(...)`。因此给 `hil_serl` 传 `--offline_dataset` 时，已加载数据会**无提示地丢失**，没有“不能同时使用”的报错。第 7 项最初称其为不支持的组合，实际是数据丢失缺陷。现已修复，见第 8 项。

<a id="8-demo-data-preloading---demo_path-equivalent----landed"></a>

### 8. 示范预加载 对应 `--demo_path` 已落地

HIL-SERL 的 `--demo_path`（`train_rlpd.py:458-466`）接收原始 pickle transition 字典列表，与 RLPD 通用的 `--offline_dataset` 结构化 h5/minari 加载器用途不同。每次启动，包括重启，都会把它无条件加载到持续接收介入数据的 `demo_buffer`，随后再通配加载恢复快照。此前 `rl-garden` 的示范缓冲区没有预加载路径，`init_demo_buffer()` 总从空开始，`hil_serl` 只能使用本次运行积累的介入数据，无法像通常的 HIL-SERL 真机运行一样，以已有遥操作示范初始化。

现已实现：

- `HilSerlLearnerLoop.__init__` 新增 `demo_dataset_paths: Sequence[str]` 通配参数；每次启动都在恢复快照前，经 `_load_demo_datasets()` 和 `add_demo_transition` 加载到 `agent.offline_replay_buffer`，与 HIL-SERL 每次重载 `demo_path` 的行为一致。
- 文件沿用已有缓冲区快照 pkl 约定：CPU 张量字典 `{"obs", "next_obs", "action", "reward", "done", ...}` 的 pickle 列表，与 `HilSerlLearnerLoop._snapshot()` 一致。该格式选择使此前运行的 `checkpoint_dir/demo_buffer/*.pkl` 可直接用于另一运行的 `--demo_dataset_paths` 初始化；HIL-SERL 原始格式则是 numpy 的 `{"observations", "actions", ...}` 字典。
- `HilSerlArgs` 新增 `demo_dataset_paths: Sequence[str] = ()`。
- `DemoInterventionMixin.init_demo_buffer()` 在 `offline_replay_buffer` 已有数据，例如来自 `--offline_dataset` 时抛异常，修复静默覆盖。RLPD 静态离线数据与 HIL-SERL 示范仍共用一个槽，同一 `RLPDHybrid` 暂不能同时使用两者，但现在会明确报错。

**第 8 项更正，已修复，以下保留原发现。** `_load_demo_datasets()` 在 `HilSerlLearnerLoop.__init__` 中先于 `_reload_snapshots()` 调用，曾为每条预加载示范递增 `self._received`，即使数据仅进入 `agent.add_demo_transition`，从未进入在线 `agent.replay_buffer`。`received_transitions` 控制 `LearnerLoop.run()` 的 `learning_starts` 等待，见 `rl_garden/real_world/learner_loop.py`；HIL-SERL 则用 `while len(replay_buffer) < config.training_starts`（`train_rlpd.py:278`），只计算在线数据，`--demo_path` 填充的 `demo_buffer` 不计入。于是 `--demo_dataset_paths` 提供至少 `learning_starts` 条示范时，`hil_serl` 在零在线 transition 时就能开始训练，而 HIL-SERL 始终等待真实在线数据。`tests/test_real_world_hil_serl_loops.py` 的 `test_hil_serl_learner_loop_preloads_demo_dataset_paths` 甚至断言预加载后 `received_transitions == 3`，固定了错误行为。**修复后**，`real_world_rl/loops/hil_serl.py` 的 `_load_demo_datasets()` 不再把预加载示范计入 `received_transitions`，`learning_starts` 等待在线数据；测试改为断言 `received_transitions == 0`。

**当时仍缺少的能力**：HIL-SERL 的 `examples/record_demos.py` 提供普通遥操作轨迹录制，无成功失败标签，直接生成 `--demo_path` 格式。此阶段 `rl-garden` 仅有 `rl_garden/models/reward/success/collect_data.py`，限定用于带成功失败标签的奖励分类器数据，尚无通用示范录制。要初始化 `--demo_dataset_paths`，当时需自行编写录制代码。

<a id="9-real-world-agentpolicy-checkpointing----landed"></a>

### 9. 真机智能体与策略 checkpoint 已落地

`LearnerLoop.run()`（`rl_garden/real_world/learner_loop.py`）曾直接通过 `agent.train(gradient_steps)` 训练，未调用 `OffPolicyAlgorithm._maybe_save_periodic_checkpoint` / `_save_checkpoint`。后两者仅由真机 `serl`、`hil_serl` 都不使用的 `OffPolicyAlgorithm.learn()` 单进程采样循环触发，还依赖 `LearnerLoop` 不递增的采样步数 `_global_step`。HIL-SERL learner 每 `checkpoint_period` 个梯度更新保存完整智能体状态，包含权重与优化器，并通过 `checkpoints.restore_checkpoint` 恢复，见 `train_rlpd.py:249-355,419-429`。

大部分恢复机制实际已有：`build_rlpd` / `build_rlpd_hybrid` 位于 `rl_garden/training/online/entrypoints/{rlpd,rlpd_hybrid}.py`，指定 `--load_checkpoint` 时已调用 `agent.load(args.load_checkpoint, ...)`；`serl.py` / `hil_serl.py` 的 `_run_learner` 也会传递 `args` / `checkpoint_dir`。因此真机 learner **通过 `--load_checkpoint` 恢复原本就可用**，只是缺少测试与文档，实际缺口是定期保存触发器。

现已在共享基类 `LearnerLoop` 中实现，`serl` 与 `hil_serl` 都以相同方式绕过 `learn()`，因此无需仅放在 `HilSerlLearnerLoop`：

- `BaseAlgorithm.global_update` 新增只读属性，暴露 `_global_update`。`load_state_dict` 正确恢复它，重启后保持单调，优于循环内局部计数。
- 每次 `_train_step()` 后调用 `LearnerLoop._maybe_save_periodic_checkpoint()`。当 `agent.checkpoint_dir` 存在且 `agent.checkpoint_freq > 0` 时，每跨越一个 `checkpoint_freq` 倍数保存 `checkpoint_{global_update}.pt`。按梯度更新计数，与 HIL-SERL learner 的更新步语义接近；`include_replay_buffer=agent.save_replay_buffer` 沿用已有参数。
- `LearnerLoop.run()` 的 `finally` 在 `agent.save_final_checkpoint` 开启时额外保存一次 `final.pt`，覆盖 `stop()`、`KeyboardInterrupt` 等正常停止；定期保存承担崩溃恢复。
- 无需新增 `HilSerlArgs` / `SerlArgs` 字段；`checkpoint_dir` / `checkpoint_freq` / `load_checkpoint` / `save_replay_buffer` / `save_final_checkpoint` 已由共享 `CheckpointArgs`（`rl_garden/common/cli_args.py`）暴露并传入智能体。

已知架构影响：`tests/test_real_world_learner_loop.py` 记录的“LearnerLoop 仅使用 `OffPolicyAlgorithm` 最小接口”约定扩展为包含 `checkpoint_dir` / `checkpoint_freq` / `save_replay_buffer` / `save_final_checkpoint` / `global_update` / `save(...)`。这是有意且已记录的扩展，`_FakeAgent` 和模块说明均已更新。

**周期细节**：上述与 HIL-SERL 的保存节奏并非精确相同。HIL-SERL 的 `checkpoint_period` 计算 `learner()` 外层迭代（`train_rlpd.py:314-355`），每轮包含 `config.cta_ratio` 次 `agent.update()`，默认 2，即一次仅 critic 和一次 critic、actor、温度联合更新。`rl-garden` 在 `sac_core.py` 每个原始梯度步骤递增 `_global_update`，`checkpoint_freq` 使用这个更细粒度计数。相同 `--checkpoint_freq` 在默认 `cta_ratio=2` 下实际周期约差两倍。两边内部定义一致，但跨项目比较参数时需考虑。

<a id="10-real-world-evaluation-mode----landed"></a>

### 10. 真机评估模式已落地

HIL-SERL actor 支持 `--eval_checkpoint_step` / `--eval_n_trajs`，加载指定 checkpoint，运行固定数量真机回合测成功率，不采集或训练，见 `train_rlpd.py:71-110`。

现已作为仅 `hil_serl` 的第三种角色实现；当时为限制影响未加入共享 `RealWorldFrankaArgs.role`，`serl.py` 保持不变：

- `HilSerlArgs.role: Literal["actor", "learner", "eval"]`，并新增 `eval_n_trajs: int = 10`。
- `_run_eval(args)` 位于 `rl_garden/training/real_world/hil_serl.py`，通过 `training/env_building.py` 的 `build_hil_env`，原名 `_build_env`，构建与 actor/learner 相同且包含奖励分类器的包装器栈，再通过 `build_rlpd_hybrid` 构建智能体。必须指定 `--load_checkpoint`，否则抛 `ValueError`；训练 `_run_actor` 则从零构建并丢弃该参数。评估运行 `eval_n_trajs` 个回合，固定 `deterministic=True`，不受控制训练探索的 `--deterministic_actor` 影响。没有同步客户端、learner 或数据推送。成功取 `terminated`；`RewardClassifierWrapper` 在分类器判成功时设为 `True`，实现于 `rl_garden/envs/wrappers/reward_classifier.py`。`truncated` 超时算失败。输出逐回合及汇总成功率、平均回报。
- `--fwbw` 与 `role="eval"` 同用时抛 `NotImplementedError`，属于本轮已声明的单臂边界之外。

**与 HIL-SERL 评估循环比较得到的细节**，见 `train_rlpd.py:67-110`：

- HIL-SERL 的 `--eval_checkpoint_step` 实际采用随机策略，`agent.sample_actions(..., argmax=False)` 位于 `train_rlpd.py:88-91`。`rl-garden` 的 `role="eval"` 始终使用 `deterministic=True`，两者成功率测量的策略行为因此不同。
- HIL-SERL 只检查 `done`，见 `while not done:` 与 `train_rlpd.py:86,95-98`，未检查 `truncated`，单纯超时可能无法结束参考循环。`rl-garden` 的 `_run_eval` 显式在 `bool(terminated) or bool(truncated)` 时退出，截断算失败。尚未验证 HIL-SERL 实际 Franka 环境是否在超时时也设置 `done=True`；若是，实际运行不受影响。

<a id="11-grasp_penalty-reward-shaping-term----landed"></a>

### 11. `grasp_penalty` 奖励塑形已落地

现有 `real_world_rl/envs/wrappers/grasp_penalty.py::GraspPenaltyWrapper` 在夹爪开闭切换时设置 `info["grasp_penalty"]`。它移植自任务专属 `usb_pickup_insertion/wrapper.py::GripperPenaltyWrapper`，这才是 HIL-SERL 的 `train_rlpd.py` / `sac_hybrid_single.py` 通过该 info 键实际使用的版本；基础 `franka_env/envs/wrappers.py::GripperPenaltyWrapper` 直接修改 `reward`，未接入真实训练循环。`RLPDHybrid` 新增 `use_grasp_penalty`，开启时回放通过 `DictReplayBuffer(store_grasp_penalty=True)` 保存 `grasp_penalty` 列，实现于 `rl_garden/buffers/dict_buffer.py`；`_train_discrete_critic` 目标改为 `(rewards + grasp_penalty) + ...`，与 HIL-SERL 公式一致。`HilSerlActorLoop._extra_transition_fields` 将其从 `info` 转发到 transition。`HilSerlArgs` 新增 `use_grasp_penalty`、`grasp_penalty_value`。

<a id="not-yet-migrated"></a>

## 尚未迁移

<a id="1-hg-dagger-training-mode--the-growing-dataset-buffer-layer"></a>

### 1. HG-DAgger 训练与持续增长数据集缓冲层

`train_hgdagger.py` 的迭代纠正需要持续增长且定期重载的磁盘示范与纠正数据集，与 `LearnerLoop` 的实时在线数据或经 RLPD `offline_dataset` 一次加载的静态数据路径不同。早期已明确将其排除，因为 `train_rlpd.py` 风格 HIL-SERL 无需此能力。

- `LearnerLoop._refresh_offline_data()` 位于 `rl_garden/real_world/learner_loop.py`，已预留为空操作钩子；`serl` 和 `hil_serl` 均未重写。
- 迁移期间参考的 `3rd_party/RLinf` 真机 DAgger/SFT 路径采用与内存 RL 回放解耦的磁盘数据流水线：LeRobot 格式写入器、独立离线 ETL，以及懒加载新数据的 PyTorch `Dataset`。持续增长的示范应采用这一类独立接口。

<a id="2-bc-behavior-cloning-agent----landed"></a>

### 2. BC 行为克隆智能体已落地

**更正**：最初编写条目时，`rl_garden/algorithms/bc.py::BC` 实际已经存在并完成注册，是仅使用 NLL 损失、无 critic 的 `OfflineRLAlgorithm`。真正缺少的只有真机入口。

现已实现 `real_world_rl/training/entrypoints/bc.py::run_real_world_bc`。BC 在静态示范上执行离线训练，没有 `serl` / `hil_serl` 的 actor/learner HTTP 分离，无需 `sync.py`、`LearnerLoop` / `ActorLoop`；它继承 `OfflineRLAlgorithm`，另两者使用 `OffPolicyAlgorithm`。示范 pkl 由共享 `real_world_rl/training/demo_loading.py::load_demo_pkls` 加载，该函数从 `HilSerlLearnerLoop._load_demo_datasets` 抽取，后者也改为调用，避免重复 glob 和 pickle 循环。

同时解决 tanh 压缩问题：`BCPolicy` / `BC` 新增默认保持原行为的 `tanh_squash: bool = True`。设为 `False` 时，使用 `rl_garden/networks/actor_critic.py` 的 `UnsquashedGaussianActor`，另一分支为 `SquashedGaussianActor`。这既匹配 HIL-SERL 的 `tanh_squash_distribution=False`，也解决明确数值问题：真实 Franka 示范近二值夹爪动作常恰在动作边界，tanh 雅可比修正在边界发散，使 `evaluate_action_log_prob` 退化。`RealWorldBCArgs` 与模拟/离线入口 `BCArgs` 都暴露该选项。

<a id="3-reward-classifier-training-script--data-collection-convention----landed"></a>

### 3. 奖励分类器训练脚本与采集约定已落地

原先 `rl_garden/envs/franka_real/classifier.py` 只有模型 `SuccessClassifier` 和推理加载器 `load_classifier_fn`，足够让 `RewardClassifierWrapper` 消费已有 checkpoint，却无法生成 checkpoint。

现已补齐，并将整个奖励模型接口移至 `rl_garden/models/reward/`。该目录原有尚未导入使用的离线 HDF5 标注分类器 `classifiers/{base,color,alignment,hsv}`，此次整理为奖励模型目录；未来 VLM 或规则奖励模型可作为同级模块加入。新增的 `success/` 对应在线真机成功分类器：

- `rl_garden/models/reward/success/model.py`：从 `franka_real/classifier.py` 原样迁入 `SuccessClassifier` / `load_classifier_fn`。
- `rl_garden/models/reward/success/data.py`：`SuccessClassifierDataset` 按成功/失败通配模式读取 `{"obs": obs}` pickle 列表，对应 HIL-SERL 的 `classifier_data/*_{success,failure}_*.pkl`。
- `rl_garden/models/reward/success/train.py`：独立 argparse CLI。监督分类训练不适合 RL Args/env_backend 结构，因此不经 `BaseAlgorithmRegistry`。流程仿照 HIL-SERL `examples/train_reward_classifier.py`，使用平衡成功失败批次、`binary_cross_entropy_with_logits`、来自 `rl_garden/encoders/augment.py` 的 `RandomShiftsAug`，不划分训练验证集，固定 epoch 数。
- `rl_garden/models/reward/success/collect_data.py`：移植 `examples/record_success_fail.py` 机制，使用 `pynput` 全局空格监听，将最近步进 transition 标记成功，其余默认失败。相机采集由调用方提供，参见 `env.py` 模块说明；这是原 `franka_real` 栈已有条件，`envs/backends/franka_real.py` 始终以 `camera_capture=None` 构建环境。CLI 当时未接相机，真实图像需要工作单元脚本调用 `main(camera_capture=...)`。

修复 `classifiers/hsv/__init__.py` 已有错误导入 `rl_garden.reward_models...` 为 `rl_garden.models.reward...` 时，还发现并修复 `classifiers/base/` 的两处导入问题：`metrics.py` 急切导入未在 `pyproject.toml` 声明的 `sklearn`，`transforms.py` 急切导入同样未声明的 `torchvision`，均改为函数内懒加载。此外 `classifiers/hsv/generate_labels.py` 开头有两个模块 docstring，导致后续 `from __future__ import annotations` 触发 `SyntaxError`，现合并为一个。

<a id="4-dual-arm-hybrid-gripper-sac_hybrid_dual"></a>

### 4. 双臂混合夹爪（`sac_hybrid_dual`）

此历史阶段的 `RLPDHybrid` / `DiscreteCritic` 仅实现 HIL-SERL `sac_hybrid_single` 布局，即一条连续机械臂加三分类夹爪 Q 头。`sac_hybrid_dual` 的两臂、12 维连续动作，以及通过两个夹爪笛卡尔积组成的联合 9 分类 Q 头当时尚未实现，因为 `FrankaRealEnv` 仅支持单台 Franka，尚无双臂环境可运行。这与当时真机 RL 栈的单臂边界一致。

<a id="5-franka-bridge-server-endpoints-set_load-close_gripper_slow-move_gripper-update_param"></a>

### 5. Franka 桥接端点 `/set_load`、`/close_gripper_slow`、`/move_gripper`、`/update_param`

比较 `3rd_party/hil-serl/serl_robot_infra/robot_servers/franka_server.py` 与已移植到 `robot_infra/controller/real/franka_bridge.py` 的基础 SERL 版本后，发现下列 HIL-SERL 路由尚未迁入：

- `/set_load`（POST）：用 `mass` / `F_x_center_load` / `load_inertia` 调用 ROS `/franka_control/set_load`，可能用于更换工具或夹爪后更新动力学。需要尚未接入的 `franka_msgs.srv.SetLoad` 依赖。
- `/close_gripper_slow`（POST）：Robotiq 专属慢速闭合，见下文。
- `/move_gripper`（POST，`franka_server.py:339-345`）：连续夹爪位置控制，在 `franka_gripper_server.py` 和 `robotiq_gripper_server.py` 上都调用 `move()`。当前 `rl-garden` 夹爪服务器仅提供二值开闭，经此桥接无法运行依赖连续宽度的策略。
- `/update_param`（POST，`franka_server.py:223-225,378-381`）：通过 `ReconfClient` 动态调整笛卡尔阻抗控制器刚度与阻尼。`franka_bridge.py` 尚无对应路由，无法经 HTTP 运行时调参。

细粒度读取路由已被有意合并：HIL-SERL 的 `/getpos`、`/getvel`、`/getforce`、`/getq`、`/getdq`、`/getjacobian` 等集中在 `franka_bridge.py` 的 `/getstate` 响应中，接口范围以 `FrankaBridgeClient` 的实际调用为准。该响应具体缺少 `q` / `dq` / `jacobian`，当前下游没有读取；`rl_garden/envs/franka_real/bridge_client.py` 的 docstring 却称字段存在但未用，这一文档错误可独立修正。

<a id="6-robotiq-gripper-support"></a>

### 6. Robotiq 夹爪支持

HIL-SERL 的 `robotiq_gripper_server.py` 与已移植 Franka Hand 版相比，`gripper_pos` 报告方向相反、默认部分打开、闭合力较低，并提供前述端点需要的 `close_slow()`。v1 Franka 桥接 `robot_infra/controller/real/gripper_server.py` 仅支持 Franka Hand，与 SERL v1 的最初范围一致，Robotiq 当时被列为按需增加的未来功能。

<a id="7-onlinedemo-buffer-split--periodic-disk-snapshot-crash-recovery----landed"></a>

### 7. 在线与示范缓冲区分离及定期磁盘快照已落地

`3rd_party/hil-serl/examples/train_rlpd.py:140-233` 在 actor 侧维护两个内存缓冲区：所有 transition 进入 `data_store`；存在 `info["intervene_action"]` 的介入数据额外进入 `intvn_data_store`，形成持续增长的人工纠正集。learner 直接从内存采样。每隔 `config.buffer_period` 步，将上次保存后累积的数据写到 `checkpoint_path/buffer/transitions_{step}.pkl` 与 `checkpoint_path/demo_buffer/transitions_{step}.pkl`，重启后通配重载两个目录，见 `train_rlpd.py:457-490`。磁盘文件用于崩溃恢复。

两部分均已实现，复用已有机制，无需引入第三套混合方式：

- **示范缓冲区**：`rl_garden/buffers/demo_intervention.py` 的 `DemoInterventionMixin(PriorDataReplayMixin)` 新增 `init_demo_buffer()` / `add_demo_transition()`，复用 RLPD 静态 `--offline_dataset` 所在的 `offline_replay_buffer` / `offline_data_ratio` 槽，改为增量增长。`_sample_train_batch` / `_concat_replay_samples` / `_shuffle_batch` 原样继承。**已知限制**：`--offline_dataset` 与实时示范共用槽，同一 `RLPDHybrid` 无法同时使用两者；现按“第 7 项更正”抛错，避免静默丢失。类定义为 `rl_garden/algorithms/rlpd_hybrid.py` 的 `RLPDHybrid(DemoInterventionMixin, RLPD)`。
- **介入标记**：`ActorLoop` / `FWBWActorLoop` 位于 `rl_garden/real_world/actor_loop.py`，新增默认空操作 `_extra_transition_fields(info) -> dict` 钩子，`HilSerlActorLoop` 重写并附加 `{"intervened": "intervene_action" in info}`。**已知限制**：FWBW 的 `_run_actor` 使用普通 `FWBWActorLoop`，未使用 `HilSerlActorLoop`，因此缺少介入标记，所有数据仅进入在线缓冲区。
- **路由与快照**：`HilSerlLearnerLoop` 位于 `rl_garden/real_world/hil_serl/learner_loop.py`，重写 `_on_transition`，匹配 HIL-SERL `train_rlpd.py:178-193`：弹出 `intervened` 后所有 transition 进入 `agent.replay_buffer`，介入数据再额外经 `agent.add_demo_transition` 复制，示范为在线数据的重复子集。每收到 `--buffer_period` 条数据，将累积的、设备转移前的 CPU 张量保存到 `<checkpoint_dir>/buffer/transitions_{n}.pkl` 和 `<checkpoint_dir>/demo_buffer/transitions_{n}.pkl`，后者仅介入数据。启动时重载两目录，接收计数仅从在线快照累计，避免重复计数。本文该历史阶段称它为 `hil_serl` 唯一持久化路径，因为当时 `LearnerLoop.run()` 尚未触发已有 `save_replay_buffer` / `include_replay_buffer` checkpoint 机制（`rl_garden/algorithms/off_policy.py:56,323`），因此当时没有双重重载风险。

  **更正**：初版互斥路由将介入数据仅送示范缓冲区，其他数据仅送在线缓冲区。HIL-SERL 实际把所有数据送入 `data_store`，并将介入数据额外放入 `intvn_data_store`，见 `train_rlpd.py:178-193`。旧实现导致在线采样部分完全没有人工纠正，与 HIL-SERL 分布不同。修复仅修改 `HilSerlLearnerLoop`，`DemoInterventionMixin`、`ActorLoop`、基础 `LearnerLoop` 不变。
- 新增 `HilSerlArgs` 字段 `demo_buffer_size`、`demo_data_ratio`、`buffer_period`；现声明于 `HilEnvArgs`，由 `HilSerlArgs`、`ResidualHilSerlArgs`、`SilriArgs` 共享。

**共享循环接口更新**：标记和路由现通过可重写钩子表达，`hil_serl` 默认行为保持不变。`ActorLoop.run()` 调用 `_act`、`_build_transition`、`_push`、`_on_episode_end`、`_on_params_refreshed`、`_on_run_end`；`HilSerlActorLoop` 仍仅重写 `_extra_transition_fields`。介入包装器始终写 `info["intervened"]`，仅介入时写 `intervene_action`；`HilSerlActorLoop` 仍依据 `"intervene_action" in info` 标记。`HilSerlLearnerLoop._buffer_kwargs(transition) -> (core, extra)` 决定传入 `replay_buffer.add` 的字段，默认继续丢弃 `intervened` 和 `actor_id`，残差与 SiLRI 重写后为 `InterventionReplayBuffer` 保留 `intervened`、`policy_action` 和各自列。

<a id="12-gripper_pos-normalization-bug--other-robot_servers-deviations"></a>

### 12. `gripper_pos` 归一化缺陷与其他 `robot_servers` 差异

`robot_infra/controller/real/gripper_server.py` 从 HIL-SERL 的 `franka_gripper_server.py` 移植，但存在此前未发现的差异：

- **缺少 `/0.08` 归一化**：HIL-SERL 在 `franka_gripper_server.py:64-66` 使用 `self.gripper_pos = np.sum(msg.position) / 0.08`，将 `gripper_pos` 归一化到约 `[0, 1]`。`gripper_server.py` 只计算 `float(sum(msg.position))`，没有除法，旁边注释却称符合 SERL 约定。原文称结果尺度相差约 12.5 倍；该值直接进入 `FrankaRealEnv` 状态观测（`rl_garden/envs/franka_real/env.py`），使相同物理夹爪宽度产生不同数值范围。此项当时尚未修复。
- **没有开闭幂等保护**：HIL-SERL 用 `self.binary_gripper_pose` 在已处目标状态时跳过重复 `/open_gripper` 或 `/close_gripper`（`franka_gripper_server.py:21-43`）。本实现每次无条件发布 `MoveActionGoal` / `GraspActionGoal`，策略持续保持开闭时每步都发送冗余命令。
- **未显式传入 `load_gripper` roslaunch 参数**：HIL-SERL 的 `franka_server.py` 按夹爪类型向 `impedance.launch` / `joint.launch` 传 `load_gripper:=true/false`。`franka_bridge.py` 省略该项；由于 `3rd_party/serl_franka_controllers` 启动文件默认 `true`，且当时仅支持 Franka Hand，实际值正确，但依赖隐式默认。

<a id="13-safety-box-only-clamps-position-not-orientation----landed-correction-already-was"></a>

### 13. 安全盒仅裁剪位置而未裁剪方向 已有实现

**更正**：后续审计发现初始条目过时，方向裁剪原已存在。`FrankaRealEnvConfig` 位于 `envs/franka_real/config.py`，包含默认关闭、必须成对设置的 `Optional` 字段 `safety_box_rpy_low` / `safety_box_rpy_high`。`FrankaRealEnv._clip_orientation()` 位于 `envs/franka_real/env.py`，转为 Euler `"xyz"`，第一角保留符号按绝对值裁剪，与 HIL-SERL `clip_safety_box()` 的环绕特例相同，其余角直接裁剪后转换回去。设置 `safety_box_rpy_low` 时在 `step()` 应用。真正差异是本项目默认 `None`、按需开启，而 HIL-SERL 始终开启。

<a id="14-no-quaternion-to-eulerrotvec-conversion-on-the-observation-side----landed-correction-already-was"></a>

### 14. 观测侧四元数到欧拉角或旋转向量转换 已有实现

**更正**：该能力原已存在。`RotvecObsWrapper` 将 `obs["state"]` 中四元数片段转为三维旋转向量。HIL-SERL 使用 `Quat2EulerWrapper` 转欧拉角；本项目有意使用 rotvec，避免万向节锁与环绕特例，并保持动作约定一致。它接入 `training/env_building.py::compose_hil_env`，原入口为 `hil_serl.py::_build_env`，由默认关闭的 `--convert_obs_rotation` 控制。后续位置审计将文件从 `rl_garden/envs/wrappers/rotvec_obs.py` 移至 `real_world_rl/envs/wrappers/rotvec_obs.py`，因为其硬编码 `FrankaRealEnv` 精确 20 维布局，属于真机层；对应测试原已位于 `real_world/tests/`。

<a id="15-missing-franka_env-wrapper-variants-humanclassifierwrapper-multistagebinaryrewardclassifierwrapper-grippercloseenv----landed"></a>

### 15. `franka_env` 包装器变体 `HumanClassifierWrapper`、`MultiStageBinaryRewardClassifierWrapper`、`GripperCloseEnv` 已落地

现已采用批量 torch 实现并遵循 `RewardClassifierWrapper` 约定。`HumanClassifierWrapper` 位于 `real_world_rl/envs/wrappers/human_classifier.py`，阻塞 `input()` 仅适用于实时遥操作，因此放在 `real_world/`。`MultiStageBinaryRewardClassifierWrapper` 位于通用 `rl_garden/envs/wrappers/multi_stage_classifier.py`，不依赖形状或动作空间，每个阶段仅奖励一次，全部阶段完成后置 `terminated`。`GripperCloseEnv` 位于 `real_world_rl/envs/wrappers/gripper_close.py`，把 7 维动作缩为 6 维，转发前补零，并裁剪 `info["intervene_action"]`。它原先放在 `rl_garden/`，后因其 7 维且末维夹爪约定专属 `FrankaRealEnv` 而迁入真机层；ManiSkill/RoboTwin 没有采用该约定，通用多阶段包装器则不受此限制。

以下保留原说明供参考：

`RewardClassifierWrapper` 位于 `rl_garden/envs/wrappers/reward_classifier.py`，对应第 3 项，仅覆盖 HIL-SERL 最简单的单阶段模型成功判断 `MultiCameraBinaryRewardClassifierWrapper`。HIL-SERL 的 `3rd_party/hil-serl/serl_robot_infra/franka_env/envs/wrappers.py` 另外两个变体当时尚无对应实现：

- `HumanClassifierWrapper`（`wrappers.py:15-34`）：回合结束时阻塞 `input()`，由人输入成功或失败，适合暂不值得训练分类器的任务。
- `MultiStageBinaryRewardClassifierWrapper`（`wrappers.py:69-100`）：顺序多阶段成功追踪，每阶段维护 `received`，累加奖励，全部阶段满足后才结束，适合多个顺序子目标。

当时还缺少 `GripperCloseEnv`（`wrappers.py:180-204`），它是始终闭爪任务使用的 `ActionWrapper`，将动作空间裁成 6 维，移除夹爪维度再补零。当时 `rl_garden/envs/wrappers/` 没有对应包装器。

<a id="16-pico-and-spacemouse-intervention-detection-are-structurally-different-informational"></a>

### 16. Pico 与 SpaceMouse 介入检测的结构差异

`robot_infra/teleop/spacemouse/spacemouse_teleop_wrapper.py` 的 `SpaceMouseTeleOpWrapper` 接近 HIL-SERL 的 `SpacemouseIntervention`（`3rd_party/hil-serl/serl_robot_infra/franka_env/envs/wrappers.py:207-252`），持续进行速度控制，以幅度死区 `norm(expert_a) > threshold` 判定介入。另一设备 `robot_infra/teleop/utils/telo_op_control_twist.py` 的 Pico `EETwistTeleOpWrapper` 当时采用绑定按钮门控：按住物理 bind 按钮时计算绑定后位移并报告非零、介入的速度增量；没有新数据包则返回零且未介入。两者统一到 `TeleOpSample.intervened`，由 `TeleopInterventionWrapper` 消费，但激活语义不同：Pico 未按 bind 时移动控制器不会介入，SpaceMouse 超死区运动会介入。选择 `--teleop_device` 时需理解设备行为。

**更新**：SpaceMouse 侧已改变。`--teleop-device spacemouse` 不再使用旧 `robot_infra/teleop/spacemouse` 的 `SpaceMouseTeleOpWrapper`，而由 `teleop/spacemouse.py` 的 `SpaceMouseTeleopSource` 驱动设备层 `SpaceMouseTeleop`（pyspacemouse 2.x），经 `TeleopPartSource` 映射。仍按范数超过 `deadzone` 或按住按钮介入，可追加 `hold_s` 保持；按钮 0 切换锁存夹爪，按钮 1 结束回合。Pico 保留 bind 门控 `PicoTeleopSource`。两者现统一到按名称注册的 `TeleopSource`、`TeleopReading.intervened`，由 `InterventionWrapper` / `TeleopInterventionWrapper` 消费，详见 `docs/teleop.md`。

<a id="17-no-real-camera-capture-integration"></a>

### 17. 缺少真实相机采集集成

HIL-SERL 在 `3rd_party/hil-serl/serl_robot_infra/franka_env/camera/{video_capture,rs_capture,multi_video_capture}.py` 提供完整相机集成，以线程 `VideoCapture` 封装直接使用 `pyrealsense2` 的 `RSCapture`，接入 `FrankaEnv.init_cameras()` / `get_im()`。本文历史阶段的 `rl-garden` 尚无等价模块；`rl_garden/envs/backends/franka_real.py` 的 `make_train_env` / `make_eval_env` 不传 `camera_capture`，真机栈默认 `None`。这是 `franka_real` 原有条件，第 3 项已记录，意味着当时必须手工编写工作单元集成才能让 `hil_serl` 获得真实图像。

<a id="18-no-real-world-rollout-video-recording----landed"></a>

### 18. 真机采样视频录制已落地

`real_world_rl/envs/wrappers/video_recorder.py::VideoRecorderWrapper` 结合上游两种方案：像 `VideoWrapper` 累积观测流图像，像 `VideoRecorder` 在回合结束时写盘。原始帧经管道发送给 `ffmpeg` 子进程，遵循 `rl_garden/envs/robotwin/adapter.py` 的评估视频方式，无需增加 `imageio` / `tensorflow`。它仅接入 `hil_serl.py`、`residual_hil_serl.py`、`silri.py` 的 `_run_actor` actor 角色，由共享 `HilEnvArgs` 新字段 `record_video_dir` 控制。

以下保留原说明供参考：

HIL-SERL 的 `VideoRecorder` / `VideoWrapper` 位于 `3rd_party/hil-serl/serl_launcher/serl_launcher/wrappers/{video_recorder,video_wrapper}.py`，将采样视频存盘供调试。原 `rl-garden` 真机路径没有对应实现；唯一视频写入代码 `rl_garden/envs/robotwin/adapter.py` 仅用于 RoboTwin 模拟评估，因此当时无法录制真机 `hil_serl` / `serl` 视频供事后排查。

<a id="19-replay-buffer-scalability-gaps-no-image-frame-dedup-no-bounded-actor-transport-queue----landed"></a>

### 19. 回放可扩展性 图像去重与有界 actor 传输队列已落地

两部分均已实现：

- **`pack_obs_and_next_obs` 帧去重**：新增继承 `DictReplayBuffer` 的 `rl_garden/buffers/memory_efficient_dict_buffer.py::MemoryEfficientDictReplayBuffer`，每个指定图像键每个时刻仅存一帧，采样时用 torch 窗口 gather 重建 `(T, H, W, C)` 堆叠，保持 GPU 驻留，无需 NumPy `sliding_window_view`。回合边界采用边缘复制，与 `ImageFrameStackWrapper` 的 `reset()` 一致；逐槽 episode ID 检查防止旧位置的时间邻居已被后续回合覆盖，测试已证明该情况可到达且被正确拒绝。上游必须包含 `ImageFrameStackWrapper`，`add()` 会显式断言。通过 `RLPDHybrid(memory_efficient_buffer=True, ...)` / `HilSerlArgs.memory_efficient_buffer` 按需启用。
- **有界 actor 传输队列**：`ActorSyncClient` 新增 `max_queue_size`，默认 600，10 Hz 下约 60 秒积压；满时丢最旧数据，始终不阻塞调用者，通过 `dropped_transitions` 计数观测。`stop()` 的结束哨兵也使用非阻塞路径；旧 `queue.put(None)` 在满队列时可能永久挂起，此次一并修复。

<a id="20-temporal-observation-stacking-not-wired-into-the-real-world-path----landed"></a>

### 20. 真机路径的时间观测堆叠已落地

`ImageFrameStackWrapper` 位于 `rl_garden/envs/wrappers/frame_stack.py`，新增非 ManiSkill 通用途径，通过显式 `image_keys` 指定相机键，因为 `FrankaRealEnv` 相机名由用户定义，未必有 `rgb*` / `depth*` 前缀。堆叠的 `single_observation_space` 直接由空间定义推导，构建时绝不调用 `env.reset()`，避免物理机器人意外运动。接入 `training/env_building.py::compose_hil_env`，原入口为 `hil_serl.py::_build_env`，位于 `make_training_envs(...)` 后，由 `RLPDHybridArgs` 继承的既有 `obs.frame_stack` 控制，无重复字段。ManiSkill 的 `image_keys=None` 路径不变。

以下保留原说明供参考：

HIL-SERL 的 `ChunkingWrapper` 位于 `3rd_party/hil-serl/serl_launcher/serl_launcher/wrappers/chunking.py`，不依赖环境，把状态和图像所有观测键堆叠为时间窗口，统一用于模拟和真机，还支持滚动时域动作块执行。`rl-garden` 最接近的 `ImageFrameStackWrapper`（`rl_garden/envs/wrappers/frame_stack.py`）仅堆叠图像，状态保持单帧，也没有动作块概念；当时仅接入 ManiSkill 后端 `rl_garden/envs/maniskill/env.py`，`hil_serl` 的 `_build_env`（`rl_garden/training/real_world/hil_serl.py`）未应用，真机 Franka 因而没有时间观测堆叠。

<a id="landed-since-the-initial-migration"></a>

### 初次迁移后落地的工作

以下并非 HIL-SERL 原有条目，但改变了上述迁移缺口的范围。它们均只支持真机循环，没有模拟器 `learn()` 路径：

- **遥操作基类**：`teleop/` 中的 `TeleopSource` / `TeleopReading`、名称注册表、设备层 SpaceMouse，以及通用 `InterventionWrapper`，见第 16 项更新与 `docs/teleop.md`。
- **共享 HIL 环境构建器**：`training/env_building.py::build_hil_env(args, *, role, enable_teleop)`；learner 传 `enable_teleop=False`，GPU 机器不打开遥操作设备。`hil_serl._build_env` 保留为 `compose_hil_env` 的薄别名。
- **带人工介入的残差 RL**（`residual_hil_serl`）：在冻结 rl-garden checkpoint、ACT 或零基础策略上学习有界残差，包含 `ResidualRLPD` 与 `ResidualRLPDHybrid`，人工覆盖总动作。HIL-SERL 原本没有该方法，见 `docs/residual.md`。
- **HIL-RL / SiLRI**（`silri`）：用状态相关拉格朗日方法从次优介入学习，见 `docs/silri.md`。此历史清单仍把 HG-DAgger（第 1 项）、主臂遥操作后端和双臂夹爪（第 4 项）列为未覆盖。

<a id="behavioral-divergences-from-hil-serls-real-robot-defaults"></a>

## 与 HIL-SERL 真机默认配置的行为差异

以下能力在 `rl-garden` 中都已具备，但默认配置或一项无条件实现选择与 HIL-SERL 真机配方不同，配方位于 `serl_launcher/serl_launcher/utils/launcher.py` 和 `examples/experiments/*/config.py`。除第一项当时硬编码外，其他差异通过参数或默认值即可对齐。

- **`backup_entropy` 当时硬编码 `True`**：`SAC.__init__`（`rl_garden/algorithms/sac_core.py`）没有 `backup_entropy` 参数，目标 Q 总减去 `alpha * logpi`。HIL-SERL 真机配置固定 `backup_entropy=False`（`serl_launcher/serl_launcher/utils/launcher.py:83,133`），当时无法传参对齐。
- **actor LayerNorm 默认关闭**：`RLPDHybridArgs` 仅覆盖 `critic_use_layer_norm=True`，`actor_use_layer_norm` 仍继承 `False`，见 `rl_garden/training/online/args.py`。HIL-SERL 的 `make_sac_pixel_agent_hybrid_single_arm` 在 critic 与策略 kwargs 均设置 `use_layer_norm=True`，见 `launcher.py:120-130`。
- **真机入口图像增强当时默认关闭**：`VisionArgs.image_augmentation` 默认 `"none"`（`rl_garden/common/cli_args.py`），`RLPDHybridArgs` 未像 `DrQv2TrainingArgs` 一样覆盖成 `"random_shift"`。HIL-SERL 为每个真机智能体启用 DrQ 随机裁剪（`launcher.py:190`）；可用 `--image_augmentation random_shift` 对齐。
- **ResNet 默认从零训练且未冻结**：HIL-SERL 的 `encoder_type="resnet-pretrained"` 真机配置总在 `resnet_v1.py` 的 `pre_pooling` 中用 `stop_gradient` 冻结主干，仅训练池化头。`ResNetEncoder` 仅在同时给出 `pretrained_weights` 与 `freeze_resnet_encoder` / `freeze_resnet_backbone=True` 时冻结；当时 `VisionArgs` 均默认关闭，因此训练整个网络。
- **默认池化不同**：HIL-SERL 固定 `pooling_method="spatial_learned_embeddings"`（`sac.py:468,487`）；`ResNetEncoder` 默认 `"spatial_softmax"`（`rl_garden/encoders/resnet.py`），当时真机入口未覆盖。两种机制均在 `rl_garden/encoders/pooling.py` 实现。
- **REDQ 集成默认遵循 RLPD 论文**：`RLPDHybridArgs` 默认 `n_critics=10, critic_subsample_size=2`，与 `RLPD` 文档相同；HIL-SERL 对所有智能体固定 `critic_ensemble_size=2, critic_subsample_size=None`（`launcher.py:86-87,136-137,186-187`），真机实际未使用 REDQ 子采样。
- **离散夹爪 critic 总停止共享编码器梯度**：`RLPDHybrid._train_discrete_critic` 在 `rl_garden/algorithms/rlpd_hybrid.py` 无条件调用 `extract_features(..., stop_gradient=True)`。HIL-SERL 的 `grasp_critic_loss_fn` 通过 `sac_hybrid_single.py:233-289` 的 `grad_params=params` 反向传播到自身池化头，仅冻结预训练主干卷积。
- **两边均未使用 FiLM 条件化**：HIL-SERL 的 `FilmConditioning` 仅由 `-film` ResNet 配置触达，真机智能体构建器未选择；`rl_garden/encoders/film.py` 同样是明确记录、未接入算法的占位实现，因此不列为缺口。
