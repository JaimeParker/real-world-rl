<!-- English source: docs/tutorial/05-base-policy.md -->

<a id="5-base-policy-bc-or-act"></a>

# 5. 基础策略：BC 或 ACT

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/05-base-policy.md)

示范训练的基础策略既可单独部署作基线（[08](08-deploy.md)），也可在 [07](07-hil-training.md) 的 `residual_hil_serl` 上学习有界修正。用 [04](04-record-demos.md) pkl 在 GPU 机器训练一种或两种，都不连接机器人。全部命令从 GPU 机器仓库根、项目 `.venv` 激活时运行。动作/观测须与现场录制一致；离线训练不打开运动设备。

| | BC（`real_world_bc`） | ACT |
|---|---|---|
| 模型 | rl-garden BC，每观测一个动作 | 动作分块 CVAE Transformer，每步预测 `--num-queries` 个动作 |
| 训练入口 | `examples/train_real_world.py real_world_bc` | `python -m rl_garden.models.act.train_trajectories` |
| 残差基础 spec | `checkpoint:<path>?algo=real_world_bc` | `act:<path>?temporal_agg=1` |
| 单独部署 | 支持，`--algo real_world_bc` | 不支持，仅作残差基础 |

<a id="bc"></a>

## BC

`real_world_bc` 从 pkl 或轨迹归档离线学习，不连机器人。`--config` 定义目标动作/观测及实验契约，加载前验证。部分示范只监督 `human_action_mask` 维度，idle hold 补位不是专家标签。无介入元数据的旧示范保留全动作监督，显式空 mask 仍为空。

在 GPU 仓库根准备**仅供离线的 `hil.yaml`**：复制现场版 `examples/presets/gello_cartesian.yaml`，移除根部全部 `teleop_*` 字段（设备、事件设备、选项路径、四个 tracker 值），保留 `env_backend`、`env_id`、`obs` 和整个 `real_robot`。BC 不接受在线 teleop CLI 字段；离线加载器从示范元数据恢复 tracker 契约，因此保留 sidecar。该副本仅供 BC，04 和 07 用完整预设。

```bash
python examples/train_real_world.py real_world_bc --config hil.yaml \
    --demo-dataset-paths demos/pick-and-place.pkl \
    --log-dir runs --exp-name bc-pick-and-place --buffer-size 100000 \
    --num-offline-steps 20000 --no-tanh-squash
```

- `--no-tanh-squash`：夹爪动作严格为 ±1，默认 tanh 分布在边界的对数概率数值退化，见 `OfflineBCArgs.tanh_squash` 注释。
- `--buffer-size`：至少覆盖全部示范步；默认 1000000，`--buffer-device cuda`，按 GPU 容量调整。
- `--num-offline-steps` 默认 100000；示例为 20000 梯度步。
- `--log-freq` 默认 1000，向 `--log-type`（默认 TensorBoard）记录损失的频率。

*应该看到：* 每个日志间隔输出指标，进程结束后有 `runs/bc-pick-and-place/checkpoints/final.pt` 和 `config.json`。部署和残差基础从训练配置重建策略。

*否则：* `recorded action contract does not match the env's (differs in [...])` 表示动作模式、关节限位、尺度、状态字段等与预设不同，重新录制或修正预设。`No demo transitions loaded` 表示路径/glob 未匹配。

<a id="act"></a>

## ACT

rl-garden 从同样 pkl 训练 ACT，无需模拟器。安装后按 [01](01-install.md#4-install-in-a-clean-venv)，在仓库根运行：

```bash
python -m rl_garden.models.act.train_trajectories --demo-paths demos/pick-and-place.pkl \
    --total-iters 50000 --batch-size 256 --num-queries 30 --no-use-cameras \
    --log-dir runs --exp-name act-pick-and-place
```

- 示范的每个 `rgb_<camera>` 是视觉策略相机；此预设无相机，命令以 `--no-use-cameras` 仅用 `state`。图像缩放到 `--image-size`（224）并按 ImageNet 归一化。
- `--backbone-weights` 初始化 ResNet：默认 `imagenet` 下载 torchvision `IMAGENET1K_V1`，或 `none`，或离线权重路径。
- `--num-queries` 为块长度，30 步在 50 Hz 为 0.6 s。`done` 切断回合，块不跨回合。
- `--save-freq N` 每 N 次额外写 `<iter>.pt`，训练期间不评估。

*应该看到：* 退出后产生 `runs/act-pick-and-place/checkpoints/final.pt`。详见 `rl-garden/rl_garden/models/act/README.md` 的轨迹 pkl 训练。真实示范 ACT 尚未实测，见 [00](00-overview.md#known-limits)。

<a id="the-act-checkpoint-contract"></a>

### ACT checkpoint 契约

残差 actor 通过 `act:` 加载：

- `obs["state"]` 宽度和动作宽度在 provider 构建时校验；需同样 `state_fields`，这里 `[tcp_pose, tcp_vel, joint_pos, joint_vel]` 共 27 值。
- 视觉 checkpoint 按排序后的 `rgb_<camera>` 读取，录制和运行须同样 `obs.rgb`，配置在 `examples/presets/gello_cartesian.yaml`。
- 默认 `state_dict_key=ema_agent`，从 checkpoint `act_config` 重建模型，按保存统计反归一化动作并裁剪到 `[-1,1]`。动作模式和尺度须仍与示范一致。

[07](07-hil-training.md) 使用的 spec 在 shell 中加引号，因为 `?` 是 glob、`&` 启动后台：

<!-- tutorial-test: skip -->
```bash
--base-provider 'act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
```

可用 `&` 组合：`temporal_agg=1` 默认聚合重叠块，权重 `exp(-temporal_agg_k * i)`；`0` 开环执行每块；另有 `temporal_agg_k=0.01`、`state_dict_key=ema_agent`。

<a id="resync-after-an-intervention"></a>

### 介入后的重新同步

人工使机器人离开原 ACT 规划轨迹。没有 resync 时，开环 provider 继续旧块直到边界；聚合 provider 仍混合介入前和介入中的预测，使 `base_actions` / `next_base_actions`、介入 BC 目标和交还后策略滞后最多一个块。
默认开启 `--residual-base-resync-on-intervention`，`--no-residual-base-resync-on-intervention` 可关闭。actor 在每个介入步后、计算下一观测的 base 前重置 provider，使介入后每个观测使用在该观测重新规划的首动作。接管发生的那步仍保留接管前算出的 base。`zero` 和 BC `checkpoint` provider 无状态，不受影响。

> **joint_abs：**基础策略须在残差环境动作空间内。joint_abs 流程用独立示范和预设。两种关节模式动作宽度相同，宽度校验无法识别把 joint_delta base 用到 joint_abs，须自行保证模式一致。

<a id="keep-the-experiment-contract-with-the-base"></a>

## 基础策略与实验契约一起保存

将 `config.json`、`experiment.json`、checkpoint 一起保存。除形状外，还检查任务、控制、预处理、相机顺序和动作含义。录制前配置好，并贯穿残差运行。见[实验契约](../real-world-experiments.md#trajectory-archives-and-training-views)。

下一步：[06 人工介入](06-intervening.md)，再读 [07 HIL](07-hil-training.md)。
