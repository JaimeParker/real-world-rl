<a id="5-base-policy-bc-or-act"></a>

# 5. 基础策略 BC 或 ACT

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/05-base-policy.md)

用示范训练的基础策略有两种用途：独立部署时可作为基线（[08](08-deploy.md)），也可供 [07](07-hil-training.md) 的 `residual_hil_serl` 在其上学习有界修正。在 GPU 机器上使用 [04](04-record-demos.md) 生成的 pkl，训练其中一种或两种策略。两种离线训练流程都无需连接机器人。

| 项目 | BC（`real_world_bc`） | ACT |
|---|---|---|
| 模型 | rl-garden 的 BC actor，每次观测输出一个动作 | 动作分块 Transformer（CVAE），每步预测 `--num-queries` 个动作 |
| 训练入口 | `examples/train_real_world.py real_world_bc` | rl-garden 中的 `python -m rl_garden.models.act.train_trajectories` |
| 残差基础策略描述 | `checkpoint:<path>?algo=real_world_bc` | `act:<path>?temporal_agg=1` |
| 独立部署 | 支持，使用 `--algo real_world_bc` | 当前仅支持作为残差基础策略 |

<a id="bc"></a>

## BC

`real_world_bc` 从示范 pkl 或轨迹归档离线训练，无需连接机器人。`--config` 定义目标动作、观测和实验约定，加载器在训练前核对记录约定。部分示范只监督 `human_action_mask` 标识的维度，未选部件的保持动作不属于专家标签。缺少介入元数据的旧示范沿用全动作监督；显式空掩码保持为空。

```bash
python examples/train_real_world.py real_world_bc --config hil.yaml \
    --demo-dataset-paths demos/pick-and-place.pkl \
    --log-dir runs --exp-name bc-pick-and-place --buffer-size 100000 \
    --num-offline-steps 20000 --no-tanh-squash
```

- `--no-tanh-squash`：录制的夹爪动作恰为 +1 或 -1，默认 tanh 压缩 actor 在端点处的对数概率会数值退化，参见 rl-garden 的 `OfflineBCArgs` 中 `tanh_squash` 注释。
- `--buffer-size` 是加载示范的回放容量，默认 1000000，位于 `--buffer-device cuda`。容量至少应覆盖录制的 transition 数量，并根据显存调整。
- `--num-offline-steps` 是梯度更新次数，默认 100000；示例使用 20000。
- `--log-freq` 决定每隔多少步将 BC 损失写入 `--log-type` 指定的日志，默认间隔 1000，日志类型默认 TensorBoard。

*预期结果*：每隔 `--log-freq` 步打印一行指标。进程退出且生成 `runs/bc-pick-and-place/checkpoints/final.pt` 与 `runs/bc-pick-and-place/config.json` 后训练结束。`checkpoints/` 旁的 `config.json` 供部署和残差基础策略提供器读取，用于重建策略。

*结果不符时*：`ValueError: ... recorded action contract does not match the env's (differs in [...])` 表示示范与预设的动作模式、关节限制、缩放或状态字段不同，需要重新录制或修正预设。`No demo transitions loaded` 表示路径或通配符没有匹配到数据。

<a id="act"></a>

## ACT

rl-garden 可以从相同 pkl 训练 ACT，无需模拟器。安装 rl-garden 后（`pip install -e rl-garden`，见 [01](01-install.md#install)），从仓库根目录运行：

```bash
python -m rl_garden.models.act.train_trajectories --demo-paths demos/pick-and-place.pkl \
    --total-iters 50000 --batch-size 256 --num-queries 30 \
    --log-dir runs --exp-name act-pick-and-place
```

- 示范观测中的每个 `rgb_<camera>` 键都会成为视觉策略的一个相机输入，此处为 `rgb_wrist`；`--no-use-cameras` 仅使用 `state`。图像缩放至 `--image-size`，默认 224，并进行 ImageNet 标准化。
- `--backbone-weights` 设置 ResNet 初始化方式：默认 `imagenet` 会下载 torchvision 的 `IMAGENET1K_V1`；也可使用 `none`，或为离线机器指定 torchvision state-dict 路径。
- `--num-queries` 是动作块长度；10 Hz 下 30 步为 3 秒。回合以 `done` 分隔，动作块不会跨越示范末尾。
- `--save-freq N` 额外每 `N` 次迭代保存 `<iter>.pt`。训练期间不执行评估。

*预期结果*：进程退出时生成 `runs/act-pick-and-place/checkpoints/final.pt`。详情见 `rl-garden/rl_garden/models/act/README.md` 中“从轨迹 pkl 训练”一节。真实示范上的 ACT 训练尚待验证，见 [00](00-overview.md#known-limits)。

<a id="the-act-checkpoint-contract"></a>

### ACT checkpoint 约定

残差 actor 使用 `act:` 提供器加载 checkpoint：

- `obs["state"]` 的维度必须匹配 checkpoint，动作空间也必须匹配其动作维度，两项都在构建提供器时检查。训练示范需要使用相同的 `state_fields`，此处为 `[joint_pos, gripper_pos]`，共 8 个值。
- 视觉 checkpoint 按排序后的 `rgb_<camera>` 键读取环境图像；录制与运行时保持 `hil.yaml` 中的 `obs.rgb` 一致。
- 提供器默认使用 `ema_agent` 权重，即 `state_dict_key=ema_agent`，并从 checkpoint 的 `act_config` 重建模型。动作按 checkpoint 统计量反标准化，再裁剪到 `[-1, 1]`，因此其含义取决于示范的 `action_mode` 和缩放设置，两者必须一致。

[07](07-hil-training.md) 使用的描述字符串如下，在 shell 中需要加引号，因为 `?` 会触发通配符匹配、`&` 会启动后台任务：

<!-- tutorial-test: skip -->
```bash
--base-provider 'act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'
```

选项用 `&` 连接：`temporal_agg=1` 为默认值，以 `exp(-temporal_agg_k * i)` 权重集成重叠动作块；`0` 则开环重放每个动作块。其他选项为 `temporal_agg_k=0.01` 和 `state_dict_key=ema_agent`。

<a id="resync-after-an-intervention"></a>

### 人工介入后的重新同步

人工操作会使机器人偏离 ACT 动作块原先规划的轨迹。如果不重新同步，开环提供器（`temporal_agg=0`）会继续执行旧动作块直到下一块开始；集成提供器（`temporal_agg=1`）则继续平均接管前和接管中预测的动作块。因此，保存的 `base_actions` / `next_base_actions`、介入数据上的 BC 目标，以及交还控制后立即执行的基础动作，最多会落后机器人一个动作块。
`--residual-base-resync-on-intervention` 默认开启，可用 `--no-residual-base-resync-on-intervention` 关闭。开启后，actor 在每个介入步骤之后、计算下一观测的基础动作之前重置提供器。这样，介入步骤后每个观测的基础动作都是基于该观测重新规划的动作块的首个动作。刚开始接管的那个步骤仍保留接管前计算的基础动作。`zero` 与 `checkpoint`（BC）提供器没有状态，因此该参数不影响它们。

> **joint_abs。** 残差基础策略必须在残差训练所用的动作空间中输出。`joint_abs` HIL 运行需要用 `joint_abs` 预设和 `joint_abs` 示范训练 BC 或 ACT。两种动作模式的维度相同，提供器的维度检查无法发现把 `joint_delta` 基础策略用于 `joint_abs` 运行的问题，因此需要自行保持模式一致。

<a id="keep-the-experiment-contract-with-the-base"></a>

## 为基础策略保留实验约定

将训练 `config.json`、`experiment.json` 与 checkpoint 一起保留。除形状外，还会核对任务、控制、预处理、相机顺序和动作解释。在录制前确定这些配置，并在残差运行中保持一致，见[实验约定](../real-world-experiments.md#trajectory-archives-and-training-views)。

下一步：[06 人工介入](06-intervening.md)，然后是 [07 HIL 训练](07-hil-training.md)。
