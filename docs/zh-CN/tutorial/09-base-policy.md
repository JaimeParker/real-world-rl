<!-- English source: docs/tutorial/09-base-policy.md -->

<a id="9-base-policy-bc-or-act"></a>

# 9. 基础策略：BC 或 ACT

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/09-base-policy.md)

**目标：** 从 [08](08-record-demos.md) 训练离线 BC 或 ACT 基础策略，保存检查点及 HIL、部署所需元数据。BC 可独立部署；[10](10-hil-training.md) 可在任一基础策略上训练有界残差。

<a id="before-you-start"></a>

## 开始前

所有命令在 **GPU 主机**的**仓库根目录**、**激活项目环境**后运行（[01](01-install.md)）。两个训练器均不连接机器人。

- 将 [08](08-record-demos.md) 的 pkl 及契约伴随文件复制到此主机的相同相对路径。
- 从 NUC 带来已调试 `local/gello_cartesian.yaml`，保留动作、观测、任务、控制设置。NUC 上完整预设保留供录制和 HIL 使用。

| | BC (`real_world_bc`) | ACT |
|---|---|---|
| 模型 | rl-garden 的 BC actor，每次观测一个动作 | 动作分块 transformer (CVAE)，每步预测 `--num-queries` 个动作 |
| 训练命令 | `rwrl train real_world_bc` | `python -m rl_garden.models.act.train_trajectories`（rl-garden 内） |
| 残差基础规格 | `checkpoint:<path>?algo=real_world_bc` | `act:<path>?temporal_agg=1` |
| 独立部署 | 支持，`--algo real_world_bc` | 不支持（仅作残差基础） |

<a id="1-train-bc"></a>

## 1. 训练 BC

`real_world_bc` 从示范 pkl 或轨迹归档离线训练，不连接机器人。`--config` 预设定义目标动作/观测及实验契约，加载器训练前校验录制契约。部分示范仅监督 `human_action_mask` 维度；空闲保持补充不是专家标签。无介入元数据的旧示范保留全动作监督；显式空掩码仍为空。

`real_world_bc` 接受录制、HIL 和部署共用的 `local/gello_cartesian.yaml`，离线 BC 训练时忽略其中的遥操作设置。

在 GPU 主机的仓库根目录，激活项目环境：

```bash
rwrl train real_world_bc --config local/gello_cartesian.yaml \
    --demo-dataset-paths demos/pick-and-place.pkl \
    --log-dir runs --exp-name bc-pick-and-place --buffer-size 100000 \
    --num-offline-steps 20000 --no-tanh-squash
```

- `--no-tanh-squash`：录制夹爪动作恰为 +1 或 -1，默认 tanh 压缩 actor 在此处对数概率数值退化（见 rl-garden `OfflineBCArgs` 的 `tanh_squash` 注释）。
- `--buffer-size` 为加载示范的回放容量（默认 1000000，位于 `--buffer-device cuda`）；至少覆盖录制 transition 数，并适应 GPU 显存。
- `--num-offline-steps`（默认 100000）为梯度步数；20000 是示例。
- `--log-freq`（默认 1000）控制 BC 损失写入 `--log-type`（默认 TensorBoard）的频率。

*应看到：* 每 `--log-freq` 步记录 BC 损失；进程退出且存在 `runs/bc-pick-and-place/checkpoints/final.pt` 和 `runs/bc-pick-and-place/config.json` 时完成。部署和残差基础提供器读取 `checkpoints/` 旁的 `config.json` 重建策略。

*否则：* `ValueError: ... recorded action contract does not match the env's (differs in [...])` 表示示范与预设设置不同（动作模式、关节限制、尺度、状态字段）：重新录制或修正预设。`No demo transitions or restored replay available` 表示路径或 glob 未匹配数据。环境失败见[失败怎么办](#if-it-fails)。

<a id="2-alternative-train-act"></a>

## 2. 替代：训练 ACT

rl-garden 可从相同 pkl 训练 ACT，无需模拟器。在 GPU 主机仓库根目录，激活项目环境：

```bash
python -m rl_garden.models.act.train_trajectories --demo-paths demos/pick-and-place.pkl \
    --total-iters 50000 --batch-size 256 --num-queries 30 --no-use-cameras \
    --log-dir runs --exp-name act-pick-and-place
```

- 示范观测每个 `rgb_<camera>` 键成为视觉策略的一台相机（本预设无相机）；命令用 `--no-use-cameras` 仅训练 `state`。图像缩放到 `--image-size`（224）并按 ImageNet 归一化。
- `--backbone-weights` 初始化 ResNet：`imagenet`（默认，下载 torchvision `IMAGENET1K_V1`）、`none`，或离线机器上的 torchvision state-dict 路径。
- `--num-queries` 为块长（50 Hz 时 30 步 = 0.6 s）。回合在 `done` 结束，块不跨越示范结尾。
- `--save-freq N` 每 `N` 次迭代另存 `<iter>.pt`。训练期间不评估。

*应看到：* 进程退出时生成 `runs/act-pick-and-place/checkpoints/final.pt`。详见 `rl-garden/rl_garden/models/act/README.md` 的“Training from trajectory pkls”。失败先检查示范路径和观测键，再看[失败怎么办](#if-it-fails)。真实示范 ACT 训练尚未实际验证（[00](00-overview.md#known-limits)）。

<a id="the-act-checkpoint-contract"></a>

### ACT 检查点契约

残差 actor 用 `act:` 提供器加载检查点：

- `obs["state"]` 宽度须匹配检查点，动作空间宽度也须匹配（构建提供器时检查）：使用同 `state_fields` 的示范训练（此处 `[tcp_pose, tcp_vel, joint_pos, joint_vel]`，27 个值）。
- 视觉检查点按排序顺序读取环境 `rgb_<camera>` 键；录制和运行的 `local/gello_cartesian.yaml` 应使用相同 `obs.rgb`。
- 提供器默认使用 `ema_agent` 权重 (`state_dict_key=ema_agent`)，从检查点 `act_config` 重建模型；动作按检查点统计反归一化，再裁剪至 `[-1, 1]`，含义须与示范一致（同 `action_mode` 和尺度）。

[10](10-hil-training.md) 的 `--base-provider` 使用带引号值 `'act:runs/act-pick-and-place/checkpoints/final.pt?temporal_agg=1'`（`?` 会 glob，`&` 会后台执行）。

选项以 `&` 连接：`temporal_agg=1`（默认，按 `exp(-temporal_agg_k * i)` 权重集成重叠块）或 `0`（开环逐块回放）、`temporal_agg_k=0.01`、`state_dict_key=ema_agent`。

<a id="resync-after-an-intervention"></a>

### 介入后重新同步

人工操作会使机器人偏离 ACT 块规划轨迹。不重新同步时，开环提供器 (`temporal_agg=0`) 持续回放旧块至下一块边界；集成提供器 (`temporal_agg=1`) 持续平均接管前和期间预测的块，使存储的 `base_actions` / `next_base_actions`、介入行 BC 目标及交还后策略所用基础动作最多滞后一个块。`--residual-base-resync-on-intervention`（默认开启；`--no-residual-base-resync-on-intervention` 关闭）使 actor 每个介入步后、计算该步下一观测的基础动作前重置提供器。因此介入步后每个观测的基础动作，均为在该观测新规划块的首动作。人工接管当步保留接管前计算的基础动作。`zero` 和 `checkpoint` (BC) 提供器无状态，此选项对它们无影响。

> **joint_abs。** 残差基础策略必须在残差运行的动作空间中行动。`joint_abs` HIL 运行应以 `joint_abs` 预设和示范训练 BC 或 ACT。两种模式动作宽度相同，提供器宽度检查不能发现 `joint_abs` 运行中误用 `joint_delta` 基础策略：需自行保持模式一致。

<a id="keep-the-experiment-contract-with-the-base"></a>

## 将实验契约与基础策略一起保存

训练 `config.json`、`experiment.json` 和检查点一起保存。任务、控制、预处理、相机顺序、动作解释与形状一并检查。录制前配置，残差运行期间保持。见[实验契约](../real-world-experiments.md#trajectory-archives-and-training-views)。

<a id="if-it-fails"></a>

## 失败怎么办

在 GPU 主机仓库根目录，激活项目环境：

```bash
rwrl doctor
```

从首个红色行和提示开始；见 [Python 环境与扩展](../doctor.md#python-environment-and-extras)。离线 GPU 主机无站点为 `SKIPPED`，不代表训练失败。契约错误时对照录制预设比较指定差异；保持伴随文件完整。

下一步：[操作员页面](../intervening.md)，然后 [10 HIL 训练](10-hil-training.md)。
