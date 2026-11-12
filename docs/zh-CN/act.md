<a id="action-chunking-with-transformers-act"></a>

# 基于 Transformer 的动作分块（ACT）

[中文入口](../README.zh-CN.md) · [English](../../rl-garden/rl_garden/models/act/README.md)

本代码用于运行论文 [《使用低成本硬件学习精细双臂操作》](https://arxiv.org/pdf/2304.13705) 中的 ACT 算法，改编自[原始代码](https://github.com/tonyzhaozh/act)。

<a id="installation"></a>

## 安装

开始前，建议使用 conda/mamba 创建一个新环境并安装依赖：

```bash
conda create -n act-ms python=3.9
conda activate act-ms
pip install -e .
```

<a id="setup"></a>

## 配置

请通读[模仿学习设置文档](https://maniskill.readthedocs.io/en/latest/user_guide/learning_from_demos/setup.html)，其中详细介绍了在 ManiSkill 中运行模仿学习基线所需的知识，包括如何下载和预处理示范数据集、如何公平地评估和比较策略，以及提高性能和避免错误的建议。

<a id="training"></a>

## 训练

我们提供了使用示范数据训练 ACT 的脚本。

有些示范执行较慢（例如运动规划或人工遥操作生成的示范），可能超过默认的回合步数上限。模仿学习算法会学到与示范相近的任务完成速度，因此这可能导致策略来不及完成任务。此时可以通过 `--max-episode-steps` 提高上限，为策略留出足够时间。通常建议将其设为训练所用示范平均长度的约两倍。`baselines.sh` 中提供了经过调优的基线脚本，为各项任务设置了推荐的 `--max-episode-steps` 值。

以下是基于状态的训练示例，使用运动规划在 PickCube-v1 任务中生成的 100 条示范进行学习。

```bash
seed=1
demos=100
python train.py --env-id PickCube-v1 \
  --demo-path ~/.maniskill/demos/PickCube-v1/motionplanning/trajectory.state.pd_ee_delta_pos.physx_cpu.h5 \
  --control-mode "pd_ee_delta_pos" --sim-backend "physx_cpu" --num_demos $demos --max_episode_steps 100 \
  --total_iters 30000 --log_freq 100 --eval_freq 5000 \
  --exp-name=act-PickCube-v1-state-${demos}_motionplanning_demos-$seed \
  --track # track training on wandb
```

<a id="training-from-trajectory-pkls"></a>

## 从轨迹 pkl 训练

`train_trajectories.py` 从以 pickle 格式保存的状态转移列表训练 ACT，无需模拟器或 ManiSkill，适用于真机上录制的遥操作示范等数据。从仓库根目录运行：

```bash
python -m rl_garden.models.act.train_trajectories \
  --demo-paths "demos/*.pkl" \
  --total-iters 50000 --batch-size 256 --num-queries 30 \
  --exp-name act-my-task
```

- 数据：每个 pkl 保存按时间顺序排列的状态转移字典列表，包含 `obs`、`action` 和 `done`，还可包含 `next_obs`、`truncated`；支持首维大小为 1 的批量轴。`obs["state"]` 是形状为 `(D,)` 的状态；每个 `rgb_<camera>` 键对应视觉策略的一台相机，图像形状为 `(H, W, 3)`、类型为 uint8，所有相机的图像尺寸须一致。`--no-use-cameras` 表示仅使用状态训练。若 `obs` 本身就是张量，也会将其视为状态。
- 回合在 `done` 或 `truncated` 处结束；某条状态转移的 `next_obs` 与下一条的 `obs` 不一致时，也会划分新回合（可用 `--no-split-on-observation-gaps` 关闭此行为）。动作块不会跨越回合结尾：超出结尾的条目会被填充，且不计入 L1 损失。
- 状态和动作使用从数据中计算的统计量进行标准化；图像缩放到 `--image-size` 指定的尺寸，并按 ImageNet 统计量归一化。`--backbone-weights` 用于初始化 ResNet，可设为 `imagenet`（默认值，下载 torchvision 的 `IMAGENET1K_V1`）、`none`，或供离线机器使用的 torchvision state-dict 文件路径。
- 输出：`runs/<exp-name>/checkpoints/final.pt`（另外每隔 `--save-freq` 次迭代保存一个 `<iter>.pt`），包含 `agent`、`ema_agent`、`norm_stats` 和完整的 `act_config`。`ACTBaseActionProvider.from_checkpoint(ckpt_path=...)` 默认使用 EMA 权重加载检查点。训练期间不进行评估。

<a id="citation"></a>

## 引用

如果使用本基线，请引用以下文献：

```
@inproceedings{DBLP:conf/rss/ZhaoKLF23,
  author       = {Tony Z. Zhao and
                  Vikash Kumar and
                  Sergey Levine and
                  Chelsea Finn},
  editor       = {Kostas E. Bekris and
                  Kris Hauser and
                  Sylvia L. Herbert and
                  Jingjin Yu},
  title        = {Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware},
  booktitle    = {Robotics: Science and Systems XIX, Daegu, Republic of Korea, July
                  10-14, 2023},
  year         = {2023},
  url          = {https://doi.org/10.15607/RSS.2023.XIX.016},
  doi          = {10.15607/RSS.2023.XIX.016},
  timestamp    = {Thu, 20 Jul 2023 15:37:49 +0200},
  biburl       = {https://dblp.org/rec/conf/rss/ZhaoKLF23.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```
