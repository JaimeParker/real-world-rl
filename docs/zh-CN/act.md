<a id="action-chunking-with-transformers-act"></a>

# 基于 Transformer 的动作分块 ACT

[中文入口](../README.zh-CN.md) · [English](../../rl-garden/rl_garden/models/act/README.md)

本实现用于运行论文 [Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware](https://arxiv.org/pdf/2304.13705) 的 ACT 算法，改编自[原始代码](https://github.com/tonyzhaozh/act)。

<a id="installation"></a>

## 安装

建议用 conda/mamba 创建新环境并安装依赖：

```bash
conda create -n act-ms python=3.9
conda activate act-ms
pip install -e .
```

<a id="setup"></a>

## 配置

阅读 [ManiSkill 模仿学习配置文档](https://maniskill.readthedocs.io/en/latest/user_guide/learning_from_demos/setup.html)，了解示范下载、预处理、公平评估比较，以及提高效果和避免缺陷的建议。

<a id="training"></a>

## 训练

项目提供从示范训练 ACT 的脚本。

某些示范较慢，例如运动规划或人工遥操作，可能超过默认回合步数。模仿学习通常学到与示范相近的完成速度，因此可以通过 `--max-episode-steps` 提高上限，让策略有时间完成任务。通常建议设为训练示范平均长度的约两倍。已调优基线在 `baselines.sh` 为各任务设置推荐值。

以下状态训练示例使用 PickCube-v1 的 100 条运动规划示范：

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

`train_trajectories.py` 可从 pickle transition 列表训练 ACT，无需模拟器或 ManiSkill，适用于真机遥操作示范。从仓库根目录运行：

```bash
python -m rl_garden.models.act.train_trajectories \
  --demo-paths "demos/*.pkl" \
  --total-iters 50000 --batch-size 256 --num-queries 30 \
  --exp-name act-my-task
```

- 数据：每个 pkl 为按时间排列的 transition 字典列表，包含 `obs`、`action`、`done`，可选 `next_obs`、`truncated`，接受首维为 1 的批量轴。`obs["state"]` 为 `(D,)` 状态；每个 `rgb_<camera>` 键为 `(H, W, 3)` uint8，所有相机尺寸一致，成为视觉策略相机输入。`--no-use-cameras` 仅使用状态。裸张量 `obs` 也视为状态。
- 回合在 `done` 或 `truncated` 处结束；当前 `next_obs` 与下一条 `obs` 不同时也拆分，可用 `--no-split-on-observation-gaps` 关闭。动作块不会跨回合，越过结束的部分填充并排除于 L1 损失。
- 状态与动作根据数据统计量标准化；图像缩放至 `--image-size` 并按 ImageNet 归一化。`--backbone-weights` 可用默认 `imagenet`，下载 torchvision `IMAGENET1K_V1`，或使用 `none`、离线 torchvision state-dict 路径。
- 输出 `runs/<exp-name>/checkpoints/final.pt`，每 `--save-freq` 次迭代另存 `<iter>.pt`，内容包括 `agent`、`ema_agent`、`norm_stats` 和完整 `act_config`。`ACTBaseActionProvider.from_checkpoint(ckpt_path=...)` 默认加载 EMA 权重。训练期间不评估。

<a id="citation"></a>

## 引用

使用本基线时，请引用以下文献：

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
