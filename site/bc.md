# 路线二 · 行为克隆：用 ACT 学习示范

同一回合里，操作员会依次靠近物体、闭合夹爪、抬起手臂。BC 用操作员的动作作为监督信号。这里选 ACT：每次给它当前观测，让它预测从这一刻起的一小段动作。若频率是 50 Hz，30 步对应 0.6 秒；换成 10 Hz，同样的 30 步覆盖 3 秒。

## 准备数据

在训练机放好路线一产生的 `demos/task.pkl` 和 `demos/task.contract.json`。预留独立录制的完整回合做离线检查，避免把相邻帧分到训练和检查两侧。SpaceMouse 和 GELLO 的数据都可以训练 ACT；混合前应核对动作模式、尺度、坐标系、夹爪、相机名和状态字段顺序。演示本身带有接管记录；对双臂单侧接管的数据，还需核实训练器怎样消费部分动作标签。本例使用单臂完整人工示范。

先用与采集一致的状态输入训练一个基线。在训练机仓库根目录执行：

```bash
python -m rl_garden.models.act.train_trajectories \
    --demo-paths demos/task.pkl --no-use-cameras \
    --num-queries 30 --total-iters 50000 --batch-size 256 \
    --log-dir runs --exp-name act-task
```

输出是 `runs/act-task/checkpoints/final.pt`。需要视觉时，采集阶段应已保存相机观测，训练时移除 `--no-use-cameras`。保留训练时相机名、裁剪和缩放配置。小数据冒烟可减小训练步数和批量；它验证数据管道，任务表现要靠独立回合评估。

## 离线检查

仅对自己生成或信任的 checkpoint 执行下面的读取。在训练机确认模型的动作分块、相机和归一化统计：

```python
import torch

ckpt = torch.load("runs/act-task/checkpoints/final.pt", map_location="cpu", weights_only=False)
print(ckpt["act_config"])
print(ckpt["camera_keys"], ckpt["norm_stats"].keys())
```

比较留出回合时，使用同一组状态字段和图像预处理，载入训练权重与统计量，按时间顺序输入观测。每个新回合先清空 ACT 的动作缓存。看预测的靠近方向、夹爪切换时刻与人工动作是否一致；多个合理抓取轨迹之间也可能有差异。
下面用留出示范的第一帧做一次独立推理。示例针对本仓单工作站录制器写出的带单环境批次的观测；路径填自己可信的 pkl。

```python
import pickle
from pathlib import Path

import numpy as np
import torch
from gymnasium import spaces
from real_world_rl.policies.base_provider import ACTBaseProvider

with Path("demos/held-out.pkl").open("rb") as stream:
    row = pickle.load(stream)[0]
obs = {key: torch.as_tensor(value) for key, value in row["obs"].items()}
observation_space = spaces.Dict({
    key: spaces.Box(0, 255, shape=tuple(value.shape[1:]), dtype=np.uint8)
    if key.startswith("rgb_") else
    spaces.Box(-np.inf, np.inf, shape=tuple(value.shape[1:]), dtype=np.float32)
    for key, value in obs.items()
})
action = torch.as_tensor(row["action"]).reshape(1, -1)
action_space = spaces.Box(-1.0, 1.0, shape=(action.shape[-1],), dtype=np.float32)
policy = ACTBaseProvider("runs/act-task/checkpoints/final.pt",
                         observation_space=observation_space, action_space=action_space)
policy.reset()
with torch.inference_mode():
    print("ACT", policy.select_action(obs))
print("示范", action)
```

下一章用相同的 ACT 加载器把预测接到环境。[ACT 训练器与模型配置](https://github.com/JaimeParker/rl-garden/tree/50db18b092aa618555e23fcbf73c53b0a9ad9e58/rl_garden/models/act)
