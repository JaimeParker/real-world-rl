# 路线三 · 真机强化学习

部署时参数固定，actor 负责推理；在线训练时 learner 还会根据新数据更新参数，再把更新发回 actor。先用上一章确认工作站能够执行策略动作、记录状态、交还控制，然后再启动学习。HIL-SERL 自己初始化策略；只有显式选择 residual 并提供 ACT 时，ACT 才作为基础策略参与在线控制。

## 先定义一轮怎样结束

在工作站上用路线一的机制各标记一次成功和失败。检查归档中的任务结果、奖励来源、人工控制标记。人工抓取后按成功键，只能说明任务成功；计算自主成功率时还要检查这一轮有无人工接管。按现场任务核对复位动作、允许的工作区和回合步数上限。

## 启动 learner 和 actor

从已验证的会话配置整理出 `local/hil.yaml`。保留环境、观测、遥操作来源、输入档案和动作尺度，删除录制专用的输出路径、配额、事件文件以及本次会话的诊断输出路径。两端使用相同代码、任务配置与示范契约。若 learner 在另一台计算机，把示范 pkl、契约和所引用的配置复制过去。actor 仍部署在能读取真实观测和遥操作输入的一端。

在两端的 Bash 终端各自设置公共参数。这里以同机为例；分机时把地址改为 actor 能访问的 learner 地址。

```bash
COMMON=(--config local/hil.yaml --sync-host 127.0.0.1 --sync-port 6000
        --log-dir runs --exp-name hil-task --checkpoint-freq 1000
        --buffer-size 20000 --demo-buffer-size 20000 --reward-from-teleop-events)
```

先在 learner 端执行：

```bash
rwrl train hil_serl --role learner "${COMMON[@]}" --demo-dataset-paths demos/task.pkl
```

观察示范是否加载，learner 是否开始等待 actor。随后在上位机确认工作区已清场，再执行：

```bash
rwrl supervise --startup-s 60 --log runs/hil-task/actor-supervise.jsonl -- \
    rwrl train hil_serl --role actor "${COMMON[@]}"
```

actor 收到策略参数后开始回合。设备服务的 gRPC 地址负责机器人通信；这里的 sync 地址用于 actor 和 learner 交换样本与参数。定位故障时先看哪一端没有收到它要等的东西。

## 追踪一次纠正

| 设备 | 现场操作 |
|---|---|
| SpaceMouse | 按录制时选定的模式介入。tracker 按按钮 0 接管，修正后再按一次交还。SERL 依据手柄活动和夹爪按键介入。 |
| GELLO | 沿用已标定的主臂和接管配置，对齐后接管。修正并交还时观察夹爪是否保持预期状态。 |

选中这一回合的同一步，依次看策略提议的动作、人工接管的维度、真正送给环境的动作及下一帧观测。actor 将样本暂存到本地，发送给 learner；learner 收到样本并写入后用于更新。观察 learner 更新步数、actor 接收新策略的记录。出现中断时保留运行目录和同步记录，在恢复前明确最后持久化的 checkpoint 和已经接收的样本。[实验契约与恢复](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/real-world-experiments.md)

## 评估一个更新后的策略

停下占用工作站的 actor，再单独加载评估 checkpoint。把下面路径改成确实已写出的文件。一次评估先验证回合能够完整结束，多轮才用于估计成功率。

```bash
rwrl train hil_serl --role eval --config local/hil.yaml --exp-name hil-task \
    --load-checkpoint runs/hil-task/checkpoints/final.pt --eval-n-trajs 1
```

与先前部署的策略比较时使用相同任务定义和成功口径，分别报告协助成功与自主成功。如果研究问题是“能否在已有 ACT 上学习修正量”，选择 residual 路径，在 learner 和 actor 两侧使用相同的 `--base-provider 'act:runs/act-task/checkpoints/final.pt?temporal_agg=1'`。[Residual 与其他真机训练入口](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/tutorial/10-hil-training.md)
