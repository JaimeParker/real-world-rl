# 部署与推理

先回答“这个策略理解的动作，和这台工作站执行的动作是否相同”。例如，两个七维向量可以分别表示七个关节目标、或六维末端运动加夹爪。只核对数组长度还不够。

## 独立运行 ACT

准备三份材料：路线二的 `final.pt`、训练示范对应的 `task.contract.json`、采集端保存的会话配置。在部署机激活项目环境，恢复录制使用的设备连接方式与路径。若为 daemon 模式，应先启动原工作站 daemon。将下面第一行改成选定会话的实际路径：

```bash
SESSION_CONFIG=demos/task.session-实际编号/resolved-config.yaml
rwrl supervise --startup-s 60 --log runs/act-deploy.jsonl -- \
    rwrl deploy-act --config "$SESSION_CONFIG" \
    --checkpoint runs/act-task/checkpoints/final.pt \
    --demo-contract demos/task.contract.json \
    --episodes 1 --max-steps-per-episode 30
```

`deploy-act` 检查示范动作契约与当前环境，按实际观测和动作空间加载 ACT，使用 checkpoint 的统计量恢复动作尺度。在每个回合开始时清空上一回合的分块缓存。默认进行时间集成，即当前时刻可综合多个预测片段对这一时刻的估计。这里没有 learner，也无需 residual checkpoint。

首次实机运行前核对 home、工作区、夹爪和现场停止方式。回合开始会 reset，步数上限约束 reset 后的策略循环。若任务显式指定了评估回合数，检查任务配置中的次数。看到 `[deploy] episode=… steps=…` 后检查实际步数、终止原因、任务结果和有无人介入。通过动作契约检查后，仍须用实测观测核对运动方向和幅度。

## 在模拟设备上重现软件初验

在项目测试环境从仓库根目录执行。这个用例启动回环 gRPC daemon，用脚本化主臂采集示范，训练小型视觉 ACT，再经正式 CLI 运行两个回合：

```bash
python -m pytest -q tests/test_single_arm_station_smoke.py -k act_checkpoint -rs
```

确认结果是通过；缺依赖时 pytest 可能显示 skipped。需要远程 π0.5 推理的读者继续阅读[从 checkpoint 到 openpi 策略服务](openpi.md)。已完成部署初验的工作站可进入[真机强化学习](rl.md)。
