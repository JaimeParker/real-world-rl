<a id="8-deploy"></a>

# 8. 部署

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/08-deploy.md)

部署在机器人上执行 checkpoint，不运行 learner、遥操作或数据采集。训练目录中位于 `checkpoints/` 旁的 `config.json` 决定动作与观测约定，命令行只提供连接信息。

<a id="the-deploy-daemon"></a>

## 部署守护进程

使用工作站 YAML 的副本启动部署服务，**移除其中的 `teleop:` 配置段**。配置了遥操作后，机器人连接时设备必须存在，而部署没有操作员。将 `local/station.yaml` 复制为 `local/station.deploy.yaml`，删除机器人的 `teleop:` 配置块。先停止训练守护进程，因为同一机器人上的第二个守护进程会收到 `DeviceBusy`。部署服务的 `--jpeg-quality` 应与训练一致，本教程未设置该参数：

```bash
python -m robot_infra.server --config local/station.deploy.yaml --watchdog-s 5
```

<a id="run-a-checkpoint"></a>

## 执行 checkpoint

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc-pick-and-place/checkpoints/final.pt \
    --algo real_world_bc --real_robot.remote ROBOT_PC:50051 \
    --episodes 5 --max-steps-per-episode 200
```

| 运行 | `--algo` | checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| `hil_serl` | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/final.pt` 或 `checkpoint_<update>.pt`。 |
| `residual_hil_serl` | `--residual-variant hybrid` 使用 `residual_hil_serl_hybrid`，`continuous` 使用 `residual_hil_serl`。 | `runs/residual-bc-pick-and-place/checkpoints/final.pt` 或 `runs/residual-act-pick-and-place/...`；基础策略提供器从 checkpoint 元数据重建。 |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/final.pt` |

例如，使用 ACT 基础策略的残差运行：

```bash
python examples/deploy_real_robot.py \
    --checkpoint runs/residual-act-pick-and-place/checkpoints/final.pt \
    --algo residual_hil_serl_hybrid --real_robot.remote ROBOT_PC:50051 --episodes 5
```

`--base-provider` 可以替换保存的基础策略描述，例如基础 checkpoint 移动后使用 `--base-provider 'act:/data/act/final.pt?temporal_agg=1'`。守护进程要求令牌时增加 `--real_robot.token T`；GPU 推理增加 `--device cuda`，默认 `cpu`。ACT 基础策略当前通过残差 checkpoint 部署。

*预期结果*：每回合输出一行 `[deploy] episode=0 steps=... return=... truncated=... rtt_ms=... health_ok=True`，最后汇总 `[deploy] summary episodes=... mean_steps=... mean_return=... rtt_ms=... all_healthy=True`。部署没有奖励来源，因此回报为 0，需观察机器人判断任务是否成功。

*结果不符时*：设备故障，例如守护进程不可达、watchdog 或安全限制问题，会向 stderr 输出 `[deploy] device error: ...` 和 `[deploy] hint: ...`，退出码为 2。命令行指定的约定字段与训练不符时抛出 `ValueError`，例如对 `joint_delta` 运行指定 `--real_robot.action_mode joint_abs`；去掉该参数，让训练配置决定。拔掉 GELLO 后机器人无法连接，说明守护进程使用的 YAML 仍含 `teleop:` 段。

约定字段和限制详见 README 的[部署](../project-readme.md#deploy)一节。
