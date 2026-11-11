<!-- English source: docs/tutorial/08-deploy.md -->

<a id="8-deploy"></a>

# 8. 部署

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/08-deploy.md)

部署从 checkpoint 执行策略，无 learner 和训练数据上传。它恢复训练 `config.json` 与 `experiment.json`，重建策略、包装器和任务。支持独立 `.pt` 和 learner 的 `bundle_manifest.json`；manifest 在推理时解析为对应 generation 模型，不恢复回放。

<a id="the-deploy-daemon"></a>

## 部署守护进程

无人接管部署时，将 `local/station.yaml` 复制为 `local/station.deploy.yaml` 并删除机器人的 `teleop:` 块。守护进程连接时，已配置设备必须在场。先停止训练守护进程，同机器人第二个实例会收到 `DeviceBusy`。部署采用与训练相同的 `--jpeg-quality`，本教程未设置：

```bash
robot-infra daemon --station local/station.deploy.yaml --watchdog-s 5
```

<a id="run-a-checkpoint"></a>

## 执行 checkpoint

无人接管运行时，显式关闭接管：

```bash
rwrl deploy --checkpoint runs/bc-pick-and-place/checkpoints/final.pt --algo \
    real_world_bc --real_robot.remote ROBOT_PC:50051 --no-enable-teleop --episodes 5 \
    --max-steps-per-episode 200
```

| 运行 | `--algo` | checkpoint |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| 使用教程夹爪的 `hil_serl` | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/bundle_manifest.json`、`final.pt` 或 `checkpoint_<update>.pt`。 |
| `residual_hil_serl` | `--residual-variant hybrid` 用 `residual_hil_serl_hybrid`，`continuous` 用 `residual_hil_serl`。 | 对应残差运行的 `bundle_manifest.json` 或 `.pt`，基础提供器从元数据重建。 |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/bundle_manifest.json` 或 `.pt`。 |

例如，使用 ACT 基础策略的残差 bundle：

```bash
rwrl deploy --checkpoint \
    runs/residual-act-pick-and-place/checkpoints/bundle_manifest.json --algo \
    residual_hil_serl_hybrid --real_robot.remote ROBOT_PC:50051 --no-enable-teleop --episodes 5
```

基础 checkpoint 移动后，可用 `--base-provider` 替换记录描述，例如 `--base-provider 'act:/data/act/final.pt?temporal_agg=1'`。需要令牌时增加 `--real_robot.token T`；GPU 推理增加 `--device cuda`，默认 `cpu`。ACT 通过残差 checkpoint 使用。checkpoint 应与运行配置、实验元数据一同保留；`--config-path` 可指定移动后的训练配置。双臂还需选择 `--env-backend real_robot_bimanual` 并提供各臂连接和相机，见 [bimanual.md](../bimanual.md#deploy)。

<a id="keep-operator-takeover-available"></a>

## 保留操作员接管

`enable_teleop` 默认 true。激活版本化任务或提供 `--teleop-device` 时，部署会组合遥操作输入源。要保留教程的 GELLO 和键盘，使用含 `teleop:` 的工作站 YAML，不传 `--no-enable-teleop`，并指定 `--teleop-device leader_arm --teleop-event-device stdin`；其余介入约定从已保存设置恢复。任务运行可保留已保存的操作员配置。测量自主能力时让策略掌握动作控制，需要时接管并将该回合报告为辅助回合。只按成功、失败标注键不算动作介入。终端事件源中，`t` 接管、`h` 交还、`s`/`f` 标结果、Esc 结束回合、`q` 退出。

分类器移动后可用 `--classifier-checkpoint` 替换。逐臂任务分类器路径也可在任务配置中替换，但任务、控制、预处理、观测布局和部件选择仍须一致。奖励来源属于已记录来源元数据，因此更换分类器内容会留下记录，同时保持动作和观测约定。

<a id="read-the-deployment-report"></a>

## 阅读部署报告

*预期结果*：逐回合输出 `[deploy] episode=... steps=... return=... truncated=... rtt_ms=... health_ok=True`，最终 `[deploy] summary ...` 包含健康与时间信息，以及 `success_rate`、`autonomous_success_rate`、`task_success_rate`、介入率和 `unknown_episodes`。主要成功率以全部已完成回合为分母，仅将未接管成功计入分子；辅助成功计入任务成功率。未知结果保留在已完成分母中，中断、重录或丢弃尝试单独统计。

教程默认任务没有自动成功检测器。关闭接管且未使用分类器时，回报为零、结果未知，需要观察机器人判断尝试。分类器或版本化任务可提供奖励和显式结果。正奖励或 `terminated=True` 本身不能确认任务成功，人工失败标签会否决成功。

<a id="if-not"></a>

## 结果不符时

设备故障会向 stderr 输出 `[deploy] device error: ...` 和 `[deploy] hint: ...`，以退出码 2 结束。动作或实验约定不符在 reset 运动前抛 `ValueError`，应使用记录设置，只覆盖允许的连接和来源字段。拔掉 GELLO 后守护进程无法连接，说明 YAML 仍含 `teleop:`。正常或异常清理关闭环境和基础提供器，强制终止仍依赖 watchdog。

参考：[部署](../project-readme.md#deploy)与[真机实验](../real-world-experiments.md)。
