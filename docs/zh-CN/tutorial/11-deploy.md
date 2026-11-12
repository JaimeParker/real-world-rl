<!-- English source: docs/tutorial/11-deploy.md -->

<a id="11-deploy"></a>

# 11. 部署

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/11-deploy.md)

**目标：** 无 learner、无训练数据上传地运行保存的检查点，阅读结果及介入报告。部署恢复 `config.json`、`experiment.json` 重建策略、包装器及任务设置。独立 `.pt` 直接加载；`bundle_manifest.json` 解析到该代模型，不恢复回放。

<a id="before-you-start"></a>

## 开始前

除非另有说明，命令在 **NUC** 的**仓库根目录**、**激活项目环境**后运行。

- 完成 [09 基础策略](09-base-policy.md) 或 [10 HIL 训练](10-hil-training.md)。将检查点、`config.json`、`experiment.json` 复制到下述路径；manifest 还需引用的 bundle 代。残差运行还需基础检查点。
- 停止录制器或 actor。本教程恢复保存的直连 `real_robot.robot`，无需守护进程。保持工作区畅通、物理停止装置可及。
- 在此工位完成 [03](03-franka-bringup.md)、[04](04-first-motion.md)。reset 运动前检查保存的 home、限制、相机设置及动作/观测契约。
- 阅读[操作员页面](../intervening.md)。无人值守部署从连接所用机器人映射移除 `teleop:`：即使禁用接管，已配置设备仍必需。保留机器人运动及观测设置。

<a id="1-choose-the-checkpoint-and-algorithm"></a>

## 1. 选择检查点与算法

| 运行 | `--algo` | 检查点 |
|---|---|---|
| BC | `real_world_bc` | `runs/bc-pick-and-place/checkpoints/final.pt` |
| 使用教程夹爪的 `hil_serl` | `rlpd_hybrid` | `runs/hil-pick-and-place/checkpoints/bundle_manifest.json`, `final.pt` or `checkpoint_<update>.pt` |
| `residual_hil_serl` | `hybrid` 用 `residual_hil_serl_hybrid`，`continuous` 用 `residual_hil_serl` | 对应残差运行的 manifest 或 `.pt`；元数据提供基础提供器 |
| `silri` | `silri` | `runs/silri-pick-and-place/checkpoints/bundle_manifest.json` or `.pt` |

部署读取保存的训练配置，不接受训练 `--config`。`--config-path` 定位移动后的 `config.json`。保留 [08](08-record-demos.md)、[10](10-hil-training.md) 使用的 `local/gello_cartesian.yaml` 有效设置。ACT 通过残差检查点运行。

<a id="2-run-without-operator-takeover"></a>

## 2. 无操作员接管运行

在 NUC 仓库根目录，激活项目环境：

```bash
rwrl supervise --startup-s 60 --log runs/bc-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/bc-pick-and-place/checkpoints/final.pt \
    --algo real_world_bc --no-enable-teleop --episodes 5 --max-steps-per-episode 200
```

*应看到：* 每个完成回合一行 `[deploy] episode=... steps=... return=... truncated=... rtt_ms=...`，最后 `[deploy] summary ...`。除运动外还需检查健康和时序。教程 `task.name: none` 无自动成功检测；无接管或分类器时奖励为零、结果未知。启动或设备失败见[失败怎么办](#if-it-fails)。

ACT 基础残差 bundle，在同一 NUC 仓库根目录，激活项目环境：

```bash
rwrl supervise --startup-s 60 --log runs/residual-act-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/residual-act-pick-and-place/checkpoints/bundle_manifest.json \
    --algo residual_hil_serl_hybrid --no-enable-teleop --episodes 5
```

*应看到：* 相同回合及摘要报告。基础路径移动后提供实际 `--base-provider 'act:/data/act/final.pt?temporal_agg=1'`。契约不匹配需恢复原动作/观测设置，不编辑数据伴随文件。

`--device cuda` 选择 GPU 推理（默认 `cpu`）。监督器心跳超时默认 2.0 s；`--startup-s 60` 为首次心跳前策略加载留时间。清理卡住时升级信号；最终强制终止依赖固件处理连接丢失（[监督](../daemon.md#supervising-a-single-process-robot-run)）。

<a id="3-alternative-keep-operator-takeover-available"></a>

## 3. 替代：保留操作员接管

使用 [05](05-teleop-gello.md) 已调试 GELLO 设置。`--teleop-device` 无默认值，同 [10](10-hil-training.md#1-set-the-shared-flags)：部署构建介入源时需显式设备或保存的预设值。`enable_teleop` 默认 true；版本化任务或显式 `--teleop-device` 激活源组合。`--no-enable-teleop` 不构建介入源。

在 NUC 仓库根目录，激活项目环境，使用与录制相同主臂选项和 tracker 契约：

```bash
rwrl supervise --startup-s 60 --log runs/hil-pick-and-place/deploy-supervise.jsonl -- \
    rwrl deploy --checkpoint runs/hil-pick-and-place/checkpoints/final.pt \
    --algo rlpd_hybrid --teleop-device leader_cartesian --teleop-event-device stdin --episodes 5
```

*应看到：* 含介入次数的回合报告及最终辅助/自主比率。`t` 接管、`h` 交还、`s`/`f` 标结果、Esc 结束回合、`q` 退出。主臂读取失败则结束部署，按[失败怎么办](#if-it-fails)处理。保持录制时 tracker 上限、速度、设备。匹配 SpaceMouse 训练运行用 `spacemouse`；保留 [10](10-hil-training.md#1-set-the-shared-flags) 的 tracker/SERL 模式及数值。

自主测量应将动作控制留给策略；接管使该回合变为辅助。仅成功/失败标签不算动作介入。移动的分类器可用 `--classifier-checkpoint` 提供；逐臂任务分类器可在任务配置替换。替代分类器内容属于来源信息；任务、控制、预处理、观测布局和部件选择仍须匹配。

<a id="4-read-the-report-and-stop"></a>

## 4. 阅读报告并停止

最终摘要含健康/时序、`success_rate`、`autonomous_success_rate`、`task_success_rate`、介入率、`unknown_episodes`。主要比率以无辅助成功回合除以所有完成回合；辅助成功计入任务成功。未知结果仍在分母中。中断或重录/丢弃尝试单独报告。仅正奖励或 `terminated=True` 不能确认成功；人工失败标签否决成功。

*应看到：* 达到回合上限或正常退出后出现摘要。监督器终端 Ctrl-C 请求清理；正常和异常清理均关闭环境及基础提供器。清理卡住时，重启前检查监督器日志和实际工位。

<a id="optional-deploy-through-a-daemon"></a>

## 可选：通过守护进程部署

异机客户端按[通过机器人守护进程运行远程 actor](../remote-actor.md)配置站点、守护进程、传输，再提供 deploy 的 `--real_robot.remote ROBOT_PC:50051` 及必要的 `--real_robot.token T`。无人值守守护进程需无 `teleop:` 站点；先停止直接硬件占用者。JPEG 设置与训练相同（教程用原始帧）。双臂运行选择 `--env-backend real_robot_bimanual`，逐臂设置见 [bimanual.md](../bimanual.md#deploy)。

<a id="if-it-fails"></a>

## 失败怎么办

在 NUC 仓库根目录，激活项目环境，先结束部署：

```bash
rwrl --station local/station.yaml doctor
```

从首个红色行及提示开始。见[网络与 FCI](../doctor.md#network-and-fci)、[SDK 与 libfranka 版本](../doctor.md#sdk-and-libfranka-version)、[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)、[相机](../doctor.md#cameras)或[设备占用](../doctor.md#busy-devices-blocked)。

设备故障向 stderr 打印 `[deploy] device error: ...`、`[deploy] hint: ...`，退出码 1；无效参数退出码 2。契约不匹配在 reset 前抛 `ValueError`：恢复录制设置，仅覆盖支持的连接/来源字段。仍要求已拔掉 GELLO 的守护进程在提供含 `teleop:` 的站点。

下一步：[真实世界实验](../real-world-experiments.md)了解契约和恢复，或 [00 总览](00-overview.md)回顾完整流程。
