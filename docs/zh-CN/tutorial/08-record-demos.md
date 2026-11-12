<!-- English source: docs/tutorial/08-record-demos.md -->

<a id="8-record-demos"></a>

# 8. 录制示范

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/08-record-demos.md)

**目标：** 录制成功人工示范，训练基础策略或 HIL learner 前检查归档。录制器构建与 HIL actor 相同的环境，将每次尝试归档至 SQLite，导出成功人工监督步到示范 pkl。

<a id="prepare-the-nuc"></a>

<a id="before-you-start"></a>

## 开始前

除非另有说明，所有命令在 **NUC** 的**仓库根目录**、**激活项目环境**后运行。

- 完成 [03 启动 Franka](03-franka-bringup.md)、[04 Home、夹爪、相机](04-first-motion.md)。无其他进程占用机械臂。
- 按 [05 遥操作：GELLO](05-teleop-gello.md) 调试主臂及选项文件。
- 阅读[操作员页面](../intervening.md)。保持工作区畅通，物理停止装置可及。reset 可能移动机械臂并打开夹爪。
- 在本地预设检查实际臂 FCI IP、Hand、home、关节限制和安全盒。无 Hand 时设置 `real_robot.robot.gripper: null`、`real_robot.gripper_in_action: false`。

`rwrl record` 是硬件进程；`rwrl supervise` 将其作为子进程启动并监视心跳。下方每条录制命令均需包装，使静默或卡住的子进程触发 SIGINT → SIGTERM → SIGKILL。心跳超时默认 **2.0 s**；最终强制终止会失去 libfranka 连接，依赖固件停止。监督器无法独立向机器人发送停止。录制命令同时启用 **0.3 s** 进程内 StepWatchdog，在进程仍可运行时检测缺失步。终端键传至子进程；stdin 无需桌面会话或 evdev 键盘权限。见[监督](../daemon.md#supervising-a-single-process-robot-run)。

| 选项 | 本例值 | 含义 |
|---|---|---|
| `--config` | `local/gello_cartesian.yaml` | 已调试环境与遥操作预设 |
| `--teleop-device` | `leader_cartesian` | 显式运动源；无默认值 |
| `--teleop-event-device` | `stdin` | 此终端的操作员键 |
| `--real_robot.watchdog_s` | `0.3` | 进程内步超时，秒 |
| `--output-path` | `demos/pick-and-place.pkl` | 新 pkl 路径；拒绝已有文件 |
| `--num-successes` | `20` (also the default) | 保留这么多回合后停止 |
| `--teleop-lead-cap-m` / `--teleop-lead-cap-rad` | `0.05` / `0.3` | 领先实测 TCP 的最大幅度 |
| `--teleop-target-speed-m-s` / `--teleop-target-speed-rad-s` | `0.05` / `0.3` | 最大目标速度 |

<a id="1-prepare-the-local-preset-and-home"></a>

## 1. 准备本地预设和 home

若 [04](04-first-motion.md#1-home) 已创建并调试此副本，保留它。否则在 NUC 仓库根目录，无需环境，复制示例一次：

<!-- tutorial-test: skip -->
```bash
cp examples/presets/gello_cartesian.yaml local/gello_cartesian.yaml
```

*应看到：* `local/gello_cartesian.yaml`。初始为纯状态观测、`cartesian_delta`、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`。补全本地硬件/主臂设置，按 [04](04-first-motion.md#1-home) 保存并验证 home。再次复制会覆盖已保存 home。home 或设备检查失败时，录制前先按[失败怎么办](#if-it-fails)处理。

<a id="2-start-the-gello-recorder"></a>

## 2. 启动 GELLO 录制器

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl supervise --log runs/record-pick-and-place/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

*应看到：* reset 后保持固定姿态，等待 `t`。启动失败则结束录制器，按[失败怎么办](#if-it-fails)处理。

环境尺度至少等于领先上限。预设 3000 步回合上限在 50 Hz 为 60 秒；录制时可用 `--max-episode-steps` 覆盖。

<a id="3-record-one-episode"></a>

## 3. 录制一个回合

1. **Reset：** 等待回 home 和夹爪打开完成。无人驱动时录制器保持固定姿态。
2. **`t`：** 接管。当前主臂姿态成为锚点，位移映射到从臂命令姿态。笛卡尔路径无需逐关节匹配。
3. **驱动：** 移动 GELLO 接近、搬运、放置物体。命令按配置速度前进，保持在实测 TCP 的领先上限内；超出领先量被丢弃，不排队。
4. **扳机：** 压下关闭、释放打开。接管时若扳机与夹爪末次命令不符，将被忽略，直到跨越到该状态，防止立即意外打开。
5. **`h`：** 适时交还。录制器采样实测姿态并固定保持。按 `t` 重新锚定恢复。
6. **`s`：** 标成功并结束回合。要让成功终止步进入人工示范 pkl，须在介入时按 `s`。若已按 `h`，标记前重新接管；锚定不会产生跳变。

`f` 丢弃尝试，`r` 丢弃并开始下一次，Esc 无成功标签结束回合。`q` 结束当前回合，处理结果后停止；Ctrl-C 也会停止并保存已保留回合。默认录制器按键奖励使 `s` 以奖励 1 终止。交还后标记仍结束回合，但终止步不会出现在仅人工 pkl 视图。

*应看到：* 回合结果和最终保存行，例如：

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

`WORKSPACE HOLD: <axes>` 表示请求目标越出安全盒。源交还并保持实测姿态；清除请求，按 `h` 再按 `t` 重新锚定。这不算成功步，也不表示可以扩大安全盒。

<a id="4-alternative-record-with-a-spacemouse"></a>

## 4. 替代：用 SpaceMouse 录制

使用相同 NUC、仓库根目录和项目环境。按 [teleop.md](../teleop.md#spacemouse) 安装 SpaceMouse 扩展。覆盖预设 GELLO 选项文件，避免将主臂选项传给 SpaceMouse 源：

```bash
rwrl supervise --log runs/record-spacemouse/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device spacemouse --teleop-event-device stdin --teleop-options-yaml None \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place-spacemouse.pkl --num-successes 20
```

*应看到：* 与 GELLO 相同的回合结果及保存行，输出为 SpaceMouse 路径。HID 读取失败见[失败怎么办](#if-it-fails)。

按钮 **0** 接管，再次按下交还。只推控制帽不会接管。驱动时控制帽请求速度，最大偏转为 0.05 m/s、0.3 rad/s。释放移除速度请求；机械臂向现有命令收敛，最多相距一个领先上限。按钮 **1** 仅在驱动时切换夹爪。接管保留末次夹爪命令。终端 `s`/`f`/`r`/Esc/`q` 用于结果及退出；此路径忽略 `t`/`h`。工作区 HOLD 后须重新按按钮 0 接管。

原生 HIL-SERL SpaceMouse 行为：在 **NUC** 的**仓库根目录**、**激活项目环境**，向 SpaceMouse 录制命令添加 `--spacemouse-mode serl`。配置站点 xyz **和 RPY** 安全盒；模式选择并保存 `real_robot.cartesian_semantics: serl`。六轴范数 `> 0.001` 的控制帽运动立即接管；释放控制帽和按钮交还。按住按钮 0 关闭、按钮 1 打开（左键优先）；均未按下发送中立零。忽略 `t`/`h`。介入时按 `s` 才将成功终止步纳入人工 pkl；空闲步仍用保持策略。训练/部署保持相同 SERL 模式：契约为 `spacemouse_serl_v1`，tracker 上限/速度不适用。见[两种模式](../teleop.md#spacemouse)。

倾斜基座或操作员特定方向可用 [SpaceMouse CLI 流程](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)调试站点任务坐标系和命名配置。受监督 `rwrl teleop run` 使用这些设置，标签后保持最后接受目标；n 请求下一轨迹，q 释放控制。上述普通录制器默认自动处理回合，除非选择 `--episode-end-behavior hold`。确认方向映射或正常停止行为已验证前，执行[验收清单](../testing/spacemouse-control-session.md)。

<a id="files-and-compatibility"></a>

<a id="5-inspect-the-files-and-compatibility"></a>

## 5. 检查文件与兼容性

成功运行生成 `demos/pick-and-place.pkl`、`demos/pick-and-place.contract.json`、`demos/pick-and-place.trajectory.sqlite3`。归档包含每个提议/执行动作、掩码和结果，包括失败、重录、中断尝试。pkl 选择成功人工步；无保留回合则无 pkl。新数据集使用新 `.pkl` 路径。`--export-trajectory-pt` 另写可移植 `.trajectory.pt` 数据。

伴随文件应与 pkl 一起保存。其包含版本化实验契约，包括 `teleop_control`（默认 tracker 路径为 `cartesian_tracker_v1`、领先上限及目标速度；SERL 为 `spacemouse_serl_v1`）。本教程要求录制、训练、部署使用**同一遥操作设备及全部四个 tracker 值**。缺失或改变 tracker 设置时拒绝，差异指向 `experiment.teleop_control`。

**当前 HEAD 差异：** tracker 契约标识 tracker 而非实际输入设备。实现中相同 tracker 值的 GELLO 与 SpaceMouse 可共享数据。要求同设备是本教程流程规则；当前契约不拒绝仅设备不同。

在 NUC 的仓库根目录，激活项目环境（命令不打开硬件）：

```bash
rwrl archive inspect demos/pick-and-place.trajectory.sqlite3
```

*应看到：* 含 `records`、`episode_starts`、`episode_ends`、`transitions`、`human_mask_counts`、`outcomes` 及逐回合测量的 JSON。源只读。文件缺失时检查输出主干路径及录制是否进入 reset。

<a id="6-optional-joint-space-recording"></a>

## 6. 可选：关节空间录制

<details>
<summary>其他模式：关节空间 leader_arm 和 joint_abs（非主线）</summary>

这些模式需要独立关节空间预设及另行录制示范。`leader_arm` 的 `--teleop-onset arm` 等待主臂进入 `--teleop-arm-tolerance-rad`（0.1 rad）后接管。默认 `ramp` 立即开始：`joint_abs` 每步以 `--teleop-ramp-step-rad`（0.05）限制初始目标；`joint_delta` 标记初始追赶，直到主臂进入一个 `joint_scale_rad` 步长内。`teleop_syncing` 仅适用这些关节空间主臂路径，不适用笛卡尔主线。

在 NUC 的仓库根目录，激活项目环境，使用单独调试的 `hil_abs.yaml`（`joint_abs`、关节限制、本地机器人和 GELLO 遥操作部件）：

```bash
rwrl supervise --log runs/record-joint-abs/supervise.jsonl -- \
    rwrl record --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device stdin \
    --output-path demos/pick-and-place-abs.pkl --num-successes 20
```

`joint_abs` 时，任一主臂关节距从臂超过 `--start-tolerance-rad`（0.8），录制器拒绝开始回合。先对齐主臂。pkl 通常省略同步行；`--keep-syncing-steps` 将其保留为普通监督步。归档派生训练视图仍排除同步监督。关节空间夹爪接管没有笛卡尔锁存；接管前匹配扳机与当前夹爪状态。

*应看到：* 回合结果及 `demos/pick-and-place-abs.pkl` 保存行。初始间距被拒绝时，先对齐再试；不要提高容差绕过检查。

</details>

<a id="7-continue-after-an-interruption"></a>

## 7. 中断后继续

录制期间归档持续提交。pkl 尚不存在时重新打开输出主干会保留归档，将未完成尝试标为中断，新尝试使用新 ID。pkl 仅导出本次调用保留成功；之前完整成功用归档读取。持久性从 append 返回开始，之前动作可能丢失。SQLite 进程崩溃及断电边界见[真实世界实验](../real-world-experiments.md#trajectory-archives-and-training-views)。重启停止的进程前阅读监督器日志；再次 reset 前检查工位并清除机器人故障。

仅在 pkl 不存在时重跑对应录制命令。已保存 pkl 则选择新输出路径；没有 pkl 追加选项。

*应看到：* 现有归档中的新尝试 ID，保留成功后出现新保存行。重启失败时检查监督器日志及下方检查项。

<a id="if-it-fails"></a>

## 失败怎么办

在 NUC 的仓库根目录，激活项目环境，先结束硬件进程：

```bash
rwrl --station local/station.yaml doctor
```

从首个红色行和提示开始。见[网络与 FCI](../doctor.md#network-and-fci)、[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)、[SpaceMouse 与 HID](../doctor.md#spacemouse-and-hid)、[相机](../doctor.md#cameras)或[设备占用](../doctor.md#busy-devices-blocked)。

- 任务超时：检查 60 秒上限；只根据合理任务时长提高 `--max-episode-steps`。
- `t` 无效：检查终端焦点、GELLO 连接/校准及是否需先 `h` 后 `t` 解除工作区 HOLD。SpaceMouse 改用按钮 0。
- 设备读取失败：停止，检查串口/HID 权限和硬件，再在无硬件进程占用站点时重跑 doctor。
- 契约不匹配：使用录制时的预设、设备和 tracker 值，不修改数据伴随文件。

更多说明：[recording-demos.md](../recording-demos.md)。
下一步：[09 基础策略](09-base-policy.md)。
