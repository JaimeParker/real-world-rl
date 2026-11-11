<!-- English source: docs/tutorial/04-record-demos.md -->

<a id="4-record-demos"></a>

# 4. 录制示范

`examples/record_real_demos.py` 构建与 HIL actor 相同的环境。它将所有尝试归档到 SQLite，并将成功回合的人工监督步导出为供 BC、ACT 和 HIL 使用的示范 pkl。先阅读 [06 人工介入](06-intervening.md)。

<a id="prepare-the-nuc"></a>

## 准备 NUC

本页所有命令均在 **NUC** 的**仓库根目录**、**项目环境已激活**时运行。先完成工作站配置和真实 doctor 检查。保持工作区清空，物理停止装置可触及。reset 可能移动机械臂和打开夹爪，因此事先检查实臂 FCI IP、Hand 是否存在、home、关节限位和工作区。

主要预设为 [`examples/presets/gello_cartesian.yaml`](../../../examples/presets/gello_cartesian.yaml)：仅状态、`cartesian_delta`、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`。其 robot 映射通过本 NUC 的 franky 直连。运行前填入工位实际值：示例工作区不代表工位验收。没有实体 Hand 时，在 robot 映射设 `gripper: null`，环境设 `gripper_in_action: false`。

预设的 `teleop_options_yaml` 指向仓库根目录的 `gello_leader.yaml`。创建该本地选项文件，内容为 `robot_yaml: gello_station.yaml`。`gello_station.yaml` 应为本地仅 leader 的机器人 YAML：`type: franka`、独立 `name`、`arm_backend: fake`、`gripper: null`、`cameras: []`，以及带串口和标定的已调试 GELLO `teleop:` 段。不要将此输入源指向实体 follower 工作站：它连接整个 leader 机器人 YAML，会向 follower 打开第二个连接。标定和其它本地 leader 配置见 [gello.md](../gello.md)。

录制前，在本 NUC 的仓库根目录激活项目环境，结束所有直接持有机械臂的进程。运行 `rwrl status`，手动引导机械臂到期望 home，然后退出引导模式。在下文录制器使用的同一预设中保存并验证 home：

<!-- tutorial-test: skip -->
```bash
rwrl --preset examples/presets/gello_cartesian.yaml home set
rwrl --preset examples/presets/gello_cartesian.yaml --yes home go
```

这与 `env.reset()` 解析的是同一 home。验证会移动机械臂并应用 `task.reset.gripper`，但跳过其它 reset 阶段。`home set` 用 PyYAML 重写此预设：注释会丢失，旧文件保存为 `examples/presets/gello_cartesian.yaml.bak`。见 [rwrl home](../cli.md#home-resolve-save-and-move)。

<a id="record-with-gello"></a>

## 用 GELLO 录制

在 NUC 的仓库根目录，激活项目环境后运行：

```bash
rwrl supervise --log runs/record-pick-and-place/supervise.jsonl -- \
    rwrl record --config examples/presets/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

supervisor 将硬件进程作为子进程运行。终端按键传给子进程；不需要桌面会话或 evdev 键盘权限。显式 StepWatchdog 设置为 **0.3 s**：当前 HEAD 的预设尚未包含它。它在进程内检查缺失控制步。独立的 supervisor 检测心跳缺失（默认 **2.0 s**），清理停滞时逐级升级信号。最终 kill 会断开 libfranka 连接，依赖固件反射停止；不能另发机器人 stop 命令。见[监督机制](../robot-infra.md#supervising-a-single-process-robot-run)。

预设已经设置以下 tracker 参数；整个流程中保持不变：

| 参数 | 值 | 含义 |
|---|---|---|
| `--teleop-lead-cap-m` | `0.05` | 相对实测 TCP 的最大平移领先量，米 |
| `--teleop-lead-cap-rad` | `0.3` | 最大旋转领先量，弧度 |
| `--teleop-target-speed-m-s` | `0.05` | 最大命令平移速度，m/s |
| `--teleop-target-speed-rad-s` | `0.3` | 最大命令旋转速度，rad/s |

环境尺度必须至少等于领先上限。预设的 3000 步回合上限在 50 Hz 下为 60 秒；录制时可用 `--max-episode-steps` 覆盖。

<a id="an-episode"></a>

## 一个回合

1. **Reset：**等待回 home 和夹爪打开完成。无人驱动时，录制器保持固定位姿。
2. **`t`：**接管。当前 leader 位姿成为锚点，相对它的位移映射到 follower 命令位姿。此笛卡尔路线不要求逐关节姿态匹配。
3. **驱动：**移动 GELLO 去接近、搬运和放置物体。命令按配置速度推进，并保持在实测 TCP 的领先上限内；多余领先量丢弃，不排队。
4. **扳机：**捏合关闭，释放打开。接管时若扳机与夹爪最后命令状态不一致，在扳机跨到该状态前忽略它，避免立即意外张开。
5. **`h`：**适时交还。录制器采样实测位姿并固定保持。按 `t` 重新锚定并继续。
6. **`s`：**标记成功并结束回合。要让成功终止步进入人工示范 pkl，需在介入时按 `s`。若已按 `h`，先再次接管再标记；重新锚定不会跳变。

`f` 丢弃尝试，`r` 丢弃并开始下一次，Esc 不带成功标签地结束回合。`q` 结束当前回合，处理结果后停止；Ctrl-C 也停止并保存已保留回合。默认录制器按键奖励使 `s` 成为奖励 1 的终止步。交还后打标签仍结束回合，但终止步不在纯人工 pkl 视图中。

*应看到：*回合结果和最终保存行，例如：

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

`WORKSPACE HOLD: <axes>` 表示请求目标离开安全盒。输入源交还并保持实测位姿；清除请求，按 `h` 再按 `t` 重新锚定。这不是成功步，也不表示允许扩大安全盒。

<a id="record-with-a-spacemouse"></a>

## 用 SpaceMouse 录制

使用同一 NUC、仓库根目录和项目环境。按 [teleop.md](../teleop.md#spacemouse) 安装 SpaceMouse extra。覆盖预设的 GELLO 选项文件，避免将 leader 选项传给 SpaceMouse 输入源：

```bash
rwrl supervise --log runs/record-spacemouse/supervise.jsonl -- \
    rwrl record --config examples/presets/gello_cartesian.yaml \
    --teleop-device spacemouse --teleop-event-device stdin --teleop-options-yaml None \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place-spacemouse.pkl --num-successes 20
```

按钮 **0** 接管，再按交还。单纯推动手柄不算接管。驱动时，手柄请求速度，满幅时最高 0.05 m/s 和 0.3 rad/s。松开手柄移除速度请求；机械臂在现有命令目标处稳定，继续移动至多一个领先上限。按钮 **1** 仅在驱动时切换夹爪。接管保留最后夹爪命令。终端 `s`/`f`/`r`/Esc/`q` 用于结果及退出；此路线忽略终端 `t`/`h`。工作区保持状态需要再次用按钮 0 接管。

若需原生 HIL-SERL SpaceMouse 行为，在 **NUC** 的**仓库根目录**、**项目环境激活**时，向 SpaceMouse 录制命令添加 `--spacemouse-mode serl`。配置工作站 xyz **和 RPY** 安全盒；该模式选择并保存 `real_robot.cartesian_semantics: serl`。手柄运动的六轴范数 `> 0.001` 时立即接管；释放手柄和按钮后交还。按住按钮 0 关闭，按钮 1 打开（左键优先）；两者均未按下时发送中立零值。`t`/`h` 被忽略。介入时按 `s`，使成功终止步进入人工示范 pkl；录制器空闲步仍使用其保持策略。训练/部署中保持相同 SERL 模式：其契约为 `spacemouse_serl_v1`，tracker 上限/速度不适用。见[两种模式](../teleop.md#spacemouse)。

对于倾斜基座或操作员特定的手柄方向，通过 [SpaceMouse CLI 流程](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace) 调试工作站任务坐标系（task frame）和具名标定档案（profile）。受监督的 `rwrl teleop run` 使用这些设置，并在标签后保持最后接受的目标；n 请求下一轨迹，q 释放控制。上述普通录制器命令默认仍自动处理回合，除非选择 `--episode-end-behavior hold`。在认定方向映射或正常停止行为已验证前，使用[验收清单](../testing/spacemouse-control-session.md)。

<a id="files-and-compatibility"></a>

## 文件与兼容性

成功运行产生 `demos/pick-and-place.pkl`、`demos/pick-and-place.contract.json` 和 `demos/pick-and-place.trajectory.sqlite3`。归档包含每个提议/执行动作、掩码和结果，包括失败、重录和中断尝试。pkl 选择成功人工步；没有保留回合就没有 pkl。新数据集使用新的 `.pkl` 输出路径。`--export-trajectory-pt` 额外写出可移植 `.trajectory.pt` 数据。

将附属文件与 pkl 一起保存。它包含版本化实验契约，包括 `teleop_control`（默认 tracker 路线为 `cartesian_tracker_v1`、领先上限和目标速度；SERL 模式为 `spacemouse_serl_v1`）。本教程要求录制、训练和部署使用**同一遥操作设备和全部四个 tracker 值**。tracker 设置缺失或变化时会拒绝，并指出 `experiment.teleop_control` 差异。

**当前 HEAD 差异：**tracker 契约描述 tracker，而非物理输入设备。实现允许 tracker 值相同的 GELLO 和 SpaceMouse 共享数据。要求相同设备是本教程的流程规则；该契约目前不会拒绝仅设备不同的情况。

*若不符合预期：*

- 任务超时：检查 60 秒上限；仅为合适的任务时长增加 `--max-episode-steps`。
- `t` 无反应：检查终端焦点、GELLO 连接/标定，以及工作区保持状态是否需要先 `h` 后 `t`。SpaceMouse 应使用按钮 0。
- 设备读取失败：停止，检查串口/HID 权限和硬件，再在无硬件进程占用工作站时重跑 doctor。
- 出现契约不匹配：使用录制时的预设、设备和 tracker 值，不要编辑数据附属文件。

<details>
<summary>其它模式：关节空间 leader_arm 和 joint_abs（非主流程）</summary>

这些模式需要独立关节空间预设和单独录制的示范。`leader_arm` 的 `--teleop-onset arm` 等待 leader 进入 `--teleop-arm-tolerance-rad`（0.1 rad）后才接管。默认 `ramp` 立即开始：`joint_abs` 将初始目标限制为每步 `--teleop-ramp-step-rad`（0.05）；`joint_delta` 标记初始追赶，直到 leader 在一个 `joint_scale_rad` 步长内。`teleop_syncing` 只适用于这些关节空间 leader 路线，不适用于笛卡尔主流程。

在 NUC 的仓库根目录激活项目环境，使用单独调试的 `hil_abs.yaml`（`joint_abs`、关节限位、本地机器人及 GELLO teleop 部件）：

```bash
rwrl supervise --log runs/record-joint-abs/supervise.jsonl -- \
    rwrl record --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device stdin \
    --output-path demos/pick-and-place-abs.pkl --num-successes 20
```

`joint_abs` 中，任一 leader 关节距 follower 超过 `--start-tolerance-rad`（0.8）时录制器拒绝开始回合。先移动 leader 对齐。pkl 通常省略 syncing 行；`--keep-syncing-steps` 将其保留为普通监督步。归档派生训练视图仍排除 syncing 监督。关节空间夹爪接管没有笛卡尔锁存机制；接管前应将扳机与当前夹爪状态匹配。

</details>

<a id="continue-after-an-interruption"></a>

## 中断后继续

录制期间归档持续提交。在 pkl 尚不存在时重新打开同一输出前缀，会保留归档并将未完成尝试标为中断；新尝试分配新 ID。pkl 导出覆盖本次调用保留的成功回合；更早的完整成功回合应使用归档。持久性从 append 返回时开始，因此 append 前的动作可能丢失。SQLite 进程崩溃与断电边界见[真实机器人实验](../real-world-experiments.md#trajectory-archives-and-training-views)。重启已停止进程前阅读 supervisor 日志；再次 reset 前核实工位并清除机器人故障。

更多细节：[recording-demos.md](../recording-demos.md)。下一步：[05 基础策略](05-base-policy.md)。
