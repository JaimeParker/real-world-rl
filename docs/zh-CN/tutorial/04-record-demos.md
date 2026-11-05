<!-- English source: docs/tutorial/04-record-demos.md -->

<a id="4-record-demos"></a>

# 4. 录制示范

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/04-record-demos.md)

`examples/record_real_demos.py` 与 HIL actor 构建同一环境，将全部尝试归档到 SQLite，再把成功回合的人工监督步骤导出给 BC、ACT、HIL 使用。先读 [06 人工介入](06-intervening.md)。

<a id="prepare-the-nuc"></a>

## 准备 NUC

本页全部命令在 **NUC、仓库根、项目环境（project environment）激活**时运行。先完成 station 与真实 doctor，清空工作区、保持物理停止装置可用。reset 可能回 home 并开夹爪，之前核对实臂 FCI IP、Hand、home、关节和工作空间限制。

主预设为 [`examples/presets/gello_cartesian.yaml`](../../../examples/presets/gello_cartesian.yaml)：仅状态、`cartesian_delta`、50 Hz、`xyz_scale_m: 0.05`、`rot_scale_rad: 0.3`，其 robot 映射在 NUC 直接连接 franky。先填入现场值，示例工作空间不等于工位批准。无实体 Hand 时在 robot 映射设 `gripper: null`，环境设 `gripper_in_action: false`。

`teleop_options_yaml` 指向仓库根的 `gello_leader.yaml`。在该本地选项文件写 `robot_yaml: gello_station.yaml`。后者须是只描述 leader 的本地机器人 YAML：`type: franka`、独立 `name`、`arm_backend: fake`、`gripper: null`、`cameras: []`，加上已配置的 GELLO `teleop:`（串口与标定）。不要指向实臂 follower station，因为该输入源会连接整份 leader YAML，导致重复连接实臂。标定和其他本地 leader 配置见 [gello.md](../gello.md)。

录制前，在同一 NUC、仓库根、项目环境激活时，先结束直接占用机械臂的进程。运行 `rwrl status`，手动拖到期望 home 后退出 guiding 模式；在下方 recorder 使用的同一预设中保存并验证 home：

<!-- tutorial-test: skip -->
```bash
rwrl --preset examples/presets/gello_cartesian.yaml home set
rwrl --preset examples/presets/gello_cartesian.yaml --yes home go
```

这是 `env.reset()` 会解析到的同一个 home。验证会移动机械臂并执行 `task.reset.gripper`，跳过其他 reset 阶段。`home set` 用 PyYAML 重写此预设，注释会丢失，原文件保存为 `examples/presets/gello_cartesian.yaml.bak`。见 [rwrl home](../cli.md#home-resolve-save-and-move)。

<a id="record-with-gello"></a>

## 用 GELLO 录制

NUC、仓库根、项目环境激活：

```bash
python -m robot_infra.supervise --log runs/record-pick-and-place/supervise.jsonl -- \
    python examples/record_real_demos.py --config examples/presets/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-event-device stdin \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place.pkl --num-successes 20
```

supervisor 将硬件进程作为子进程启动，终端键送给子进程，无需桌面会话或 evdev 键盘权限。显式 StepWatchdog 为 **0.3 s**，当前 HEAD 预设尚未包含它；它在进程内检查缺失的 step。独立 supervisor 默认 **2.0 s** 心跳静默后升级信号，清理卡住时最终杀进程，依赖 libfranka 断连触发固件 reflex 停止，不能另发机器人 stop。见[监督说明](../robot-infra.md#supervising-a-single-process-robot-run)。

预设已设置以下 tracker 参数，整条流水线须一致：

| 参数 | 值 | 含义 |
|---|---|---|
| `--teleop-lead-cap-m` | `0.05` | 命令领先实测 TCP 的最大平移，米 |
| `--teleop-lead-cap-rad` | `0.3` | 最大旋转领先，弧度 |
| `--teleop-target-speed-m-s` | `0.05` | 命令最大平移速度，m/s |
| `--teleop-target-speed-rad-s` | `0.3` | 命令最大旋转速度，rad/s |

环境尺度须至少等于领先上限。3000 步在 50 Hz 为 60 秒，录制可用 `--max-episode-steps` 覆盖。

<a id="an-episode"></a>

## 一个回合

1. **reset：**等待 home 和开夹爪结束。无人驱动时，录制器保持固定姿态。
2. **`t`：**接管，以当前 leader 姿态为锚，相对位移映射到 follower 命令姿态；此笛卡尔路线无需逐关节对齐。
3. **驱动：**移动 GELLO 完成接近、搬运和放置。命令按配置速度推进，领先实测 TCP 不超过上限，多余部分丢弃而不排队。
4. **扳机：**握紧闭合、释放张开。接管时扳机与夹爪最后命令不一致，会忽略扳机直到跨到当前状态，避免立即误开。
5. **`h`：**适时交还；录制器采样实测位姿后固定保持。再按 `t` 重新锚定继续。
6. **`s`：**标成功并结束。成功终止步要进入人工 pkl，须在接管中按 `s`。若已按 `h`，先再按 `t` 无跳变锚定，再标记。

`f` 丢弃，`r` 丢弃并开始下一次，Esc 无成功标签地结束。`q` 结束当前回合，处理结果后停止；Ctrl-C 也停止并保存已保留回合。默认按键奖励令 `s` 终止且奖励 1。交还后标记仍能结束回合，但终止步不会进入人工 pkl 视图。

*应该看到：* 回合结果和最终保存行，例如：

```text
[record] episode=0 steps=87 transitions=87 kept successes=1/20
[record] episode=1 steps=40 transitions=0 discarded (<reason>) successes=1/20
[record] saved 20 episodes (1734 transitions) to demos/pick-and-place.pkl
```

`WORKSPACE HOLD: <axes>` 表示请求目标越出安全盒。输入源交还并保持实测姿态；清除请求后 `h`、`t` 再锚定。这不是成功步骤，也不是扩大安全盒的许可。

<a id="record-with-a-spacemouse"></a>

## 用 SpaceMouse 录制

同样在 NUC、仓库根、项目环境激活。按 [teleop.md](../teleop.md#spacemouse) 安装 extra。覆盖 GELLO 选项文件，避免把 leader 选项传入 SpaceMouse：

```bash
python -m robot_infra.supervise --log runs/record-spacemouse/supervise.jsonl -- \
    python examples/record_real_demos.py --config examples/presets/gello_cartesian.yaml \
    --teleop-device spacemouse --teleop-event-device stdin --teleop-options-yaml None \
    --real_robot.watchdog_s 0.3 \
    --output-path demos/pick-and-place-spacemouse.pkl --num-successes 20
```

按键 **0** 接管，再按交还；只推杆不算接管。接管时推杆输入速度，全幅最多 0.05 m/s、0.3 rad/s。松手停止新增速度请求，机械臂仍可向现有命令收敛，最多一个领先上限。按键 **1** 只在驱动时切换夹爪；接管保留最后夹爪命令。结果和退出用终端 `s`/`f`/`r`/Esc/`q`，终端 `t`/`h` 被忽略。工作空间 HOLD 后须重新按键 0 接管。

<a id="files-and-compatibility"></a>

## 文件与兼容性

成功运行产生 `demos/pick-and-place.pkl`、`.contract.json` 和 `.trajectory.sqlite3`。归档保存全部请求/执行动作、mask、结果，包括失败、重录、中断尝试；pkl 只选成功人工步骤，没有保留回合则无 pkl。新数据集使用新 `.pkl` 路径；`--export-trajectory-pt` 另存可移植 `.trajectory.pt`。

sidecar 必须与 pkl 同行，包含版本化实验契约及 `teleop_control`（`cartesian_tracker_v1`、领先上限、目标速度）。本教程录制、训练、部署使用**同一 teleop 设备与四个 tracker 值**。缺少或改变 tracker 设置会报 `experiment.teleop_control` 差异。

**当前 HEAD 差异：**契约标识 tracker，而非实体输入设备。实现允许相同 tracker 参数的 GELLO 与 SpaceMouse 共用数据；同设备是本教程的工作流要求，单纯设备不同不会被当前契约拒绝。

*否则：*

- 超时：检查 60 秒回合上限，仅按合适任务时长增加 `--max-episode-steps`。
- `t` 无效：查终端焦点、GELLO 连接/标定，及 workspace HOLD 是否需 `h` 后 `t`；SpaceMouse 则按键 0。
- 读设备失败：停止，检查串口/HID 权限和硬件，确保无硬件进程占 station 后再跑 doctor。
- 契约不匹配：恢复录制的预设、设备和 tracker 值，不要修改 sidecar 凑兼容。

<details>
<summary>其他模式：关节空间 leader_arm 与 joint_abs（非主线）</summary>

这些模式需要独立关节预设和示范。`leader_arm` 的 `--teleop-onset arm` 等 leader 到 follower 的每关节误差不超过 `--teleop-arm-tolerance-rad`（0.1 rad）才接管。默认 `ramp` 立即开始：`joint_abs` 初始目标按 `--teleop-ramp-step-rad`（0.05）每步限速，`joint_delta` 在进入一个 `joint_scale_rad` 步长前标记初始追赶。只有这些关节 leader 路线使用 `teleop_syncing`，笛卡尔主线没有。

在 NUC、仓库根、项目环境激活，使用单独配置的 `hil_abs.yaml`（`joint_abs`、关节限位、本地 robot 与 GELLO teleop 部件）：

```bash
python -m robot_infra.supervise --log runs/record-joint-abs/supervise.jsonl -- \
    python examples/record_real_demos.py --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs --teleop-event-device stdin \
    --output-path demos/pick-and-place-abs.pkl --num-successes 20
```

`joint_abs` 初始任一关节差超过 `--start-tolerance-rad`（0.8）会拒绝开始，先对齐 leader。pkl 默认排除 syncing；`--keep-syncing-steps` 将其保留为普通监督步，但归档派生训练视图仍排除 syncing 监督。关节空间夹爪没有笛卡尔锁存，接管前须匹配扳机与当前夹爪状态。

</details>

<a id="continue-after-an-interruption"></a>

## 中断后继续

归档在录制中提交。pkl 尚不存在时重开同一输出 stem 会保留归档，将未完尝试标为中断，新尝试获新 ID。pkl 只导出当前调用保留的成功，早先完整成功从归档恢复。持久性始于 append 返回，之前执行但未 append 的动作可能丢失。进程崩溃与断电边界见[真机实验](../real-world-experiments.md#trajectory-archives-and-training-views)。重启停止进程前先读 supervisor 日志，核对工位并清除机器人故障后再 reset。

详见[录制参考](../recording-demos.md)。下一步：[05 基础策略](05-base-policy.md)。
