<!-- English source: docs/tutorial/07-teleop-leader-follower.md -->

<a id="7-teleop-leader-follower"></a>

# 7. 遥操作：主臂/从臂

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/07-teleop-leader-follower.md)

**目标：** 选择关节空间或笛卡尔主臂映射，检查接管行为，理解 GELLO 或第二台机械臂如何提供主臂读数。

<a id="before-you-start"></a>

## 开始前

除非另有说明，命令在**从臂 NUC** 的**仓库根目录**、**激活项目环境**后运行，录制需包含学习依赖。

- 完成 [03](03-franka-bringup.md)、[04](04-first-motion.md)、[05 GELLO](05-teleop-gello.md)。保持物理停止装置可及、工作区畅通；录制 reset 可能移动从臂。阅读[操作顺序](../intervening.md#the-sequence)。
- `leader_arm` 读取从臂机器人的关节空间遥操作部件（通常为守护进程提供的 GELLO），需要关节空间环境预设。`leader_cartesian` 读取独立主臂部件，以正运动学驱动 `cartesian_delta` 从臂。
- 下文第二台机械臂路径**需要硬件验证**。须现场检查手动引导、FK/TCP 一致性、夹爪读数及物理停止行为。GELLO 映射也需实际方向与运动学检查；模拟测试不能确立硬件行为。

<a id="1-check-joint-space-alignment"></a>

## 1. 检查关节空间对齐

使用已校准 GELLO 站点和运行中的从臂守护进程，无录制器或 actor 发送命令：

```bash
rwrl --station local/station.yaml teleop read --duration-s 30 --rate-hz 3 --with-follower
```

*应看到：* `joint_abs` 主臂的 `q`、实测 `follower_q`、逐关节 `delta`（主臂减从臂，弧度），以及 `gripper`、`intervening`、触发的 `events`。只移动主臂观察读数，命令不发送从臂运动。多姿态比较符号和关节顺序；不符则重做 [GELLO 校准](05-teleop-gello.md#2-calibrate-and-save-the-units-signs-and-offsets)。

`--with-follower` 要求守护进程路径。直读只打开遥操作输入，读取从臂需另建机械臂连接。无守护进程时先完成 [03 的守护进程设置](03-franka-bringup.md)，或省略 `--with-follower` 仅检查输入。连接错误见[失败怎么办](#if-it-fails)。

<a id="2-record-with-joint-space-leaderarm"></a>

<a id="2-record-with-joint-space-leader_arm"></a>

## 2. 使用关节空间 `leader_arm` 录制

创建独立关节空间预设，与 05 的笛卡尔预设分开。在 NUC 的仓库根目录复制（无需环境）：

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil_abs.yaml
```

编辑 `hil_abs.yaml`：将 `real_robot.action_mode` 从 `joint_delta` 改为 `joint_abs`，取消 `joint_limits` 注释，用站点已核查限制替换示例边界。将 `remote` 及所需 `token` 指向运行中的从臂守护进程，`obs.rgb` 匹配相机。`state_fields` 保留 `joint_pos`，为任务调试 home/reset，确保守护进程站点提供 `max_delta_per_step.joint_rad`。绝对关节控制同时需要关节限制和单步上限。见[机器人 YAML 参考](../gello.md#3-the-robot-yaml-real_robot-franky)。

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl supervise --log runs/record-leader-joint/supervise.jsonl -- \
    rwrl record --config hil_abs.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_abs \
    --teleop-onset arm --teleop-event-device stdin \
    --output-path demos/leader-joint-check.pkl --num-successes 1
```

*应看到：* reset 后接管由对齐门控：`t` 请求控制，但 `arm` 等待所有主臂关节距从臂不超过 0.1 rad。先对齐，再引导，匹配扳机与期望夹爪状态，用 `h` 交还。介入时 `s` 保留成功示范，或按操作员页使用 `f`/`r`/Esc/`q`。普通录制器自动处理回合，默认不采用 `teleop run` 的标签保持和 `n` 顺序。启动或待接管失败见[失败怎么办](#if-it-fails)。

`--robot-teleop-action-mode joint_abs` 在连接前声明远程遥操作部件；环境 `real_robot.action_mode` 决定输出动作。`--teleop-device leader_arm` 选择部件；`--teleop-event-device stdin` 在此终端（含 SSH）提供接管和标签，其切换替代部件自身介入状态。选项 YAML 为多机器人站点时用 `--teleop-robot NAME` 选择机器人；本单臂配置无需它。必要时 `--teleop-options-yaml PATH` 提供源选项。

请有意识地选择接管起始方式：

| `--teleop-onset` | 行为 |
|---|---|
| `arm` | 等待最大关节误差 ≤ `--teleop-arm-tolerance-rad`（默认 0.1 rad）；reset 解除待接管。对齐前该步不算介入。 |
| `ramp` (default) | 立即接管；初始 `joint_abs` 目标每步最多前进 `--teleop-ramp-step-rad`（默认 0.05 rad），直至对齐。 |

尝试 `ramp` 时替换命令中的 `arm` 并选择新输出路径。初始追赶标记为 `teleop_syncing`：归档保留执行动作，人工 pkl 默认排除同步步（`--keep-syncing-steps` 可保留）。`joint_delta` 的初始追赶标记持续到主臂首次进入一个 `joint_scale_rad` 步长内。各动作模式使用独立兼容数据集。见[起始方式取舍](../gello.md#onset-ramp-or-arm)和[主臂源](../teleop.md#leader-arms-gello)。

<a id="3-use-gello-as-a-cartesian-leader"></a>

## 3. 用 GELLO 作为笛卡尔主臂

受监督 GELLO 流程复用已调试本地笛卡尔预设和 [05 会话命令](05-teleop-gello.md#4-complete-one-supervised-session)。它选择 `leader_cartesian`，不使用上述关节空间起始策略。

接管时正运动学得到主臂姿态 `(p_l0, R_l0)`，当前命令姿态 `(p_c, R_c)` 锚定于此。移动主臂请求 `p* = p_c + (p_l − p_l0)`、`R* = (R_l R_l0^T) R_c`。tracker 限制速度及领先实测从臂 TCP 的幅度；默认上限为 0.05 m / 0.3 rad，速度为 0.05 m/s / 0.3 rad/s。环境尺度至少应等于上限。此路径无需逐关节对齐。

会话将站点 `workspace`、`task_frame` 复制到解析环境。`real_robot` 上源也从环境配置读取，回退到本地机器人 YAML；显式源选项优先。工作区（任务坐标中的位置及可选方向）与基坐标安全盒同时生效。`WORKSPACE HOLD` 交还并保持实测姿态；仅返回边界内不会恢复跟随。清除请求，按 `h` 再按 `t` 重新锚定。见[映射与限制](../teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)。

*应看到：* 主臂小位移产生同向有界笛卡尔运动，交还后保持实测姿态。方向或 FK/TCP 对齐错误时，停止并验证模型后再继续；见[失败怎么办](#if-it-fails)。

<a id="4-use-a-second-arm-as-the-cartesian-leader"></a>

## 4. 用第二台机械臂作为笛卡尔主臂

**需要硬件验证。** 第二台机械臂可通过 `arm_leader` 遥操作后端提供 `joint_abs` 读数。它只读不发运动命令，连接不会启用手动引导。按主臂厂商流程安排手动操作。

在**主臂主机**的仓库根目录复制，无需环境：

<!-- tutorial-test: skip -->
```bash
cp examples/stations/leader.franka-as-leader.example.yaml local/leader.yaml
```

修改副本为实际主臂地址与后端。相关结构如下：

```yaml
type: franka
name: franka_leader_station
arm_backend: fake
gripper: null
cameras: []
teleop:
  name: leader
  backend: arm_leader
  options:
    arm: {type: franky, robot_ip: 172.16.0.2, realtime: ignore}
    gripper: false
    intervention: always
```

`arm.type` 选择后端，其余 `arm` 键传给构造器。外层模拟臂是载体：只有遥操作部件打开真实主臂，避免重复 libfranka 会话。`gripper` 默认 `false`；无主臂夹爪时从臂保持末次夹爪动作。`intervention: always` 报告激活，由从臂终端事件决定接管。

在**主臂主机**的仓库根目录，**激活机械臂 SDK 和守护进程环境**：

```bash
rwrl daemon --station local/leader.yaml --bind 0.0.0.0:50052
```

*应看到：* 守护进程监听所选地址。它提供主臂遥操作部件，不提供或控制从臂。连接失败时在此主机以 `--station local/leader.yaml` 运行 doctor，按[失败怎么办](#if-it-fails)处理。

在**从臂 NUC** 保存 `local/leader-cartesian.yaml`，使用以下平铺源选项映射；将 `leader-host` 替换为主臂可达主机（守护进程使用令牌时添加 `remote_token`）：

```yaml
remote_address: leader-host:50052
mapping: anchored
kinematics: panda
```

`from_env` 仅接受一种主臂路径：`remote_address`、`robot_yaml`（本地载体 YAML）或进程内 `leader` 部件/解析器。远程路径在源启动时连接。`kinematics` 默认 `panda`，是唯一内置模型；主臂必须报告恰好七个关节的 `joint_abs`。其他臂模型需注册匹配运动学实现；模型选择不会自动跟随 `arm.type`。

在**从臂 NUC** 的**仓库根目录**，**激活学习环境**，使用已调试从臂笛卡尔预设并覆盖 GELLO 源选项。普通录制器从预设读取从臂设置，因此须在其 `real_robot` 下（或本地机器人 YAML）加入已审查 `task_frame`、`workspace`；它不像 `teleop run` 自动合并 `local/station.yaml`。预设本地打开从臂时，先结束直接占用从臂的守护进程；主臂守护进程继续运行：


```bash
rwrl supervise --log runs/record-second-leader/supervise.jsonl -- \
    rwrl record --config local/gello_cartesian.yaml \
    --teleop-device leader_cartesian --teleop-options-yaml local/leader-cartesian.yaml \
    --teleop-event-device stdin \
    --output-path demos/second-leader-check.pkl --num-successes 1
```

*应看到：* 从臂 reset、固定姿态保持，`t` 后锚定运动、`h` 后交还。保留数据前检查手动引导、平移/旋转方向、主臂 FK 与实际 TCP、工作区保持及夹爪通道。这些预期行为是调试目标，**不是硬件验证结果**。完整契约见[第二台机械臂作主臂](../teleop.md#a-second-arm-as-the-leader)。

<a id="if-it-fails"></a>

## 失败怎么办

先在从臂 NUC 运行 `rwrl doctor --station local/station.yaml`（或主臂主机运行上述自身站点命令），在 [doctor 红灯处理](../doctor.md#red-lights-what-to-do)找到红色行。

- GELLO 串口/输入访问：[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)。
- 其他进程占用臂或串口：[设备占用 (BLOCKED)](../doctor.md#busy-devices-blocked)。
- 守护进程不可达或 FCI 失败：[网络与 FCI](../doctor.md#network-and-fci)。
- 对齐始终不能接管：检查关节顺序/符号及 `delta`；先修正校准再改容差。关节数/FK 不匹配需正确运动学模型。
- `WORKSPACE HOLD`：清除请求，重新接管锚定；检查现有限制和实际坐标系，不要扩大边界来解除保持。
- 加载数据时契约不匹配：使用同动作模式、tracker 设置的数据集；见 [08 兼容性](08-record-demos.md#files-and-compatibility)。

下一步：[08 录制示范](08-record-demos.md)。
