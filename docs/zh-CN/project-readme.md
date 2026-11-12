# real-world-rl

**文档：** [English](../README.md) · [中文文档入口](../README.zh-CN.md)

**从这里开始：** [硬件教程](tutorial/00-overview.md)以一条 Franka 和 GELLO 主臂，从空白 NUC 到部署策略，每步一条 `rwrl` 命令：[安装](tutorial/01-install.md)、[配置工作站](tutorial/02-setup.md)、[启动 Franka](tutorial/03-franka-bringup.md)，然后录制、训练、部署。

从 [rl-garden](https://github.com/JaimeParker/rl-garden) 拆出的真机 RL 编排层：`ActorLoop`/`LearnerLoop`/sync（SERL、HIL-SERL、residual、SiLRI 变体）、`franka_real` 和 `real_robot` 环境后端、`TeleopSource` 注册表与遥操作介入包装器，以及奖励分类器数据采集工具。

本包将 `rl_garden` 作为普通依赖导入（算法、缓冲区、策略及 `EnvBackend`/`BasePolicy` 约定），通过 rl-garden 的 `rlgarden.env_backends` 入口组注册 `franka_real`，添加它无需修改 `rl_garden` 内部文件。硬件/通信代码位于本仓库 `real_world_rl` 的无 torch 设备层（[设备层指南](devices.md)）；学习侧（torch、rl-garden）属于 `learning` 扩展，因此仅提供 daemon 的机器人 PC 安装 `pip install -e '.[franky,grpc]'`。`franka_real` 自身仅通过 HTTP/JSON（`FrankaBridgeClient`）耦合设备层，不共享 Python 类型。

本仓库是真机工作的顶层仓库，将 rl-garden 放在 [`rl-garden/`](../../rl-garden) git 子模块，固定到本仓库测试通过的提交。

<a id="methods"></a>

## 方法

每种方法是 `rwrl train <method> --role {actor,learner,eval}` 子命令（`real_world_bc` 无角色）。`residual_hil_serl` 和 `silri` 也通过 `rlgarden.algorithms` 注册到 rl-garden 在线注册表，使 `examples/deploy_real_robot.py` 可从 checkpoint 重建智能体（`rl-garden-eval` 无法评估残差 checkpoint，因为需要基础策略提供者）。使用 `rwrl train`、`rwrl record`、`rwrl deploy`、`rwrl collect-reward` 进行训练、录制、部署及奖励分类器数据采集。已有控制台脚本、Python 模块入口和 `examples/` 脚本保留为兼容别名。

SpaceMouse 默认 `--spacemouse-mode tracker`；可用 `--spacemouse-mode serl` 选择原始运动接管、释放后交还策略和按住按钮控制夹爪，须配置 XYZ/RPY 安全框（[模式指南](teleop.md#spacemouse)）。

| 方法 | 内容 | 指南 |
|---|---|---|
| `serl` | 双进程 SERL：机器人 PC 上 actor、GPU 机器上 learner，RLPD（状态或图像观测）。 | [真机运行](#running-on-hardware) |
| `hil_serl` | SERL 加人工介入（遥操作包装器）、增长的示范缓冲区、可选奖励分类器及 DQN 夹爪（`RLPDHybrid`）。 | [真机运行](#running-on-hardware)、[`docs/hil-serl-roadmap.md`](hil-serl-roadmap.md) |
| `residual_hil_serl` | 冻结基础策略（zero、rl-garden checkpoint 或 ACT）上的有界残差 HIL-SERL；`--residual-variant continuous\|hybrid`。 | [`docs/residual.md`](residual.md) |
| `silri` | HIL-RL（SiLRI）：通过随状态变化的拉格朗日乘子，将 actor 拉向学习得到的人工介入模型。 | [`docs/silri.md`](silri.md) |
| `real_world_bc` | 基于示范 pkl 的离线行为克隆，无 actor/learner 分离。 | [示范与快照约定附属文件](#demo-and-snapshot-contract-sidecar) |

<a id="repository-layout"></a>

## 仓库布局

```
real-world-rl/                   repository real-world-rl (top level)
  pyproject.toml                 distribution real-world-rl (package real_world_rl; rl_garden in the learning extra)
  rl-garden/                     git submodule: rl-garden, pinned to the tested commit
  real_world_rl/                 package real_world_rl
    loops/                       actor/learner/sync: base classes (actor, learner, sync) and one module per method (serl, hil_serl, residual, silri)
    algorithms/ policies/        real-world algorithms and policies (residual, SiLRI, base action providers)
    buffers/                     InterventionReplayBuffer (residual, SiLRI)
    envs/                        env backends (franka_real, real_robot, real_robot_bimanual) and wrappers
    teleop/ kinematics/          teleop sources; robot kinematics (numpy)
    training/                    registry, shared args, env building, entrypoints/ (one per method); python -m entry
    demos/ reward/               demo recording, reward-classifier data collection
    deploy.py                    eval-only deployment of a checkpoint
    common/ integrations/        small shared helpers; external integrations (integrations/rlinf)
    devices/ server/             torch-free device layer: devices and backends; the robot daemon and RemoteRobot
    discovery/ doctor/           hardware discovery and station loading; device checks
    setup/ supervise/            station setup wizard; process supervisor and heartbeat
    cli/                         the rwrl command
    legacy/                      legacy controllers, ROS1 Franka bridge and old teleop, kept for evaluation (no rwrl command)
  docs/                          README.md (index), tutorial/ (00-overview ... 11-deploy), cli.md, doctor.md, intervening.md (operator page), remote-actor.md, devices.md (device reference), daemon.md (robot daemon), bimanual.md, teleop.md, spacemouse-commissioning.md, gello.md, recording-demos.md, residual.md, silri.md, hil-serl-roadmap.md
  examples/                      train_real_world.py, deploy_real_robot.py, teleop demo and recording scripts; presets/hil_single_arm.yaml; stations/ (example station yamls)
  configs/                       RLinf config
  tests/                         tests/devices/ covers the device layer
```

设备层（`real_world_rl/{devices,server,discovery,setup,supervise,doctor}`）原为 `robot_infra` 包，更早是独立 `rlgarden-robot-infra` 仓库，历史仍保留。它不导入 `rl_garden` 或 `real_world_rl` 学习侧；命令（`setup`、`daemon`、`supervise`、`discover`）作为 `rwrl` 子命令运行。

旧名 `rlgarden-real-world`（包 `rl_garden_real_world`，控制台脚本 `rlgarden-real-*`），于 2026 年 11 月更名。

<a id="installation"></a>

## 安装

在 NUC 或开发机的父目录、未激活环境时运行：

```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl && bash scripts/install.sh
source .venv/bin/activate
```

安装器选项、仅 daemon 的机器人 PC、手工安装及步骤失败处理见[教程 01 安装](tutorial/01-install.md)。

<a id="tests"></a>

## 测试

无需硬件或网络，测试使用 fake 机器人和遥操作。从仓库根目录运行，先完成上方可编辑安装，或将两棵源码树加入 `PYTHONPATH`：

```bash
python -m pytest tests
PYTHONPATH=.:rl-garden python -m pytest tests   # without installing
```

rl-garden 自身测试在子模块内运行（见 `rl-garden/.agents/runbooks/testing.md`）。

<a id="running-on-hardware"></a>

## 真机运行

**首次使用硬件？** [教程](tutorial/00-overview.md)（[索引](README.md)）以单 Franka 和 GELLO 主臂讲解配置工作站、录制示范、BC/ACT、HIL-SERL / residual / SiLRI 及部署；双臂见 [docs/bimanual.md](bimanual.md#two-arms-the-tutorial-steps)。本节为参考。

以下均驱动 `real_robot` 环境（任意设备层机器人）。`franka_real`（HTTP 桥接）始终为 7 维笛卡尔，不在本节范围。安装本包（`pip install -e ./rl-garden -e '.[learning]'`）后通过 `rlgarden.env_backends` 注册 `real_robot`；否则 `--env-backend real_robot` 失败，报 `Unknown env backend 'real_robot'`。双臂合为一个环境使用 `--env-backend real_robot_bimanual`，见 [`docs/bimanual.md`](bimanual.md)。

<a id="robot-daemon"></a>

### 机器人守护进程

Daemon 在机器人 PC 运行（见设备层指南的[开始服务](daemon.md#serve-the-robot)）：

```bash
rwrl setup                  # writes ./local/station.yaml (the setup wizard)
rwrl daemon --watchdog-s 5  # serves it: no --station needed
```

[配置向导](tutorial/02-setup.md)生成 Franka 工作站 YAML（双臂每台机器人一个 daemon 进程）。手写机器人 YAML 可用 `rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 --watchdog-s 5` 提供服务。

Daemon 将超过 `--watchdog-s`（默认 0.5 秒）无动作的伺服臂置为空闲，环境在 `reset()` 结束时进入伺服。若 reset 后首个策略步可能更慢（首次推理、learner 忙），增大 `--watchdog-s`；否则首次 `step()` 报 `DeviceFault: the daemon watchdog stopped the arm ...`。

<a id="configuring-the-env"></a>

### 环境配置

相机与图像大小来自观测参数组，而非 `--real_robot.*`：`--obs.rgb wrist --obs.image-size 128 128`（远程机器人连接后才报告尺寸，有相机时须显式给 `--obs.image-size`）。`--real_robot.cameras` 从 `--obs.rgb` 填入，设为其他值抛 `ObservationContractError`。`--real_robot.joint_limits` 为 `[lo, hi]` 对列表，CLI 会展平而无法表达，应在 `--config` YAML 预设中给出；任意 `train_real_world.py` 运行均支持（`--print-config` 不接触机器人即可验证）。

单臂预设 [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml) 使用 `joint_delta`、一台 128×128 `wrist` 相机、`127.0.0.1:50051` daemon。复制到 `hil.yaml`，编辑 `remote` 和 `obs.rgb`（[教程 10 附录](tutorial/10-hil-training.md#the-env-preset-localgello_cartesianyaml)）。`joint_abs` 预设修改模式并加入限制：

```yaml
real_robot:
  action_mode: joint_abs
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
```

<a id="action-contract"></a>

### 动作约定

`action_mode` 确定 [-1, 1] 归一化动作含义；`gripper_in_action` 为 true（默认）时末尾添加夹爪维度。

| `action_mode` | 机械臂动作 | `joint_limits` | 要求 |
|---|---|---|---|
| `cartesian_delta` | 6 维 twist：xyz 乘 `xyz_scale_m`，旋转乘 `rot_scale_rad` | 可选 | 状态含 `tcp_pose` |
| `joint_delta` | `num_joints` 个增量乘 `joint_scale_rad` | 可选（裁剪） | `joint_pos` |
| `joint_abs` | `num_joints` 个值线性映射到 `joint_limits` | **必需**（构建时建立映射） | `joint_pos`，机器人须报告 `max_delta_per_step.joint_rad` |

hil_serl（`--env-backend real_robot`）还在构建环境时通过 `TeleopInterventionWrapper` 对照此约定检查遥操作源：

| teleop `action_mode` | 夹爪 | 支持情况 |
|---|---|---|
| `cartesian_delta` | 有 | 支持（spacemouse / pico / 机器人 teleop 部件） |
| `joint_delta` / `joint_abs` | 有 | 支持 `--teleop-device robot`（YAML `teleop:`）、`leader_arm` / `gello`（GELLO），双臂用 `bimanual` |
| 任意兼容模式 | 无（`gripper_in_action=false`） | 连续 HIL RLPD、连续 residual、连续 SiLRI 支持；混合/离散夹爪变体至少需一个夹爪 |

<a id="safety-limits"></a>

### 安全限制

机器人 PC 的 YAML 中配置 `safety_box`、`joint_limits`、`max_delta_per_step`，daemon 执行；环境只能**更严格**：

- 连接后（首个 `reset()`），未设的 `safety_box` / `joint_limits` / `home_joints` 从机器人继承。环境限制更松会在首个 reset 抛 `DeviceFault`，需收紧环境或放宽 YAML。
- `joint_abs` 另要求机器人报告 `max_delta_per_step.joint_rad`，因为连续绝对动作可跨整个关节范围。所有入口（hil_serl、deploy）均要求；缺失时首个 reset 报 `DeviceFault: action_mode joint_abs needs the robot's per-step joint limit ...`。
- `step()` 的 `info["safety_clips"]` 是设备裁剪伺服命令的累计数，每 10 步刷新（需 daemon 往返），机器人不能报告时为 `None`。持续增长表示策略不断请求越界目标。`reset()` 用 `move_to_joints` 归位，所以越界 `home_joints` 会在此抛 `SafetyViolation`。

<a id="teleoperation"></a>

### 遥操作

HIL 方法（`hil_serl`、`residual_hil_serl`、`silri`）通过 `--teleop-device` 选择的 `TeleopSource` 接收人工介入（`pico`、`spacemouse`、`robot`、`fake`、`gello`、`leader_arm`、`keyboard`、`leader_cartesian`，双臂为 `bimanual`；`spacemouse_twist` 为仿真原始 twist，不用于真机）。仅 actor、eval 打开设备；learner 禁用遥操作包装器，不在 GPU 机器打开 HID/ZMQ。`--teleop-device spacemouse` 是 actor 本机 SpaceMouse，经设备层 `SpaceMouseTeleop` 驱动（需 `hidapi==0.14.0.post4`，导入 `hid`）。`TeleopSource` 约定、包装器 `info` 键（`intervene_action`、`policy_action`、`intervened`、`intervention_onset`、`human_episode_end`、`teleop_events`）、示范/录制脚本及新增设备方式见 [`docs/teleop.md`](teleop.md)。

<a id="teleop---teleop-device-robot"></a>

### 机器人 teleop（`--teleop-device robot`）

hil_serl 可从机器人自身 teleop 部件介入（YAML `teleop:`：SpaceMouse、GELLO 主臂或 fake），由 daemon 提供。主臂（`--teleop-device leader_arm`/`gello`、起始策略、事件设备）见[主臂 GELLO](teleop.md#leader-arms-gello)。`--teleop-options-yaml PATH` 提供 CLI 无专用参数的选项（GELLO 串口/标定、SpaceMouse 索引、daemon 地址），`--teleop-event-device keyboard|stdin` 从第二设备获取介入切换/事件，均见 [`docs/teleop.md`](teleop.md)。`--teleop-device spacemouse` 则在 actor 本机经相同设备层 `SpaceMouseTeleop` 打开 SpaceMouse（无需 YAML `teleop:`）：按钮 0 接管/交还，按钮 1 切夹爪，操纵帽通过笛卡尔命令 tracker 输入速度，标记来自 `--teleop-event-device stdin`。设备表、接口约定、新增设备见 [`docs/teleop.md`](teleop.md)。对于 `--teleop-device robot`：

- `cartesian_delta` 原样传递；`joint_abs` 归一化到 `joint_limits`；`joint_delta` 变为 `(q_leader - q_follower) / joint_scale_rad`（`state_fields` 需 `joint_pos`）。
- [0, 1] 夹爪 `g` 变为 `2g - 1`。无夹爪 teleop 保持环境最后夹爪动作（reset 后为打开 +1），接管机械臂不松开物体。
- `episode_end` 以 `truncated` 结束回合，设置 `info["human_episode_end"]`；reset 归位期间按键丢弃。读取失败以 `DeviceFault` 停止 actor，不静默丢弃介入。
- 远程机器人连接后才报告 teleop，因此先用 `--robot-teleop-action-mode {cartesian_delta,joint_abs,joint_delta}` 声明（无夹爪再加 `--no-robot-teleop-has-gripper`）。连接时与实际部件核对，不匹配抛 `DeviceFault`。
- 配置的 teleop 必须存在才能连接。无人接管部署用 `--no-enable-teleop` 和不含 `teleop:` 的 YAML；辅助部署保留设备。

无需硬件的 fake teleop 演练（机器人 YAML；`on_end: loop` 重复脚本，持续结束回合）：

```yaml
teleop:
  name: leader
  backend: fake
  options:
    action_space: joint_abs
    num_joints: 7
    has_gripper: true
    on_end: loop
    script:
      - {action: [0, 0, 0, 0, 0, 0, 0], active: false, hold: 5}
      - {action: [0.1, 0, 0, -1.5, 0, 1.5, 0.7], gripper: 0.0, active: true, hold: 5}
      - {events: {episode_end: true}}
```

<a id="deploy"></a>

### 部署

```bash
rwrl deploy --checkpoint runs/.../checkpoints/checkpoint_<step>.pt \
    --algo rlpd_hybrid --real_robot.remote ROBOT_PC:50051 --real_robot.token T \
    --episodes 5 --max-steps-per-episode 100
```

（serl checkpoint 用 `--algo rlpd`，hil_serl 用 `rlpd_hybrid`，`real_world_bc` 运行用 `real_world_bc`，`residual_hil_serl` / `residual_hil_serl_hybrid` 与 `silri` 见下文；双臂加 `--env-backend real_robot_bimanual`，见 [`docs/bimanual.md`](bimanual.md#deploy)。）训练目录的 `config.json`（`checkpoints/` 旁）是动作/观测约定的权威来源：

- 部署环境从运行读取 `action_mode`、`gripper_in_action`、`num_joints`、`joint_limits`、`xyz_scale_m`、`rot_scale_rad`、`joint_scale_rad`、`state_fields`、`image_size`、`control_hz` 及任务/控制/预处理设置。工作站拥有的 reset 信息与回合上限可为部署调整。自动重建观测包装器：`obs.frame_stack`（堆叠 `classifier_image_keys`）、`convert_obs_rotation`。`--real_robot.*` 仅提供连接字段（`robot`、`remote`、`token`、`home_joints`、`safety_box`、`reset_settle_s` 等）。
- CLI 约定字段不同于训练值会报 `ValueError: --real_robot.* contradicts the training run's real_robot config ...`。**限制：** 等于 dataclass 默认值的 CLI 值无法区分是否显式设置，因此对 `joint_abs` 运行传 `--real_robot.action_mode cartesian_delta` 会静默采用运行的 `joint_abs`。
- 无 `real_robot` 配置的旧运行或 `franka_real` 会警告并回退 CLI，不验证约定。
- `DeviceError`（daemon 不可达、watchdog、安全）输出消息/提示，以 2 退出。
- `--jpeg-quality` 不在运行配置内，部署 daemon 应使用与训练相同值（或都不设置）。
- 部署接受 `bundle_manifest.json` 或独立 `.pt`。`enable_teleop` 默认 true；活动任务或显式 `--teleop-device` 组合操作员设置。无人接管用 `--no-enable-teleop`。分类器/任务可提供显式结果；主 `success_rate` 是已完成回合中的自主成功率。两种部署模式见[教程 11](tutorial/11-deploy.md)。

<a id="residual-rl-residual_hil_serl"></a>

### 残差 RL（`residual_hil_serl`）

`residual_hil_serl` 在冻结基础策略（`--base-provider zero | checkpoint:<path>?algo=<name> | act:<path>`）上学习有界残差 `clamp(base + scale * u, -1, 1)`，沿用 hil_serl 的 actor/learner/eval、示范缓冲区和介入。人工可选择部件；可选 BC 项使用残差缩放非零的实际人工维度。`--residual-variant continuous`（`ResidualRLPD`）在全部动作维度学习残差，`hybrid`（`ResidualRLPDHybrid`）仅机械臂残差加 DQN 夹爪。仅 actor、eval、deploy 构建基础策略，deploy 从 checkpoint 元数据重建（`--algo residual_hil_serl` 或 `residual_hil_serl_hybrid`）。见 [`docs/residual.md`](residual.md)。

<a id="hil-rl-silri"></a>

### HIL-RL（`silri`）

`silri` 实现 HIL-RL / SiLRI（arXiv 2512.24288）：actor-critic 在随状态变化的拉格朗日乘子下，将 actor 拉向学习得到的人工介入高斯模型，一致人工行为约束策略，不一致行为交给 critic。角色、设备、示范 pkl 同 hil_serl；learner 在示范缓冲区预训练，介入到来时刷新人工模型，actor 在每次接管前的 transition 标记 `intervention_onset`。部署用 `--algo silri`。见 [`docs/silri.md`](silri.md)。

<a id="demo-and-snapshot-contract-sidecar"></a>

### 示范与快照约定附属文件

hil_serl learner 的回放和示范缓冲区快照每个 pkl 旁带 `<name>.contract.json`（`transitions_N.pkl` → `transitions_N.contract.json`）；`--demo-dataset-paths` 的 pkl 与此文件一起读取。它是一行排序 JSON：

```json
{"action_mode": "joint_abs", "arm_action_dim": 7, "gripper_in_action": true, "joint_limits": [[-2.8, 2.8], ...], "joint_scale_rad": 0.05, "num_joints": 7, "rot_scale_rad": 0.1, "state_fields": ["joint_pos", "gripper_pos"], "xyz_scale_m": 0.02}
```

加载时与环境约定比较（浮点用 `allclose`）。不匹配抛 `ValueError: ... recorded action contract does not match the env's (differs in ['joint_limits']) ...`。早于缩放/限制字段的附属文件仅检查已有键并警告。无附属文件的 pkl 仅检查动作宽度并警告（`... has no action-contract metadata (<name>.contract.json missing) ...`）。

<a id="end-to-end-try-out-with-the-fake-robot"></a>

### fake 机器人端到端演练

一台机器，无硬件（项目容器已验证；短训练且策略未训练完成）。机器人 PC 侧使用 `type: franka` 的 fake `robot.yaml`（`arm_backend: fake`、`gripper: {backend: fake}`、名为 `wrist` 的 fake 相机、示例工作站的关节限制及 `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}`、上方 fake teleop）：

```bash
rwrl daemon --station robot.yaml --bind 127.0.0.1:50061 --token T \
    --jpeg-quality 90 --watchdog-s 5 --no-preflight
```

Learner 和 actor（两个终端；`hil.yaml` 为上方预设，设 `remote: 127.0.0.1:50061`、`token: T`、`joint_abs` 模式与限制）：

```bash
COMMON="--config hil.yaml --teleop-device robot --robot-teleop-action-mode joint_abs \
    --log-dir runs --log-type none --exp-name smoke --batch-size 16 --learning-starts 32 \
    --buffer-size 2000 --checkpoint-freq 10 --sync-host 127.0.0.1 --sync-port 6055 --publish-freq 10"
rwrl train hil_serl --role learner $COMMON
rwrl train hil_serl --role actor $COMMON
```

用 Ctrl-C 停止两者。Learner 写入 `runs/smoke/config.json` 和 `runs/smoke/checkpoints/checkpoint_<update>.pt`，再部署最后 checkpoint：

```bash
rwrl deploy --checkpoint runs/smoke/checkpoints/checkpoint_<update>.pt \
    --algo rlpd_hybrid --real_robot.remote 127.0.0.1:50061 --real_robot.token T \
    --episodes 2 --max-steps-per-episode 15
```

每回合输出一行 `[deploy] episode=... steps=15 ... health_ok=True`，以及一行 `[deploy] summary ...`。真机使用同一命令，但 daemon 在机器人 PC，`--bind 0.0.0.0:50051`，使用真实 `ROBOT_PC:50051`，`--sync-host` 指向 learner 可达地址，并使用真实 `control_hz`。

SERL（仅状态或图像）的双进程真机训练使用同样参数：

```bash
rwrl train serl --role learner ...
rwrl train serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.num_joints 7 \
    --obs.rgb wrist --obs.image-size 128 128 ...
```

（`serl` 真机参数仅通过 `--print-config` 检查，未实际运行 SERL。）

RLinf 从 [`configs/rlinf/realworld_rlgarden_franka.yaml`](../../configs/rlinf/realworld_rlgarden_franka.yaml) 开始。包必须能在 Ray env-worker 进程导入，RLinf 会终止 env 节点运行中的全部 roscore。

**RLinf**：通过 Gymnasium 注册驱动本包，不经过 rl-garden 适配器。RLinf 真机配置的 env 块设置 `env_type: real`、`init_params.id: "real_world_rl.integrations.rlinf.register:RLGardenReal-v0"`；`RealWorldEnv` 用 `gym.make` 创建 `RealRobotEnv`（每 env worker 一台机器人），`override_cfg` 为配置。环境块及风险见 [`configs/rlinf/realworld_rlgarden_franka.yaml`](../../configs/rlinf/realworld_rlgarden_franka.yaml)，约定见 `tests/test_rlinf_registration.py`。rl-garden Pattern-A RLinf 适配器（`rl_garden/integrations/rlinf/{sac_actor,sac_rollout}.py`）运行 RLinf 自身 env worker，无真机/图像路径，不能驱动本包。

用 `rwrl train serl --role ...` 训练（双进程 actor/learner 启动形式见该文件模块 docstring）。遥操作演示/录制流程见 [`docs/teleop.md`](teleop.md)，HIL-SERL 已迁移/未迁移能力及原因见 [`docs/hil-serl-roadmap.md`](hil-serl-roadmap.md)。

实验恢复、自主/辅助指标、部件接管、轨迹归档见[真机实验](real-world-experiments.md)。人工动作掩码让局部示范在 BC、residual、SiLRI 训练中保持对齐，无夹爪 HIL 使用连续策略。硬件验证仍待完成。

当前 learner 恢复使用 `bundle_manifest.json`、配套 checkpoint 目录和 SQLite journal。Actor 上传用持久 outbox，重启应保留身份/路径。所选部件和观测预处理进入实验约定。见[教程 10](tutorial/10-hil-training.md#9-resume-an-interrupted-run)和[机械臂控制配置](control-profiles.md)。
