<a id="real-world-rl"></a>

# real-world-rl

[中文入口](../README.zh-CN.md) · [English](../../README.md)

**文档：**[文档索引](README.md) · [中文文档入口](../README.zh-CN.md)

本项目是从 [rl-garden](https://github.com/JaimeParker/rl-garden) 拆出的真机 RL 编排层，包含 `ActorLoop` / `LearnerLoop` / 同步机制，支持 SERL、HIL-SERL、残差与 SiLRI；还提供 `franka_real`、`real_robot` 环境后端，`TeleopSource` 注册表、遥操作介入包装器和奖励分类器数据采集工具。

本包正常依赖 `rl_garden` 的算法、缓冲区、策略及 `EnvBackend` / `BasePolicy` 约定，并通过 `rlgarden.env_backends` 入口点注册 `franka_real`，无需改动 `rl_garden` 内部文件。硬件通信代码位于仓库的 [`robot_infra/`](robot-infra.md)，它是独立、无 torch 依赖的发行包 `rlgarden-robot-infra`，机器人 PC 可仅安装 `pip install -e robot_infra`。`franka_real` 通过 `FrankaBridgeClient` 与它进行 HTTP/JSON 通信，不共享 Python 类型。

本仓库是真机工作的顶层工程，使用 [`rl-garden/`](../../rl-garden) Git 子模块，固定到本仓库测试已通过的提交。

<a id="methods"></a>

## 方法

每个方法对应 `python examples/train_real_world.py <method> --role {actor,learner,eval}` 的子命令，`real_world_bc` 没有角色。`residual_hil_serl` 和 `silri` 还通过 `rlgarden.algorithms` 注册到 rl-garden 在线注册表，使 `examples/deploy_real_robot.py` 可从 checkpoint 重建智能体。残差 checkpoint 需要基础提供器，因此不能用 `rl-garden-eval` 评估。
`examples/` 的训练、录制与部署脚本是已安装包命令的薄包装：`real-world-rl-train` / `python -m real_world_rl.training`、`real-world-rl-record` / `python -m real_world_rl.demos`、`real-world-rl-deploy` / `python -m real_world_rl.deploy`；奖励分类器数据采集使用 `real-world-rl-collect-reward` / `python -m real_world_rl.reward.collect_data`。

| 方法 | 作用 | 指南 |
|---|---|---|
| `serl` | 双进程 SERL，actor 位于机器人 PC，learner 位于 GPU 机器，使用状态或图像 RLPD。 | [真机运行](#running-on-hardware) |
| `hil_serl` | 在 `serl` 上加入人工介入包装器、持续增长示范缓冲区、可选奖励分类器和 DQN 夹爪 `RLPDHybrid`。 | [真机运行](#running-on-hardware)、[`docs/hil-serl-roadmap.md`](hil-serl-roadmap.md) |
| `residual_hil_serl` | 在 `hil_serl` 基础上，对冻结的零策略、rl-garden checkpoint 或 ACT 学习有界残差；使用 `--residual-variant continuous\|hybrid`。 | [`docs/residual.md`](residual.md) |
| `silri` | HIL-RL / SiLRI，以状态相关拉格朗日乘子将 actor 约束到学得的人工介入模型附近。 | [`docs/silri.md`](silri.md) |
| `real_world_bc` | 从示范 pkl 离线行为克隆，无 actor/learner 分离。 | [示范与快照约定附属文件](#demo-and-snapshot-contract-sidecar) |

<a id="repository-layout"></a>

## 仓库布局

```
real-world-rl/                   repository real-world-rl (top level)
  pyproject.toml                 distribution real-world-rl (imports rl_garden)
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
  robot_infra/                   in-repo device layer: distribution rlgarden-robot-infra (package robot_infra, torch-free)
    pyproject.toml
    robot_infra/                 devices, discovery, doctor, daemon, RemoteRobot
    tests/
  docs/                          README.md (index), tutorial/ (00-overview ... 08-deploy), bimanual.md, teleop.md, gello.md, recording-demos.md, residual.md, silri.md, hil-serl-roadmap.md
  examples/                      train_real_world.py, deploy_real_robot.py, teleop demo and recording scripts; presets/hil_single_arm.yaml
  configs/                       RLinf config
  tests/
```

`robot_infra/` 是本仓库内的设备层发行包，以前为独立 `rlgarden-robot-infra` 仓库，历史已保留。它不导入 `rl_garden` 或 `real_world_rl`。

项目旧名 `rlgarden-real-world`，包名 `rl_garden_real_world`，命令为 `rlgarden-real-*`，于 2026 年 11 月更名。

<a id="installation"></a>

## 安装

克隆时同时获取 rl-garden 子模块：

```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
# in an existing clone: git submodule update --init
```

开发时从仓库根目录以可编辑方式安装三个发行包：

```bash
pip install -e rl-garden -e robot_infra -e .
```

`rl-garden/` 固定在本仓库测试通过的提交。更新时有意执行 `git -C rl-garden checkout <commit>`，运行测试，再提交新 gitlink。若没有子模块，普通 `pip install .` 会改从 `pyproject.toml` 获取 `rl_garden @ git+...` 依赖。

机器人 PC 只需设备层，见 [`robot_infra/README.md`](robot-infra.md)：运行 `pip install -e robot_infra`，并添加硬件需要的可选组，如 `'robot_infra[franky,grpc]'`。`pyspacemouse` 用于 `--teleop-device spacemouse`，懒加载且仅需安装在实际使用设备的机器上。

<a id="tests"></a>

## 测试

测试采用 fake 机器人与遥操作，无需硬件或网络。完成前述可编辑安装后，从根目录运行；也可通过 `PYTHONPATH` 提供三份源码：

```bash
python -m pytest tests robot_infra/tests
PYTHONPATH=.:rl-garden:robot_infra python -m pytest tests robot_infra/tests   # without installing
```

rl-garden 自身测试在子模块内运行，见 `rl-garden/.agents/runbooks/testing.md`。

<a id="running-on-hardware"></a>

## 真机运行

**第一次配置硬件？** [教程](tutorial/00-overview.md)及其[索引](README.md)带你使用单台 Franka 和 GELLO，完成工作站配置、示范录制、BC 与 ACT、HIL-SERL / 残差 / SiLRI 及部署。双臂见 [docs/bimanual.md](bimanual.md#two-arms-the-tutorial-steps)。本节提供参考细节。

下文控制 `real_robot` 环境，即任意 `robot_infra` 机器人；HTTP 桥接 `franka_real` 固定使用 7 维笛卡尔动作，另见其说明。先安装本包 `pip install -e rl-garden -e robot_infra -e .`，经 `rlgarden.env_backends` 注册后端，否则 `--env-backend real_robot` 会报 `Unknown env backend 'real_robot'`。双臂使用 `--env-backend real_robot_bimanual`，见 [`docs/bimanual.md`](bimanual.md)。

<a id="robot-daemon"></a>

### 机器人守护进程

守护进程运行在机器人 PC，见设备层 README 的[如何开始](robot-infra.md#how-to-start)：

```bash
python -m robot_infra.setup                  # writes ./local/station.yaml (the setup wizard)
python -m robot_infra.server --watchdog-s 5  # serves it: no --config needed
```

[配置向导](robot-infra.md#setup-wizard)生成 Franka 工作站 YAML，双臂每机器人一个守护进程。手写机器人 YAML 可用 `python -m robot_infra.server --config robot.yaml --bind 0.0.0.0:50051 --watchdog-s 5` 启动。

机械臂在 `--watchdog-s` 内未收到动作会进入空闲，默认 0.5 秒；环境在 `reset()` 结束时进入伺服。若首次推理或繁忙 learner 使 reset 后第一步耗时更长，应提高 `--watchdog-s`，否则首个 `step()` 抛 `DeviceFault: the daemon watchdog stopped the arm ...`。

<a id="configuring-the-env"></a>

### 环境配置

相机与图像尺寸来自观测参数组，例如 `--obs.rgb wrist --obs.image-size 128 128`。远程机器人连接后才报告尺寸，因此使用相机时必须指定 `--obs.image-size`。`--real_robot.cameras` 从 `--obs.rgb` 填充，不同值会触发 `ObservationContractError`。`--real_robot.joint_limits` 是 `[lo, hi]` 成对列表，命令行会将其展平，因此需要放在 `--config` YAML 预设中。所有 `train_real_world.py` 运行都支持预设，`--print-config` 可在不连接硬件时验证。`--real_robot.*` 用于其余机器人配置。

单臂预设为 [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml)，使用 `joint_delta`、一台 128 × 128 的 `wrist` 相机和 `127.0.0.1:50051` 守护进程。复制为 `hil.yaml` 后修改 `remote` 与 `obs.rgb`，见[教程 02](tutorial/02-station.md#the-env-preset-hilyaml)。`joint_abs` 预设需修改模式并增加限制：

```yaml
real_robot:
  action_mode: joint_abs
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
```

<a id="action-contract"></a>

### 动作约定

`action_mode` 定义 [-1, 1] 归一化动作的含义；`gripper_in_action` 默认为真，会在末尾增加夹爪维度。

| `action_mode` | 机械臂动作 | `joint_limits` | 依赖 |
|---|---|---|---|
| `cartesian_delta` | 6 维速度增量，xyz 按 `xyz_scale_m` 缩放，旋转按 `rot_scale_rad` 缩放。 | 可选。 | 状态中的 `tcp_pose`。 |
| `joint_delta` | `num_joints` 个增量，按 `joint_scale_rad` 缩放。 | 可选，用于裁剪。 | `joint_pos`。 |
| `joint_abs` | `num_joints` 个值，线性映射到 `joint_limits`。 | **必需**，构建时建立映射。 | `joint_pos`，且机器人报告 `max_delta_per_step.joint_rad`。 |

hil_serl 的 `--env-backend real_robot` 在构建 `TeleopInterventionWrapper` 时也检查遥操作源符合约定：

| 遥操作 `action_mode` | 夹爪 | 支持情况 |
|---|---|---|
| `cartesian_delta` | 有 | 支持 spacemouse、pico 或机器人遥操作部件。 |
| `joint_delta` / `joint_abs` | 有 | 支持 `--teleop-device robot`，读取 YAML 的 `teleop:`，也支持 `leader_arm` / `gello`，双臂使用 `bimanual`。 |
| 任意 | 无，`gripper_in_action=false` | `hil_serl`、`residual_hil_serl`、`silri` 的介入包装器都要求夹爪维度，因此不支持。 |

<a id="safety-limits"></a>

### 安全限制

机器人 PC 上的 YAML 保存 `safety_box`、`joint_limits`、`max_delta_per_step`，由守护进程执行；环境侧只能设置**更严格**的限制：

- 连接后的第一次 `reset()` 会继承设备上未显式设置的 `safety_box` / `joint_limits` / `home_joints`。环境限制较宽松时抛 `DeviceFault`，需要收紧环境值或有意调整 YAML。
- `joint_abs` 还要求机器人报告 `max_delta_per_step.joint_rad`，因为连续 `joint_abs` 动作可跳过整个关节范围。hil_serl 与 deploy 等所有入口均执行检查，缺失时首次 reset 抛 `DeviceFault: action_mode joint_abs needs the robot's per-step joint limit ...`。
- `step()` 的 `info["safety_clips"]` 为设备累计裁剪伺服命令数，每 10 步通过一次守护进程往返刷新；设备无法报告时为 `None`。持续增长表示策略频繁请求越界目标。`reset()` 经 `move_to_joints` 归位，`home_joints` 越界会抛 `SafetyViolation`。

<a id="teleoperation"></a>

### 遥操作

HIL 方法 `hil_serl`、`residual_hil_serl`、`silri` 通过 `--teleop-device` 选择的 `TeleopSource` 接收人工介入，设备包括 `pico`、`spacemouse`、`robot`、`fake`、`gello`、`leader_arm`、`keyboard`、`leader_cartesian` 及双臂 `bimanual`。只有 actor 和 eval 打开设备，learner 禁用遥操作包装器，GPU 机器不打开 HID 或 ZMQ。`--teleop-device spacemouse` 经设备层 `SpaceMouseTeleop` 驱动 actor 本机设备，需要 `pyspacemouse>=2,<3`。接口约定、包装器 `info` 字段 `intervene_action`、`policy_action`、`intervened`、`intervention_onset`、`human_episode_end`、`teleop_events`，以及演示、录制和新增设备方式见 [`docs/teleop.md`](teleop.md)。

<a id="teleop---teleop-device-robot"></a>

### 机器人遥操作（`--teleop-device robot`）

hil_serl 可以经守护进程读取机器人 YAML 的 `teleop:` 部件，支持 SpaceMouse、GELLO 主臂或 `fake`。`--teleop-device leader_arm` / `gello`、起始策略和事件设备见 [GELLO 主臂](teleop.md#leader-arms-gello)。`--teleop-options-yaml PATH` 提供 CLI 未暴露的端口、标定、设备索引和守护进程地址；`--teleop-event-device keyboard|spacemouse` 从第二设备接收切换和事件，均见 [`docs/teleop.md`](teleop.md)。`--teleop-device spacemouse` 直接通过相同设备层 `SpaceMouseTeleop` 打开 actor 本机设备，无需 YAML `teleop:` 段；设备表、`TeleopSource` 约定和扩展方法同见 [`docs/teleop.md`](teleop.md)。`--teleop-device robot` 的规则如下：

- `cartesian_delta` 原样传递；`joint_abs` 按 `joint_limits` 归一化；`joint_delta` 转成 `(q_leader - q_follower) / joint_scale_rad`，要求 `state_fields` 包含 `joint_pos`。
- [0, 1] 内的夹爪 `g` 转为 `2g - 1`。设备没有夹爪通道时保持最后发送动作，reset 后为打开 +1，避免接管时掉落物体。
- 遥操作 `episode_end` 产生 `truncated` 并设置 `info["human_episode_end"]`；`reset()` 归位期间的按键被丢弃。读取失败通过 `DeviceFault` 停止 actor。
- 远程机器人连接后才报告部件，因此提前用 `--robot-teleop-action-mode {cartesian_delta,joint_abs,joint_delta}` 声明；无夹爪设备增加 `--no-robot-teleop-has-gripper`。连接时与实际部件核对，不符抛 `DeviceFault`。
- 已配置的遥操作设备必须存在才能连接机器人；部署 YAML 应去掉 `teleop:`。

无硬件 fake 遥操作演练，以下为机器人 YAML。`on_end: loop` 重复脚本，让回合持续结束：

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
python examples/deploy_real_robot.py --checkpoint runs/.../checkpoints/checkpoint_<step>.pt \
    --algo rlpd_hybrid --real_robot.remote ROBOT_PC:50051 --real_robot.token T \
    --episodes 5 --max-steps-per-episode 100
```

serl checkpoint 使用 `--algo rlpd`，hil_serl 用 `rlpd_hybrid`，`real_world_bc` 运行用 `real_world_bc`，残差用 `residual_hil_serl` / `residual_hil_serl_hybrid`，SiLRI 用 `silri`，见下节。双臂增加 `--env-backend real_robot_bimanual`，参见 [`docs/bimanual.md`](bimanual.md#deploy)。训练目录中 `checkpoints/` 旁的 `config.json` 是动作与观测约定的依据：

- 部署环境从训练记录读取 `action_mode`、`gripper_in_action`、`num_joints`、`joint_limits`、`xyz_scale_m`、`rot_scale_rad`、`joint_scale_rad`、`state_fields`、`image_size`、`control_hz`、`max_episode_steps`。自动恢复观测包装器 `obs.frame_stack`，堆叠 `classifier_image_keys`，以及 `convert_obs_rotation`。`--real_robot.*` 仅提供 `robot`、`remote`、`token`、`home_joints`、`safety_box`、`reset_settle_s` 等连接或现场字段。
- CLI 约定字段与训练不符会报 `ValueError`：`--real_robot.* contradicts the training run's real_robot config ...`。**限制**：显式值恰好等于 dataclass 默认值时无法区分是否指定，因此对 `joint_abs` 运行给出 `--real_robot.action_mode cartesian_delta` 会静默继续使用训练的 `joint_abs`。
- 旧运行或 `franka_real` 没有 `real_robot` 配置时会警告并回退 CLI，此时不验证约定。
- 守护进程不可达、watchdog 或安全问题等 `DeviceError` 会打印消息与提示，以退出码 2 结束。
- 守护进程 `--jpeg-quality` 未保存在运行配置中，部署时需手工采用训练相同值或同样不设置。

<a id="residual-rl-residual_hil_serl"></a>

### 残差 RL（`residual_hil_serl`）

`residual_hil_serl` 在冻结基础策略上训练有界残差 `clamp(base + scale * u, -1, 1)`，基础策略由 `--base-provider zero | checkpoint:<path>?algo=<name> | act:<path>` 指定。沿用 hil_serl 的 actor / learner / eval、示范缓冲区和人工介入；人工覆盖总动作，可选 BC 项拉近残差与人工纠正。`--residual-variant continuous` 使用 `ResidualRLPD`，作用于全部维度；`hybrid` 使用 `ResidualRLPDHybrid`，仅作用机械臂，夹爪为 DQN。只有 actor、eval、deploy 构建基础策略；部署从元数据重建，算法名为 `--algo residual_hil_serl` 或 `residual_hil_serl_hybrid`。见 [`docs/residual.md`](residual.md)。

<a id="hil-rl-silri"></a>

### HIL-RL（`silri`）

`silri` 实现 HIL-RL / SiLRI（arXiv 2512.24288）。actor-critic 的 actor 在状态相关拉格朗日乘子约束下靠近人工介入的高斯模型，一致人工行为加强约束，不一致行为则让 critic 更多主导。它共用 hil_serl 的 actor / learner / eval、遥操作设备和示范 pkl；learner 在示范上预训练，随介入刷新行为模型，actor 为每次接管前的 transition 标记 `intervention_onset`。部署使用 `--algo silri`，见 [`docs/silri.md`](silri.md)。

<a id="demo-and-snapshot-contract-sidecar"></a>

### 示范与快照约定附属文件

hil_serl learner 的在线与示范快照在每个 pkl 旁生成 `<name>.contract.json`，例如 `transitions_N.pkl` 对应 `transitions_N.contract.json`；`--demo-dataset-paths` 也同时读取附属文件。文件是一行按键排序的 JSON：

```json
{"action_mode": "joint_abs", "arm_action_dim": 7, "gripper_in_action": true, "joint_limits": [[-2.8, 2.8], ...], "joint_scale_rad": 0.05, "num_joints": 7, "rot_scale_rad": 0.1, "state_fields": ["joint_pos", "gripper_pos"], "xyz_scale_m": 0.02}
```

加载时与环境约定比较，浮点数使用 `allclose`。不符抛 `ValueError: ... recorded action contract does not match the env's (differs in ['joint_limits']) ...`。早于缩放和限制字段的旧附属文件仅检查已有键并警告。没有附属文件的 pkl 仅检查动作维度，并警告 `... has no action-contract metadata (<name>.contract.json missing) ...`。

<a id="end-to-end-try-out-with-the-fake-robot"></a>

### fake 机器人端到端演练

单机无硬件流程已在项目容器验证，训练时间短，策略尚未训练好。机器人 PC 侧准备 `type: franka` 的 fake `robot.yaml`，配置 `arm_backend: fake`、`gripper: {backend: fake}`、名为 `wrist` 的 fake 相机、关节限制、`max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}` 及上文 fake 遥操作，参考 robot_infra 示例 YAML：

```bash
python -m robot_infra.server --config robot.yaml --bind 127.0.0.1:50061 --token T \
    --jpeg-quality 90 --watchdog-s 5 --no-preflight
```

learner 与 actor 使用两个终端。`hil.yaml` 基于前述预设，修改为 `remote: 127.0.0.1:50061`、`token: T`，使用 `joint_abs` 及对应限制：

```bash
COMMON="--config hil.yaml --teleop-device robot --robot-teleop-action-mode joint_abs \
    --log-dir runs --log-type none --exp-name smoke --batch-size 16 --learning-starts 32 \
    --buffer-size 2000 --checkpoint-freq 10 --sync-host 127.0.0.1 --sync-port 6055 --publish-freq 10"
python examples/train_real_world.py hil_serl --role learner $COMMON
python examples/train_real_world.py hil_serl --role actor $COMMON
```

Ctrl-C 停止两个进程。learner 写出 `runs/smoke/config.json` 和 `runs/smoke/checkpoints/checkpoint_<update>.pt`，随后部署最后一个 checkpoint：

```bash
python examples/deploy_real_robot.py --checkpoint runs/smoke/checkpoints/checkpoint_<update>.pt \
    --algo rlpd_hybrid --real_robot.remote 127.0.0.1:50061 --real_robot.token T \
    --episodes 2 --max-steps-per-episode 15
```

每回合打印 `[deploy] episode=... steps=15 ... health_ok=True`，最后打印 `[deploy] summary ...`。真机使用相同命令，将守护进程放在机器人 PC，改为 `--bind 0.0.0.0:50051`，连接真实 `ROBOT_PC:50051`，`--sync-host` 设置为可访问的 learner 地址，并采用真实 `control_hz`。

SERL 状态或图像双进程真机训练使用相同参数：

```bash
python examples/train_real_world.py serl --role learner ...
python examples/train_real_world.py serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.num_joints 7 \
    --obs.rgb wrist --obs.image-size 128 128 ...
```

serl 的真机参数仅通过 `--print-config` 检查，尚未实际执行 serl 训练。

RLinf 从 [`configs/rlinf/realworld_rlgarden_franka.yaml`](../../configs/rlinf/realworld_rlgarden_franka.yaml) 开始。Ray 环境 worker 必须能导入本包；RLinf 会终止环境节点上运行的所有 roscore。

**RLinf** 通过 Gymnasium 注册访问本包。真机配置的环境块设置 `env_type: real` 及 `init_params.id: "real_world_rl.integrations.rlinf.register:RLGardenReal-v0"`，RLinf 的 `RealWorldEnv` 随后通过 `gym.make` 构建每 worker 一台机器人的 `RealRobotEnv`，以 `override_cfg` 为配置。环境块及风险见 [`configs/rlinf/realworld_rlgarden_franka.yaml`](../../configs/rlinf/realworld_rlgarden_franka.yaml)，约定测试见 `tests/test_rlinf_registration.py`。rl-garden 的 Pattern-A RLinf 适配器 `rl_garden/integrations/rlinf/{sac_actor,sac_rollout}.py` 运行 RLinf 自身 worker，没有真机或图像路径，无法驱动本包。

训练使用 `python examples/train_real_world.py serl --role ...`，双进程启动形式见该文件模块说明。遥操作演示与录制见 [`docs/teleop.md`](teleop.md)，HIL-SERL 移植能力及原因见 [`docs/hil-serl-roadmap.md`](hil-serl-roadmap.md)。
