<!-- English source: docs/teleop.md -->

<a id="teleoperation"></a>

# 遥操作

[中文入口](../README.zh-CN.md) · [English](../teleop.md)

`real_world_rl` 中的人工介入由 `TeleopSource` 读取操作员设备，`InterventionWrapper` 将策略动作替换为人工动作；`examples/` 的示范脚本使用同一套输入源。

<a id="the-teleopsource-contract"></a>

## `TeleopSource` 接口约定

定义位于 `real_world_rl.teleop` 的 `teleop/base.py`：

```python
@dataclass(frozen=True)
class TeleopReading:
    action: np.ndarray          # float32 (A,), env-normalized [-1, 1], gripper last when has_gripper
    intervened: bool            # the operator is driving; action only matters then
    episode_end: bool = False   # operator's episode-end button
    events: Mapping[str, bool] = {}   # success / failure / rerecord edges that fired, else empty
    timestamp: float = 0.0
    human_action_mask: Any = None  # actual human-controlled dimensions
    override_mask: Any = None      # overridden dimensions including hold supplements

class TeleopSource(ABC):
    action_mode: str            # "cartesian_delta" | "joint_delta" | "joint_abs"
    arm_action_dim: int
    has_gripper: bool
    def start(self) -> None     # open the device (default no-op)
    def poll(self) -> TeleopReading            # abstract
    def reset(self) -> None     # after an env reset: drop latched events and sticky state
    def close(self) -> None     # release the device (safe to call twice)
    @classmethod
    def from_env(cls, env, **kwargs)           # registry factory; default cls(**kwargs)
```

- 构建包装器时检查 `action_mode`、`arm_action_dim`、`has_gripper` 与环境动作约定。若不匹配，例如在 `joint_delta` 机器人上使用笛卡尔 SpaceMouse，会在启动设备前抛出 `ValueError`。
- `poll()` 必须传播设备故障，吞掉异常会把策略动作误录成人工示范。
- `as_teleop_source(obj)` 原样返回 `TeleopSource`；对于符合鸭子类型的旧对象，包装为 `LegacyTeleopAdapter`。旧对象的 `poll()` 返回值须含 `.intervened`、`.action`、`.episode_end`；`reset()`、`close()` 和 `action_mode` / `arm_action_dim` / `has_gripper` 为可选，后三者默认 cartesian_delta / 6 / True。

<a id="devices"></a>

## 设备

`TeleopInterventionWrapper(env, device=..., **kwargs)` 经 `make_teleop_source` 构建输入源，HIL 用 `--teleop-device` 选择。HIL、残差、SiLRI 从参数派生 kwargs，再将 `--teleop-options-yaml PATH` 映射合并覆盖，用于 CLI 没有单独参数的选项。例如本机 GELLO：

```yaml
# gello.yaml, for --teleop-device gello --teleop-options-yaml gello.yaml
port: /dev/serial/by-id/usb-FTDI_...
joint_ids: [1, 2, 3, 4, 5, 6, 7]
joint_offsets_rad: [...]   # from gello_calibrate
joint_signs: [1, 1, -1, 1, 1, 1, 1]
gripper_id: 8
gripper_open_rad: ...
gripper_closed_rad: ...
hold_s: 0.0
```

第二 SpaceMouse 可用 `device_index: 1`，以及 deadzone、invert_axes；franka_real 栈的 leader_cartesian 可指定服务 leader 的地址，见[笛卡尔 leader](#leader-arm-on-a-cartesian-robot-leader_cartesian)：

```yaml
# leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
remote_address: <robot-host>:<port>
```

组合事件设备时，此文件配置运动半部。wrapper 将“最后发出的夹爪命令”传给 robot、leader_arm、gello、leader_cartesian、spacemouse、bimanual 及其组合运动源，让无夹爪 leader 保持原命令、防止掉物，也让 SpaceMouse 从当前命令接管。动作末尾每臂一夹爪维，getter 每臂一项，单臂取 0，bimanual 各取对应项。未声明 num_arms 默认为一臂；声明的臂数或 arm_names 顺序不符即拒绝。

| device | 输入源 | 读取内容 | 动作空间 | 说明 |
|---|---|---|---|---|
| `pico` | `PicoTeleopSource` | ZeroMQ SUB Pico VR | cartesian_delta，6＋夹爪 | zmq_url、hand、pos_scale、rot_scale、twist_limit、intervention_threshold，见下文格式 |
| `spacemouse` | `SpaceMouseTeleopSource` | 本机 SpaceMouse 经设备层，速度输入到 tracker | cartesian_delta，6＋夹爪 | 0 接管/交还，1 夹爪；device_index=0、deadzone=0.05、invert_axes=()、poll_interval_s=0.005、read_timeout_s，及四个 lead-cap/target-speed 参数；pyspacemouse>=2,<3 在 start 导入 |
| `spacemouse_twist` | `SpaceMouseTwistTeleopSource` | 同设备原始 twist，仅模拟示例 | cartesian_delta，6＋夹爪 | 动作直接进环境，移动即介入，hold_s 延长；0 切换 reset 后为开的夹爪，1 结束；默认 device_index=0、deadzone=1e-3、invert_axes=()、poll_interval_s=0.005、hold_s=0.0、read_timeout_s |
| `robot` | `TeleopPartSource` | 环境机器人 teleop 部件，本地或远程 | cartesian_delta / joint_abs / joint_delta | teleop_contract、joint_limits、joint_scale_rad、read_timeout_s；机器人拥有部件，close 不断开；CLI 声明 robot-teleop-action-mode 与夹爪 |
| `fake` | `ScriptedTeleopSource` | 测试脚本 | 遵循环境、含臂数 | 无 readings 时永不介入，末行重复 |
| `gello` | `GelloTeleopSource` | actor 本机 Dynamixel GELLO | joint_abs / joint_delta | 端口、ID、符号偏移等及 hold_s、read_timeout_s、onset、arm_tolerance_rad、ramp_step_rad；gello extra 的 SDK 在 start 导入 |
| `leader_arm` | `LeaderArmPartSource` | robot teleop 作为关节 leader | joint_abs / joint_delta | robot 参数加 onset、arm-tolerance-rad、ramp-step-rad |
| `keyboard` | `KeyboardTeleopSource` | pynput 桌面键盘 | 遵循环境，动作零 | t 接管、h 交还、Esc 结束，s/f/r 结果，q 结束并发 operator_quit，recorder/actor 回合后停；单独只适合事件测试；需 reward-collection、桌面，start 时导入 |
| `stdin` | `StdinTeleopSource` | 本进程终端，cbreak，close 恢复，支持 SSH | 遵循环境，动作零 | 与 keyboard 同键；fd 默认 stdin、poll_interval_s=0.02；EOF 下轮抛 DeviceFault |
| `composite` | `CompositeTeleopSource` | 运动与事件源组合 | 运动源空间 | motion、events、对应 kwargs，intervention_from 默认 events 或 motion；HIL 由 event-device 构建 |
| `leader_cartesian` | `LeaderArmCartesianSource` | Panda FK 关节转笛卡尔 | cartesian_delta，6＋夹爪 | 见[笛卡尔 leader](#leader-arm-on-a-cartesian-robot-leader_cartesian) |
| `bimanual` | `BimanualTeleopSource` | 每臂独立源 | 先双臂再双夹爪 | device_per_arm、per_arm_kwargs、common_kwargs、intervention_from=any 或 all；见[双臂](#two-arms-bimanual) |

<a id="spacemouse"></a>

### SpaceMouse

cartesian_delta 环境（real_robot 或 franka_real）使用终端标签，因为 Compact 两个按钮均已占用：

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device spacemouse --teleop-event-device stdin \
    --output-path demos/task_20.pkl --num-successes 20
```

HIL actor 用相同两参数。源通过设备层 SpaceMouseTeleop 打开，TeleopConfig(backend=spacemouse)，轮询线程供 read，轴映射 `[-y, x, z, -roll, -pitch, -yaw]`，invert_axes 翻转 0–5 输出。与 leader_cartesian 共用 CartesianTargetTracker，见[映射](#mapping)。

| 输入 | 作用 |
|---|---|
| 按钮 0 | 接管，再按交还；移动推杆不算接管 |
| 按钮 1 | 驱动时切换夹爪 ±1 |
| 推杆 | 驱动时输入速度 |
| stdin / keyboard 的 s/f/r、Esc、q | 成功/失败/重录、结束、退出 |

- **速度：**介入步请求 `twist * target_speed * dt`，dt 为两次介入间隔，接管时为零，因此无跳变。平移在基坐标系，旋转向量左乘；全幅为 0.05 m/s、0.3 rad/s，tracker 同样限制对角总速度。各轴 deadzone 默认满幅 0.05，忽略静止噪声。
- **领先：**命令领先实测最多 0.05 m / 0.3 rad，多余丢弃。请求从 command 而非旧 request 开始，滞后不会积累；释放后停在现有 command，最多再走一个 cap。环境尺度至少 0.05/0.3，默认 0.02/0.1 会在构建时拒绝。
- **交还：**保持实测姿态；recorder 共用 tracker，持续固定保持而非零 twist。请求越界记录 WORKSPACE HOLD，输入或跟踪故障也交还；重新按 0。两次介入间隔超过 0.3 s 时保持实测，下一步从该处继续。
- **夹爪：**每次接管从 wrapper 的 last_gripper_action_getter 原值开始，闭合夹爪不会因接管张开；按钮 1 才切换，交还时无效。
- **按钮按下沿只生效一次：**设备层锁存直到读取，短于控制步也不漏，长按一次，跨 reset 长按不算新按。
- **事件源 t/h 被忽略并警告：**否则会与按钮 0 状态冲突，只有按钮 0 控制接管。
- start 打开设备才导入 SDK，构建无需 pyspacemouse，缺失报 SdkMissing 和安装提示。

运行记录与 leader_cartesian 同一 teleop_control tracker 块。旧 `--teleop-event-device spacemouse` 已移除，旧命令/预设报错并指向 `--teleop-device spacemouse --teleop-event-device stdin`。

robot YAML 的 `--teleop-device robot` / backend spacemouse 仍经机器人读取设备层旧语义：原始 twist 直传，推杆或按钮活动即介入、hold_s 延长，0 切换 sticky gripper。所有路线 SpaceMouse 按钮均不结束回合，须事件设备 Esc。

<a id="robot-teleop-part---teleop-device-robot"></a>

### 机器人遥操作部件（`--teleop-device robot`）

`--env-backend real_robot` 的 hil_serl 可从机器人自身遥操作部件接收介入，对应 YAML 的 `teleop:` 段，可为 SpaceMouse、`gello` 主臂或 `fake`，由本地或守护进程服务。主臂建议用 `--teleop-device leader_arm`，见[主臂](#leader-arms-gello)。适配器 `TeleopPartSource` 位于 `real_world_rl/teleop/part.py`，将笛卡尔或关节 `joint_abs` / `joint_delta` 动作映射到环境 `[-1, 1]` 空间，并转发 `success` / `failure` / `rerecord` 事件。映射、`--robot-teleop-action-mode` 声明和 YAML 示例见 README“动作约定”与“遥操作（`--teleop-device robot`）”。

<a id="leader-arms-gello"></a>

### 主臂 GELLO

硬件、标定、机器人 YAML、示范录制，以及 `hil_serl`、`residual_hil_serl`、`silri` 的端到端说明见 [gello.md](gello.md)。首次上手的配置向导、BC、ACT 和部署见[教程](tutorial/00-overview.md)。本节介绍输入源。

主臂报告绝对关节位置。`LeaderArmPartSource` 位于 `real_world_rl/teleop/leader_arm.py`，按 `TeleopPartSource` 的方式映射到环境关节动作模式：`joint_abs` 按 `joint_limits` 归一化；`joint_delta` 采用 `(q_leader - q_follower) / joint_scale_rad`，每步按一个 `joint_scale_rad` 饱和。此外增加操作员接管瞬间的起始策略。GELLO 有两种读取方式：

- `--teleop-device leader_arm`：GELLO 作为机器人 PC 上守护进程服务的 YAML `teleop:` 部件，经 `RemoteRobot` 读取。服务命令为 `python -m robot_infra.server --config robot.yaml ...`。`python -m robot_infra.setup` 会把该段及标定结果写入工作站 YAML；也可手动写入，例如 `robot_infra/docs/robot.franka.example.yaml`。`python -m robot_infra.devices.backends.teleops.gello_calibrate --port ... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8` 会打印单台设备的 `options` 块：

  ```yaml
  teleop:
    name: gello
    backend: gello
    options:
      port: /dev/serial/by-id/usb-FTDI_...
      joint_ids: [1, 2, 3, 4, 5, 6, 7]
      joint_offsets_rad: [...]    # from gello_calibrate
      joint_signs: [1, 1, -1, 1, 1, 1, 1]
      gripper_id: 8
      gripper_open_rad: ...
      gripper_closed_rad: ...
      intervention: toggle        # or always (pure teleop)
      event_device: /dev/input/by-id/...   # optional evdev pedal/keyboard on the robot PC
  ```

- `--teleop-device gello`（`GelloTeleopSource`）：GELLO 连接 actor 机器，使用同一设备层后端，以前述选项作为输入源 kwargs；训练 CLI 通过 `--teleop-options-yaml` 提供这些值。

通过守护进程运行 HIL 的示例如下。从臂使用 `joint_delta`；对于主臂，`--robot-teleop-action-mode` 只需声明关节空间，在远程连接前描述部件：

```bash
python examples/train_real_world.py hil_serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.action_mode joint_delta \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-onset ramp --teleop-event-device keyboard
```

接管起始由 `--teleop-onset` 控制，对应输入源参数 `onset`：

- `ramp`（默认）：`joint_abs` 发出的目标为 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`，参数 `--teleop-ramp-step-rad` 默认 0.05；按关节限制归一化，因此记录动作就是实际命令。这些介入步骤带 `info["teleop_events"]["teleop_syncing"] = True`。归档保留记录，训练排除同步专家标签。`joint_delta` 标记初始追赶，直到主从差距首次进入 `joint_scale_rad`；同次接管后续饱和运动仍为普通教学。
- `arm`：只有 `|q_leader - q_follower|_inf <= arm_tolerance_rad` 后开关才生效，`--teleop-arm-tolerance-rad` 默认 0.1。此前步骤不算介入，每秒最多记录一次逐关节误差；应移动主臂对齐从臂。就绪后持续到开关释放，每次环境 reset 解除就绪。

`q_follower` 来自环境最近观测的 `joint_pos`，所以 `state_fields` 必须包含 `joint_pos`。默认切换取部件的 `intervening`，即机器人 PC 上由 evdev `event_device` 驱动的 `intervention: toggle`，或 `always`；也可由事件设备提供：

`--teleop-event-device keyboard` 或 `stdin` 将运动源包为 CompositeTeleopSource：运动来自 teleop-device，介入状态及 episode_end/success/failure/rerecord/operator_quit 来自 actor 本机事件源。t 接管、h 交还、Esc 结束、s/f/r 结果、q 退出。关节源仍对该状态应用 onset，部件自己的 intervention 被忽略。SpaceMouse 则由按钮 0 自管接管，仅使用事件源标签，见 [SpaceMouse](#spacemouse)。

<a id="leader-arm-on-a-cartesian-robot-leader_cartesian"></a>

### 笛卡尔机器人上的主臂（`leader_cartesian`）

GELLO 报告关节角；仅接受笛卡尔速度增量的环境可通过 Panda 正向运动学接收它们。`LeaderArmCartesianSource` 位于 `real_world_rl/teleop/leader_cartesian.py`，设备名 `device="leader_cartesian"`，将主臂关节转为请求末端姿态，再由 `CartesianTargetTracker`（`real_world_rl/teleop/cartesian_tracker.py`）产生 follower 的 twist，见[映射](#mapping)。`examples/presets/gello_cartesian.yaml` 是单臂 50 Hz 预设。

<a id="why"></a>

#### 使用原因

旧 `franka_real` 栈使用 SERL HTTP 桥接，仅提供 `/pose`（绝对笛卡尔阻抗目标）、`/open_gripper` / `/close_gripper`、`/jointreset`、`/startimp` / `/stopimp` 和 `/getstate`。它没有流式关节命令，关节空间主臂源 `--teleop-device leader_arm` / `gello` 无法直接控制，见[主臂](#leader-arms-gello)。`FrankaRealEnv` 接受 7 维动作，即基坐标系速度增量 6 维加夹爪，因此在 actor 侧转换：

```
GELLO joints q --PandaKinematics--> T* = (p*, R*) --vs follower TCP (p, R)--> twist in [-1, 1]
```

同一输入源也适用于 `cartesian_delta` 模式的 `real_robot`，例如守护进程使用 `arm_backend: serl_http`。

<a id="pieces"></a>

#### 组成

| 组件 | 位置 | 作用 |
| --- | --- | --- |
| `PandaKinematics` | `real_world_rl/kinematics/panda.py` | 基于 numpy 的 Craig 改进 DH 正向运动学，提供 `flange_pose(q)`、`ee_pose(q)`（xyz、xyzw 四元数）、`ee_transform(q)`（4×4）。末端默认 Franka Hand，沿法兰 z 轴偏移 0.1034 米并旋转 -pi/4，由 `ee_offset_m`、`ee_yaw_rad` 配置。 |
| `LeaderArmCartesianSource` | `real_world_rl/teleop/leader_cartesian.py` | 遥操作输入源，任意 7 关节 `joint_abs` 遥操作部件均可作为主臂。 |
| `FrankaRealEnv.last_tcp_pose` | `real_world_rl/envs/franka_real/env.py` | 最近 reset/step 测量的 TCP 姿态 `O_T_EE`，xyz 加 xyzw 四元数，作为从臂姿态。 |
| 占位机器人 YAML | `robot_infra/docs/robot.gello_station.example.yaml` | 通过机器人守护进程提供主机上的 GELLO 服务。 |

<a id="the-stub-robot-daemon-on-the-robot-host"></a>

#### 机器人主机上的占位守护进程

GELLO 连接运行 SERL 桥接的机器人主机。占位机器人配置为 `type: franka`、`arm_backend: fake`，无夹爪和相机，将 GELLO 作为 `teleop:` 部件，经机器人守护进程服务：

```bash
# on the robot host, next to the SERL bridge
pip install -e "robot_infra[gello]"          # + [gello-events] for an evdev pedal/keyboard
python -m robot_infra.devices.backends.teleops.gello_calibrate --port /dev/serial/by-id/usb-FTDI_... \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8            # paste the printed options into the yaml
python -m robot_infra.server --config robot.gello_station.yaml --bind 0.0.0.0:50052
```

fake 机械臂不占用硬件，也从未被发送控制命令，因此不会与桥接争用机器人。端口应与主机上真实机器人守护进程不同。

<a id="actor-side"></a>

#### Actor 侧

必须且只能指定一个 leader：

- remote_address="host:port"，可选 remote_token：start 连接 stub RemoteRobot，close 断开。
- robot_yaml 路径：本机加载同一 stub YAML，本机连接 GELLO，start 时连接。
- leader 部件/解析器：调用方拥有，供测试或定制。

follower 位姿、尺度、安全盒来自 wrapper 深处环境：FrankaRealEnv.last_tcp_pose、cfg.action_scale、cfg.safety_box_low/high；real_robot 为最新 state.tcp_pose、动作契约 xyz_scale_m/rot_scale_rad、配置 safety_box。None 表示源不查盒，环境与设备层仍裁剪。可用 follower_pose_getter、pos_scale_m、rot_scale_rad、safety_box 覆盖。尺度必须至少等于 lead caps，否则构建拒绝；默认 0.02/0.1 需扩大至预设的 0.05/0.3（或 franka-real action-scale-pos/rot），或减小 cap。

```python
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

env = TeleopInterventionWrapper(
    franka_real_env,
    device="leader_cartesian",
    remote_address="robot-host:50052",
    mapping="anchored",
)
```

训练 CLI 选 leader_cartesian，用 teleop-options-yaml 将映射合并到运动源 kwargs：

```yaml
# station.yaml (actor side)
remote_address: robot-host:50052   # the stub daemon above; or robot_yaml: <path> for a local GELLO
mapping: anchored                  # or absolute
gripper_binarize_threshold: 0.5
```

```bash
python examples/train_real_world.py hil_serl --role actor --env-backend franka_real ... \
    --teleop-device leader_cartesian --teleop-options-yaml station.yaml
```

默认 intervention always 每步驱动，适合纯遥操作/示范。HIL 可用 robot-host 的 toggle + event_device，或 actor 侧 stdin/keyboard 的 t/h，见[leader](#leader-arms-gello)。

<a id="mapping"></a>

#### 映射

real_robot 的 cartesian_delta 发 `measured + action * scale`，阻抗后端将其作平衡点。每步从实测重新算的零 twist 会把平衡点锚到漂移位置（首次实机 24 s 漂移 137 mm）。每臂改用 CartesianTargetTracker 保留目标：

- hold pose 在 reset 和交还采样一次，然后无人驱动时固定。
- 持续 command pose，接管时沿用现有 hold，无跳变；按四个 tracker 速度/上限（0.05 m/s、0.3 rad/s、0.05 m、0.3 rad）推进并拉回距实测 cap 内，多余丢弃不排队，leader 突然往返不会积累超调。

归一化动作使 follower 指向 command：

```
a_pos = (p_cmd - p) / pos_scale_m
a_rot = rotvec(R_cmd R^T) / rot_scale_rad
```

这是环境 step 的逆，平移 `p + a_pos * pos_scale_m`，旋转 `from_rotvec(a_rot * rot_scale_rad) * R`，左乘意味着基坐标系。尺度不小于 cap 时动作不超 [-1,1]；否则抛 CommandOutOfRange，不静默裁剪。交还、输入/跟踪失败、回合结束均在实测处 hold。两次介入轮询相隔超过 0.3 s 当输入丢失，保持、警告间隔，下轮重新锚定；时钟倒退抛 MissedControlDeadline。请求越安全盒时记录 WORKSPACE HOLD 并返回未介入，直到交还再接管。

recorder 的 CartesianHold 共用源 tracker，以固定姿态动作保持，每步最多一个 cap。无 leader_cartesian/spacemouse 时独立 tracker，cap 为 min(0.05 m, xyz_scale_m) / min(0.3 rad, rot_scale_rad)，偏离很远时饱和逐步回位，不报越幅错误。

leader_cartesian/spacemouse 运行将 tracker 版本 cartesian_tracker_v1 与四值写入实验 teleop_control，缺失或值不同的示范不接受。契约标 tracker 而非设备，相同参数的 GELLO/SpaceMouse 数据可混用。

mapping 决定请求姿态：

- anchored 默认：每次从未介入到介入及 reset 后首次接管时，保存 command `(p_c0, R_c0)` 与 leader `(p_l0, R_l0)=FK(q_0)`，之后分别叠加基坐标系平移和旋转：

  ```
  p* = p_c0 + (p_l - p_l0)
  R* = (R_l R_l0^T) R_c0
  ```

  接管保持 hold，不猛跳。leader 腕转让 follower 绕自身 TCP 原地转，距离无关；若用 `FK(q) T_l0^{-1} T_f0` 组合反而绕基原点，约平移 `|p_f0| * theta`。FK 误差只通过接管后变化进入；leader/follower 可不同姿态，交还后可重新握持如离合器，leader 肘/null-space 不参与笛卡尔命令。
- absolute：`T* = FK(q)`，follower 去 leader 末端位置，需 FK 与真实 O_T_EE 一致并在接管前将 leader 放近，否则按目标速度从 hold 向 leader 走。

除非已标定并实机核对，优先 anchored。

<a id="gripper"></a>

#### 夹爪

leader 宽度 w 属于 [0,1]，1 为开；超过 gripper_binarize_threshold 为 +1，否则 -1，默认 0.5 即扳机过半闭合。精确 ±1 是 hybrid 离散 critic 的清晰标签，franka_real 的 a>0.5 开判定也明确。阈值 None 则连续输出 2w-1。无夹爪通道保留环境最后夹爪命令，需 last_gripper_action_getter，如 robot 路线。

接管时扳机与最后命令不符（开为 action>0），源持续最后命令直到扳机跨到对应状态才跟随。最后命令由 wrapper getter 提供，因此释放扳机接管不会立即掉物。

<a id="verify-the-dh-table-on-the-robot"></a>

#### 在真机验证 DH 表

`PandaKinematics` 符合 Franka 公布的参考姿态：`q = 0` 时法兰位于 `(0.088, 0, 0.926)`，z 轴朝下；准备姿态 `[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]` 时约为 `(0.307, 0, 0.590)`。但尚未与真机比较。使用 `mapping="absolute"` 前，应在准备姿态及工作区内分散的两三个腕部旋转姿态，将计算结果与桥接自身报告的姿态比较：

```python
import numpy as np
from scipy.spatial.transform import Rotation

from real_world_rl.envs.franka_real.bridge_client import FrankaBridgeClient
from real_world_rl.kinematics import PandaKinematics

state = FrankaBridgeClient("http://robot-host:5000").get_state()  # q and pose (O_T_EE)
position, quat = PandaKinematics().ee_pose(state["q"])
pose = np.asarray(state["pose"])
print("position error [m]:", np.linalg.norm(position - pose[:3]))
print("rotation error [rad]:", (Rotation.from_quat(quat) * Rotation.from_quat(pose[3:]).inv()).magnitude())
```

预期误差应低于约 1 毫米和 0.01 rad。若沿工具轴或绕轴存在固定偏移，表示 Desk 的末端设置与 Franka Hand 默认值不同，可传入 `kinematics=PandaKinematics(ee_offset_m=..., ee_yaw_rad=...)`。随关节配置变化的误差更可能来自 DH 表。`anchored` 对两类误差都较不敏感。

<a id="two-arms-bimanual"></a>

### 双臂（`bimanual`）

`--env-backend real_robot_bimanual` 的动作布局为 `[left arm, right arm, left gripper, right gripper]`，见 [bimanual.md](bimanual.md)，会拒绝单臂设备 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`spacemouse_twist`、`pico`。`--teleop-device bimanual` 对应 `real_world_rl/teleop/bimanual.py` 的 `BimanualTeleopSource`，在每臂自身 `RealRobotEnv` 上构建单臂源，独立读取机器人和从臂，再组装双臂动作。`--teleop-arm-device` 选择逐臂设备：

| `--teleop-arm-device` | 每臂输入源 | 说明 |
| --- | --- | --- |
| `leader_arm`（默认） | 该臂机器人遥操作部件上的 `LeaderArmPartSource` | 由该臂守护进程读取 YAML `teleop:` GELLO；`--robot-teleop-action-mode` 在连接前声明两个部件，`--teleop-onset` 参数应用于每臂。 |
| `robot` | 该臂机器人部件上的 `TeleopPartSource` | 任意遥操作部件，映射规则同 `--teleop-device robot`。 |
| `gello` | `GelloTeleopSource` | 每臂一套连接 actor 的 GELLO，各自需要 `port` 和标定，详见下文。 |

`leader_cartesian` 暂不能作为逐臂设备，它仅将一个主臂转换为一条笛卡尔从臂的速度增量。

双臂输入源请求介入：默认 `intervention_from: any` 在任一操作员介入时令 `intervened` 为真；`all` 要求两人同时介入。`any` 下未介入操作员对应的机械臂保持：`joint_abs` 使用从臂关节位置，增量模式用零；夹爪保持上次发送动作。因此空闲主臂和扳机不会移动从臂或打开夹爪。`episode_end` 和事件在两臂间取 OR；`info["teleop_events"]` 同时包含 `<arm>_<event>` 和普通 `<event>`（`success`、`teleop_syncing` 等）。

使用 `--teleop-event-device keyboard` 或 stdin 将 bimanual 作为组合运动半部：t/h 同时接管/交还双臂，各臂 onset 仍有效；键盘提供结束、结果和退出键。

`bimanual` 的 `--teleop-options-yaml` 可以使用按臂名分段的映射加公共键。`intervention_from` 属于双臂源自身，其余公共键传给每臂，各臂段覆盖公共值：

```yaml
# bimanual.yaml, for --teleop-device bimanual --teleop-arm-device gello
intervention_from: any
hold_s: 0.0
left:
  port: /dev/serial/by-id/usb-FTDI_..._LEFT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
right:
  port: /dev/serial/by-id/usb-FTDI_..._RIGHT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
```

使用 `--teleop-arm-device gello` 时，也可直接提供由 `python -m robot_infra.setup` 生成的 robot_infra 工作站 YAML，即 `robots: {left: ..., right: ...}`。各机器人 `teleop.options` 和 `hold_s` 成为该臂 kwargs，按名称匹配环境 `arm_names`。单臂 `--teleop-device gello` 也可使用单机器人工作站或机器人 YAML。逐臂使用 `leader_arm` 或 `robot` 时，各守护进程自己读取 GELLO，因此该路径拒绝工作站 YAML。

通过两个守护进程执行 HIL：

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

`--teleop-device fake` 也声明环境臂数，可以无需操作员演练双臂 actor。

<a id="interventionwrapper"></a>

## `InterventionWrapper`

实现位于 `real_world_rl/envs/wrappers/intervention.py`。`InterventionWrapper(env, source)` 接受 `TeleopSource`；`TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)` 按设备名构建，或适配 `teleop` 对象。两者都通过 `.source` 暴露输入源，别名为 `.teleop`。

`step(action)` 先轮询输入源，将输入源人工掩码与选定部件取交集。人工掩码非空时，在策略设备上组合人工与空闲部件动作，得到 `(1, A)` float32 张量。`idle_behavior="hold"` 保持未选部件，`"policy"` 在这些部件保留策略提议。实际人工控制与保持覆盖分别记录：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作；**仅介入时存在**。 |
| `policy_action` | 原样接收的策略提议动作。 |
| `intervened` | Python bool：执行了操作员动作。 |
| `intervention_onset` | Python bool：本步介入且本回合上一步未介入。 |
| `human_episode_end` | Python bool：操作员结束回合按钮。 |
| `teleop_events` | 已触发 `success`/`failure`/`rerecord` 边沿及主臂追赶 `teleop_syncing` 的 `dict`；**仅有事件时存在**。 |
| `human_action_mask` | 实际由人工驱动的维度，含同步阶段的来源。 |
| `override_mask` | 替换策略提议的维度，包含保持补充动作。 |

`episode_end` 使 `step()` 返回 `truncated=True`，保持 `terminated` 为假，因此 actor 循环会 reset，同时保留非终止语义。

事件顺序：

- 构建时：验证输入源与环境约定，然后调用 `source.start()`。
- `reset(**kw)`：先 `env.reset()`，后 `source.reset()`，丢弃机器人归位期间的按键。最后夹爪动作恢复打开（+1），接管起始检测器重置。
- `close()`：先 `source.close()`，再 `env.close()`；输入源关闭失败时也会关闭环境。

包装器记住最后发送的夹爪动作并裁剪到 `[-1, 1]`。有夹爪的环境配合无夹爪遥操作设备时保持该动作，避免掉落物体。

<a id="adding-a-device"></a>

## 新增设备

继承 `TeleopSource`，声明动作空间、实现 `poll()` 并注册：

```python
from real_world_rl.teleop import TeleopReading, TeleopSource, register_teleop_source

@register_teleop_source("joystick")
class JoystickTeleopSource(TeleopSource):
    action_mode = "cartesian_delta"
    arm_action_dim = 6
    has_gripper = True

    def __init__(self, port: str = "/dev/input/js0") -> None: ...

    def start(self) -> None: ...      # open the device
    def poll(self) -> TeleopReading: ...
    def reset(self) -> None: ...      # drop latched presses
    def close(self) -> None: ...
```

随后 `make_teleop_source("joystick", env, port=...)` 调用 `JoystickTeleopSource.from_env(env, port=...)`，默认执行 `cls(**kwargs)`。可重写 `from_env`，通过 `real_world_rl.action_contract.get_env_action_contract(env)` 读取环境动作约定。未知名称会抛 `ValueError` 并列出已注册项。使用名称前必须完成注册，因此下游软件包应在构建环境前导入其模块，或通过自身入口点暴露。内置输入源按名称首次使用时懒加载，导入 `real_world_rl.teleop` 不会引入 `zmq` 或 `pyspacemouse`。

动作必须处于环境归一化 `[-1, 1]` 空间，夹爪在最后。Pico 等以物理单位输出速度增量的笛卡尔设备，可参考示范脚本的归一化处理。

<a id="eetwist-teleop-interface-pico"></a>

## EETwist 遥操作接口 Pico

`PicoTeleopSource` 封装仓库设备层中的 `EETwistTeleOpWrapper`，实现位于 `robot_infra/robot_infra/teleop/utils/telo_op_control_twist.py`。设备名仅为 `pico`；包装器通过 ZeroMQ `SUB` 套接字接收设备数据，Pico 服务应向下列地址发布 JSON 数组：

```python
tcp://*:7777
```

客户端通过 `--zmq-url` 连接，例如：

```bash
--zmq-url tcp://192.168.6.2:7777
```

`PicoTeleopSource.reset(*, episode_end_pressed=False)` 将标记转发给包装器，避免仍按住的回合结束按钮再次触发；`examples/record_teleop_wsrl.py` 使用该能力。

<a id="pico-data-format"></a>

### Pico 数据格式

Pico 数据可为单手 `13` 个值或双手 `26` 个值。双手时，前 `13` 个属于左手，后 `13` 个属于右手。

每只手的字段：

| 索引 | 含义 |
| --- | --- |
| `0:3` | 手部位置。 |
| `6:10` | 手部四元数，`[x, y, z, w]`。 |
| `10` | 夹爪按钮。 |
| `11` | 绑定 / 离合按钮。 |
| `12` | 回合结束按钮。 |

选择当前参考姿态时按住 `bind`，此时速度增量为零。释放后将手部姿态变化转换为末端速度增量。

`EETwistTeleOpWrapper.poll()` 返回 `TeleOpSample`，`PicoTeleopSource` 转为 `TeleopReading`，其中 `action` 为速度增量加夹爪，同时包含 `intervened`、`episode_end`：

```python
action       # twist + gripper
twist        # shape (6,)
gripper      # scalar gripper command
bind_pressed
episode_end
intervened
```

没有新设备数据时，`twist=0`，`gripper` 保留前值，`intervened=False`。

<a id="basic-usage"></a>

## 基本用法

可视化遥操作：

```bash
python examples/run_teleop_demo.py \
  --env-id PegInsertionSidePegOnly-v1 \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

不连接环境，仅打印收到的遥操作状态：

```bash
python robot_infra/robot_infra/teleop/examples/run_twist_input_inspector.py \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

相同演示可使用 SpaceMouse，通过 `--spacemouse-index`、`--spacemouse-deadzone`、`--spacemouse-invert-axes`、`--spacemouse-hold-s` 调整，也可使用空闲 `fake` 输入源：

```bash
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse_twist
```

模拟示例使用 `spacemouse_twist` 原始 twist：推杆移动即介入，动作直传环境，按钮 0 切夹爪、1 结束回合。仅模拟可用，真机使用 `--teleop-device spacemouse`，见 [SpaceMouse](#spacemouse)。

`run_teleop_demo.py` 创建 ManiSkill 环境，通过 `make_teleop_source` 构建输入源，轮询动作，将 Pico 物理速度增量按控制器动作范围归一化，然后步进环境。SpaceMouse 动作已经归一化。

<a id="recording-demos-on-the-real-robot"></a>

## 在真机上录制示范

`examples/record_real_demos.py` 在 HIL 使用的 `real_robot`、`real_robot_bimanual` 或 `franka_real` 环境中录制遥操作回合，采用与 actor 相同的环境和遥操作参数，输出 learner 通过 `--demo-dataset-paths` 加载的示范 pkl，仅保留介入步骤。见 [recording-demos.md](recording-demos.md)；GELLO 示例见 [gello.md](gello.md#5-recording-demos)。

<a id="recording-wsrl-data"></a>

## 录制 WSRL 数据

`examples/record_teleop_wsrl.py` 将遥操作回合录制为 WSRL 数据生成器使用的相同 H5 格式。

示例：

```bash
python examples/record_teleop_wsrl.py \
  --output-path demos/peg_teleop_wsrl.h5 \
  --env-id PegInsertionSidePegOnly-v1 \
  --obs-mode rgb \
  --control-mode pd_ee_twist \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

`--output-path` 必须是 H5 文件路径，建议以 `.h5` 结尾。文件以写入模式打开，同路径已有文件会被覆盖。

只有 `intervened=True` 的样本会步进环境并保存。`intervened=False` 时持续轮询设备，不记录 transition。

操作员回合结束按钮，即 Pico 对应按钮或 `spacemouse_twist` 按钮 1，会结束当前回合。每个回合后，终端打印回合长度、`max_episode_steps`、成功标记与回报，再询问是否保存。

<a id="important-arguments"></a>

### 重要参数

- `output_path`：写入的 H5 数据集文件。
- `device`：`pico`、`spacemouse_twist` 或 `fake`，设备专属参数与演示相同。
- `dt`：遥操作轮询周期，单位秒，默认 `1 / 30`，约 `30 Hz`；值越小，轮询与步进越快。
- `env_id`：ManiSkill 环境 ID。
- `obs_mode`：观测模式，通常为 `state`、`rgb` 或 `rgbd`。
- `include_state`：视觉观测是否附加本体状态。
- `control_mode`：需匹配末端速度控制器，通常为 `pd_ee_twist`。
- `sim_backend`、`render_backend`：ManiSkill 模拟与渲染后端。
- `reward_mode`：传给环境的奖励模式。
- `robot_uids`：环境机器人 ID。
- `fix_box`、`fix_peg_pose`、`peg_density`、`debug_pose_vis`：自定义插销环境选项。
- `env_kwargs_json`：传递其他环境 kwargs 的入口。
- `end_on_env_done`：开启后，环境终止、截断或成功均可结束回合。默认 false，仅遥操作结束事件停止录制。
- `pos_scale`、`rot_scale`、`twist_limit`：可选速度映射覆盖值。
- `intervention_threshold`：判定人工输入是否活动的速度增量范数阈值。

保存的 H5 含 `traj_*` 分组，字段如下：

```text
obs
actions
rewards
terminated
truncated
```

该文件可通过 `--offline_dataset` 提供给 WSRL、CQL 或 Cal-QL 离线训练。

真机数据若仅用于离线预训练，无模拟环境、无评估，可使用 rl-garden 的 `examples/pretrain_offline.py cql` 或 `examples/pretrain_offline.py calql`，生成独立离线 CQL/Cal-QL checkpoint。若需由 WSRL 离线到在线流程恢复，使用 `wsrl` 子命令。入口从 H5 推断观测与动作规格，生成之后可加载到兼容实时环境中在线微调的预训练 checkpoint。完整流程见 `rl-garden/` 子模块的 `rl-garden/docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env`。


<a id="partial-control-and-training-provenance"></a>

## 局部控制与训练来源

使用 `--teleop-selected-parts` 选择部件，教程工作站为 `arm.arm` / `arm.gripper`，双臂为 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`。默认 `--teleop-idle-behavior hold` 保持未选部件，`policy` 将其交给策略。空选择使用输入源可用部件。没有夹爪通道的输入源不提供专家夹爪标签。支持连续无夹爪 HIL；混合/离散分支需要夹爪。同步记录照常执行和归档，但其训练投影没有专家监督。掩码 BC/SiLRI/残差损失、显式结果和持久记录见[真机实验](real-world-experiments.md)。
