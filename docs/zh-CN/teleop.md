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

`TeleopInterventionWrapper(env, device=..., **kwargs)` 通过 `make_teleop_source(device, env, **kwargs)` 构建输入源。hil_serl 的设备来自 `--teleop-device`。

hil_serl、残差方法和 SiLRI 先从参数生成输入源 kwargs：`robot` / `leader_arm` 使用机器人部件声明与限制，`gello` / `leader_arm` 读取接管参数；随后合并 `--teleop-options-yaml PATH` 的 YAML 映射，文件中的值覆盖推导值。可用它提供 CLI 未暴露的设备选项，例如本地 GELLO：

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

第二台 SpaceMouse 可设置 `device_index: 1`；在 `franka_real` 栈使用 `--teleop-device leader_cartesian` 时，可指定提供主臂服务的机器人守护进程，见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)：

```yaml
# leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
remote_address: <robot-host>:<port>
```

使用 `--teleop-event-device` 时，该文件配置组合输入源中的运动部分。

包装器将“最后发送的夹爪动作”传给 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`bimanual`，以及以这些输入源构建的组合运动部分。因此主臂没有夹爪通道时可以保持夹爪，避免掉落物体。环境动作末尾按动作约定每臂一个夹爪维度，getter 返回每臂一项；单臂输入源读取第 0 项，`bimanual` 将对应项分给各臂。输入源未声明 `num_arms` 时默认为单臂。包装器会拒绝臂数不同，或声明的 `arm_names` 与环境名称和顺序不一致的输入源。

| `device` | 输入源 | 读取方式 | 动作空间 | 说明 |
| --- | --- | --- | --- | --- |
| `pico` | `PicoTeleopSource` | 经 ZeroMQ `SUB` 套接字读取 Pico VR 控制器，使用 `EETwistTeleOpWrapper`。 | cartesian_delta，6 维加夹爪。 | kwargs：`zmq_url`、`hand`、`pos_scale`、`rot_scale`、`twist_limit`、`intervention_threshold`。数据格式见下文。 |
| `spacemouse` | `SpaceMouseTeleopSource` | 通过设备层 `SpaceMouseTeleop` 读取本机设备。 | cartesian_delta，6 维加夹爪。 | kwargs：`device_index=0`、`deadzone=1e-3`、`invert_axes=()`、`poll_interval_s=0.005`、`hold_s=0.0`、`read_timeout_s`。需要 `pyspacemouse>=2,<3`，在 `start()` 时导入。 |
| `robot` | `TeleopPartSource` | 读取 `real_robot` 环境背后机器人的遥操作部件，即 YAML 的 `teleop:`，可本地或经守护进程服务。 | cartesian_delta、joint_abs 或 joint_delta，映射至环境空间。 | kwargs：`teleop_contract`、`joint_limits`、`joint_scale_rad`、`read_timeout_s`；hil_serl 对应 `--robot-teleop-action-mode`、`--no-robot-teleop-has-gripper`。部件归机器人所有，`close()` 保持其连接。 |
| `fake` | `ScriptedTeleopSource` | 脚本化读数；未提供 `readings` 时模拟从不介入的操作员。 | 遵循环境动作约定，包括臂数。 | 可传 `readings=[TeleopReading, ...]`，最后一条读数重复。 |
| `gello` | `GelloTeleopSource` | 经设备层 `gello` 遥操作后端读取本机 Dynamixel 总线。 | 环境的 joint_abs 或 joint_delta，关节映射与 `robot` 相同。 | kwargs：`port`、`joint_ids`、`joint_offsets_rad`、`joint_signs` 等设备选项，见[主臂](#leader-arms-gello)；另有 `hold_s`、`read_timeout_s`、`onset`、`arm_tolerance_rad`、`ramp_step_rad`。依赖 `dynamixel-sdk`，属于 robot_infra 的 `gello` 可选组，在 `start()` 时导入。 |
| `leader_arm` | `LeaderArmPartSource` | 将 `real_robot` 背后的遥操作部件视为关节主臂，通常为守护进程服务的 GELLO。 | 环境的 joint_abs 或 joint_delta。 | 包含 `robot` 的 kwargs，另有 `onset`、`arm_tolerance_rad`、`ramp_step_rad`；hil_serl 对应 `--teleop-onset`、`--teleop-arm-tolerance-rad`、`--teleop-ramp-step-rad`。 |
| `keyboard` | `KeyboardTeleopSource` | 经 `pynput` 读取本机键盘。 | 遵循环境约定，动作始终为零。 | Space 切换介入，Esc 结束回合，`s` / `f` / `r` 触发 `success` / `failure` / `rerecord`。作为事件设备，单独使用仅适合事件测试。需要 `pynput`，来自 `reward-collection`，在 `start()` 导入。 |
| `composite` | `CompositeTeleopSource` | 一个运动输入源加一个事件输入源。 | 采用运动输入源空间。 | kwargs：注册名 `motion`、`events`，以及 `motion_kwargs`、`events_kwargs`、`intervention_from="events"`，后者也可设为 `"motion"`。hil_serl 通过 `--teleop-event-device` 构建。 |
| `leader_cartesian` | `LeaderArmCartesianSource` | 用 Panda 正向运动学把关节主臂转换为笛卡尔速度增量，用于 `franka_real` 栈。 | cartesian_delta，6 维加夹爪。 | 见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)。 |
| `bimanual` | `BimanualTeleopSource` | `real_robot_bimanual` 每臂一个单臂输入源，由 `--teleop-arm-device` 选择。 | 环境空间，先两臂动作，再两夹爪。 | kwargs：`device_per_arm`、`per_arm_kwargs`、`common_kwargs`、`intervention_from="any"`，也可 `"all"`。见[双臂](#two-arms-bimanual)。 |

<a id="spacemouse"></a>

### SpaceMouse

`SpaceMouseTeleopSource` 通过设备层 `SpaceMouseTeleop` 打开设备，使用 `TeleopConfig(backend="spacemouse")`，轮询线程向 `read()` 供数；再由 `TeleopPartSource` 映射到环境空间。其语义沿用设备层：

- 操纵帽驱动 `[-1, 1]` 笛卡尔速度增量，pyspacemouse 状态轴映射为 `[-y, x, z, -roll, -pitch, -yaw]`；`invert_axes` 翻转指定输出索引 0–5 的符号。
- 速度增量范数超过 `deadzone` 或按钮按住时介入；最后活动后继续保持 `hold_s` 秒。
- 按钮 0 切换锁存夹爪，启动及每次 `reset()` 后打开；`+1` 打开、`-1` 闭合。按钮 1 触发 `episode_end`。
- 按钮在按下边沿触发。reset 会清空记忆的按钮状态，因此重置时仍按住的按钮会再次触发。
- 包装器调用 `start()` 打开设备时才导入 SDK，因此未安装 `pyspacemouse` 也可构建输入源；缺少 SDK 时抛出带安装提示的 `SdkMissing`。

hil_serl 使用 `--teleop-device spacemouse`。机器人 YAML 路径使用 `--teleop-device robot` 并配置 `backend: spacemouse`，通过机器人及守护进程读取同一设备，语义相同。

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

- 默认 `ramp`：`joint_abs` 输出 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`，`--teleop-ramp-step-rad` 默认 0.05；随后按关节限制归一化，因此记录动作为实际命令。介入步骤带 `info["teleop_events"]["teleop_syncing"] = True`，仍进入示范缓冲区，可由下游过滤。`joint_delta` 本身已按每步 `joint_scale_rad` 饱和，无额外变化。
- `arm`：仅当 `|q_leader - q_follower|_inf <= arm_tolerance_rad` 时切换生效，`--teleop-arm-tolerance-rad` 默认 0.1。此前步骤保持未介入，最多每秒记录一次逐关节误差，需手动对齐。就绪后保持到释放切换信号，每次 reset 解除就绪。

`q_follower` 来自环境最近观测的 `joint_pos`，所以 `state_fields` 必须包含 `joint_pos`。默认切换取部件的 `intervening`，即机器人 PC 上由 evdev `event_device` 驱动的 `intervention: toggle`，或 `always`；也可由事件设备提供：

`--teleop-event-device keyboard` 或 `spacemouse` 用 `CompositeTeleopSource` 包装运动设备：运动来自 `--teleop-device`，介入切换和 `episode_end` / `success` / `failure` / `rerecord` 来自 actor 机器上的事件设备。键盘使用 Space、Esc、`s` / `f` / `r`。主臂仍对切换应用自身起始策略，部件原有 `intervention` 设置被覆盖。事件设备为 `spacemouse` 时，介入跟随操纵帽或按住按钮的活动状态。

<a id="leader-arm-on-a-cartesian-robot-leader_cartesian"></a>

### 笛卡尔机器人上的主臂（`leader_cartesian`）

GELLO 报告关节角；仅接受笛卡尔速度增量的环境可通过 Panda 正向运动学接收它们。`LeaderArmCartesianSource` 位于 `real_world_rl/teleop/leader_cartesian.py`，设备名 `device="leader_cartesian"`，将主臂关节转换为末端目标，并输出使从臂靠近目标的速度增量。

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

#### actor 侧

输入源必须且只能指定一个主臂来源：

- `remote_address="host:port"`，可选 `remote_token`：为占位守护进程构建 `RemoteRobot`，由包装器调用 `start()` 连接，在 `close()` 时断开。
- `robot_yaml=<path>`：在本机加载同类占位 YAML，用于 GELLO 连接 actor 的情况，`start()` 时连接。
- `leader=<part or resolver>`：调用方持有的遥操作部件，用于测试或自定义配置。

从臂姿态和单步缩放来自环境：沿包装器链获取 `FrankaRealEnv.last_tcp_pose` 和 `cfg.action_scale`；`real_robot` 笛卡尔环境则使用 `last_observation["state"]["tcp_pose"]` 及动作约定 `xyz_scale_m` / `rot_scale_rad`。可用 `follower_pose_getter=`、`pos_scale_m=`、`rot_scale_rad=` 覆盖。

```python
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

env = TeleopInterventionWrapper(
    franka_real_env,
    device="leader_cartesian",
    remote_address="robot-host:50052",
    mapping="anchored",
)
```

训练 CLI 通过 `--teleop-device leader_cartesian` 选择，kwargs 以 YAML 映射由 `--teleop-options-yaml` 传入，合并至运动输入源参数：

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

默认 `intervention: always` 时操作员每步都控制，适合纯遥操作或示范采集。HIL 可在机器人主机配置 `intervention: toggle` 与 `event_device`，或使用 actor 侧 `--teleop-event-device keyboard`，见[主臂](#leader-arms-gello)。

<a id="mapping"></a>

#### 映射

机械臂动作是从测得的从臂姿态 `(p, R)` 到目标 `(p*, R*)` 的基坐标系速度增量，使用环境归一化单位：

```
a_pos = clip((p* - p) / pos_scale_m, -1, 1)
a_rot = clip(rotvec(R* R^T) / rot_scale_rad, -1, 1)
```

该过程是 `FrankaRealEnv.step` 的逆运算：位置更新为 `p + a_pos * pos_scale_m`，旋转更新为 `from_rotvec(a_rot * rot_scale_rad) * R`。旋转左乘，因此旋转向量位于基坐标系。未饱和动作直接到达目标；饱和动作在默认超过 0.02 米或 0.1 rad 时走一个完整步幅，下次轮询再从新实测姿态重算，避免误差累积与漂移。

`mapping` 选择目标：

- 默认 `anchored`：每次介入起始，即未介入后的首次介入轮询，以及 reset 后首次介入，保存从臂姿态 `(p_f0, R_f0)` 和主臂姿态 `(p_l0, R_l0) = FK(q_0)`。随后将主臂基坐标系位移应用于从臂，平移与旋转解耦：

  ```
  p* = p_f0 + (p_l - p_l0)
  R* = (R_l R_l0^T) R_f0
  ```

  起始步骤输出零速度增量，避免接管抖动。无论主从臂位置相距多远，主臂腕部旋转都会让从臂绕自身 TCP 原地旋转。若使用组合式 `FK(q) T_l0^{-1} T_f0`，则会绕基坐标原点摆动，产生约 `|p_f0| * theta` 的平移。FK 误差只通过起始后变化量影响结果。主从臂姿态可不同，像离合器一样在介入之间重新握持主臂；与所有笛卡尔命令相同，忽略主臂肘部和零空间运动。
- `absolute`：`T* = FK(q)`，从臂前往主臂末端位置，要求 `PandaKinematics.ee_pose` 与机器人 `O_T_EE` 一致，验证方式见下文。接管前主臂需靠近从臂，否则从臂会以饱和速率前往主臂姿态；默认缩放下 10 Hz 对应 0.2 米/秒和 1 rad/秒。

在主臂完成标定并与真机对照验证前，优先使用 `anchored`。

<a id="gripper"></a>

#### 夹爪

主臂宽度目标 `w` 位于 `[0, 1]`，1 表示打开；当 `w > gripper_binarize_threshold` 时输出 `+1` 打开，否则 `-1` 闭合。默认阈值 0.5，即扳机超过半程时关闭。精确 +-1 为混合离散夹爪 critic 提供清晰标签，`franka_real` 在 `a > 0.5` 时打开，因此执行明确。`gripper_binarize_threshold=None` 改为连续 `2w - 1`。缺少夹爪通道时保持最后发送动作，需要 `last_gripper_action_getter`，与 `--teleop-device robot` 相同。

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

`--env-backend real_robot_bimanual` 的动作布局为 `[left arm, right arm, left gripper, right gripper]`，见 [bimanual.md](bimanual.md)，会拒绝单臂设备 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`pico`。`--teleop-device bimanual` 对应 `real_world_rl/teleop/bimanual.py` 的 `BimanualTeleopSource`，在每臂自身 `RealRobotEnv` 上构建单臂源，独立读取机器人和从臂，再组装双臂动作。`--teleop-arm-device` 选择逐臂设备：

| `--teleop-arm-device` | 每臂输入源 | 说明 |
| --- | --- | --- |
| `leader_arm`（默认） | 该臂机器人遥操作部件上的 `LeaderArmPartSource` | 由该臂守护进程读取 YAML `teleop:` GELLO；`--robot-teleop-action-mode` 在连接前声明两个部件，`--teleop-onset` 参数应用于每臂。 |
| `robot` | 该臂机器人部件上的 `TeleopPartSource` | 任意遥操作部件，映射规则同 `--teleop-device robot`。 |
| `gello` | `GelloTeleopSource` | 每臂一套连接 actor 的 GELLO，各自需要 `port` 和标定，详见下文。 |

`leader_cartesian` 暂不能作为逐臂设备，它仅将一个主臂转换为一条笛卡尔从臂的速度增量。

操作员覆盖整个动作。默认 `intervention_from: any` 下，任一臂操作员介入便置 `intervened` 为真；`all` 要求两臂都介入。`any` 下，未介入的臂执行保持动作：`joint_abs` 使用从臂当前关节，增量模式使用零，夹爪使用最后发送动作，因此闲置主臂或扳机不会移动从臂或打开夹爪。各臂 `episode_end` 和事件做逻辑或；`info["teleop_events"]` 同时包含 `<arm>_<event>` 和普通 `<event>`，例如 `success`、`teleop_syncing`。

使用 `--teleop-event-device keyboard` 时，双臂源作为组合中的运动部分，一个键盘同时切换两臂，各臂起始策略仍生效，并提供回合结束与结果按键。

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

`step(action)` 先轮询输入源。`reading.intervened` 为真时，用操作员动作替换 `action`，转为与策略动作同设备的 `(1, A)` float32 张量。写入字段如下：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作，**仅介入时存在**。 |
| `policy_action` | 原样保存策略提出的动作。 |
| `intervened` | Python bool，是否执行人工动作。 |
| `intervention_onset` | Python bool，本步介入而回合上一步未介入。 |
| `human_episode_end` | Python bool，操作员结束回合按钮。 |
| `teleop_events` | 触发的 `success` / `failure` / `rerecord` 边沿 `dict`；主臂在限速起始步骤添加 `teleop_syncing`。**仅有事件时存在**。 |

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
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse
```

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

操作员回合结束按钮，即 Pico 对应按钮或 SpaceMouse 按钮 1，会结束当前回合。每个回合后，终端打印回合长度、`max_episode_steps`、成功标记与回报，再询问是否保存。

<a id="important-arguments"></a>

### 重要参数

- `output_path`：写入的 H5 数据集文件。
- `device`：`pico`、`spacemouse` 或 `fake`，设备专属参数与演示相同。
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
