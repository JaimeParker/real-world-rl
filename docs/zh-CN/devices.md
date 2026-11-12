<a id="the-device-layer"></a>

# 设备层

无需 torch 的设备层位于 `real_world_rl/{devices,server,discovery,setup,supervise,doctor}`，提供带类型的 Arm、Gripper、Camera、Teleop 接口约定、硬件后端、设备侧安全限制、发现与诊断功能。它不导入 `rl_garden` 或学习子包。`rwrl` 通过统一解析器提供这些命令。设备层曾名为 `robot_infra`（更早是独立的 `rlgarden-robot-infra`），历史记录仍保留。`learning` 扩展提供 gym 环境和训练功能。[机器人守护进程](daemon.md) 为这些设备提供服务，并向客户端暴露 RemoteRobot。

<a id="device-commands-in-rwrl"></a>

## rwrl 中的设备命令

| 命令 | 用途 | 参考 |
|---|---|---|
| `rwrl setup` | 发现并调试工作站。 | [CLI](cli.md) |
| `rwrl discover` | 探测已连接硬件，不连接机器人。 | [CLI](cli.md) |
| `rwrl doctor` | 检查主机、环境和工作站设备。 | [CLI](cli.md#robot-host-diagnosis-and-repair) |
| `rwrl daemon` | 通过 gRPC 提供工作站机器人服务。 | [CLI](cli.md)、[守护进程](daemon.md) |
| `rwrl supervise` | 通过心跳监控子进程。 | [CLI](cli.md)、[进程监督](daemon.md#supervising-a-single-process-robot-run) |
| `rwrl teleop calibrate` | 输出标定后的设备 YAML。 | [CLI](cli.md) |

<a id="install-and-setup"></a>

## 安装与配置

按[安装](tutorial/01-install.md)、[配置工作站](tutorial/02-setup.md)和[启动 Franka](tutorial/03-franka-bringup.md)操作。SpaceMouse 的安装与权限见[遥操作指南](teleop.md#spacemouse-installation-and-linux-permissions)。

<a id="station-file-reference"></a>

## 工作站文件参考

示例在机器人 PC 上运行，工作目录为仓库根目录，已激活项目环境。

机器人配置是带 `type` 键的 YAML 映射，也可嵌套在顶层 `robot:` 下。未知键会被拒绝，并列出有效键。省略的键使用下方默认值。完整注释示例见 [`examples/stations/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml) 和 [`examples/stations/robot.piper.example.yaml`](../../examples/stations/robot.piper.example.yaml)。[配置向导](tutorial/02-setup.md)会为 Franka 工作站生成此文件。

多机器人工作站（如双 Franka 双臂工作站）在 `robots:` 下按名称列出各机器人的映射；省略 `name` 时使用映射键。每台机器人可带 `daemon:` 块（单机器人文件也支持），守护进程在解析机器人配置前读取并移除此块：

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    # ... any robot key, as above
    daemon: {bind: 0.0.0.0:50051, token: null}   # default bind 0.0.0.0:50051, no token
  right:
    type: franka
    robot_ip: 172.16.1.2
    daemon: {bind: 0.0.0.0:50052, token: null}
```

`rwrl daemon` 随后为每台机器人启动一个守护进程，`rwrl doctor` 逐台检查；`--robot <name>` 可选择其中一台。两套 GELLO、三台相机的完整示例见 [`examples/stations/station.bimanual.example.yaml`](../../examples/stations/station.bimanual.example.yaml)。`Robot.from_yaml` 仅读取不带 `daemon:` 块的单机器人文件。

上层包可通过 `parse_station`、`load_station` 或 `load_station_with_env` 的 `sections={name: validator}` 注册额外的机器人映射段。这些段与 `daemon:` 一样描述机器人，但不属于设备配置：加载器在调用 `RobotConfig.from_dict` 前移除已注册键，验证其值，并将返回值保存在 `StationRobot.sections`。设备层不解释段内内容。未指定段（`sections=None` 或 `{}`）时，这些加载器及 `load_config`、`load_config_with_env` 严格解析，不执行入口点发现。real_world_rl 的段（`teleop_profiles`）由 `real_world_rl.station.load_station` 和 `load_station_with_env` 注册；所有 `rwrl` 命令都使用它们，包括 daemon、setup、discover 和设备 doctor。遇到未知键时，错误会注明加载器及已注册段名。未注册的未知键仍会失败；单机器人映射和 `robots:` 映射都支持段。配置向导输出设备配置，不输出上层段。

**Franka Panda + Franka Hand + 一台 RealSense**

```yaml
type: franka
name: franka                  # default "franka"
arm_backend: franky           # franky (default) | serl_http (legacy ROS1 bridge) | fake
robot_ip: 172.16.0.2          # FCI IP set in Desk (default 172.16.0.2)
# bridge_url: http://127.0.0.1:5000   # serl_http only: URL of the legacy Flask bridge
realtime: ignore              # ignore (default) | enforce (requires a PREEMPT_RT kernel)
relative_dynamics_factor: 0.2 # franky velocity/accel/jerk scale for move_to_* (default 0.2)
home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # 7 values (default shown)
safety_box: [0.3, -0.3, 0.05, 0.7, 0.3, 0.6]  # [xmin, ymin, zmin, xmax, ymax, zmax] m, base frame of tcp_pose (O_T_EE)
joint_limits:                 # 7 [lo, hi] pairs in rad; home_joints must lie inside
  - [-2.8, 2.8]
  - [-1.7, 1.7]
  - [-2.8, 2.8]
  - [-3.0, -0.1]
  - [-2.8, 2.8]
  - [0.0, 3.7]
  - [-2.8, 2.8]
max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.1}  # servo step cap from the measured state
claim: true                   # daemon takes a per-host exclusive lock while connected
gripper:                      # null = no gripper
  backend: franky             # franky (default) | serl_http | fake
  max_width_m: 0.08
  mode: binary                # binary (default) | continuous
  binary_threshold: 0.5       # binary mode: an action > threshold opens
  shares_arm_session: false   # the Franka Hand opens its own connection
cameras:                      # default: []
  - name: wrist
    backend: realsense        # realsense (default) | fake
    serial: ""                # "" = use the only attached camera; required when >1 is attached
    width: 640
    height: 480
    fps: 30
    enable_depth: false
```

**AgileX Piper（CAN）**

```yaml
type: piper
name: piper
arm_backend: pyagxarm         # pyagxarm (default) | piper_sdk | fake
can_interface: can0           # default can0
bitrate: 1000000              # must match the interface (default 1000000)
firmware_profile: null        # pyagxarm only: default | v183 | v188 | v189; null probes the arm
home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 values, rad
safety_box: [0.1, -0.3, 0.1, 0.5, 0.3, 0.5]   # m, base frame; tcp_pose is the FLANGE pose (no tool offset)
joint_limits: [[-2.6, 2.6], [0.0, 3.1], [-2.9, 0.0], [-1.7, 1.7], [-1.2, 1.2], [-2.0, 2.0]]  # rad
max_delta_per_step: {xyz_m: 0.02, joint_rad: 0.05}
claim: true
gripper:
  backend: pyagxarm           # must match the arm backend: pyagxarm | piper_sdk
  max_width_m: 0.07
  mode: continuous            # Piper default
  binary_threshold: 0.5
  shares_arm_session: true    # required: the Piper gripper borrows the arm's CAN session
cameras: []
```

安全限制与遥操作（均可选，三个限制键均从机器人 PC 的 YAML 读取）：

- `safety_box`：`[xmin, ymin, zmin, xmax, ymax, zmax]`，单位米，使用 `read_state()["tcp_pose"]` 坐标系（Franka 为 `O_T_EE`，Piper 为**法兰**，不是夹爪尖端）。仅限制位置，不改变姿态。
- `joint_limits`：每个关节一对 `[lo, hi]`，单位弧度。`home_joints` 必须在范围内。
- `max_delta_per_step`：`xyz_m` 和/或 `joint_rad`，限制伺服目标与实测状态的最大距离。仅用于伺服，`move_to_*` 不受步长限制。
- 机器人 PC 上的 YAML 是权威配置。越界的 `servo_*` 目标会被**裁剪**并计数（`safety_clips`）；越界的 `move_to_*` 目标会以 `SafetyViolation` **拒绝**，不发送命令（`safety_rejects`）。守护进程、`send_action` 和本地 API 均如此，见[安全规则](#safety-rules)。
- `teleop:` 将操作员输入设备作为机器人部件加入（见[步骤 7](devices.md#add-a-teleop-device)）。

说明：

- `pyagxarm` 和 `piper_sdk` 夹爪必须设置 `shares_arm_session: true`，`franky` 和 `serl_http` 则禁止这样设置。不匹配会在构建时抛出 `ValueError`。
- 使用 `arm_backend: piper_sdk` 时还须设置 `gripper.backend: piper_sdk`。此后端忽略 `firmware_profile`，始终以 1 Mbit/s 打开总线，因此仅核对接口的 `bitrate`。
- `serl_http` 不支持关节空间控制，且需要运行旧版桥接服务（见[旧模块](#legacy)）。

可不接触硬件，仅构建机器人以检查文件：

```python
from real_world_rl.devices import Robot

robot = Robot.from_yaml("robot.yaml")   # builds parts; does not connect
print(robot.describe_text())
```

```text
franka (franka)
  arm: FrankyArm [sdk=franky, connected=False]
  gripper: FrankyGripper [sdk=franky, connected=False]
  cameras: PartGroup [sdk=None, connected=False]
    wrist: RealSenseCamera [sdk=pyrealsense2, connected=False]
```

配置 `teleop:` 后末尾会多一行，例如 `  teleop: SpaceMouseTeleop [sdk=hid, connected=False]`。

<a id="station-lookup-and-setup-semantics"></a>

### 工作站查找与配置语义

向导先探测硬件：`172.16.0.2`、`172.16.1.2`、`$ROBOT_IP`、`--robot-ip` 及 `--scan-subnet CIDR` 中各主机的 TCP 22/443/1337；RealSense 序列号；各 `/dev/serial/by-id/usb-FTDI_*` 和 `usb-ROBOTIS_*` 串口上以 57600 至 4000000 波特率广播的 Dynamixel ping；以及 SpaceMouse。随后逐臂询问机器人 IP、机械臂及夹爪后端、home 关节、相机（`1=wrist,2=front`）、遥操作设备（GELLO 串口与舵机 ID，最后一个 ID 默认作为夹爪扳机）、介入模式和守护进程端口，并内联标定每套 GELLO（与[设备约定](#add-a-teleop-device)中的标定 CLI 相同步骤；跳过时写入零偏移与 `# TODO calibrate` 注释）。最后写入带注释的工作站 YAML，运行 doctor 并输出后续命令。

每条机械臂都获得设备侧限制：Panda 数据表的 `joint_limits`（`--joint-limits`）、伺服步长上限 `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`（`--max-delta-xyz-m`、`--max-delta-joint-rad`），以及交互询问的 `safety_box`（基坐标系，单位米，建议 `[0.3, -0.3, 0.05, 0.7, 0.3, 0.6]`，`--safety-box`），安全框必须包含 home 位姿。

在机器人 PC 的仓库根目录中，激活项目环境后操作。命令见[配置教程](tutorial/02-setup.md)。

daemon、doctor 和 discovery 无需 `--station` 也可查找工作站 YAML，按以下顺序取第一个：

1. `$RWRL_STATION`
2. `./local/station.yaml`（相对当前目录；向导默认位置，`--scope local`；`local/` 已被 git 忽略）
3. `$XDG_CONFIG_HOME/rwrl/station.yaml`，默认 `~/.config/rwrl/station.yaml`（`--scope user`）

在一个发布周期内，仍会在新名称之后读取 0.1.0 之前的旧名称：在 `$RWRL_STATION` 后读取 `$ROBOT_INFRA_CONFIG`，在用户位置后读取 `~/.config/robot_infra/station.yaml`；`rwrl doctor` 会将它们报告为 `WARN` 并给出迁移命令。

`--station <yaml>` 仍优先于以上三项。

每个问题都有对应参数（`rwrl setup --help`）；逐臂参数可为每臂提供一个值，也可只给一个共用值。`--non-interactive` 遇到首个既无参数也无 `--station` 值的问题就失败；`--non-interactive --station X` 会原样重写 `X`。已有文件先备份为 `station.yaml.bak.<UTC %Y%m%dT%H%M%SZ>`，再原子替换。除非使用 `--allow-unprobed`，否则必须提供探测到的值。

<a id="discover"></a>

## 发现设备

见[配置工作站](tutorial/02-setup.md)和 [CLI](cli.md)。Discovery 仅探测，不连接机器人，也不列出 fake 后端。无工作站时主机发现输出 `none`。

<a id="doctor"></a>

## 设备诊断

分层检查、消息、修复与故障排查见 [doctor](doctor.md)，首次真实握手见[启动 Franka](tutorial/03-franka-bringup.md)。Claim 是 RobotService 在连接期间持有的主机内独占锁；普通 `Robot.from_yaml` 脚本不获取锁。`claim: false` 显式退出此机制。只有持有硬件连接的进程才算拥有者（见[预期设备](doctor.md#expected-devices)）。

<a id="add-a-teleop-device"></a>

## 添加遥操作设备

以下示例在机器人 PC 的仓库根目录、项目环境已激活时运行。应用层人工介入语义见[遥操作](teleop.md)。

`teleop:` 将操作员输入设备作为机器人部件加入。无需硬件的 fake 后端示例（每次 `read()` 推进脚本一步）：

```yaml
teleop:
  name: teleop                # the daemon serves teleop.<name>.read / .reset
  backend: fake               # fake | spacemouse | gello
  hold_s: 0.0                 # keep intervening this long after the last activity (default 0)
  options:                    # backend-private
    action_space: joint_abs   # fake: cartesian_delta (default) | joint_abs | joint_delta
    num_joints: 7             # joint spaces only
    has_gripper: true         # default true
    on_end: hold_last         # hold_last (default) | idle | loop, what happens after the script
    script:
      - {action: [0, 0, 0, 0, 0, 0, 0], active: false, hold: 2}
      - {action: [0.5, 0, 0, 0, 0, 0, 0], gripper: 0.0, active: true, hold: 2}
      - {events: {episode_end: true}}
```

一步为 `{action, gripper, buttons, events, active, hold}`；`hold` 是该步持续的读取次数，事件在第一次读取时触发。`fail_after: n` 使第 n 个样本之后的每次轮询失败，用于测试错误路径。

```python
robot = Robot.from_yaml("robot.yaml")          # local; RemoteRobot works the same way
robot.connect()
sample = robot.teleop.read()                   # blocks for a new sample, consumes the events
print(sample.seq, sample.action_space, sample.action, sample.gripper, sample.intervening, sample.events["episode_end"])
latest = robot.teleop.latest()                 # no blocking, does not consume events
robot.teleop.reset()                           # drops latched events and the hold_s hysteresis
```

`read()` 返回 `TeleopSample`；通过守护进程（`teleop.<name>.read`）传输时为包含相同键的字典：

| 字段 | 类型 | 含义 |
|---|---|---|
| `action` | `float32` ndarray | `cartesian_delta`：`(6,)`、[-1, 1] 内的归一化 twist；`joint_abs` / `joint_delta`：`(N,)` 关节值，单位弧度 |
| `action_space` | `str` | `cartesian_delta`、`joint_abs` 或 `joint_delta` |
| `gripper` | `float` 或 `None` | [0, 1] 内的归一化宽度目标；`None` 表示设备无夹爪通道 |
| `buttons` | `dict[str, bool]` | 当前电平（SpaceMouse：`button_0`、`button_1`） |
| `events` | `dict[str, bool]` | 锁存的边沿 `episode_end`、`success`、`failure`、`rerecord`（始终包含四键），加上已触发的后端特有边沿（SpaceMouse：`button_0`、`button_1` 按下）；`read()` 会清除 |
| `intervening` | `bool` | 人工正在操作（后端的 `active`，通过 `hold_s` 延长） |
| `timestamp` | `float` | 轮询机器上的 `time.monotonic()`，不可跨主机比较 |
| `seq` | `int` | 每个轮询样本均递增，重连后继续；缺号表示丢样本 |

`read(timeout_s)` 超时会抛出 `DeviceFault`；连续 5 次轮询失败会抛出可恢复的 `DeviceFault`（需重连机器人）；等待时机器人断开则抛出 `DeviceNotConnected`。它采用单消费者模型：两个客户端读取同一遥操作设备会分走彼此的事件。

`backend: spacemouse`（扩展 `spacemouse`；选项 `device_index` 默认 0、`deadzone` 默认 1e-3、`invert_axes`、`poll_interval_s` 默认 0.005）是带夹爪的 `cartesian_delta` 设备：操纵帽给出 twist `[-y, x, z, -roll, -pitch, -yaw]`，每次按键被锁存为 `button_0`/`button_1` 事件（两次读取之间按下也不丢失），按钮 0 切换保持的夹爪目标（连接时 1.0，即打开）。两个按钮都不产生 `episode_end`。应用层按钮映射见[遥操作指南](teleop.md#spacemouse)。需要 `/dev/hidraw*` 读写权限；`rwrl doctor` 以 `os:hid` 检查并输出 udev 规则安装命令。见[已知限制](#known-limitations-and-needs-hardware-verification)。

`backend: gello`（扩展 `gello`，使用 `event_device` 还需 `gello-events`）是 GELLO 主臂（通过 U2D2 或 OpenRB-150 连接的被动 Dynamixel 舵机，Protocol 2.0）：`joint_abs` 设备，每个 `joint_ids` 条目对应一个关节，设置 `gripper_id` 后具有夹爪通道。每次轮询执行一次同步读取；关节 `i` 将 `wrap((raw_rad - joint_offsets_rad[i]) * joint_signs[i])` 包裹到 `[wrap_centers_rad[i] - pi, wrap_centers_rad[i] + pi)`，连接时无需起始位姿（中心应设为 `joint_limits` 中点；Panda 第 6 关节超过 pi，需要正的中心值）。夹爪为 `clip((raw - gripper_closed_rad) / (gripper_open_rad - gripper_closed_rad), 0, 1)`，1 为打开，可用 `gripper_binarize_threshold` 二值化。`intervention: always`（默认）每次轮询都报告 `intervening`；`toggle` 跟随 `evdev` 从 `event_device`（机器人 PC 的脚踏或键盘）读取的切换键，其余键产生 `episode_end`/`success`/`failure`/`rerecord`（`event_keys`）。连接和断开时均关闭扭矩；舵机缺失或同步读取失败会使轮询失败，不返回陈旧样本。完整选项及默认值见 [`examples/stations/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml)。每套设备标定一次（关闭扭矩，手动移动主臂）；[配置向导](tutorial/02-setup.md)内联执行这些步骤并将结果写入工作站 YAML。单独执行 CLI 时会输出可粘贴的配置块：

```bash
rwrl teleop calibrate --backend gello \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --calibration-pose 0 0 0 -1.57 0 1.57 0 --joint-limits-yaml robot.yaml
```

标定会检查每个关节的符号（按提示沿机器人正方向移动一个关节），根据标定位姿在四分之一转网格上求偏移，采集扳机开闭角度，并从 YAML 的 `joint_limits` 获取包裹中心。串口需要读写权限（`dialout` 组）；doctor 通过 `os:serial` 和 `os:groups` 检查，并警告 CH340 适配器上的 brltty（`os:brltty`）及过慢的 FTDI `latency_timer`（`os:latency_timer`）。程序绝不写入舵机波特率：请用 Dynamixel Wizard 一次性修改（8 个舵机在 1000000 时约 130 Hz，在 57600 时约 25 Hz）。

<a id="architecture"></a>

## 架构

```text
Connection (ABC)                 connect()/disconnect() lifecycle, owner borrowing, SDK, health()
└── Part                         observation_features, get_observation()
    ├── Camera   [registry]      background capture thread, read()/latest()/reopen()
    │   ├── realsense, fake
    │   └── (RemoteCamera is a Part, not a Camera)
    ├── Teleop   [registry]      background poll thread, read()/latest()/reset(), latched events
    │   ├── gello, spacemouse, fake
    │   └── (RemoteTeleop is a Part, not a Teleop)
    └── ControllablePart         action_features, send_action()
        ├── Arm      [registry]  read_state, servo_cartesian/joints, move_to_pose/joints
        │   │                    (template methods, see below), stop, recover, set_control_mode, preflight
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteArm
        ├── Gripper  [registry]  open/close/move(width in [0, 1]), width; binary | continuous
        │   ├── franky, serl_http, pyagxarm, piper_sdk, fake
        │   └── RemoteGripper
        └── PartGroup            named children; connect in order, disconnect in reverse
            └── Robot  [type registry]  config -> arm (+ gripper) (+ cameras group)
                ├── FrankaRobot "franka"   (FrankaConfig)
                ├── PiperRobot  "piper"    (PiperConfig)
                └── RemoteRobot "remote"   (RemoteRobotConfig; registered by importing real_world_rl.server)

Robot PC:   Robot ── RobotService (method table, watchdog, DeviceClaim) ── GrpcTransport
GPU side:   RemoteRobot ── GrpcClient (msgpack over gRPC, token metadata, read-only retry)
```

后端按名称注册（`Arm.register("franky")`）。`Arm.backends()`、`Gripper.backends()`、`Camera.backends()` 和 `Teleop.backends()` 列出后端，`Robot.types()` 列出机器人类型。`shares_arm_session: true` 的夹爪借用机械臂会话作为 `owner`，不单独建立连接。

| 后端 | SDK（扩展） | 关节控制 | 力/力矩（`tcp_force`/`tcp_torque`） | 说明 |
|---|---|---|---|---|
| arm `franky` | `franky`（`franky`） | 支持 | 支持 | 伺服模式使用扭矩级阻抗 tracker，`move_to_*` 使用 Ruckig（仅 `IDLE`）；应用 `realtime` 和 `relative_dynamics_factor` |
| arm `serl_http` | 无（HTTP） | 不支持 | 支持 | `bridge_url` 上旧 ROS1 桥接的客户端；仅笛卡尔伺服；状态无 `joint_pos`/`joint_vel` |
| arm `pyagxarm` | `pyAgxArm`（`pyagxarm`） | 支持 | 不支持 | Piper 默认后端；探测或指定 `firmware_profile`；状态为 `tcp_pose`、`joint_pos`、`joint_vel`（法兰位姿） |
| arm `piper_sdk` | `piper_sdk`（`piper-sdk`） | 支持 | 不支持 | 厂商 SDK，固定 1 Mbit/s；`stop()` 重发实测关节以保持位置 |
| arm `fake` | 无 | 支持 | 支持 | 用于测试和 `--mock` 的确定性一阶机械臂 |
| gripper `franky` | `franky` | - | - | Franka Hand；独立会话 |
| gripper `serl_http` | 无 | - | - | 通过桥接，仅二值控制 |
| gripper `pyagxarm` / `piper_sdk` | 同机械臂 | - | - | 需要 `shares_arm_session: true` 及匹配的机械臂后端 |
| camera `realsense` | `pyrealsense2`（`realsense`） | - | - | RGB（`enable_depth` 可附加以米为单位的对齐深度） |
| `fake` gripper / camera | 无 | - | - | 测试与 `--mock` |
| teleop `spacemouse` | `hidapi==0.14.0.post4`，导入 `hid`（`spacemouse`） | - | - | 笛卡尔 twist + 保持式夹爪；`Teleop.backends()` 为 `fake`、`gello`、`spacemouse` |
| teleop `gello` | `dynamixel-sdk`（`gello`；`gello-events` 提供 `evdev`） | - | - | GELLO 主臂，`joint_abs` + 可选夹爪；关闭扭矩；evdev 切换与事件 |
| teleop `fake` | 无 | - | - | 脚本样本、`push()` 注入、`fail_after`；测试与 `--mock` |

**`Arm` 是模板方法类。** 公共方法 `servo_cartesian`、`servo_joints`、`move_to_pose` 和 `move_to_joints` 先清理输入（形状、有限数值、单位四元数），应用安全限制，再调用后端钩子 `_servo_cartesian`、`_servo_joints`、`_move_to_pose` 和 `_move_to_joints`。公共 `stop()` 同样先清除 `last_cartesian_target`，再调用后端 `_stop`。后端实现这些钩子（以及 `read_state`、`recover`、`_enter_control_mode`），绝不覆盖公共方法，因此任何通向电机的路径都经过防护。后端 `read_state()` 必须通过 `self._remember_state(state)` 返回结果：步长限制复用该缓存状态（最长 `MAX_DELTA_STATE_AGE_S` = 0.5 秒，否则重读），避免每次伺服调用都查询机器人。`Arm.from_config(config)` 用 `ArmSafety.from_config(config)` 构建后端。第三方后端省略 `_remember_state` 仍可工作，但每次受步长限制的伺服调用会多一次 `read_state()`。

<a id="safety-rules"></a>

### 安全规则

- **`disconnect()` 从不切断电源。** 释放机械臂只关闭 SDK 会话。Piper 后端从不调用 `disable()`/`reset()`/`DisableArm`/`ResetPiper`，因为失电的 Piper 会下落。`stop()` 停止运动，保持电源与制动器原状。
- **机械臂未就绪时拒绝伺服命令。** 所有后端均拒绝匹配伺服模式之外的伺服命令。franky、pyagxarm 和 piper_sdk 在故障状态也会拒绝（先调用 `recover()`）。Piper 后端还会拒绝陈旧反馈：pyagxarm 超过 0.5 秒、piper_sdk 超过 1.0 秒无新关节帧。硬件后端的 `move_to_*` 要求 `IDLE`。动作会检查形状与有限性，`tcp_pose` 四元数会检查单位范数。
- **连接会使能电机。** 连接 Piper（直接连接、doctor 或客户端通过 daemon 连接）会使能电机，`disconnect()` 后仍保持使能。运行 doctor 前清空工作区。
- **安全限制。** 机器人主机 YAML 中的 `safety_box`、`joint_limits`、`max_delta_per_step` 是权威配置。`Arm` 在公共命令方法执行这些限制，覆盖本地 `Robot`、守护进程（`arm.servo_*`、`arm.move_to_*`、`robot.send_action`、`robot.step`）及 `send_action` 等所有路径。`RemoteArm` 没有自己的防护，而是转发给 daemon。
  - `servo_*`：先以实测状态为基准将目标缩短到 `max_delta_per_step`，再裁剪进基坐标安全框/关节限制。任何被接受的修改都计一次 `safety_clips`。配置 `workspace` 时，笛卡尔目标在基坐标框裁剪前后都须满足其有向位置框和可选 SO(3) 姿态半径。工作空间拒绝会抛出 `SafetyViolation`、递增 `safety_rejects`，不发送命令。
  - `move_to_*`：目标超出安全框/关节限制时抛出 `SafetyViolation`，计一次 `safety_rejects`，不发送命令。`move_to_*` 不受步长限制。
  - `SafetyViolation` 不是设备故障：在接触后端前抛出，不会锁存故障，无需 `recover()`，保持控制模式，通过 daemon 时也不刷新 watchdog。客户端看到同一异常类。
  - `arm.health()` 报告 `safety_clips` / `safety_rejects`；`robot.describe` 以 `arm.safety` 暴露限制。构建时验证：`home_joints` 须在 `joint_limits` 内，安全框须满足 min <= max。
  - `safety_box` 和笛卡尔步长上限只限制位置。工作空间还可通过 `orientation_xyzw`（在其坐标系中）及 `max_angle_rad`（SO(3) 测地半径，避免欧拉角分支切割）限制姿态。设备侧没有姿态步长上限。关节命令不检查笛卡尔工作空间边界：这些限制不能保证关节轨迹、整臂间隙或无碰撞运动。
  - 目标检查使用 `1e-9` 数值容差。反馈检查使用工作空间键 `measured_margin_m`（默认 `0.003 m`）及 `measured_margin_rad`（默认 `0.03 rad`），均须为非负有限值。环境和遥操作 tracker 使用这些反馈裕量，但不放宽目标准入。无 workspace 的 `task_frame` 仅用于显示/采样，不增加空间约定。配置 workspace 后，两种几何信息都会在 reset 前与 daemon 核对。
- **Watchdog。** 客户端停止发送时，daemon 将正在伺服的机械臂置为空闲（见步骤 4）。
- **`DeviceClaim`。** `$XDG_CACHE_HOME/rwrl/claims/`（默认 `~/.cache/rwrl/claims/`）中的节点本地建议性文件锁，键为 `<type>:<robot_ip | can_interface | name>`。客户端连接期间由 daemon 持有，同一机器人的第二个 daemon 会收到 `DeviceBusy`。它不能防止其他主机或不获取锁的代码访问。`claim: false` 可退出。兼容一个发布周期：同时检查 `~/.cache/robot_infra/claims/` 中已有的同名锁；已被持有的旧锁会抛出 `DeviceBusy`，提示包含路径；未被持有的旧锁会与新锁一同持有直到释放。兼容机制不会创建旧目录。SpaceMouse 读取器默认也锁定 HID 端点，作为机器人遥操作部件构建时遵循相同的 `RobotConfig.claim`。其内置设备表与 PySpaceMouse 1.0.3 一致，站点本地设备表可新增或覆盖型号。

<a id="error-model"></a>

### 错误模型

设备错误继承 `DeviceError`（一种 `RuntimeError`），带有 `message`、`hint`、`device`。`str(error)` 在第二行输出提示。错误经 `to_dict()`/`from_dict()` 往返后保留，因此客户端重新抛出 daemon 的同类异常。

| `kind` | 异常类 | 典型原因 |
|---|---|---|
| `sdk_missing` | `SdkMissing` | 未安装厂商包 |
| `device_not_found` | `DeviceNotFound` | 机器人不可达、CAN 接口或相机序列号缺失 |
| `device_not_connected` | `DeviceNotConnected` | 在 `connect()` 前调用 |
| `device_already_connected` | `DeviceAlreadyConnected` | 已定义；当前后端不抛出（`connect()` 幂等） |
| `device_busy` | `DeviceBusy` | 其他进程持有 claim |
| `device_fault` | `DeviceFault`（`recoverable: bool`） | reflex、急停、无反馈、错误模式、watchdog |
| `version_mismatch` | `VersionMismatch` | libfranka 与机器人服务器版本不匹配；误装不含 trezor `hid.device()` API 的 `hid` 发行包 |
| `safety_violation` | `SafetyViolation` | 笛卡尔伺服/移动目标超出 `workspace`（位置或姿态），或 `move_to_*` 目标超出 `safety_box` / `joint_limits`；发送前拒绝，以 `safety_rejects` 计数 |
| `transport` | `TransportError`（一种 `DeviceFault`） | daemon 不可达、超时、令牌不匹配 |

<a id="command-protocol"></a>

### 命令协议

设备层原生命令 `setup`、`daemon`、`supervise`、`discover` 是 `rwrl` 子命令，没有独立二进制或 `python -m` 入口。`rwrl doctor` 对每台工作站机器人运行设备检查，并检查主机与项目（[cli.md](cli.md#robot-host-diagnosis-and-repair)）；所有 doctor 代码位于 `real_world_rl.doctor`。命令协议和解析器不导入 torch。工作站输入为 `--station PATH`。Setup 将该文件用作默认值来源，而非输出路径；工作站查找默认值不变。错误写入 stderr，退出码常量位于 `real_world_rl.cli.errors`。

在机器人 PC 的仓库根目录、项目环境已激活时运行：

```bash
rwrl --help
rwrl daemon --help
rwrl doctor --station local/station.yaml --json
```

`rwrl doctor` 接受 `--yes` 跳过确认，但仍执行预检，即使没有修复参数也如此。修复确认只接受 `y`（忽略大小写及两端空白）。共用退出策略：0 为成功，1 为运行时/设备失败或尚未解决的 doctor 问题，2 为用法或配置错误。Setup 中断返回 130；supervisor 启动失败返回 127，停止升级返回 1，其余情况保留子进程状态。

不输出 JSON 的命令（`setup`、`daemon`、`supervise`）拒绝全局 `--json`，用法退出码为 2。`doctor`、`discover` 支持 `--json`；`rwrl --json discover` 输出 `schema_version: 1` 和列表 `cameras`、`teleops`、`can_interfaces`、`hosts`、`hid_devices`、`unknown_hid`（见[连接并发现](devices.md#discover)）。`supervise` 保留 `-- <child command...>` 边界，子进程参数原样传递。`teleop calibrate --backend gello` 输出 teleop YAML 块；`setup --only teleop --calibrate` 在向导中执行同样的标定并写入工作站 YAML。

<a id="backend-dependencies"></a>

### 后端依赖

| 扩展 | 安装内容 | 何时需要 | 机器 |
|---|---|---|---|
| `franky` | `franky-control` | 通过 libfranka 连接 Franka 臂/手（`arm_backend: franky`）。默认 wheel 捆绑一个 libfranka 版本；其他固件需安装匹配 wheel（见[安装正确的 franky wheel](doctor.md#installing-the-right-franky-wheel)）。 | 机器人 PC |
| `pyagxarm` | `pyAgxArm`（来自 GitHub） | 通过 `arm_backend: pyagxarm` 连接 Piper（Piper 默认后端） | 机器人 PC |
| `piper-sdk` | `piper_sdk` | 通过厂商 SDK 连接 Piper（`arm_backend: piper_sdk`） | 机器人 PC |
| `realsense` | `pyrealsense2` | RealSense 相机 | 机器人 PC |
| `spacemouse` | `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`） | `teleop: {backend: spacemouse}`（仓库内读取器和发现） | 机器人 PC |
| `gello` | `dynamixel-sdk>=3.7.31,<3.9` | `teleop: {backend: gello}`（GELLO 主臂）及其标定 CLI | 机器人 PC |
| `gello-events` | `evdev` | gello 的 `event_device`（脚踏/键盘切换和事件，Linux） | 机器人 PC |
| `grpc` | `grpcio>=1.60` | daemon（`rwrl daemon`）及 `RemoteRobot` | 机器人 PC **和** GPU 机器 |
| `jpeg` | `pillow`、`opencv-python-headless`（`cv2`） | `rwrl daemon --jpeg-quality N`；`server/codec.py` 的 JPEG 路径导入 `cv2` | 机器人 PC **和** GPU 机器 |
| `legacy-bridge` | `flask`、`pyzmq`、`scipy`、`pyspacemouse` | 旧版 Flask 桥接/夹爪服务和旧版 `teleop/spacemouse/`（旧 `read_all()` API，与仓库内读取器独立；ROS1 需另装） | 机器人 PC |
| `sim` | `mani_skill` | `controller/simulator/` 中的 ManiSkill 阻抗控制器及示例 | GPU 机器 |
| `dev` | `pytest`、`scipy` | 运行测试 | 任意 |

厂商 SDK 延迟导入。核心包、CLI 和 `--mock` 无需安装任何 SDK 即可运行。

<a id="discovery-report"></a>

### 发现报告

Discovery 仅探测，从不连接机器人。`--station` 可选：对指定配置（否则使用[工作站文件](tutorial/02-setup.md)）中每台机器人的 `robot_ip` 执行 ping，并探测 TCP 22、80、443、1337、50051。

未连接设备的机器上（真实输出）：

```text
$ rwrl discover --station robot.yaml
cameras:
  none
teleops:
  none
can interfaces:
  none
hosts:
  172.16.0.2: ping=no reply open ports=none
hid devices:
  none
```

连接硬件后，各行格式为 `realsense: <serial>, <serial>`、`spacemouse: <vid>:<pid>:<path>`（未安装 `hidapi` 时为 `/dev/hidrawN`）、`can0: operstate=up bitrate=1000000`、`172.16.0.2: ping=ok open ports=<list>` 和 `/dev/hidraw0`。不列出 `fake` 后端。

`rwrl --station robot.yaml --json discover`（或 `rwrl --json discover`）以一个 JSON 对象输出同一报告：`schema_version: 1` 和六个列表，无结果时为空：`cameras`（`backend`、`serial`）、`teleops`（`backend`、`device`）、`can_interfaces`（`name`、`operstate`、`bitrate`）、`hosts`（`ip`、`ping_ok`、`open_ports`）、`hid_devices`（路径）及 `unknown_hid`（`vid`、`pid`、`product`、`path`、`message`：外观像 SpaceMouse 但设备表中没有的 HID 设备）。上方机器对应如下 JSON（由同一报告推导，非实际运行采集）：

```json
{"schema_version": 1, "cameras": [], "teleops": [], "can_interfaces": [], "hosts": [{"ip": "172.16.0.2", "ping_ok": false, "open_ports": []}], "hid_devices": [], "unknown_hid": []}
```

| 部分 | 来源 | 核对内容 |
|---|---|---|
| `cameras` | 各相机后端的 `discover()`（RealSense 序列号） | 将序列号复制到 `cameras[].serial` |
| `teleops` | 各遥操作后端的 `discover()`（SpaceMouse：已安装 `hidapi` 时为 `vid:pid:path`，否则列出所有 `/dev/hidraw*`；gello：`/dev/serial/by-id/usb-FTDI_*` 和 `usb-ROBOTIS_*`） | `teleop:` 中配置的设备应出现 |
| `can interfaces` | `/sys/class/net/*` 中 CAN 类型接口，波特率来自 `ip -details link show` | `operstate=up` 且波特率与 YAML 一致 |
| `hosts` | 对配置的 `robot_ip` 执行 `ping` 和 TCP 连接 | `ping=ok`；Franka 的 1337 开放通常表示 FCI 可达。 |
| `hid devices` | `/dev/hidraw*` | SpaceMouse 及其他 HID 遥操作设备 |
| `warnings`（仅有警告时） | 设备表中缺少 ID 的 3Dconnexion（`256f`）HID 设备和 Logitech（`046d`）多轴控制器 | `WARN unknown 3Dconnexion-vendor HID device <vid>:<pid>; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table) (<path>)`：添加[站点设备表](spacemouse-commissioning.md#site-device-table)条目 |

既无 `--station` 也无工作站文件时，`hosts:` 输出 `none`。

<a id="spacemouse-device-identity"></a>

### SpaceMouse 设备身份

仓库内读取器移植了 vendored PySpaceMouse 1.0.3 表中的全部 10 个型号，表中没有 Enterprise。站点遇到其他型号时，写入工作站文件旁的本地 `spacemouse-devices.yaml`（或 teleop 选项 `device_table` 指定路径），以新增条目或覆盖相同 ID 的内置条目；见[站点设备表](spacemouse-commissioning.md#site-device-table)。`discover()`、doctor 和 `open()` 精确匹配合并表的 vendor/product ID 对，因此不会将同厂商其他设备（如 Logitech `046d` 键盘接收器）当作 SpaceMouse。使用 `hid.device()`，不导入 `pyspacemouse`/`easyhid`。不要安装另一个名为 `hid` 的发行包。旧 `legacy-bridge` 扩展和 `SpaceMouseExpert` 仍使用原 API。

<a id="testing"></a>

## 测试

设备层测试位于仓库测试套件的 `tests/devices/`。从仓库根目录、项目环境已激活时运行：

```bash
pip install -e '.[dev,grpc]'
python -m pytest tests/devices -q
```

测试使用 fake 后端和 SDK 桩，无需硬件。缺少 `grpcio` 时跳过 gRPC 测试。维护者在远程容器运行：

```bash
docker exec <container> bash -lc 'cd <checkout> && python -m pytest tests/devices -q'
```

<a id="known-limitations-and-needs-hardware-verification"></a>

## 已知限制与硬件待验证项

以下内容均未在真机执行，fake 仅覆盖逻辑。

- **SpaceMouse USB vendor ID。** `discover()` 按 `0x256f`（3Dconnexion）和 `0x046d`（Logitech 老型号）筛选，均未验证；无 `hidapi` 时列出所有 `/dev/hidraw*`。
- **udev 与 Docker。** `os:hid` 输出的 udev 规则及 `docker run --device=/dev/hidrawN` 提示尚未在机器人 PC 验证。容器须在 daemon 启动前传入设备节点。
- **SpaceMouse wheel 与时序。** 仓库内读取器使用 `set_nonblocking(1)`，每隔 `poll_interval_s`（默认 0.005 秒，即 200 Hz）读取一份报告。排队延迟、拆分报告时序、设备方向和断连行为尚未上机验证。`hidapi==0.14.0.post4` wheel 在全新 Ubuntu 22.04 / CPython 3.10 / x86_64 上的 ELF/系统库依赖仍待验证。
- **Piper 主臂遥操作**（主从跟随）未实现；主臂后端是 `gello`（Dynamixel）。`joint_delta` 仅通过 fake 验证。
- **GELLO 真机。** `gello` 后端及标定 CLI 只对 fake `dynamixel_sdk` 和 fake `evdev` 运行过。未验证项：`discover()` 匹配的 U2D2/OpenRB-150 by-id 名称、各波特率下同步读取时序、FTDI 延迟计时器（默认 16 ms；1 ms 是常见 Dynamixel 建议，后端不设置），以及关节跨整圈边界时当前位置是否连续。
- **姿态步长。** 工作空间可相对参考约束姿态；`safety_box` 和 `max_delta_per_step` 仍只限制位置（`xyz_m`）和关节（`joint_rad`）。设备侧无姿态步长上限。gym 环境以 `rot_scale_rad` 限制自身旋转增量，并将策略目标投影到配置的工作空间。
- **步长上限基准。** 上限相对最后读取状态（最长 0.5 秒前），不是上一目标，因此目标与真实位置的最坏距离约为上限两倍，但不累积。
- **绕过防护。** 旧 ROS1 桥接（`serl_http`）是 HTTP 服务，直接访问 `bridge_url` 的客户端不经过 `Arm`，上述限制均不生效。第三方后端须实现 `_servo_*`/`_move_*` 钩子，不得覆盖公共方法。
- **已配置的遥操作设备必须存在。** 设备缺失会使 `connect()` 失败并回滚整个机器人，部署也如此。部署时可使用不含 `teleop:` 的 YAML。
- **版本偏差。** 带这些功能的 daemon 会在 `arm.health()`（新字段）破坏升级前客户端，旧客户端也无法读取 JPEG 帧。先升级客户端。
- **`--jpeg-quality` 未记录**在训练配置中，部署无法检查它是否匹配训练值。
- **遥操作读取上限与时钟。** daemon 的 `teleop.<name>.read` 最长等待 2 秒；`timestamp` 来自机器人 PC 的单调时钟。`RemoteTeleop.get_observation()` 返回客户端最后一个样本，而 daemon 的 `robot.get_observation()` 包含从其最新样本取得的 `teleop` 条目（不消费事件）。

<a id="legacy"></a>

## 旧模块

这些模块早于设备层，为现有用户保留。计划工作见 [`docs/devices-roadmap.md`](../devices-roadmap.md)。

- **`controller/`**：纯 torch 笛卡尔阻抗和位姿控制器（`core/`、`adapters/`），在 `controller/simulator/maniskill/` 中封装用于 ManiSkill（扩展 `sim`）。用于 GPU 批量仿真，不用于真机。见 [`real_world_rl/legacy/controller/README.md`](../../real_world_rl/legacy/controller/README.md)。
- **`controller/real/franka_bridge.py`** 与 **`gripper_server.py`**：转发到 ROS1 `serl_franka_controllers` 阻抗控制器和 Franka Hand 的 Flask/HTTP 服务。`serl_http` 后端是其客户端。在 ROS 机器运行 `python -m real_world_rl.legacy.controller.real.franka_bridge --robot_ip <ip> --port 5000`（扩展 `legacy-bridge`；ROS1 单独安装，见模块 docstring）。
- **`teleop/`**：设备层之前的遥操作（`teleop/spacemouse/` 使用旧 `pyspacemouse` 的 `read_all()` API，扩展 `legacy-bridge`；`teleop/utils/` 通过 ZeroMQ 提供 Pico twist 遥操作；检查工具为 `examples/run_twist_input_inspector.py`）。`Teleop` 部件（[步骤 7](devices.md#add-a-teleop-device)）取代 SpaceMouse 代码。Pico 路径尚未迁移，`real-world-rl` 的 `--teleop-device pico` 仍导入这些模块，因此暂时保留。
