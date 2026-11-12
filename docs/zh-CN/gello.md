<a id="gello-leader-arm-hardware-to-hil-training"></a>

# GELLO 主臂：从硬件到 HIL 训练

GELLO 是 Franka Panda 的被动式 3D 打印副本，关节采用 Dynamixel 舵机，读取绝对关节角。本指南从工作台介绍到人工介入训练：硬件与权限、标定、机器人 YAML、示范录制，以及 `hil_serl`、`residual_hil_serl`、`silri` 命令。遥操作源语义（设备表、接管起始策略、介入包装器的 `info` 键）见 [teleop.md](teleop.md)；示范录制另见 [recording-demos.md](recording-demos.md)。首次操作可按[教程](tutorial/00-overview.md)使用配置向导（内联执行第 2 节标定并写入第 3 节 YAML），然后继续 BC、ACT、HIL 训练和部署。

硬件设计来自 GELLO（MIT 许可，Philipp Wu，https://github.com/wuphilipp/gello_software）。设备层的 `gello` 遥操作后端基于 ROBOTIS `dynamixel_sdk` 重新实现，未使用 GELLO 代码。

接入路径选择：

| 从臂技术栈 | 动作模式 | 遥操作设备 | GELLO 连接到 |
| --- | --- | --- | --- |
| `real_robot`（设备层，franky） | `joint_delta`（教程各阶段统一使用）或 `joint_abs`；示范、基础策略和 HIL 必须统一 | `--teleop-device leader_arm`（机器人 daemon 将 GELLO 作为机器人 `teleop:` 部件提供） | 机器人 PC |
| `real_robot` | 同上 | `--teleop-device gello`（本地打开的 `GelloTeleopSource`） | actor 机器 |
| `franka_real`（SERL HTTP 桥接，仅笛卡尔） | `cartesian_delta` | `--teleop-device leader_cartesian`（Panda 正向运动学） | 机器人主机，经由占位机器人 daemon |

<a id="1-hardware"></a>

## 1. 硬件

- **总线适配器。** ROBOTIS U2D2（FTDI 芯片，显示为 `/dev/serial/by-id/usb-FTDI_*`、`ttyUSB*`），或运行 USB-to-Dynamixel 透传程序的 OpenRB-150（`/dev/serial/by-id/usb-ROBOTIS_*`、`ttyACM*`）。始终配置稳定的 `/dev/serial/by-id/...` 路径；`ttyUSB0` 会随插入顺序变化。
- **舵机 ID 与波特率。** 下方默认关节 ID 为 1–7，扳机 ID 为 8。后端以 `baudrate`（默认 57600，XL330 出厂值）打开串口，绝不写舵机 EEPROM。若用 Dynamixel Wizard 改过波特率（如提高到 1 Mbps 加快读取），须相应设置 `baudrate:`。57600 下同步读取 8 个舵机约需 35 ms，适合 10 Hz 控制循环；更快循环应提高波特率。
- **先运行 doctor。** 工作站指定 GELLO `port` 后，在 GELLO 所接机器的项目环境中运行 `rwrl --station robot.yaml doctor --fix`。确认一次后，若串口不可读写，会将用户加入 `dialout` 并写入 `/etc/udev/rules.d/99-rwrl-gello-*.rules` 访问规则；若 FTDI 延迟不是 1 ms，会写入持久延迟规则，再次检查；重新插拔或重新登录可能仍是待办动作（[cli.md](cli.md#robot-host-diagnosis-and-repair)）。下方手工命令适用于没有项目环境的机器，或用于手动核对。
- **FTDI 延迟计时器（U2D2）。** FTDI 驱动默认缓存串口数据 16 ms，使每次读取增加最多 16 ms。设为 1 ms：

  ```bash
  cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer          # 16 by default
  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  # persistent, /etc/udev/rules.d/99-ftdi-latency.rules:
  # ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
  ```

  OpenRB-150 是 USB CDC 设备，无延迟计时器。
- **权限。** 用户需要 `dialout` 组（`sudo usermod -aG dialout $USER`，然后重新登录）或 udev 规则；Docker 中通过 `--device=/dev/serial/by-id/...` 传入节点。`rwrl doctor --station robot.yaml` 检查配置端口是否存在、可读写（`os:serial`）。后端从不终止进程或修改权限。
- **供电。** 舵机必须通电才能读取（U2D2 不通过 USB 为 Dynamixel 总线供电；需电源集线器或 OpenRB-150 电源）。后端连接及关闭时都为所有舵机写入扭矩关闭，绝不使能扭矩：主臂仅由手动移动。

> **任何开启扭矩的用途都须注意。** 开启 GELLO 舵机扭矩（Dynamixel Wizard 测试，或第 9 节未来的主臂跟随）会使主臂自行运动。应使用额定电压（XL330 标称 5 V，绝不能用 12 V），电源须能同时提供所有舵机堵转电流，设置低电流或 PWM 限制，并让手和线缆远离运动区。该主臂 XL330 开启扭矩的行为尚未验证。

在 GELLO 所接机器上安装后端：

```bash
pip install -e ".[gello]"                    # dynamixel-sdk
pip install -e ".[gello-events]"             # evdev, only for an event_device (pedal/keyboard) on that machine
```

<a id="2-calibration"></a>

## 2. 标定

方向、位姿、扳机和读回的引导步骤见 [05 遥操作：GELLO](tutorial/05-teleop-gello.md)。标定命令输出 `teleop:` 块；`--use` 在工作站差异经审阅后写入标定的 `teleop.options`，无夹爪通道时移除夹爪键。

| 参数 | 默认值/用途 |
|---|---|
| `--port` | 工作站 `teleop.options.port` |
| `--joint-ids` | 工作站 `teleop.options.joint_ids` |
| `--gripper-id` | 工作站 `teleop.options.gripper_id`；缺省表示无夹爪通道 |
| `--baudrate` | 工作站值，否则 57600 |
| `--joint-signs` | 工作站值，否则全为 `+1` |
| `--calibration-pose` | Franka 七关节位姿 `0 0 0 -pi/2 0 pi/2 0`（弧度） |
| `--joint-limits-yaml` | 用于包裹中心的其他机器人 YAML；否则使用工作站 `joint_limits` |
| `--skip-sign-check` | 保留初始符号；默认关闭 |
| `--use` | 将标定选项保存到工作站；默认关闭 |

生成的机器人 YAML 字段如下（占位角度须替换为该设备的标定值）：

```yaml
teleop:
  name: gello
  backend: gello
  hold_s: 0.0
  options:
    port: /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0
    baudrate: 57600                 # must match the servos; never written to them
    joint_ids: [1, 2, 3, 4, 5, 6, 7]
    joint_offsets_rad: [...]        # from the calibration CLI
    joint_signs: [...]              # from the calibration CLI
    wrap_centers_rad: [...]         # from the calibration CLI (midpoints of joint_limits)
    gripper_id: 8                   # null: no gripper channel (the actor holds the last gripper action)
    gripper_open_rad: ...           # raw trigger angle, released
    gripper_closed_rad: ...         # raw trigger angle, squeezed
    gripper_binarize_threshold: 0.5 # 1/0 width instead of a continuous one; see section 7
    intervention: always            # always: every poll intervenes | toggle: follows the toggle key of event_device
    event_device: null              # /dev/input/by-id/...-event-kbd (evdev, on this machine)
    event_keys: {toggle: KEY_SPACE, episode_end: KEY_ESC, success: KEY_S, failure: KEY_F, rerecord: KEY_R}
    poll_interval_s: 0.0
```

<a id="3-the-robot-yaml-real_robot-franky"></a>

## 3. 机器人 YAML（`real_robot`、franky）

从 `examples/stations/robot.franka.example.yaml` 开始，加入安全限制和 GELLO。也可使用 `rwrl setup`，生成包含机器人、相机、GELLO 标定和 daemon 端口的工作站 YAML（`./local/station.yaml`，daemon 无需 `--config` 即可找到）；它写入 Panda `joint_limits`、`max_delta_per_step` 上限和询问得到的 `safety_box`。运动前对照实际工作站检查这些值；手写文件也需相同限制：

```yaml
type: franka
name: franka
arm_backend: franky
robot_ip: 172.16.0.2
home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
joint_limits:
  - [-2.8, 2.8]
  - [-1.7, 1.7]
  - [-2.8, 2.8]
  - [-3.0, -0.1]
  - [-2.8, 2.8]
  - [0.0, 3.7]
  - [-2.8, 2.8]
max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}   # recommended joint_rad: 0.03 to 0.05
gripper:
  backend: franky
  mode: binary
  binary_threshold: 0.5
cameras:
  - {name: wrist, backend: realsense, width: 640, height: 480, fps: 30}
teleop:
  name: gello
  backend: gello
  options: {...}                    # the calibration block of section 2
```

- `max_delta_per_step.joint_rad` 将每个伺服目标限制在距实测关节该距离内。保持为每步 0.03–0.05 rad（10 Hz 下为 0.3–0.5 rad/s）：它限制任何策略或人工动作每步能做的事。无此项时 `joint_abs` 拒绝运行。
- actor 侧使用 `joint_scale_rad` 0.05（`real_robot` 默认值）：`joint_delta` 的 `+-1` 表示每步请求 `+-0.05` rad，daemon 将超过 `joint_rad` 的部分裁剪。`state_fields` 必须包含 `joint_pos`（主臂源要与之比较）。

在机器人 PC 提供服务：

```bash
rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 --token T
```

actor 的预设（`hil.yaml`，通过 `--config` 传入；README 的“真机运行”节列出全部字段）：

```yaml
env_backend: real_robot
env_id: my_task
obs:
  rgb: [wrist]
  image_size: [128, 128]
real_robot:
  remote: ROBOT_PC:50051
  token: T
  num_joints: 7
  action_mode: joint_delta          # or joint_abs; the same for demos, base policy and HIL runs
  joint_scale_rad: 0.05
  joint_limits: [[-2.8, 2.8], [-1.7, 1.7], [-2.8, 2.8], [-3.0, -0.1], [-2.8, 2.8], [0.0, 3.7], [-2.8, 2.8]]
  state_fields: [joint_pos, gripper_pos]
  control_hz: 10
  max_episode_steps: 200
```

若 GELLO 接在 actor 机器（`--teleop-device gello`），机器人 YAML 不配置 `teleop:`，将标定 `options` 作为平铺映射，经 `--teleop-options-yaml gello.yaml` 传入（[teleop.md](teleop.md#devices)）。

<a id="4-franka_real-the-stub-robot-daemon"></a>

## 4. `franka_real`：占位机器人 daemon

`franka_real` 使用仅接受笛卡尔目标的 SERL HTTP 桥接。GELLO 接在机器人主机，由占位机器人（`type: franka`、`arm_backend: fake`、无夹爪、无相机）提供服务，其唯一真实设备是 GELLO，配置见 `examples/stations/robot.gello_station.example.yaml`：

```bash
# on the robot host, next to the SERL bridge
rwrl daemon --station robot.gello_station.yaml --bind 0.0.0.0:50052
```

actor 用 Panda 正向运动学将主臂关节变成笛卡尔 twist（`--teleop-device leader_cartesian --teleop-options-yaml station.yaml`，`station.yaml` 中设 `remote_address: robot-host:50052`）。从臂跟踪持久命令目标：无人驱动时保持固定姿态；命令领先实测 TCP 不超过 `--teleop-lead-cap-m 0.05` / `--teleop-lead-cap-rad 0.3`，推进速度不超过 `--teleop-target-speed-m-s 0.05` / `--teleop-target-speed-rad-s 0.3`。环境缩放至少应等于领先上限（`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`，或 `franka_real` 动作缩放）。同一输入源也能驱动 `cartesian_delta` 的 `real_robot`；`examples/presets/gello_cartesian.yaml` 是其单臂 50 Hz、仅状态观测预设。映射、tracker、夹爪和 DH 检查见 [teleop.md](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)。

<a id="5-recording-demos"></a>

## 5. 录制示范

受监督录制、操作员标记、归档检查和动作约定兼容性见 [08 录制示范](tutorial/08-record-demos.md)。示范与加载它的训练须使用同一动作模式；关节空间同步/导出细节见 [07 主从跟随](tutorial/07-teleop-leader-follower.md#2-record-with-joint-space-leader_arm)和[录制器参考](recording-demos.md)。

<a id="6-hil-training"></a>

## 6. HIL 训练

三种 HIL 方法使用相同的遥操作参数。`--robot-teleop-action-mode joint_delta` 在远程机器人连接前声明其 teleop 部件；对主臂只需声明一种关节空间，实际输出由环境 `action_mode` 决定。

`hil_serl`、`residual_hil_serl`（BC 或 ACT 基础策略）、`silri` 的完整 learner/actor 命令使用 `--teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard`，见[教程 10](tutorial/10-hil-training.md)；按键与接管起始方式见[操作员页](intervening.md)。

介入切换的来源：

- `--teleop-event-device keyboard`（教程路径）：通过 `pynput` 读取 actor 机器键盘（`t` 接管、`h` 交还、Esc 结束回合、`s`/`f`/`r` 产生成功/失败/重录，`q` 结束回合并在回合后停止运行；`pynput` 需要该机器桌面会话）。`--teleop-event-device stdin` 从运行命令的终端读取相同按键（无需桌面，可通过 SSH）。YAML 保持 `intervention: always`，事件设备状态会替代它。
- 机器人 PC 上的 evdev 脚踏或键盘：在 YAML 的 `teleop:` 选项中设 `intervention: toggle`、`event_device: /dev/input/by-id/...`（扩展 `gello-events`，用户加入 `input` 组），省略 `--teleop-event-device`。按键由 `event_keys` 指定（默认值见上文）。

<a id="two-gellos-two-arms"></a>

### 两套 GELLO、双臂

双臂工作站（`--env-backend real_robot_bimanual`，[bimanual.md](bimanual.md)）为每台机器人配一套 GELLO。`rwrl setup` 生成双臂工作站 YAML：探测 GELLO 总线、内联标定每套设备，写入 `robots: {left, right}`，每台有自己的 `teleop:` 与 daemon 端口（设备层指南的[配置向导](tutorial/02-setup.md)；`examples/stations/station.bimanual.example.yaml` 是注释示例）。随后以 `--teleop-device bimanual` 训练或录制：`--teleop-arm-device leader_arm` 通过各臂 daemon 读取 GELLO；`--teleop-arm-device gello --teleop-options-yaml <station.yaml>` 在 actor 机器按同一文件选项打开两套 GELLO。一个 `--teleop-event-device keyboard`（或 `stdin`）同时接管/交还两臂；介入规则见 [teleop.md](teleop.md#two-arms-bimanual)。

<a id="7-what-the-hil-rows-record"></a>

## 7. HIL 数据行记录什么

人工介入时，actor 执行主臂源动作，并将它作为 transition 的 `action` 上传。细节如下（对应 `tests/test_leader_arm_hil_loops.py` 的脚本化端到端检查）：

- **`joint_delta`。** 动作为 `clip((q_leader - q_follower) / joint_scale_rad, -1, 1)`：主臂较远时每步饱和至 `joint_scale_rad`，直到从臂追上，对齐后为零。`ramp` 下，主臂首次进入 `joint_scale_rad` 范围前的初始追赶标为 syncing；对齐后再次饱和的教学步骤仍是正常监督。
- **`joint_abs` 与 `--teleop-onset ramp`。** 切换后最初几步限速：目标为 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`，保存动作就是该目标（实际执行命令）。这些步骤带 `info["teleop_events"]["teleop_syncing"] = True`。归档保留实际动作和人工掩码，训练排除其专家监督。爬坡结束后，动作为主臂归一化关节目标。
- **`--teleop-onset arm`。** 主臂与从臂距离进入 `--teleop-arm-tolerance-rad`（0.1）前，切换不生效：策略继续动作，数据行不算介入，每秒记录一次逐关节误差。随后每步算介入，直到交还。
- **`residual_hil_serl`。** 保存的 `action` 是人工总动作（不是相对基础策略的偏移）；`base_actions` 是同一观测的基础策略动作，`policy_action` 是残差策略建议动作。
- **`silri`。** `intervention_onset` 标记人工首个介入步骤之前的 transition：`ramp` 时为切换前的策略步，`arm` 时为主臂就绪前最后一步（切换到就绪之间仍是策略动作）。
- **夹爪。** `[0, 1]` 内扳机宽度 `w` 变成 `2w - 1`。混合夹爪头（`hil_serl` DQN、`--residual-variant hybrid`、`silri` 离散夹爪）将保存夹爪动作标为最近的 `-1`/`0`/`+1`，因此半扣扳机会成为“保持”标签。对这些方法应在 GELLO 选项设 `gripper_binarize_threshold: 0.5`，保存动作便严格为 `+1`（开）或 `-1`（闭）。

<a id="onset-ramp-or-arm"></a>

### 接管起始：`ramp` 或 `arm`

`ramp`（默认）立即接管，以每步 `ramp_step_rad`（`joint_abs`）或 `joint_scale_rad`（`joint_delta`）让从臂靠近主臂。两种模式均将追赶标为 syncing，训练投影排除其专家监督。`arm` 将过渡过程排除在数据与机器人动作之外，但要求操作员每次接管前先手动将主臂对齐从臂（每次环境 reset 均解除就绪）。示范应从静止机器人开始时优先用 `arm`，快速修正用 `ramp`。

<a id="8-not-verified-yet"></a>

## 8. 尚待验证

- 流程仅使用 fake 验证（脚本化主臂部件和 fake Dynamixel SDK 的 CPU 测试），尚未在真实 GELLO 或机器人运行。
- 第 1 节硬件信息（适配器、波特率、延迟、读取时间）及逐设备偏移/符号为默认值或估计，需在实际设备检查。
- XL330 开启扭矩行为与供电（第 1 节警告）；后端保持扭矩关闭。
- `leader_cartesian` 的 Panda DH 表尚未与真机比较（检查方法见 [teleop.md](teleop.md#verify-the-dh-table-on-the-robot)）。

<a id="9-future-work-leader-tracking-wp8"></a>

## 9. 后续工作：主臂跟随（WP8）

开启扭矩后，策略动作期间主臂可跟随从臂，接管时两臂已对齐，无需两种接管起始策略。这需要第 1 节的扭矩供电与安全工作（限流位置控制、操作员抓住主臂时柔顺交接），尚未实现。
