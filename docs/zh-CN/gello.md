<a id="gello-leader-arm-hardware-to-hil-training"></a>

# GELLO 主臂 从硬件到 HIL 训练

[中文入口](../README.zh-CN.md) · [English](../gello.md)

GELLO 是被动式、3D 打印的 Franka Panda 仿形主臂，以 Dynamixel 舵机读取绝对关节角。本指南从硬件台架走到人工介入训练，涵盖硬件与权限、标定、机器人 YAML、示范录制和 `hil_serl`、`residual_hil_serl`、`silri` 命令。遥操作输入源的设备表、接管起始策略及介入包装器 `info` 字段见 [teleop.md](teleop.md)；示范录制详见 [recording-demos.md](recording-demos.md)。首次上手建议阅读[教程](tutorial/00-overview.md)，其配置向导直接执行第 2 节标定并生成第 3 节 YAML，随后继续 BC、ACT、HIL 训练和部署。

硬件设计来自 Philipp Wu 的 GELLO，采用 MIT 许可证：https://github.com/wuphilipp/gello_software 。`robot_infra` 的 `gello` 遥操作后端基于 ROBOTIS `dynamixel_sdk` 重新实现，未使用 GELLO 源码。

接入路径选择：

| 从臂软件栈 | 动作模式 | 遥操作设备 | GELLO 连接位置 |
| --- | --- | --- | --- |
| `real_robot`（robot_infra、franky） | `joint_delta` 为教程各阶段主线，也支持 `joint_abs`；示范、基础策略和 HIL 必须一致。 | `--teleop-device leader_arm`，守护进程将 GELLO 作为机器人 `teleop:` 部件提供。 | 机器人 PC。 |
| `real_robot` | 同上。 | `--teleop-device gello`，本地打开的 `GelloTeleopSource`。 | actor 机器。 |
| `franka_real`（SERL HTTP 桥接，仅笛卡尔动作） | `cartesian_delta` | `--teleop-device leader_cartesian`，使用 Panda 正向运动学。 | 机器人主机，由占位机器人守护进程提供。 |

<a id="1-hardware"></a>

## 1. 硬件

- **总线适配器。** 可用 ROBOTIS U2D2，其 FTDI 芯片显示为 `/dev/serial/by-id/usb-FTDI_*`、`ttyUSB*`；或运行 USB-to-Dynamixel 透传固件的 OpenRB-150，显示为 `/dev/serial/by-id/usb-ROBOTIS_*`、`ttyACM*`。请配置稳定的 `/dev/serial/by-id/...` 路径，`ttyUSB0` 会随插拔顺序改变。
- **舵机 ID 与波特率。** 下文默认关节 ID 为 1–7，扳机为 8。后端按 `baudrate` 打开串口，默认 57600，与 XL330 出厂值一致，不会写舵机 EEPROM。若通过 Dynamixel Wizard 改过波特率，例如提高到 1 Mbps，需同步修改 `baudrate:`。57600 波特下同步读取 8 个舵机约需 35 毫秒，适合 10 Hz 循环；更快循环需要提高波特率。
- **FTDI 延迟计时器（U2D2）。** FTDI 驱动默认缓存串口数据 16 毫秒，可能给每次读取增加最多 16 毫秒。可设为 1 毫秒：

  ```bash
  cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer          # 16 by default
  echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
  # persistent, /etc/udev/rules.d/99-ftdi-latency.rules:
  # ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
  ```

  OpenRB-150 是 USB CDC 设备，没有该延迟计时器。
- **权限。** 用户需要加入 `dialout` 组，运行 `sudo usermod -aG dialout $USER` 后重新登录，或配置 udev；Docker 中通过 `--device=/dev/serial/by-id/...` 传入节点。`python -m robot_infra.doctor --config robot.yaml` 检查端口存在及读写权限（`os:serial`）。后端不会终止其他进程或更改权限。
- **供电。** 舵机需要供电才能读取。U2D2 无法通过 USB 给 Dynamixel 总线供电，应使用供电集线器或 OpenRB-150 电源。后端在连接和关闭时都向每个舵机写入力矩关闭命令，始终保持被动，由人工移动主臂。

> **使能力矩时的注意事项。** Dynamixel Wizard 测试或第 9 节未来主臂跟随功能启用 GELLO 舵机力矩后，主臂会自主运动。使用符合额定值的电源，XL330 标称 5 V，禁止使用 12 V；电源应能同时提供所有舵机的堵转电流，并设置较低的电流或 PWM 限制，保持手与线缆避开运动部件。该主臂的 XL330 力矩开启行为尚待验证。

在连接 GELLO 的机器上安装后端：

```bash
pip install -e "robot_infra[gello]"          # dynamixel-sdk
pip install -e "robot_infra[gello-events]"   # evdev, only for an event_device (pedal/keyboard) on that machine
```

<a id="2-calibration"></a>

## 2. 标定

标定命令确定每台设备的关节方向、偏移和扳机范围，打印 `teleop:` YAML 配置块。`python -m robot_infra.setup` 会逐臂直接执行并写入工作站文件，因此使用向导时可以跳过下方手动粘贴步骤，见 [robot_infra README](robot-infra.md#setup-wizard)。

```bash
python -m robot_infra.devices.backends.teleops.gello_calibrate \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXXXXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --joint-limits-yaml robot.yaml
```

1. **方向检查。** 命令行逐关节要求沿机器人正方向移动并按 Enter，显示观测方向，除非回答 `n` 否则采用。移动不足 0.1 rad 时保留当前方向。`--joint-signs` 提供初始方向，默认全部 `+1`；`--skip-sign-check` 直接采用给定方向。
2. **偏移。** 将主臂保持标定姿态并按 Enter。默认 Franka 姿态为 `0 0 0 -pi/2 0 pi/2 0`，可用 `--calibration-pose` 改变。偏移在四分之一圈网格上求解，因为舵盘以 90 度步长安装。残差大于 pi/8 时报告警告，表示姿态或方向有误。
3. **扳机。** 完全打开后按 Enter，再完全闭合后按 Enter。
4. **角度环绕中心。** 使用 `--joint-limits-yaml` 提供含 `joint_limits` 的机器人 YAML 时，以各关节范围中点为环绕中心；未提供则用零并警告，这对范围超过 pi 的 Panda 第 6 关节不正确。

将打印的配置块粘贴到第 3 节机器人 YAML。完整选项及默认值如下：

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

真机使用前检查结果：在守护进程运行时读取主臂，或读取 `Robot.from_yaml` 的 `teleop` 部件，对照标定姿态和其他几个姿态下的主从臂关节角。

<a id="3-the-robot-yaml-real_robot-franky"></a>

## 3. 机器人 YAML（`real_robot`、franky）

从 `robot_infra/docs/robot.franka.example.yaml` 开始，加入安全限制与 GELLO。也可使用 `python -m robot_infra.setup`，生成包含机器人、相机、GELLO 标定和守护进程端口的工作站 YAML，默认 `./local/station.yaml`，守护进程不传 `--config` 也能找到。本文原有说明称向导将 `safety_box`、`joint_limits` 和 `max_delta_per_step` 留为 `null`，需要手工补充下方限制；实际使用时结合教程核对生成文件。

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

- `max_delta_per_step.joint_rad` 限制每个伺服目标相对实测关节位置的距离。建议每步 0.03–0.05 rad，10 Hz 下为 0.3–0.5 rad/s；它约束策略和人工动作的单步幅度。`joint_abs` 缺少该项时拒绝运行。
- actor 侧使用 `joint_scale_rad` 0.05，即 `real_robot` 默认值。`joint_delta` 动作 `+-1` 请求每步 `+-0.05` rad，守护进程继续按 `joint_rad` 裁剪。`state_fields` 必须含 `joint_pos`，主臂输入源要与它比较。

在机器人 PC 上启动服务：

```bash
python -m robot_infra.server --config robot.yaml --bind 0.0.0.0:50051 --token T
```

actor 预设 `hil.yaml` 通过 `--config` 传入，完整字段见 README“真机运行”：

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

若 GELLO 连接 actor 机器，使用 `--teleop-device gello`，机器人 YAML 中移除 `teleop:` 段，将标定 `options` 以平面映射写入 `--teleop-options-yaml gello.yaml`，见 [teleop.md](teleop.md#devices)。

<a id="4-franka_real-the-stub-robot-daemon"></a>

## 4. `franka_real` 的占位机器人守护进程

`franka_real` 访问仅接受笛卡尔目标的 SERL HTTP 桥接。GELLO 连接机器人主机，由占位机器人提供服务：`type: franka`、`arm_backend: fake`，无夹爪、无相机，唯一真实设备为 GELLO。配置示例为 `robot_infra/docs/robot.gello_station.example.yaml`：

```bash
# on the robot host, next to the SERL bridge
python -m robot_infra.server --config robot.gello_station.yaml --bind 0.0.0.0:50052
```

actor 用 Panda 正向运动学将主臂关节转换为笛卡尔速度增量，使用 `--teleop-device leader_cartesian --teleop-options-yaml station.yaml`，且 `station.yaml` 包含 `remote_address: robot-host:50052`。映射、夹爪和 DH 验证见 [teleop.md](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)。

<a id="5-recording-demos"></a>

## 5. 录制示范

`examples/record_real_demos.py` 与 actor 使用相同的环境构建方式，仅记录介入步骤，输出 learner 通过 `--demo-dataset-paths` 加载的 pkl 和动作约定附属文件。录制动作模式必须与后续训练一致，附属文件不符时 learner 拒绝加载：`joint_delta` 全流程使用 `joint_delta` 示范，`joint_abs` 另录一套 `joint_abs` 数据，见[教程 04](tutorial/04-record-demos.md)。

```bash
python examples/record_real_demos.py --config hil.yaml \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-event-device keyboard --output-path demos/task.pkl --num-successes 20
```

录制器默认排除 ramp 同步步骤，HIL actor 则保留；`--keep-syncing-steps` 可让录制器也保留。`--start-tolerance-rad`、回合起始同步和事件等参数见 [recording-demos.md](recording-demos.md)。

<a id="6-hil-training"></a>

## 6. HIL 训练

三个 HIL 方法使用相同遥操作参数。`--robot-teleop-action-mode joint_delta` 在远程连接前声明机器人遥操作部件；主臂只需声明关节空间，实际输出由环境 `action_mode` 决定。

`hil_serl`、`residual_hil_serl`（BC 或 ACT 基础策略）和 `silri` 的完整 learner / actor 命令见[教程 07](tutorial/07-hil-training.md)，共用 `--teleop-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard`。操作员按键和接管方式见[教程 06](tutorial/06-intervening.md)。

人工介入切换信号可以来自：

- `--teleop-event-device keyboard`，即教程方案：通过 `pynput` 读取 actor 机器桌面会话，Space 切换、Esc 结束回合、`s` / `f` / `r` 触发成功、失败、重录。也支持 `spacemouse`，介入状态随 SpaceMouse 活动变化。YAML 保持 `intervention: always`，事件设备的切换信号会覆盖它。
- 机器人 PC 上的 evdev 脚踏或键盘：在 `teleop:` 选项中设置 `intervention: toggle` 和 `event_device: /dev/input/by-id/...`，安装 `gello-events` 并让用户加入 `input` 组，去掉 `--teleop-event-device`。按键由 `event_keys` 定义，默认值见上文。

<a id="two-gellos-two-arms"></a>

### 两套 GELLO 与双臂

双臂工作站使用 `--env-backend real_robot_bimanual`，见 [bimanual.md](bimanual.md)，每台机器人配置自己的 GELLO。`python -m robot_infra.setup` 探测两条总线、依次标定，并生成 `robots: {left, right}`，各自包含 `teleop:` 和守护进程端口。向导说明见 [robot_infra README](robot-infra.md#setup-wizard)，带注释示例为 `robot_infra/docs/station.bimanual.example.yaml`。训练或录制时使用 `--teleop-device bimanual`；`--teleop-arm-device leader_arm` 通过各臂守护进程读取，`--teleop-arm-device gello --teleop-options-yaml <station.yaml>` 则用同一工作站文件的配置在 actor 上打开两套 GELLO。一个 `--teleop-event-device keyboard` 同时切换两臂，介入规则见 [teleop.md](teleop.md#two-arms-bimanual)。

<a id="7-what-the-hil-rows-record"></a>

## 7. HIL 数据记录内容

介入时 actor 执行主臂输入源动作，并作为 transition 的 `action` 推送。以下行为由 `tests/test_leader_arm_hil_loops.py` 的脚本化端到端检查覆盖：

- **`joint_delta`。** 动作为 `clip((q_leader - q_follower) / joint_scale_rad, -1, 1)`。主臂较远时每步按 `joint_scale_rad` 饱和，直到从臂追上；对齐后动作归零。这些步骤不标记同步。
- **`joint_abs` 配合 `--teleop-onset ramp`。** 切换后的初始步骤限速，目标为 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`，保存动作为实际执行目标。它们属于介入步骤，因此进入示范缓冲区，同时携带 `info["teleop_events"]["teleop_syncing"] = True`，可由下游过滤。ramp 结束后使用主臂归一化关节目标。
- **`--teleop-onset arm`。** 主从臂误差进入 `--teleop-arm-tolerance-rad`（0.1）前，切换不生效，策略继续执行，数据保持未介入，关节误差每秒打印一次。就绪后每步都介入，直到释放切换信号。
- **`residual_hil_serl`。** 保存的 `action` 是人工总动作；`base_actions` 是同一观测的基础策略动作，`policy_action` 是残差策略提出的动作。
- **`silri`。** `intervention_onset` 标记首次人工介入前的 transition。`ramp` 下为切换前策略步骤；`arm` 下为主臂就绪前最后一步，切换到就绪之间仍属于策略步骤。
- **夹爪。** `[0, 1]` 内的扳机宽度 `w` 映射为 `2w - 1`。混合夹爪头，包括 `hil_serl` DQN、`--residual-variant hybrid` 和 `silri` 离散头，将保存动作映射到 `-1` / `0` / `+1` 中最近的标签，因此半捏可能成为“保持”。这些方法应设置 `gripper_binarize_threshold: 0.5`，使保存动作恰为 `+1`（打开）或 `-1`（闭合）。

<a id="onset-ramp-or-arm"></a>

### 接管起始方式 `ramp` 或 `arm`

默认 `ramp` 立即接管，使从臂在 `joint_abs` 下每步以 `ramp_step_rad`、在 `joint_delta` 下每步以 `joint_scale_rad` 靠近主臂。过渡步骤进入示范缓冲区，仅 `joint_abs` 带同步标记。`arm` 需要每次先手工匹配主从臂，避免机器人接管追赶运动及相应数据；每次 reset 都会解除就绪。示范需要从静止开始时优先 `arm`，快速纠正时用 `ramp`。

<a id="8-not-verified-yet"></a>

## 8. 尚待验证

- 当前流程仅通过 fake 设备运行，CPU 测试使用脚本化主臂和 fake Dynamixel SDK；尚未在物理 GELLO 或机器人上执行。
- 第 1 节的适配器、波特率、延迟、读取时间和每台设备偏移、方向为默认值或估计，应在实际设备上核对。
- XL330 的力矩开启行为与供电要求仍待验证，见第 1 节；后端保持力矩关闭。
- `leader_cartesian` 使用的 Panda DH 表尚未与真机比较，检查方式见 [teleop.md](teleop.md#verify-the-dh-table-on-the-robot)。

<a id="9-future-work-leader-tracking-wp8"></a>

## 9. 后续工作 主臂跟随 WP8

开启力矩后，策略执行期间主臂可以跟踪从臂，使接管开始时两者已经对齐，无需额外接管起始策略。这需要先完成第 1 节的供电和安全工作，包括限流位置控制，以及操作员握住主臂时的柔顺交接；当前尚未实现。
