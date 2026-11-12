<!-- English source: docs/devices/README.md -->

<a id="rlgarden-robot-infra"></a>

# rlgarden-robot-infra

> 命令已改为 rwrl（`robot-infra <命令>` 与 `python -m robot_infra.*` 均已移除，请用 `rwrl <命令>`），正文待同步。

[中文入口](../README.zh-CN.md) · [English](../devices/README.md)

`robot_infra` 是 [rl-garden](https://github.com/JaimeParker/rl-garden) 的真机设备层，现位于 [real-world-rl](https://github.com/JaimeParker/real-world-rl) 仓库的 `robot_infra/` 目录，作为独立发行包；CLI 与设备层运行时不导入 torch，只有 `controller/` 需要 torch，发行包仍声明该依赖。它原为单独的 `rlgarden-robot-infra` 仓库，历史已保留。机器人 PC 仅需克隆该仓库，并从根目录运行 `pip install -e robot_infra`，或按硬件安装 `pip install -e 'robot_infra[franky,grpc]'`，见[安装](#install)；无需使用 `robot_infra/` 外的组件。GPU 机器则安装 `rl-garden/` 子模块及两个发行包：`pip install -e rl-garden -e robot_infra -e .`。它提供：

- **设备层**：带类型的 `Arm` / `Gripper` / `Camera` / `Teleop` 约定，支持 Franka 的 franky 与旧 ROS1 桥接、AgileX Piper 的 pyAgxArm 与 piper_sdk、Intel RealSense 和 SpaceMouse，并执行 TCP 工作区盒、关节限制和单步上限等**设备侧限制**。
- **设备发现**：探测相机、遥操作设备、CAN 接口、机器人主机和 HID。
- **doctor 检查**：逐层检查机器人配置，为每个失败项打印修复提示。
- **机器人守护进程**：在机器人 PC 上通过 gRPC 服务一台机器人。
- `RemoteRobot`：供 GPU 机器使用的客户端机器人镜像。

真机 RL 的 gym 环境位于 [real-world-rl](https://github.com/JaimeParker/real-world-rl)，由该项目导入本包。`robot_infra` 自身不依赖 `rl_garden`。

<a id="robot-infra-cli"></a>

## robot-infra CLI

设备发行包安装 `robot-infra`，原生命令包括 `setup`、`daemon`、`supervise`、`discover`、`doctor` 和 `gello-calibrate`。命令协议和解析器不导入 torch 或 `real_world_rl`。
工作站输入为 `--station PATH`。Daemon、discover 和 doctor 暂时保留 `--config PATH`，setup 暂时保留 `--from PATH`，帮助中标为 "(alias, will be removed)"。Setup 将该文件作为默认值来源，不作为输出路径；工作站查找默认行为不变。错误使用 stderr，退出码常量位于 `robot_infra.cli.errors`。
设备命令注册 `--station`、`--robot`、`--mock`、`--json` 和 `--yes` 全局选项；`--preset` 和 `--direct` 属于 `rwrl`。
原有 Python 模块入口继续兼容，调用同一运行函数。完整项目通过 `rwrl` 暴露同一组命令对象，区别是 `rwrl doctor` 检查主机/项目，`robot-infra doctor` 检查设备。

在机器人 PC 的仓库根目录，激活包含 `rlgarden-robot-infra` 的环境：

```bash
robot-infra --help
robot-infra daemon --help
robot-infra doctor --station local/station.yaml --json
```

两种 doctor 都接受 `--yes`，只跳过确认，仍运行预检查，即使没有修复标志。修复确认只接受 `y`，忽略大小写及两侧空白。共同退出策略为成功 0、运行/设备失败或未解决的检查结果 1、用法或配置错误 2。Setup 中断返回 130，supervisor 启动失败返回 127，停止升级返回 1，否则保留子进程状态。

无 JSON 输出的命令（`setup`、`daemon`、`supervise`、`gello-calibrate`）拒绝全局 `--json`，用法退出码为 2。
`doctor` 和 `discover` 支持 `--json`；`robot-infra --json discover` 打印 `schema_version: 1` 以及列表 `cameras`、`teleops`、`can_interfaces`、`hosts`、`hid_devices` 和 `unknown_hid`（见[接入与发现](#2-plug-in-and-discover)）。
`supervise` 保留 `-- <child command...>` 边界；子命令参数原样传给子进程。各原生命令的 `--help` 列出原模块参数。`gello-calibrate` 打印遥操作 YAML 块；`setup --only teleop --calibrate` 在向导内运行同一标定并写工作站 YAML。Python 模块入口继续作为兼容别名。

<a id="install"></a>

## 安装

要求 Python >= 3.10。

```bash
pip install -e .                    # core: devices, discovery, doctor, fake backends
pip install -e '.[franky,grpc]'     # e.g. a Franka robot PC running the daemon
```

| 可选组 | 安装内容 | 使用场景 | 机器 |
|---|---|---|---|
| `franky` | `franky-control` | 通过 libfranka 控制 Franka 机械臂和 Hand，`arm_backend: franky`。默认 wheel 内置一个 libfranka 版本，其他固件需匹配 wheel，见[安装正确的 franky wheel](#installing-the-right-franky-wheel)。 | 机器人 PC。 |
| `pyagxarm` | GitHub 上的 `pyAgxArm` | Piper 的默认 `arm_backend: pyagxarm`。 | 机器人 PC。 |
| `piper-sdk` | `piper_sdk` | 厂商 SDK，`arm_backend: piper_sdk`。 | 机器人 PC。 |
| `realsense` | `pyrealsense2` | RealSense 相机。 | 机器人 PC。 |
| `spacemouse` | `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`） | `teleop: {backend: spacemouse}`（自带的 HID 读取器与设备发现）。 | 机器人 PC。 |
| `gello` | `dynamixel-sdk>=3.7.31,<3.9` | GELLO `teleop: {backend: gello}` 与标定命令。 | 机器人 PC。 |
| `gello-events` | `evdev` | Linux 上 GELLO `event_device` 脚踏或键盘切换与事件。 | 机器人 PC。 |
| `grpc` | `grpcio>=1.60` | `robot-infra daemon` 守护进程及 `RemoteRobot`。 | 机器人 PC **与** GPU 机器。 |
| `jpeg` | `pillow`、`opencv-python-headless`（`cv2`） | `robot-infra daemon --jpeg-quality N`；`server/codec.py` 的 JPEG 路线导入 `cv2`。 | 机器人 PC **与** GPU 机器。 |
| `real` | `flask`、`pyzmq`、`scipy`、`pyspacemouse` | 旧 Flask 桥接、夹爪服务与 `teleop/spacemouse/`（旧 `read_all()` API，与自带的 HID 读取器分开；ROS1 另装）。 | 机器人 PC。 |
| `sim` | `mani_skill` | `controller/simulator/` 中 ManiSkill 阻抗控制器。 | GPU 机器。 |
| `dev` | `pytest` | 运行测试。 | 任意机器。 |

厂商 SDK 懒加载，核心包、命令行和 `--mock` 无需安装 SDK 即可使用。

<a id="setup-wizard"></a>

### 配置向导

Franka 工作站可为单臂或双臂。连接机器人以太网、RealSense 相机和 GELLO 主臂 USB 后，在机器人 PC 运行：

```bash
pip install -e '.[franky,grpc,realsense,gello]'
robot-infra setup
```

向导探测 `172.16.0.2`、`172.16.1.2`、`$ROBOT_IP`、`--robot-ip` 及 `--scan-subnet CIDR` 所有主机的 TCP 22/443/1337；读取 RealSense 序列号；在所有 `/dev/serial/by-id/usb-FTDI_*`、`usb-ROBOTIS_*` 串口以 57600–4000000 波特广播 ping；并发现 SpaceMouse。随后逐臂询问机器人 IP、机械臂与夹爪后端、归位关节、相机（`1=wrist,2=front`）、遥操作设备（GELLO 端口和舵机 ID，最后一个 ID 默认供夹爪扳机）、介入方式和守护进程端口。每套 GELLO 现场标定，步骤同[第 7 步](#7-add-a-teleop-device)；跳过则写入零偏移与 `# TODO calibrate`。最后生成带注释工作站 YAML、运行 doctor，并打印后续命令。

每臂获得 Panda 数据表 `joint_limits`，可通过 `--joint-limits` 设置；单步上限为 `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`，对应 `--max-delta-xyz-m`、`--max-delta-joint-rad`；还会询问 `safety_box`，基坐标系、米，建议 `[0.3, -0.3, 0.05, 0.7, 0.3, 0.6]`，可用 `--safety-box` 设置，必须包含归位姿态。

守护进程、doctor、discovery 无需 `--station` 也能找到工作站 YAML，依次采用：

1. `$ROBOT_INFRA_CONFIG`。
2. 当前目录的 `./local/station.yaml`，向导默认 `--scope local`，`local/` 已被 Git 忽略。
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml`，默认 `~/.config/robot_infra/station.yaml`，对应 `--scope user`。

`--station <yaml>` 优先于上述三项。

```bash
robot-infra setup --mock                     # canned hardware and a simulated GELLO; the doctor runs with --mock
robot-infra setup --station local/station.yaml  # the old file's values are the defaults
robot-infra setup --station local/station.yaml --only cameras   # redo one section: cameras | teleop | daemon
robot-infra setup --non-interactive --mock --arms 1 --arm-names arm --robot-ip 172.16.0.2 \
    --arm-backend franky --gripper franky --home-joints 0,-0.785,0,-2.356,0,1.571,0.785 \
    --cameras 123422070001=wrist --teleop none --daemon-port 50051 --output /tmp/station.yaml
```

每个问题都有命令行参数，见 `robot-infra setup --help`；逐臂参数可每臂一个值，也可一个值用于全部。`--non-interactive` 在遇到既无参数也无 `--station` 值的首个问题时失败；`--non-interactive --station X` 原样重写 `X`。已有文件先备份成 `station.yaml.bak.<UTC %Y%m%dT%H%M%SZ>`，再原子替换。除非指定 `--allow-unprobed`，配置值必须来自已探测结果。

<a id="spacemouse-python-dependencies-and-udev"></a>

### SpaceMouse Python 依赖与 udev

在已有项目环境中，版本依赖链为仓库根目录的 `spacemouse` extra → `rlgarden-robot-infra[spacemouse]==0.0.1` → `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`）。在 NUC/设备主机、仓库根目录、项目环境激活时运行：

```bash
pip install -e '.[spacemouse]'
```

自带的 HID 读取器移植了随仓库提供的 PySpaceMouse 1.0.3 表中的全部 10 款型号；Enterprise 不在该表中。站点遇到的其他型号写入站点的 local 设备表：站点文件旁的 `spacemouse-devices.yaml`（或 teleop 选项 `device_table` 指定的路径），用于新增条目，或覆盖同一 id 的内置条目，见[站点设备表](spacemouse-commissioning.md#site-device-table)。`discover()`、doctor 和 `open()` 都按合并后设备表中厂商号与产品号的精确配对匹配，同厂商的其他设备（例如 Logitech `046d` 键盘接收器）不会被当成 SpaceMouse。它使用 `hid.device()`，不导入 `pyspacemouse`/`easyhid`。不要安装另一个名为 `hid` 的发行包。旧 `real` extra 和 `SpaceMouseExpert` 仍使用各自现有的 API。新建项目环境时，在 NUC、仓库根目录、未激活任何环境的情况下使用安装器：

```bash
bash scripts/install.sh --extras franka-gello,spacemouse
source .venv/bin/activate
```

目标平台为 Ubuntu 22.04 / CPython 3.10 / x86_64。wheel 路线避开了通过 easyhid 查找系统 `libhidapi`；wheel 的 ELF/系统库要求仍需在干净机器上检查。安装和设备运行均**尚未在真机上验证**。

udev 规则文件随包提供，路径为 `real_world_rl/teleop/spacemouse/udev/99-spacemouse.rules`，以 `MODE="0664", GROUP="plugdev"` 允许访问厂商 `256f` 和 `046d` 的 hidraw 设备。在 NUC、仓库根目录、任意 shell（无需项目环境），具有 sudo 权限时一次性安装：

```bash
sudo cp real_world_rl/teleop/spacemouse/udev/99-spacemouse.rules /etc/udev/rules.d/
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw
```

注销并重新登录，然后拔插设备。也可在 NUC 的任意目录、**项目环境激活**且具有 sudo 权限时，用以下命令复制随包提供的文件，再运行相同的加组/重载命令：

```bash
sudo cp "$(python -c 'import os, real_world_rl.teleop.spacemouse as slice; print(os.path.dirname(slice.__file__))')/udev/99-spacemouse.rules" /etc/udev/rules.d/
```

完整安装范围见 [SpaceMouse 安装](teleop.md#spacemouse-installation-and-linux-permissions)。设备层的按钮电平与锁存按下事件仍不同于应用的接管/夹爪语义（见[第 7 步](#7-add-a-teleop-device)）。

<a id="how-to-start"></a>

## 如何开始

除第 5 步在 GPU 机器执行外，其他步骤都在连接机械臂、CAN 适配器和相机的机器人 PC 上完成。守护进程启动后的示范、BC、HIL-SERL 和部署见真机教程 [`docs/tutorial/`](tutorial/00-overview.md)。

<a id="1-write-robotyaml"></a>

### 1. 编写 `robot.yaml`

机器人配置是包含 `type` 的 YAML 映射，也可放在顶层 `robot:` 键下。未知键会被拒绝并列出合法项，省略项采用下方默认值。完整带注释示例见 [`docs/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml) 与 [`docs/robot.piper.example.yaml`](../../examples/stations/robot.piper.example.yaml)。Franka 可由[配置向导](#setup-wizard)生成。

多机器人工作站，例如双 Franka，在 `robots:` 下按名称列出映射；未填写 `name` 时使用映射键。每台可含 `daemon:`，单机器人文件也允许，守护进程读取后在解析机器人配置前移除：

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

`robot-infra daemon` 为每台启动独立进程，`robot-infra doctor` 逐台检查，`--robot <name>` 选择单台。两套 GELLO、三台相机的完整示例为 [`docs/station.bimanual.example.yaml`](../../examples/stations/station.bimanual.example.yaml)。`Robot.from_yaml` 仅读取不含 `daemon:` 的单机器人文件。

上层包可通过 `parse_station`、`load_station` 或 `load_station_with_env`，使用 `sections={name: validator}` 注册额外的机器人映射站点分节（section）。与 `daemon:` 类似，这些分节描述机器人，但不属于设备配置：加载器在调用 `RobotConfig.from_dict` 前移除每个已注册键，校验其值，并将返回值存入 `StationRobot.sections`。设备层不解释分节内容。未注册的未知键仍会报错；单机器人和 `robots:` 映射均支持分节。设置向导渲染设备配置，不渲染上层分节。

**Franka Panda、Franka Hand 与一台 RealSense**

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

安全限制和遥操作均可选，三个限制键均从机器人 PC 的 YAML 读取：

- `safety_box`：`[xmin, ymin, zmin, xmax, ymax, zmax]`，米，位于 `read_state()["tcp_pose"]` 坐标系。Franka 为 `O_T_EE`，Piper 为**法兰**。仅限制位置，不限制方向。
- `joint_limits`：每关节一对 `[lo, hi]`，单位 rad；`home_joints` 必须在范围内。
- `max_delta_per_step`：`xyz_m` 和/或 `joint_rad`，限制伺服目标相对实测状态的距离，仅作用伺服，`move_to_*` 不受单步限制。
- 机器人 PC 的 YAML 为限制依据。越界 `servo_*` 目标被**裁剪**并计入 `safety_clips`；越界 `move_to_*` 被 `SafetyViolation` **拒绝**，不发命令并计入 `safety_rejects`。守护进程、`send_action` 和本地 API 均相同，见[安全规则](#safety-rules)。
- `teleop:` 将操作员输入设备加入机器人部件，见[第 7 步](#7-add-a-teleop-device)。

注意：

- `pyagxarm` 与 `piper_sdk` 夹爪必须设置 `shares_arm_session: true`，`franky` 与 `serl_http` 禁止该值，不符会在构建时抛 `ValueError`。
- `arm_backend: piper_sdk` 还要求 `gripper.backend: piper_sdk`。它忽略 `firmware_profile`，固定按 1 Mbit/s 打开总线，因此 `bitrate` 仅用于接口核对。
- `serl_http` 无关节空间控制，需要运行旧版桥接，见[旧模块](#legacy)。

可以不接触硬件，构建机器人以检查文件：

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

有 `teleop:` 时增加一行，例如 `  teleop: SpaceMouseTeleop [sdk=hid, connected=False]`。

<a id="2-plug-in-and-discover"></a>

### 2. 连接并发现设备

```bash
robot-infra discover --station robot.yaml
```

discovery 仅探测，不连接机器人。`--station` 可选；它对指定配置或[工作站文件](#setup-wizard)中每台机器人的 `robot_ip` 执行 ping，并探测 TCP 22、80、443、1337、50051。

未连接任何设备的机器实际输出如下：

```text
$ robot-infra discover --station robot.yaml
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

连接硬件后输出形式为 `realsense: <serial>, <serial>`、`spacemouse: <vid>:<pid>:<path>`，无 `hidapi` 时为 `/dev/hidrawN`，以及 `can0: operstate=up bitrate=1000000`、`172.16.0.2: ping=ok open ports=<list>`、`/dev/hidraw0`。不会列出 `fake` 后端。

`robot-infra --station robot.yaml --json discover`（或 `rwrl --json discover`）将同一报告打印为一个 JSON 对象：`schema_version: 1` 和六个列表，未发现时各列表为空，即 `cameras`（`backend`、`serial`）、`teleops`（`backend`、`device`）、`can_interfaces`（`name`、`operstate`、`bitrate`）、`hosts`（`ip`、`ping_ok`、`open_ports`）、`hid_devices`（路径）和 `unknown_hid`（`vid`、`pid`、`product`、`path`、`message`：看起来像 SpaceMouse、但不在设备表中的 HID 设备）。上述机器的输出如下（由同一报告推导，不是实际运行捕获）：

```json
{"schema_version": 1, "cameras": [], "teleops": [], "can_interfaces": [], "hosts": [{"ip": "172.16.0.2", "ping_ok": false, "open_ports": []}], "hid_devices": [], "unknown_hid": []}
```

| 部分 | 来源 | 检查要点 |
|---|---|---|
| `cameras` | 相机后端 `discover()`，RealSense 序列号。 | 将序列号填入 `cameras[].serial`。 |
| `teleops` | 遥操作后端 `discover()`；SpaceMouse 有 `hidapi` 时给 `vid:pid:path`，否则列所有 `/dev/hidraw*`；GELLO 匹配 `/dev/serial/by-id/usb-FTDI_*` 和 `usb-ROBOTIS_*`。 | 配置在 `teleop:` 的设备应出现。 |
| `can interfaces` | `/sys/class/net/*` 中 CAN 类型接口，通过 `ip -details link show` 取波特率。 | `operstate=up`，波特率与 YAML 一致。 |
| `hosts` | 对配置 `robot_ip` 执行 `ping` 和 TCP 连接。 | `ping=ok`；Franka 1337 开放通常表示 FCI 可访问。 |
| `hid devices` | `/dev/hidraw*`。 | SpaceMouse 和其他 HID 遥操作设备。 |
| `warnings`（仅在有内容时出现） | id 不在 SpaceMouse 设备表中的 3Dconnexion（`256f`）HID 设备，以及 Logitech（`046d`）多轴控制器。 | `WARN unknown 3Dconnexion-vendor HID device <vid>:<pid>; add it to spacemouse-devices.yaml (see docs/spacemouse-commissioning.md#site-device-table) (<path>)`：在[站点设备表](spacemouse-commissioning.md#site-device-table)中加一条。 |

没有 `--station` 也没有工作站文件时，`hosts:` 显示 `none`。

<a id="3-check-with-the-doctor"></a>

### 3. 运行 doctor 检查

```bash
robot-infra doctor --station robot.yaml [--robot NAME] [--mock] [--fix] [--json] [--install-sdk [--yes]]
```

未传 `--station` 时检查[工作站文件](#setup-wizard)。多机器人逐台检查，每份报告前有 `== robot '<name>' ==`，末尾有 `overall:`；`--robot NAME` 只检查一台。

doctor 按 0–5 顺序执行六层，`teleop:` 段为 0、1、2、4 层增加检查：

| 层 | 检查 | 说明 |
|---|---|---|
| 0 `deps` | 检查机器人所需的 pip extras 已安装：每个后端对应一组（`franky`、`pyagxarm`、`piper-sdk`、`realsense`、`spacemouse`、`gello`，GELLO 的 `event_device` 另需 `gello-events`），YAML 含 `daemon:` 时还需 `grpc`。缺失报 FAIL `missing extras for this station: franky, gello`，提示 `pip install -e 'robot_infra[franky,gello]'`；完整为 OK `extras ok: franky, gello, grpc`。无 `daemon:` 时（单进程路线）缺 `grpc` 仅 WARN `optional, not installed: grpc (daemon route)`，缺 `jpeg` 仅提示。 | 使用 `--mock` 时 fake 无需这些 extras，也不要求 grpc。 |
| 1 `sdk` | 后端名称可解析且 SDK 可导入，包括 `sdk:arm`、`sdk:gripper`、`sdk:camera:<name>`、`sdk:teleop:<backend>`。 | 未知后端跳过第 3–5 层。 |
| 2 `os` | ping `robot_ip` 与端口 1337（仅供参考）；CAN 接口存在/已启动/波特率；相机序列号已连接；`os:teleop:<name>`（发现对应后端的设备）；`os:hid`（存在可读写的 `/dev/hidraw*`，仅 SpaceMouse）；`os:serial`（GELLO 的 `port` 存在且可读写；`--fix` 尝试不加 sudo 执行 `setfacl -m u:$USER:rw <port>`）；`os:groups`（用户登录会话属于 GELLO 端口所需的 `dialout` 组和其 `event_device` 所需的 `input` 组）；`os:brltty`（brltty 可能占用 CH340 适配器）；`os:latency_timer`（FTDI 适配器的 `latency_timer` 为 1 ms）。 | 按配置字段驱动，无需 SDK 的后端（fake）跳过，绝不运行 sudo。 |
| 3 `handshake` | 除 `claim: false` 外取得设备占用锁，然后 `robot.connect()`。 | 失败或锁被占用时跳过第 4–5 层。 |
| 4 `feedback` | 机械臂返回状态、夹爪报告宽度、各相机 2 秒内有帧、`feedback:teleop:<name>` 轮询线程存活健康。 | |
| 5 `mode` | Franka 实时内核、模式与 libfranka 版本；Piper 状态、从臂反馈与 pyagxarm 固件配置，或 piper_sdk 夹爪行程。 | |

`feedback:gripper` 打印宽度（米）、占 `max_width_m` 的比例、抓取状态和后端来源；非有限数、比例超出 [0, 1] 或 `max_width_m <= 0` 会报 `WARN`，并提示读数不可信，核对 `max_width_m` 与 SDK 的宽度单位。

参数：

- `--mock` 用 fake 替换每个后端并跳过第 2 层，无硬件检查 YAML 与安装。
- `--fix` 对失败项执行自动修复再重查，例如 Franka Reflex/UserStopped/Guiding 下 `mode:robot_mode` 的 `arm.recover()`，无帧相机的 `camera.reopen()`，以及当前用户不能打开 GELLO 端口时不提权执行 `setfacl`（`os:serial`）。
- `--json` 输出 `{layer, name, ok, message, hint, skipped, warning}` 对象列表，多机器人为 `{<name>: [...]}`。
- `--install-sdk` 在 libfranka 不匹配后安装适配 wheel，确认后安装；加 `--yes` 不询问，见[wheel 选择](#installing-the-right-franky-wheel)。
- 退出码：任何机器人的任何检查失败为 `1`，否则 `0`，跳过项不算失败。 `WARN` 是通过但带操作提示，例如下次重插可能被 brltty 占用，不使运行失败；汇总为 `N ok (M with warnings)`。

doctor 会连接并断开机器人，不会关闭电源。

Franka 示例的 `--mock` 运行结果：

```text
$ rwrl doctor --station robot.yaml --mock
PASS           franka.deps: no extras needed (fake or SDK-free backends)
PASS           franka.sdk:arm: no SDK needed
PASS           franka.sdk:gripper: no SDK needed
PASS           franka.sdk:camera:wrist: no SDK needed
SKIPPED        franka.os:ping: skipped (--mock)
SKIPPED        franka.os:tcp: skipped (--mock)
SKIPPED        franka.os:camera:wrist: skipped (--mock)
PASS           franka.handshake: robot connected
PASS           franka.feedback:arm: arm feedback received
PASS           franka.feedback:gripper: gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
PASS           franka.feedback:camera:wrist: frames arriving
PASS           franka.mode:none: backend declares no mode checks
Result: no blocking findings; review SKIPPED/WARN coverage
```

相同配置在没有 franky、机器人和相机的机器上：

```text
$ rwrl doctor --station robot.yaml
FAIL           franka.deps: missing extras for this station: franky
    hint: pip install -e 'robot_infra[franky]' (from the repository root; or pip install 'rlgarden-robot-infra[franky]'); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
FAIL           franka.sdk:arm: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or: pip install 'rlgarden-robot-infra[franky]')
FAIL           franka.sdk:gripper: FrankyGripper needs the 'franky' package
    hint: pip install franky-control (or: pip install 'rlgarden-robot-infra[franky]')
PASS           franka.sdk:camera:wrist: 'pyrealsense2' importable
FAIL           franka.os:ping: 172.16.0.2 does not reply
    hint: check: robot powered on and brakes released; Desk reachable at https://172.16.0.2/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP
PASS           franka.os:tcp: port 1337 closed (informational)
FAIL           franka.os:camera:wrist: no RealSense device found (serial unset)
    hint: plug in the camera and run `robot-infra discover`
FAIL           franka.handshake: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or: pip install 'rlgarden-robot-infra[franky]')
SKIPPED        franka.feedback: skipped: handshake failed
SKIPPED        franka.mode: skipped: handshake failed
Needs attention: 6 FAIL
  FAIL           franka.deps: missing extras for this station: franky
  ...
Result: unresolved failures / blocked checks / pending user actions
```

加入 `teleop: {backend: spacemouse}` 后，`--mock` 替换为 fake 并丢弃设备选项，增加以下输出：

```text
$ rwrl doctor --station robot.yaml --mock
...
PASS           franka.sdk:teleop:fake: no SDK needed
SKIPPED        franka.os:teleop:teleop: skipped (--mock)
...
PASS           franka.feedback:teleop:teleop: teleop poll thread alive
```

不使用 `--mock`，且未安装 `hidapi`、未连接 SpaceMouse 时，遥操作输出如下；`os:hid` 仅检查 SpaceMouse 后端：

```text
FAIL           franka.sdk:teleop:spacemouse: SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)
    hint: pip install -e '.[spacemouse]'
FAIL           franka.os:teleop:teleop: no 'spacemouse' teleop device found
    hint: plug in the teleop device and run `robot-infra discover`
FAIL           franka.os:hid: no HID device found
    hint: no /dev/hidraw* node: check the USB cable and port; inside Docker pass it with `docker run --device=/dev/hidrawN`
```

存在 `/dev/hidraw*` 但当前用户无法打开时，`os:hid` 以 `no read/write access to ['/dev/hidraw3']` 失败，并提示用 `sudo cp` 安装[随包提供的厂商 udev 规则](#spacemouse-python-dependencies-and-udev)、加入 `plugdev`、重载规则、重新登录并拔插。Python SDK 缺失时则给出上述 pip 命令（NUC、仓库根目录、项目环境激活）；原生模块导入失败会单独报告 wheel/平台加载问题。

未连接 CAN 适配器时的 Piper 示例，输出有所删节：

```text
FAIL           piper.os:can_exists: CAN interface can0 not found
    hint: run `bash can_activate.sh can0 1000000` from piper_sdk, or `sudo ip link set can0 up type can bitrate 1000000`
FAIL           piper.os:can_up: can0 operstate=missing
FAIL           piper.os:can_bitrate: can0 bitrate=None, expected 1000000
FAIL           piper.handshake: CAN interface 'can0' not found
    hint: plug in the USB-CAN adapter and run: bash can_activate.sh can0 1000000  (piper_sdk) or: sudo ip link set can0 up type can bitrate 1000000
```

未安装 pyAgxArm 时，SDK 提示为 `pip install 'rlgarden-robot-infra[pyagxarm]'  (pyAgxArm is installed from GitHub)`。pyAgxArm 不在 PyPI，可选组从 GitHub 安装。

<a id="troubleshooting"></a>

#### 故障排查

下表保留检查打印的消息和提示，`<ip>`、`can0` 等按实际配置替换。

| 症状 | 检查位置与输出 | 处理方式 |
|---|---|---|
| 缺 pip extras | `deps` FAIL `missing extras for this station: franky, gello`，提示 `pip install -e 'robot_infra[franky,gello]' ...`。 | 从仓库根目录运行给出的命令，列表精确对应缺失的 extras。 |
| Franka 不可达：未上电、制动锁定、Desk 无控制权或 FCI、线缆接在 Arm 口、防火墙 | `os:ping` FAIL `<ip> does not reply` 或 `handshake` FAIL `cannot reach Franka at <ip>: ...`（例如 `libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.`），提示 `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP`。 | 按提示逐项检查。Desk 页面能打开还不够，必须持有控制权并激活 FCI。PC 接 Control box，而非机械臂底座；防火墙须允许来自机器人的 UDP。 |
| libfranka 不匹配 | `handshake` FAIL `libfranka mismatch: robot server 10, installed library 9`，提示 `the robot server version 10 comes from libfranka's handshake error (authoritative); to cross-check, read the system version in Desk → Settings → System and look it up in the compatibility matrix https://frankarobotics.github.io/docs/compatibility.html; then: robot-infra doctor --station <cfg> --install-sdk`。 | 见[安装正确的 franky wheel](#installing-the-right-franky-wheel)。连接后 `mode:libfranka_version` 显示已安装的 libfranka 版本。 |
| 用户不在 `dialout` / `input` | `os:groups` FAIL `not in group(s) dialout, input; no read/write access to [...]`（ACL 或 udev 已允许访问时为 WARN），提示 `sudo usermod -aG dialout,input $USER, then log out and back in`。 | 执行命令后重新登录，运行中的会话保留旧组。 |
| GELLO 串口不可读 | `os:serial` FAIL `no read/write access to <port>`，提示 `grant this user access now: setfacl -m u:$USER:rw <port> ...`。 | `--fix` 尝试不加 sudo 执行 `setfacl`，否则加 sudo。ACL 在重插后丢失，组或 udev 规则才持久。 |
| brltty 占用 CH340 适配器 | `os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; brltty has likely claimed <port>`（端口仍存在时为 WARN），提示 `sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter`。 | 屏蔽 brltty（`--runtime` 屏蔽在重启后失效），然后重新插入。 |
| FTDI 读取慢 | `os:latency_timer` WARN `ttyUSB0 latency_timer is 16 ms; 1 ms is recommended for Dynamixel reads`，提示包含 udev 规则 `ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"`。 | 安装 udev 规则（提示给出文件名和重载命令），然后重新插入。CH340 适配器没有 `latency_timer`。 |
| 无 PREEMPT_RT 内核。 | `realtime: enforce` 下 `mode:realtime_kernel` 报 `realtime: enforce but the kernel is not PREEMPT_RT`，或 `handshake` 报 `libfranka realtime setup failed ...`；`realtime: ignore` 时通过并显示 `control jitter possible`。 | 安装 PREEMPT_RT 和实时组限制，或用 `realtime: ignore`。 |
| Franka 为 Reflex / UserStopped / Guiding。 | `mode:robot_mode` 报 `robot mode is Reflex`，提示 `release the external activation device / clear errors in Desk, then run with --fix (recover)`。 | 释放外部激活设备后运行 doctor `--fix`。 |
| CAN 接口缺失、未启动或波特率错。 | `os:can_exists`、`os:can_up`、`os:can_bitrate` 失败，握手报 `CAN interface 'can0' is down, not up` 或 `runs at N bit/s, expected 1000000`。 | `sudo ip link set can0 up type can bitrate 1000000`，或运行 piper_sdk 的 `can_activate.sh can0 1000000`。 |
| Piper 未使能，可能是供电或急停。 | `handshake` 报 `Piper on 'can0' did not enable within 5s`，提示 `check that the arm is powered, that 'can0' is up at 1000000 bit/s, and that its emergency stop is released`。 | 接通电源、释放急停并重试；连接后的 `mode:piper_status` 报告状态。 |
| 无 CAN 反馈，机械臂可能处于主臂或示教模式。 | `handshake` 报 `... enabled but sent no joint feedback within 2s`，或 `mode:follower` 报 `no joint feedback`，提示 `no CAN feedback: the arm may be in master/teach mode; switch it to follower (slave) mode and power-cycle`。 | 切换从臂模式并断电重启。 |
| pyagxarm 的 Piper 固件配置不匹配。 | `handshake` 报 `firmware probe ... failed` / `cannot map Piper firmware ...`，提示 `set firmware_profile explicitly (default\|v183\|v188\|v189) in the robot config`。 | 在 YAML 显式设置 `firmware_profile`。 |
| 相机序列号不存在。 | `os:camera:<name>` 报 `camera '<name>' serial '<s>' not found`，提示 `set serial; discovered: [...]`；未填 `serial` 时为 `no RealSense device found (serial unset)` 或 `serial unset and N devices attached; set serial to one of [...]`；握手报 `RealSense serial '<s>' for camera '<name>' not attached`。 | 从 `robot-infra discover` 复制序列号，多相机时必须填 `serial`。 |
| 相机没有帧。 | `feedback:camera:<name>` 报 `no frames within 2s`，提示 `check USB 3 bandwidth; try another port; run with --fix to reopen`；或 `RealSense '<s>' failed to start`，提示 `check the USB 3 cable/port and udev rules (librealsense scripts/setup_udev_rules.sh)`。 | 使用 USB 3 端口和线缆，避免多相机共享集线器，安装 librealsense udev 后尝试 `--fix`。 |
| 守护进程不可达。 | 客户端 `TransportError: robot daemon unreachable at <addr>`。 | 启动服务，检查 `--bind`、地址和防火墙。 |
| 令牌不匹配。 | 客户端 `TransportError: token mismatch`，提示 `pass the same --token to the daemon and the remote config`。 | 两端使用同一令牌。 |
| watchdog 已停臂。 | 客户端 `DeviceFault: the daemon watchdog stopped the arm after 0.62s without actions (limit 0.5s)`，提示 `... call set_control_mode(...) again to resume servoing, and keep the action rate above 1/watchdog_s`。 | 提高动作频率至 `1/watchdog_s` 以上，或增大 `--watchdog-s`，再调用 `set_control_mode`。 |
| 机器人已被占用。 | 客户端 `DeviceBusy: device 'franka:172.16.0.2' is claimed by another process`，提示停止占用者或设置 `claim: false`。 | 停止本机通过 `RobotService` 使用同设备的守护进程或 `LocalClient`；普通 `Robot.from_yaml` 脚本不持该锁。有意退出锁保护时配置 `claim: false`。 |

<a id="installing-the-right-franky-wheel"></a>

#### 安装正确的 franky wheel

`franky` extra 安装捆绑某个 libfranka 版本的默认 `franky-control` wheel。机器人服务器版本不同会拒绝握手，doctor 报告 `libfranka mismatch: robot server <N>, installed library <M>`。此时运行：

```bash
robot-infra doctor --station robot.yaml --install-sdk         # asks before installing
robot-infra doctor --station robot.yaml --install-sdk --yes   # installs without asking
```

使用 `--install-sdk` 时，doctor 从 libfranka 握手错误获取权威 robot server 版本。Desk 不直接显示该数值；在 Settings → System 读取系统版本如 5.9.2，通过[兼容表](https://frankarobotics.github.io/docs/compatibility.html)映射交叉核对，然后打印实际 pip 命令：

```bash
<this python> -m pip install franky-control --extra-index-url https://timschneider42.github.io/franky/whl/by-robot-server-version/<N>/ --force-reinstall --no-deps
```

随后询问 `Install now? [y/N]`，只有 `y`（忽略大小写和两侧空白）才安装到运行 doctor 的解释器；之后用新解释器重跑相关机器人的 doctor，因为当前解释器中仍加载着旧 libfranka，并返回该次运行的退出码。除非指定 `--yes`，否则未经确认不会安装。`--json` 永不询问或安装，仅输出 `{"results": <the original device report>, "install_sdk": [{robot, server_version, library_version, command}]}`。报告不同服务器版本的机器人不会自动安装，因为同一 Python 环境只有一个 franky wheel；改为逐台打印命令。`--no-deps` 假设 franky 的依赖（`numpy`、`websockets`）已具备，先安装 `franky` extra 即可满足。

项目环境已安装 `real-world-rl` 时，`rwrl doctor` 是独立且覆盖更广的诊断入口：检查主机、项目环境和已配置设备，预览针对性修复（`--fix` 应用主机与软件修复），并写入含主机检查和修复的版本化 JSON 外层。设备探测始终使用 `fix=False`，不会修复设备。在 NUC 激活项目环境后运行，见 [rwrl](cli.md#robot-host-diagnosis-and-repair)。

<a id="4-serve-the-robot"></a>

### 4. 提供机器人服务

```bash
pip install -e '.[grpc]'   # plus the robot's backend extras
robot-infra daemon --station robot.yaml --bind 0.0.0.0:50051 [--token T] [--watchdog-s 0.5] [--no-preflight]
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--station` | [工作站文件](#setup-wizard) | 机器人或工作站 YAML。 |
| `--robot` | 所有机器人 | 只服务指定机器人；省略时多机器人各启动一个进程。 |
| `--bind` | 机器人 `daemon.bind`，否则 `0.0.0.0:50051` | 监听 `host:port`，仅限单机器人。 |
| `--token` | 机器人 `daemon.token`，否则无 | 共享令牌，客户端经 gRPC 元数据 `x-robot-token` 发送相同值，仅限单机器人。 |

多机器人且未指定 `--robot` 时，命令作为监督进程，为每台运行 `robot-infra daemon --station <station> --robot <name>`，各自使用独立进程会话，并传递 `--watchdog-s`、`--jpeg-quality`、`--no-preflight`、`--transport`。它转发 SIGINT/SIGTERM，等待所有子进程，以第一个非零退出码结束。此时 `--bind` / `--token` 会报错，应逐臂设置 `daemon.bind` / `daemon.token`；两台机器人使用同一 `daemon.bind` 也报错。SIGKILL 杀死监督进程后守护进程仍运行，需用 SIGTERM 停止。
| `--watchdog-s` | `0.5` | 超过指定秒数未收到动作，让伺服机械臂进入空闲。 |
| `--no-preflight` | 关闭 | 启动服务前跳过 doctor。 |
| `--transport` | `grpc` | 当前唯一传输方式。 |
| `--jpeg-quality` | 无 | 所有相机 `color` 图像的 JPEG 质量 1–100，详见下文。 |

`--jpeg-quality N` 将各相机 `color` 编为 JPEG，降低 WiFi 带宽；守护进程**及所有客户端**需安装 Pillow 或 OpenCV，例如 `pip install -e '.[jpeg]'`。它是全局**有损**压缩，仅作用 `color`，`depth` 等字段仍为原始数据，客户端解码为 `uint8 (H, W, 3)`。策略看到压缩像素，因此**训练与部署必须采用相同值**或都不设置。`robot.describe` 顶层 `image_encoding` 报告 `{"jpeg": 80}`，原始传输为 `null`，客户端也可读 `robot.cameras["wrist"].image_encoding`。缺少 Pillow/OpenCV 时 `connect()` 抛 `SdkMissing: JPEG frames need Pillow or OpenCV`，提示 `pip install "rlgarden-robot-infra[jpeg]"`。

守护进程和客户端来自同仓库，升级顺序为**先客户端，后守护进程**。新 `arm.health` 多出 `safety_clips`、`safety_rejects`，旧客户端无法解析；`--jpeg-quality` 也要求支持 JPEG 解码的客户端。JPEG 支持之前的客户端无法解码，具体错误尚未检查。

执行过程：

1. **配置。** 加载 YAML 后，为 `""` 或 null 的字段读取同名大写环境变量，例如 `FIRMWARE_PROFILE=v188`。`ROBOT_IP`、`CAN_INTERFACE` 仅在 YAML 显式设置 `robot_ip: ""` / `can_interface: ""` 时生效，因为其默认值非空。相机 `serial` 为空且同后端仅连接一台时自动填充。守护进程、doctor、`robot-infra discover --station` 均应用自动填充，`Robot.from_yaml` 按文件原值读取。
2. **预检查。** 除 `--no-preflight` 外，运行完整非 `--mock` doctor。任何失败都打印 `preflight failed; fix the items above or start with --no-preflight` 并以 1 退出。doctor 持设备锁连接和断开，同机器人已有服务会导致 `handshake` 失败。fake 跳过 OS 检查，显示 `backend 'fake' needs no OS device`，因此 fake 演示无需关闭预检查。
3. **启动输出**，下例来自 fake 配置：

   ```text
   franka (franka)
     arm: FakeArm [sdk=None, connected=False]
     gripper: FakeGripper [sdk=None, connected=False]
     cameras: PartGroup [sdk=None, connected=False]
       wrist: FakeCamera [sdk=None, connected=False]
     teleop: FakeTeleop [sdk=None, connected=False]
   serving franka 'franka' on 127.0.0.1:50061 (token required)
   ```

   客户端调用 `connect()` 时才连接机器人并取得设备锁，`disconnect()` 时释放。
4. **Watchdog。** 在 `CARTESIAN_SERVO` 或 `JOINT_SERVO` 中，每隔 `watchdog_s / 4` 检查最后动作时间。`send_action`、`step`、`servo_cartesian`、`servo_joints` 都算动作，进入伺服模式也开始计时。超过 `watchdog_s` 后调用 `arm.stop()`、切至 `IDLE`，记录 `watchdog: no action for 0.624s in joint_servo mode; stopping arm 'arm' and idling it`。之后伺服命令抛 watchdog `DeviceFault`，直到重新调用 `set_control_mode(...)`。仅允许在 `IDLE` 下执行的 `move_to_*` 不受 watchdog 监控。
5. **遥操作。** YAML 含 `teleop:` 时提供 `teleop.<name>.read` 与 `teleop.<name>.reset`。读取不刷新 watchdog，客户端 `timeout_s` 最多 2 秒，避免长期占用 gRPC 工作线程。存在 `teleop` 配置就必须有设备，缺失会令 `connect()` 失败并回滚整个机器人；无需遥操作的部署应移除该段。
6. **Ctrl-C / SIGTERM。** 停止传输与 watchdog，断开机器人并释放锁，保持电源和电机使能状态，见[安全规则](#safety-rules)。

客户端根据 `robot.describe` 构建镜像。前述 fake 机器人响应如下，已有删节：

```text
arm: {"num_joints": 7, "supported_state_fields": ["tcp_pose", "tcp_vel", "joint_pos", "joint_vel", "tcp_force", "tcp_torque"], "control_mode": "idle", "safety": {"safety_box": [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5], "joint_limits": [[-1.0, 1.0], ...], "max_delta_per_step": {"xyz_m": 0.05, "joint_rad": 0.1}}, "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
image_encoding: {'jpeg': 80}
camera cfg: {'name': 'wrist', 'serial': '', 'width': 64, 'height': 48, 'fps': 30, 'enable_depth': False, 'backend': 'fake'}
teleop: {'name': 'teleop', 'backend': 'fake', 'action_space': 'joint_abs', 'has_gripper': True, 'hold_s': 0.0}
```

未配置限制时 `arm.safety` 为 `null`；`arm.home_joints` 为 YAML 值或 `null`。
描述还包括 `arm.safety_capabilities`：`task_workspace_v1` 表示执行笛卡尔工作区约束，`accepted_cartesian_target_v1` 表示保持已接受的目标，Franky 还提供 `stop_trace_v1`。客户端在使用新行为前协商能力；旧服务端无法证明支持这些能力。已连接遥操作设备的描述含 `device_identity`（USB 厂商/产品 ID 及可用的序列号/型号）；`RemoteTeleop.device_identity` 暴露相同身份信息，供标定档案（profile）验证使用。连接前可能无法取得身份信息。

现有机械臂命令之外新增四个 RPC：`arm.last_cartesian_target`、`arm.begin_stop_trace`、`arm.stop_with_trace`（`post_stop_s=2.1`）与 `arm.last_stop_trace`。`last_cartesian_target` 是当前控制片段内最近一个通过机械臂安全限制并交给后端的笛卡尔目标的副本。任何 `stop()`、`stop_with_trace()`、控制模式切换或断开连接都会结束该片段；此后该值为 `null`，直到下一次成功的 `servo_cartesian`。它始终不是实测反馈。`piper_sdk` 和 `pyagxarm` 仍会在后端转换目标（毫米或千分之一度取整、四元数转 RPY），因此它并不精确等于这些机械臂实际收到的值。`Arm.stop()` 是模板方法：先清除目标，再调用后端的 `_stop()`。后端实现 `_stop()`，绝不实现 `stop()`。`begin_stop_trace` 在现有连接上开始有界采样；SDK 指纹/静态元数据在停止锁与服务锁之外采集，并按连接缓存。等待先前采样线程结束也在停止锁之外进行。

`stop()` 保留 Franky 原有的错误语义：失败只记录在健康状态中，不抛异常。只有 `stop_with_trace` 会因停止失败抛 `DeviceFault`，同时在 `last_stop_trace` 保留 FAIL 证据。缺少记录器时，它先发送停止命令，再尝试创建记录器；准备失败会在直接停止后产生 BLOCKED 证据。停止后观察期间仍可调用读取/停止 RPC，但拒绝控制写入。`robot.step` 中缓慢的相机采集不会持有 `arm.stop` 所需的分派锁。

停止轨迹记录停止开始时的 `control_mode`、`tracker_active`，以及每份样本的 SDK `robot_mode`。机械臂已处于 IDLE/未活动状态时，即使每份姿态都静止，也会得到 BLOCKED（`no active motion at stop`）。机器人模式故障和停止失败得到 FAIL；模式或覆盖证据缺失、未知时不能得到 PASS。基于采样的停止验收不构成连续时间的物理保证。

<a id="5-use-the-robot-from-the-gpu-machine"></a>

### 5. 从 GPU 机器访问机器人

```bash
pip install -e '.[grpc]'
```

```python
import real_world_rl.server  # registers robot type "remote"
from real_world_rl.devices import ControlMode, Robot

robot = Robot.from_dict({"type": "remote", "address": "robot-pc:50051", "token": "T"})
robot.connect()                       # builds arm/gripper/cameras from the daemon's describe
print(robot.describe_text())
robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
q = robot.arm.read_state()["joint_pos"]
obs = robot.step({"arm": {"joint_pos": q}, "gripper": {"gripper": [1.0]}})  # action, then observation
print(robot.health())                 # Health(ok=True, detail='rtt=1.2 ms, watchdog_trips=0', ...)
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

没有先 `import robot_infra.server` 时，`Robot.from_dict` 会报 `unknown robot type 'remote'; registered: ['franka', 'piper']`。

- 动作按部件嵌套，机械臂键由控制模式决定：`CARTESIAN_SERVO` 使用 `tcp_pose`，7 个值即 xyz 与 xyzw 四元数；`JOINT_SERVO` 使用 `joint_pos`。夹爪使用 [0, 1] 归一化宽度。
- `step()` 返回 `{"arm": {...}, "gripper": {"gripper_pos": ...}, "cameras": {<name>: {"color": ...}}}`，机械臂字段来自后端 `SUPPORTED_STATE_FIELDS`。
- `RemoteRobotConfig` 还含 `timeout_s` 默认 5.0、`reconnect_attempts` 默认 5、`reconnect_backoff_s` 默认 0.5。断连后重连并只重试只读调用；命令不会重试，应重连并重新设置控制模式。
- 单机无需 gRPC 时，可在同进程包装本地机器人：

  ```python
  from real_world_rl.devices import Robot
  from real_world_rl.server import RemoteRobot, RobotService
  from real_world_rl.server.transport import LocalClient

  service = RobotService(Robot.from_yaml("robot.yaml"), watchdog_s=0.5)
  robot = RemoteRobot.from_client(LocalClient(service))   # same API as above, incl. watchdog
  ```

- 客户端镜像设备信息：`robot.arm.safety_limits` 为前述 `arm.safety` 字典或 `None`，`robot.arm.home_joints` 为列表或 `None`，相机编码为 `robot.cameras["wrist"].image_encoding`，遥操作为 `robot.teleop`，无 `teleop:` 时为 `None`。限制由守护进程执行，`RemoteRobotConfig` 忽略本地 `safety_box`、`joint_limits`、`max_delta_per_step`、`teleop`。
- 越界命令在客户端表现如下，在 `robot.disconnect()` 前运行：

  ```python
  import numpy as np
  from real_world_rl.devices import SafetyViolation

  robot.arm.set_control_mode(ControlMode.JOINT_SERVO)
  robot.arm.servo_joints(np.full(7, 5.0))      # no error: clipped on the daemon, safety_clips += 1
  robot.arm.set_control_mode(ControlMode.IDLE)
  try:
      robot.arm.move_to_joints(np.full(7, 5.0))   # outside joint_limits
  except SafetyViolation as error:
      print(error.message, error.hint)            # nothing was sent; the arm and its mode are unchanged
  print(robot.arm.health().safety_clips, robot.arm.health().safety_rejects)   # cumulative counters since the daemon started, e.g. 2 1
  print(robot.remote_health()["arm"]["safety_clips"])                         # the same counters from the daemon's robot.health
  ```

  `SafetyViolation` 继承 `DeviceError`，不属于 `DeviceFault`；它不锁存故障，无需 `recover()`，也不刷新 watchdog。`robot.health()` 返回 `Health`，每臂计数位于 `robot.arm.health()` 和 `robot.remote_health()`。
- `real-world-rl` 将该机器人封装为 gym 环境。

<a id="6-use-the-robot-locally-no-daemon"></a>

### 6. 本地使用机器人 无守护进程

在机器人 PC 上可直接调用设备层：

```python
import numpy as np
from real_world_rl.devices import ControlMode, Robot

robot = Robot.from_yaml("robot.yaml")
robot.connect()
robot.arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
pose = robot.arm.read_state()["tcp_pose"]
robot.send_action({"arm": {"tcp_pose": pose}, "gripper": {"gripper": np.array([0.0])}})
obs = robot.get_observation()
robot.arm.set_control_mode(ControlMode.IDLE)
robot.disconnect()
```

本地路径不使用守护进程的 watchdog 和占用锁，但仍执行安全限制。`Robot.from_yaml` 按 YAML 的 `safety_box` / `joint_limits` / `max_delta_per_step` 构建机械臂，因此 `robot.arm.safety_limits`、`robot.arm.health().safety_clips` 和 `SafetyViolation` 行为与经守护进程相同。

<a id="supervising-a-single-process-robot-run"></a>

#### 监督单进程机器人运行

在机器人主机的仓库根目录激活完整项目环境后，使用 `rwrl supervise` 和 `rwrl record`。无守护进程时，机器人进程通过进程内 SDK 驱动机械臂，因此外部无法调用 `arm.stop()`。`rwrl supervise` 将它作为独立 watchdog 进程的子进程运行：

    rwrl supervise --log runs/<run>/supervise.jsonl -- rwrl record --config hil.yaml ...

| 参数 | 默认 | 含义 |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | 静默后向子进程组依次 SIGINT、SIGTERM、SIGKILL |
| `--grace-s` | `3.0` | SIGINT 到 SIGTERM |
| `--kill-s` | `2.0` | SIGTERM 到 SIGKILL |
| `--startup-s` | `30.0` | 首消息或此时间前不要求心跳 |
| `--long-op-s` | `60.0` | 报告 reset 等长操作后允许的静默 |
| `--log` | 无 | jsonl，pid、单调时间升级事件、退出码/信号 |

子进程在独立会话中运行，通过 `ROBOT_INFRA_HEARTBEAT_FD` 接收管道写端。`robot_infra.supervise.Heartbeat.from_env()` 是子进程端接口：每步调用 `beat()`，重置前调用 `begin_long_operation()`；无该变量时不执行操作。Ctrl-C、SIGTERM 或终端关闭只传到监督进程，由它转发 SIGINT 并等待子进程；它以子进程退出码结束，升级信号后为非零。监督进程死亡时子进程收到 SIGINT，下次心跳抛 `SupervisorLost`。仅 Linux，依赖 prctl 和进程组。

监督进程只能终止进程，随后 libfranka 断开控制连接，机器人由固件以 `Reflex` 状态停止。在 Desk 清错或运行 `robot-infra doctor --station robot.yaml --fix` 恢复。安装完整项目后，`rwrl --station robot.yaml recover` 可直接清除（它调用机械臂的 `recover()`，与设备 doctor 的 `--fix` 对 Reflex 执行的自动修复相同）；`rwrl doctor --fix` 只修复主机和软件，从不恢复机械臂。进程外受控 `stop()` 只存在于守护进程路线（`--watchdog-s`）；进程内第一层为通过 `real_robot.watchdog_s` 启用的 `StepWatchdog`（见 `docs/real-world-experiments.md`）。

<a id="7-add-a-teleop-device"></a>

### 7. 添加遥操作设备

`teleop:` 将操作员输入设备作为机器人部件。以下 fake 后端可无硬件使用，每次 `read()` 推进一个脚本步骤：

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

步骤格式为 `{action, gripper, buttons, events, active, hold}`。`hold` 是持续读取次数，事件仅在第一次触发。`fail_after: n` 使第 n 个样本后的所有轮询失败，用于测试故障路径。

```python
robot = Robot.from_yaml("robot.yaml")          # local; RemoteRobot works the same way
robot.connect()
sample = robot.teleop.read()                   # blocks for a new sample, consumes the events
print(sample.seq, sample.action_space, sample.action, sample.gripper, sample.intervening, sample.events["episode_end"])
latest = robot.teleop.latest()                 # no blocking, does not consume events
robot.teleop.reset()                           # drops latched events and the hold_s hysteresis
```

`read()` 返回 `TeleopSample`，经守护进程 `teleop.<name>.read` 时以相同键字典传输：

| 字段 | 类型 | 含义 |
|---|---|---|
| `action` | `float32` ndarray | `cartesian_delta` 为 `(6,)`、[-1, 1] 归一化速度增量；`joint_abs` / `joint_delta` 为 `(N,)`、rad 关节值。 |
| `action_space` | `str` | `cartesian_delta`、`joint_abs` 或 `joint_delta`。 |
| `gripper` | `float` 或 `None` | [0, 1] 归一化宽度目标；`None` 表示无夹爪通道。 |
| `buttons` | `dict[str, bool]` | 当前电平；SpaceMouse 为 `button_0`、`button_1`。 |
| `events` | `dict[str, bool]` | 锁存边沿 `episode_end`、`success`、`failure`、`rerecord`，以及实际触发的后端专属边沿（SpaceMouse 的 button_0/button_1），始终包含四键，`read()` 后清除。 |
| `intervening` | `bool` | 人工正在操作，由后端 `active` 加 `hold_s` 延长。 |
| `timestamp` | `float` | 轮询机器的 `time.monotonic()`，不能跨主机直接比较。 |
| `seq` | `int` | 每个样本递增，跨重连仍连续；间断表示丢样本。 |

`read(timeout_s)` 超时抛 `DeviceFault`，连续 5 次轮询失败后抛可恢复 `DeviceFault`，需重连机器人；等待期间断开则抛 `DeviceNotConnected`。它是单消费者接口，两个客户端读取同一遥操作设备会分走事件。

`backend: spacemouse`（extra `spacemouse`；选项 `device_index` 为 0、`deadzone` 为 1e-3、`invert_axes`、`poll_interval_s` 为 0.005）是带夹爪的 `cartesian_delta` 设备。推杆 twist 为 `[-y, x, z, -roll, -pitch, -yaw]`；每次按键锁存为 `button_0` / `button_1` 事件，避免两次读取间漏按。按钮 0 仍切换设备层粘滞夹爪（连接时打开），两按钮均不发送 `episode_end`。real_world_rl 的 `--teleop-device spacemouse` 另赋语义：0 接管、1 夹爪，忽略设备 `gripper` / `intervening`。需可读写 `/dev/hidraw*`；`robot-infra doctor` 通过 `os:hid` 检查并打印 udev 规则安装命令，见[已知限制](#known-limitations-and-needs-hardware-verification)。

`backend: gello` 使用 `gello` 组，`event_device` 另需 `gello-events`。它读取经 U2D2 或 OpenRB-150 连接、Protocol 2.0 的被动 Dynamixel 主臂，输出 `joint_abs`，每个 `joint_ids` 一关节，设置 `gripper_id` 后有夹爪。每次轮询同步读一次，关节 `i` 为 `wrap((raw_rad - joint_offsets_rad[i]) * joint_signs[i])`，环绕区间 `[wrap_centers_rad[i] - pi, wrap_centers_rad[i] + pi)`，连接无需指定起始姿态；中心应取 `joint_limits` 中点，Panda 第 6 关节超过 pi，需正中心。夹爪为 `clip((raw - gripper_closed_rad) / (gripper_open_rad - gripper_closed_rad), 0, 1)`，1 表示打开，可用 `gripper_binarize_threshold` 二值化。默认 `intervention: always` 每次都报告 `intervening`；`toggle` 经 `evdev` 读取机器人 PC 的 `event_device` 脚踏或键盘，其他键由 `event_keys` 触发 `episode_end` / `success` / `failure` / `rerecord`。连接和断开都关闭力矩，缺舵机或同步读取失败会使轮询失败，避免复用旧样本。完整选项见 [`docs/robot.franka.example.yaml`](../../examples/stations/robot.franka.example.yaml)。每台需标定一次，关闭力矩并手动移动；[配置向导](#setup-wizard)直接执行并写入工作站文件。独立标定命令会打印可粘贴配置：

```bash
robot-infra gello-calibrate \
    --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_XXXX-if00-port0 \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --calibration-pose 0 0 0 -1.57 0 1.57 0 --joint-limits-yaml robot.yaml
```

标定逐关节要求沿机器人正方向移动以检查方向，根据标定姿态在四分之一圈网格求偏移，捕获扳机开闭角，并读取 YAML `joint_limits` 的环绕中心。串口需要 `dialout` 组等读写权限，doctor 的 `os:serial` 和 `os:groups` 会检查，并以 `os:brltty` 提醒 CH340 冲突，以 `os:latency_timer` 提醒 FTDI `latency_timer` 读取延迟。后端不写舵机波特率，需在 Dynamixel Wizard 中设置；8 舵机估计 1000000 波特约 130 Hz、57600 波特约 25 Hz。

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

后端按名称注册，例如 `Arm.register("franky")`。`Arm.backends()`、`Gripper.backends()`、`Camera.backends()`、`Teleop.backends()` 列出后端，`Robot.types()` 列机器人类型。`shares_arm_session: true` 的夹爪借用机械臂会话作为 `owner`，无需独立连接。

| 后端 | SDK 与可选组 | 关节控制 | 力与力矩（`tcp_force` / `tcp_torque`） | 说明 |
|---|---|---|---|---|
| 机械臂 `franky` | `franky`（`franky`） | 支持 | 支持 | 伺服采用力矩级阻抗跟踪，Ruckig `move_to_*` 仅 `IDLE`；应用 `realtime` 和 `relative_dynamics_factor`。 |
| 机械臂 `serl_http` | 无，HTTP | 不支持 | 支持 | 访问 `bridge_url` 的旧 ROS1 客户端，仅笛卡尔伺服，状态无 `joint_pos` / `joint_vel`。 |
| 机械臂 `pyagxarm` | `pyAgxArm`（`pyagxarm`） | 支持 | 不支持 | Piper 默认，探测或设置 `firmware_profile`；状态含法兰 `tcp_pose`、`joint_pos`、`joint_vel`。 |
| 机械臂 `piper_sdk` | `piper_sdk`（`piper-sdk`） | 支持 | 不支持 | 厂商 SDK，固定 1 Mbit/s，`stop()` 重发实测关节以保持。 |
| 机械臂 `fake` | 无 | 支持 | 支持 | 确定性一阶模拟，用于测试和 `--mock`。 |
| 夹爪 `franky` | `franky` | - | - | Franka Hand，独立会话。 |
| 夹爪 `serl_http` | 无 | - | - | 通过桥接，仅二值开闭。 |
| 夹爪 `pyagxarm` / `piper_sdk` | 与机械臂相同 | - | - | 要求 `shares_arm_session: true` 与匹配机械臂后端。 |
| 相机 `realsense` | `pyrealsense2`（`realsense`） | - | - | RGB，`enable_depth` 可增加对齐深度，单位米。 |
| `fake` 夹爪 / 相机 | 无 | - | - | 测试和 `--mock`。 |
| 遥操作 `spacemouse` | `hidapi==0.14.0.post4`，导入 `hid`（`spacemouse`） | - | - | 笛卡尔速度增量与锁存夹爪；`Teleop.backends()` 为 `fake`、`gello`、`spacemouse`。 |
| 遥操作 `gello` | `dynamixel-sdk`（`gello`），`evdev` 来自 `gello-events` | - | - | GELLO `joint_abs` 主臂与可选夹爪，力矩关闭，evdev 切换与事件。 |
| 遥操作 `fake` | 无 | - | - | 脚本样本、`push()` 注入、`fail_after`，用于测试和 `--mock`。 |

**`Arm` 采用模板方法。** 公开 `servo_cartesian`、`servo_joints`、`move_to_pose`、`move_to_joints` 先校验形状、有限值与单位四元数，再应用限制，最后调用 `_servo_cartesian`、`_servo_joints`、`_move_to_pose`、`_move_to_joints`。同样，公开 `stop()` 先清除 `last_cartesian_target`，再调用后端钩子 `_stop`。后端仅实现这些钩子和 `read_state`、`recover`、`_enter_control_mode`，不覆盖公开方法，确保所有电机路径经过检查。`read_state()` 必须通过 `self._remember_state(state)` 返回；单步上限复用该缓存，最多 `MAX_DELTA_STATE_AGE_S` = 0.5 秒，过旧才重读，避免每次伺服都查询。`Arm.from_config(config)` 结合 `ArmSafety.from_config(config)` 构建。第三方后端跳过 `_remember_state` 仍能运行，但每次受限伺服会额外 `read_state()`。

<a id="safety-rules"></a>

### 安全规则

- **`disconnect()` 不关闭电源。** 释放只关闭 SDK 会话。Piper 断电可能下坠，因此后端从不调用 `disable()` / `reset()` / `DisableArm` / `ResetPiper`。`stop()` 使运动停止，保留电源和制动状态。
- **未就绪时拒绝伺服。** 所有后端仅在匹配伺服模式接受命令；franky、pyagxarm、piper_sdk 还会在故障中拒绝，需先 `recover()`。Piper 对陈旧反馈也拒绝：pyagxarm 0.5 秒、piper_sdk 1.0 秒无新关节帧。硬件 `move_to_*` 要求 `IDLE`。动作检查形状与有限值，`tcp_pose` 检查单位四元数。
- **连接会使能电机。** 直接连接、doctor 或客户端经守护进程连接 Piper 都会使能，`disconnect()` 后仍保持。运行 doctor 前清空工作区。
- **安全限制。** 机器人主机 YAML 的 `safety_box`、`joint_limits`、`max_delta_per_step` 为依据。`Arm` 在公开方法执行，因此本地 `Robot`、守护进程的 `arm.servo_*`、`arm.move_to_*`、`robot.send_action`、`robot.step` 和 `send_action` 全路径适用。`RemoteArm` 转发到守护进程执行。
  - `servo_*` 先按实测状态的 `max_delta_per_step` 缩短，再裁剪进基坐标系安全盒或关节范围。已接受目标发生任意变化计一次 `safety_clips`。配置 `workspace` 后，笛卡尔目标还必须在基坐标系安全盒裁剪前后均满足工作区的有向位置盒与可选 SO(3) 姿态半径约束。工作区拒绝目标时抛 `SafetyViolation`，递增 `safety_rejects`，不发送命令。
  - `move_to_*` 越界抛 `SafetyViolation`，计一次 `safety_rejects`，不发送命令；不执行单步限幅。
  - `SafetyViolation` 在触及后端前抛出，不锁存故障、无需 `recover()`、保持控制模式，经守护进程时不刷新 watchdog，客户端收到相同异常类。
  - `arm.health()` 报告 `safety_clips` / `safety_rejects`，`robot.describe` 以 `arm.safety` 暴露限制。构建时验证 `home_joints` 位于 `joint_limits` 内，安全盒满足 min <= max。
  - `safety_box` 和笛卡尔单步上限只限制位置。工作区还可通过 `orientation_xyzw`（工作区坐标系内）与 `max_angle_rad`（SO(3) 测地半径，避开欧拉角分支切割）约束姿态。设备侧没有姿态单步上限。关节命令不检查笛卡尔工作区边界：这些限制无法证明关节轨迹、整臂净空或运动无碰撞。
  - 目标检查使用 `1e-9` 数值容差。反馈检查使用工作区键 `measured_margin_m`（默认 `0.003 m`）与 `measured_margin_rad`（默认 `0.03 rad`），两者均须为有限非负值。环境和遥操作跟踪器使用这些反馈余量，但绝不放宽目标准入。没有工作区时，`task_frame`（任务坐标系，task frame）只作为显示/采样参考，不增加空间约定。配置工作区时，重置前会将两者的几何定义与守护进程核对。
- **Watchdog。** 客户端静默后，守护进程让伺服机械臂进入空闲，见第 4 步。
- **`DeviceClaim`。** 基于 `~/.cache/robot_infra/claims/` 的节点本地建议性文件锁，键为 `<type>:<robot_ip | can_interface | name>`。客户端连接期间由守护进程持有，同机器人第二进程收到 `DeviceBusy`。它不防护其他主机或不取得锁的代码，可用 `claim: false` 有意退出。SpaceMouse 读取器默认也会占用其 HID 端点，作为机器人遥操作部件构建时遵循同一个 `RobotConfig.claim` 设置。其内置设备表与 PySpaceMouse 1.0.3 相同；站点的 local 设备表可新增或覆盖型号。

<a id="error-model"></a>

### 错误模型

设备错误继承 `DeviceError`，它是 `RuntimeError`，携带 `message`、`hint`、`device`。`str(error)` 在第二行打印提示。错误可经 `to_dict()` / `from_dict()` 往返，因此客户端重新抛出与服务端相同类型。

| `kind` | 异常类 | 典型原因 |
|---|---|---|
| `sdk_missing` | `SdkMissing` | 厂商软件包未安装。 |
| `device_not_found` | `DeviceNotFound` | 机器人不可达、CAN 接口或相机序列号缺失。 |
| `device_not_connected` | `DeviceNotConnected` | 在 `connect()` 前调用。 |
| `device_already_connected` | `DeviceAlreadyConnected` | 已定义，当前后端不抛出，因为 `connect()` 幂等。 |
| `device_busy` | `DeviceBusy` | 其他进程持有锁。 |
| `device_fault` | `DeviceFault`（`recoverable: bool`） | Reflex、急停、无反馈、模式错误或 watchdog。 |
| `version_mismatch` | `VersionMismatch` | libfranka 与机器人服务器版本不同；错误的 `hid` 发行包缺少 trezor 的 `hid.device()` API。 |
| `safety_violation` | `SafetyViolation` | 笛卡尔伺服/移动目标超出 `workspace`（位置或姿态），或 `move_to_*` 目标超出 `safety_box` / `joint_limits`；发送前拒绝，计入 `safety_rejects`。 |
| `transport` | `TransportError`，继承 `DeviceFault` | 守护进程不可达、超时或令牌不符。 |

<a id="testing"></a>

## 测试

```bash
pip install -e '.[dev,grpc]'
python -m pytest tests -q
```

测试使用 fake 后端和 SDK 替身，无需硬件。缺 `grpcio` 时跳过 gRPC 测试。原文记录的上次运行是远程容器 Python 3.11，结果 `486 passed`；维护者在远程容器运行：

```bash
docker exec <container> bash -lc 'cd <checkout>/robot_infra && python -m pytest tests -q'
```

<a id="known-limitations-and-needs-hardware-verification"></a>

## 已知限制与硬件待验证项

以下尚未在真实硬件验证，fake 仅覆盖逻辑。

- **SpaceMouse USB 厂商 ID。** `discover()` 筛选 3Dconnexion `0x256f` 与旧 Logitech `0x046d`，均待验证；没有 `hidapi` 时列出全部 `/dev/hidraw*`。
- **udev 与 Docker。** `os:hid` 输出的规则和 `docker run --device=/dev/hidrawN` 提示未在机器人 PC 验证，容器需在守护进程启动前传入节点。
- **SpaceMouse wheel 与时序。** 自带的 HID 读取器使用 `set_nonblocking(1)`，每隔 `poll_interval_s`（默认 0.005 秒，200 Hz）轮询一份报告。队列延迟、分拆报告时序、设备方向和断连行为尚未在真机上验证。`hidapi==0.14.0.post4` wheel 的 ELF/系统库依赖仍未在干净的 Ubuntu 22.04 / CPython 3.10 / x86_64 机器上验证。
- **Piper 主臂遥操作**尚未实现；主臂后端为 Dynamixel `gello`。`joint_delta` 仅由 fake 覆盖。
- **GELLO 真机。** `gello` 后端和标定仅使用 fake `dynamixel_sdk`、`evdev`。待验证包括 `discover()` 匹配的 U2D2/OpenRB-150 by-id 名称、不同波特率同步读取时序、FTDI 默认 16 毫秒延迟及常用 1 毫秒优化，后端不会设置它，以及跨整圈边界的位置连续性。
- **姿态单步大小。** 工作区可限制相对参考姿态的旋转；`safety_box` 和 `max_delta_per_step` 仍只限制位置（`xyz_m`）和关节（`joint_rad`）。设备没有姿态单步上限。gym 环境通过 `rot_scale_rad` 限制自身旋转增量，并将策略目标投影到已配置的工作区内。
- **单步基线。** 限幅相对最多 0.5 秒前的最近实测状态，因此目标与真实位置最坏距离约两倍上限，但不会持续累积。
- **绕过检查。** 旧 ROS1 桥接（`serl_http`）是 HTTP 服务，直接访问其 `bridge_url` 的客户端不经过 `Arm`，限制不生效。第三方后端必须实现 `_servo_*` / `_move_*` 钩子，保持公开方法不被覆盖。
- **已配置遥操作必须存在。** 设备缺席时 `connect()` 失败并回滚机器人，部署也一样；部署应去掉 `teleop:`。
- **版本差异。** 新服务的 `arm.health()` 增加字段会使旧客户端失败，旧客户端也不能读 JPEG，先升级客户端。
- **`--jpeg-quality` 未记录**于训练配置，部署无法检查与训练值一致。
- **遥操作读取上限与时钟。** 守护进程 `teleop.<name>.read` 最多等待 2 秒；`timestamp` 来自机器人 PC 的单调时钟。`RemoteTeleop.get_observation()` 返回客户端最后样本，守护进程 `robot.get_observation()` 的 `teleop` 则来自服务端最新样本，不消费事件。

<a id="legacy"></a>

## 旧模块

以下模块早于设备层，为已有用户保留。后续计划见 [`docs/robot-infra-roadmap.md`](../devices/robot-infra-roadmap.md)。

- **`controller/`**：纯 torch 笛卡尔阻抗与姿态控制器位于 `core/`、`adapters/`，经 `controller/simulator/maniskill/` 封装用于 GPU 批量模拟，需要 `sim`。详见 [`real_world_rl/legacy/controller/README.md`](../../real_world_rl/legacy/controller/README.md)。
- **`controller/real/franka_bridge.py`** 与 **`gripper_server.py`**：转发到 ROS1 `serl_franka_controllers` 阻抗控制器和 Franka Hand 的 Flask/HTTP 服务，`serl_http` 是其客户端。ROS 机器运行 `python -m robot_infra.controller.real.franka_bridge --robot_ip <ip> --port 5000`，需要 `real`，ROS1 另装，细节见模块说明。
- **`teleop/`**：旧设备层之前的遥操作，包括使用旧 `pyspacemouse` `read_all()` API 和 `real` 组的 `teleop/spacemouse/`、经 ZeroMQ 接收 Pico 速度的 `teleop/utils/` 与 `teleop/examples/`。[第 7 步](#7-add-a-teleop-device)的 `Teleop` 部件已替代旧 SpaceMouse。Pico 路线尚未迁移，`real-world-rl` 的 `--teleop-device pico` 仍导入这些模块，因此继续保留。
