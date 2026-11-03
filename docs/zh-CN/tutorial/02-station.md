<a id="2-configure-the-station"></a>

# 2. 配置工作站

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/02-station.md)

`python -m robot_infra.setup` 是交互式配置向导。在机器人 PC 上连接并接通机械臂、相机和 GELLO 后运行。它会探测硬件、询问配置、现场标定 GELLO、生成带注释的 `station.yaml`、运行 doctor 检查，并打印后续命令。问题定义在 `robot_infra/setup/wizard.py`，每个问题都有对应参数，参见 `python -m robot_infra.setup --help`。

```bash
python -m robot_infra.setup
```

<a id="what-it-asks"></a>

## 向导会询问什么

下方记录来自 `--mock` 运行，使用预设硬件，可在任意机器上尝试。答案写在各冒号后，留空接受方括号内的默认值。真实运行会打印实际探测结果：响应 TCP 22、443 或 1337 的 Franka 主机、RealSense 序列号，以及每个 U2D2 或 OpenRB-150 串口上响应广播 ping 的舵机 ID。

```text
$ python -m robot_infra.setup --mock --output /tmp/station.mock.yaml
hardware probe (--mock: canned results):
franka hosts:
  172.16.0.2: open ports 22, 443, 1337
  172.16.1.2: open ports 22, 443, 1337
realsense cameras:
  123422070001 (D400)
  123422070002 (D400)
  123422070003 (D400)
gello / dynamixel buses:
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, servo ids [1, 2, 3, 4, 5, 6, 7, 8]
spacemouse devices:
  none
number of arms (1 or 2) [1]:
arm name(s) [arm]:
-- arm 'arm' (1/1) --
franka hosts that answered:
  1) 172.16.0.2
  2) 172.16.1.2
robot IP of 'arm' (number or IP) [172.16.0.2]:
arm backend (franky | serl_http | fake) [franky]:
gripper backend (franky | serl_http | fake | none) [franky]:
home joints of 'arm' (7 values in rad, or none) [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]:
realsense cameras:
  1) 123422070001 (D400)
  2) 123422070002 (D400)
  3) 123422070003 (D400)
cameras of 'arm': numbers or serials, each optionally =name (e.g. 1=wrist,2=front), or none [123422070001=wrist,123422070002=front,123422070003=front_2]: 1=wrist
teleop device of 'arm' (gello | spacemouse | fake | none) [gello]:
gello / dynamixel buses:
  1) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
  2) /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK1-if00-port0: 57600 baud, ids [1, 2, 3, 4, 5, 6, 7, 8]
GELLO serial port of 'arm' (number or path) [/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0]:
GELLO baud rate [57600]:
GELLO joint servo ids (7, in the Panda's joint order) [1,2,3,4,5,6,7]:
GELLO gripper trigger servo id (or none) [8]:
intervention (always | toggle: toggle needs a pedal or keyboard) [always]:
event device for toggle/episode keys (/dev/input/by-id/... or none) [none]:
calibrate the GELLO on /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_MOCK0-if00-port0 now (yes | no) [yes]:
  ... the calibration prompts (below) ...
daemon port of 'arm' [50051]:
wrote /tmp/station.mock.yaml
checking it with the doctor (--mock):
  ... one doctor line per check, then "summary: ..." ...
next steps:
  ...
```

回答时注意：

- **机器人 IP、相机、GELLO 串口**可以填写列表序号或具体值。未被探测到的值会被拒绝，除非传入 `--allow-unprobed`。
- **相机**填写 `SERIAL[=NAME]` 或列表序号，以逗号分隔。`NAME` 是后续 `obs.rgb` 使用的观测名称，对应 `rgb_<name>`；本教程使用一台名为 `wrist` 的相机。
- **人工介入**保持 `always`：actor 机器的键盘通过 `--teleop-event-device keyboard` 提供切换信号，并覆盖该设置。`toggle` 适用于机器人 PC 上的 evdev 脚踏或键盘，见 [gello.md](../gello.md#6-hil-training)。
- **没有指定 `--output` 或 `--scope` 时**，最后会询问保存位置：`write the station to (local = ./local/station.yaml, user = ~/.config/robot_infra/station.yaml, or a path) [local]:`。已有文件会先备份为 `station.yaml.bak.<UTC timestamp>`。
- **GELLO 夹爪**：新配置的夹爪通道采用 `gripper_binarize_threshold: 0.5`，保存的夹爪动作恰为 `+1`（打开）或 `-1`（闭合），为 [07](07-hil-training.md) 的离散夹爪头提供清晰标签。

<a id="the-inline-gello-calibration"></a>

## 向导内的 GELLO 标定

在标定问题中回答 `yes`，会执行与 `python -m robot_infra.devices.backends.teleops.gello_calibrate` 相同的流程。舵机力矩关闭，主臂由你手动移动。顺序如下：

1. 对七个关节逐一操作：沿机器人的正方向移动该关节并按 Enter。程序打印 `joint N: observed sign +1 (current +1); use it? [Y/n]`。按 Enter 接受，回答 `n` 保留当前方向。移动不足 0.1 rad 时保留原方向；`--mock` 模式没有运动，因此会显示 `moved less than 0.1 rad; keeping sign +1`。
2. 将主臂保持在标定姿态并按 Enter。提示中显示 `[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]`，即第 4 关节为 -90 度、第 6 关节为 +90 度，其余为 0。随后打印 `offset residuals (rad): [...]`。残差大于 pi/8 时，会提示主臂姿态或关节方向可能有误，需要重新标定。
3. 将扳机保持在完全打开状态并按 Enter，再保持完全闭合并按 Enter。扳机移动不足 0.05 rad 时标定失败。

回答 `no` 时，YAML 使用零偏移、全部 `+1` 方向和 `# TODO calibrate` 注释；之后可以运行下方命令标定。标定失败时会询问 `retry the calibration? [Y/n]`。

```bash
python -m robot_infra.setup --from local/station.yaml --only teleop --calibrate
```

<a id="where-the-file-lands"></a>

## 文件保存位置

向导默认使用 `--scope local`，保存到相对当前运行目录的 `./local/station.yaml`；`local/` 已被 Git 忽略。`--scope user` 保存到 `~/.config/robot_infra/station.yaml`，`--output` 可指定其他位置。不传 `--config` 时，守护进程、doctor 和 discovery 依次采用以下第一个可用位置：

1. `$ROBOT_INFRA_CONFIG`
2. `./local/station.yaml`
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml`，默认是 `~/.config/robot_infra/station.yaml`。

显式指定的 `--config <yaml>` 优先级最高。后续 `robot_infra` 命令应从同一目录运行，或设置 `ROBOT_INFRA_CONFIG`。

<a id="check-the-safety-limits"></a>

## 检查安全限制

向导写入 Panda 关节限制与单步伺服上限，即 `joint_limits` 和 `max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`，并询问 `safety_box`，建议一个桌面工作区。首次运动前打开 `station.yaml`，对照实际工作单元检查这三项。工作区位于机器人基坐标系（`tcp_pose = O_T_EE`，单位米），必须包含归位姿态、物体和箱子；`home_joints` 必须处于 `joint_limits` 内。守护进程会裁剪伺服目标并拒绝越界的 `move_to_*` 目标，见[安全规则](../robot-infra.md#safety-rules)。训练环境的限制只能更严格。使用 `--from` 重跑向导时会保留你编辑过的值。

<a id="regenerating-and-rehearsing"></a>

## 重新生成与演练

```bash
python -m robot_infra.setup --from local/station.yaml                    # old values are the defaults
python -m robot_infra.setup --from local/station.yaml --only cameras     # redo one section: cameras | teleop | daemon
python -m robot_infra.setup --mock --output /tmp/station.mock.yaml       # rehearse without hardware
```

演练时用 `--output` 指定其他文件，避免替换真实的 `local/station.yaml`。脚本化使用方式见 [robot_infra README](../robot-infra.md#setup-wizard) 中的 `--non-interactive`。

<a id="the-env-preset-hilyaml"></a>

## 环境预设（`hil.yaml`）

训练、录制和数据采集命令都通过 `--config` 从同一份 YAML 预设读取工作站相机名称、守护进程地址和动作约定。标准单臂预设是 [`examples/presets/hil_single_arm.yaml`](../../../examples/presets/hil_single_arm.yaml)。将其复制到仓库根目录，修改副本：

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil.yaml
```

- `real_robot.remote`：填写 `ROBOT_PC:50051`，端口来自 `station.yaml`；actor 在机器人 PC 上运行时使用 `127.0.0.1:50051`。
- `obs.rgb`：填写 `station.yaml` 中的相机名称，此处为 `[wrist]`。
- `real_robot.token`：仅在工作站文件设置了 `daemon.token` 时取消注释。

在不操作机器人的情况下检查预设：

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

*预期结果*：以 JSON 输出解析后的配置，其中包含 `env_backend: real_robot` 和 `real_robot.action_mode: joint_delta`。

> **joint_abs。** 另存一份 `hil_abs.yaml`，设置 `action_mode: joint_abs` 并取消 `joint_limits` 行的注释。该字段是成对值的列表，命令行无法表达。教程会在 `joint_abs` 有差异的地方使用这份副本。`joint_abs` 还要求 `station.yaml` 包含 `max_delta_per_step.joint_rad`，向导会自动写入，因为相邻 `joint_abs` 动作可能跨越整个关节范围。

<a id="task-images-and-control-profiles"></a>

## 任务 图像与控制参数配置

教程默认 `real_robot.task.name` 为 `none`，由操作员或分类器提供成功标签。任务、reset、深度和逐相机预处理选项见[真机实验](../real-world-experiments.md#task-presets-and-observation-settings)。在录制前确定配置，并在 BC、HIL 和部署中保持一致。`preprocess.<camera>.roi` 以像素表示 `[top, left, height, width]`，同时必须填写真实的 `source_resolution: [height, width]`。RGB 采用双线性缩放；深度为单通道米制，采用最近邻缩放，无效像素置零。改变这些设置会改变策略的观测约定。

工作站 YAML 可为每臂提供 `control:` 阻抗配置，实验侧的 `real_robot.control` 可覆盖它。仅在 IDLE 时配置，覆盖值只能收紧已声明的力矩变化率和误差裁剪边界，已安装 SDK 还须支持相应关键字。[机械臂控制参数](../control-profiles.md)说明字段、实际生效值和向导的 `--control` 选项。向导保留已有配置，新工作站写为 `control: null`；增益需要按实际工具和负载选择。

以 `real_robot.control_hz` 为机器人控制频率。非默认全局 `--control-hz` 可以填入仍为默认值的环境频率；两者均为非默认且不一致时会被拒绝。录制、训练和部署使用解析后的同一频率，该值属于实验约定。

下一步：[03 守护进程](03-daemon.md)。
