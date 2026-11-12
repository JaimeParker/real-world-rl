<!-- English source: docs/doctor.md -->

<a id="rwrl-doctor"></a>

# rwrl doctor

[中文入口](../README.zh-CN.md) · [English](../doctor.md)

`rwrl doctor` 是机器人主机的统一诊断工具，检查主机、项目环境和各站点机器人的分层设备检查，以统一状态和退出码输出报告。`rwrl setup` 及 `rwrl daemon` 预检采用相同设备报告格式。教程 [01](tutorial/01-install.md) 用于无硬件检查，[03](tutorial/03-franka-bringup.md) 用于真实握手；本页为参考。

<a id="running-it"></a>

## 运行

在机器人主机激活项目环境后运行（源码检出或已安装包均可，无需位于仓库根目录）。站点可选：

```bash
rwrl doctor
rwrl doctor --station local/station.yaml
rwrl doctor --station local/station.yaml --robot left
rwrl doctor --station local/station.yaml --mock
rwrl --station local/station.yaml doctor --fix
```

| 选项 | 含义 |
|---|---|
| `--station PATH` | 检查的站点；默认按[站点查找](cli.md#station-lookup)。无站点时只检查主机 |
| `--robot NAME` | 检查多机器人站点中的一个机器人 |
| `--preset PATH` | 同时读取环境预设：将其遥操作设备列为预期设备，比较其机器人与站点（[下文](#comparing-a-preset-with-the-station)） |
| `--mock` | 所有设备用模拟后端；跳过主机和 OS 层 |
| `--fix` | 预览、一次确认、执行主机及软件修复、复查 |
| `--install-sdk` | 仅修复经握手确认的 franky 不匹配 |
| `--report-dir PATH` | 报告基目录（覆盖 `$RWRL_DOCTOR_REPORT_DIR`） |
| `--json` | stdout 输出一个 JSON 对象；文本去 stderr |
| `--yes` | 跳过修复计划确认；前置条件仍适用 |

`--station`、`--robot`、`--preset`、`--mock`、`--json`、`--yes` 可在 `doctor` 前后使用；后面的值优先。`rwrl doctor --help` 列出选项。

doctor 连接每个配置设备后断开，不恢复、回 home 或移动机器人，不停止或接管其他进程，不更改网络地址、固件、内核、启动设置。初始化设备仍可能有副作用，例如关闭 GELLO 扭矩。

<a id="reading-the-output"></a>

## 阅读输出

```text
PASS           host.python: /venv/bin/python
SKIPPED        coverage.gello: GELLO not configured
    hint: run rwrl setup
WARN           arm.preset:robot_ip: station="a"; preset="b"
    hint: regenerate the preset
FAIL           arm.device: Traceback (most recent call last):
  ...
ConnectionError: robot unreachable
    hint: check the Ethernet cable
          and the robot IP in Desk
BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
PENDING_ACTION groups.dialout: log out and back in
Report: local/doctor/20261112T101500-3f2a9c1d
Needs attention: 1 FAIL, 1 BLOCKED, 1 PENDING_ACTION
  FAIL           arm.device: ConnectionError: robot unreachable
    hint: check the Ethernet cable
  BLOCKED        arm.os:claim: device inspection requires an idle station
    hint: finish the controlling program
  PENDING_ACTION groups.dialout: log out and back in
Result: unresolved failures / blocked checks / pending user actions
```

- **检查行：** `STATUS id: message`，状态补齐到 14 字符。非 `PASS` 行后附 `    hint: ...`；提示续行与正文对齐。
- **计划行**（可修复时）：`PLAN <id>: <title>`，随后是命令或文件及内容；此处无法执行时为 `  BLOCKED: <reason>`，需你继续操作时为 `  Afterward: <step>`。`--fix` 仅询问一次 `Apply this plan? [y/N]`。
- **`Report:`** 本次报告保存目录（[下文](#report-directory)）。
- **`Needs attention:`** 仅存在未解决问题时显示：计数，以及各 `FAIL`、`BLOCKED`、`PENDING_ACTION` 检查和修复的消息末行及提示首行。
- **`Result:`** `no blocking findings; review SKIPPED/WARN coverage` 或 `unresolved failures / blocked checks / pending user actions`。

确认修复后，doctor 重新打印各修复及检查的新状态。

<a id="statuses-and-colours"></a>

### 状态与颜色

| 状态 | 含义 | 颜色 | 退出码 |
|---|---|---|---|
| `PASS` | 已检查且正常 | 绿色 | 0 |
| `WARN` | 正常，但需按提示操作（预设字段过时、延迟计时器偏高） | 黄色 | 0 |
| `SKIPPED` | 未检查：未配置、`--mock` 或前层失败；不能证明硬件可用 | 灰色 | 0 |
| `FAIL` | 已检查且异常 | 红色 | 1 |
| `BLOCKED` | 无法检查或修复：设备占用、缺少能力/权限/依赖、计划未确认 | 粗体品红色 | 1 |
| `PENDING_ACTION` | 已修复，需继续操作（重新登录、插拔）才能验证 | 黄色 | 1 |

`Report:`、`Needs attention:` 为粗体，`PLAN` 为青色，`Result:` 为粗体绿色或红色。仅终端流使用颜色；设置 `NO_COLOR`（任意值）、`TERM=dumb`、Windows 控制台及管道输出均为纯文本。保存报告不含颜色码。

<a id="what-it-checks"></a>

## 检查内容

<a id="host-rows"></a>

### 主机行

| 标识 | 检查 |
|---|---|
| `host.platform`, `host.python`, `host.tools` | 平台；解释器是否为项目 venv 或非 base conda 环境；诊断工具 |
| `deps.<distribution>`, `deps.<distribution>.source`, `sdk.<extra>` | 项目及预期设备 Python 依赖与版本约束；SDK 导入 |
| `system.library.<name>` | libusb、libudev 等共享库 |
| `usb.discovery`, `usb.<kind>`, `usb.spacemouse.table`, `usb.spacemouse.unknown.<vid>:<pid>` | 可见 USB、SpaceMouse/RealSense 访问及 udev 规则、SpaceMouse 设备表 |
| `gello.<n>`, `gello.<n>.group`, `gello.<n>.events`, `gello.<n>.latency`, `gello.<n>.brltty`, `gello.candidates` | 已调试 GELLO 串口/事件访问、用户组、FTDI 延迟、brltty 冲突；未配置串口适配器仅为候选 |
| `network.route.<ip>` | 到各机器人地址的路由 |
| `realtime.kernel`, `realtime.limits` | PREEMPT_RT 内核和实时限制（可能需要内核操作时显示 `MAJOR CHANGE / DOUBLE CHECK`；不执行修改） |
| `configuration.*` | 站点加载；旧于 0.1.0 的站点名称产生 `configuration.legacy.*` WARN 并附迁移命令 |
| `station` | 未配置站点时 `SKIPPED` |
| `read.<kind>`, `coverage.<kind>` | 未配置 SpaceMouse/RealSense 的 SDK 读取；未配置也未识别的种类为 `SKIPPED` |
| `sdk.franky.conflict` | 需要不同 franky wheel 的机器人 |

<a id="device-rows"></a>

### 设备行

各站点机器人得到六层 `<robot>.<check>` 行（如 `arm.handshake`）。`teleop:` 部分在第 1、2、4 层添加行。

| 层 | 报告行 | 说明 |
|---|---|---|
| 0 deps | `deps` | 各后端所需扩展，含 `daemon:` 时另需 `grpc`；无此块时缺 `grpc` 仅为 `WARN` |
| 1 sdk | `sdk:arm`, `sdk:gripper`, `sdk:camera:<name>`, `sdk:teleop:<backend>` | 后端解析及 SDK 导入；未知后端跳过第 3–5 层 |
| 2 os | `os:ping`, `os:tcp`, `os:camera:<name>`, `os:teleop:<name>`, `os:hid`, `os:serial`, `os:groups`, `os:brltty`, `os:latency_timer`, `os:can_*` | 网络、设备节点、权限；`os:tcp` 仅供参考 |
| 3 handshake | `handshake` | 获取设备占用声明（除非 `claim: false`），再 `connect()`；失败跳过第 4–5 层 |
| 4 feedback | `feedback:arm`, `feedback:gripper`, `feedback:camera:<name>`, `feedback:teleop:<name>` | 各设备返回数据；相机须在 2 s 内出帧 |
| 5 mode | `mode:robot_mode`, `mode:libfranka_version`, `mode:realtime_kernel`, or `mode:none` | 后端预检 |

设备探测本身失败时 `<robot>.device` 为 `FAIL`；其他进程占用站点时为 `BLOCKED`。下方[详细示例](#device-diagnostic-details)展示设备消息。

<a id="expected-devices"></a>

### 预期设备

站点和预设决定预期设备。已配置设备缺失为 `BLOCKED`；未配置且未识别为 `SKIPPED`，不算可用。识别的 SpaceMouse 和 RealSense USB 也独立检查。

只有持有硬件连接的进程算占用者：守护进程、录制器、actor、deploy 入口、`setup`、连接原语（`status`、`q`、`tcp`、`home`、`gripper`、`camera`、`stop`、`recover`）、`teleop run/read/calibrate/diagnose/probe-hid`、`workspace capture` 和学习命令。离线命令（`--help`、`workspace show/set/validate/frame`、`teleop profiles`、`teleop inspect-stop`、`archive inspect`）不会令 doctor 报告 `BLOCKED`。

<a id="--mock"></a>

## `--mock`

`--mock` 将每个后端替换为模拟后端（丢弃遥操作私有选项），跳过第 2 层及主机 OS、USB、依赖检查。无硬件检查站点 YAML 和安装，不验证设备。

<a id="comparing-a-preset-with-the-station"></a>

## 比较预设与站点

提供站点和 `--preset` 时，doctor 将预设机器人（内嵌 `real_robot.robot`、裸 `robot` 映射或指定站点文件）与站点机器人比较：`robot_ip`、`arm_backend`、`gripper`、按名称匹配的相机（`serial`、`width`、`height`、`fps`、`enable_depth`）、`home_joints`、`joint_limits`、`safety_box`、`task_frame`、`workspace`。一致时输出一行 `PASS` `<robot>.preset`（`preset hardware fields match the station`）。各差异字段产生 `WARN` `<robot>.preset:<field>`，如 `arm.preset:robot_ip: station="172.16.0.2"; preset="172.16.0.3"` 或 `arm.preset:cameras.wrist.fps: station=30; preset=15`，提示 `the preset copy of the station is stale; regenerate it or edit the field`。WARN 不改退出码，须阅读。多机器人站点内嵌机器人需 `--robot`，否则比较为 `SKIPPED`。

预设遥操作为 GELLO 时也指定主臂选项文件 (`teleop_options_yaml`)；该文件必须存在，否则 doctor 以用法错误停止。

<a id="repairs---fix"></a>

## 修复：`--fix`

```bash
rwrl --station local/station.yaml doctor --fix
```

doctor 展示计划（命令、专用配置文件内容、剩余手动步骤），询问一次，执行批准计划并复查。仅 `y`（不区分大小写）确认。独立修复在失败后继续；依赖修复为 `BLOCKED`。复查发现新问题需另行审查计划。无终端且无 `--yes` 时不修复，计划为 `BLOCKED`（`plan not confirmed; no repairs executed`）。

修复可安装缺失主机工具、添加用户组、写专用 `/etc/udev/rules.d/99-rwrl-*.rules` 及当前用户实时限制、重载 udev，并用当前解释器修复项目环境 Python 包（绝不 `sudo pip`）。GELLO 串口或 SpaceMouse 节点不可读写时，也提供 `setfacl -m u:<user>:rw -- <node>`：立即生效，重插失效。备份旧内容，拒绝并发编辑。重新插拔和登录在 doctor 验证前保持 `PENDING_ACTION`。缺失兄弟项目发行包从已核实本地源码修复，否则 `BLOCKED`，提示从可信包来源安装。

| 主机 | 支持能力 |
|---|---|
| 原生 Linux | 主机/USB 诊断，通过已安装后端探测设备 |
| 有 `apt-get` 的 Debian/Ubuntu 系 | 缺失工具、libusb/libudev 包 |
| 有 udev、用户组工具、PAM 限制或运行中 systemd 的原生 Linux | 对应规则、组、限制或已证实服务冲突修复 |
| 容器、WSL、macOS、Windows | Python 元数据/导入诊断及 Python 修复；原生探测和系统修复为 `BLOCKED` |

非 root 时系统修复逐条使用 sudo；缺少 sudo 授权为 `BLOCKED`，不从管道读取密码。无人值守前先在终端授权 sudo：

```bash
sudo -v
rwrl --station local/station.yaml --yes --json doctor --fix
```

<a id="--install-sdk"></a>

## `--install-sdk`

`franky` 扩展安装默认 `franky-control` wheel，绑定一个 libfranka 版本。机器人服务器版本不同时拒绝握手，doctor 报告 `<robot>.handshake` FAIL `libfranka mismatch: robot server <N>, installed library <M>`。然后：

```bash
rwrl doctor --station local/station.yaml --install-sdk
```

doctor 规划一项修复 `PLAN sdk:franky: Install Franky wheel for handshake-confirmed server version <N>`，附准确 `pip install franky-control --extra-index-url ...` 命令，询问一次，仅在 `y` 后安装到当前项目解释器（`--yes` 跳过询问）。服务器版本来自 libfranka 握手错误；用 Desk → Settings → System 系统版本和[兼容矩阵](https://frankarobotics.github.io/docs/compatibility.html)交叉核查。不要用 FCI 功能版本 (`FCI 1.0.0`)。不带 `--fix` 的 `--install-sdk` 不执行其他修复。需要不同服务器版本的机器人被标为 `sdk.franky.conflict` `BLOCKED`，应使用独立项目环境。之后重跑普通 doctor。

<a id="report-directory"></a>

## 报告目录

每次保存已脱敏凭据的诊断：`before.json`、`plan.json`、`report.json`、`commands.log`（所执行命令的原始输出）；配置错误另存 `error.json`，必要时含备份或暂存文件。doctor 不保存自己的控制台文本。基目录依次选择：

1. `--report-dir PATH`；
2. `$RWRL_DOCTOR_REPORT_DIR`；
3. 源码检出中可写且被 git 忽略的 `local/doctor/`；
4. 用户状态目录：Linux 为 `${XDG_STATE_HOME:-~/.local/state}/rwrl/doctor/`，macOS 为 `~/Library/Application Support/rwrl/doctor/`，Windows 为 `%LOCALAPPDATA%/rwrl/doctor/`。

每次创建 `<UTC timestamp>-<id>/` 子目录；POSIX 目录模式 0700、文件 0600。显式指定的不可写目录会报错；检出不可写则回退用户状态目录。备份保留原始字节，整个目录应视为私有。

<a id="json-output"></a>

## JSON 输出

`--json` 在 stdout 输出一个对象：`schema_version: 1`、`status`（`PASS` 或 `FAIL`）、`checks`、`repairs`、`remaining_plan`、`report_directory`、逐机器人低层 `results`（各为 `{layer, name, ok, message, hint, skipped, warning}` 列表）。检查行、计划、提示去 stderr。参数、配置、报告创建错误返回退出码 2，JSON 错误含 `schema_version`、`error`、`report_directory`。

<a id="exit-codes"></a>

## 退出码

| 结果 | 退出码 |
|---|---|
| 仅 `PASS`、`WARN` 或 `SKIPPED` | 0；判定硬件通过前仍须阅读 `SKIPPED`、`WARN` |
| 任意 `FAIL`、`BLOCKED` 或 `PENDING_ACTION` | 1 |
| 参数或配置无效 | 2 |

<a id="red-lights-what-to-do"></a>

## 红灯：如何处理

从首个红色行开始：握手失败跳过所有反馈和模式行，缺扩展也会使其 SDK 行失败。表格引用原始消息和提示；`<ip>` 等值来自站点。Piper 和 CAN 行见[设备层故障表](doctor.md#troubleshooting)。

<a id="network-and-fci"></a>

### 网络与 FCI

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.os:ping` FAIL `<ip> does not reply` | 未通电、网线接 Arm 端口、地址或子网错误 | 电源、**Control** 端口、NUC 地址（[02](tutorial/02-setup.md#before-you-start)） |
| `<robot>.handshake` FAIL `cannot reach Franka at <ip>: ...`, hint `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated ...` | FCI 未激活、其他机器持控制权、防火墙丢 UDP | 按提示顺序检查；刷新 Desk、**Request control**、激活 FCI |
| `network.route.<ip>` FAIL | 无机器人子网路由 | 为 Control 接口添加地址（[02](tutorial/02-setup.md#before-you-start)） |

<a id="sdk-and-libfranka-version"></a>

### SDK 与 libfranka 版本

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.handshake` FAIL `libfranka mismatch: robot server <N>, installed library <M>` | franky wheel 不匹配机器人服务器 | `rwrl doctor --station local/station.yaml --install-sdk`（[上文](#--install-sdk)） |
| `<robot>.sdk:arm` FAIL `FrankyArm needs the 'franky' package` | 缺 `franky` 扩展 | 在仓库根目录执行提示的 `pip install -e '.[franky]'` |
| `sdk.franky.conflict` BLOCKED | 两机器人需不同 franky wheel | 每个服务器版本独立项目环境 |

<a id="python-environment-and-extras"></a>

### Python 环境与扩展

| 报告行 | 原因 | 处理 |
|---|---|---|
| `host.python` WARN | 解释器不是项目 venv 或非 base conda 环境 | 激活项目环境（[01](tutorial/01-install.md)） |
| `<robot>.deps` FAIL `missing extras for this station: franky, gello` | 缺站点后端扩展 | 仓库根目录执行提示的 `pip install -e '.[franky,gello]'` |
| `deps.<distribution>` FAIL | 依赖缺失或违反版本约束 | `rwrl doctor --fix` 在项目环境重装 |
| `system.library.<name>` FAIL | 缺共享库（libusb、libudev） | apt 系统用 `rwrl doctor --fix`；否则用系统包管理器安装 |

<a id="serial-input-groups-and-usb-latency"></a>

### 串口、输入组和 USB 延迟

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.os:serial` FAIL `no read/write access to <port>` | 当前用户无法打开 GELLO 端口 | `rwrl doctor --fix`（组、udev 或立即生效但重插失效的 `setfacl`） |
| `<robot>.os:groups` FAIL `not in group(s) dialout, input; ...` | 登录会话缺 `dialout`（串口）或 `input` (evdev) | `rwrl doctor --fix` 或下方命令，再重新登录 |
| `<robot>.os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; ...` | brltty 占用 CH340 | 冲突确认后 `rwrl doctor --fix` 或下方命令，再重新插拔 |
| `<robot>.os:latency_timer` WARN `<tty> latency_timer is 16 ms; 1 ms is recommended ...` | FTDI 读取慢 | `rwrl doctor --fix` 写 udev 规则，再重新插拔；CH340 无延迟计时器 |

`--fix` 未提供适合修复时，使用等效手动命令。在机器人主机，无需环境。用稳定 `/dev/serial/by-id/` 路径，不假定 `ttyUSB0`：

```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$(id -un)"
sudo usermod -aG input "$(id -un)"
```

退出再登录，重新激活项目环境并重跑 doctor。`input` 仅 evdev 键盘或踏板需要；终端键无需组，`pynput` 监听器需要桌面会话，仅有属于 `input` 的 SSH shell 不够。

CH340 适配器反复出现/消失时先检查日志；确认 brltty 且无盲文设备需要它时，释放适配器直到重启，再重新插拔：

```bash
journalctl -b -u brltty-udev.service --no-pager
sudo systemctl mask --runtime --now brltty-udev.service
```

FTDI `latency_timer` 偏高时，临时写入（路径替换为自己的适配器）：

```bash
GELLO_PORT=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter-if00-port0
GELLO_TTY="$(basename "$(readlink -f "$GELLO_PORT")")"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/$GELLO_TTY/latency_timer"
```

不存在 `latency_timer` 文件表示驱动没有该功能，不是权限错误。

<a id="cameras"></a>

### 相机

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.os:camera:<name>` FAIL `camera '<name>' serial '<s>' not found`, hint `set serial; discovered: [...]` | 序列号错误或未插相机 | 从 `rwrl discover` 复制序列号；多相机时必须设 `serial` |
| `<robot>.feedback:camera:<name>` FAIL `no frames within 2s` | USB 2 端口/线缆、共享集线器、缺 udev 规则 | USB 3 端口及线缆、librealsense udev 规则；重连后 `rwrl status` |
| `<robot>.sdk:camera:<name>` FAIL | 缺 `pyrealsense2` | `python -m pip install -e '.[realsense]'` |

<a id="spacemouse-and-hid"></a>

### SpaceMouse 与 HID

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.sdk:teleop:spacemouse` FAIL `SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)` | 缺 `spacemouse` 扩展 | `pip install -e '.[spacemouse]'` |
| `<robot>.os:hid` FAIL `no HID device found` or `no read/write access to [...]` | 未插设备或无 udev 规则 | [SpaceMouse udev 步骤](teleop.md#spacemouse-installation-and-linux-permissions)或 `rwrl doctor --fix` |
| `usb.spacemouse.unknown.<vid>:<pid>` WARN | 设备表缺条目 | `rwrl teleop probe-hid`（[现场设备表](spacemouse-commissioning.md#site-device-table)） |

<a id="busy-devices-blocked"></a>

### 设备占用 (BLOCKED)

| 报告行 | 原因 | 处理 |
|---|---|---|
| `<robot>.device` BLOCKED `device inspection requires an idle station: ...` | 录制器、actor、守护进程或连接中的 `rwrl` 命令占用机器人 | 结束该程序后重试；doctor 不会停止它 |
| `<robot>.handshake` BLOCKED `... claimed by another process` | 其他进程持设备占用声明 | 同上 |
| `read.<kind>` BLOCKED | 设备占用者阻止 SDK 读取 | 同上 |
| `usb.<kind>` BLOCKED | 已配置 USB 设备缺失 | 插上设备、检查线缆 |

<a id="robot-mode-reflex-userstopped-guiding"></a>

### 机器人模式：Reflex、UserStopped、Guiding

`<robot>.mode:robot_mode` FAIL `robot mode is Reflex`（或 `UserStopped`、`Guiding`），提示 `release the external activation device / clear errors in Desk, then run rwrl recover`。检查工位，释放激活装置或在 Desk 清错，再运行 `rwrl recover` 和 doctor（[03](tutorial/03-franka-bringup.md#5-after-a-reflex-stop)）。`--fix` 不恢复机器人。

<a id="realtime-kernel-and-limits"></a>

### 实时内核与限制

| 报告行 | 原因 | 处理 |
|---|---|---|
| `realtime.kernel` WARN | 非 PREEMPT_RT 内核 | `realtime: ignore` 允许但可能控制抖动；`realtime: enforce` 要求手动安装 RT 内核 |
| `realtime.limits` FAIL or `PENDING_ACTION` | rtprio、memlock 限制缺失或未生效 | `rwrl doctor --fix` 写入，重新登录 |
| `<robot>.mode:realtime_kernel` FAIL `realtime: enforce but the kernel is not PREEMPT_RT` | 站点强制实时 | 安装 RT 内核或设 `realtime: ignore` |

<a id="device-diagnostic-details"></a>

## 设备诊断详情

在机器人主机仓库根目录，激活项目环境：


```bash
rwrl doctor --station robot.yaml
```

`rwrl doctor` 为每个站点机器人运行下述设备检查，与主机、项目检查一并报告为 `<robot>.<check>` 行（如 `arm.handshake`），状态为 `PASS`、`FAIL`、`WARN` 或 `SKIPPED`；报告、选项、退出码见 [cli.md](cli.md#robot-host-diagnosis-and-repair)。`rwrl setup` 和 `rwrl daemon` 预检采用相同设备格式。

无 `--station` 时 doctor 检查[站点文件](tutorial/02-setup.md)。多机器人站点逐个检查；`--robot NAME` 只检查一个。

doctor 按 0–5 顺序运行六层（`teleop:` 在第 0、1、2、4 层增加检查）：

| 层 | 检查 | 说明 |
|---|---|---|
| 0 `deps` | 各后端 pip 扩展（`franky`、`pyagxarm`、`piper-sdk`、`realsense`、`spacemouse`、`gello`，gello `event_device` 另需 `gello-events`），机器人 yaml 含 `daemon:` 时另需 `grpc`。FAIL `missing extras for this station: franky, gello`，提示 `pip install -e '.[franky,gello]' (from the repository root)`；OK `extras ok: franky, gello, grpc`。无 `daemon:`（单进程路径）缺 `grpc` 仅 WARN `optional, not installed: grpc (daemon route)`；缺 `jpeg` 仅提及 | `--mock` 模拟后端无需扩展，永不要求 grpc |
| 1 `sdk` | 各后端名可解析且 SDK 可导入（`sdk:arm`、`sdk:gripper`、`sdk:camera:<name>`、`sdk:teleop:<backend>`） | 未知后端跳过第 3–5 层 |
| 2 `os` | ping `robot_ip` 及 1337 端口（参考）；CAN 存在/启动/比特率；相机序列号连接；`os:teleop:<name>`（发现对应后端设备）；`os:hid`（存在可读写 `/dev/hidraw*`，仅 SpaceMouse）；`os:serial`（gello `port` 存在且可读写；`rwrl doctor --fix` 提供 `setfacl -m u:<user>:rw -- <port>`，重插失效）；`os:groups`（会话属于 gello 端口 `dialout` 及 `event_device` 的 `input`）；`os:brltty`（brltty 会占用 CH340）；`os:latency_timer`（FTDI `latency_timer` 为 1 ms） | 配置字段驱动；无需 SDK 的模拟后端跳过；不运行 sudo |
| 3 `handshake` | 获取设备占用声明（除非 `claim: false`），再 `robot.connect()` | 失败（含声明被占）跳过第 4–5 层 |
| 4 `feedback` | 臂返回状态；夹爪报告宽度；各相机 2 s 内出帧；`feedback:teleop:<name>`：遥操作轮询线程存活且健康 | |
| 5 `mode` | 后端预检：Franka RT 内核、模式、libfranka 版本；Piper 状态、从臂反馈、固件配置 (pyagxarm) 或夹爪行程 (piper_sdk) | |

`feedback:gripper` 打印米制宽度、占 `max_width_m` 比例、抓取状态和后端来源；非有限值、比例越出 [0, 1] 或 `max_width_m <= 0` 产生 `WARN`，提示检查 `max_width_m` 与 SDK 宽度单位。

选项：

- `--mock` 替换各后端为模拟后端并跳过第 2 层，无硬件检查 YAML 和安装。
- `rwrl doctor --fix` 仅修复主机和软件，不恢复或重开设备。Franka Reflex/UserStopped/Guiding (`mode:robot_mode`) 用 `rwrl recover` 清除；无帧相机重连后用 `rwrl status` 复查，设备提示会说明。
- `--json` 打印 `rwrl doctor` 报告（`schema_version: 1`、`status`、`checks`、`repairs`）；各机器人设备检查位于 `results`，按机器人名索引 `{layer, name, ok, message, hint, skipped, warning}` 列表。
- `--install-sdk`（`--yes` 不询问）在 libfranka 版本不匹配后安装匹配机器人 wheel；见[安装正确 franky wheel](doctor.md#installing-the-right-franky-wheel)。
- 退出码：任一机器人检查失败为 `1`，否则 `0`。跳过不导致失败。`WARN` 为带有需处理提示的正常检查（如 brltty 可能在下次重插占用 CH340），不导致失败。

doctor 连接机器人后断开，不关闭任何设备电源。

Franka 示例 `--mock` 的设备行如下（`rwrl doctor` 同时打印主机行；`rwrl setup` 及守护进程预检恰好打印这些行和结果）：

```text
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

相同配置在无 franky、无机器人、无相机的机器上（已缩减 needs-attention 块）：

```text
FAIL           franka.deps: missing extras for this station: franky
    hint: pip install -e '.[franky]' (from the repository root); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
FAIL           franka.sdk:arm: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
FAIL           franka.sdk:gripper: FrankyGripper needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
PASS           franka.sdk:camera:wrist: 'pyrealsense2' importable
FAIL           franka.os:ping: 172.16.0.2 does not reply
    hint: check: robot powered on and brakes released; Desk reachable at https://172.16.0.2/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP
PASS           franka.os:tcp: port 1337 closed (informational)
FAIL           franka.os:camera:wrist: no RealSense device found (serial unset)
    hint: plug in the camera and run `rwrl discover`
FAIL           franka.handshake: FrankyArm needs the 'franky' package
    hint: pip install franky-control (or, from the repository root: pip install -e '.[franky]')
SKIPPED        franka.feedback: skipped: handshake failed
SKIPPED        franka.mode: skipped: handshake failed
Needs attention: 6 FAIL
  FAIL           franka.deps: missing extras for this station: franky
    hint: pip install -e '.[franky]' (from the repository root); if the handshake then reports a libfranka mismatch, rerun the doctor with --install-sdk
  ...
Result: unresolved failures / blocked checks / pending user actions
```

含 `teleop: {backend: spacemouse}` 时，`--mock` 替换遥操作为模拟设备（丢弃选项），增加以下行：

```text
...
PASS           franka.sdk:teleop:fake: no SDK needed
SKIPPED        franka.os:teleop:teleop: skipped (--mock)
...
PASS           franka.feedback:teleop:teleop: teleop poll thread alive
```

无 `--mock`，且机器无 `hidapi`、无 SpaceMouse 时，遥操作行如下（仅 SpaceMouse 后端检查 `os:hid`）：

```text
FAIL           franka.sdk:teleop:spacemouse: SpaceMouse needs the 'hidapi==0.14.0.post4' distribution (import hid)
    hint: pip install -e '.[spacemouse]'
FAIL           franka.os:teleop:teleop: no 'spacemouse' teleop device found
    hint: plug in the teleop device and run `rwrl discover`
FAIL           franka.os:hid: no HID device found
    hint: no /dev/hidraw* node: check the USB cable and port; inside Docker pass it with `docker run --device=/dev/hidrawN`
```

存在 `/dev/hidraw*` 但用户无法打开时，`os:hid` 以 `no read/write access to ['/dev/hidraw3']` 失败，提示用 `sudo cp` 安装[随包厂商规则](teleop.md#spacemouse-installation-and-linux-permissions)、加入 `plugdev`、重载规则、重新登录和插拔。缺 Python SDK 则给出上方 pip 命令（NUC、仓库根目录、激活项目环境）；原生导入失败另报 wheel/平台加载问题。

未插 CAN 适配器的 Piper 示例（缩减）：

```text
FAIL           piper.os:can_exists: CAN interface can0 not found
    hint: run `bash can_activate.sh can0 1000000` from piper_sdk, or `sudo ip link set can0 up type can bitrate 1000000`
FAIL           piper.os:can_up: can0 operstate=missing
FAIL           piper.os:can_bitrate: can0 bitrate=None, expected 1000000
FAIL           piper.handshake: CAN interface 'can0' not found
    hint: plug in the USB-CAN adapter and run: bash can_activate.sh can0 1000000  (piper_sdk) or: sudo ip link set can0 up type can bitrate 1000000
```

未安装 pyAgxArm 时 SDK 提示 `pip install -e '.[pyagxarm]' (from the repository root; pyAgxArm is installed from GitHub)`。pyAgxArm 不在 PyPI，因此扩展从 GitHub 安装。

<a id="troubleshooting"></a>

### 故障排查

表格引用检查打印的消息和提示。`<ip>`、`can0` 等值由配置填入。

| 症状 | 位置 / 输出 | 处理 |
|---|---|---|
| 缺 pip 扩展 | `deps` FAIL `missing extras for this station: franky, gello`, hint `pip install -e '.[franky,gello]' ...` | 仓库根目录执行所示命令（列表恰为缺失扩展）。 |
| Franka 不可达：断电、制动锁定、Desk 无控制权/FCI、接 Arm 端口、防火墙 | `os:ping` FAIL `<ip> does not reply` or `handshake` FAIL `cannot reach Franka at <ip>: ...`（例如 `libfranka: Connection to FCI refused. Please install FCI feature or enable FCI mode in Desk.`）, hint `check: robot powered on and brakes released; Desk reachable at https://<ip>/desk; you hold control in Desk; FCI activated (Desk → Activate FCI); cable on the Control port; firewall allows UDP` | 依提示顺序检查。仅能打开 Desk 不够，须持控制权且激活 FCI。PC 接控制箱而非臂底座。防火墙放行机器人 UDP。 |
| libfranka 版本不匹配 | `handshake` FAIL `libfranka mismatch: robot server 10, installed library 9`, hint `the robot server version 10 comes from libfranka's handshake error (authoritative); to cross-check, read the system version in Desk → Settings → System and look it up in the compatibility matrix https://frankarobotics.github.io/docs/compatibility.html; then: rwrl doctor --station <cfg> --install-sdk` | 见[安装正确 franky wheel](doctor.md#installing-the-right-franky-wheel)。连接后 `mode:libfranka_version` 显示已装版本。 |
| 用户不在 `dialout` / `input` | `os:groups` FAIL `not in group(s) dialout, input; no read/write access to [...]`（ACL/udev 已授权时为 WARN）, hint `sudo usermod -aG dialout,input $USER, then log out and back in` | 执行命令再登录，运行中会话保留旧组。 |
| Gello 端口不可读 | `os:serial` FAIL `no read/write access to <port>`, hint `grant this user access now: setfacl -m u:$USER:rw <port> ...` | `--fix` 尝试无 sudo 的 `setfacl`，否则加 sudo。ACL 重插失效；组/udev 持久。 |
| brltty 占用 CH340 | `os:brltty` FAIL `brltty active and a CH340 adapter (USB id 1a86) is attached; brltty has likely claimed <port>`（端口尚存时 WARN）, hint `sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter` | 屏蔽 brltty（`--runtime` 重启失效），重新插拔。 |
| FTDI 读取慢 | `os:latency_timer` WARN `ttyUSB0 latency_timer is 16 ms; 1 ms is recommended for Dynamixel reads`，提示规则 `ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"` | 安装 udev 规则（提示含文件和重载命令），重新插拔。CH340 无 `latency_timer`。 |
| 无 PREEMPT_RT 内核 | `realtime: enforce` 时：`mode:realtime_kernel` FAIL `realtime: enforce but the kernel is not PREEMPT_RT`，或 `handshake` FAIL `libfranka realtime setup failed ...`；`realtime: ignore` 时检查通过并提示 `control jitter possible` | 安装 PREEMPT_RT 内核及实时组限制，或用 `realtime: ignore`。 |
| Franka 在 Reflex / UserStopped / Guiding | `mode:robot_mode` FAIL `robot mode is Reflex`, hint `release the external activation device / clear errors in Desk, then run rwrl recover` | 释放激活装置，再运行 `rwrl recover`。 |
| CAN 接口缺失/未启动/比特率错 | `os:can_exists`, `os:can_up`, `os:can_bitrate` FAIL（见上文）；`handshake` FAIL `CAN interface 'can0' is down, not up` or `runs at N bit/s, expected 1000000` | `sudo ip link set can0 up type can bitrate 1000000`（或 piper_sdk 的 `can_activate.sh can0 1000000`）。 |
| Piper 无法使能（电源、急停） | `handshake` FAIL `Piper on 'can0' did not enable within 5s`, hint `check that the arm is powered, that 'can0' is up at 1000000 bit/s, and that its emergency stop is released` | 通电、释放急停、重试。连接后 `mode:piper_status` 显示状态。 |
| 无 CAN 反馈（主臂/示教模式） | `handshake` FAIL `... enabled but sent no joint feedback within 2s`, or `mode:follower` FAIL `no joint feedback`; hint `no CAN feedback: the arm may be in master/teach mode; switch it to follower (slave) mode and power-cycle` | 切换从臂 (slave) 模式，重新上电。 |
| Piper 固件配置错误 (pyagxarm) | `handshake` FAIL `firmware probe ... failed` / `cannot map Piper firmware ...`, hint `set firmware_profile explicitly (default\|v183\|v188\|v189) in the robot config` | 在 yaml 设置 `firmware_profile`。 |
| 相机序列号未找到 | `os:camera:<name>` FAIL `camera '<name>' serial '<s>' not found`, hint `set serial; discovered: [...]`（未设 `serial` 时：`no RealSense device found (serial unset)` or `serial unset and N devices attached; set serial to one of [...]`）；`handshake` FAIL `RealSense serial '<s>' for camera '<name>' not attached` | 从 `rwrl discover` 复制序列号。多相机必须设 `serial`。 |
| 相机无帧 | `feedback:camera:<name>` FAIL `no frames within 2s`, hint `check USB 3 bandwidth; try another port; rerun rwrl status after reconnecting the camera`; or `RealSense '<s>' failed to start`, hint `check the USB 3 cable/port and udev rules (librealsense scripts/setup_udev_rules.sh)` | 用 USB 3 端口和线缆，避免多相机共享集线器；装 librealsense udev，重连后 `rwrl status`。 |
| 守护进程不可达 | 客户端 `TransportError: robot daemon unreachable at <addr>` | 启动守护进程，检查 `--bind`、地址、防火墙。 |
| 令牌不匹配 | 客户端 `TransportError: token mismatch`, hint `pass the same --token to the daemon and the remote config` | 两端用相同令牌。 |
| 看门狗停止机械臂 | 客户端 `DeviceFault: the daemon watchdog stopped the arm after 0.62s without actions (limit 0.5s)`, hint `... call set_control_mode(...) again to resume servoing, and keep the action rate above 1/watchdog_s` | 动作频率高于 `1/watchdog_s`，或增大守护进程 `--watchdog-s`；再调用 `set_control_mode`。 |
| 机器人已占用 | 客户端 `DeviceBusy: device 'franka:172.16.0.2' is claimed by another process`, hint `stop the other process using 'franka:172.16.0.2', or set `claim: false` in the robot config` | 停止本机通过 `RobotService` 提供该机器人的其他进程（守护进程或 `LocalClient`）。普通 `Robot.from_yaml` 脚本不获取声明。有意退出保护可设 yaml `claim: false`。 |

<a id="installing-the-right-franky-wheel"></a>

### 安装正确的 franky wheel

`franky` 扩展安装默认 `franky-control` wheel，绑定一个 libfranka 版本。机器人服务器版本不同会拒绝握手，doctor 报告 `libfranka mismatch: robot server <N>, installed library <M>`。然后：

```bash
rwrl doctor --station robot.yaml --install-sdk         # asks before installing
rwrl doctor --station robot.yaml --install-sdk --yes   # installs without asking
```

失败 `<robot>.handshake` 行从 libfranka 握手错误读取机器人服务器版本（权威来源）。Desk 不显示该数字：交叉核查时在 Desk → Settings → System 读系统版本（如 5.9.2），查[兼容矩阵](https://frankarobotics.github.io/docs/compatibility.html)映射到机器人服务器版本。doctor 随后规划修复 `PLAN sdk:franky: Install Franky wheel for handshake-confirmed server version <N>`，准确命令为

```text
<this python> -m pip install franky-control --extra-index-url https://timschneider42.github.io/franky/whl/by-robot-server-version/<N>/ --force-reinstall --no-deps
```

并仅询问一次 `Apply this plan? [y/N]`。只有 `y`（不区分大小写，忽略两侧空白）才安装，仅装入运行 doctor 的项目环境解释器；`--install-sdk` 无 `--fix` 不做其他修复。之后复查，设备检查用新解释器加载新 libfranka。无 `--yes` 且无终端时不安装，计划为 `BLOCKED`。报告不同服务器版本的机器人不自动安装（一个 Python 环境仅一个 franky wheel），以 `sdk.franky.conflict` `BLOCKED` 报告。`--no-deps` 假定 franky 依赖（`numpy`、`websockets`）已安装，预先安装 `franky` 扩展可满足。

除设备检查，`rwrl doctor` 还检查主机、项目环境，预览定向修复（`--fix` 一次确认后修复主机和软件）；设备检查本身不修复。在 NUC 激活项目环境后运行（见 [rwrl](cli.md#robot-host-diagnosis-and-repair)）。
