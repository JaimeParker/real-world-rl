<!-- English source: docs/tutorial/01-install.md -->

<a id="1-from-power-on-to-a-doctor-handshake"></a>

# 1. 从上电到 doctor 握手

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/01-install.md)

本页配置一台 NUC 与一台或两台 Franka FR3。现场须保持工作区清空、急停可用。doctor 握手成功只检查连通性和反馈，不验证运动路径或任务。

**命令环境：**所有 shell 块在 NUC 的仓库根目录 `real-world-rl/` 执行。步骤 1–3 使用 Ubuntu 系统工具，不需要 Python venv；步骤 4 安装项目环境（project environment），之后步骤 5–6 及工作站页的 Python 命令均先激活它。检查本机接口后替换示例网卡名，浏览器检查也在 NUC 上进行。

<a id="1-power-on-the-cell"></a>

## 1. 工位上电

控制箱上电后等待 Desk 可访问。清空工位，释放急停，在 Desk 使用 **Unlock joints**；安装了 Hand 时开启 **End Effector Power On**。随后两步建立完成检查所需的网络和 Desk 会话。

*应该看到：* Desk 显示机械臂已连接、关节已解锁；装有 Hand 时末端电源已开启。网页与实体都要核对。

*否则：* 检查控制箱供电、急停链和 Desk 故障。未上电的机器人不会回复 ping。不能用 Desk 运动任务或 Hand 重新初始化代替连通性检查，这些操作可能移动硬件。无实体 Hand 的工位稍后应设 `gripper: null`、`gripper_in_action: false`。

<a id="2-connect-the-control-network"></a>

## 2. 连接 Control 网络

NUC 接控制箱的 **Control** 口，不是臂底座的 **Arm** 口。也可通过交换机连接 NUC 与控制箱，无需点对点直连。每臂须有唯一 IP。

出厂 FCI 地址是 `172.16.0.2`；双臂示例左臂 `.2`、右臂 `172.16.0.4`。先核对物理身份。NUC 示例为 `172.16.0.1/24`。保留互联网连接的默认路由；隔离的机器人链路无需互联网网关。

在 NUC、仓库根目录、无需 venv，添加地址前先检查：

<!-- tutorial-test: skip -->
```bash
ip -br -4 addr
ip route get 172.16.0.2
ROBOT_IF=enp86s0  # replace with the Ethernet interface wired to Control
```

地址不存在时，以下命令**临时**添加，重启后丢失：

<!-- tutorial-test: skip -->
```bash
sudo ip addr add 172.16.0.1/24 dev "$ROBOT_IF"
sudo ip link set "$ROBOT_IF" up
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
ip route get 172.16.0.2
```

持久化示例使用 **NetworkManager**，前提是它管理 NUC 的该 Ethernet 接口。复用现有连接配置，仅接口没有配置时才新建。在 NUC、仓库根、无需 venv：

<!-- tutorial-test: skip -->
```bash
nmcli device status
nmcli connection show
ROBOT_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_IF")"
if [ -z "$ROBOT_CONNECTION" ] || [ "$ROBOT_CONNECTION" = "--" ]; then
  ROBOT_CONNECTION=franka-control
  sudo nmcli connection add type ethernet ifname "$ROBOT_IF" con-name "$ROBOT_CONNECTION"
fi
sudo nmcli connection modify "$ROBOT_CONNECTION" \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 ipv4.gateway "" \
  ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "$ROBOT_CONNECTION"
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
ip route get 172.16.0.2
```

*应该看到：* ping 回复，路由走预期网卡、源地址为 `172.16.0.1`。重启后复查持久化；第二臂对 `172.16.0.4` 重复 ping 和路由检查。

*否则：* 检查上电、Control 接线、链路、地址唯一性和实际子网。更改机器人网络前先查 VPN 路由。若 `nmcli` 显示接口未管理，应通过实际负责它的网络管理器持久化，不要再加竞争配置。局域网 Desk/FCI 不要求 Franka World 联网。

<a id="3-verify-six-separate-desk-conditions"></a>

## 3. 分别核对 Desk 六项条件

在 NUC 浏览器打开 **https://172.16.0.2/desk/**。出现自签证书告警时，核实为自己的控制箱再继续。每臂各有 Desk 页面，第二臂为 `https://172.16.0.4/desk/`。Desk 在控制箱运行，NUC 不需安装 Desk 软件包。

六项分别核对：

1. **页面可达：**加载的是目标物理臂的 Desk。
2. **状态灯绿：**机械臂/控制状态就绪，检查任何故障。
3. **已登录：**当前浏览器会话已认证。
4. **控制权在本机：**其他机器打开过 Desk 时，先在此 **Request control**。旧页面可能显示陈旧状态，刷新后再判断控制权。
5. **已安装 FCI：**System 页 **Installed Features** 有 Franka Control Interface。**FCI 1.0.0** 是功能版本，不是固件、robot server 或 libfranka 版本。
6. **已激活 FCI：**在 Desk 菜单激活 FCI 会话。入口随系统版本变化；找不到时先核对控制权、FCI 特性、机器人模式和正在运行的任务。运行 Desk 任务不能代替激活 FCI。

逐臂记下 Dashboard 的 **Control** 和 **System image** 版本，如 `5.9.2`。Desk 显示的是系统版本；franky wheel 按 **robot server** 版本选择，[兼容性矩阵](https://frankarobotics.github.io/docs/compatibility.html)提供系统版本映射，doctor 则从 libfranka 握手错误读出 server 版本。不能从 FCI 功能版本或不可用的 Software Updates 检查推断。

*应该看到：* 刷新后六项均满足；步骤 5 的真实 FCI 握手确认会话，不能只凭绿灯页面。

*否则：* 先解决具体条件，不急于重装 Python 包。Hand 初始化异常先刷新正确臂的 Desk，核对电源和实体；常见开错页面问题见[多臂检查](02-station.md#multiple-arms)。

<a id="4-install-the-project-environment"></a>

## 4. 安装项目环境

**项目环境**是本步骤安装三个本地包（rl-garden、robot_infra 和 real-world-rl）的 Python 环境，venv 或 conda 均可。`scripts/install.sh` 默认创建 `.venv`；手动路线可使用任何已激活的 venv 或 conda 环境。

checkout 应含初始化后的 `rl-garden/` 子模块。新 NUC 从父目录、未激活任何环境时执行：

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
```

安装器 `scripts/install.sh` 以及 `franka-gello` 和 `test` extras（定义在 `pyproject.toml` 中）已包含在本 checkout 中。安装器需要支持 venv 的 Python 3.10 和包下载网络。

在 NUC、仓库根、无活动环境：

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh --extras franka-gello
source .venv/bin/activate
```

安装器创建或复用 `.venv`，默认解释器 `python3.10`，清除继承的 `PYTHONPATH`，以 editable 方式安装 rl-garden、robot_infra 和本包，然后运行 doctor。未提供 station 时最终为 **mock** 检查，针对一个机械臂和夹爪都是 fake 的临时站点运行，成功不表示真实握手。RealSense 还需在同一项目环境安装 `robot_infra[realsense]`。

<details>
<summary>默认安装的手动等价命令（即安装器所做的事）</summary>

在 NUC、仓库根目录，在任何已激活的 venv 或 conda 环境中运行 pip 命令。创建并激活环境的示例：

<!-- tutorial-test: skip -->
```bash
unset PYTHONPATH
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ./rl-garden -e ./robot_infra -e '.[franka-gello,test]'
python -c 'import rl_garden, robot_infra, real_world_rl'
rwrl doctor --mock
```

</details>

RealSense 在 NUC、仓库根、项目环境激活时执行：

<!-- tutorial-test: skip -->
```bash
python -m pip install -e 'robot_infra[realsense]'
```

*应该看到：* 导入成功，mock doctor 汇总零失败。

*否则：* 查看安装失败阶段及重试提示，核对 Python/venv 和子模块。`Unknown env backend 'real_robot'` 表示运行解释器中缺少本包。`franky-control` 提供 `franky` 导入及其 libfranka 构建，无需 ROS，包名也不是 `franka`。

<a id="optional-spacemouse-dependencies"></a>

### 可选 SpaceMouse 依赖

在 NUC（或设备所在机器）的仓库根目录，激活第 4 步的项目环境后运行：

<!-- tutorial-test: skip -->
```bash
pip install -e '.[spacemouse]'
```

这会为自带的 HID 读取器添加 `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`），此路线不使用 `pyspacemouse`/`easyhid`。不要安装另一个名为 `hid` 的发行包。若从空环境开始，在 NUC、仓库根目录、未激活任何环境时改用：

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh --extras franka-gello,spacemouse
source .venv/bin/activate
```

打开设备前，在 NUC 完成一次性的 [SpaceMouse udev 步骤](../teleop.md#spacemouse-installation-and-linux-permissions)。wheel 的目标平台是 Ubuntu 22.04 / CPython 3.10 / x86_64；干净机器上的 wheel/系统库检查仍待完成，此路线**尚未在真机上验证**。

<a id="5-configure-once-then-run-doctor"></a>

## 5. 配置一次，再运行 doctor

新工位现在运行[向导](02-station.md)，创建 `local/station.yaml`。它探测硬件，询问机械臂、Hand、相机、GELLO，提供标定并运行 doctor。暂未连接的外设选 `none`，稍后再加；不可编造标定值凑检查通过。已配置工位复用原文件。

确保 recorder、actor、守护进程均未占用设备。在 NUC、仓库根、项目环境激活：

```bash
rwrl doctor --station local/station.yaml
```

| 层 | 检查内容 | 失败时 |
|---|---|---|
| 1 SDK | 后端导入 | 在项目环境安装缺少的 extra |
| 2 OS | 网络、设备存在及权限 | 回看步骤 1–3，或按步骤 6 修复 |
| 3 Handshake | 连接各配置设备 | 查 FCI 激活、控制权及 SDK 兼容性 |
| 4 Feedback | 实际状态/帧读取 | 查对应设备、线缆、相机带宽 |
| 5 Mode | 机器人模式允许使用 | 运动前检查 Desk 故障与模式 |

*应该看到：* 各项结果及 `summary: N ok, 0 failed, S skipped`。真实握手须用真实配置且不带 `--mock`；跳过项需要调查，不能算硬件验证。doctor 连接后断开，不执行 home 轨迹，但设备初始化仍可能有副作用，如 GELLO 关闭扭矩。

*否则：* 按失败项提示处理。不可达时回到步骤 1–3：上电、Control 网线、IP/路由、刷新后的 Desk 控制权和 FCI。UDP 超时查防火墙。设备 claim 被占用时先停持有它的进程。

libfranka/server 不匹配时，使用引导修复 `--install-sdk`。在 NUC、仓库根、项目环境激活：

<!-- tutorial-test: skip -->
```bash
rwrl doctor --station local/station.yaml --install-sdk
```

引导修复从 libfranka 握手错误读取 robot server 版本，要求通过 Desk → Settings → System 的系统版本与兼容表交叉核对，确认后安装匹配 wheel。选型依据是 **robot server 版本**，不是 `FCI 1.0.0`。安装后重跑普通 doctor。保留 Desk 版本记录；不要反复装同一默认 wheel 并期待握手结果变化。

<a id="6-fix-serial-and-keyboard-checks"></a>

## 6. 修复串口与键盘检查

doctor 会报告 GELLO 串口缺失或不可访问，也会诊断 CH340/brltty 冲突、输入权限与 USB 延迟。先使用引导修复，它会预览每项更改，并在改动 OS 前请求确认。在 NUC、仓库根、项目环境激活：

<!-- tutorial-test: skip -->
```bash
rwrl doctor --station local/station.yaml --fix
```

若 doctor 更改了用户组成员，注销并重新登录，再重跑普通 `rwrl doctor` 命令。`--fix` 未针对你的情况提供修复，或你希望手动执行时，使用下面的手动命令作为后备。
它们均在 NUC、仓库根执行；系统命令无需 venv，最后 doctor 使用项目环境。

`dialout` 权限失败时：

<!-- tutorial-test: skip -->
```bash
ls -l /dev/serial/by-id/
sudo usermod -aG dialout "$(id -un)"
```

注销并重新登录、激活项目环境后复查。使用稳定的 `/dev/serial/by-id/` 路径，不要假定编号为 `ttyUSB0`。

evdev 键盘或脚踏权限失败时：

<!-- tutorial-test: skip -->
```bash
sudo usermod -aG input "$(id -un)"
```

注销并重新登录。`input` 允许读取输入事件，仅 evdev 路线需要。终端 stdin 不需要该组；`pynput` 依赖已登录的桌面会话，SSH shell 加入 `input` 并不能替代桌面。

CH340 适配器因 brltty 占用而出现后又消失时，先查日志；确认冲突且不需要盲文设备服务时，临时释放后重插：

<!-- tutorial-test: skip -->
```bash
journalctl -b -u brltty-udev.service --no-pager
sudo systemctl mask --runtime --now brltty-udev.service
ls -l /dev/serial/by-id/
```

runtime mask 重启失效。反复冲突应从工位 OS 配置解决，不依赖临时 ACL 或 mask 跨重启保留。

USB 串口 `latency_timer` 偏高时，先从上述列表选择实际路径。示例为 CH340 路径，适配器不同时请替换：

<!-- tutorial-test: skip -->
```bash
GELLO_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
GELLO_TTY="$(basename "$(readlink -f "$GELLO_PORT")")"
LATENCY_FILE="/sys/bus/usb-serial/devices/$GELLO_TTY/latency_timer"
if [ -f "$LATENCY_FILE" ]; then
  cat "$LATENCY_FILE"
  echo 1 | sudo tee "$LATENCY_FILE"
  cat "$LATENCY_FILE"
fi
```

单位为毫秒，写入仅临时有效。只有提供该属性的驱动支持；没有属性不是权限错误，不应自行创建文件。重插/重启后复查。

*应该看到：* 串口持续存在且可读写，所选输入方式工作，支持该属性的设备读取为 `1`。激活项目环境后重跑：

```bash
rwrl doctor --station local/station.yaml
```

*否则：* 核对 USB 设备/驱动、刷新后的组成员资格及串口占用。相机或帧缺失另查序列号、USB 3 和厂商 udev 规则。`realtime: ignore` 允许 generic 内核但不保证时序，`realtime: enforce` 需要合适的实时配置。

<a id="a-robot-someone-else-has-already-commissioned"></a>

## 别人已经配置过的机器人

复用 `local/station.yaml` 和 NUC 上的项目环境。每次开始只做三件事：

1. 在此 NUC 打开正确臂 Desk，必要时 **Request control**，刷新验证控制权。
2. 确认该臂 **FCI 会话已激活**。
3. 其他占用设备的进程停止后，在 NUC、仓库根、项目环境激活时运行：

```bash
rwrl doctor --station local/station.yaml
```

*应该看到：* 真实握手通过且零失败。*否则：* 按前述失败层定位，不要因为另一浏览器仍持控制权就覆盖已配置 station 或重新标定。

下一步：[02 工作站](02-station.md)。
