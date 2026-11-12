<!-- English source: docs/tutorial/02-setup.md -->

<a id="2-set-up-the-station"></a>

# 2. 设置站点

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/02-setup.md)

**目标：** 得到经过核查的 `local/station.yaml`，指定此机械臂、Hand、相机和 GELLO，以及适合工位的 home 和限制。后续每条 `rwrl` 命令都读取此文件。

除非代码块另有说明，所有命令均在 NUC 的仓库根目录、激活项目环境后运行（见 [01](01-install.md)）。

<a id="before-you-start"></a>

## 开始前

向导会探测硬件，因此设备应通电、接线并可达。逐项核查，详细说明折叠在下方。

1. **机械臂通电并解锁。** 控制箱已开机、急停已释放，Desk 中已完成 **Unlock joints**，装有 Hand 时已开启 **End Effector Power On**。*检查：* Desk 显示机械臂已连接且关节解锁。
2. **Control 端口网络。** NUC 接控制箱的 **Control** 端口（不是臂底座的 **Arm** 端口），可以经过交换机。NUC 在该接口的地址为 `172.16.0.1/24`，出厂 FCI 地址为 `172.16.0.2`。*检查（无需环境）：*

   <!-- tutorial-test: skip -->
   ```bash
   ip route get 172.16.0.2
   ping -c 3 -W 1 172.16.0.2
   ```

   路由应经连接 Control 的以太网接口发出，源地址为 `172.16.0.1`，且 ping 有回复。
3. **Desk 控制权与 FCI。** 在 NUC 浏览器中打开 `https://172.16.0.2/desk/`，登录，此机器持有控制权（**Request control** 后刷新），且 FCI 会话已**激活**。*检查：* 刷新页面后核对下方六项条件。
4. **没有其他进程占用设备。** 先停止录制器、actor 或守护进程。

<details>
<summary>为 NUC 配置 Control 网络地址</summary>

先检查，再选择连接 Control 的接口。在 NUC 的仓库根目录，无需环境：

<!-- tutorial-test: skip -->
```bash
ip -br -4 addr
ROBOT_IF=enp86s0  # replace with the Ethernet interface wired to Control
```

这会**临时**添加地址（重启后失效）：

<!-- tutorial-test: skip -->
```bash
sudo ip addr add 172.16.0.1/24 dev "$ROBOT_IF"
sudo ip link set "$ROBOT_IF" up
ping -I "$ROBOT_IF" -c 3 -W 1 172.16.0.2
```

若 NUC 的以太网由 NetworkManager 管理，要持久配置地址，复用该接口现有连接配置；没有时新建：

<!-- tutorial-test: skip -->
```bash
ROBOT_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "$ROBOT_IF")"
if [ -z "$ROBOT_CONNECTION" ] || [ "$ROBOT_CONNECTION" = "--" ]; then
  ROBOT_CONNECTION=franka-control
  sudo nmcli connection add type ethernet ifname "$ROBOT_IF" con-name "$ROBOT_CONNECTION"
fi
sudo nmcli connection modify "$ROBOT_CONNECTION" \
  ipv4.method manual ipv4.addresses 172.16.0.1/24 ipv4.gateway "" \
  ipv4.never-default yes connection.autoconnect yes
sudo nmcli connection up "$ROBOT_CONNECTION"
ip route get 172.16.0.2
```

保留互联网连接的默认路由；机器人链路无需网关。每台机械臂需要唯一地址（双臂示例用 `172.16.0.2` 和 `172.16.0.4`）。若 `nmcli` 报告设备未受管理，使用实际管理它的网络管理器配置。更改机器人网络前检查 VPN 路由，重启后再次检查路由。

</details>

<details>
<summary>Desk 的六项条件及需记录的版本</summary>

在 NUC 打开 `https://172.16.0.2/desk/`（会提示自签名证书；确认是你的控制箱）。Desk 在控制箱运行，无需在 NUC 安装。

1. 目标实际机械臂的**页面可达**。
2. **状态灯为绿色**；检查所有故障。
3. **已登录。**
4. **此机器持有控制权：** 若另一台机器持有，点击 **Request control** 并刷新，旧页面可能显示过时状态。
5. **已安装 FCI：** System → **Installed Features** 列出 Franka Control Interface。**FCI 1.0.0** 等条目是功能版本，不是固件或 libfranka 版本。
6. 从 Desk 菜单**激活 FCI**（位置随系统版本而变）。运行 Desk 任务不能替代此步骤。

记录 Dashboard 的 **Control** 和 **System image** 版本（例如 `5.9.2`）。franky wheel 按 *robot server* 版本选择；第 03 页 doctor 从 libfranka 握手错误中读取它；[兼容矩阵](https://frankarobotics.github.io/docs/compatibility.html)给出系统版本与它的对应关系。

</details>

机械臂尚不可达时，向导仍可写文件：传入 `--allow-unprobed` 接受探测未找到的 IP、相机序列号或串口，再在 [03](03-franka-bringup.md) 检查。

<a id="1-see-what-is-connected"></a>

## 1. 查看已连接设备

```bash
rwrl discover
rwrl --json discover
```

*应看到：* `cameras:`（RealSense 序列号）、`teleops:`（如 GELLO 的 `/dev/serial/by-id/...` 路径）、`can interfaces:`、`hosts:` 和 `hid devices:`；未找到设备时各项为 `none`。`hosts:` 仅列出现有站点文件的机器人 IP；`discover` 不会连接它们。`--json` 将同一报告输出为一个对象，含 `schema_version: 1` 和 `cameras`、`teleops`、`can_interfaces`、`hosts`、`hid_devices`、`unknown_hid` 列表。

<a id="2-run-the-wizard"></a>

## 2. 运行向导

```bash
rwrl setup
```

向导探测硬件、询问问题，在你同意时校准 GELLO，写入带注释的 `station.yaml`，对它运行 doctor 并打印后续命令。每个问题都有对应选项（`rwrl setup --help`）；提供选项后不再询问。例如一台机械臂带 GELLO，其余逐项询问：

```bash
rwrl setup --arms 1 --arm-names arm --robot-ip 172.16.0.2 --teleop gello --scope local
```

| 选项 | 含义及省略时的默认值 |
|---|---|
| `--arms 1` or `--arms 2` | 机械臂数量；新站点建议 1 |
| `--arm-names arm` or `--arm-names left right` | 文件和 doctor 标识中的名称 (`arm.handshake`) |
| `--robot-ip 172.16.0.2` | 各臂 FCI 地址，也会探测 |
| `--gripper franky` | `franky`（Franka Hand）、`none`（无 Hand）、`fake` 或 `serl_http` |
| `--cameras none` or `--cameras SERIAL=wrist` | 每臂：`rwrl discover` 的序列号及观测名称，逗号分隔 |
| `--camera-resolution 1280x720`, `--camera-fps 15`, `--camera-depth yes` | 每臂相机配置；未设置时新相机为 640x480、30 fps、无深度，保留的相机沿用原值 |
| `--teleop gello` | `gello`、`spacemouse`、`fake` 或 `none` |
| `--gello-port PATH` | 主臂的 `/dev/serial/by-id/...` 路径（或向导列表编号） |
| `--no-calibrate` | 推迟 GELLO 校准；交互模式默认询问 |
| `--scope local` / `--scope user` / `--output PATH` | 写入 `./local/station.yaml`、`~/.config/rwrl/station.yaml` 或指定路径 |
| `--allow-unprobed` | 接受探测未找到的值 |

回答说明：

- **机器人 IP、相机、GELLO 端口**可填列表编号或实际值。
- **相机名称**成为观测键：`wrist` 在 `obs.rgb` 中为 `rgb_wrist`。本教程使用一台名为 `wrist` 的相机。
- **介入：** 保持 `always`；actor 的按键事件会替代它。`toggle` 用于 NUC 上的 evdev 踏板（[gello.md](../gello.md#6-hil-training)）。
- **GELLO 夹爪：** 新通道使用 `gripper_binarize_threshold: 0.5`，存储动作恰为 `+1`（开）或 `-1`（闭）。
- 现有文件会先备份为 `station.yaml.bak.<UTC timestamp>`。

全部答案通过选项提供时，向导不再提问。在 NUC 的仓库根目录，激活项目环境，使用自己的 GELLO 路径：

```bash
GELLO_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
rwrl setup --non-interactive --arms 1 --arm-names arm --robot-ip 172.16.0.2 \
    --arm-backend franky --gripper franky --home-joints 0,-0.785,0,-2.356,0,1.571,0.785 \
    --cameras none --teleop gello --gello-port $GELLO_PORT --gello-baudrate 57600 \
    --gello-joint-ids 1,2,3,4,5,6,7 --gello-gripper-id 8 --intervention always \
    --event-device none --daemon-port 50051 --scope local
```

`--non-interactive` 推迟校准；若某问题既无选项值也无 `--station` 值则失败。

**GELLO 校准。** 回答 `yes` 将执行与 `rwrl teleop calibrate --backend gello` 相同的引导流程：关闭扭矩，沿机器人正方向移动各关节，保持校准姿态，然后开闭扳机。见 [05](05-teleop-gello.md) 和 [gello.md](../gello.md#2-calibration)。未校准时文件使用零偏移、`+1` 符号及 `# TODO calibrate` 注释；不要用这些值移动机器人。之后可任选命令校准：

```bash
rwrl setup --station local/station.yaml --only teleop --calibrate
rwrl --station local/station.yaml teleop calibrate --backend gello --use
```

**无硬件演练。** `--mock` 使用预置探测结果和模拟 GELLO；请写到真实站点之外：

```bash
rwrl setup --mock --output local/station.mock.yaml
```

*应看到：* 带注释 YAML 的路径、以 `Result:` 结束的 doctor 设备行及后续命令。*否则：* 见[失败怎么办](#if-it-fails)。

<a id="3-where-the-file-lands"></a>

## 3. 文件保存位置

`--scope local` 相对运行目录写入 `./local/station.yaml`（`local/` 被 git 忽略）。后续命令无需 `--station` 即可找到它，按 `--station PATH`、`$RWRL_STATION`、`./local/station.yaml`、`~/.config/rwrl/station.yaml` 顺序取首个匹配（[站点查找](../cli.md#station-lookup)）。在同一目录运行后续命令，或设置 `RWRL_STATION`。

<a id="4-check-limits-and-home"></a>

## 4. 检查限制与 home

向导写入 Panda 关节限制、伺服单步上限 (`max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`) 及接受的 `safety_box`。安全盒采用机器人基坐标系（`tcp_pose = O_T_EE`，米），必须包含 home、物体和容器；`home_joints` 必须位于 `joint_limits` 内。设备层按这些限制裁剪伺服目标，拒绝越界 `move_to_*` 目标（[安全规则](../devices.md#safety-rules)）；训练环境只能进一步收紧限制。

无需打开文件即可检查（仅读取站点文件，不连接设备）：

```bash
rwrl --station local/station.yaml workspace show
```

*应看到：* 含 `joint_limits`、`max_delta_per_step`、`home_joints`、`base_safety_box` 和 `home_within_limits: true` 的 JSON。`false` 时附 `home_violations`，每个越界关节一项；`null` 表示缺少 home 或限制。编辑后重试；使用 `--station` 重跑向导会保留修改。

<a id="5-what-the-station-file-contains"></a>

## 5. 站点文件内容

| 键 | 含义 |
|---|---|
| `type`, `name` | 机器人类型 (`franka`) 及 doctor 标识所用名称 |
| `arm_backend`, `robot_ip`, `realtime` | `franky`、FCI 地址、`ignore` 或 `enforce` (PREEMPT_RT) |
| `gripper` | Hand (`backend: franky`) 或 `null` |
| `cameras` | 每台相机的名称、`serial`、`width`、`height`、`fps`、`enable_depth` |
| `teleop` | 后端和选项；GELLO 的端口、舵机 ID 和校准 |
| `home_joints`, `joint_limits`, `max_delta_per_step`, `safety_box` | 上述 home 姿态和限制 |
| `control` | 可选阻抗配置（[控制配置](../control-profiles.md)） |
| `daemon` | 仅用于远程 actor 路径（[远程 actor](../remote-actor.md)） |
| `task_frame`, `workspace` | 可选任务坐标系与工作区（[04](04-first-motion.md#5-optional-task-frame-and-workspace)） |

每个字段见[设备层指南](../devices.md#station-file-reference)。

<a id="multiple-arms"></a>

## 多臂

每台机械臂拥有独立 IP、Desk 页面和已激活 FCI 会话；本例 **left 为 `172.16.0.2`，right 为 `172.16.0.4`**。分配相机、GELLO 端口或 home 前，对照实际机械臂核实，并激活两臂 FCI。无相机、推迟校准的双臂交互配置：

```bash
rwrl setup --arms 2 --arm-names left right \
    --robot-ip 172.16.0.2 172.16.0.4 --cameras none none \
    --teleop gello --no-calibrate --scope local
```

为每个 GELLO 选择独立串口，运动前完成两者校准。没有 Hand 的臂选择无夹爪；软件宽度读数不能证明装有 Hand。切勿将一臂的 home 关节复制给另一臂。*应看到：* `robots.left`、`robots.right` 使用预期 IP。[03](03-franka-bringup.md) 逐臂检查；其他双臂设置见 [bimanual.md](../bimanual.md)。

<a id="redo-one-section"></a>

## 重做某一部分

```bash
rwrl setup --station local/station.yaml
rwrl setup --station local/station.yaml --only cameras
```

第一条重新询问全部问题，旧值作为默认值；`--only cameras`、`teleop` 或 `daemon` 仅重做一部分，其余保留。

<a id="if-it-fails"></a>

## 失败怎么办

- **向导未列出应有设备：** 运行 `rwrl discover`，检查接线与电源后重试；或传入 `--allow-unprobed` 及实际值。
- **向导 doctor 有红色行：** 查看具体行，再到 [rwrl doctor：红灯处理](../doctor.md#red-lights-what-to-do)。机械臂不可达见[网络与 FCI](../doctor.md#network-and-fci)；GELLO 端口见[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)。
- **`home_within_limits: false`：** 修正文件中的 `home_joints` 或 `joint_limits`。

下一步：[03 启动 Franka](03-franka-bringup.md)。
