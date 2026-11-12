<!-- English source: docs/tutorial/03-franka-bringup.md -->

<a id="3-bring-up-the-franka"></a>

# 3. 启动 Franka

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/03-franka-bringup.md)

**目标：** 完成真实 FCI 握手：`rwrl doctor` 报告 `PASS arm.handshake: robot connected`，以 `Result: no blocking findings` 结束，且 `rwrl status` 能读取机械臂。本页使用 doctor 排查机械臂，不执行运动。

所有命令在 NUC 的仓库根目录、激活项目环境后运行。

<a id="before-you-start"></a>

## 开始前

必须完成与 [02](02-setup.md#before-you-start) 相同的检查：

1. 机械臂通电、急停释放、关节解锁、末端执行器电源开启。
2. NUC 连接 **Control** 端口，`ping -c 3 -W 1 172.16.0.2` 有回复。
3. NUC 上的 Desk：持有控制权（刷新确认），FCI 已**激活**。
4. 无录制器、actor 或守护进程占用机械臂：Franka FCI 仅允许一个控制连接。

已有 `local/station.yaml`（[02](02-setup.md)）。

<a id="1-run-the-doctor"></a>

## 1. 运行 doctor

```bash
rwrl doctor --station local/station.yaml
```

双臂时逐臂检查：

```bash
rwrl doctor --station local/station.yaml --robot left
rwrl doctor --station local/station.yaml --robot right
```

doctor 连接每台机械臂后断开，不执行 home 或运动。初始化配置设备仍可能有副作用，例如关闭 GELLO 扭矩。

<a id="2-read-the-report"></a>

## 2. 阅读报告

每行格式为 `STATUS id: message`，状态补齐到 14 字符；非 `PASS` 行后附 `    hint: ...`。终端中的状态带颜色（[00](00-overview.md#status-words)）。报告分为两组：

- **主机行** (`host.*`, `usb.*`, `gello.*`, `network.route.<ip>`, `realtime.*`, `deps.*`, `sdk.*`, `coverage.*`)：NUC、权限及项目环境。
- **设备行** `<robot>.<check>`（此处为 `arm.<check>`），共六层：

| 层 | 报告行 | 含义 |
|---|---|---|
| 0 deps | `arm.deps` | 已安装站点所需 pip 扩展 |
| 1 sdk | `arm.sdk:arm`, `arm.sdk:gripper`, `arm.sdk:camera:<name>`, `arm.sdk:teleop:<backend>` | 各后端 SDK 可导入 |
| 2 os | `arm.os:ping`, `arm.os:tcp`, `arm.os:camera:<name>`, `arm.os:serial`, `arm.os:groups`, `arm.os:brltty`, `arm.os:latency_timer` | 网络、设备节点及权限 |
| 3 handshake | `arm.handshake` | 设备占用声明及 `connect()`；失败则跳过第 4–5 层 |
| 4 feedback | `arm.feedback:arm`, `arm.feedback:gripper`, `arm.feedback:camera:<name>`, `arm.feedback:teleop:<name>` | 各设备返回数据 |
| 5 mode | `arm.mode:robot_mode`, `arm.mode:libfranka_version`, `arm.mode:realtime_kernel` | 机械臂模式允许使用 |

报告行后是 `Report:`（本次报告目录）、`Needs attention:` 块（存在问题时重复每个 `FAIL`、`BLOCKED`、`PENDING_ACTION` 行及其首行提示）和 `Result:`。仅有 `PASS`、`WARN`、`SKIPPED` 时退出码为 0，否则为 1。

无需硬件即可查看格式：对使用模拟后端的示例站点运行 doctor。在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl doctor --station examples/stations/robot.franka.example.yaml --mock
```

*应看到*（示例机器人名为 `franka`；平台、解释器路径和报告目录会不同）：

<!-- doctor-example: mock -->
```text
PASS           host.platform: Linux ubuntu 22.04 x86_64; container=False; wsl=False
PASS           host.python: /home/robot/real-world-rl/.venv/bin/python
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
Report: /home/robot/real-world-rl/local/doctor/20261112T101500-3f2a9c1d
Result: no blocking findings; review SKIPPED/WARN coverage
```

`--mock` 将所有后端替换为模拟后端并跳过第 2 层，因此只检查 YAML 和安装。真实站点的设备行显示真实后端，例如 `PASS           arm.sdk:arm: 'franky' importable`、`PASS           arm.handshake: robot connected` 和 `PASS           arm.mode:robot_mode: robot mode is Idle`。`SKIPPED` 不证明部件可用：查明跳过原因。

<a id="3-red-lights"></a>

## 3. 红灯

找到第一个红色行及其提示。常见情况：

| 报告行 | 常见原因 | 处理 |
|---|---|---|
| `arm.os:ping` FAIL `172.16.0.2 does not reply` | 电源、网线接在 Arm 端口、地址错误 | [网络与 FCI](../doctor.md#network-and-fci) |
| `arm.handshake` FAIL `cannot reach Franka at 172.16.0.2: ...` | FCI 未激活、另一机器持有 Desk 控制权、防火墙阻挡 UDP | [网络与 FCI](../doctor.md#network-and-fci) |
| `arm.handshake` FAIL `libfranka mismatch: robot server N, installed library M` | franky wheel 与机器人服务器不匹配 | `rwrl doctor --station local/station.yaml --install-sdk`；[SDK 与 libfranka 版本](../doctor.md#sdk-and-libfranka-version) |
| `arm.deps` or `arm.sdk:*` FAIL | 缺少扩展 | 提示中的 `pip install -e '.[...]'`；[Python 环境与扩展](../doctor.md#python-environment-and-extras) |
| `arm.os:serial`, `arm.os:groups`, `arm.os:brltty`, `arm.os:latency_timer` | GELLO 串口访问 | `rwrl doctor --station local/station.yaml --fix`；[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency) |
| `arm.feedback:camera:<name>` FAIL `no frames within 2s` | USB 带宽、线缆、udev 规则 | [相机](../doctor.md#cameras) |
| `arm.device` BLOCKED `device inspection requires an idle station: ...` | 录制器、actor 或守护进程占用机械臂 | 结束该进程；[设备占用](../doctor.md#busy-devices-blocked) |
| `arm.mode:robot_mode` FAIL `robot mode is Reflex` | 反射停止、用户停止或引导模式 | 下方第 5 步；[机器人模式](../doctor.md#robot-mode-reflex-userstopped-guiding) |

libfranka 不匹配时，引导修复要求交叉核对机器人服务器版本（Desk → Settings → System，结合兼容矩阵），确认后才安装匹配的 wheel：

```bash
rwrl doctor --station local/station.yaml --install-sdk
```

串口和输入权限的引导修复会预览所有改动，统一询问一次，执行并再次检查：

```bash
rwrl doctor --station local/station.yaml --fix
```

若更改了组成员关系，退出登录后重新登录，再运行第 1 步普通 doctor。

<a id="4-read-the-arms-state"></a>

## 4. 读取机械臂状态

doctor 为绿色且无其他进程占用机械臂时：

```bash
rwrl --station local/station.yaml status
rwrl --station local/station.yaml q
rwrl --station local/station.yaml tcp
```

*应看到*，例如：

```text
connection: direct franky
robot: arm (franka)
arm: FrankyArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=False, source FrankyGripper
cameras: wrist
```

随后是遥操作名称、含七个关节位置的 `q (rad): [...]`，以及 `tcp xyz (m): [...]; quat (xyzw): [...]`。`connection: direct franky` 表示 `rwrl` 自行打开机械臂；含 `daemon:` 块的站点则报告 `connection: daemon HOST:PORT`（[连接归属](../cli.md#connection-ownership)）。每条命令运行时连接，结束后断开。从包含 `local/` 的目录运行时可省略 `--station local/station.yaml`。

<a id="5-after-a-reflex-stop"></a>

## 5. 反射停止后

碰撞、关节越限或外部激活装置会令机械臂进入 Reflex 或 UserStopped；doctor 报告 `arm.mode:robot_mode` FAIL，提示 `release the external activation device / clear errors in Desk, then run rwrl recover`。检查工位，释放激活装置或在 Desk 清除错误，然后：

```bash
rwrl --station local/station.yaml recover
rwrl doctor --station local/station.yaml
```

*应看到：* `recover: ok`，随后 `arm.mode:robot_mode` 恢复 `PASS`。`rwrl recover` 与 `status` 使用相同连接；直连时先结束录制器或 actor。

对比环境预设和站点（`rwrl doctor --preset ...`）需要预设的遥操作文件，将在 [08](08-record-demos.md) 完成。

<a id="a-robot-someone-else-has-already-commissioned"></a>

## 他人已完成调试的机器人

复用 NUC 上现有 `local/station.yaml` 和项目环境；每次会话前只需完成三步：

1. 在 NUC 打开此臂的 Desk，必要时 **Request control** 并刷新。
2. 确认此臂 FCI 会话已**激活**。
3. 停止其他占用设备的进程，运行第 1 步 doctor。

*应看到：* 真实握手且无失败行。不要仅因其他浏览器保留控制权就覆盖已调试站点或重新校准。

<a id="if-it-fails"></a>

## 失败怎么办

查看 doctor 哪一行为红色，打开 [rwrl doctor：红灯处理](../doctor.md#red-lights-what-to-do)对应章节。先修复首个红色行：握手失败会跳过所有反馈和模式行。

下一步：[04 Home、夹爪、相机](04-first-motion.md)。
