<a id="1-install"></a>

# 1. 安装

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/01-install.md)

<a id="hardware-checklist"></a>

## 硬件清单

- 一台带 Franka Hand 的 Franka Panda，且已在 Desk 中启用 FCI。
- 一台 Linux 机器人 PC，通过以太网连接机械臂的 Control 控制箱端口。
- 一台通过 USB 3 连接的腕部 Intel RealSense 相机；向导将其命名为 `wrist`。额外相机依次命名为 `front`、`front_2` 等。
- 一套 GELLO、USB 总线适配器（ROBOTIS U2D2 或 OpenRB-150）和已供电的舵机。U2D2 的 USB 连接不为 Dynamixel 总线供电；包括 FTDI 延迟计时器在内的硬件细节见 [gello.md](../gello.md#1-hardware)。
- 一台用于 learner 和 BC / ACT 训练的 GPU 机器。
- actor 机器上的键盘，用于切换介入与回合操作。
- 任务所需的一个待抓取物体和一个箱子，均放在 [02](02-station.md#check-the-safety-limits) 中配置的工作区范围内。

<a id="network"></a>

## 网络

机械臂 FCI 地址在 Desk 中设置，默认是 `172.16.0.2`。为机器人 PC 上连接控制箱的网卡配置同一子网地址；本仓库不会代为配置。开始前先在 Desk 中启用 FCI，doctor 无法替你启用。actor 必须能够访问机器人 PC 上守护进程的 gRPC 端口（默认 50051），以及 learner 的同步端口（[07](07-hil-training.md) 使用 6000）。

<a id="linux-permissions"></a>

## Linux 权限

- GELLO 串口：用户需要加入 `dialout` 组（运行 `sudo usermod -aG dialout $USER` 后重新登录），或配置 udev 规则。
- RealSense：安装 librealsense 的 udev 规则（源码中的 `scripts/setup_udev_rules.sh`），并使用 USB 3 端口与线缆。
- 实时内核为可选项。默认的 `realtime: ignore` 可以在没有 PREEMPT_RT 内核时运行，此时 doctor 会报告 `control jitter possible`；`realtime: enforce` 要求 PREEMPT_RT 内核，见 [robot_infra 故障排查表](../robot-infra.md#troubleshooting)。
- 本教程的键盘方案通过 `pynput` 读取 actor 机器的桌面会话，无需加入 `input` 组。机器人 PC 上使用 evdev 脚踏或键盘时需要该组，见 [gello.md](../gello.md#6-hil-training)。

**Docker。** 守护进程在容器中运行时，需要在启动前传入 USB 设备节点，例如 GELLO 的 `--device=/dev/serial/by-id/...` 和相机设备；容器还必须能通过 UDP FCI 链路访问机器人，并让 actor 访问 gRPC 端口（使用 `--network host` 或发布端口）。本仓库测试尚未覆盖容器；robot_infra README 将 udev 和 Docker 列为[待验证项](../robot-infra.md#known-limitations-and-needs-hardware-verification)。

<a id="install"></a>

## 安装

克隆时一并获取 rl-garden 子模块，再按机器角色安装。可选依赖组来自 `robot_infra/pyproject.toml` 和 `pyproject.toml`：

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl

# Robot PC: wizard, doctor, discovery and the daemon.
pip install -e 'robot_infra[franky,grpc,realsense,gello]'

# GPU machine and actor machine: all three distributions (grpc is the client side of the daemon).
pip install -e rl-garden -e 'robot_infra[grpc]' -e .

# Actor machine: pynput for --teleop-event-device keyboard (needs that machine's desktop session).
pip install -e '.[reward-collection]'
```

| 可选依赖组 | 软件包 | 用途 |
|---|---|---|
| `robot_infra[franky]` | `franky-control` | Franka 机械臂与 Hand；其他机器人固件需要匹配的 wheel，见 [Franka 故障排查条目](../robot-infra.md#troubleshooting)。 |
| `robot_infra[realsense]` | `pyrealsense2` | RealSense 相机。 |
| `robot_infra[gello]` | `dynamixel-sdk` | GELLO 后端与标定。 |
| `robot_infra[grpc]` | `grpcio` | 守护进程及其全部客户端。 |
| `reward-collection` | `pynput` | `--teleop-event-device keyboard`。 |

如果一台机器承担所有角色：

<!-- tutorial-test: skip -->
```bash
pip install -e rl-garden -e 'robot_infra[franky,grpc,realsense,gello]' -e '.[reward-collection]'
```

安装完成后，`real-world-rl-train`（或 `python -m real_world_rl.training`）等价于 `python examples/train_real_world.py`，`real-world-rl-record` 等价于 `python examples/record_real_demos.py`，`real-world-rl-deploy` 等价于 `python examples/deploy_real_robot.py`。教程采用 `examples/` 的写法。

*预期结果*：安装正常完成。可以用 fake 设备测试集在无硬件条件下检查；需要 pytest，即 `dev` 可选依赖组：

<!-- tutorial-test: skip -->
```bash
python -m pytest tests robot_infra/tests -q
```

*结果不符时*：后续出现 `Unknown env backend 'real_robot'`，说明运行所用环境没有安装当前软件包（`-e .`）。

下一步：[02 工作站](02-station.md)。
