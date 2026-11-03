<a id="3-start-the-daemon-and-check"></a>

# 3. 启动守护进程并检查

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/03-daemon.md)

机器人守护进程在机器人 PC 上运行，通过 gRPC 提供机械臂、夹爪、相机和 GELLO 服务。其他进程，包括录制器、actor、eval 和 deploy，都是它的客户端。

<a id="check-the-station"></a>

## 检查工作站

在尚未启动机器人服务时运行：

```bash
python -m robot_infra.discovery     # what hardware is visible (never connects to a robot)
python -m robot_infra.doctor        # the five-layer check of the station's robot
```

*预期结果*：discovery 输出相机序列号、GELLO 串口，以及 `172.16.0.2: ping=ok open ports=<list>`。对于 Franka，1337 端口开放通常表示 FCI 可访问。未连接设备时，各部分显示 `none`，见[示例输出](../robot-infra.md#2-plug-in-and-discover)。doctor 为每项检查输出一行，从 `[1/5] sdk:arm ... OK  ...` 到 `[5/5] mode:...`，最后显示 `summary: N ok, 0 failed, S skipped`；任意检查失败时退出码为 1。doctor 会连接机器人，检查后断开。`--fix` 尝试自动修复失败项，`--json` 输出机器可读结果：

```bash
python -m robot_infra.doctor --fix
```

<a id="serve"></a>

## 启动服务

不传 `--config` 时，守护进程使用工作站文件：

```bash
python -m robot_infra.server --watchdog-s 5
```

训练时需要注意 `--watchdog-s 5`：机械臂超过看门狗时间未收到动作就会进入空闲模式，默认值为 0.5 秒，而 reset 后首次策略推理可能更久。*预期结果*：先出现与 doctor 相同的预检查输出，再列出机器人部件并显示 `serving franka 'arm' on 0.0.0.0:50051`。第一个客户端连接时才连接机器人。保持终端打开；Ctrl-C 停止守护进程，机器人电源保持开启。

同一机器人由守护进程服务期间，请先停止服务再运行 `python -m robot_infra.doctor`：doctor 会争用同一设备锁并报告 `handshake` 失败。

<a id="if-a-check-fails-the-three-common-failures"></a>

## 检查失败时的三类常见问题

每条失败输出后都有 `hint: ...`，按提示处理。最常见的情况如下：

1. **GELLO 串口权限。** 出现 `[2/5] os:serial ... FAIL  no read/write access to /dev/serial/by-id/...` 时，将用户加入 `dialout` 组（`sudo usermod -aG dialout $USER`，之后重新登录）或配置 udev 规则。`serial port ... not found` 表示线缆或端口有误，检查 `/dev/serial/by-id/`。
2. **无法访问 Franka。** `os:ping` 失败，或显示 `handshake ... FAIL  cannot reach Franka at <ip>: ...`，并提示 `check: ping <ip>; FCI activated in Desk; cable on the Control (not Arm) port; 'UDP receive: Timeout' means a firewall`。在 Desk 启用 FCI，将网线插入 Control 控制箱端口，允许机器人的 UDP 数据通过防火墙，并检查 YAML 中的机器人 IP。
3. **相机序列号不明确或设备缺失。** 出现 `os:camera:<name> ... FAIL  serial unset and N devices attached; set serial to one of [...]` 或 `camera '<name>' serial '<s>' not found`。向导本会写入序列号，因此这种情况通常来自手工修改或重新插拔。将 `python -m robot_infra.discovery` 输出的序列号写入 `cameras[].serial`。`no frames within 2s` 可能是 USB 3 带宽或 udev 规则问题，可尝试 `--fix`。

完整故障表还包括 libfranka 版本不匹配、Reflex 模式与实时内核，见 [robot_infra 故障排查表](../robot-infra.md#troubleshooting)。

<a id="dry-run-without-hardware"></a>

## 无硬件演练

doctor 可以把各后端替换为 fake，检查工作站文件本身：

```bash
python -m robot_infra.doctor --config local/station.yaml --mock
```

README 的 [fake 机器人端到端演练](../project-readme.md#end-to-end-try-out-with-the-fake-robot)在单机上依次运行 fake 守护进程、learner、actor 和 checkpoint 部署。先运行一次，熟悉后续页面的命令。将 `--teleop-device leader_arm` 换成 `--teleop-device fake`，即可运行无需操作员的 actor；该脚本化输入源不会介入，见 [teleop.md](../teleop.md#devices)。`tests/test_single_arm_station_smoke.py` 使用 fake 设备执行本教程的单臂流程：

<!-- tutorial-test: skip -->
```bash
python -m pytest tests/test_single_arm_station_smoke.py -q
```

下一步：[04 录制示范](04-record-demos.md)，操作前先读 [06 人工介入](06-intervening.md)。
