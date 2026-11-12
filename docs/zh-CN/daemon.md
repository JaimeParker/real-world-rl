<a id="the-robot-daemon"></a>

# 机器人守护进程

守护进程在机器人 PC 上运行，为 GPU 机器上的 RemoteRobot 提供设备 RPC。若 actor 运行在机器人主机、learner 在远程，见[远程 actor 与 learner](remote-actor.md)，该页介绍训练传输。工作站字段与安全约定见[设备参考](devices.md)。

<a id="serve-the-robot"></a>

## 提供机器人服务

在机器人 PC 的仓库根目录、项目环境已激活时运行：

```bash
pip install -e '.[grpc]'   # plus the robot's backend extras
rwrl daemon --station robot.yaml --bind 0.0.0.0:50051 --token T --watchdog-s 0.5
```

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--station` | [工作站文件](tutorial/02-setup.md) | 机器人或工作站 YAML |
| `--robot` | 所有机器人 | 仅服务工作站中的一台；省略时多机器人工作站为每台启动一个 daemon 进程 |
| `--bind` | 机器人的 `daemon.bind`，否则 `0.0.0.0:50051` | 监听 `host:port`（仅单机器人） |
| `--token` | 机器人的 `daemon.token`，否则无 | 共享密钥，客户端须发送同一值（gRPC 元数据 `x-robot-token`）（仅单机器人） |

| `--watchdog-s` | `0.5` | 经过此秒数无动作后，将伺服中的机械臂置为空闲 |
| `--no-preflight` | 关闭 | 跳过服务前的 doctor 检查 |
| `--transport` | `grpc` | 唯一传输方式 |
| `--jpeg-quality` | 无 | 各相机 `color` 图像的 JPEG 质量 1–100，见下文 |

多机器人且未给 `--robot` 时，此命令作为 supervisor，为每台执行 `rwrl daemon --station <station> --robot <name>`（各自独立进程会话，传递 `--watchdog-s`、`--jpeg-quality`、`--no-preflight`、`--transport`），转发 SIGINT/SIGTERM，等待全部结束后以首个非零子进程退出码退出。此时 `--bind`/`--token` 是错误（应逐机器人设置 `daemon.bind`/`daemon.token`），两台共用 `daemon.bind` 也会报错。以 SIGKILL 终止 supervisor 会留下运行中的 daemon，须用 SIGTERM 停止它们。

`--jpeg-quality N`（daemon **和**所有客户端均须在仓库根目录的项目环境安装 `pip install -e '.[jpeg]'`，需要 Pillow 或 OpenCV）将各相机 `color` 图像以 JPEG 代替原始字节发送，以降低 WiFi 带宽。它是**有损**的 daemon 全局设置：仅压缩 `color`，`depth` 和其他字段保持原始格式，客户端解码回 `uint8 (H, W, 3)` 数组。因此策略看到压缩像素，**训练与部署应使用相同值**（或都不启用）。`robot.describe` 通过顶层 `image_encoding`（如 `{"jpeg": 80}`，原始格式为 `null`）报告此值，客户端可读 `robot.cameras["wrist"].image_encoding`。客户端缺少 Pillow/OpenCV 时，`connect()` 抛出 `SdkMissing: JPEG frames need Pillow or OpenCV`（提示 `pip install -e '.[jpeg]' (from the repository root)`）。

Daemon 与客户端来自同一仓库；应**先升级客户端，再升级 daemon**。新 daemon 的 `arm.health` 多了 `safety_clips`、`safety_rejects` 两字段，升级前客户端无法解析；启用 `--jpeg-quality` 也需客户端支持 JPEG 解码。JPEG 支持之前的客户端无法解码帧（其具体错误未经检查）。

执行过程：

1. **配置。** 加载 YAML，然后用同名大写环境变量填充每个空字段（`""` 或 null），例如 `FIRMWARE_PROFILE=v188`。`ROBOT_IP`、`CAN_INTERFACE` 仅在 YAML 设置 `robot_ip: ""` / `can_interface: ""` 时生效，因为两者有非空默认值。相机 `serial` 为空且该后端仅连接一台相机时自动使用其序列号。Daemon、doctor 和 `rwrl discover --station` 都执行自动填充；`Robot.from_yaml` 按文件原样读取。
2. **预检。** 未传 `--no-preflight` 时运行完整 doctor（非 `--mock`）并输出报告。任一检查失败，daemon 输出 `preflight failed; fix the items above or start with --no-preflight` 并以 1 退出。Doctor 持有设备 claim 后连接并断开机器人，因此同一机器人已有 daemon 时会报告 `handshake` 失败。fake 后端跳过操作系统检查（`backend 'fake' needs no OS device`），所以示例/fake 配置无需 `--no-preflight`。
3. **启动输出**（fake 后端配置）：

   ```text
   franka (franka)
     arm: FakeArm [sdk=None, connected=False]
     gripper: FakeGripper [sdk=None, connected=False]
     cameras: PartGroup [sdk=None, connected=False]
       wrist: FakeCamera [sdk=None, connected=False]
     teleop: FakeTeleop [sdk=None, connected=False]
   serving franka 'franka' on 127.0.0.1:50061 (token required)
   ```

   仅在客户端调用 `connect()` 时连接机器人。此时 daemon 获取设备 claim，`disconnect()` 时释放。
4. **Watchdog。** 机械臂处于 `CARTESIAN_SERVO` 或 `JOINT_SERVO` 时，daemon 每隔 `watchdog_s / 4` 检查距上次动作的时间。动作包括 `send_action`、`step`、`servo_cartesian` 和 `servo_joints`；进入伺服模式也开始计时。超过 `watchdog_s` 无动作时，daemon 调用 `arm.stop()`，切换到 `IDLE`，记录 `watchdog: no action for 0.624s in joint_servo mode; stopping arm 'arm' and idling it`。之后的伺服命令抛出 watchdog `DeviceFault`，直到客户端再次调用 `set_control_mode(...)`。`move_to_*` 调用（硬件后端仅在 `IDLE` 允许）不受监控。
5. **遥操作。** YAML 含 `teleop:` 时，daemon 提供 `teleop.<name>.read` 和 `teleop.<name>.reset`。读取不刷新 watchdog，客户端 `timeout_s` 上限为 2 秒，防止占住 gRPC 工作线程。`teleop` 一旦配置便是必需部件：设备缺失会使 `connect()` 失败并断开整个机器人，因此无需遥操作的部署应使用不带此段的 YAML。
6. **Ctrl-C / SIGTERM。** Daemon 停止传输与 watchdog，断开机器人并释放 claim。从不关闭机器人电源，也不禁用电机（见[安全规则](devices.md#safety-rules)）。

客户端根据 `robot.describe` 构建镜像。上方 fake 机器人对应如下（已节选）：

```text
arm: {"num_joints": 7, "supported_state_fields": ["tcp_pose", "tcp_vel", "joint_pos", "joint_vel", "tcp_force", "tcp_torque"], "control_mode": "idle", "safety": {"safety_box": [-0.5, -0.5, -0.5, 0.5, 0.5, 0.5], "joint_limits": [[-1.0, 1.0], ...], "max_delta_per_step": {"xyz_m": 0.05, "joint_rad": 0.1}}, "home_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
image_encoding: {'jpeg': 80}
camera cfg: {'name': 'wrist', 'serial': '', 'width': 64, 'height': 48, 'fps': 30, 'enable_depth': False, 'backend': 'fake'}
teleop: {'name': 'teleop', 'backend': 'fake', 'action_space': 'joint_abs', 'has_gripper': True, 'hold_s': 0.0}
```

无配置限制时 `arm.safety` 为 `null`；`arm.home_joints` 为 YAML 值（或 `null`）。描述还包括 `arm.safety_capabilities`：`task_workspace_v1` 表示笛卡尔工作空间约束，`accepted_cartesian_target_v1` 表示已接受目标保持，Franky 还支持 `stop_trace_v1`。客户端使用新行为前协商能力；旧服务器无法证明支持。已连接遥操作描述带 `device_identity`（USB vendor/product，以及可用的序列号/型号）；`RemoteTeleop.device_identity` 暴露同一身份用于档案验证。连接前可能无法取得身份。

四个 RPC 补充现有机械臂命令：`arm.last_cartesian_target`、`arm.begin_stop_trace`、`arm.stop_with_trace`（`post_stop_s=2.1`）、`arm.last_stop_trace`。`last_cartesian_target` 是当前控制段中通过安全检查并交给后端的最新笛卡尔目标副本。任何 `stop()`、`stop_with_trace()`、控制模式切换或断开都会结束控制段，之后为 `null`，直到下一次成功的 `servo_cartesian`。它绝不是实测反馈。`piper_sdk` 和 `pyagxarm` 后端仍会转换目标（毫米或千分之一度舍入、四元数转 RPY），所以它不完全等同于机械臂收到的值。`Arm.stop()` 是模板方法：先清除目标，再调用后端 `_stop()`。后端实现 `_stop()`，不得实现 `stop()`。`begin_stop_trace` 在现有连接上启动有界采样；SDK 指纹/静态元数据在停止/服务锁之外收集，并按连接缓存。等待旧采样器线程结束也在停止锁外进行。

`stop()` 保留普通 Franky 错误语义：在 health 记录失败而不抛出。仅 `stop_with_trace` 对停止失败抛出 `DeviceFault`，并在 `last_stop_trace` 保留 FAIL 证据。它先发送停止，再尝试创建缺失的记录器；准备失败会在直接停止后生成 BLOCKED 证据。停止后的观测期间仍允许读取/停止 RPC，但拒绝控制写入。`robot.step` 的慢相机采集不持有 `arm.stop` 所需的调度锁。

停止轨迹记录开始停止时的 `control_mode`、`tracker_active` 以及每个样本的 SDK `robot_mode`。已处于 IDLE/未激活的机械臂即使所有位姿静止，也得到 BLOCKED（`no active motion at stop`）。机器人模式故障与停止失败得到 FAIL；缺失或未知的模式/覆盖证据不能得到 PASS。采样停止验收不是连续时间的物理保证。

<a id="use-the-robot-from-the-gpu-machine"></a>

## 从 GPU 机器访问机器人

在 GPU 机器的本仓库根目录、项目环境已激活时运行：

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

缺少 `import real_world_rl.server` 时，`Robot.from_dict` 抛出 `unknown robot type 'remote'; registered: ['franka', 'piper']`。

- 动作按部件嵌套，机械臂键取决于控制模式：`CARTESIAN_SERVO` 使用 `tcp_pose`（7 个值：xyz 加 xyzw 四元数），`JOINT_SERVO` 使用 `joint_pos`。夹爪使用 [0, 1] 归一化宽度。
- `step()` 返回 `{"arm": {...}, "gripper": {"gripper_pos": ...}, "cameras": {<name>: {"color": ...}}}`。机械臂字段由后端 `SUPPORTED_STATE_FIELDS` 决定。
- `RemoteRobotConfig` 其他键：`timeout_s`（5.0）、`reconnect_attempts`（5）、`reconnect_backoff_s`（0.5）。连接丢失后客户端重连，仅重试只读调用。命令绝不重试：需重连并重新设置控制模式。
- 单机不使用 gRPC 时，可在进程内包装本地机器人：

  ```python
  from real_world_rl.devices import Robot
  from real_world_rl.server import RemoteRobot, RobotService
  from real_world_rl.server.transport import LocalClient

  service = RobotService(Robot.from_yaml("robot.yaml"), watchdog_s=0.5)
  robot = RemoteRobot.from_client(LocalClient(service))   # same API as above, incl. watchdog
  ```

- Daemon 的设备侧事实镜像到客户端：`robot.arm.safety_limits`（上方 `arm.safety` 字典或 `None`）、`robot.arm.home_joints`（列表或 `None`）、`robot.cameras["wrist"].image_encoding` 和 `robot.teleop`（无 `teleop:` 时为 `None`）。客户端不执行限制（`RemoteRobotConfig` 忽略 `safety_box`、`joint_limits`、`max_delta_per_step` 和 `teleop`），由 daemon 执行。
- 越界命令在客户端的行为如下（在 `robot.disconnect()` 前运行）：

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

  `SafetyViolation` 是 `DeviceError`（不是 `DeviceFault`）：不锁存故障，无需 `recover()`，不刷新 daemon watchdog。`robot.health()` 为 `Health`；逐臂计数在 `robot.arm.health()` 和 `robot.remote_health()`。
- `real-world-rl` 将此机器人包装成 gym 环境。

<a id="use-the-robot-locally-no-daemon"></a>

## 本地使用机器人（无守护进程）

在机器人 PC 的仓库根目录、项目环境已激活时，可直接驱动设备层：

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

此路径没有 watchdog，也不获取设备 claim；它们属于 daemon 功能。但安全限制在此仍生效：`Robot.from_yaml` 用 YAML 的 `safety_box`/`joint_limits`/`max_delta_per_step` 构建机械臂，因此 `robot.arm.safety_limits`、`robot.arm.health().safety_clips` 和 `SafetyViolation` 的行为与通过 daemon 完全相同。

<a id="supervising-a-single-process-robot-run"></a>

## 监督单进程机器人运行

在机器人主机的仓库根目录、完整项目环境已激活时，使用 `rwrl supervise` 和 `rwrl record`。不使用 daemon 时，机器人进程通过进程内 SDK 驱动机械臂，外部无法调用 `arm.stop()`。`rwrl supervise` 将其作为独立 watchdog 进程的子进程运行：

    rwrl supervise --log runs/<run>/supervise.jsonl -- rwrl record --config hil.yaml ...

| 选项 | 默认值 | 含义 |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | 静默超过此时间，向子进程组依次发送 SIGINT、SIGTERM、SIGKILL |
| `--grace-s` | `3.0` | SIGINT 到 SIGTERM 的间隔 |
| `--kill-s` | `2.0` | SIGTERM 到 SIGKILL 的间隔 |
| `--startup-s` | `30.0` | 首条消息或此时间到达之前无需心跳 |
| `--long-op-s` | `60.0` | 子进程声明长操作（环境 reset）后允许的静默时间 |
| `--log` | 无 | jsonl：pid、带单调时间戳的升级动作、退出码/信号 |

子进程在独立会话中运行，通过 `RWRL_HEARTBEAT_FD` 接收管道写端。`real_world_rl.supervise.Heartbeat.from_env()` 为子进程侧接口：每步 `beat()`，reset 前 `begin_long_operation()`。无该变量时不做任何事。Ctrl-C、SIGTERM 或关闭终端只到达 supervisor，由它转发 SIGINT 并等待子进程；退出码沿用子进程，升级停止后为非零。若 supervisor 死亡，子进程收到 SIGINT，下一次心跳抛出 `SupervisorLost`。仅支持 Linux（prctl、进程组）。

Supervisor 只能终止进程。随后 libfranka 断开控制连接，机器人在固件层以 `Reflex` 停止。用 Desk 或 `rwrl --station robot.yaml recover` 清除（调用机械臂 `recover()`）；`rwrl doctor --fix` 仅修复主机和软件，从不恢复机械臂。受控的进程外 `stop()` 仅存在于 daemon 路径（`--watchdog-s`）。进程内第一层是 `StepWatchdog`，通过 `real_robot.watchdog_s` 启用（见 `docs/real-world-experiments.md`）。
