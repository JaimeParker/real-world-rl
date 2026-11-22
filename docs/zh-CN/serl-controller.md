<!-- English source: docs/serl-controller.md -->

<a id="serl-controller-cartesian_controller-serl"></a>

# SERL 控制器（`cartesian_controller: serl`）

rwrl 可以直接运行 SERL 自己的 Franka 控制器，也就是 SERL 与 HIL-SERL 在 franka_ros 上使用的 ros_control 插件 `serl_franka_controllers` 里的笛卡尔阻抗控制器和关节复位，而且不需要 ROS。`native/serl_controller/` 是一个 C++/pybind11 包，它的运动类派生自 franky 自己的运动类，franky 本身没有改动。黄金测试会编译 SERL 的原始源码，要求两者在每个 tick 上一致（绝对误差与相对误差都 ≤ 1e-12）。移植结果逐位一致。见 [native/serl_controller/README.md](../../native/serl_controller/README.md)。

参考版本：HIL-SERL（`rail-berkeley/hil-serl` @ `c32939b`）、SERL（`rail-berkeley/serl` @ `1fa2af7`）和 `serl_franka_controllers` @ `1f140ef`。本页讲 Franka 控制和 SpaceMouse 遥操作手感，不讲 HIL-SERL 强化学习训练的复现程度，见[与 HIL-SERL 的已知差异](#known-differences-from-hil-serl-rl-side-out-of-scope)。硬件检查清单见 [SERL 控制器：真机验证](serl-robot-validation.md)。

<a id="choosing-a-controller"></a>

## 选择控制器

SpaceMouse 怎样带动 Franka，取决于两个互不相关的设置：

- **工作站**（机器人主机）里的 `cartesian_controller`：由谁执行 Cartesian 伺服目标。可选 `impedance`（franky 的扭矩 tracker，默认）、`internal`（机器人自带控制器，通过速度运动执行）或 `serl`（SERL 的阻抗控制器）。见[设备：Cartesian 伺服控制器](devices.md#station-file-reference)。
- **preset** 里的 `spacemouse_mode`，或命令行的 `--spacemouse-mode`：怎样把摇杆输入变成目标。可选 `tracker`（默认：按 button 0 接管，得到一个持续存在、领先量有上限的指令）或 `serl`（HIL-SERL 的干预方式：推动摇杆即干预，每一步的目标是测量位姿加 action × scale）。见[遥操作：SpaceMouse](teleop.md#spacemouse)。

| `cartesian_controller` \ SpaceMouse | `tracker` | `serl` |
|---|---|---|
| `impedance` | 默认组合。franky 的 `CartesianImpedanceTracker` 按 `control:` 增益跟随 tracker 指令（`rwrl setup` 会写入近似 SERL 的增益）。非实时内核也能用。 | HIL-SERL 的干预语义，由 franky 的 tracker 执行：增益近似 SERL，但控制器不是 SERL 的。 |
| `internal` | tracker 指令交给机器人自带控制器执行（速度运动在 0.2 s 内消除误差，≤ 0.25 m/s）。需要 `realtime: enforce`。尚未在硬件上验证。 | HIL-SERL 目标每一步的跳变都变成一次速度运动。实时性要求同左。 |
| `serl` | SERL 的阻抗控制器跟随 tracker 指令。需要一起构建的 franky + `serl_controller` 和 `realtime: enforce`。 | 最接近 HIL-SERL：HIL-SERL 的干预方式、SERL 的控制器和 SERL 的关节复位。需要这一对和 `realtime: enforce`。 |

切换只改配置，不改代码：

```yaml
# station (robot PC), e.g. local/station.yaml; restart `rwrl daemon` afterwards
cartesian_controller: serl        # impedance | internal | serl
realtime: enforce                 # internal and serl want a PREEMPT_RT kernel
robot_model: fr3                  # serl: the joint limits of SERL's joint reset
# preset (actor side), or --spacemouse-mode on the command line
spacemouse_mode: serl             # tracker | serl
```

每次修改后运行 `rwrl doctor`。机械臂的 `control_metadata` 记录了 `cartesian_controller`，SpaceMouse 会话也记录了参与比较的 `teleop_control` 块，所以换了其中任何一个，再恢复（resume）已保存的实验都会被拒绝（`effective control/SDK metadata differs from the saved experiment` 或 `teleop_control` 不一致），不会在不同的动力学下悄悄继续。要改回去，把原来的值写回并重启守护进程。setup 向导替换文件前会留一份 `station.yaml.bak.<UTC>`，CLI 编辑则留 `station.yaml.<id>.bak`。

<a id="setup"></a>

## 配置

<a id="station-fields"></a>

### 工作站字段

```yaml
type: franka
arm_backend: franky                # serl needs franky
cartesian_controller: serl
realtime: enforce                  # SERL ran under franka_control's realtime_config: enforce
robot_model: panda                 # panda (default) | fr3; confirm in Desk, see below
serl: null                         # SERL compliance parameters; null = the .cfg defaults (table below)
serl_auto_recover: null            # null = {enabled: true, max_per_episode: 5}; see Auto-recovery
load: null                         # optional payload, every mode (see devices.md)
joint_limits: [...]                # still refuse move_to_joints targets outside them
```

- `robot_model`（`panda` | `fr3`，默认 `panda`）决定 SERL 关节复位软限位阶段使用 franka_description 中哪个型号的关节限位。setup 向导也会把该型号数据表上的 `joint_limits` 作为默认值：Panda 为 `[[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973], [-0.0175, 3.7525], [-2.8973, 2.8973]]`，FR3 为 `[[-2.9007, 2.9007], [-1.8361, 1.8361], [-2.9007, 2.9007], [-3.0770, -0.1169], [-2.8763, 2.8763], [0.4398, 4.6216], [-3.0508, 3.0508]]`。已有的工作站保留自己的 `joint_limits`。
- `serl` 保存 SERL 的参数（见下面的[参数](#parameters)）。这个模式下 `control:` 里 Cartesian tracker 的键不起作用；关节相关键、`max_delta_tau` 和 `gains_time_constant` 仍用于关节伺服 tracker。
- `serl_auto_recover: {enabled, max_per_episode}`：分别是布尔值和 ≥ 0 的整数，未知键会报错。

<a id="installing-franky-and-serl_controller"></a>

### 安装 franky 与 serl_controller

`serl_controller` 的运动类派生自 franky 的 C++ 类，扩展链接 franky 自己的 `libfranky.so` 以及 franky 构建时用的 libfranka，所以两者必须来自**同一次构建**，官方 franky wheel 不能和它一起用。franky 本身保持未修改（v2.0.0）。

**在机器人主机上本地构建（推荐的路径）。** Franka 的 NUC 就是普通 Ubuntu，不需要 Docker。激活项目环境，在仓库根目录运行：

```bash
LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
```

脚本用 apt 安装编译所需的包；在环境里安装 CMake ≥ 3.24、scikit-build-core 和 pybind11 3.0.4（与 franky 相同）；克隆 franky v2.0.0，核对 commit 并确认检出未修改；用 franky 自己未修改的 `docker/common/install-*` 脚本把 Eigen、pybind11、ruckig 和 `LIBFRANKA_VERSION` 版本的 libfranka（含 poco，0.14 起还有 pinocchio）装进 `/usr/local`（已有同版本 libfranka 时复用；另一版本只有设置 `REPLACE_LIBFRANKA=1` 才替换）；从源码把 franky pip 安装到环境里（版本 `2.0.0+libfranka.X`）；用 scikit-build-core（不隔离构建）对着这个 franky 构建并安装 `serl_controller`；最后运行 CTest（含两个与 SERL 原始源码对比的 golden 测试）、Python 绑定测试、导入检查和 doctor 的 `sdk:serl_controller` 检查。日志在 `~/.cache/rwrl-native-build/logs/`（`WORK=` 可改位置）。其他开关：`SKIP_APT=1`、`SKIP_DEPS=1`、`SKIP_TESTS=1`、`RETRIES=N`（网络步骤）、`PIP_INDEX_URL`。它已在干净的 Ubuntu 22.04（CPython 3.10.12、x86_64、libfranka 0.17.0）上完整跑通；Ubuntu 20.04 需要为本项目另装 Python ≥ 3.10。

`LIBFRANKA_VERSION` 由机器人服务器版本决定（从 `rwrl doctor` 的 `libfranka mismatch: robot server N, installed library M` 行读取，或在 Desk → Settings → System 读系统版本，对照 Franka 的[兼容矩阵](https://frankarobotics.github.io/docs/compatibility.html)交叉核对）。franky v2.0.0 的 `doc/compatibility.json` 给出每个服务器版本所需的最低 libfranka；建议版本是 franky 发布 wheel 时用的版本：

| 机器人服务器版本 | libfranka 范围 | 建议 `LIBFRANKA_VERSION` |
|---|---|---|
| 10 | ≥ 0.18.0 | 0.21.2 |
| 9 | 0.15.0 – 0.17.x | 0.17.0（已验证） |
| 8 | 0.14.1 – 0.14.x | 0.14.2 |
| 7 | 0.13.3 – 0.14.0 | 0.13.3 |
| 6 | 0.10.0 – 0.13.2 | 0.12.1 |
| 5 | 0.9.1 – 0.9.x | 0.9.2 |
| 4 | 0.8.0 – 0.9.0 | 0.8.0 |
| ≤ 3 | < 0.8 | 不支持（serl_controller 需要 libfranka ≥ 0.8） |

**安装顺序。** `cartesian_controller: serl` 时，这个脚本就是 franky 的安装方式：用它代替 `rwrl doctor --install-sdk` 或 `pip install franky-control`，后两者会装回官方 wheel，拆散这一对。工作站为 `cartesian_controller: serl` 时 doctor 拒绝 `--install-sdk`（`sdk.franky.serl` BLOCKED，并给出重建命令）；其他工作站装有 `serl_controller` 时，计划里会给出警告。之前装过官方 wheel 没关系，脚本会先卸载它。之后再运行 `pip install -e '.[franky]'` 会保留源码构建（它的版本满足依赖要求）。

**在别处构建 wheel（可选）。** `native/serl_controller/docker/build-on-host.sh` 在有 Docker 的构建机上构建一对 manylinux wheel（默认 CPython 3.10、libfranka 0.17.0），给多台相同的机器人主机用，见 native README。同一个构建目录的两个 wheel 一起安装：

```bash
pip uninstall -y franky-control serl-controller
W=path/to/wheels                                      # one build directory
(cd $W && sha256sum -c SHA256SUMS)
pip install $W/franky_control-*.whl $W/serl_controller-*.whl
rwrl doctor                                            # sdk:serl_controller must PASS
```

`serl_controller` 自带一个 `build_info.json`，记录 franky 的版本与 commit、链接时 `libfranky.so` 的 sha256、布局（`wheel`：franky wheel 自带的 libfranka，`libfranka_file`；`source`：franky 构建时用的系统 libfranka，`libfranka_path` 及其 sha256）和构建 id。不配对时 doctor 的 `sdk:serl_controller` 会 FAIL（构建之后系统 libfranka 被替换也会），导入时也会失败并抛出指明缺失库的 `VersionMismatch`。要换回官方 franky wheel，先把工作站改成 `impedance` 或 `internal`，再执行 `rwrl doctor --install-sdk`。

<a id="realtime-kernel"></a>

### 实时内核

SERL 的 1 kHz 扭矩循环是在强制实时下运行的。没有 PREEMPT_RT 内核时，调度抖动可能触发 libfranka 的通信 reflex。PREEMPT_RT 内核要在机器人主机上手动安装：在启用了 Ubuntu Pro 的 Ubuntu 上执行 `sudo pro enable realtime-kernel`，否则按 Franka 的 "Setting up the real-time kernel" 指南操作。然后运行 `rwrl doctor --fix` 写入 rtprio 和 memlock 限制，重新登录，再设置 `realtime: enforce`。

<a id="doctor-checks"></a>

### 诊断检查

使用 `cartesian_controller: serl`（或 `internal`）时，`rwrl doctor` 会给这台机器人多加几行检查：

| 行 | 层 | 结果 | 含义与处理 |
|---|---|---|---|
| `sdk:serl_controller` | 1 sdk | PASS `serl_controller X and franky_control Y from the same build (...)` / FAIL | 不导入任何扩展，把 `build_info.json` 和已安装的 franky 对照：版本、`libfranky.so` 的 sha256 和 libfranka（wheel 自带的文件，或源码构建所用的系统库及其 sha256）。FAIL 时重跑本地安装脚本（或从同一个构建目录重装两个 wheel）。 |
| `config:serl_control` | 1 sdk | PASS / WARN `cartesian_controller: serl ignores control: [...]` | 工作站的 `control:` 设置了 franky Cartesian tracker 的增益，而 serl 模式根本不用它们。把它们移到 `serl:`（使用 SERL 的名称）或者删掉。 |
| `mode:serl_realtime` | 5 mode | PASS / WARN `... with realtime: ignore ...` | 安装 PREEMPT_RT 内核并设置 `realtime: enforce`。 |
| `mode:internal_realtime` | 5 mode | PASS / WARN（仅 `cartesian_controller: internal`） | libfranka 的 Cartesian 速度接口在 1 kHz 下出现任何不连续都会中止。安装 PREEMPT_RT 内核并设置 `realtime: enforce`。 |
| `mode:robot_model` | 5 mode | PASS `robot_model: fr3 (robot URDF: fr3)` / WARN | 所有模式都有。把 `robot_model` 与机器人对照（见下）。WARN 会指出不一致；在 serl 模式下无法交叉核对时也会 WARN。 |

<a id="robot_model-panda-or-fr3"></a>

## robot_model：panda 还是 fr3

`mode:robot_model` 把工作站里的值和已连接机器人报告的信息对照。franky 构建里有 `Robot.server_version` 时就用它。franky 2.0 没有绑定机器人的服务器版本（只有 `Gripper.server_version`，那是夹爪自己的协议），所以这时改用机器人提供的 URDF（`Robot.model_urdf`，libfranka ≥ 0.15）里的关节名：`fr3_joint*` 表示 `fr3`，`panda_joint*` 表示 `panda`。两者都拿不到时，impedance 和 internal 模式下照常通过；serl 模式下会警告 `could not be cross-checked`，因为只有 SERL 的关节复位会用到这些限位。

服务器版本本身也能区分型号。按 Franka 的兼容性表，服务器版本 ≤ 5 对应系统 4.x（Franka Emika Robot，`panda`），≥ 6 对应系统 5.x（Franka Research 3，`fr3`）。libfranka ≥ 0.10 只能连接系统 5.x。例如握手报告服务器版本 9 的机器人属于系统 5.x，所以大概率是 `fr3`。**请在 Desk 中确认**：在 Settings → System 下查看系统版本（4.x = Franka Emika Robot，5.x = Franka Research 3）和机器人型号，然后在工作站里设置 `robot_model`（`rwrl setup --robot-model` 会在关节限位之前询问它，让数据表默认值与型号一致）。

<a id="auto-recovery-serl_auto_recover"></a>

## 自动恢复（`serl_auto_recover`）

HIL-SERL 的 `FrankaEnv` 在每次 `/pose` 前都会 POST 一次 `/clearerr`。所以一次 reflex（比如轻微碰撞）停掉 SERL 控制器后，错误会被清除，机械臂接着跟随下一个目标。rwrl 在下一个伺服目标到来时做同样的事：

1. 发现控制器带着运动错误停了。普通的 `stop()` 或被抢占的运动不算 reflex，不会自动恢复。
2. 如果 `enabled` 为真，且本 episode 自动恢复的次数还少于 `max_per_episode`，就记录日志 `arm '<name>': SERL impedance controller stopped (<error>); auto-recovery N/M this episode: clearing errors and restarting the controller at the measured pose`。然后调用 libfranka 的自动错误恢复（`recover_from_errors`，即 franka_control 的 error_recovery action 执行的操作），在测量位姿处启动一个**全新的** `SerlCartesianImpedanceMotion`，最多等 50 ms 让它跑完第一个 tick（这样 `starting()` 不会覆盖新目标），再应用这个目标。
3. 如果恢复失败（比如外部急停装置被按下），或者次数已达上限，这次伺服调用会抛出可恢复的 `DeviceFault`，消息是 `franka SERL impedance controller stopped: <error> (serl_auto_recover could not clear it)` 或 `... (serl_auto_recover: max_per_episode 5 reached, 5 auto-recoveries this episode)`。之后在 `rwrl recover` 之前拒绝伺服。

**Episode 的定义。** 两种情况会把本 episode 的计数清零：机械臂从其他模式进入 Cartesian 伺服（`set_control_mode(CARTESIAN_SERVO)`），或调用 `reset_serl_auto_recovery()`。`RealRobotEnv` 在每次 reset 结束时调用后者。`arm.serl_auto_recovery` 报告 `enabled`、`max_per_episode`、`episode_count`、`total_count` 和 `last`（错误文本、计数、时间和恢复时的 TCP 位姿），守护进程（`arm.serl_auto_recovery`、`arm.reset_serl_auto_recovery`）和 `RemoteArm` 也都提供。`control_metadata["serl"]["auto_recover"]` 记录这项策略。设置 `enabled: false` 时，每次 reflex 都算故障，与以前相同。

**与 HIL-SERL 的确切差异。** HIL-SERL 里控制器*实例*在 reflex 之后仍然存在：franka_control 完成错误恢复后，franka_control_node 重启控制循环，controller_manager 重置正在运行的控制器，也就是在同一个 `CartesianImpedanceController` 上调用 `stopping()` / `starting()`。`starting()` 只把位姿目标和零空间姿态设成当前测量值，滤波后的刚度、阻尼和零空间增益，以及积分项 `error_i` 和参数，都保留原值。rwrl 则启动一个新控制器，和 rwrl 每次启动一样，也和 SERL 自己重新 launch `impedance.launch` 一样：K、D、Ki 在约 0.2 s 内从 0 重新爬升，零空间刚度从 20 开始，`error_i` 从 0 开始。当前生效的参数会保留，包括 `update_serl_params` 的在线更新。所以自动恢复后的约 0.2 s 内，机械臂比在 HIL-SERL 里更软。另外，HIL-SERL 没有每个 episode 的次数上限。

<a id="what-runs"></a>

## 运行方式

| rwrl 调用 | SERL | 实现 |
|---|---|---|
| `set_control_mode(CARTESIAN_SERVO)` | `start_impedance`（roslaunch `impedance.launch`） | 用当前参数新建 `SerlCartesianImpedanceMotion`；`robot.move(motion, asynchronous=True, limit_rate=True, cutoff_frequency=100.0)`（franka_control 的 `rate_limiting: true`、`cutoff_frequency: 100`） |
| `servo_cartesian(pose)` | `/pose` → `equilibrium_pose` | `motion.set_target(xyz, quat_xyzw)`，基座坐标系（`O_T_EE`），下一 tick 生效 |
| `update_serl_params(partial)` | `/update_param` → dynamic_reconfigure | 部分合并，钳制到 `.cfg` 范围，下一 tick 生效（或在下次启动时） |
| `move_to_joints(q)`（IDLE） | `/jointreset`（`joint.launch`） | `SerlJointPositionMotion`：10 s 线性插值 q = (t/10)·q_target + ((10−t)/10)·q_start，然后保持；指令经过 franka_hw 的 `PositionJointSoftLimitsInterface`（见下）；在机器人内部关节阻抗控制器上运行，速率限制与截止频率相同；结束方式见下 |
| `stop()`、离开该模式 | `stop_impedance`（结束 launch 进程） | franky 的 `TorqueStopMotion`（过渡到关节阻尼后结束） |
| `move_to_pose` | （无；SERL 用阻抗控制器移动） | Ruckig `CartesianMotion`，不变 |
| 关节伺服 | （无） | franky 的 `JointImpedanceTracker`，不变 |

关节复位的结束流程照搬 `franka_server.reset_joint`：等待 2 s，然后每秒检查一次关节，直到 `np.allclose(target - q, 0, atol=1e-2, rtol=1e-2)` 或再检查 30 次。超时只记录日志（"SERL joint reset TIMEOUT"），复位照常完成，与 SERL 相同。随后在 10 s 插值结束后以 `MotionFinished` 结束运动（保持在目标；SERL 是直接结束控制器进程，不管处在哪里），清除错误，并在 `arm.last_joint_reset` 记录 `target`、`reached`、`polls`、`max_abs_error_rad`、`duration_s`、`soft_limited_ticks` 和 `robot_model`（守护进程同样提供：`arm.last_joint_reset`、`RemoteArm.last_joint_reset`）。与 franka_server 一样，阻抗控制器由下一次 `set_control_mode(CARTESIAN_SERVO)` 重新启动。复位过程中出现 reflex 会抛出可恢复的 `DeviceFault`。`blocking=False` 启动插值后立即返回，保持到下一次 `stop()` 或模式切换。通过 `RemoteArm` 在 `cartesian_controller: serl` 的守护进程上阻塞调用 `move_to_joints` 时，客户端最多等待 60 s（`SERL_JOINT_RESET_DEADLINE_S`）；其他调用仍用 `timeout_s`。

<a id="soft-joint-limits-franka_hws-positionjointsoftlimitsinterface"></a>

### 软关节限位（franka_hw 的 `PositionJointSoftLimitsInterface`）

franka_hw 把每个关节位置指令交给 ros_control 的 `joint_limits_interface::PositionJointSoftLimitsHandle`，参数来自 franka_control 加载的 URDF（franka_description `robots/<robot_model>/joint_limits.yaml` 经 `franka-limits` 宏生成：`<limit lower upper velocity>`、`<safety_controller k_position="100" soft_lower_limit=lower soft_upper_limit=upper>`）。关节复位运动在指令链路的同一位置复现它：周期为 0 的那个 tick 重置限位（指令为起始姿态），其余每个 tick 在控制器 update 之后执行。第一次执行时以测量位置为起点，之后以上一次发出的指令为起点，每个关节的指令被裁剪到 `[pos + v_min·dt, pos + v_max·dt] ∩ [lower, upper]`，其中 `v = clamp(−k_position·(pos − soft_limit), ±max_velocity)`。因此插值不会让关节超过 URDF 速度上限，接近限位时以 k_position 100 /s 指数减速并停在限位上，从限位外开始的关节会被拉回。该移植逐行对照原始 ros_control 头文件并通过黄金测试（见原生包 README）。由工作站的 `robot_model` 选择限位表（默认 `panda`，即 franka_control 的默认 `robot:=panda`，或 `fr3`；见 [robot_model](#robot_model-panda-or-fr3)）。工作站 `joint_limits` 仍会在运动开始前拒绝超限目标。

若 reflex 终止了笛卡尔控制器，下一个伺服目标会清除错误并重启控制器，每个 episode 最多 `serl_auto_recover.max_per_episode` 次（见[自动恢复](#auto-recovery-serl_auto_recover)）；超过次数或关闭自动恢复时，下一次伺服调用抛出可恢复的 `DeviceFault`，在 `recover()`（`rwrl recover`）之前拒绝伺服，之后下一个目标会启动一个新的控制器，与 SERL 重新 launch 相同。每次启动都是新控制器（刚度和阻尼在约 0.2 s 内从 0 爬升，`filter_params = 0.005`），与 SERL 一致。

<a id="parameters"></a>

## 参数

工作站 `serl:` 块是这台机器人的默认值，名称、默认值和范围与 SERL 的 `cfg/compliance_param.cfg` 相同（`real_world_rl.devices.control.SERL_COMPLIANCE_PARAMS`）；省略的键保留 `.cfg` 默认值：

| 键 | 默认值 | 范围 |
|---|---|---|
| `translational_stiffness` | 2000 | 0–6000 |
| `translational_damping` | 89 | 0–400 |
| `rotational_stiffness` | 150 | 0–300 |
| `rotational_damping` | 7 | 0–30 |
| `nullspace_stiffness` | 0.2 | 0–100 |
| `joint1_nullspace_stiffness` | 100 | 0–100 |
| `translational_clip_{x,y,z}`、`translational_clip_neg_{x,y,z}` | 0.01 | 0–0.1 |
| `rotational_clip_{x,y,z}`、`rotational_clip_neg_{x,y,z}` | 0.05 | 0–0.1 |
| `translational_Ki`、`rotational_Ki` | 0 | 0–100 |

平移误差逐轴裁剪到 `[-clip_neg, clip]`，姿态误差为四元数虚部（sin(θ/2)·axis）。超出范围的值会被**钳制**，与 dynamic_reconfigure 相同（HIL-SERL 依赖这一点：它的 `PRECISION_PARAM` 发送 0.5 的旋转限幅，实际按 0.1 运行）；被钳制的键会写入日志并列在 `control_metadata["serl"]["clamped"]`。未知键和非有限值报错。

`arm.update_serl_params({...})` 在控制器运行时修改参数（未运行时保存到下次启动）；更新会一直保留，直到再次修改，与 ROS 参数服务器上的 dynamic_reconfigure 值相同。`arm.serl_params` 是完整的生效参数。两者都可通过守护进程（`arm.update_serl_params`、`arm.serl_params`）和 `RemoteArm` 使用。`control_metadata["serl"]` 报告 `serl_controller` 的版本与构建、工作站默认值（`station_params`）、当前 `params`、`clamped`、`limit_rate` 和 `cutoff_frequency`。该模式下 `control:` 块中 Cartesian tracker 的键（`translational_stiffness`、`rotational_stiffness`、`cartesian_damping`、`translational_error_clip`、`rotational_error_clip`）被忽略；加载工作站时会记录一条列出这些键的警告，`rwrl doctor` 的 `config:serl_control` 报 WARN。关节相关键、`max_delta_tau` 和 `gains_time_constant` 仍用于关节伺服 tracker。

<a id="profiles"></a>

### 参数组（profile）

任务 preset 可在 `real_robot.serl_profiles` 下命名参数组。与 HIL-SERL（`requests.post(url + "update_param", json=PROFILE)`）一样，切换到某个参数组是**在当前参数之上的部分更新**：参数组未设置的键保留之前的值。`real_world_rl.devices.control.serl_profile_update(profiles, name)` 给出要传给 `arm.update_serl_params` 的更新，`apply_serl_profile(current, profiles, name)` 计算结果。若各参数组设置的键不同，校验时会记录警告：此时结果取决于切换顺序。以 HIL-SERL 的 `ram_insertion` 为例：

```yaml
real_robot:
  serl_profiles:
    compliance:               # HIL-SERL ram_insertion COMPLIANCE_PARAM (the policy's episodes)
      translational_stiffness: 2000
      translational_damping: 89
      rotational_stiffness: 150
      rotational_damping: 7
      translational_Ki: 0
      translational_clip_x: 0.0075
      translational_clip_y: 0.0016
      translational_clip_z: 0.0055
      translational_clip_neg_x: 0.002
      translational_clip_neg_y: 0.0016
      translational_clip_neg_z: 0.005
      rotational_clip_x: 0.01
      rotational_clip_y: 0.025
      rotational_clip_z: 0.005
      rotational_clip_neg_x: 0.01
      rotational_clip_neg_y: 0.025
      rotational_clip_neg_z: 0.005
      rotational_Ki: 0
    precision:                # HIL-SERL ram_insertion PRECISION_PARAM (resets)
      translational_stiffness: 2000
      translational_damping: 89
      rotational_stiffness: 250
      rotational_damping: 9
      translational_Ki: 0.0
      translational_clip_x: 0.1
      translational_clip_y: 0.1
      translational_clip_z: 0.1
      translational_clip_neg_x: 0.1
      translational_clip_neg_y: 0.1
      translational_clip_neg_z: 0.1
      rotational_clip_x: 0.5    # runs as 0.1 (clamped, as in HIL-SERL)
      rotational_clip_y: 0.5
      rotational_clip_z: 0.5
      rotational_clip_neg_x: 0.5
      rotational_clip_neg_y: 0.5
      rotational_clip_neg_z: 0.5
      rotational_Ki: 0.0
```

这些限幅是为 HIL-SERL 的 RAM 插入装置调的，只用来示范格式，不是其他任务可直接用的数值。preset 加载时校验参数组；在 reset 时切换参数组（HIL-SERL 在 reset 前发送 `PRECISION_PARAM`，之后发送 `COMPLIANCE_PARAM`）由环境负责；`RealRobotEnv` 自己不会切换（只把它们记录在元数据里）。上面两个参数组设置的键相同，因此对它们而言切换顺序无关。


<a id="deliberate-differences-from-serl"></a>

## 有意与 SERL 不同之处

停止是平滑的（franky 的 `TorqueStopMotion`；关节复位在保持阶段结束），而不是直接结束 ROS 进程。`set_target` 拒绝非有限位姿和非单位四元数。自动恢复启动新控制器，而不是在旧控制器上调用 `starting()`（见上）。硬件验证见 [SERL 控制器：真机验证](serl-robot-validation.md)。

<a id="spacemouse-in-serl-mode-the-task-frame"></a>

## serl 模式下的 SpaceMouse：任务坐标系

`--spacemouse-mode serl` 复现 HIL-SERL 的 `SpaceMouseExpert` + `SpacemouseIntervention`。HIL-SERL 不做任何操作员旋转，设备的 twist 直接以机器人**基座坐标系**进入环境，这默认机器人是正装的。rwrl 在每条 SpaceMouse 路径上，都在生成末端目标之前把 twist（平移和旋转用同一个矩阵）转到基座系；三种控制器都不再旋转：

| 模式 | 转到基座系用的旋转 | `teleop_control` 的 `input_frame` |
|---|---|---|
| serl（默认） | 只用工作站 `task_frame` 的旋转 `base_R_table`（不加档案 yaw） | `task_frame` |
| serl，`use_profile_rotation: true` | 档案的 `input_rotation`（`base_R_table · Rz(yaw)`） | `profile` |
| serl，没有 `task_frame` | 不旋转（原始基座系，同 HIL-SERL） | `base` |
| tracker，选了档案 | 档案的 `input_rotation` | （档案） |
| tracker，没有档案、有 `task_frame` | 只用 `base_R_table` | `task_frame` |
| tracker，两者都没有 | 不旋转 | （基座） |

正装且没有 `task_frame` 时，这与 HIL-SERL 完全相同。基座侧装（例如绕自身 x 轴转了约 45°，`base_R_table = Rx(±45°)`）时，原始基座系会让“向上推”沿倾斜的基座 z 轴走；有了测得的 `task_frame`（`rwrl workspace frame set --from-points`，见工作站教程），“上”又是竖直向上。策略动作本来就是基座系增量，从不旋转。

**摆放（不需要标定）。** 按 HIL-SERL 的方式摆放 SpaceMouse：前方朝任务系 +x（桌面前方、离开底座），左方朝 +y，上为 +z（竖直）。然后检查六个单轴：每次推动让 TCP 沿同一任务系轴移动，每次扭转让它绕同一轴转动。`rwrl workspace capture` 给出 TCP 在任务系中的位置（`task_position`），在单轴推动前后各读一次。SpaceMouse 不能这样摆时，设置 `use_profile_rotation: true`，让标定的档案 yaw 也参与：

```yaml
# teleop_options_yaml (or the station's teleop.options)
input_rotation: [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
use_profile_rotation: true        # serl mode only; default false (task_frame rotation only)
```

`use_profile_rotation` 只能是 `true` 或 `false`，而且只属于 serl 模式：tracker 模式总是应用档案旋转，遇到 `use_profile_rotation: false` 会报错。参与比较的 `teleop_control` 块记录这个选择和输入坐标系，所以用另一种坐标系录制的运行在恢复时会被拒绝（`teleop_control` 不一致），而不会悄悄换掉驱动坐标系。想按旧行为继续，就用记录这次运行的代码恢复，或者开一个新运行。

serl 模式下，每一步的目标是测量位姿加 action × `xyz_scale_m` / `rot_scale_rad`（例如 0.02 m 和 0.1 rad，即 SERL 插销任务的值）。没有 tracker，也没有领先量上限。配合 `cartesian_controller: serl` 时，摇杆推满相当于每步向 SERL 控制器要求 2 cm，由 SERL 的 `translational_clip_*`（默认 0.01 m）限制这个误差产生的力。

<a id="known-differences-from-hil-serl-rl-side-out-of-scope"></a>

## 与 HIL-SERL 的已知差异（强化学习侧，不在本页范围内）

下面这些属于 HIL-SERL 的训练栈，与控制器或遥操作手感无关。列出来是为了避免有人把控制器移植误当成完整复现了 HIL-SERL。这里一项都没有改。

- **RelativeFrame 的动作变换。** HIL-SERL @ `c32939b` 的 `RelativeFrame` 用的是 `construct_transform_matrix`，即 `blockdiag(R, R)`；SERL @ `1fa2af7` 和 RLinf 用的是左下块为 `[p]×R` 的伴随矩阵（`construct_adjoint_matrix`）。rwrl 的 RLinf 路径（`real_world_rl/envs/rlinf_cartesian.py`）保留 SERL/RLinf 的伴随矩阵。因此 TCP 不在基座原点时，同一个末端坐标系动作映射成的基座 twist 与 HIL-SERL 不同。
- **夹爪惩罚。** HIL-SERL 的 `GripperPenaltyWrapper` 在执行的夹爪动作会改变测量到的夹爪状态时（`gripper_pos > 0.95` 视为张开），从奖励里减去 `penalty = 0.1`。rwrl 的 `GraspPenaltyWrapper`（`envs/wrappers/grasp_penalty.py`）根据执行动作的切换，单独报告 −0.05 的 `grasp_penalty`。大小、触发条件和惩罚落在哪里都不一样。
- **欧拉角观测。** HIL-SERL 用一个 wrapper（`Quat2EulerWrapper`）把相对 `tcp_pose` 四元数转成 `xyz` 欧拉角；rwrl 的 RLinf 路径自己计算 `relative_rotation.as_euler("xyz")`。除了轴顺序，其余约定没有逐项比较。
- **超时。** HIL-SERL 的 `FrankaEnv.step` 把 `done = path_length >= max_episode_length or reward or terminate` 作为 *terminated* 返回（truncated 始终为 False）；`RealRobotEnv` 把时间上限报告为 **truncated**，操作员结束 episode 也是 truncated。因此超时时的 bootstrap 行为不同。
- **reset 时切换参数组。** HIL-SERL 的 `go_to_reset` 先 POST `PRECISION_PARAM`，可选执行 `/jointreset`，插值移动到 reset 位姿，再 POST `COMPLIANCE_PARAM`，`reset()` 还会再 POST 一次 `COMPLIANCE_PARAM`。rwrl 会校验 `real_robot.serl_profiles`，并提供 `serl_profile_update` / `arm.update_serl_params`，但 `RealRobotEnv` 在 reset 时不切换参数组。
- **`euler_2_quat` 的俯仰角符号。** HIL-SERL 的 `utils/rotations.euler_2_quat`（用于 `RESET_POSE`、`interpolate_move` 的目标和随机 reset）构造一个 pyquaternion 并返回它的 `[w, x, y, z]` 元素，而 `FrankaEnv` 把它当作 `[x, y, z, w]` 读取。对常见的"夹爪朝下"位姿（`[π, pitch, roll]`），往返 `quat_2_euler(euler_2_quat(e))` 得到的俯仰角符号相反，所以 HIL-SERL 的 reset 姿态在俯仰方向是镜像的。rwrl 不复现这一点。
- **`action_indices`。** HIL-SERL 的 `SpacemouseIntervention(action_indices=...)` 只保留列出的专家动作维度，其余清零。rwrl 的 serl 模式六个轴全部通过。`diagnostic_axes` 只用于调试，不属于训练约定。
