<!-- English source: docs/teleop.md -->

<a id="teleoperation"></a>

# 遥操作

`real_world_rl` 的人在环介入：`TeleopSource` 读取一个操作员设备，`InterventionWrapper` 用操作员动作替换策略动作，`examples/` 中的示范脚本使用相同输入源。

<a id="the-teleopsource-contract"></a>

## `TeleopSource` 接口约定

`real_world_rl.teleop`（`teleop/base.py`）：

```python
@dataclass(frozen=True)
class TeleopReading:
    action: np.ndarray          # float32 (A,), env-normalized [-1, 1], gripper last when has_gripper
    intervened: bool            # the operator is driving; action only matters then
    episode_end: bool = False   # operator's episode-end button
    events: Mapping[str, bool] = {}   # success / failure / rerecord edges that fired, else empty
    timestamp: float = 0.0
    human_action_mask: Any = None  # actual human-controlled dimensions
    override_mask: Any = None      # overridden dimensions including hold supplements

class TeleopSource(ABC):
    action_mode: str            # "cartesian_delta" | "joint_delta" | "joint_abs"
    arm_action_dim: int
    has_gripper: bool
    def start(self) -> None     # open the device (default no-op)
    def poll(self) -> TeleopReading            # abstract
    def reset(self) -> None     # after an env reset: drop latched events and sticky state
    def close(self) -> None     # release the device (safe to call twice)
    @classmethod
    def from_env(cls, env, **kwargs)           # registry factory; default cls(**kwargs)
```

- 构建包装器时，将 `action_mode`、`arm_action_dim` 和 `has_gripper` 与环境动作契约比较；不匹配（例如在 `joint_delta` 机器人上使用笛卡尔 SpaceMouse）会在设备启动前抛出 `ValueError`。
- `poll()` 必须让设备失败向上传播：吞掉错误会把策略动作记录成操作员示范。
- `as_teleop_source(obj)` 原样返回 `TeleopSource`；对鸭子类型的旧对象，使用 `LegacyTeleopAdapter` 包装（`poll()` 返回带 `.intervened`、`.action`、`.episode_end` 的对象；可选 `reset()`、`close()` 和 `action_mode`/`arm_action_dim`/`has_gripper`，默认 cartesian_delta / 6 / True）。

<a id="devices"></a>

## 设备

`TeleopInterventionWrapper(env, device=..., **kwargs)` 通过 `make_teleop_source(device, env, **kwargs)` 构建输入源；hil_serl 中设备来自 `--teleop-device`。

hil_serl（以及 residual、SiLRI）从参数派生输入源 kwargs（`robot`/`leader_arm`：机器人部件声明和限位；`gello`/`leader_arm`：起始参数），再叠加 `--teleop-options-yaml PATH`：一个覆盖派生值的输入源 kwargs YAML 映射。对于 CLI 没有标志的设备选项使用它，例如本地 GELLO：

```yaml
# gello.yaml, for --teleop-device gello --teleop-options-yaml gello.yaml
port: /dev/serial/by-id/usb-FTDI_...
joint_ids: [1, 2, 3, 4, 5, 6, 7]
joint_offsets_rad: [...]   # from gello_calibrate
joint_signs: [1, 1, -1, 1, 1, 1, 1]
gripper_id: 8
gripper_open_rad: ...
gripper_closed_rad: ...
hold_s: 0.0
```

第二个 SpaceMouse 可用 `device_index: 1`（以及 `deadzone`、`invert_axes`）；或者，在 `franka_real` 栈的 `--teleop-device leader_cartesian` 中指定提供主臂的机器人 daemon（见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)）：

```yaml
# leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
remote_address: <robot-host>:<port>
```

使用 `--teleop-event-device` 时，该文件提供给组合源的运动部分。

包装器将“最后发送的夹爪动作”传给 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`bimanual`（以及基于它们构建的组合源运动部分），因此不带夹爪的 leader 部件会保持夹爪而不是掉落已抓物体，SpaceMouse 每次接管也从夹爪当前命令开始。环境动作末尾每臂有一个夹爪维度（动作契约布局），所以 getter 每臂返回一个条目，单臂源读取第 0 项（`bimanual` 将各臂自己的条目交给对应源）；输入源默认驱动一条臂，除非声明 `num_arms`。臂数量与环境不同，或声明的 `arm_names` 与环境及其顺序不同，包装器都会拒绝。

| `device` | 输入源 | 读取内容 | 动作空间 | 备注 |
| --- | --- | --- | --- | --- |
| `pico` | `PicoTeleopSource` | 通过 ZeroMQ `SUB` socket 的 Pico VR 控制器（`EETwistTeleOpWrapper`） | cartesian_delta，6 + 夹爪 | kwargs：`zmq_url`、`hand`、`pos_scale`、`rot_scale`、`twist_limit`、`intervention_threshold`。数据格式见下文。 |
| `spacemouse` | `SpaceMouseTeleopSource` | 本机 SpaceMouse，经设备层 `SpaceMouseTeleop`，作为笛卡尔命令 tracker 上的速度输入 | cartesian_delta，6 + 夹爪 | 按钮 0 接管/交还，按钮 1 切换夹爪；见 [SpaceMouse](#spacemouse)。kwargs：`device_index=0`、`deadzone=0.05`、`invert_axes=()`、`poll_interval_s=0.005`、`read_timeout_s`，以及 tracker 参数（`lead_cap_m`、`lead_cap_rad`、`target_speed_m_s`、`target_speed_rad_s`：`--teleop-lead-cap-*`/`--teleop-target-speed-*` 标志）。需要 `hidapi==0.14.0.post4`（导入 `hid`），在 `start()` 导入。 |
| `spacemouse_twist` | `SpaceMouseTwistTeleopSource` | 同一 SpaceMouse 的原始 twist（仅 ManiSkill 模拟示例，不用于真实机器人） | cartesian_delta，6 + 夹爪 | 手柄 twist 原样传给环境，移动即介入（`hold_s` 延长），按钮 0 切换锁存夹爪（每次 reset 后打开），按钮 1 结束回合。kwargs：`device_index=0`、`deadzone=1e-3`、`invert_axes=()`、`poll_interval_s=0.005`、`hold_s=0.0`、`read_timeout_s`。 |
| `robot` | `TeleopPartSource` | `real_robot` 环境后方机器人的 teleop 部件（robot YAML `teleop:` 段，本地或 daemon 提供） | cartesian_delta、joint_abs 或 joint_delta，映射到环境空间 | kwargs：`teleop_contract`、`joint_limits`、`joint_scale_rad`、`read_timeout_s`（hil_serl：`--robot-teleop-action-mode`、`--no-robot-teleop-has-gripper`）。机器人持有此部件，`close()` 保持其连接。 |
| `fake` | `ScriptedTeleopSource` | 脚本读数（用于测试；没有 `readings` 时为永不介入的操作员） | 遵循环境动作契约，包括臂数量 | 传 `readings=[TeleopReading, ...]` 编排；最后一个读数重复。 |
| `gello` | `GelloTeleopSource` | 本机 GELLO，经设备层 `gello` teleop（Dynamixel 总线） | joint_abs 或 joint_delta（随环境），leader 关节按 `robot` 方式映射 | kwargs：设备选项（`port`、`joint_ids`、`joint_offsets_rad`、`joint_signs` 等，见[主臂](#leader-arms-gello)）、`hold_s`、`read_timeout_s`、`onset`、`arm_tolerance_rad`、`ramp_step_rad`。需要 `dynamixel-sdk`（robot_infra extra `gello`），在 `start()` 导入。 |
| `leader_arm` | `LeaderArmPartSource` | `real_robot` 环境后方机器人的 teleop 部件，作为关节空间主臂读取（机器人 daemon 提供 GELLO） | joint_abs 或 joint_delta（随环境） | kwargs：`robot` 的选项加 `onset`、`arm_tolerance_rad`、`ramp_step_rad`（hil_serl：`--teleop-onset`、`--teleop-arm-tolerance-rad`、`--teleop-ramp-step-rad`）。 |
| `keyboard` | `KeyboardTeleopSource` | 通过 `pynput` 读取本机键盘 | 遵循环境动作契约；动作始终全零 | `t` 接管，`h` 交还，Esc 结束回合，`s`/`f`/`r` 产生 `success`/`failure`/`rerecord`，`q` 结束回合并产生 `operator_quit`（录制器和 actor 循环在该回合后停止）。这是事件设备，单独使用仅适合纯事件测试。需要 `pynput`（extra `reward-collection`）及桌面会话，在 `start()` 导入。 |
| `stdin` | `StdinTeleopSource` | 运行进程的终端（cbreak 模式，`close()` 恢复；支持 SSH，不需桌面会话） | 遵循环境动作契约；动作始终全零 | 与 `keyboard` 相同按键。kwargs：`fd`（默认 stdin）、`poll_interval_s=0.02`。输入结束在下次轮询抛出 `DeviceFault`。 |
| `composite` | `CompositeTeleopSource` | 一个运动源加一个事件源 | 随运动源 | kwargs：`motion`、`events`（注册名）、`motion_kwargs`、`events_kwargs`、`intervention_from="events"`（或 `"motion"`）。hil_serl 根据 `--teleop-event-device` 构建。 |
| `leader_cartesian` | `LeaderArmCartesianSource` | 经 Panda 正运动学将关节空间主臂转换为笛卡尔 twist（`franka_real` 栈） | cartesian_delta，6 + 夹爪 | 见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)。 |
| `bimanual` | `BimanualTeleopSource` | `real_robot_bimanual` 环境的每条臂各一个单臂源（`--teleop-arm-device`） | 随环境，先双臂再双夹爪 | kwargs：`device_per_arm`、`per_arm_kwargs`、`common_kwargs`、`intervention_from="any"`（或 `"all"`）。见[双臂](#two-arms-bimanual)。 |

<a id="spacemouse"></a>

### SpaceMouse

`--teleop-device spacemouse` 支持两种模式，用 `--spacemouse-mode {tracker,serl}` 选择（顶层 YAML `spacemouse_mode`，默认 `tracker`）。训练、示范录制和真机奖励采集共用此标志。部署的可选覆盖值默认为 `None`：继承训练模式，没有训练配置时用 `tracker`。

| 行为 | `tracker`（默认） | `serl` |
| --- | --- | --- |
| 接管 | 按钮 0 切换接管/交还；单纯移动手柄不介入。 | 原始六轴 L2 范数严格 `> 0.001` 时介入，包括旋转。无逐轴死区。每步重新判断；释放手柄和夹爪按钮后，下一个空闲步交还控制。 |
| 夹爪 | 驱动时按钮 1 切换开/关；接管保留最后命令。 | 按住按钮 0 关闭：每步重新均匀采样 `[-1, -0.9)`。否则按住按钮 1 打开：`uniform(0.9, 1)`。同时按住时按钮 0 优先。均未按下时中立 `0.0`。只有环境带夹爪动作时按钮才引起接管。 |
| 机械臂目标 | 持久命令，带目标速度和实测位姿领先限制。 | 每步最新实测位姿 + 原始动作 × 环境尺度；旋转左乘实测四元数。同时裁剪 xyz 和 RPY 安全盒（roll 按幅值裁剪、保留符号）。无时间步重缩放或 tracker 目标。 |
| 夹爪执行 | 现有环境行为。 | 缩放后动作 `<= -0.5` 且实测归一化宽度 `> 0.85` 才关闭；`>= 0.5` 且宽度 `< 0.85` 才打开。命令要求已过时间严格大于 `gripper_sleep_s`（默认 `0.6 s`），随后休眠该时长。零为中立。 |
| 终端/键盘 | 忽略 `t`/`h`，状态变化时警告；标签/结束/退出仍有效。 | 同样由 `s`/`f`/`r`/Esc/`q` 提供标签/结束/退出。`t`/`h` 不控制接管，状态变化时警告。 |

倾斜机器人基座可通过 `teleop_options_yaml` 提供 `input_rotation`，即有限、有效的 3×3 旋转矩阵 `base_R_input`。两种 SpaceMouse 模式均将平移和角输入旋转到机器人基坐标系。轴翻转及 tracker 逐轴死区在旋转前执行；SERL 接管仍使用原始六轴范数。旋转后的每个三维向量仅在需要适配动作立方体时统一缩小，保持方向。不应用原点平移。持久化解析后配置前，解析矩阵和标定档案（profile）名称/文件/yaw 保存到不参与比较的 `teleop_input` 溯源。恢复运行使用这些数值输入选项，不重新读取已改变的档案。记录动作在基坐标系中，因此操作员重标定不使旧示范或恢复契约失效。旧安全盒仍在机器人基坐标系。可选工作站 `task_frame`（任务坐标系，task frame）和 `workspace` 增加桌面约束，与原有限制取交集；仅输入旋转不会移动安全边界。两者都不改变机器人安装/重力配置或负载模型。

SpaceMouse tracker 在发送请求目标前及速度/领先限制后检查任务工作区。候选被拒绝时交还控制，并将录制器保持锚定到实测位姿；按钮 0 可再次接管。训练中交还会通过相同设备约束恢复策略。策略目标在任务坐标系中逐分量裁剪；超过 `max_angle_rad` 的旋转通过缩放相对参考的旋转向量投影到其 SO(3) 锥。SERL 先执行原 XYZ/RPY 盒裁剪，再执行工作区投影。实测 TCP 反馈超出工作区及反馈裕量时锁存故障。裕量默认 `measured_margin_m: 0.003`、`measured_margin_rad: 0.03`；已接受目标仍使用 `1e-9` 容差。两种遥操作模式和策略动作都以设备及环境约束为准。

具名操作员标定档案使用 [CLI 标定流程](spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)。平移和角向量都使用组合输入旋转 `base_R_task @ Rz(operator_yaw)`。固定 HID 轴映射是设备约定，不是实测安装变换。不要向已标定的 `base_R_task` 再添加 roll 修正。绝对位置使用包括原点的完整任务变换；反馈观测和策略动作保留现有基坐标系契约。

SERL 需先配置工作站 **xyz 和 RPY 安全盒**，缺少任一都会构建报错。`real_robot` 中为 `safety_box`（`[xmin, ymin, zmin, xmax, ymax, zmax]`）、`safety_box_rpy_low`、`safety_box_rpy_high`；尺度仍为 `xyz_scale_m`/`rot_scale_rad`。`franka_real` 中将 `safety_box_low`、`safety_box_high` 和两个 RPY 限制放入 `franka_real.env_kwargs_json`；现有位置/旋转尺度仍适用。该模式在环境构建前选择 `cartesian_semantics: serl`，即使是不打开遥操作设备的 learner。显式配置的 `current` 与此模式冲突；程序调用中未设置的 dataclass 默认值提升为 `serl`。模式解析在训练/录制参数序列化前完成，因此保存的 SERL 配置包含 `spacemouse_mode: serl` 和 `cartesian_semantics: serl`。部署保留已保存环境语义；`--spacemouse-mode serl` 覆盖值与已保存 `cartesian_semantics: current` 冲突。

录制、训练和部署使用相同模式。SERL 记录以下参与比较的 `teleop_control` 契约（非默认 `invert_axes` 会增加该列表）：

```json
{"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, "gripper": "serl_buttons"}
```

SERL 要求单条六轴 `cartesian_delta` 臂，并完整选择机械臂/夹爪。固定 HID 映射 `[-y, x, z, -roll, -pitch, -yaw]` 及配置的输入旋转后，动作仍在基坐标系；不支持相对坐标系、相对观测和 `action_indices`。`teleop_options_yaml` 可设 `device_index`（默认 `0`）、`invert_axes`（默认 `[]`）、`poll_interval_s`（默认 `0.005`）、`read_timeout_s`（默认 `1.0`）。此模式拒绝 tracker 选项、`deadzone`、`hold_s`、夹爪保持 getter 及嵌套 `mode` 键，应使用顶层模式标志。设备层 `max_delta_xyz_m` 和位姿盒限制仍适用，可能比原生 HIL-SERL 更严格。硬件行为和安装仍**未验证**。`spacemouse_twist` 仍为独立的仅模拟输入源，按钮和锁存夹爪行为不同；它不是 SERL 模式。

<a id="spacemouse-installation-and-linux-permissions"></a>

#### SpaceMouse 安装与 Linux 权限

在 NUC（或连接 SpaceMouse 的机器）的仓库根目录，激活[教程第 4 步的项目环境](tutorial/01-install.md#4-install-the-project-environment)后，使用以下命令安装全部 SpaceMouse Python 依赖：

```bash
pip install -e '.[spacemouse]'
```

依赖链为根目录 `spacemouse` extra → `rlgarden-robot-infra[spacemouse]==0.0.1` → `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`）。设备层包含自己的解码器；此路线不导入 `pyspacemouse` 或 `easyhid`。不要安装另一个名为 `hid` 的 PyPI 发行包，它占用相同导入名但 API 不同。模块缺失时抛出 `SdkMissing` 并给出上述 pip 命令；安装了错误的 `hid` 发行包会单独报告。

新建环境时，在 NUC 的仓库根目录、未激活任何环境的情况下运行：

```bash
bash scripts/install.sh --extras franka-gello,spacemouse
source .venv/bin/activate
```

单个 extra 的 pip 命令假定三个本地项目已经安装，不会从空 venv 引导安装内部发行包。wheel 目标为 Ubuntu 22.04 / CPython 3.10 / x86_64。此传输方式避开 easyhid 对系统 `libhidapi` 的查找；固定版本 wheel 的 ELF/系统库依赖仍需在干净 Ubuntu 22.04 机器验证。wheel 安装、USB 权限、设备方向、时序和断连行为均**尚未在真机验证**。此 wheel 路线不要求安装系统 `libhidapi`；没有合适 wheel 的平台上从源码构建不属于该目标范围。

一次性安装随包提供的厂商规则。在 NUC 的仓库根目录、任意 shell（无需项目环境）、具有 sudo 权限时运行：

```bash
sudo cp robot_infra/robot_infra/setup/99-spacemouse.rules /etc/udev/rules.d/
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw
```

该文件涵盖厂商 `256f`（3Dconnexion）和 `046d`（旧 Logitech），使用 `MODE="0664", GROUP="plugdev"`。注销并重新登录，再拔插 SpaceMouse。若仓库不可用，以下替代复制命令可在 NUC 任意目录运行，要求**项目环境已激活**且有 sudo 权限；随后执行上述相同的加组/重载命令：

```bash
sudo cp "$(python -c 'import robot_infra.setup, os; print(os.path.dirname(robot_infra.setup.__file__))')/99-spacemouse.rules" /etc/udev/rules.d/
```

打开/权限失败会指向此规则文件及安装命令。doctor 将 `hid` SDK 与 `/dev/hidraw*` 访问分别检查；Docker 内还需将 HID 节点传入容器。旧 `SpaceMouseExpert` 和 `real` extra 保留各自的 `pyspacemouse` API。

在 NUC 的仓库根目录激活项目环境并配置好工位 `hil.yaml`，对于 `cartesian_delta` 环境（`cartesian_delta` 模式的 `real_robot` 或 `franka_real`），由终端提供操作员标签（SpaceMouse Compact 的两个按钮均已占用）：

```bash
rwrl record --config hil.yaml \
    --teleop-device spacemouse --teleop-event-device stdin \
    --output-path demos/task_20.pkl --num-successes 20
```

HIL actor 使用相同两个标志。配置两个安全盒后，添加 `--spacemouse-mode serl` 可选择上表原生行为。以下描述默认 **tracker** 模式。`SpaceMouseTeleopSource`（`real_world_rl/teleop/spacemouse.py`）通过设备层 `SpaceMouseTeleop` 打开设备（`TeleopConfig(backend="spacemouse")`，轮询线程向 `read()` 提供数据；HID 读取器状态按 `[-y, x, z, -roll, -pitch, -yaw]` 映射为 `[-1, 1]` 的 twist，`invert_axes` 翻转所列输出索引 0–5 的符号），并驱动与 `leader_cartesian` 相同的 `CartesianTargetTracker`（见[映射](#mapping)）：

| 输入 | 作用 |
| --- | --- |
| 按钮 0 | 接管，再按交还。单纯移动手柄不介入，因此轻推不会接管。 |
| 按钮 1 | 驱动时切换夹爪打开（`+1`）/关闭（`-1`） |
| 手柄 | 驱动时输入速度（见下文） |
| `--teleop-event-device stdin`（或 `keyboard`）的 `s`/`f`/`r`、Esc、`q` | `success`/`failure`/`rerecord`、回合结束、退出 |

- **速度。**每个介入步请求 tracker 命令移动 `twist * target_speed * dt`（`dt` 为距上次介入步的时间，接管时为 0，因此接管不跳变）：平移为基坐标系的 `twist[:3] * --teleop-target-speed-m-s * dt`，旋转向量 `twist[3:] * --teleop-target-speed-rad-s * dt` 与环境 step 一样左乘。满幅为目标速度（0.05 m/s、0.3 rad/s），tracker 也将对角速度限制在此值。各轴处于 `deadzone` 内（满幅 0.05，高于 Compact 静止噪声）视为零。
- **领先上限。**命令领先实测 TCP 不超过 `--teleop-lead-cap-m`/`--teleop-lead-cap-rad`（0.05 m、0.3 rad），多余丢弃。请求从命令而非前次请求开始，因此机械臂滞后不会让请求不断超前：松开手柄后机械臂停在命令处，距原位至多一个领先上限。环境 `xyz_scale_m`/`rot_scale_rad` 必须至少等于领先上限（`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`；`real_robot` 默认 0.02/0.1 会在构建源时被拒绝）。
- **交还**保持实测位姿；录制器保持策略共用 tracker，使臂保持在该位姿（不是零 twist）直到下次接管。请求位置超出安全盒（日志 `WORKSPACE HOLD: <axes>`）、输入或跟踪失败也会交还：按按钮 0 再接管。控制循环停滞（两次介入步超过 0.3 s）时保持实测位姿，下一步从该处继续。
- **夹爪。**每次接管时夹爪动作从最后命令开始（原样保留介入包装器的 `last_gripper_action_getter`，因此接管不会打开已闭合夹爪）；之后按钮 1 切换到另一状态。交还期间按钮 1 无效。
- **按钮按下沿仅生效一次。**设备层锁存每次按下直到下次读取，因此短于一个环境步的按键不会丢失；长按计一次，跨环境 reset 按住不计新按下。
- **事件设备的 `t`/`h` 被忽略**（有警告）：它们设置接管状态，可能与按钮 0 接管冲突。按钮 0 是唯一切换键。
- SDK 在 `start()` 打开设备时导入（由包装器调用），因此构建输入源无需 `hidapi`；缺少 SDK 时抛出 `SdkMissing` 并提示安装。

使用 `--teleop-device spacemouse` 的运行将 tracker 记录进实验契约（`teleop_control`，与 `leader_cartesian` 同一块）。`--teleop-event-device spacemouse` 已移除（SpaceMouse 不再提供标签）；使用它的旧命令或预设会失败，提示改用 `--teleop-device spacemouse --teleop-event-device stdin`。

robot YAML 路线（`--teleop-device robot` 和 `backend: spacemouse` teleop 段）通过机器人及其 daemon 读取同一设备，采用设备层自己的旧语义：twist 原样传给环境，手柄移动或按钮按住时介入（`hold_s` 延长），按钮 0 切换设备的锁存夹爪。所有路线中两个按钮都不结束回合；回合结束使用事件设备 Esc。

<a id="robot-teleop-part---teleop-device-robot"></a>

### 机器人遥操作部件（`--teleop-device robot`）

`--env-backend real_robot` 上的 hil_serl 也可从机器人自身的 teleop 部件取得介入（robot YAML `teleop:` 段：spacemouse、`gello` 主臂或 `fake` 后端，本地或机器人 daemon 提供；主臂更适合 `--teleop-device leader_arm`，见[主臂](#leader-arms-gello)）。`TeleopPartSource`（`real_world_rl/teleop/part.py`）将笛卡尔或关节（`joint_abs`/`joint_delta`）遥操作动作映射到环境 `[-1, 1]` 动作空间，并转发设备的 `success`/`failure`/`rerecord` 事件；映射、`--robot-teleop-action-mode` 声明和 YAML 示例见 README 的“动作契约”与“Teleop（`--teleop-device robot`）”章节。

<a id="leader-arms-gello"></a>

### 主臂 GELLO

端到端指南（硬件、标定、robot YAML、示范录制及 `hil_serl`、`residual_hil_serl`、`silri` 的 HIL 命令）见 [gello.md](gello.md)；包含设置向导、BC、ACT 和部署的首次上手指南见[教程](tutorial/00-overview.md)。本节描述输入源。

主臂报告绝对关节位置。`LeaderArmPartSource`（`real_world_rl/teleop/leader_arm.py`）与 `TeleopPartSource` 一样映射到环境关节动作模式（`joint_abs`：按 `joint_limits` 归一化；`joint_delta`：`(q_leader - q_follower) / joint_scale_rad`，每步在一个 `joint_scale_rad` 处饱和），并添加操作员接管瞬间的起始策略。读取 GELLO 有两种方式：

- `--teleop-device leader_arm`：GELLO 是机器人 PC 上 daemon 提供的 robot YAML 的 `teleop:` 部件（`robot-infra daemon --station robot.yaml ...`），经 `RemoteRobot` 读取。`rwrl setup` 将该段（包括标定）写入工作站 YAML；也可手动添加到 robot YAML（例如 `robot_infra/docs/robot.franka.example.yaml`）；`rwrl gello-calibrate --port ... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8` 打印单台设备的 `options` 块：

  ```yaml
  teleop:
    name: gello
    backend: gello
    options:
      port: /dev/serial/by-id/usb-FTDI_...
      joint_ids: [1, 2, 3, 4, 5, 6, 7]
      joint_offsets_rad: [...]    # from gello_calibrate
      joint_signs: [1, 1, -1, 1, 1, 1, 1]
      gripper_id: 8
      gripper_open_rad: ...
      gripper_closed_rad: ...
      intervention: toggle        # or always (pure teleop)
      event_device: /dev/input/by-id/...   # optional evdev pedal/keyboard on the robot PC
  ```

- `--teleop-device gello`（`GelloTeleopSource`）：GELLO 插在 actor 机器上，通过同一设备层后端打开，上述选项作为输入源 kwargs；训练 CLI 从 `--teleop-options-yaml`（见上文）取得它们。

经 daemon 的 HIL 配方（`joint_delta` follower；主臂的 `--robot-teleop-action-mode` 只需指定关节空间，在远程机器人连接前声明部件）：

```bash
rwrl train hil_serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.action_mode joint_delta \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-onset ramp --teleop-event-device keyboard
```

起始策略（`--teleop-onset`，输入源 kwarg `onset`）：

- `ramp`（默认）：`joint_abs` 发出目标 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`（`--teleop-ramp-step-rad`，0.05），按关节限位归一化，因此记录的动作就是实际执行命令；这些介入步带 `info["teleop_events"]["teleop_syncing"] = True`。归档保留它们，训练排除 syncing 专家标签。`joint_delta` 的初始追赶被标记，直到 leader 首次进入 `joint_scale_rad` 内；同次接管后续的饱和运动属于普通示教。
- `arm`：只有 `|q_leader - q_follower|_inf <= arm_tolerance_rad`（`--teleop-arm-tolerance-rad`，0.1）时接管开关才生效。此前该步不算介入，逐关节误差被记录（每秒最多一次）：应将 leader 对齐 follower。激活后保持激活直到开关释放；每次环境 reset 撤销激活。

`q_follower` 是环境最新观测的 `joint_pos`，因此 `state_fields` 必须包含 `joint_pos`。开关来自部件的 `intervening`（机器人 PC 上带 evdev `event_device` 的 `intervention: toggle`，或 `always`），除非由事件设备提供：

`--teleop-event-device keyboard`（或 `stdin`）将运动设备包装为 `CompositeTeleopSource`：运动来自 `--teleop-device`，介入状态和 `episode_end`/`success`/`failure`/`rerecord`/`operator_quit` 来自 actor 机器的事件设备（keyboard 和 stdin：`t` 接管、`h` 交还、Esc 结束回合、`s`/`f`/`r`、`q` 退出）。leader 输入源仍对该状态应用起始策略；部件自身 `intervention` 设置此时被忽略。（`--teleop-device spacemouse` 保留按钮 0 自身接管，只从事件设备取事件，见 [SpaceMouse](#spacemouse)。）

<a id="leader-arm-on-a-cartesian-robot-leader_cartesian"></a>

### 笛卡尔机器人上的主臂（`leader_cartesian`）

GELLO 主臂报告关节角。仅接受笛卡尔 twist 的环境通过 Panda 正运动学使用这些角度：`LeaderArmCartesianSource`（`real_world_rl/teleop/leader_cartesian.py`，`device="leader_cartesian"`）将 leader 关节转换为请求末端位姿，`CartesianTargetTracker`（`real_world_rl/teleop/cartesian_tracker.py`）将其转成驱动 follower 的 twist（见[映射](#mapping)）。`examples/presets/gello_cartesian.yaml` 是单臂 50 Hz 预设。

<a id="why"></a>

#### 使用原因

旧 `franka_real` 栈与 SERL HTTP bridge 通信，仅有 `/pose`（绝对笛卡尔阻抗目标）、`/open_gripper`/`/close_gripper`、`/jointreset`、`/startimp`/`/stopimp` 和 `/getstate`。没有流式关节命令，因此关节空间 leader 源（`--teleop-device leader_arm`/`gello`，见[主臂](#leader-arms-gello)）不能驱动它，而 `FrankaRealEnv` 接受 7 维动作：基坐标系 twist（6）+ 夹爪。因此转换在 actor 侧完成：

```
GELLO joints q --PandaKinematics--> T* = (p*, R*) --vs follower TCP (p, R)--> twist in [-1, 1]
```

同一输入源也服务于 `cartesian_delta` 模式的 `real_robot`（例如使用 `arm_backend: serl_http` 的 daemon）。

<a id="pieces"></a>

#### 组成

| 组成 | 位置 | 作用 |
| --- | --- | --- |
| `PandaKinematics` | `real_world_rl/kinematics/panda.py` | numpy 正运动学，Craig 修正 DH；`flange_pose(q)`、`ee_pose(q)`（xyz、quat xyzw）、`ee_transform(q)`（4×4）。EE 默认值为 Franka Hand：沿法兰 z 轴 0.1034 m，并绕其旋转 -pi/4（`ee_offset_m`、`ee_yaw_rad`）。 |
| `LeaderArmCartesianSource` | `real_world_rl/teleop/leader_cartesian.py` | 遥操作输入源；任意 `joint_abs` 七关节 teleop 部件均可作为 leader。 |
| `FrankaRealEnv.last_tcp_pose` | `real_world_rl/envs/franka_real/env.py` | 最近 reset/step 的实测 TCP 位姿（`O_T_EE`，xyz + quat xyzw）；输入源的 follower 位姿。 |
| 占位机器人 YAML | `robot_infra/docs/robot.gello_station.example.yaml` | 通过机器人 daemon 从机器人主机提供 GELLO。 |

<a id="the-stub-robot-daemon-on-the-robot-host"></a>

#### 机器人主机上的占位 daemon

GELLO 插在机器人主机（运行 SERL bridge 的机器）上。一个 `type: franka`、`arm_backend: fake`、无夹爪和相机的占位机器人将其作为 `teleop:` 部件，由机器人 daemon 提供：

```bash
# on the robot host, next to the SERL bridge
pip install -e "robot_infra[gello]"          # + [gello-events] for an evdev pedal/keyboard
rwrl gello-calibrate --port /dev/serial/by-id/usb-FTDI_... \
    --joint-ids 1 2 3 4 5 6 7 --gripper-id 8            # paste the printed options into the yaml
robot-infra daemon --station robot.gello_station.yaml --bind 0.0.0.0:50052
```

fake 臂不占用硬件且不会收到运动命令，因此占位机器人不与 bridge 争用实体机器人。使用与同机真实机器人 daemon 不同的端口。

<a id="actor-side"></a>

#### Actor 侧

输入源恰好需要一个 leader：

- `remote_address="host:port"`（可选 `remote_token`）：指向占位 daemon 的 `RemoteRobot`，由 `start()` 连接（介入包装器调用），由 `close()` 断开；
- `robot_yaml=<path>`：在本机加载相同占位 YAML（GELLO 插在 actor 机器），由 `start()` 连接；
- `leader=<part or resolver>`：调用方持有的 teleop 部件（测试、自定义配置）。

follower 位姿、逐步尺度和安全盒来自环境：沿包装链查找 `FrankaRealEnv.last_tcp_pose`、`cfg.action_scale`、`cfg.safety_box_low/high`；对于 `real_robot` 笛卡尔环境，则为 `last_observation["state"]["tcp_pose"]`、动作契约的 `xyz_scale_m`/`rot_scale_rad` 和配置的 `safety_box`（`None`：输入源不检查工作区，环境和设备层仍裁剪）。可传 `follower_pose_getter=`、`pos_scale_m=`、`rot_scale_rad=`、`safety_box=` 覆盖。尺度必须至少等于领先上限（`xyz_scale_m >= lead_cap_m`、`rot_scale_rad >= lead_cap_rad`），否则拒绝构建：默认尺度（0.02 m / 0.1 rad）需要调大（如预设中的 `--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`，或 `--franka-real.action-scale-pos`/`--franka-real.action-scale-rot`），或降低上限。

```python
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

env = TeleopInterventionWrapper(
    franka_real_env,
    device="leader_cartesian",
    remote_address="robot-host:50052",
    mapping="anchored",
)
```

训练 CLI 用 `--teleop-device leader_cartesian` 选择输入源，并通过 `--teleop-options-yaml` 将 kwargs 作为 YAML 映射传入（合并到运动源 kwargs）：

```yaml
# station.yaml (actor side)
remote_address: robot-host:50052   # the stub daemon above; or robot_yaml: <path> for a local GELLO
mapping: anchored                  # or absolute
gripper_binarize_threshold: 0.5
```

```bash
rwrl train hil_serl --role actor --env-backend franka_real ... \
    --teleop-device leader_cartesian --teleop-options-yaml station.yaml
```

默认 `intervention: always` 使操作员每步驱动（纯遥操作或示范采集）。HIL 可在机器人主机设置 `intervention: toggle` 加 `event_device`，或从 actor 侧事件设备获取介入（`--teleop-event-device stdin` 或 `keyboard`：`t` 接管、`h` 交还；见[主臂](#leader-arms-gello)）。

<a id="mapping"></a>

#### 映射

`cartesian_delta` 模式的 `RealRobotEnv` 发送 `measured + action * scale`，阻抗后端（Franky 的笛卡尔阻抗 tracker）将该位姿作为平衡点。因此每步根据实测位姿重算 twist 永远无法保持静止：零动作会将平衡点重新锚定到机械臂漂移后的位置（首次硬件实验 24 s 漂移 137 mm）。输入源改为在每臂的 `CartesianTargetTracker` 中保留目标：

- **保持位姿**：在环境 reset 和每次交还时各采样一次（当时的实测位姿），无人驱动时固定；
- 持久的**命令位姿**。接管时为当前命令（保持位姿，无跳变）；操作员驱动时以至多 `--teleop-target-speed-m-s` / `--teleop-target-speed-rad-s`（0.05 m/s、0.3 rad/s）向请求位姿推进，再拉回实测位姿的 `--teleop-lead-cap-m` / `--teleop-lead-cap-rad`（0.05 m、0.3 rad）范围内。多余量丢弃，不排队：leader 跳出再返回不会让 follower 过冲。

发出的机械臂动作以环境归一化单位将 follower 指向命令：

```
a_pos = (p_cmd - p) / pos_scale_m
a_rot = rotvec(R_cmd R^T) / rot_scale_rad
```

这是环境 step 的逆（`p + a_pos * pos_scale_m` 和 `from_rotvec(a_rot * rot_scale_rad) * R`：旋转左乘，所以旋转向量在基坐标系）。尺度至少等于领先上限时动作不会离开 `[-1, 1]`；若将越界，tracker 抛出 `CommandOutOfRange` 而非裁剪。交还、输入或跟踪失败、回合结束都会将 tracker 置于实测位姿的保持状态。控制循环停滞（两次介入轮询超过 0.3 s）视为输入丢失：follower 保持，警告给出间隔，下次轮询像接管一样重新锚定 leader（时钟倒退抛出 `MissedControlDeadline`）。请求位置超出安全盒也如此：输入源记录 `WORKSPACE HOLD: <axes>`，持续报告未介入，直到操作员交还后再接管。

录制器保持策略（`real_world_rl/teleop/hold.py`、`CartesianHold`）共用输入源 tracker：无人驱动时发送回到固定保持位姿的动作（不是零 twist），每步至多一个领先上限。没有 `leader_cartesian`（或同样共用 tracker 的 `spacemouse`）时，保留自身 tracker，上限为 `min(0.05 m, xyz_scale_m)` / `min(0.3 rad, rot_scale_rad)`，所以机械臂被推离保持位姿较远时得到饱和返回动作，不会报错。使用 `--teleop-device leader_cartesian`（或 `spacemouse`）的运行在实验契约中记录 tracker（`teleop_control`：版本 `cartesian_tracker_v1` 和上述四个参与比较的值）：未带 tracker 或值不同的示范不能加载。该块描述 tracker 而非设备，因此参数相同的 GELLO 和 SpaceMouse 示范可以共同加载。

`mapping` 选择请求位姿：

- `anchored`（默认）：每次接管（未介入后首次介入轮询，以及环境 reset 后首次介入）时，输入源保存 tracker 命令 `(p_c0, R_c0)` 和 leader 位姿 `(p_l0, R_l0) = FK(q_0)`。之后将 leader 基坐标系位移应用到命令，平移和旋转解耦：

  ```
  p* = p_c0 + (p_l - p_l0)
  R* = (R_l R_l0^T) R_c0
  ```

接管保留保持动作，因此不会使机器人猛跳。无论两个位姿相距多远，leader 腕部旋转使 follower 绕自身 TCP 原地转动（组合形式 `FK(q) T_l0^{-1} T_f0` 则会使 follower 绕基坐标系原点摆动，产生约 `|p_f0| * theta` 平移）。FK 误差只通过起始以来的变化进入。leader 和 follower 姿态可以不同（介入之间像离合器一样重新握持 leader）；如所有笛卡尔命令，leader 肘部/零空间运动被忽略。
- `absolute`：`T* = FK(q)`。follower 移向 leader 末端所在位置，需要 `PandaKinematics.ee_pose` 匹配机器人 `O_T_EE`（需验证，见下文），且接管前 leader 位姿接近 follower，否则 follower 将以目标速度从保持位姿移到 leader 位姿。

除非 leader 已对机器人完成标定和验证，否则优先使用 `anchored`。

<a id="gripper"></a>

#### 夹爪

leader 宽度目标 `w` 位于 `[0, 1]`（1 = 打开）；`w > gripper_binarize_threshold` 时变为 `+1`（打开），否则为 `-1`（关闭）。默认 0.5 使扳机越过半行程后关闭夹爪。精确 ±1 为混合（离散）夹爪 critic 提供清晰标签，`franka_real`（`a > 0.5` 打开）也可无歧义执行；`gripper_binarize_threshold=None` 则输出连续 `2w - 1`。没有夹爪通道的 leader 保持最后发给环境的夹爪动作，需要 `last_gripper_action_getter`（与 `--teleop-device robot` 相同）。

接管时若扳机与夹爪最后命令状态不一致（打开：action > 0；介入包装器通过 `last_gripper_action_getter` 提供），输入源持续发送最后动作，直到扳机跨到夹爪状态，之后才跟随扳机。因此以打开扳机接管不会掉落已抓物体。

<a id="verify-the-dh-table-on-the-robot"></a>

#### 在真机验证 DH 表

`PandaKinematics` 符合 Franka 公布的参考位姿（`q = 0` 时法兰为 `(0.088, 0, 0.926)`，z 向下；准备姿态 `[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]` 时法兰约 `(0.307, 0, 0.590)`），但尚未与真机比较。依赖 `mapping="absolute"` 前，在若干关节配置下与 bridge 自身位姿比较（准备姿态，以及工作区内分散的两三个位姿，并旋转腕部）：

```python
import numpy as np
from scipy.spatial.transform import Rotation

from real_world_rl.envs.franka_real.bridge_client import FrankaBridgeClient
from real_world_rl.kinematics import PandaKinematics

state = FrankaBridgeClient("http://robot-host:5000").get_state()  # q and pose (O_T_EE)
position, quat = PandaKinematics().ee_pose(state["q"])
pose = np.asarray(state["pose"])
print("position error [m]:", np.linalg.norm(position - pose[:3]))
print("rotation error [rad]:", (Rotation.from_quat(quat) * Rotation.from_quat(pose[3:]).inv()).magnitude())
```

预期误差低于约 1 mm 和 0.01 rad。沿工具轴或绕工具轴的固定偏差表示 Desk 末端设置与 Franka Hand 默认值不同：向输入源传 `kinematics=PandaKinematics(ee_offset_m=..., ee_yaw_rad=...)`。随关节配置变化的误差指向 DH 表本身；`anchored` 映射对两者均较不敏感。

<a id="two-arms-bimanual"></a>

### 双臂（`bimanual`）

`--env-backend real_robot_bimanual`（见 [bimanual.md](bimanual.md)）动作是 `[left arm, right arm, left gripper, right gripper]`，拒绝单臂设备（`robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`spacemouse_twist`、`pico`）。`--teleop-device bimanual`（`BimanualTeleopSource`，`real_world_rl/teleop/bimanual.py`）在每臂自己的 `RealRobotEnv` 上构建单臂源，各自读取自身机器人和 follower，并将读数组成双臂动作。`--teleop-arm-device` 选择每臂设备：

| `--teleop-arm-device` | 每臂输入源 | 备注 |
| --- | --- | --- |
| `leader_arm`（默认） | 基于该臂机器人 teleop 部件的 `LeaderArmPartSource` | 该臂 daemon 提供 GELLO（其 robot YAML `teleop:` 段）；`--robot-teleop-action-mode` 在远程机器人连接前声明两个部件；`--teleop-onset` 参数应用于每臂 |
| `robot` | 基于该臂机器人 teleop 部件的 `TeleopPartSource` | 任意 teleop 部件，按 `--teleop-device robot` 方式映射 |
| `gello` | `GelloTeleopSource` | GELLO 插在 actor 机器，每臂一个；各需独立 `port` 和标定（见下文） |

每臂不支持 `leader_cartesian`：它将一个 leader 转成一条笛卡尔臂的 twist。

双臂源请求介入：任一臂操作员介入时 `intervened` 为 true（默认 `intervention_from: any`），或仅两者都介入时为 true（`all`）。`any` 下未介入臂保持静止：其臂动作块为保持动作（`joint_abs` 为 follower 关节，delta 模式为零），夹爪为最后发送动作，因此空闲 leader 不会移动 follower，空闲扳机不会打开夹爪。`episode_end` 和事件在两臂间取 OR；`info["teleop_events"]` 同时以 `<arm>_<event>` 和普通 `<event>`（`success`、`teleop_syncing` 等）携带事件。

使用 `--teleop-event-device keyboard`（或 `stdin`）时，双臂源是组合源的运动部分：`t`/`h` 同时接管/交还双臂（各臂起始策略仍适用），键盘提供回合结束、结果和退出键。

`bimanual` 的 `--teleop-options-yaml` 可以是每个臂名一段加公共键的映射（`intervention_from` 属于源自身；其它公共键传给每臂，臂段覆盖公共值）：

```yaml
# bimanual.yaml, for --teleop-device bimanual --teleop-arm-device gello
intervention_from: any
hold_s: 0.0
left:
  port: /dev/serial/by-id/usb-FTDI_..._LEFT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
right:
  port: /dev/serial/by-id/usb-FTDI_..._RIGHT-if00-port0
  joint_ids: [1, 2, 3, 4, 5, 6, 7]
  joint_offsets_rad: [...]
  joint_signs: [...]
```

或者在 `--teleop-arm-device gello` 下，直接使用 robot_infra 工作站 YAML（`robots: {left: ..., right: ...}`，由 `rwrl setup` 写出）：各机器人 `teleop.options`（及 `hold_s`）成为该臂 kwargs，按机器人名称匹配环境 `arm_names`。单臂 `--teleop-device gello` 使用单机器人工作站或 robot YAML 也相同。每臂用 `leader_arm` 或 `robot` 时，各 daemon 读取自身 GELLO，因此拒绝工作站 YAML。

经两个 daemon 的 HIL：

```bash
rwrl train hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

`--teleop-device fake` 也声明环境臂数量，因此可在无操作员时干运行双臂 actor。

<a id="interventionwrapper"></a>

## `InterventionWrapper`

`real_world_rl/envs/wrappers/intervention.py`。`InterventionWrapper(env, source)` 接受 `TeleopSource`；`TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)` 按设备名构建输入源（或适配传入 `teleop` 对象）。两者都以 `.source` 暴露输入源（别名 `.teleop`）。

`step(action)` 先轮询输入源，将其人工掩码与所选部件取交集。人工掩码非空时，在策略设备上将人工和空闲部件动作组合为 `(1, A)` float32 张量。`idle_behavior="hold"` 保持未选部件；`"policy"` 在这些部件保留策略提议。实际人工控制和保持覆盖分别记录：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 实际执行的 `(1, A)` 动作；**仅介入时** |
| `policy_action` | 原样收到的策略提议动作 |
| `intervened` | Python bool：操作员动作已执行 |
| `intervention_onset` | Python bool：当前介入，本回合上一步未介入 |
| `human_episode_end` | Python bool：操作员回合结束按钮 |
| `teleop_events` | 已触发 `success`/`failure`/`rerecord` 边沿及 leader 追赶 `teleop_syncing` 的 `dict`；**仅存在时** |
| `human_action_mask` | 人实际驱动的维度，包括 syncing 期间的溯源 |
| `override_mask` | 替换策略提议的维度，包括保持补充动作 |

`episode_end` 使 `step()` 返回 `truncated=True`（不会是 `terminated`），因此 actor 循环 reset，但不将其视为终止。

事件顺序：

- 构建：按环境契约验证输入源，再调用 `source.start()`。
- `reset(**kw)`：先 `env.reset()` 再 `source.reset()`；机器人回 home 期间的按键丢弃。“最后夹爪动作”记录恢复为打开（+1），起始检测器重启。
- `close()`：先 `source.close()` 再 `env.close()`（输入源关闭失败时也执行）。

包装器记住最后发送给环境的夹爪动作（裁剪到 `[-1, 1]`）；无夹爪遥操作源在带夹爪环境上会保持夹爪，不会掉落已抓物体。

<a id="adding-a-device"></a>

## 新增设备

继承 `TeleopSource`，声明动作空间，实现 `poll()` 并注册：

```python
from real_world_rl.teleop import TeleopReading, TeleopSource, register_teleop_source

@register_teleop_source("joystick")
class JoystickTeleopSource(TeleopSource):
    action_mode = "cartesian_delta"
    arm_action_dim = 6
    has_gripper = True

    def __init__(self, port: str = "/dev/input/js0") -> None: ...

    def start(self) -> None: ...      # open the device
    def poll(self) -> TeleopReading: ...
    def reset(self) -> None: ...      # drop latched presses
    def close(self) -> None: ...
```

之后 `make_teleop_source("joystick", env, port=...)` 调用 `JoystickTeleopSource.from_env(env, port=...)`，默认是 `cls(**kwargs)`；可覆盖 `from_env` 读取环境动作契约（`real_world_rl.action_contract.get_env_action_contract(env)`）。未知名称抛出 `ValueError` 并列出已注册名称。注册必须在使用名称前完成，因此下游包在构建环境前导入模块（或通过自身入口暴露）。内置源在名称首次使用时导入，因此导入 `real_world_rl.teleop` 不会引入 `zmq` 或 `hid`。

动作位于环境归一化 `[-1, 1]` 空间，夹爪在末尾。对于 twist 使用物理单位的笛卡尔设备（如 Pico），参考示范脚本的归一化方式。

<a id="eetwist-teleop-interface-pico"></a>

## EETwist 遥操作接口（Pico）

`PicoTeleopSource` 包装仓库内 `robot_infra` 设备层的 `robot_infra/robot_infra/teleop/utils/telo_op_control_twist.py` 中的 `EETwistTeleOpWrapper`。其唯一设备为 `pico`；包装器从 ZeroMQ `SUB` socket 接收设备数据。Pico 服务端应向以下位置发布 JSON 数组：

```python
tcp://*:7777
```

客户端通过 `--zmq-url` 连接，例如：

```bash
--zmq-url tcp://192.168.6.2:7777
```

`PicoTeleopSource.reset(*, episode_end_pressed=False)` 将该标志转发到包装器，避免仍按住的回合结束按钮再次触发；`examples/record_teleop_wsrl.py` 使用它。

<a id="pico-data-format"></a>

### Pico 数据格式

Pico 数据可为单手（`13` 个值）或双手（`26` 个值）。双手时，前 `13` 个值为左手，后 `13` 个值为右手。

每只手：

| 索引 | 含义 |
| --- | --- |
| `0:3` | 手位置 |
| `6:10` | 手四元数，`[x, y, z, w]` |
| `10` | 夹爪按钮 |
| `11` | 绑定/离合按钮 |
| `12` | 回合结束按钮 |

选择当前参考位姿时按住 `bind`。绑定期间 twist 为零。释放后，手位姿变化转换为末端 twist。

`EETwistTeleOpWrapper.poll()` 返回 `TeleOpSample`，`PicoTeleopSource` 将其转为 `TeleopReading`（`action` = twist + 夹爪、`intervened`、`episode_end`）：

```python
action       # twist + gripper
twist        # shape (6,)
gripper      # scalar gripper command
bind_pressed
episode_end
intervened
```

没有新设备数据到达时，`twist=0`，`gripper` 保留前值，`intervened=False`。

<a id="basic-usage"></a>

## 基本用法

可视化遥操作：

```bash
python examples/run_teleop_demo.py \
  --env-id PegInsertionSidePegOnly-v1 \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

不连接环境，仅打印传入遥操作状态：

```bash
python robot_infra/robot_infra/teleop/examples/run_twist_input_inspector.py \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

同一示例使用 SpaceMouse（通过 `--spacemouse-index`、`--spacemouse-deadzone`、`--spacemouse-invert-axes`、`--spacemouse-hold-s` 调整），或空闲 `fake` 源：

```bash
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse_twist
```

模拟示例使用原始 twist SpaceMouse 源 `spacemouse_twist`：移动手柄即介入，twist 原样进入环境，按钮 0 切换夹爪，按钮 1 结束回合。它仅用于模拟；真实机器人使用 `--teleop-device spacemouse`（[SpaceMouse](#spacemouse)）。

`run_teleop_demo.py` 创建 ManiSkill 环境，通过 `make_teleop_source` 构建输入源，轮询遥操作动作，对 Pico 物理 twist 应用控制器动作归一化（SpaceMouse 动作已归一化），并步进环境。

<a id="recording-demos-on-the-real-robot"></a>

## 在真机上录制示范

`examples/record_real_demos.py` 在 HIL 运行的 `real_robot`、`real_robot_bimanual` 或 `franka_real` 环境中录制遥操作回合（与 actor 相同的环境和遥操作参数），写成 learner 用 `--demo-dataset-paths` 加载的示范 pkl 格式，仅保留介入步。见 [recording-demos.md](recording-demos.md)；GELLO 见 [gello.md](gello.md#5-recording-demos)。

<a id="recording-wsrl-data"></a>

## 录制 WSRL 数据

`examples/record_teleop_wsrl.py` 将遥操作回合录制为与 WSRL 数据生成器相同的 H5 格式。

示例：

```bash
python examples/record_teleop_wsrl.py \
  --output-path demos/peg_teleop_wsrl.h5 \
  --env-id PegInsertionSidePegOnly-v1 \
  --obs-mode rgb \
  --control-mode pd_ee_twist \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

`--output-path` 必须为 H5 文件路径，不是目录。建议使用 `.h5` 后缀。文件以写入模式打开，因此相同路径已有文件会被覆盖。

仅 `intervened=True` 样本在环境中执行并保存。`intervened=False` 时，脚本继续轮询设备，不记录转移。

操作员回合结束按钮（Pico 回合结束按钮、`spacemouse_twist` 按钮 1）结束当前回合。每回合后，终端打印回合长度、`max_episode_steps`、成功标志及回报，再询问是否保存回合。

<a id="important-arguments"></a>

### 重要参数

- `output_path`：要写入的 H5 数据集文件。
- `device`：`pico`、`spacemouse_twist` 或 `fake`（设备专属参数与示例相同）。
- `dt`：遥操作轮询周期，秒。默认 `1 / 30`，约 `30 Hz`。较小值使轮询和步进更快。
- `env_id`：ManiSkill 环境 ID。
- `obs_mode`：观测模式，通常为 `state`、`rgb` 或 `rgbd`。
- `include_state`：视觉观测中包含本体状态。
- `control_mode`：应匹配末端 twist 控制器，通常为 `pd_ee_twist`。
- `sim_backend`、`render_backend`：ManiSkill 模拟和渲染后端。
- `reward_mode`：传给环境的奖励模式。
- `robot_uids`：环境机器人 ID。
- `fix_box`、`fix_peg_pose`、`peg_density`、`debug_pose_vis`：自定义 peg 环境选项。
- `env_kwargs_json`：额外环境 kwargs 的补充入口。
- `end_on_env_done`：为 true 时，环境终止、截断或成功可结束回合。默认 false，因此仅遥操作回合结束事件停止录制。
- `pos_scale`、`rot_scale`、`twist_limit`：twist 映射的可选覆盖值。
- `intervention_threshold`：用于判断人工输入是否活动的 twist 范数阈值。

保存的 H5 包含 `traj_*` 组，内容为：

```text
obs
actions
rewards
terminated
truncated
```

此文件可通过 `--offline_dataset` 传给 WSRL、CQL 或 Cal-QL 离线训练。

对于只需离线预训练的真机数据（无模拟环境、无 eval），使用 rl-garden 的 `examples/pretrain_offline.py cql` 或 `examples/pretrain_offline.py calql` 生成独立离线 CQL/Cal-QL 检查点。若检查点要由 WSRL 离线→在线流程恢复，使用 `wsrl` 子命令。该入口从 H5 推断观测/动作规格，生成的预训练检查点可在之后兼容的实时环境训练运行中加载并在线微调。完整流程见 `rl-garden/` 子模块的 `rl-garden/docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env`。

<a id="partial-control-and-training-provenance"></a>

## 局部控制与训练溯源

用 `--teleop-selected-parts` 选择部件（教程工作站为 `arm.arm` / `arm.gripper`，双臂为 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`）。默认 `--teleop-idle-behavior hold` 保持未选部件；`policy` 使其继续自主控制。空选择使用输入源可用部件。没有夹爪通道的输入源不提供专家夹爪标签。支持连续无夹爪 HIL；混合/离散分支要求夹爪。syncing 行仍执行/归档，但其训练投影无专家监督。掩码 BC/SiLRI/残差损失、显式结果和持久记录见[真实机器人实验](real-world-experiments.md)。
