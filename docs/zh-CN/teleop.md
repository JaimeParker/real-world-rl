<a id="teleoperation"></a>

# 遥操作

`real_world_rl` 的人工介入路径：`TeleopSource` 读取一个操作员设备，`InterventionWrapper` 用人工动作替换策略动作，`examples/` 中示范脚本复用相同输入源。

**从哪里开始：** 按 [05 GELLO](tutorial/05-teleop-gello.md)、[06 SpaceMouse](tutorial/06-teleop-spacemouse.md) 或 [07 主从跟随](tutorial/07-teleop-leader-follower.md)检查输入并完成首次受监督会话，再读 [08 录制示范](tutorial/08-record-demos.md)。本页是输入源约定和设备参考。

<a id="the-teleopsource-contract"></a>

## `TeleopSource` 接口约定

`real_world_rl.teleop`（`teleop/core/contract.py`）：

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

- 构建包装器时，对照环境动作约定检查 `action_mode`、`arm_action_dim`、`has_gripper`；不匹配（如在 `joint_delta` 机器人上用笛卡尔 SpaceMouse）会在启动设备前抛出 `ValueError`。
- `poll()` 必须让设备失败向外传播；吞掉错误会将策略动作记成人工示范。
- `as_teleop_source(obj)` 对 `TeleopSource` 原样返回，对鸭子类型旧对象用 `LegacyTeleopAdapter` 包装（`poll()` 返回具有 `.intervened`、`.action`、`.episode_end` 的对象；可选 `reset()`、`close()` 及 `action_mode`/`arm_action_dim`/`has_gripper`，默认 cartesian_delta / 6 / True）。

<a id="devices"></a>

## 设备

`TeleopInterventionWrapper(env, device=..., **kwargs)` 通过 `make_teleop_source(device, env, **kwargs)` 构建源；hil_serl 中设备来自 `--teleop-device`。

hil_serl（以及 residual、SiLRI）从参数推导源 kwargs（`robot`/`leader_arm`：机器人部件声明和限制；`gello`/`leader_arm`：接管起始参数），再合并 `--teleop-options-yaml PATH`：源 kwargs 的 YAML 映射，覆盖推导值。用于 CLI 没有专门参数的选项，例如本地 GELLO：

```yaml
# gello.yaml, for --teleop-device gello --teleop-options-yaml gello.yaml
port: /dev/serial/by-id/usb-FTDI_...
joint_ids: [1, 2, 3, 4, 5, 6, 7]
joint_offsets_rad: [...]   # from teleop calibrate --backend gello
joint_signs: [1, 1, -1, 1, 1, 1, 1]
gripper_id: 8
gripper_open_rad: ...
gripper_closed_rad: ...
hold_s: 0.0
```

或第二个 SpaceMouse 的 `device_index: 1`（以及 `deadzone`、`invert_axes`）；`franka_real` 技术栈上的 `--teleop-device leader_cartesian` 则可配置提供主臂服务的机器人 daemon（见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)）：

```yaml
# leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
remote_address: <robot-host>:<port>
```

使用 `--teleop-event-device` 时，此文件供复合源的运动部分使用。

包装器把“最后发送的夹爪动作”传给 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`pico`、`bimanual`（及以其构建的复合源运动部分），使无夹爪主臂保持夹爪，不松开已抓物体；SpaceMouse、Pico 每次接管从夹爪当前命令开始。环境动作末尾每臂一个夹爪维度（按动作约定布局），getter 返回每臂一项，单臂源读取第 0 项（`bimanual` 为各臂源分配各自项）。输入源默认驱动一臂，除非声明 `num_arms`；源臂数与环境不同，或声明的 `arm_names` 与环境名称/顺序不一致时，包装器拒绝。

| `device` | 输入源 | 读取内容 | 动作空间 | 说明 |
| --- | --- | --- | --- | --- |
| `pico` | `PicoTeleopSource` | 通过设备层 `PicoTeleop`（ZeroMQ 订阅者）读取 Pico VR 控制器，将位姿输入锚定到笛卡尔命令 tracker | cartesian_delta，6 + 夹爪 | bind 接管/交还，扳机控制夹爪；见 [Pico](#pico)。kwargs：`address="tcp://localhost:7777"`、`topic=""`、`hand="right"`、`bind_button="bind"`、`poll_interval_s=0.005`、`hold_s=0.0`、`input_rotation`、`gripper_binarize_threshold=0.5`、`remote_address`、`read_timeout_s`，以及 tracker 选项（`--teleop-lead-cap-*`/`--teleop-target-speed-*`）。需要 `pyzmq`（扩展 `pico`），在 `start()` 导入。 |
| `pico_twist` | `PicoTwistTeleopSource` | Pico 原始 twist（`EETwistTeleOpWrapper`；仅 ManiSkill 仿真示例，不用于真机） | cartesian_delta，6 + 夹爪 | 旧 `pico`：kwargs 为 `zmq_url`、`hand`、`pos_scale`、`rot_scale`、`twist_limit`、`intervention_threshold`；构建时打开订阅者。 |
| `spacemouse` | `SpaceMouseTeleopSource` | 本机 SpaceMouse，经设备层 `SpaceMouseTeleop` 作为笛卡尔命令 tracker 的速度输入 | cartesian_delta，6 + 夹爪 | 按钮 0 接管/交还，按钮 1 切换夹爪；见 [SpaceMouse](#spacemouse)。kwargs：`device_index=0`、`deadzone=0.05`、`invert_axes=()`、`poll_interval_s=0.005`、`read_timeout_s` 和 tracker 选项（`lead_cap_m`、`lead_cap_rad`、`target_speed_m_s`、`target_speed_rad_s`，即 `--teleop-lead-cap-*`/`--teleop-target-speed-*`）。需要 `hidapi==0.14.0.post4`（导入 `hid`），在 `start()` 导入。 |
| `spacemouse_twist` | `SpaceMouseTwistTeleopSource` | 同一 SpaceMouse 的原始 twist（仅 ManiSkill 仿真示例，不用于真机） | cartesian_delta，6 + 夹爪 | twist 原样送入环境，移动操纵帽算介入（`hold_s` 延长），按钮 0 切换保持式夹爪（每次 reset 后打开），按钮 1 结束回合。kwargs：`device_index=0`、`deadzone=1e-3`、`invert_axes=()`、`poll_interval_s=0.005`、`hold_s=0.0`、`read_timeout_s`。 |
| `robot` | `TeleopPartSource` | `real_robot` 环境背后机器人的 teleop 部件（机器人 YAML 的 `teleop:`，本地或 daemon 提供） | cartesian_delta、joint_abs 或 joint_delta，映射到环境空间 | kwargs：`teleop_contract`、`joint_limits`、`joint_scale_rad`、`read_timeout_s`（hil_serl：`--robot-teleop-action-mode`、`--no-robot-teleop-has-gripper`）。部件由机器人拥有，`close()` 保持其连接。 |
| `fake` | `ScriptedTeleopSource` | 脚本读取（用于测试；无 `readings` 时从不介入） | 遵循环境动作约定，包括臂数 | 用 `readings=[TeleopReading, ...]` 编排，最后一项重复。 |
| `gello` | `GelloTeleopSource` | 本机 GELLO，经设备层 `gello` teleop（Dynamixel 总线） | joint_abs 或 joint_delta（随环境），按 `robot` 方式映射主臂关节 | kwargs：设备选项（`port`、`joint_ids`、`joint_offsets_rad`、`joint_signs` 等，见[主臂](#leader-arms-gello)）、`hold_s`、`read_timeout_s`、`onset`、`arm_tolerance_rad`、`ramp_step_rad`。需要 `dynamixel-sdk`（robot_infra 扩展 `gello`），在 `start()` 导入。 |
| `leader_arm` | `LeaderArmPartSource` | 将 `real_robot` 背后的机器人 teleop 部件作为关节空间主臂读取（机器人 daemon 提供的 GELLO） | joint_abs 或 joint_delta（随环境） | kwargs：`robot` 选项加 `onset`、`arm_tolerance_rad`、`ramp_step_rad`（hil_serl：`--teleop-onset`、`--teleop-arm-tolerance-rad`、`--teleop-ramp-step-rad`）。 |
| `keyboard` | `KeyboardTeleopSource` | 经 `pynput` 读取本机键盘 | 遵循环境动作约定；动作始终为零 | `t` 接管，`h` 交还，Esc 结束回合，`s`/`f`/`r` 产生 `success`/`failure`/`rerecord`，`q` 结束回合并产生 `operator_quit`（recorder 和 actor 循环在该回合后停止）。属于事件设备，单独使用仅适合纯事件测试。需要 `pynput`（扩展 `reward-collection`）及桌面会话，在 `start()` 导入。 |
| `stdin` | `StdinTeleopSource` | 运行进程的终端（cbreak 模式，`close()` 恢复；支持 SSH，无需桌面） | 遵循环境动作约定；动作始终为零 | 与 `keyboard` 同样按键。kwargs：`fd`（默认 stdin）、`poll_interval_s=0.02`。输入结束使下次轮询抛出 `DeviceFault`。 |
| `composite` | `CompositeTeleopSource` | 运动源加事件源 | 运动源的空间 | kwargs：`motion`、`events`（注册名）、`motion_kwargs`、`events_kwargs`、`intervention_from="events"`（或 `"motion"`）。hil_serl 根据 `--teleop-event-device` 构建。 |
| `leader_cartesian` | `LeaderArmCartesianSource` | 经 Panda 正向运动学把关节空间主臂转为笛卡尔 twist（`franka_real` 技术栈） | cartesian_delta，6 + 夹爪 | 见[笛卡尔机器人上的主臂](#leader-arm-on-a-cartesian-robot-leader_cartesian)。 |
| `bimanual` | `BimanualTeleopSource` | `real_robot_bimanual` 每臂一个单臂源（`--teleop-arm-device`） | 随环境，两臂动作后接两个夹爪 | kwargs：`device_per_arm`、`per_arm_kwargs`、`common_kwargs`、`intervention_from="any"`（或 `"all"`）。见[双臂](#two-arms-bimanual)。 |

### SpaceMouse

`--teleop-device spacemouse` 支持两种模式，由 `--spacemouse-mode {tracker,serl}` 选择（顶层 YAML `spacemouse_mode`，默认 `tracker`）。训练、示范录制、真机奖励采集共用此参数。Deploy 的可选覆盖默认 `None`：继承训练模式，无训练配置则用 `tracker`。

| 行为 | `tracker`（默认） | `serl` |
| --- | --- | --- |
| 接管 | 按钮 0 切换接管/交还；仅移动操纵帽不介入。 | 原始六轴 L2 范数严格 `> 0.001` 时介入，含旋转，无逐轴死区。每步重新判断；松开操纵帽和夹爪按钮后，下一空闲步交还。 |
| 夹爪 | 驱动时按钮 1 切换开/闭；接管保留最后命令。 | 按住按钮 0 关闭，每步在 `[-1, -0.9)` 重新均匀采样；否则按住按钮 1 打开，`uniform(0.9, 1)`。同时按下时按钮 0 优先；均未按为中立 `0.0`。仅环境有夹爪动作时按钮才引起接管。 |
| 机械臂目标 | 持久命令，带目标速度和相对实测位姿的领先限制。 | 每步最新实测位姿 + 原始动作 × 环境缩放；旋转左乘实测四元数。裁剪 XYZ 和 RPY 安全框（roll 按绝对值裁剪并保留符号）。无时间步重缩放或 tracker 目标。 |
| 夹爪执行 | 原环境行为。 | 缩放动作 `<= -0.5` 且实测归一化宽度 `> 0.85` 才关闭；`>= 0.5` 且宽度 `< 0.85` 才打开。距上次命令时间须严格大于 `gripper_sleep_s`（默认 `0.6 s`），随后 sleep 相同间隔。零为中立。 |
| 终端/键盘 | 忽略 `t`/`h`，状态变化时警告；标记/结束/退出仍可用。 | 同样按键：`s`/`f`/`r`/Esc/`q` 标记/结束/退出。`t`/`h` 不控制接管，状态变化时警告。 |

机器人基座倾斜时，`teleop_options_yaml` 可提供 `input_rotation`，即有限且合法的 3×3 旋转矩阵 `base_R_input`。两种模式均将平移和角输入旋转到机器人基坐标。轴反转和 tracker 逐轴死区先于旋转；SERL 接管仍用原始六轴范数。旋转后的每个三向量仅在需要时等比缩小至动作立方体内，保留方向。不施加原点平移。解析矩阵和档案名/文件/yaw 在持久化解析配置前保存为不参与比较的 `teleop_input` 来源信息。恢复使用数值选项，不重读已改变档案。记录动作为基坐标，因此操作员重标定不会使旧示范或恢复约定失效。旧安全框仍在机器人基坐标系。可选工作站 `task_frame`、`workspace` 增加桌面约束，与原限制取交集；输入旋转本身不移动安全边界。两者都不改变机器人安装/重力配置或负载模型。

SpaceMouse tracker 在发送请求目标前、速度/领先限制后均检查任务工作空间。候选被拒绝时交还控制，并将 recorder 保持锚定在实测位姿；按钮 0 可再次接管。训练中交还后，策略通过相同设备约束恢复。策略位置目标在任务坐标逐分量裁剪，超过 `max_angle_rad` 的旋转通过缩放相对参考 rotvec 投影到 SO(3) 锥。SERL 先执行原 XYZ/RPY 框裁剪，再作工作空间投影。实测 TCP 超出工作空间及反馈裕量会锁存故障。裕量默认 `measured_margin_m: 0.003`、`measured_margin_rad: 0.03`，已接受目标仍用 `1e-9` 容差。设备和环境约束对两种遥操作模式及策略动作均有最终约束力。

命名操作员档案使用 [CLI 标定流程](spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)。平移和角向量的组合输入旋转均为 `base_R_task @ Rz(operator_yaw)`。固定 HID 轴映射是设备约定，不是实测安装变换。不要对已标定 `base_R_task` 再加 roll 修正。绝对位置使用含原点的完整任务变换；反馈观测和策略动作保留原基坐标约定。

SERL 须先配置工作站 **XYZ 和 RPY 安全框**，缺任一个均为构建错误。`real_robot` 中为 `safety_box`（`[xmin, ymin, zmin, xmax, ymax, zmax]`）、`safety_box_rpy_low`、`safety_box_rpy_high`；缩放仍为 `xyz_scale_m`/`rot_scale_rad`。`franka_real` 中将 `safety_box_low`、`safety_box_high` 和两项 RPY 限制放入 `franka_real.env_kwargs_json`，使用原位置/旋转缩放。模式在构建环境前选择 `cartesian_semantics: serl`，不打开遥操作设备的 learner 也如此。显式配置 `current` 与此模式冲突；未设置的程序化 dataclass 默认值会提升为 `serl`。训练/录制参数序列化前完成模式解析，所以保存的 SERL 配置同时含 `spacemouse_mode: serl` 与 `cartesian_semantics: serl`。Deploy 保留保存的环境语义；`--spacemouse-mode serl` 覆盖与已存 `cartesian_semantics: current` 冲突。

录制、训练、部署使用同一模式。SERL 记录以下参与比较的 `teleop_control` 约定（非默认 `invert_axes` 还会加入该列表）：

```json
{"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, "gripper": "serl_buttons"}
```

SERL 要求单个六轴 `cartesian_delta` 机械臂，完整选择机械臂/夹爪。固定 HID 映射 `[-y, x, z, -roll, -pitch, -yaw]` 及配置的输入旋转后，动作仍在基坐标系；不支持相对坐标系、相对观测或 `action_indices`。`teleop_options_yaml` 可设 `device_index`（默认 `0`）、`invert_axes`（默认 `[]`）、`poll_interval_s`（默认 `0.005`）、`read_timeout_s`（默认 `1.0`）。此模式拒绝 tracker 选项、`deadzone`、`hold_s`、夹爪保持 getter 和嵌套 `mode` 键；使用顶层模式参数。设备层 `max_delta_xyz_m` 与位姿框仍生效，可能比原生 HIL-SERL 更紧。硬件行为与安装仍**未验证**。`spacemouse_twist` 仍是独立仿真源，按钮/保持式夹爪行为不同，不是 SERL 模式。

<a id="spacemouse-installation-and-linux-permissions"></a>

#### SpaceMouse 安装与 Linux 权限

在 NUC（或连接 SpaceMouse 的机器）的仓库根目录，激活[教程安装第 4 步的项目环境](tutorial/01-install.md#4-install-the-project-environment)，安装全部 SpaceMouse Python 依赖：

```bash
pip install -e '.[spacemouse]'
```

`spacemouse` 扩展安装 `hidapi==0.14.0.post4`（trezor/cython-hidapi，导入 `hid`）。设备层有自己的解码器，此路径不导入 `pyspacemouse` 或 `easyhid`。不要安装另一 PyPI 发行包 `hid`，它占用相同导入名但 API 不同。缺模块抛出 `SdkMissing` 并给出上方 pip 命令；误装 `hid` 单独报告。

全新环境，在 NUC 仓库根目录、未激活任何环境时运行：

```bash
bash scripts/install.sh --extras franka-gello,spacemouse
source .venv/bin/activate
```

单个扩展 pip 命令假定三个本地项目已安装，不能从空 venv 初始化内部发行包。wheel 目标为 Ubuntu 22.04 / CPython 3.10 / x86_64。此传输避免 easyhid 的系统 `libhidapi` 查找；固定 wheel 的 ELF/系统库依赖仍须在干净 Ubuntu 22.04 验证。Wheel 安装、USB 权限、方向、时序、断连行为**尚未上机验证**。此 wheel 路径不要求安装系统 `libhidapi`，无合适 wheel 平台上的源码构建不在此目标内。

先在 NUC 仓库根目录、项目环境激活后运行 `rwrl --station local/station.yaml doctor --fix`，SpaceMouse 已插入且工作站配置 `spacemouse` teleop。USB 节点不可读写时，确认一次后建议 `plugdev` 组与 `/etc/udev/rules.d/99-rwrl-spacemouse.rules` 中受支持 vendor/product ID 规则；报告待办时重新插拔并登录（[cli.md](cli.md#robot-host-diagnosis-and-repair)）。下方手工安装适用于无项目环境时。

安装一次随包提供的厂商规则。在 NUC 仓库根目录，任意 shell（无需项目环境），具备 sudo 权限时运行：

```bash
sudo cp real_world_rl/teleop/spacemouse/udev/99-spacemouse.rules /etc/udev/rules.d/
sudo usermod -aG plugdev "$USER"
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw
```

文件覆盖 vendor `256f`（3Dconnexion）与 `046d`（旧 Logitech），设置 `MODE="0664", GROUP="plugdev"`。退出并重新登录，拔插 SpaceMouse。仓库不可用时，下方替代复制命令可在 NUC 任意目录、**项目环境已激活**且有 sudo 权限时运行，然后执行上方相同的组/reload 命令：

```bash
sudo cp "$(python -c 'import os, real_world_rl.teleop.spacemouse as slice; print(os.path.dirname(slice.__file__))')/udev/99-spacemouse.rules" /etc/udev/rules.d/
```

打开/权限失败会指向规则文件及安装命令。Doctor 分别检查 `hid` SDK 和 `/dev/hidraw*` 访问；Docker 内还须传入 HID 节点。旧 `SpaceMouseExpert` 与 `legacy-bridge` 保留独立 `pyspacemouse` API。

在 NUC 仓库根目录、项目环境激活、工作站 `hil.yaml` 已配置时，针对 `cartesian_delta` 环境（该模式的 `real_robot` 或 `franka_real`），用终端输入操作员标记（SpaceMouse Compact 两按钮均已占用）：

```bash
rwrl record --config hil.yaml \
    --teleop-device spacemouse --teleop-event-device stdin \
    --output-path demos/task_20.pkl --num-successes 20
```

HIL actor 接受相同两个参数。配置两种安全框后，加 `--spacemouse-mode serl` 选择上表原生行为。以下描述默认 **tracker** 模式。`SpaceMouseTeleopSource`（`real_world_rl/teleop/spacemouse/source.py`）通过设备层 `SpaceMouseTeleop` 打开设备（`TeleopConfig(backend="spacemouse")`，轮询线程为 `read()` 供样本；twist 在 `[-1, 1]`，HID 读取器状态轴映射 `[-y, x, z, -roll, -pitch, -yaw]`，`invert_axes` 反转所列输出索引 0–5），并驱动与 `leader_cartesian` 相同的 `CartesianTargetTracker`（见[映射](#mapping)）：

| 输入 | 效果 |
| --- | --- |
| 按钮 0 | 接管，再按交还。仅移动操纵帽不介入，轻碰不会接管。 |
| 按钮 1 | 驱动期间切换夹爪打开（`+1`）/关闭（`-1`） |
| 操纵帽 | 驱动期间为速度输入（见下文） |
| `--teleop-event-device stdin`（或 `keyboard`）的 `s`/`f`/`r`、Esc、`q` | `success`/`failure`/`rerecord`、结束回合、退出 |

- **速度。** 每个介入步请求 tracker 命令移动 `twist * target_speed * dt`（`dt` 是距上个介入步时间，接管时为 0，所以接管不跳变）：平移为基坐标 `twist[:3] * --teleop-target-speed-m-s * dt`，旋转向量 `twist[3:] * --teleop-target-speed-rad-s * dt` 左乘，与环境步相同。满偏转对应目标速度（0.05 m/s、0.3 rad/s），tracker 也将对角运动限制到此速度。`deadzone` 内各轴视为零（满量程 0.05，高于 Compact 静止噪声）。
- **领先上限。** 命令领先实测 TCP 不超过 `--teleop-lead-cap-m`/`--teleop-lead-cap-rad`（0.05 m、0.3 rad），超额丢弃。请求从命令而非上个请求开始，机械臂滞后不会让请求无限前进：释放操纵帽后停在命令位置，距当时位置最多一个领先上限。环境 `xyz_scale_m`/`rot_scale_rad` 至少等于领先上限（`--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`；`real_robot` 默认 0.02/0.1 在构建源时被拒绝）。
- **交还**保持实测位姿；recorder 保持策略共享 tracker，保持该位姿（不是零 twist）直到下次接管。请求位置超安全框（日志 `WORKSPACE HOLD: <axes>`）、输入或跟踪失败也会交还：按按钮 0 再接管。控制循环停顿（两个介入步间超过 0.3 秒）会保持实测位姿，下一步从这里继续。
- **夹爪。** 每次接管从夹爪最后命令开始（介入包装器 `last_gripper_action_getter`，原样保留，因此不打开闭合夹爪），按钮 1 再切换状态。交还期间按钮 1 无效。
- **按钮按下仅生效一次。** 设备层锁存每次按下直到下次读取，短于环境一步的按键也不丢失；长按只算一次，跨 reset 长按不算按下。
- **忽略事件设备的 `t`/`h`**（有警告）：它们设置的接管状态可能与按钮 0 冲突。按钮 0 是唯一切换来源。
- `start()` 打开设备时才导入 SDK（包装器调用它），构建源不需要 `hidapi`；缺 SDK 抛 `SdkMissing` 并提示安装。

`--teleop-device spacemouse` 将 tracker 记录到实验约定（`teleop_control`，与 `leader_cartesian` 相同块）。已移除 `--teleop-event-device spacemouse`（SpaceMouse 不再提供标记）；旧命令或预设会失败，并提示 `--teleop-device spacemouse --teleop-event-device stdin`。

机器人 YAML 路径（`--teleop-device robot` 加 `backend: spacemouse` 的 teleop 段）通过机器人及 daemon 读取同一设备，采用设备层原有语义：twist 原样进入环境，操纵帽移动或按钮按住时介入（`hold_s` 延长），按钮 0 切换设备保持式夹爪。任何路径的两个按钮都不结束回合；回合结束用事件设备 Esc。

<a id="robot-teleop-part---teleop-device-robot"></a>

### 机器人遥操作部件（`--teleop-device robot`）

`--env-backend real_robot` 的 hil_serl 也可从机器人自身 teleop 部件介入（YAML 的 `teleop:`：spacemouse、`gello` 主臂或 `fake`，本地或 daemon 提供；主臂更适合 `--teleop-device leader_arm`，见[主臂](#leader-arms-gello)）。`TeleopPartSource`（`real_world_rl/teleop/core/part.py`）将笛卡尔或关节（`joint_abs`/`joint_delta`）遥操作动作映射到环境 `[-1, 1]`，转发 `success`/`failure`/`rerecord` 事件。映射、`--robot-teleop-action-mode` 声明和 YAML 示例见 README 的“动作约定”与“机器人 teleop”节。

<a id="leader-arms-gello"></a>

### 主臂（GELLO）

硬件、标定、机器人 YAML、录制及 `hil_serl`、`residual_hil_serl`、`silri` 命令的端到端指南见 [gello.md](gello.md)；使用配置向导、BC、ACT 与部署的首次操作见[教程](tutorial/00-overview.md)。本节介绍输入源。

主臂报告绝对关节位置。`LeaderArmPartSource`（`real_world_rl/teleop/leader/joint.py`）像 `TeleopPartSource` 一样映射到环境关节动作模式（`joint_abs`：归一化至 `joint_limits`；`joint_delta`：`(q_leader - q_follower) / joint_scale_rad`，每步饱和到一个 `joint_scale_rad`），并添加操作员接管时的起始策略。读取 GELLO 有两种方式：

- `--teleop-device leader_arm`：GELLO 是机器人 PC 上 daemon 提供的机器人 YAML `teleop:` 部件（`rwrl daemon --station robot.yaml ...`），通过 `RemoteRobot` 读取。`rwrl setup` 将该段（含标定）写入工作站 YAML；也可手工添加（如 `examples/stations/robot.franka.example.yaml`）。`rwrl --station robot.yaml teleop calibrate --backend gello --port ... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8` 输出该设备的 `options`（未指定参数来自工作站 `teleop.options`；`--use` 写回工作站，见 [gello.md](gello.md#2-calibration)）：

  ```yaml
  teleop:
    name: gello
    backend: gello
    options:
      port: /dev/serial/by-id/usb-FTDI_...
      joint_ids: [1, 2, 3, 4, 5, 6, 7]
      joint_offsets_rad: [...]    # from teleop calibrate --backend gello
      joint_signs: [1, 1, -1, 1, 1, 1, 1]
      gripper_id: 8
      gripper_open_rad: ...
      gripper_closed_rad: ...
      intervention: toggle        # or always (pure teleop)
      event_device: /dev/input/by-id/...   # optional evdev pedal/keyboard on the robot PC
  ```

- `--teleop-device gello`（`GelloTeleopSource`）：GELLO 接 actor 机器，通过同一设备层后端打开；上方设备选项作为源 kwargs，训练 CLI 经 `--teleop-options-yaml` 传入。

通过 daemon 的 HIL 示例（`joint_delta` 从臂；`--robot-teleop-action-mode` 对主臂只需指定一种关节空间，用于远程连接前声明部件）：

```bash
rwrl train hil_serl --role actor --env-backend real_robot \
    --real_robot.remote ROBOT_PC:50051 --real_robot.action_mode joint_delta \
    --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
    --teleop-onset ramp --teleop-event-device keyboard
```

接管起始（`--teleop-onset`，源 kwarg `onset`）：

- `ramp`（默认）：`joint_abs` 发出目标 `q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`（`--teleop-ramp-step-rad`，0.05），归一化到关节限制，所以记录动作为执行命令；这些介入步带 `info["teleop_events"]["teleop_syncing"] = True`。归档保留，训练排除同步专家标签。`joint_delta` 下初始追赶标记到主臂首次进入 `joint_scale_rad` 范围；同一次接管之后的饱和运动是正常教学。
- `arm`：仅在 `|q_leader - q_follower|_inf <= arm_tolerance_rad`（`--teleop-arm-tolerance-rad`，0.1）时切换生效。此前步骤不算介入，逐关节误差最多每秒记录一次：需将主臂移动到从臂位置。就绪后保持直到交还；每次 reset 解除就绪。

`q_follower` 来自环境最新观测的 `joint_pos`，所以 `state_fields` 须包含它。除非事件设备提供切换，切换状态来自部件 `intervening`（机器人 PC 上 evdev `event_device` 配合 `intervention: toggle`，或 `always`）：

`--teleop-event-device keyboard`（或 `stdin`）用 `CompositeTeleopSource` 包装运动设备：运动来自 `--teleop-device`，介入状态和 `episode_end`/`success`/`failure`/`rerecord`/`operator_quit` 来自 actor 事件设备（keyboard/stdin：`t` 接管、`h` 交还、Esc 结束、`s`/`f`/`r`、`q` 退出）。主臂源仍对该状态应用起始策略，忽略部件自身 `intervention` 设置。（`--teleop-device spacemouse` 保留按钮 0 接管，仅从事件设备读取事件，见 [SpaceMouse](#spacemouse)。）

<a id="leader-arm-on-a-cartesian-robot-leader_cartesian"></a>

### 笛卡尔机器人上的主臂（`leader_cartesian`）

主臂报告关节角。仅接受笛卡尔 twist 的环境通过可选正向运动学（默认 `kinematics: panda`）使用这些角度：`LeaderArmCartesianSource`（`real_world_rl/teleop/leader/cartesian.py`，`device="leader_cartesian"`）将主臂关节转成末端请求位姿，`CartesianTargetTracker`（`real_world_rl/teleop/core/targets/cartesian_tracker.py`）再转成驱动从臂的 twist（见[映射](#mapping)）。`examples/presets/gello_cartesian.yaml` 是单臂 50 Hz 预设。

<a id="why"></a>

#### 使用原因

旧 `franka_real` 通过 SERL HTTP 桥接，只支持 `/pose`（绝对笛卡尔阻抗目标）、`/open_gripper`/`/close_gripper`、`/jointreset`、`/startimp`/`/stopimp`、`/getstate`。没有流式关节命令，所以关节空间主臂源（`--teleop-device leader_arm`/`gello`，见[主臂](#leader-arms-gello)）无法驱动它，`FrankaRealEnv` 接受 7 维动作：基坐标 twist（6）+ 夹爪。因此转换在 actor 侧执行：

```
GELLO joints q --PandaKinematics--> T* = (p*, R*) --vs follower TCP (p, R)--> twist in [-1, 1]
```

同一源也服务于 `cartesian_delta` 的 `real_robot`（如 `arm_backend: serl_http` 的 daemon）。

<a id="pieces"></a>

#### 组成

| 组件 | 位置 | 内容 |
| --- | --- | --- |
| `PandaKinematics` | `real_world_rl/kinematics/panda.py` | numpy 正向运动学，Craig modified DH；`flange_pose(q)`、`ee_pose(q)`（xyz、quat xyzw）、`ee_transform(q)`（4×4）。EE 默认 Franka Hand，沿法兰 z 轴 0.1034 m、绕 z 轴 -pi/4（`ee_offset_m`、`ee_yaw_rad`）。 |
| `LeaderArmCartesianSource` | `real_world_rl/teleop/leader/cartesian.py` | 遥操作源；任何关节数与所选运动学一致的 `joint_abs` 部件均可作为主臂。 |
| `FrankaRealEnv.last_tcp_pose` | `real_world_rl/envs/franka_real/env.py` | 最新 reset/step 实测 TCP（`O_T_EE`，xyz + quat xyzw），作为从臂位姿。 |
| 占位机器人 YAML | `examples/stations/robot.gello_station.example.yaml` | 通过机器人 daemon 从机器人主机提供 GELLO。 |

<a id="the-stub-robot-daemon-on-the-robot-host"></a>

#### 机器人主机上的占位 daemon

GELLO 接机器人主机（运行 SERL 桥接的机器）。占位机器人使用 `type: franka`、`arm_backend: fake`、无夹爪/相机，将 GELLO 作为 `teleop:`，由机器人 daemon 提供：

```bash
# on the robot host, next to the SERL bridge
pip install -e ".[gello]"                    # + [gello-events] for an evdev pedal/keyboard
rwrl --station robot.gello_station.yaml teleop calibrate --backend gello \
    --port /dev/serial/by-id/usb-FTDI_... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
    --use                                                # writes the options into the yaml
rwrl daemon --station robot.gello_station.yaml --bind 0.0.0.0:50052
```

Fake arm 不占硬件，也不接收动作，所以不与桥接竞争机器人。应使用与该主机真实 daemon 不同的端口。

<a id="actor-side"></a>

#### Actor 侧

源恰好需要一个主臂：

- `remote_address="host:port"`（可选 `remote_token`）：占位 daemon 的 `RemoteRobot`，由 `start()`（介入包装器调用）连接，`close()` 断开；
- `robot_yaml=<path>`：本机加载相同占位 YAML（GELLO 接 actor 机器），由 `start()` 连接；
- `leader=<part or resolver>`：调用者拥有的 teleop 部件（测试、自定义配置）。

从臂位姿、每步缩放、安全框来自环境：沿包装器链找到 `FrankaRealEnv.last_tcp_pose`、`cfg.action_scale`、`cfg.safety_box_low/high`；`real_robot` 笛卡尔环境则用 `last_observation["state"]["tcp_pose"]`、动作约定 `xyz_scale_m`/`rot_scale_rad` 和配置的 `safety_box`（`None` 时源不检查旧式安全框）。可用 `follower_pose_getter=`、`pos_scale_m=`、`rot_scale_rad=`、`safety_box=` 覆盖。缩放至少须等于领先上限（`xyz_scale_m >= lead_cap_m`、`rot_scale_rad >= lead_cap_rad`），否则拒绝构建；默认 0.02 m / 0.1 rad 需增大（预设中的 `--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`，或 `--franka-real.action-scale-pos`/`--franka-real.action-scale-rot`），或减小上限。

`real_robot` 中，与 Pico 一样，从环境配置读取工作站 `workspace`、`task_frame`，回退到本地机器人 YAML。显式源 kwargs 可覆盖。任务坐标系定义 workspace 位置轴与可选姿态约束。`workspace` 与基坐标 `safety_box` 同时生效。任一越界请求都会交还并保持实测从臂位姿；回到框内不会自动恢复跟随。日志为 `WORKSPACE HOLD: <axes>; the follower holds its pose and the leader handed back; toggle again to take over`。释放接管请求，再请求以重新锚定。

```python
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

env = TeleopInterventionWrapper(
    franka_real_env,
    device="leader_cartesian",
    remote_address="robot-host:50052",
    mapping="anchored",
)
```

训练 CLI 用 `--teleop-device leader_cartesian` 选择源，`--teleop-options-yaml` 传 kwargs 的 YAML 映射（合并到运动源 kwargs）：

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

默认 `intervention: always` 每步由人工驱动（纯遥操作/示范采集）。HIL 时可在机器人主机设置 `intervention: toggle` 加 `event_device`，或从 actor 事件设备读取介入（`--teleop-event-device stdin` 或 `keyboard`：`t` 接管、`h` 交还，见[主臂](#leader-arms-gello)）。

<a id="a-second-arm-as-the-leader"></a>

#### 使用第二条机械臂作为主臂

设备层 `arm_leader` Part 通过 `Arm.backend(arm.type)` 读取第二条机械臂，报告 `joint_abs` 关节位置，不发送运动命令。此路径**需要硬件验证**，包括手动引导、FK/TCP 一致性和夹爪读取。连接 Part 不会开启引导模式；须按后端/厂商流程安排主臂手动操作。

第二条机械臂主机上的机器人 YAML 可用 [`leader.franka-as-leader.example.yaml`](../../examples/stations/leader.franka-as-leader.example.yaml)。外层 `arm_backend: fake`、`gripper: null`、`cameras: []` 使机器人仅作载体，只有 teleop Part 打开真实主臂：

```yaml
type: franka
name: franka_leader_station
arm_backend: fake
gripper: null
cameras: []
teleop:
  name: leader
  backend: arm_leader
  options:
    arm: {type: franky, robot_ip: 172.16.0.2, realtime: ignore}
    gripper: false
    intervention: always
```

`arm` 必需，其余嵌套键传给该后端构造器。`gripper` 默认 `false`（无通道）；`true` 时，匹配的 `Gripper` 后端提供 `[0, 1]` 归一化宽度（1 为打开），转发 `robot_ip` 等共用构造选项，集成夹爪借用机械臂会话。`intervention: always` 默认输出 `active=True`；`intervention: event` 输出 `active=False`，由复合事件设备决定接管。两种模式自身均不提供按钮/事件。

修改示例 IP 后，在第二条机械臂主机的仓库根目录、已安装机械臂 SDK 和 daemon 依赖的 venv 中运行：

```bash
rwrl daemon --station examples/stations/leader.franka-as-leader.example.yaml --bind 0.0.0.0:50052
```

在从臂 actor 主机，将以下源选项保存为仓库根目录的 `leader_cartesian.yaml`（`leader-host` 替换为第二条机械臂主机）：

```yaml
remote_address: leader-host:50052
mapping: anchored
kinematics: panda
```

在该主机 learning venv，为从臂训练/录制命令添加 `--teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml --teleop-event-device stdin`。终端 `t` 接管、`h` 交还。无夹爪主臂保持从臂最后发送的夹爪动作。

`kinematics` 在 YAML 接受注册名，在 Python 接受运动学实例。`real_world_rl.kinematics` 导出 `Kinematics`（`NUM_JOINTS`、返回 4×4 变换的 `ee_transform(q)`）、`register(name)`、`kinematics_for(name)`、`kinematics_names()`。内置仅 `panda`；其他臂注册无参实现并匹配关节数。显式名称保存为 `teleop_input.kinematics` 来源信息，并保留在 `source_options`，不进入比较的 `teleop_control`。省略时保留现有默认快照。

回合边界门控以**释放切换 = 中立**。`session_input()` 消费一个主臂读取，不推进 tracker 或改变锚点。六轴在请求接管时为归一化请求位姿领先量（位置除以 `lead_cap_m`，旋转向量除以 `lead_cap_rad`，裁剪至 `[-1, 1]`），否则为零；`buttons["toggle"]` 携带请求电平。首次锚定前请求使用主臂 FK 位姿。即使领先为零，保持接管请求仍是非中立。

<a id="mapping"></a>

#### 映射

`cartesian_delta` 的 `RealRobotEnv` 发送 `measured + action * scale`，阻抗后端（Franky 笛卡尔阻抗 tracker）将其作为平衡位姿。因此每步从实测位姿重算 twist 永远无法保持静止：零动作会将平衡重锚到漂移后的机械臂位置（首次硬件会话 24 秒漂移 137 mm）。源改为每臂在 `CartesianTargetTracker` 中保留目标：

- **保持位姿**：在环境 reset、每次交还时采样一次当时实测位姿，无人驱动时固定；
- 持久的**命令位姿**：接管时为当前命令（保持位姿，无跳变）；人工驱动时以最多 `--teleop-target-speed-m-s` / `--teleop-target-speed-rad-s`（0.05 m/s、0.3 rad/s）靠近请求，再拉回到距实测位姿 `--teleop-lead-cap-m` / `--teleop-lead-cap-rad`（0.05 m、0.3 rad）以内。超额丢弃，不排队；主臂跳开再返回不会让从臂过冲。

发出的机械臂动作以环境归一化单位将从臂指向命令：

```
a_pos = (p_cmd - p) / pos_scale_m
a_rot = rotvec(R_cmd R^T) / rot_scale_rad
```

这是环境步的逆（`p + a_pos * pos_scale_m` 与 `from_rotvec(a_rot * rot_scale_rad) * R`：旋转左乘，所以旋转向量在基坐标系）。缩放至少为领先上限时动作不会离开 `[-1, 1]`；否则 tracker 抛 `CommandOutOfRange` 而非裁剪。交还、输入/跟踪失败、回合结束都将 tracker 设为实测位姿保持。控制循环停顿（介入轮询间超过 0.3 秒）算输入丢失：从臂保持，警告报告间隔，下次轮询像接管一样重锚主臂（时钟倒退抛 `MissedControlDeadline`）。请求位置超安全框也如此：日志 `WORKSPACE HOLD: <axes>`，读取为未介入，直到操作员交还再接管。

recorder 保持策略（`real_world_rl/teleop/core/hold.py`，`CartesianHold`）共享源 tracker：无人驱动时发送回固定保持位姿的动作（不是零 twist），每步最多一个领先上限。无 `leader_cartesian`（或同样共享 tracker 的 `spacemouse`）时使用独立 tracker，上限为 `min(0.05 m, xyz_scale_m)` / `min(0.3 rad, rot_scale_rad)`，因此机械臂被推远后会收到饱和返回动作而非错误。`--teleop-device leader_cartesian`（或 `spacemouse`）将 tracker 记录到实验约定（参与比较的 `teleop_control`：版本 `cartesian_tracker_v1` 与上方四值）；无此块或值不同的示范无法加载。块描述 tracker 而非设备，所以参数相同的 GELLO 和 SpaceMouse 示范可一起加载。

`mapping` 选择请求位姿：

- `anchored`（默认）：每次接管（未介入后首个介入轮询，以及 reset 后首个介入轮询）保存 tracker 命令 `(p_c0, R_c0)` 和主臂位姿 `(p_l0, R_l0) = FK(q_0)`。随后将主臂基坐标位移施加到命令，平移/旋转解耦：

  ```
  p* = p_c0 + (p_l - p_l0)
  R* = (R_l R_l0^T) R_c0
  ```

  接管保留保持动作，不会猛动机器人。无论两位姿多远，主臂腕部旋转只让从臂绕自身 TCP 原地旋转（组合形式 `FK(q) T_l0^{-1} T_f0` 则会让从臂绕基坐标原点摆动，平移约 `|p_f0| * theta`）。FK 误差仅通过接管后变化进入。主从姿态可不同（介入之间可像离合器一样重新抓主臂）；与任何笛卡尔命令一样，忽略主臂肘部/零空间运动。
- `absolute`：`T* = FK(q)`。从臂前往主臂末端所在位置，要求 `PandaKinematics.ee_pose` 匹配机器人 `O_T_EE`（按下文验证），且接管前主臂靠近从臂，否则从臂会按目标速度从保持位姿走到主臂位姿。

除非主臂已对照机器人标定验证，否则优先用 `anchored`。

<a id="gripper"></a>

#### 夹爪

主臂宽度目标 `w` 在 `[0, 1]`（1 为打开），当 `w > gripper_binarize_threshold` 时变为 `+1`（开），否则 `-1`（闭）；默认 0.5 使扳机超过半程后关闭。精确 +-1 是混合（离散）夹爪 critic 的清晰标签，`franka_real`（`a > 0.5` 打开）也能明确执行。`gripper_binarize_threshold=None` 则输出连续 `2w - 1`。无夹爪主臂保持环境最后夹爪动作，需要 `last_gripper_action_getter`（同 `--teleop-device robot`）。

接管时扳机与夹爪最后命令状态不一致（打开为 action > 0，由包装器经 `last_gripper_action_getter` 提供），源会持续发送最后动作，直到扳机跨到夹爪当前状态，再跟随扳机。因此扳机打开时接管不会松开已抓物体。

<a id="verify-the-dh-table-on-the-robot"></a>

#### 在真机验证 DH 表

`PandaKinematics` 匹配 Franka 公布的参考位姿（`q = 0` 时法兰 `(0.088, 0, 0.926)`、z 向下；准备位姿 `[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]` 时法兰约 `(0.307, 0, 0.590)`），但尚未对照真机。依赖 `mapping="absolute"` 前，应在几个关节配置与桥接位姿比较（准备位姿，以及工作区分布的两三个腕部旋转位姿）：

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

预期误差约低于 1 mm、0.01 rad。沿工具轴或绕工具轴的恒定偏差表示 Desk 末端设置不同于 Franka Hand 默认，可向源传 `kinematics=PandaKinematics(ee_offset_m=..., ee_yaw_rad=...)`。随关节配置变化的误差指向 DH 表本身；`anchored` 对两类误差均更不敏感。

<a id="two-arms-bimanual"></a>

### 双臂（`bimanual`）

`--env-backend real_robot_bimanual`（见 [bimanual.md](bimanual.md)）动作为 `[左臂, 右臂, 左夹爪, 右夹爪]`，拒绝单臂设备（`robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`spacemouse_twist`、`pico`）。`--teleop-device bimanual`（`BimanualTeleopSource`，`real_world_rl/teleop/core/bimanual.py`）为每臂自己的 `RealRobotEnv` 构建单臂源，各自读取机器人/从臂，再组装双臂动作。`--teleop-arm-device` 选择逐臂设备：

| `--teleop-arm-device` | 每臂输入源 | 说明 |
| --- | --- | --- |
| `leader_arm`（默认） | 该臂机器人 teleop 上的 `LeaderArmPartSource` | 各臂 daemon 提供 GELLO（YAML 的 `teleop:`）；`--robot-teleop-action-mode` 在远程连接前声明两个部件；`--teleop-onset` 参数应用于每臂 |
| `robot` | 该臂机器人 teleop 上的 `TeleopPartSource` | 任意 teleop 部件，按 `--teleop-device robot` 映射 |
| `gello` | `GelloTeleopSource` | actor 机器每臂接一套 GELLO，各需独立 `port` 和标定（见下文） |

逐臂不提供 `leader_cartesian`：它将一个主臂转换为一个笛卡尔臂的 twist。

双臂源请求介入：任一臂操作员介入时 `intervened` 为真（默认 `intervention_from: any`），或仅两者都介入时为真（`all`）。`any` 下未介入臂保持静止：机械臂块为保持动作（`joint_abs` 为从臂关节，delta 模式为零），夹爪为最后发送动作，所以空闲主臂不移动从臂，空闲扳机不打开夹爪。`episode_end` 与事件跨臂取 OR；`info["teleop_events"]` 同时带 `<arm>_<event>` 和普通 `<event>`（`success`、`teleop_syncing` 等）。

`--teleop-event-device keyboard`（或 `stdin`）使双臂源作为复合源运动部分：`t`/`h` 同时接管/交还两臂（各臂起始策略仍生效），键盘提供回合结束、结果、退出键。

`bimanual` 的 `--teleop-options-yaml` 可为各臂名称段加公共键的映射（`intervention_from` 属于双臂源自身，其余公共键传给每臂，各臂段覆盖）：

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

或在 `--teleop-arm-device gello` 下直接使用工作站 YAML（`rwrl setup` 写出的 `robots: {left: ..., right: ...}`）：各机器人 `teleop.options`（及 `hold_s`）成为对应臂 kwargs，按机器人名匹配环境 `arm_names`。单机器人工作站/机器人 YAML 对单臂 `--teleop-device gello` 也如此。逐臂 `leader_arm` 或 `robot` 时，各 daemon 自行读取 GELLO，因此拒绝工作站 YAML。

通过两个 daemon 进行 HIL：

```bash
rwrl train hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

`--teleop-device fake` 也声明环境臂数，可无操作员演练双臂 actor。

## `InterventionWrapper`

位于 `real_world_rl/envs/wrappers/intervention.py`。`InterventionWrapper(env, source)` 接受 `TeleopSource`；`TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)` 按设备名构建（或适配传入 `teleop`）。两者通过 `.source`（别名 `.teleop`）暴露输入源。

`step(action)` 先轮询源，将源人工掩码与所选部件取交集。人工掩码非空时，将人工与空闲部件动作组为策略设备上的 `(1, A)` float32 tensor。`idle_behavior="hold"` 保持未选部件；`"policy"` 保留策略建议。实际人工控制与保持覆盖分别记录：

| `info` 键 | 值 |
| --- | --- |
| `intervene_action` | 执行的 `(1, A)` 动作；**仅介入时存在** |
| `policy_action` | 收到的原始策略建议动作 |
| `intervened` | Python bool：是否执行人工动作 |
| `intervention_onset` | Python bool：本步介入而本回合上步未介入 |
| `human_episode_end` | Python bool：操作员回合结束按钮 |
| `teleop_events` | 已触发 `success`/`failure`/`rerecord` 边沿及主臂追赶 `teleop_syncing` 字典；**仅有事件时存在** |
| `human_action_mask` | 实际人工驱动维度，含同步期间来源 |
| `override_mask` | 替代策略建议的维度，含保持补充 |

`episode_end` 让 `step()` 返回 `truncated=True`（不是 `terminated`），actor 循环 reset，但不当作终止状态。

事件顺序：

- 构建：对照环境约定验证源，再 `source.start()`。
- `reset(**kw)`：先 `env.reset()` 再 `source.reset()`，丢弃归位期间按键。“最后夹爪动作”恢复打开（+1），起始检测器重启。
- `close()`：先 `source.close()` 再 `env.close()`（即使源关闭失败也关闭环境）。

包装器记住最后发送的夹爪动作（裁剪到 `[-1, 1]`）；环境有夹爪但设备无夹爪时保持它，避免松开物体。

<a id="adding-a-device"></a>

## 新增设备

继承 `TeleopSource`、声明动作空间、实现 `poll()` 并注册：

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

`make_teleop_source("joystick", env, port=...)` 随后调用 `JoystickTeleopSource.from_env(env, port=...)`，默认执行 `cls(**kwargs)`；可覆盖 `from_env` 读取环境动作约定（`real_world_rl.action_contract.get_env_action_contract(env)`）。未知名称抛 `ValueError` 并列出已注册名称。注册须先于使用，下游包须在构建环境前导入模块（或通过自己的入口点暴露）。内置源首次使用名称时才导入，因此 `import real_world_rl.teleop` 不加载 `zmq` 或 `hid`。

动作在环境归一化 `[-1, 1]` 空间中，夹爪在最后。物理单位 twist 的笛卡尔设备（如 `pico_twist`）可参考示范脚本归一化方式。

<a id="capabilities"></a>

### 能力声明

介入包装器、保持策略、会话门控、训练环境构建器从类而非名称了解源能力：通过 `real_world_rl.teleop.core.registry.source_class(name, options)` 取得类，不构建源。只设置与默认不同的类属性：

| 类属性（默认 `False`） | 何时设为 `True` | 效果 |
| --- | --- | --- |
| `USES_LAST_GRIPPER_ACTION` | 接受 `last_gripper_action_getter`（保持或从最后夹爪动作开始） | 包装器传入 getter（`composite` 转给这样的运动部分） |
| `REQUIRES_FULL_ACTION` | 同时驱动所有臂与夹爪维度 | 拒绝部分 `--teleop-selected-parts` |
| `DRIVES_ONE_ARM` | 恰好驱动一臂 | `--env-backend real_robot_bimanual` 拒绝（须用 `--teleop-device bimanual`） |
| `TRACKS_CARTESIAN_TARGET` | 通过 `CartesianTargetTracker` 移动从臂，并暴露为 `cartesian_tracker` | 接收 `--teleop-lead-cap-*`/`--teleop-target-speed-*` kwargs，保持策略共享 tracker |
| `RECORDS_TELEOP_INPUT` | 需要将解析选项冻结到运行 `teleop_input` | 恢复复用快照，不重读选项文件 |
| `SUPPORTS_INPUT_DIAGNOSTICS` | 接受 `diagnostic_axes`/`diagnostic_gain` | 允许 `--teleop-diagnostic-axes`/`--teleop-diagnostic-gain` |

三个 classmethod 回答构建源前的问题：`resolved_class(options)`（这些选项下 `from_env` 构建的类；SpaceMouse `mode: serl` 构建 SERL 源）、`options_from_station(station_path, station, robot=, profile=, provenance=)`（工作站 YAML 配置的源 kwargs，默认无）、`control_spec(tracker_options=, source_options=)`（恢复时比较的 `teleop_control` 块，默认 `None`）。回合边界门控（`--episode-end-behavior hold`）需要运动源提供 `session_input()`（`SessionInputSource` 协议：一个新鲜 `(axes, buttons)` 读取，不移动目标）。

## Pico

`--teleop-device pico`（`PicoTeleopSource`，`real_world_rl/teleop/pico/source.py`）从 Pico VR 控制器驱动 `cartesian_delta` 环境（该模式的 `real_robot` 或 `franka_real`）。Pico 是**位姿**设备：手位姿锚定到与 `leader_cartesian`、`spacemouse` 相同的 `CartesianTargetTracker`（见[映射](#mapping)），从臂接管不跳变，领先实测位姿不超过领先上限，移动不超过目标速度。设备本身是设备层 teleop 部件 `PicoTeleop`（`backend: pico`，`real_world_rl/teleop/pico/backend.py`），也可放在机器人 YAML 的 `teleop:`、通过 daemon 提供，并出现在 `rwrl doctor`（`pico` 扩展、`pyzmq`）。

`--teleop-device` 无默认值：所有构建源的 actor、eval、`rwrl record`、奖励采集与 deploy 命令须指定设备（`--teleop-device {spacemouse,gello,leader_arm,leader_cartesian,pico,keyboard,robot,bimanual,fake,spacemouse_twist}`），或在 `--config` 预设中设置 `teleop_device`。两者都没有时以用法错误停止，不再打开 Pico（旧默认）。Learner 不构建源，无需设备。

旧原始 twist Pico 源为 `pico_twist`（`real_world_rl/teleop/pico/twist_sim.py`），仅供 ManiSkill 仿真示例（见[基本用法](#basic-usage)）。

<a id="message-format"></a>

### 消息格式

Pico 侧在 ZeroMQ `PUB` socket（如 `tcp://*:7777`）发布 JSON 数组：单手 `13` 值，双手 `26` 值、左手在前。带 `topic` 时发送两帧 `[topic, json]`。每手：

| 索引 | 含义 |
| --- | --- |
| `0:3` | 手位置（米，Pico 跟踪坐标系） |
| `6:10` | 手四元数 `[x, y, z, w]` |
| `10` | 扳机（`0` 释放至 `1` 扣下） |
| `11` | bind 按钮 |
| `12` | 回合结束按钮 |

发布者须持续流送：部件每条消息发布一个样本，静默超过 `read_timeout_s`（1 秒）会使下次读取抛 `DeviceFault`，与其他 teleop 部件一致。

<a id="the-device-part-backend-pico"></a>

### 设备部件（`backend: pico`）

| 选项（`TeleopConfig.options`） | 默认值 | 含义 |
| --- | --- | --- |
| `address` | `tcp://localhost:7777` | `SUB` socket 连接的 Pico 发布者 |
| `topic` | `""` | 订阅前缀，空表示所有消息 |
| `hand` | `right` | 选择双手消息中的哪条记录 |
| `bind_button` | `bind` | 接管按钮（`bind` 索引 11，或 `episode_end` 索引 12）；另一个结束回合 |
| `poll_interval_s` | `0.005` | 轮询线程周期 |

`hold_s`（`TeleopConfig.hold_s`，默认 0）在 bind 释放后延长 `intervening`。线程每隔 `poll_interval_s` 读取上次轮询后排队的全部消息：按键取自所有消息（短于轮询周期也不丢失），位姿取最新。样本（`rwrl teleop read` 读取内容）包含：

| 样本字段 | 值 |
| --- | --- |
| `diagnostics["hand_pose"]` | Pico 跟踪坐标手位姿 `xyz + quat_xyzw`（daemon 也传输） |
| `action` | 相对前样本的手位移 `[dp, rotvec]`，未缩放、裁剪至 `[-1, 1]`（保持部件 `cartesian_delta` 形状；`pico` 源不读取它） |
| `gripper` | `1 - trigger`：`1` 打开（扳机释放）到 `0` 关闭 |
| `buttons` | `bind`、`episode_end`、`trigger` 电平 |
| `events` | `bind`、`episode_end` 按下，锁存到下次 `read()` |
| `intervening` | bind 按住（加 `hold_s`） |

连接或 `reset()` 后首条消息仅设置按钮基准，所以跨越它的长按不算按下。部件连接时才导入 `pyzmq`；`discover()` 无结果（网络发布者没有本地设备节点）。

<a id="take-over-and-the-anchored-mapping"></a>

### 接管与锚定映射

| 输入 | 效果 |
| --- | --- |
| bind 按下 | 接管，下次按下交还；仅移动手不介入。 |
| 驱动时手运动 | `p* = p_c + (p_l - p_l0)`，`R* = (R_l R_l0^T) R_c`（`pico_anchored_v1`）：`(p_c, R_c)` 是 tracker 命令，`(p_l0, R_l0)` 是接管时手位姿，`p_l = B p_hand`、`R_l = B R_hand`，其中 `B = input_rotation` |
| 扳机 | `1 - trigger > gripper_binarize_threshold`（0.5）时为 `+1`（开），否则 `-1`；`gripper_binarize_threshold: null` 输出连续 `2w - 1`。接管时与最后夹爪命令不一致的扳机会被忽略，直到一致，因此接管不松物。 |
| 回合结束按下 | 结束回合并交还 |
| 请求超安全框或工作站 workspace | 日志 `WORKSPACE HOLD: <axes>`，保持实测位姿并交还，下次 bind 再接管 |
| 控制循环停顿（> `max_dt_s`） | 保持实测位姿，下次轮询在此重锚手位姿 |
| 读取失败 | 向外传播并交还 |

交还保持实测位姿，无人驱动时 recorder 保持策略共享 tracker。再次接管锚定当时手位置，所以交还、移动、接管构成离合操作。`--teleop-event-device stdin` 或 `keyboard` 添加结果标记（`s`/`f`/`r`）、回合结束（Esc）、退出（`q`）；bind 仍是唯一接管来源（`t`/`h` 警告）。

`input_rotation`（`base_R_input`，合法 3×3 旋转，默认单位阵）将 Pico 跟踪坐标转到机器人基坐标。跟踪坐标取决于 Pico 应用及头显重新居中的位置；首次真实接管前须调试：以小 `--teleop-target-speed-m-s`（如 0.01）接管，手向前、左、上各移动约 5 cm，检查从臂沿基坐标 `+x`、`+y`、`+z` 运动。`input_rotation` 第 `i` 列是 Pico 第 `i` 轴在基坐标的方向。平移 1:1，无运动缩放。

与 `leader_cartesian` 相同，运行比较 tracker 块（`teleop_control`、`cartesian_tracker_v1`，来自 `--teleop-lead-cap-*`/`--teleop-target-speed-*`），不比较 Pico 自身选项。每次读取 diagnostics 注明映射（`target_mapping: pico_anchored_v1`）并携带原始手位姿；`real_robot` 诊断 JSONL 启用时记录它们。两者都是来源信息，从不参与比较。

<a id="running-it"></a>

### 运行

在 actor（或 recorder）机器仓库根目录、项目环境已激活时，安装一次扩展：

```bash
pip install -e '.[pico]'
```

接触机器人前检查数据流（无环境，输出 bind/回合结束及旧桥接手 twist）：

```bash
python examples/run_twist_input_inspector.py --zmq-url tcp://192.168.6.2:7777
```

将 Pico 选项写入 `local/pico.yaml`（源 kwargs 平铺映射；tracker 键来自参数，不在此文件）：

```yaml
# local/pico.yaml, for --teleop-device pico --teleop-options-yaml local/pico.yaml
address: tcp://192.168.6.2:7777
hand: right
# base_R_input from the commissioning above; identity when omitted
# input_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
```

随后在同机仓库根目录、项目环境激活后，使用工作站 `hil.yaml` 配置的 `cartesian_delta` 环境（`xyz_scale_m >= 0.05`、`rot_scale_rad >= 0.3`，默认领先上限），终端输入标记：

```bash
rwrl record --config hil.yaml \
    --teleop-device pico --teleop-options-yaml local/pico.yaml --teleop-event-device stdin \
    --output-path demos/task_20.pkl --num-successes 20
```

HIL actor 接受相同三个参数（`rwrl train hil_serl --role actor --config hil.yaml --teleop-device pico --teleop-options-yaml local/pico.yaml --teleop-event-device stdin`）。

**通过机器人 daemon。** `real_robot` 工作站可将 Pico 作为机器人 `teleop:` 部件，在机器人运行处读取（本地或机器人 PC 由 `rwrl daemon` 提供，与其他部件相同）：

```yaml
# in the station robot's yaml
teleop:
  backend: pico
  hold_s: 0.0
  options:
    address: tcp://192.168.6.2:7777
    hand: right
```

此时 actor 的 `local/pico.yaml` 指定读取环境机器人部件，不再打开自己的订阅者：

```yaml
robot_teleop: true
```

`franka_real` 中，提供 Pico teleop 的机器人 daemon 可通过 `remote_address: <robot-host>:<port>`（及 `remote_token`）读取，同 `leader_cartesian`。

<a id="basic-usage"></a>

## 基本用法

ManiSkill 仿真可视化遥操作，使用原始 twist Pico 源 `pico_twist`（旧 `pico`；仅仿真，真机用 `--teleop-device pico`，见 [Pico](#pico)）：

```bash
python examples/run_teleop_demo.py \
  --env-id PegInsertionSidePegOnly-v1 \
  --device pico_twist \
  --zmq-url tcp://192.168.6.2:7777
```

不连接环境，仅输出收到的遥操作状态：

```bash
python examples/run_twist_input_inspector.py \
  --device pico \
  --zmq-url tcp://192.168.6.2:7777
```

相同示例使用 SpaceMouse（`--spacemouse-index`、`--spacemouse-deadzone`、`--spacemouse-invert-axes`、`--spacemouse-hold-s` 调整）或空闲 `fake`：

```bash
python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse_twist
```

仿真示例用 `spacemouse_twist` 原始 twist 源：移动操纵帽算介入，twist 原样送环境，按钮 0 切换夹爪、按钮 1 结束回合。仅用于仿真；真机用 `--teleop-device spacemouse`（见 [SpaceMouse](#spacemouse)）。

`run_teleop_demo.py` 创建 ManiSkill 环境，经 `make_teleop_source` 构建源、轮询动作，对 `pico_twist` 物理 twist 应用控制器动作归一化（SpaceMouse 已归一化），再推进环境。

<a id="recording-demos-on-the-real-robot"></a>

## 在真机上录制示范

`examples/record_real_demos.py` 在 HIL 的 `real_robot`、`real_robot_bimanual` 或 `franka_real` 环境（与 actor 相同环境/遥操作参数）录制遥操作回合，保存为 learner 经 `--demo-dataset-paths` 加载的示范 pkl，仅保留介入步骤。见 [recording-demos.md](recording-demos.md)；GELLO 见 [gello.md](gello.md#5-recording-demos)。

<a id="recording-wsrl-data"></a>

## 录制 WSRL 数据

`examples/record_teleop_wsrl.py` 将遥操作回合录成与 WSRL 数据生成器相同的 H5 格式。

示例：

```bash
python examples/record_teleop_wsrl.py \
  --output-path demos/peg_teleop_wsrl.h5 \
  --env-id PegInsertionSidePegOnly-v1 \
  --obs-mode rgb \
  --control-mode pd_ee_twist \
  --device pico_twist \
  --zmq-url tcp://192.168.6.2:7777
```

`--output-path` 必须为 H5 文件路径，不能是目录，建议 `.h5` 后缀。以写模式打开，同路径已有文件将覆盖。

仅 `intervened=True` 样本推进环境并保存；`intervened=False` 时继续轮询设备，不记录 transition。

操作员回合结束按钮（`pico_twist` 的 Pico 回合结束按钮，或 `spacemouse_twist` 按钮 1）结束当前回合。每回合后终端输出回合长度、`max_episode_steps`、成功标志、回报，再询问是否保存。

<a id="important-arguments"></a>

### 重要参数

- `output_path`：写入的 H5 数据集文件。
- `device`：`pico_twist`（默认）、`spacemouse_twist` 或 `fake`（与示例相同的设备参数）。
- `dt`：轮询周期，秒；默认 `1 / 30`，约 `30 Hz`，越小轮询与环境步越快。
- `env_id`：ManiSkill 环境 ID。
- `obs_mode`：观测模式，通常 `state`、`rgb`、`rgbd`。
- `include_state`：视觉观测中加入本体状态。
- `control_mode`：应匹配 EE twist 控制器，通常 `pd_ee_twist`。
- `sim_backend`、`render_backend`：ManiSkill 仿真/渲染后端。
- `reward_mode`：传入环境的奖励模式。
- `robot_uids`：环境机器人 ID。
- `fix_box`、`fix_peg_pose`、`peg_density`、`debug_pose_vis`：自定义 peg 环境选项。
- `env_kwargs_json`：传递额外环境 kwargs。
- `end_on_env_done`：为 true 时环境终止、截断、成功可结束回合，默认 false，仅遥操作回合结束停止录制。
- `pos_scale`、`rot_scale`、`twist_limit`：可选 twist 映射覆盖。
- `intervention_threshold`：判断人工输入激活的 twist 范数阈值。

保存的 H5 包含 `traj_*` 组：

```text
obs
actions
rewards
terminated
truncated
```

该文件可通过 `--offline_dataset` 传给 WSRL、CQL、Cal-QL 离线训练。

真机数据仅做离线预训练（无仿真环境、无 eval）时，用 rl-garden 的 `examples/pretrain_offline.py cql` 或 `examples/pretrain_offline.py calql` 生成独立离线 CQL/Cal-QL checkpoint。若后续由 WSRL 离线→在线流程恢复，则用 `wsrl` 子命令。此入口从 H5 推断观测/动作规格，生成可在兼容实时环境训练中加载并在线微调的预训练 checkpoint。完整流程见 `rl-garden/` 子模块的 `rl-garden/docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env`。

<a id="partial-control-and-training-provenance"></a>

## 局部控制与训练溯源

用 `--teleop-selected-parts` 选择部件（教程工作站为 `arm.arm` / `arm.gripper`，双臂为 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`）。默认 `--teleop-idle-behavior hold` 保持未选部件，`policy` 让它们自主动作。空选择使用源可用部件。无夹爪通道的源从不提供专家夹爪标签。支持连续无夹爪 HIL；混合/离散分支需要夹爪。同步行仍执行/归档，但训练投影无专家监督。掩码 BC/SiLRI/残差损失、显式结果和持久记录见[真机实验](real-world-experiments.md)。
