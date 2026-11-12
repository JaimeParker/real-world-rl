<!-- English source: docs/cli.md -->

<a id="rwrl-station-command-line"></a>

# rwrl：工作站命令行

`rwrl` 统一训练、录制、部署和工作站操作。硬件原语可用于：检查工作站、保存 home 姿态、回 home、操作夹爪、停止机械臂或恢复错误。人和 agent 使用同一套命令；`--json`、`--yes` 和退出码使内置原语适合非交互调用。

<a id="where-to-run"></a>

## 在哪里运行

安装 `real-world-rl` 包及工作站所需的 `robot_infra` 依赖后，该环境中即可使用 `rwrl`，等价于 `python -m real_world_rl.cli`。见[安装教程](tutorial/01-install.md)。硬件命令在连接设备的**机器人主机**上运行，并**激活项目环境**。NUC 只是可选机型，命令不依赖特定电脑；支持源码检出和已安装的包。工作站和预设的相对路径相对于当前工作目录，源码检出便于运行仓库相对路径示例。每次原语调用都会连接设备，并在结束后断开，失败时也会断开。

全局选项必须放在**命令之前**，包括将 `--preset` 放在 `home` 或 `doctor` 之前。`rwrl --help` 查看包装层帮助；`rwrl doctor --help`、`rwrl setup --help`、`rwrl daemon --help`、`rwrl supervise --help` 显示各 CLI 的帮助。

<a id="global-options"></a>

## 全局选项

| 选项 | 默认值与含义 |
|---|---|
| `--station PATH` | 解析顺序为显式路径 → `$ROBOT_INFRA_CONFIG` → `local/station.yaml` → `~/.config/robot_infra/station.yaml`。设置了 `$XDG_CONFIG_HOME` 时，最后一个位置随之改变。显式和环境变量路径直接采用；最后两个位置选择首个存在的文件。 |
| `--robot NAME` | 默认未设置：选择唯一机器人；多机器人工作站调用原语必须指定名称。 |
| `--preset PATH` | 默认未设置，使用工作站 home。`home` 命令读取现有环境预设，支持裸映射或 `real_robot` 下的配置。`home set` 必须提供此选项；`doctor` 也用它确定预期设备，并将其中的机器人与工作站比较。 |
| `--direct` | 默认关闭，遵循工作站的 daemon 声明。仅在其 daemon 端口不可达时强制直连。 |
| `--mock` | 默认关闭；对**直连**使用 doctor 的 fake 后端替换设备后端。仍需工作站文件，也不会绕过 daemon 选择。 |
| `--json` | 默认关闭；内置原语在 stdout 输出一个 JSON 结果，诊断、运动目标和确认提示写入 stderr。 |
| `--yes` | 默认关闭；跳过内置运动确认或 doctor 修复计划确认；前置检查仍然有效。 |

不提供 JSON 输出的命令（`setup`、`daemon`、`supervise`、`gello-calibrate`
及学习命令）会拒绝 `--json`，用法错误退出码为 2，见下文。

<a id="learning-commands"></a>

## 学习命令

`train`、`record`、`deploy` 和 `collect-reward` 分别调用现有训练、示范录制、部署和奖励数据采集入口。包括 `--help` 在内的参数属于各入口，会原样传递。
全局 `--preset PATH` 为学习命令在参数前添加 `--config PATH`；两者同时指定会报用法错误。全局 `--station`、`--robot`、`--direct`、`--mock`、`--json` 和 `--yes` 不适用于学习命令，使用时返回退出码 2。请将入口自身的参数放在子命令后。

在训练主机的仓库根目录激活项目环境后，使用 `rwrl train --help` 查看算法参数解析器，不启动训练。

原有四个 console script、Python 模块入口和示例脚本继续兼容。训练参数仍由训练入口定义，rwrl 层不定义或重新解释它们。

<a id="connection-ownership"></a>

## 连接归属

若所选工作站机器人声明了 `daemon:`，`rwrl` 以 0.5 秒 TCP 超时探测其 bind 端口，可达时使用 daemon。客户端将通配地址 `0.0.0.0` 转成 `127.0.0.1`。此时 status 首行为 `connection: daemon HOST:PORT`。若声明了 daemon 但不可达，默认报错并以 1 退出，提示启动 `rwrl daemon` 或传入 `--direct`；不会自动退回直连。

没有 `daemon:` 时直接连接。使用 franky 的 Franka 工作站显示 `connection: direct franky`；mock 直连显示 `direct fake`。声明的 daemon 端口可达时，`--direct` 被拒绝并以 2 退出：`error: the daemon owns the arm (hint: omit --direct)`。

Franka FCI 只允许一个控制连接。直连路线中，先结束 recorder/actor，再用 `rwrl` 打开另一个硬件连接；daemon 路线使用同一个已声明 daemon。`stop` 和 `recover` 也遵循相同连接选择，不能绕过直接持有机械臂的进程。

<a id="commands"></a>

## 命令

下表的“确认”指包装层运动提示。读取原语仍会打开配置的设备。内置原语成功退出码为 0，运行时或设备失败为 1，用法、配置选择错误或拒绝确认等为 2；细节见后文。

| 命令 | 作用 | 机器人动作 | 确认 | 退出码 |
|---|---|---|---|---|
| `status` | 连接路径、机器人描述、臂健康、夹爪反馈、相机名称、teleop 名称；远程时含 daemon RTT。文本显示臂的 `ok`、控制模式和错误；`arm_health.robot_mode`、`detail`、`last_feedback_age_s` 及安全裁剪/拒绝计数仅在 `--json` 中出现 | 读取 | 无 | 0 / 1 / 2 |
| `q` | 当前关节位置，单位弧度 | 读取 | 无 | 0 / 1 / 2 |
| `tcp` | TCP 位置（米）及 `xyzw` 顺序四元数 | 读取 | 无 | 0 / 1 / 2 |
| `home show` | 解析后的 home、当前关节、有符号差值和最大绝对差值 | 读取 | 无 | 0 / 1 / 2 |
| `home set` | 将实测关节写入指定预设 | 读取；写文件 | 无 | 0 / 1 / 2 |
| `home go` | 阻塞运动到 home，再执行配置的 reset 夹爪动作 | 臂运动，可能还有夹爪运动 | 需要，或 `--yes` | 0 / 1 / 2 |
| `gripper show` | 米制及归一化宽度、抓取状态、后端来源 | 读取 | 无 | 0 / 1 / 2 |
| `gripper open` | 张开到归一化宽度 1 | 夹爪运动 | 需要，或 `--yes` | 0 / 1 / 2 |
| `gripper close` | 闭合到归一化宽度 0 | 夹爪运动 | 需要，或 `--yes` | 0 / 1 / 2 |
| `gripper move F` | 设归一化宽度为有限数 `F`，满足 `0 <= F <= 1` | 夹爪运动 | 需要，或 `--yes` | 0 / 1 / 2 |
| `stop` | 调用臂 `stop()`；成功打印 `stop: ok` | 停臂 | 无 | 0 / 1 / 2 |
| `recover` | 调用臂 `recover()`；成功打印 `recover: ok` | 清除臂错误 | 无 | 0 / 1 / 2 |
| `camera show` | 全部工作站相机，或仅 `--name NAME`：配置、在 `--frames N` 次读取上实测的帧率（默认 30；`0` 不读取并报告 "not measured"）、帧龄、图像形状、dtype 和健康状态 | 读取；与 `status` 一样打开机器人连接 | 无 | 0；某个相机无帧或不健康时为 1（其他相机照常报告）；2 |
| `camera grab --name NAME --output DIR` | 保存一帧为 `NAME-color.png`、`NAME-depth.png`（相机提供深度时；16 位，浮点深度以毫米存储）及 `NAME.json` sidecar；创建 `DIR`，从不覆盖已有文件；`--timeout-s S`（默认 2） | 读取；写文件 | 无 | 0 / 1 / 2 |
| `doctor` | 检查主机与已配置设备，预览针对性修复 | 读取；`--fix` 修复主机/软件，不恢复机器人或执行运动 | 整份修复计划确认一次，或 `--yes` | 0 / 1 / 2 |
| `setup` | 发现并配置工作站 | 发现、标定和 doctor | 向导原有提问 | 原样保留原 CLI 状态 |
| `daemon` | 通过网络提供机器人服务 | 预检查，为客户端运动提供硬件服务 | 无包装层提示 | 原样保留原 CLI 状态 |
| `supervise` | 监督机器人进程 | 取决于子命令 | 取决于子命令 | 原样保留原 CLI 状态 |
| `teleop read` | 打印工作站 teleop（GELLO 或 SpaceMouse）的实时样本：`q` 或 `twist`、夹爪、intervening、已触发事件；按 `--rate-hz`（默认 3）每个样本一行文本，直到 Ctrl-C 或 `--duration-s`，最后一行汇总。`--json` 打印 JSON 行，结尾为 `{"summary": ...}`。直连路线只打开 teleop 部分；`--with-follower`（仅 daemon 路线）增加 follower 的 `follower_q`，对 `joint_abs` leader 还增加逐关节 leader 减 follower 的 `delta` | 仅输入；无机器人运动 | 无 | 0 / 1（设备失败或 teleop 不健康）/ 2 |
| `teleop calibrate` | 标定工作站 teleop；`--backend spacemouse\|gello`（默认：工作站 teleop 后端）。SpaceMouse 拟合并验证方向标定档案（必须提供 `--profile`；见 [spacemouse-commissioning.md](spacemouse-commissioning.md)）；GELLO 运行引导式主臂标定，未设置的 `--port`、`--joint-ids`、`--gripper-id`、`--baudrate`、`--joint-signs` 取自工作站 `teleop.options`，wrap 中心取自工作站 `joint_limits`，并打印 `teleop:` YAML 块（[gello.md](gello.md#2-calibration)）。使用另一后端的标志为用法错误 | 仅输入；可选写文件 | 保存/启用确认；`--use` 选择标定档案（SpaceMouse），或将标定后的 `teleop.options` 写入工作站（GELLO） | 0 / 1 / 2 |
| `teleop profiles list/show/use` | （SpaceMouse）检查或选择具名标定档案（profile）；见 [spacemouse-commissioning.md](spacemouse-commissioning.md) | 无运动；use 写入工作站 | use 时确认保存 | 0 / 1 / 2 |
| `teleop diagnose` | （SpaceMouse）仅输入诊断；见 [spacemouse-commissioning.md](spacemouse-commissioning.md) | 无机器人运动 | 无 | 0 / 1 / 2 |
| `teleop run` | （SpaceMouse）带回合保持的受监督录制；`--diagnose translation\|rotation\|all` 启用运动诊断；见 [spacemouse-commissioning.md](spacemouse-commissioning.md#supervised-recording-sessions) | reset 与遥操作 | 需要，或 `--yes` | 子进程状态或包装层错误 |
| `workspace show/capture/set/validate` | 检查、采样或配置约束；见 [spacemouse-commissioning.md](spacemouse-commissioning.md)。`show` 还报告 `joint_limits`、`max_delta_per_step`、`home_joints` 和 `home_within_limits`（无 home 或限位时为 `null`；`home_violations` 列出每个超出限位的关节） | capture 读取 TCP；无运动 | 涉及保存时确认 | 0 / 1 / 2；文本模式验证失败为 5 |
| `workspace frame set --name NAME` | 由 `--base-T-frame`（16 个行优先数字，单位米）或 `--from-points PTS --origin A --x-axis B --y-axis C`（点名由 `workspace capture --save` 保存；右手系，y 相对 x 正交化，退化点被拒绝）提出工作站 `task_frame`；`--source TEXT` 记录来源；`--save` 只写入 `task_frame`。工作站工作区位于另一坐标系时拒绝；同名坐标系会对该工作区重新定基并予以说明 | 无运动；`--save` 写入工作站 | 确认保存 | 0 / 1 / 2 |
| `workspace frame show` | 工作站 `task_frame`，含其在基座坐标系中的原点和单位轴 | 只读取工作站文件 | 无 | 0 / 1 / 2 |
| `teleop inspect-stop` | 离线评估已保存的停止证据；见 [spacemouse-commissioning.md](spacemouse-commissioning.md#live-input-calibration-and-diagnosis) | 无硬件连接 | 无 | 0 PASS / 3 FAIL / 4 BLOCKED；其他情况为错误码 |
| `archive inspect ARCHIVE` | 只读读取 recorder SQLite 归档：结果、掩码计数、保持漂移、每个 human_action_mask 片段的接管跳变和残差，以缩进 JSON 打印。`--csv PATH` 和 `--measurements PATH` 写出新的逐步和测量 CSV；`--drift-window-s`（默认 10）设置保持漂移窗口；每个 `--stop-time-s T`（可重复）增加一次稳定评估，使用 `--settle-speed-mm-s`（默认 2）和 `--settle-hold-s`（默认 0.5）。`--json` 在一行内打印相同字段，非有限数为 `null`，另加 `status`（`REPORT`、`PASS`、`FAIL` 或 `BLOCKED`）。不加载工作站：`--station`、`--robot`、`--preset`、`--direct`、`--mock` 和 `--yes` 为用法错误。`python examples/inspect_teleop_archive.py` 仍是参数和输出相同的别名，但其退出码忽略稳定状态 | 无硬件连接 | 无 | 无 `--stop-time-s` 时为 0；有则 0 PASS / 3 FAIL / 4 BLOCKED（停止时间没有覆盖它的 TCP 回合）；用法错误或归档不可读为 2 |

`setup`、`daemon`、`supervise`、`discover` 和 `gello-calibrate` 是 `robot_infra` 提供的原生命令，也可通过 `robot-infra` 使用。两个 CLI 使用同一组注册命令对象和实现。
工作站输入统一使用 `--station PATH`；daemon、discover 和设备 doctor 暂时保留 `--config PATH`，setup 暂时保留 `--from PATH`，帮助中均标为 "(alias, will be removed)"。
对于 setup，工作站输入提供已有默认值，**不是输出路径**。命令后的局部值优先于根级 `--station` 值；工作站查找顺序保持不变。
`--mock` 和 `--robot` 保留适用的设备语义；其他不相关的全局选项不配置这些命令。`setup`、`daemon`、`supervise` 和 `gello-calibrate` 没有 JSON 输出，使用 `--json` 会报用法错误（退出码 2），而不是默默输出文本。supervisor 的 `--` 之后的参数属于子命令。

`discover` 输出可见硬件，不连接机器人主机。使用 `--json` 时，它打印一个含 `schema_version: 1` 和六个列表的对象，未发现时各列表为空：`cameras`（`backend`、`serial`）、`teleops`（`backend`、`device`）、`can_interfaces`（`name`、`operstate`、`bitrate`）、`hosts`（`ip`、`ping_ok`、`open_ports`）、`hid_devices`（路径）和 `unknown_hid`（`vid`、`pid`、`product`、`path`、`message`；见[站点设备表](spacemouse-commissioning.md#site-device-table)）。仅当 `unknown_hid` 非空时，文本输出才多出 `warnings:` 一段。

`setup` 接受三个逐臂相机流标志，每臂一个值或一个值用于全部臂：`--camera-resolution WIDTHxHEIGHT`（例如 `1280x720`）、`--camera-fps N` 和 `--camera-depth yes|no`。它们作用于该臂的相机；未设置时，新相机使用相机默认值，从 `--station` 保留的相机保持原值。它们不增加向导提问。

`rwrl gello-calibrate` 是 `rwrl teleop calibrate --backend gello` 的别名，将被移除；它保留设备命令的标志（`--port` 和 `--joint-ids` 必填，不使用工作站默认值）。`robot-infra gello-calibrate` 不变。两者都打印 GELLO `teleop:` YAML 块。
`setup --only teleop --calibrate` 在向导内执行该标定并写入工作站 YAML。
`robot-infra doctor` 仍是设备层诊断；`rwrl doctor` 是下文的主机与项目诊断。

Doctor 直接注册在 `rwrl` 解析器中；`rwrl doctor --help` 显示自身帮助。它共用全局 `--station`、`--robot`、`--preset`、`--mock`、`--json` 和 `--yes`。
为兼容旧用法，doctor 后的 `--config PATH` 是 `--station PATH` 的别名；`--robot`、`--preset`、`--mock`、`--json`、`--yes` 和 `--station` 也可放在 doctor 后。同一选项同时出现在命令前后时，后面的值优先。
`python -m real_world_rl.cli.doctor` 保留为过渡入口，委托给 `rwrl doctor`。两种 doctor 的 `--yes` 都只跳过确认，仍运行全部预检查，即使没有请求修复。
两种 JSON 报告共用 `schema_version: 1`、`status`（`PASS` 或 `FAIL`）、`checks` 和 `repairs`，保留 rwrl doctor 的既有字段名。设备 doctor 原有的列表/映射或安装 SDK 对象包装在 `results` 下，其主机专用 `checks` 和 `repairs` 为空列表。
Python doctor、setup、daemon 和监督接口见[设备层 README](robot-infra.md#3-check-with-the-doctor)。

<a id="robot-host-diagnosis-and-repair"></a>

## 机器人主机诊断与修复

在机器人主机激活项目 venv 或非 base conda 环境。支持源码检出与已安装的包，不要求位于仓库根目录。全新机器应先按[安装教程](tutorial/01-install.md) 安装 Python、环境和项目；doctor 修复该环境，不负责引导安装 Python。主机与 USB 发现不需要工作站文件：

```bash
rwrl doctor
rwrl --station local/station.yaml --preset hil.yaml doctor
rwrl --station local/station.yaml doctor --fix
```

前两个命令只检查并显示修复计划，不执行修复。`--fix` 先显示命令、专用配置文件内容及后续操作，确认一次后执行已批准的计划，再重新检查。独立修复在其它项目失败后继续；依赖失败的修复会阻塞。重查发现的新问题需重新审阅计划。

每项操作按能力选择，不使用机型、发行版版本或 CPU 架构白名单。硬件 SDK 仍须提供兼容的 wheel、驱动及受支持设备；识别操作系统不等于硬件验收通过。

| 主机能力 | 可用行为 |
|---|---|
| 原生 Linux | 主机/USB 诊断，以及通过已安装后端探测配置的设备。 |
| Debian/Ubuntu 系列，具备 `apt-get` | 修复缺失的诊断工具、libusb/libudev 系统包。 |
| 原生 Linux，具备 udev、用户组工具、PAM limits 或运行中的 systemd | 仅执行相应规则、用户组、限制或有证据的服务冲突修复；其它发行版即使没有 apt 也可使用相应能力。 |
| 容器、WSL、macOS、Windows | Python 元数据/导入诊断及项目环境中的 Python 修复；阻塞原生硬件探测和主机系统修复。 |

不具备所需修复能力时，明确给出 `BLOCKED` 原因及手工后续操作；其它独立的受支持操作可继续。非 apt 发行版的系统包安装仍由用户手工处理。缺失的同仓库项目发行包在存在已核验本地源码树时从该源码修复；已安装包模式缺少该源码时，给出 `BLOCKED` 提示，要求从可信项目包来源安装匹配发行包。doctor 不猜测源码路径，也不静默改装名称相似的公共包。

检查覆盖：

- 缺失的诊断工具及项目/设备 Python 依赖，包括包元数据中的版本约束。保留已满足的包；SDK 导入损坏时修复其已安装版本。
- SpaceMouse USB/libusb 访问及受支持产品 ID、系统 libusb/libudev 库是否可加载（缺失或损坏时使用 apt 修复），以及 hidraw 规则。仅 hidraw 节点可读不能证明 USB SDK 可访问。
- 已调试 GELLO 的串口/事件访问、实际生效的用户组、指定 FTDI 的延迟设置及 brltty 冲突证据。未调试的串口适配器仅作为候选；doctor 不选择其标定。
- RealSense USB 权限与读帧，以及已配置 Franka/Hand 的连接、SDK 握手、反馈和实时权限。
- 内核实时状态；可能需要内核操作时给出手工 **重大修改 / DOUBLE CHECK** 警告。不执行内核或启动配置修改。

工作站和预设配置决定预期设备；已识别的 SpaceMouse/RealSense USB 设备也可独立检查。配置的设备缺失为 `BLOCKED`。未配置、未识别的设备为 `SKIPPED`，不算已正常工作。doctor 从不恢复错误、回 home 或命令机器人运动。活动 daemon、控制器或设备持有者会阻塞独占探测/修复；doctor 不停止或接管其它进程。
只有持有硬件连接的进程才算持有者：daemon、recorder、actor 和 deploy 入口，`setup`、`gello-calibrate`，会建立连接的原语（`status`、`q`、`tcp`、`home`、`gripper`、`camera`、`stop`、`recover`），`teleop run/read/calibrate/diagnose`、`workspace capture` 以及学习命令。离线 `rwrl` 命令（`--help`、`workspace show/set/validate/frame`、`teleop profiles`、`teleop inspect-stop`、`archive inspect`）不会使 doctor 报告 `BLOCKED`。

设备行保留设备 doctor 的检查，但 `rwrl doctor --fix` 从不恢复或重新打开设备，因此提示改为指向 rwrl 命令："--fix (recover)" 变为 `run rwrl recover (or robot-infra doctor --fix)`，"--fix to reopen" 变为 `rerun rwrl status after reconnecting the camera (or robot-infra doctor --fix)`。

提供工作站和 `--preset` 时，doctor 还将预设中的机器人（内联 `real_robot.robot` 或裸 `robot` 映射，或其指向的工作站文件）与工作站机器人比较：`robot_ip`、`arm_backend`、`gripper`、按名称比较的相机（`serial`、`width`、`height`、`fps`、`enable_depth`）、`home_joints`、`joint_limits`、`safety_box`、`task_frame` 和 `workspace`。每个不同字段为一条 `WARN`，id 为 `<robot>.preset:<field>`（例如 `arm.preset:home_joints` 或 `arm.preset:cameras.wrist.fps`），并显示两边的值；字段全部一致时给出一条 `PASS` 行 `<robot>.preset`。WARN 不改变退出码。多机器人工作站上的内联机器人需要 `--robot`，否则比较为 `SKIPPED`。

非 root 用户的系统修复逐条使用 sudo；Python 修复只用当前项目解释器，绝不使用 sudo pip。交互执行可请求 sudo 授权。无人值守执行前，先在终端授权 sudo：

```bash
sudo -v
rwrl --station local/station.yaml --yes --json doctor --fix
```

`--yes` 跳过计划确认，平台、占用、环境和 sudo 检查仍然有效。无 TTY 且未指定 `--yes` 时拒绝修复。缺少 sudo 授权为 `BLOCKED`；不从管道读取密码。`doctor --install-sdk` 仅按失败握手返回的服务端版本选择受支持的 Franky 替换版本。服务端版本冲突时需分离环境；不会按猜测的固件版本选择 SDK。

修复可安装缺失的主机工具、增加用户组、写入专用 `/etc/udev/rules.d/99-rwrl-*.rules` 和当前用户的实时限制，并重载 udev 规则。托管文件的旧内容会被备份；并发修改会被拒绝。设备可能仍需拔插或重新登录，在独立验证前保持 `PENDING_ACTION`。网卡地址/路由、固件、内核、启动配置和重启均由用户手工处理。

每次运行保存已隐藏凭据的诊断：`before.json`、`plan.json`、`report.json` 和 `commands.log`，以及必要的精确配置备份/暂存文件。报告基础目录按以下顺序选择：

1. `doctor --report-dir PATH`。
2. `$RWRL_DOCTOR_REPORT_DIR`。
3. 源码检出目录下可写、已被 gitignore 忽略的 `local/doctor/`。
4. 用户状态目录：Linux 为 `${XDG_STATE_HOME:-~/.local/state}/rwrl/doctor/`，macOS 为 `~/Library/Application Support/rwrl/doctor/`，Windows 为 `%LOCALAPPDATA%/rwrl/doctor/`。

每次创建 `<UTC timestamp>-<id>/` 子目录。显式指定不可写目录时报错；源码目录不可写时退回用户状态目录。报告不写入 `site-packages`。POSIX 报告目录权限为 0700、文件为 0600；Windows 应通过 ACL 保证基础目录私密。备份保留原始字节，应将整个目录视为私密数据。`--json` 在 stdout 输出一个 schema-version-1 对象，包含 `checks`、`repairs`、`remaining_plan`、`report_directory` 及每个机器人的底层 `results`；诊断与提示写 stderr。参数、配置和报告创建错误以 2 退出，请求 JSON 时输出 JSON 错误对象。报告目录已创建时，配置错误还会保存 `error.json`。

| Doctor 结果 | 退出码 |
|---|---|
| 仅有 `PASS`、`WARN` 或 `SKIPPED` | 0；硬件验收前仍须核对检查覆盖 |
| 存在 `FAIL`、`BLOCKED` 或 `PENDING_ACTION` | 1 |
| 参数或配置无效 | 2 |

高级用户可调用 `real_world_rl.cli.doctor.main(argv)`，或继续使用底层 `robot_infra.doctor` API。底层有自己的修复语义；`rwrl doctor` 的设备探测始终使用 `fix=False`。

<a id="home-resolve-save-and-move"></a>

## Home：解析、保存与运动

Home 解析与 `env.reset()` 一致，顺序如下：

1. 预设 `task.reset.home_joints`。
2. 预设 `home_joints`。
3. 工作站 `home_joints`（远程时为 daemon 臂的 `home_joints`）。

值为 null 则继续查下一来源。全部没有 home 时报错并以 1 退出。`home show` 打印来源、home、当前关节、`home - q` 及最大绝对差值。

要复现本页全部 mock 示例，在机器人主机另开一个终端，为这些相对路径示例进入仓库根目录，并激活项目环境。如有需要先创建 `local/`，再自行编写 `local/mock-station.yaml`，内容如下：

```yaml
type: franka
name: mock_station
arm_backend: fake
claim: false
gripper:
  backend: fake
home_joints: [0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7]
```

在该终端中先选择此文件，再运行 mock 命令：

```bash
export ROBOT_INFRA_CONFIG=local/mock-station.yaml
```

后续全部 mock 示例均在同一终端运行。此配置没有相机、teleop 或 daemon 声明，其 home 显示如下：

```bash
rwrl --mock home show
```

```text
source: station home_joints
home (rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
q (rad): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
delta (home - q, rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
max delta: 1.500000 rad
```

`home go` 只执行 reset 的 home 阶段：阻塞关节运动后，若有夹爪且 `task.reset.gripper` 为 `open` 或 `close`，再执行该动作。`keep` 保持夹爪不变。它跳过 lift、waypoints、jitter 和 settle，不是完整任务 reset；结果报告运动后的关节和差值。

`home set` 必须指定已有的 `--preset PATH`。它读取实测关节，保留四位小数后写入 `task.reset.home_joints`，缺少 task/reset 映射时会创建。实测值和舍入值均检查可用的工作站/臂限制、预设 `joint_limits` 和预设 `robot.joint_limits`。越界时以 1 失败，在修改预设和备份之前停止。此命令不请求运动确认。

YAML 编辑保留被修改键之外的注释、引号、空行和键顺序。原子替换前，将原始精确字节保存到预设（或工作站/标定档案文件）旁唯一的 `P.<id>.bak`。检测到并发编辑会拒绝写入。文本输出显示旧 → 新 home，并指出备份路径。

<a id="typical-workflow"></a>

## 典型流程

在机器人主机的仓库根目录、项目环境激活时，使用与工作站同一实臂和限位相符的已有本地 `hil.yaml` 预设。先结束所有直连的 recorder/actor。

```bash
rwrl status
# Manually guide the arm to the desired home using the station's guiding procedure.
# Leave guiding mode before commanding a move.
rwrl --preset hil.yaml home set
rwrl --preset hil.yaml --yes home go
rwrl q
```

供参考，在单独的 mock 终端使用前文自行编写的 mock 工作站 YAML 时，报告如下：

```bash
rwrl --mock status
```

```text
connection: direct fake
robot: mock_station (franka)
arm: FakeArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
cameras: none
teleop: none
```

<a id="confirmation-exit-codes-and-json"></a>

## 确认、退出码与 JSON

`home go` 和夹爪运动先打印目标和最大差值，即使有 `--yes` 也如此。没有 `--yes` 时，TTY 提示 `proceed? [y/N] `；仅 `y`（忽略大小写与两端空白）继续。其他回答或 EOF 取消并以 2 退出；非 TTY 输入拒绝运动并以 2 退出。对同一 mock 工作站，在非 TTY 下执行：

```bash
rwrl --mock home go
```

Stdout：

```text
target home [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]; max delta 1.5000 rad
```

Stderr：

```text
error: refusing to move without --yes
```

该工作站上显式确认的夹爪命令：

```bash
rwrl --mock --yes gripper close
```

```text
target gripper close 0.0000; max delta 1.0000 norm
gripper width 0.0000 m (0.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
```

| 内置退出码 | 含义与示例 |
|---|---|
| 0 | 命令完成，或显示帮助。 |
| 1 | 运行时/设备失败：声明的 daemon 不可达、无 home/夹爪、home 关节值无效、越界或其他异常。 |
| 2 | 用法/配置选择错误：未知参数、缺少工作站、多机器人工作站缺少 `--robot`、`home set` 缺少 `--preset`、`gripper move F` 无效、daemon 可达时使用 `--direct`、运动被拒绝或取消。 |

错误在 stderr 打印 `error: MESSAGE`，有 hint 时追加 ` (hint: HINT)`。`--json` 还在 stdout 打印 `{"error": "MESSAGE", "hint": "HINT"}`（hint 可为空）。`RWRL_DEBUG=1` 会为报告的异常额外在 stderr 打印 traceback。两个命令行均使用 `robot_infra.cli.errors` 中的退出码常量。
原生设备命令的错误也使用 stderr。Supervisor 保留子进程状态；启动失败返回 127，停止升级返回 1，setup 中断返回 130。停止轨迹结果保留 3/4，文本模式工作区验证保留 5。

内置命令成功时，stdout 为一个 JSON 对象，包含 `command`、`path` 和该命令特有字段。以下完整 JSON 示例由 fake 臂初始关节状态推导：

```bash
rwrl --mock --json q
```

```json
{"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

<a id="calling-from-an-agent"></a>

## Agent 调用约定

先运行 `rwrl status` 检查连接与硬件。非交互调用内置原语时，运动使用 `--json --yes`，只读命令使用 `--json`。全局选项放在命令前，检查退出码，同时读取 stderr 的 `error: … (hint: …)` 诊断及 JSON 错误。`--json` 将 stdout 留给结果，SDK 输出、目标和提示移到 stderr。`--yes` 授权所请求的运动，因此发出前核实目标工作站、home 和工作区。调用 doctor、setup、daemon、supervise 时遵循设备命令自身的规则。

缺少必需参数时退出码为 2。使用 `--json` 时，错误除 `error` 和 `hint` 外还包含 `required_fields`（CLI 标志名或缺失子命令的目标名称）。SpaceMouse 运行时检查也会报告缺失的预设、采集输出/名称及样本文件要求。这只描述缺少的输入，不授权运动或免除确认。标定的 `NEEDS_SAMPLES` 结果则列出缺失的方向标签，表示采集尚未完成。

<a id="spacemouse-commissioning"></a>

## SpaceMouse 调试

通过 [SpaceMouse 调试指南](spacemouse-commissioning.md) 标定 SpaceMouse 标定档案、审阅任务坐标系（task frame）工作区，并运行受监督的录制和诊断会话。
