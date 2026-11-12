<!-- English source: docs/cli.md -->

<a id="rwrl-station-command-line"></a>

# rwrl：站点命令行

[中文入口](../README.zh-CN.md) · [English](../cli.md)

`rwrl` 统一训练、录制、部署和站点操作。人和 agent 使用相同命令；`--json`、`--yes` 和退出码使内置原语可非交互调用。本页为参考；[教程](tutorial/00-overview.md)按顺序介绍命令。

安装 `real-world-rl` 及站点设备扩展后，环境中即可使用 `rwrl`（[安装](tutorial/01-install.md)），等价于 `python -m real_world_rl.cli`。`train`、`record`、`deploy`、`collect-reward`、`teleop run`、`archive inspect` 还需 `learning`；缺失时退出 2，提示 `pip install -e ./rl-garden -e '.[learning]'`。硬件命令在连接设备的**机器人主机**、**激活项目环境**后运行。相对站点/预设路径从当前目录解析。各原语调用时连接，结束后断开，包括失败情况。

全局选项放在**命令之前**，包括 `home` 或 `doctor` 前的 `--preset`。`rwrl --help` 列命令；`rwrl <command> --help` 显示各命令选项。

<a id="global-options"></a>

## 全局选项

| 选项 | 默认值与含义 |
|---|---|
| `--station PATH` | 未设置：按[站点查找](#station-lookup)。 |
| `--robot NAME` | 未设置：单个机器人；多机器人站点的原语需指定名称。 |
| `--preset PATH` | 未设置：用站点 home。对 `home` 为环境预设（裸映射或 `real_robot` 下设置）；`home set` 必需。`doctor` 用于确定预期设备及比较机器人与站点。学习命令中转为 `--config PATH`。 |
| `--direct` | 关闭：遵循站点守护进程声明。仅守护进程端口不可达时允许强制直连。 |
| `--mock` | 关闭：**直连**使用模拟后端。仍需站点文件，不绕过守护进程选择。 |
| `--json` | 关闭：内置原语在 stdout 打印一个 JSON；诊断、运动目标、询问去 stderr。 |
| `--yes` | 关闭：跳过运动或 doctor 修复计划确认；前置条件仍适用。 |

无 JSON 输出的命令（`setup`、`daemon`、`supervise`、学习命令）拒绝 `--json`，退出 2。`doctor` 也接受命令后的 `--station`、`--robot`、`--preset`、`--mock`、`--json`、`--yes`，后值优先。

<a id="station-lookup"></a>

## 站点查找

读取站点的命令按顺序取首个来源：

1. `--station PATH`（也含 `rwrl setup --station`、`rwrl daemon --station`、`rwrl discover --station`）；
2. `$RWRL_STATION`；
3. `./local/station.yaml`；
4. `$XDG_CONFIG_HOME/rwrl/station.yaml`（默认 `~/.config/rwrl/station.yaml`）。

显式及环境路径原样使用。一个发布周期内仍在新名称后读取旧于 0.1.0 的名称：`$ROBOT_INFRA_CONFIG` 在 `$RWRL_STATION` 后，`~/.config/robot_infra/station.yaml` 最后；`rwrl doctor` 均以 `WARN` 附迁移命令报告。`setup` 的 `--station` 提供现有默认值，**不是**输出路径（输出用 `--scope local|user` 或 `--output PATH`）。

<a id="learning-commands"></a>

## 学习命令

`train`、`record`、`deploy`、`collect-reward` 调用训练、示范、部署、奖励采集入口。参数（含 `--help`）属于对应入口，原样透传；`rwrl train --help` 显示训练解析器。全局 `--preset PATH` 前置 `--config PATH`；两者同时给出为用法错误。全局 `--station`、`--robot`、`--direct`、`--mock`、`--json`、`--yes` 不适用，返回 2。控制台脚本、模块入口、示例脚本仍可用。

<a id="connection-ownership"></a>

## 连接归属

所选站点机器人声明 `daemon:` 时，`rwrl` 以 0.5 秒 TCP 超时探测 bind 端口，可达时使用守护进程（通配 `0.0.0.0` 转为 `127.0.0.1`）。`status` 以 `connection: daemon HOST:PORT` 开头。声明但不可达时出错（退出 1），提示启动 `rwrl daemon` 或传 `--direct`；不会自动回退直连。

无 `daemon:` 时直连：franky Franka 站点报告 `connection: direct franky`，模拟为 `direct fake`。声明的守护进程端口可达时 `--direct` 被拒绝，退出 2：`error: the daemon owns the arm (hint: omit --direct)`。

Franka FCI 仅允许一个控制连接。直连时先结束录制器或 actor，再用 `rwrl` 建另一硬件连接；守护进程路径使用同一守护进程。`stop`、`recover` 也遵循此选择，不能绕过直连占用。

<a id="commands"></a>

## 命令

“确认”指包装器运动询问（`--yes` 跳过）。除非另注，内置命令成功退出 0，运行/设备失败为 1，用法/配置错误或拒绝确认为 2（[退出码](#confirmation-exit-codes-and-json)）。

**站点原语**（每个均打开机器人连接）

| 命令 | 效果 | 确认 |
|---|---|---|
| `status` | 连接路径、机器人、机械臂健康、夹爪反馈、相机和遥操作名称；远程有守护进程 RTT。`--json` 增加 `arm_health` 详情及安全裁剪/拒绝数 | 否 |
| `q` | 弧度关节位置 | 否 |
| `tcp` | 米制 TCP 位置及 `xyzw` 四元数 | 否 |
| `home show` | 解析 home、当前关节、`home - q` 及最大值 | 否 |
| `home set` | 实测关节保存到 `--preset`（[home](#home-resolve-save-and-move)） | 否 |
| `home go` | 阻塞移动到 home，再执行预设 reset 夹爪动作 | 是 |
| `gripper show` | 米制及归一化宽度、抓取状态、后端 | 否 |
| `gripper open` / `close` / `move F` | 归一化宽度 1、0 或 `F`，`0 <= F <= 1` | 是 |
| `stop` | 机械臂 `stop()`；打印 `stop: ok` | 否 |
| `recover` | 机械臂 `recover()`；打印 `recover: ok` | 否 |
| `camera show [--name NAME] [--frames N]` | 配置、N 次读取帧率（默认 30）、帧龄、形状、健康；无帧退出 1 | 否 |
| `camera grab --name NAME --output DIR` | 一帧保存为 `NAME-color.png`、`NAME-depth.png`（有深度时）、`NAME.json`；不覆盖；`--timeout-s`（默认 2） | 否 |

**诊断与配置**

| 命令 | 效果 |
|---|---|
| `doctor` | 检查主机和站点设备；`--fix` 一次确认后修复主机及软件。参考：[rwrl doctor](doctor.md) |
| `setup` | 站点向导：发现、询问、校准 GELLO、写站点、运行 doctor（[教程 02](tutorial/02-setup.md)） |
| `discover` | 打印可见硬件，不连接机器人主机（[详情](#discover)） |
| `daemon` | 通过网络提供站点机器人（[远程 actor](remote-actor.md)） |
| `supervise -- COMMAND` | 运行并监督机器人进程，含心跳和停止升级 |

`setup`、`daemon`、`supervise`、`discover` 是原生设备层命令 (`real_world_rl.cli.device_commands`)，退出码自定（下文）；`setup`、`daemon`、`supervise` 无 JSON 输出。监督器 `--` 后参数属于子进程。

**遥操作**

| 命令 | 效果 | 退出码 |
|---|---|---|
| `teleop read` | 站点遥操作实时采样（[详情](#teleop-read)） | 0 / 1（设备失败、遥操作不健康）/ 2 |
| `teleop calibrate` | 校准站点遥操作；`--backend spacemouse\|gello`（[详情](#teleop-calibrate)） | 0 / 1 / 2 |
| `teleop profiles list/show/use` | SpaceMouse 方向配置（[指南](spacemouse-commissioning.md)） | 0 / 1 / 2 |
| `teleop diagnose` | SpaceMouse 仅输入诊断（[指南](spacemouse-commissioning.md)） | 0 / 1 / 2 |
| `teleop run` | 带回合保持的受监督录制；`--backend gello` 用 GELLO 笛卡尔主臂；reset 前询问（[指南](spacemouse-commissioning.md#supervised-recording-sessions)） | 子进程状态或包装器错误 |
| `teleop probe-hid` | 为现场设备表记录类似 SpaceMouse 的 HID（[详情](#teleop-probe-hid)） | 0 / 1（无设备、占用）/ 2 |
| `teleop inspect-stop --trace FILE` | 离线评估保存的停止轨迹（[指南](spacemouse-commissioning.md#live-input-calibration-and-diagnosis)） | 0 PASS / 3 FAIL / 4 BLOCKED |

**工作区与归档**（除 `workspace capture` 外仅读站点文件）

| 命令 | 效果 | 退出码 |
|---|---|---|
| `workspace show` | 约束、`joint_limits`、`max_delta_per_step`、`home_joints`、`home_within_limits`（缺 home 或限制为 `null`；逐关节 `home_violations`） | 0 / 1 / 2 |
| `workspace capture --name N --output PTS --save` | 将当前 TCP 记录为命名点；无运动 | 0 / 1 / 2 |
| `workspace set` / `validate` | 提出或保存工作区；检查姿态（[指南](spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)） | 0 / 1 / 2；文本验证失败 5 |
| `workspace frame show` / `frame set` | 站点 `task_frame`（[详情](#workspace-frame-set)） | 0 / 1 / 2 |
| `archive inspect ARCHIVE` | 离线读录制器 SQLite 归档（[详情](#archive-inspect)） | 0；带 `--stop-time-s`：0 PASS / 3 FAIL / 4 BLOCKED；2 |

**学习：** `train`、`record`、`deploy`、`collect-reward`（[上文](#learning-commands)）。

<a id="command-details"></a>

### 命令详情

<a id="discover"></a>

#### discover

`--json` 打印一个对象，含 `schema_version: 1` 和六个列表，未找到时为空：`cameras`（`backend`、`serial`）、`teleops`（`backend`、`device`）、`can_interfaces`（`name`、`operstate`、`bitrate`）、`hosts`（`ip`、`ping_ok`、`open_ports`；站点机器人 IP）、`hid_devices`（路径）、`unknown_hid`（`vid`、`pid`、`product`、`path`、`message`；见[现场设备表](spacemouse-commissioning.md#site-device-table)）。仅 `unknown_hid` 非空时文本增加 `warnings:`。

<a id="teleop-operations"></a>

#### 遥操作操作

除 `read`、`inspect-stop` 外，各操作在遥操作后端设备部分执行，遵循站点后端（`calibrate`、`run` 也接受 `--backend`，站点无遥操作时必需）。后端未实现操作时，用法错误列出支持后端，例如 `backend 'gello' does not implement teleop diagnose`。`probe-hid` 不论站点遥操作为何，均探测主机 HID。`rwrl teleop <operation> --help` 按后端分组选项。

<a id="teleop-read"></a>

#### teleop read

打印 `q` 或 `twist`、夹爪、介入及触发事件，以 `--rate-hz`（默认 3）每采样一行，直至 Ctrl-C 或 `--duration-s`，再打印摘要；`--json` 输出 JSON 行及末尾 `{"summary": ...}`。直连仅打开遥操作部件。`--with-follower`（仅守护进程路径）增加 `follower_q`，`joint_abs` 主臂还增加逐关节主臂减从臂 `delta`。不移动机器人。

<a id="teleop-calibrate"></a>

#### teleop calibrate

SpaceMouse 拟合并验证方向配置（必需 `--profile`）。GELLO 运行引导主臂校准，从站点 `teleop.options` 填充未设 `--port`、`--joint-ids`、`--gripper-id`、`--baudrate`、`--joint-signs`，从站点 `joint_limits` 取回绕中心，打印 `teleop:` YAML（[gello.md](gello.md#2-calibration)）。`--use` 为 SpaceMouse 选择配置，或审查 diff 后为 GELLO 将校准 `teleop.options` 写入站点。使用其他后端选项为用法错误。`setup --only teleop --calibrate` 在向导内运行相同校准。

<a id="teleop-probe-hid"></a>

#### teleop probe-hid

记录已连接 3Dconnexion HID（或 Logitech 多轴控制器）的报告描述符及 `--duration-s`（默认 10）秒原始输入，写入全新 `<station dir>/teleop/probes/<vid>-<pid>/`（可用 `--output DIR` 替代；无站点时必需；已有目录为用法错误），包含 `descriptor.hex`、`reports.jsonl`、设备表草稿 `candidate.yaml`。`--device VID:PID` 从多设备中选择。不会写设备表，不记录序列号。

<a id="workspace-frame-set"></a>

#### workspace frame set

`--name NAME` 配合 `--base-T-frame`（16 个行主序数字，米）或 `--from-points PTS --origin A --x-axis B --y-axis C`（`workspace capture --save` 保存的点名；右手系，y 相对 x 正交化，拒绝退化点）。`--source TEXT` 记录来源；`--save` 审查差异后仅写 `task_frame`。站点工作区在另一坐标系时拒绝；同名坐标系会重定位工作区并说明。

<a id="archive-inspect"></a>

#### archive inspect

只读归档，打印缩进 JSON：结果、掩码计数、保持漂移及各 `human_action_mask` 段接管跳变/残差。`--csv PATH`、`--measurements PATH` 写新步/测量 CSV；`--drift-window-s`（默认 10）设漂移窗口；每个 `--stop-time-s T`（可重复）按 `--settle-speed-mm-s`（默认 2）、`--settle-hold-s`（默认 0.5）增加稳定评估。`--json` 同字段单行，非有限数为 `null`，另含 `status`（`REPORT`、`PASS`、`FAIL`、`BLOCKED`；停止时刻无跨越 TCP 回合为 `BLOCKED`）。不加载站点：`--station`、`--robot`、`--preset`、`--direct`、`--mock`、`--yes` 均为用法错误。别名 `python examples/inspect_teleop_archive.py` 的退出码忽略稳定状态。

<a id="robot-host-diagnosis-and-repair"></a>

## 机器人主机诊断与修复

`rwrl doctor` 检查主机、项目环境及所有站点机器人，预览定向修复，`--fix` 应用。检查、状态、修复、报告目录、JSON、退出码见专页 [rwrl doctor](doctor.md)。

<a id="home-resolve-save-and-move"></a>

## Home：解析、保存与运动

home 解析与 `env.reset()` 一致，顺序为：

1. 预设 `task.reset.home_joints`。
2. 预设 `home_joints`。
3. 站点 `home_joints`（远程时为守护进程机械臂的 `home_joints`）。

空值回退下一来源；无 home 为错误（退出 1）。`home go` 仅执行 reset 的 home 阶段：阻塞关节运动，再在夹爪存在且 `task.reset.gripper` 为 `open`/`close` 时执行夹爪动作（`keep` 不变）。跳过抬升、途经点、抖动、稳定。

`home set` 需现有 `--preset PATH`。读取实测关节，四位小数写入 `task.reset.home_joints`，补建缺失 task/reset 映射。实测及舍入值均对照站点/机械臂限制和预设 `joint_limits`、`robot.joint_limits` 检查；越界写入前退出 1。此处及站点/配置 YAML 编辑保留注释、引号、空行、键序；先将准确旧字节备份为旁边唯一 `<file>.<id>.bak`，拒绝并发编辑。完整流程见[教程 04](tutorial/04-first-motion.md#1-home)。

<a id="confirmation-exit-codes-and-json"></a>

## 确认、退出码与 JSON

`home go` 和夹爪运动先打印目标与最大变化，带 `--yes` 也一样。不带 `--yes` 时终端询问 `proceed? [y/N] `，只有 `y`（不区分大小写，忽略首尾空白）继续。其他回答或输入结束取消，退出 2；非终端拒绝运动，退出 2（`error: refusing to move without --yes`）。

| 内置退出码 | 含义与示例 |
|---|---|
| 0 | 命令完成（或显示帮助）。 |
| 1 | 运行/设备失败：声明守护进程不可达、缺 home/夹爪、home 关节无效、边界失败或其他异常。 |
| 2 | 用法/配置选择错误：未知参数、缺站点、多机器人缺 `--robot`、`home set` 缺 `--preset`、无效 `gripper move F`、守护进程可达却用 `--direct`、拒绝/取消运动。 |

错误在 stderr 打印 `error: MESSAGE`，有提示时附 ` (hint: HINT)`。`--json` 还在 stdout 打印 `{"error": "MESSAGE", "hint": "HINT"}`（提示可空）。`RWRL_DEBUG=1` 另向 stderr 打印 traceback。退出码常量在 `real_world_rl.cli.errors`。原生命令错误也用 stderr。监督器保留子进程状态；启动失败返回 127，停止升级 1，setup 中断 130。停止轨迹保留 3/4，文本工作区验证保留 5。doctor 退出码见[专页](doctor.md#exit-codes)。

内置命令成功时 stdout 为一个 JSON 对象，含 `command`、`path`、命令特定字段；例如[教程 04](tutorial/04-first-motion.md) 模拟站点：

```text
$ rwrl --station local/mock-station.yaml --mock --json q
{"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

<a id="calling-from-an-agent"></a>

## 从 agent 调用

先用 `rwrl status` 检查连接及硬件。无人值守内置原语运动用 `--json --yes`，只读命令用 `--json`。全局选项放命令前，检查退出码。除 JSON 错误也读 stderr `error: … (hint: …)`；`--json` 将 stdout 留给结果。`--yes` 授权请求运动，发出前核实站点、home、工作区。doctor、setup、daemon、supervise 遵循各自设备命令规则。

缺必需参数返回 2。`--json` 错误除 `error`、`hint` 还含 `required_fields`（CLI 选项名或缺失子命令目标）。SpaceMouse 运行检查也报告缺预设、采集输出/名称、样本文件需求。这仅描述缺失输入，不授权运动或免除确认。校准 `NEEDS_SAMPLES` 则列缺失方向标签。

<a id="spacemouse-commissioning"></a>

## SpaceMouse 调试

方向配置、任务坐标工作区、受监督录制和诊断会话见 [SpaceMouse 调试指南](spacemouse-commissioning.md)。
