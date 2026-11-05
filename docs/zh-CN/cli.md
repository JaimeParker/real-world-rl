<!-- English source: docs/cli.md -->

<a id="rwrl-station-command-line"></a>

# rwrl：工作站命令行

[中文入口](../README.zh-CN.md) · [English](../cli.md)

`rwrl` 提供录制、训练和部署脚本之间使用的硬件原语：检查工作站、保存 home 姿态、回 home、操作夹爪、停止机械臂或恢复错误。人和 agent 使用同一套命令；`--json`、`--yes` 和退出码使内置原语适合非交互调用。

<a id="where-to-run"></a>

## 在哪里运行

安装 `real-world-rl` 包及工作站所需的 `robot_infra` 依赖后，该环境中即可使用 `rwrl`，等价于 `python -m real_world_rl.cli`。见[安装教程](tutorial/01-install.md)。以下全部命令在 **NUC、仓库根目录、项目环境（project environment）已激活**时运行。工作站和预设的相对路径均相对于该目录。每次原语调用都会连接设备，并在结束后断开，失败时也会断开。

全局选项必须放在**命令之前**，包括将 `--preset` 放在 `home` 之前。`rwrl --help` 查看包装层帮助；`rwrl doctor --help`、`rwrl setup --help`、`rwrl daemon --help`、`rwrl supervise --help` 显示被转发 CLI 的帮助。

<a id="global-options"></a>

## 全局选项

| 选项 | 默认值与含义 |
|---|---|
| `--station PATH` | 解析顺序为显式路径 → `$ROBOT_INFRA_CONFIG` → `local/station.yaml` → `~/.config/robot_infra/station.yaml`。设置了 `$XDG_CONFIG_HOME` 时，最后一个位置随之改变。显式和环境变量路径直接采用；最后两个位置选择首个存在的文件。 |
| `--robot NAME` | 默认未设置：选择唯一机器人；多机器人工作站调用原语必须指定名称。 |
| `--preset PATH` | 默认未设置，使用工作站 home。`home` 命令读取现有环境预设，支持裸映射或 `real_robot` 下的配置。`home set` 必须提供此选项。 |
| `--direct` | 默认关闭，遵循工作站的 daemon 声明。仅在其 daemon 端口不可达时强制直连。 |
| `--mock` | 默认关闭；对**直连**使用 doctor 的 fake 后端替换设备后端。仍需工作站文件，也不会绕过 daemon 选择。 |
| `--json` | 默认关闭；内置原语在 stdout 输出一个 JSON 结果，诊断、运动目标和确认提示写入 stderr。 |
| `--yes` | 默认关闭；跳过内置运动确认，仍会打印目标与最大差值。 |

后两个选项对转发命令的作用范围不同，见下文。

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
| `status` | 连接路径、机器人描述、臂健康、夹爪反馈、相机名称、teleop 名称；远程时含 daemon RTT | 读取 | 无 | 0 / 1 / 2 |
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
| `doctor` | 转发到 `robot_infra.doctor.cli` | 硬件检查；`--fix` 可能恢复错误 | doctor 原有规则 | 原样保留原 CLI 状态 |
| `setup` | 转发到 `robot_infra.setup.__main__` | 发现、标定和 doctor | 向导原有提问 | 原样保留原 CLI 状态 |
| `daemon` | 转发到 `robot_infra.server.daemon` | 预检查，为客户端运动提供硬件服务 | 无包装层提示 | 原样保留原 CLI 状态 |
| `supervise` | 转发到 `robot_infra.supervise.supervisor` | 取决于子命令 | 取决于子命令 | 原样保留原 CLI 状态 |

转发保留剩余参数、输出和返回状态，不添加原语的 JSON 外层或包装层运动提示。转发命令之后的选项属于该 CLI。包装层仅转换以下全局选项：

| 转发命令 | 转换的全局选项 |
|---|---|
| `doctor` | `--station` → `--config`、`--robot`、`--mock`、`--json`、`--yes` |
| `setup` | `--station` → `--from`（已有默认配置，**不是输出路径**）、`--mock` |
| `daemon` | `--station` → `--config`、`--robot` |
| `supervise` | 无；supervisor 和子命令选项均放在 `supervise` 之后 |

其他包装层全局选项对这些被转发 CLI 不生效。尤其是 doctor 的 `--yes` 仅与 `--install-sdk` 搭配，单独使用会触发 doctor 用法错误。doctor JSON 是原有报告，不是 `status` 对象。doctor、setup、daemon 和监督行为见[设备层 README](robot-infra.md#3-check-with-the-doctor)。

<a id="home-resolve-save-and-move"></a>

## Home：解析、保存与运动

Home 解析与 `env.reset()` 一致，顺序如下：

1. 预设 `task.reset.home_joints`。
2. 预设 `home_joints`。
3. 工作站 `home_joints`（远程时为 daemon 臂的 `home_joints`）。

值为 null 则继续查下一来源。全部没有 home 时报错并以 1 退出。`home show` 打印来源、home、当前关节、`home - q` 及最大绝对差值。

要复现本页全部 mock 示例，在 NUC 另开一个终端，进入仓库根目录并激活项目环境。如有需要先创建 `local/`，再自行编写 `local/mock-station.yaml`，内容如下：

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

`home go` 只执行 reset 的 home 阶段：阻塞关节运动后，若有夹爪且 `task.reset.gripper` 为 `open` 或 `close`，再执行该动作。`keep` 保持夹爪不变。它跳过 lift、waypoints、jitter 和 settle，不等于完整任务 reset；结果报告运动后的关节和差值。

`home set` 必须指定已有的 `--preset PATH`。它读取实测关节，保留四位小数后写入 `task.reset.home_joints`，缺少 task/reset 映射时会创建。实测值和舍入值均检查可用的工作站/臂限制、预设 `joint_limits` 和预设 `robot.joint_limits`。越界时以 1 失败，在修改预设和备份之前停止。此命令不请求运动确认。

**PyYAML 会重写预设，因此预设中的注释会丢失。** 对预设 `P`，原文件的精确字节在原子替换前保存为 `P.bak`，覆盖更旧的备份。文本输出显示旧 → 新 home，并警告注释不保留、指出备份路径。

<a id="typical-workflow"></a>

## 典型流程

在 NUC、仓库根、项目环境激活时，使用与工作站同一实臂和限位相符的已有本地 `hil.yaml` 预设。先结束所有直连的 recorder/actor。

```bash
rwrl status
# Manually guide the arm to the desired home using the station's guiding procedure.
# Leave guiding mode before commanding a move.
rwrl --preset hil.yaml home set
rwrl --preset hil.yaml --yes home go
rwrl q
```

上面两条注释要求按工位拖动流程将臂移到期望 home，并在命令运动前退出 guiding 模式。供参考，在单独的 mock 终端使用前文自行编写的 mock 工作站 YAML 时，报告如下：

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

错误在 stderr 打印 `error: MESSAGE`，有 hint 时追加 ` (hint: HINT)`。`--json` 还在 stdout 打印 `{"error": "MESSAGE", "hint": "HINT"}`，hint 可为空。`RWRL_DEBUG=1` 会为报告的异常额外在 stderr 打印 traceback。转发命令保留自身状态与输出规则，异常也可能由包装层处理。

内置命令成功时，stdout 为一个 JSON 对象，包含 `command`、`path` 和该命令特有字段。以下完整 JSON 示例由 fake 臂初始关节状态推导：

```bash
rwrl --mock --json q
```

```json
{"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}
```

<a id="calling-from-an-agent"></a>

## Agent 调用约定

先运行 `rwrl status` 检查连接与硬件。非交互调用内置原语时，运动使用 `--json --yes`，只读命令使用 `--json`。全局选项放在命令前，检查退出码，同时读取 stderr 的 `error: … (hint: …)` 诊断及 JSON 错误。`--json` 将 stdout 留给结果，SDK 输出、目标和提示移到 stderr。`--yes` 授权所请求的运动，因此发出前核实目标工作站、home 和工作空间。调用 doctor、setup、daemon、supervise 时遵循被转发 CLI 自身的规则。
