<!-- English source: docs/tutorial/05-teleop-gello.md -->

<a id="5-teleop-gello"></a>

# 5. 遥操作：GELLO

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/05-teleop-gello.md)

**目标：** 校准 GELLO，在不移动从臂的情况下检查读数，再完成一次受监督的笛卡尔会话，之后才采集数据集。

<a id="before-you-start"></a>

## 开始前

所有命令在**连接 GELLO 的 NUC** 的**仓库根目录**、**激活项目环境**后运行。若 GELLO 位于独立主机，在该机校准和检查输入；运动会话在从臂 NUC 运行。

- 完成 [03 启动 Franka](03-franka-bringup.md) 和 [04 Home、夹爪、相机](04-first-motion.md)。确保物理停止装置可及。
- 检查 [GELLO 硬件与串口接线](../gello.md#1-hardware)。站点 `teleop.backend` 必须为 `gello`；`teleop.options.port` 应指定稳定的 `/dev/serial/by-id/...` 适配器，并正确配置关节 ID 和波特率。
- 保留 02–04 中调试好的 `local/station.yaml` 和 `local/gello_cartesian.yaml`。会话需要学习依赖、已审查的 home 和限制，且无竞争的录制器或 actor。校准需要独占 GELLO 串口。

<a id="1-check-access-first"></a>

## 1. 先检查访问权限

```bash
rwrl doctor --station local/station.yaml
```

*应看到：* GELLO 检查 (`gello.*`) 和串口/组检查通过。若标记访问或延迟问题，在同一机器运行修复，按提示重新插拔或登录，再运行 doctor：

```bash
rwrl doctor --station local/station.yaml --fix
```

红色行先按[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)处理，再校准。先结束占用串口的进程，不要重复打开。

<a id="2-calibrate-and-save-the-units-signs-and-offsets"></a>

## 2. 校准并保存本机符号与偏移

停止使用 GELLO 的守护进程后：

```bash
rwrl --station local/station.yaml teleop calibrate --backend gello --use
```

确认配置关节 ID 对应实际关节。每次符号检查仅沿从臂正方向移动指定关节，按 Enter。然后保持主臂校准姿态并确认；最后按提示完全打开、关闭扳机。接受前审查站点改动。

*应看到：* 校准后的符号、偏移、回绕中心和（安装时）扳机端点；`--use` 写入站点 `teleop.options`。不带 `--use` 时只打印 YAML 供审查。姿态残差大表示应检查姿态或符号；运动前重做。偏移在四分之一圈网格上求解，超过 pi/8 的残差会报告。回绕中心来自站点关节限制中点（或替代限制 YAML）。无界限时警告并默认零，这对 Panda 关节 6 不正确；接受校准前核查限制。访问失败见[失败怎么办](#if-it-fails)。

| 校准选项 | 默认值 / 用途 |
|---|---|
| `--port` | 站点 `teleop.options.port` |
| `--joint-ids` | 站点关节 ID，按主臂顺序 |
| `--gripper-id` | 站点夹爪 ID；缺省表示无扳机通道 |
| `--baudrate` | 站点值，否则 57600；须与舵机一致 |
| `--joint-signs` | 站点符号，否则均为 `+1` |
| `--calibration-pose` | 七关节 Franka 姿态 `0 0 0 -pi/2 0 pi/2 0`，弧度 |
| `--joint-limits-yaml` | 回绕中心使用的替代机器人 YAML；否则用站点 `joint_limits` |
| `--skip-sign-check` | 保留初始符号；默认关闭 |
| `--use` | 审查后保存到站点；默认关闭 |

存储的 YAML 字段和夹爪选项见[校准参考](../gello.md#2-calibration)。

<a id="3-read-without-commanding-the-follower"></a>

## 3. 只读，不向从臂发送命令

若站点声明守护进程，校准后在 NUC 的独立终端、仓库根目录、激活项目环境后重启，并保持该终端运行：

```bash
rwrl daemon --station local/station.yaml
```

*应看到：* 守护进程监听站点配置地址。直连时保持它停止，在读取命令使用全局 `--direct`。不要在直接占用相同设备的进程旁运行守护进程。

在操作员终端：

```bash
rwrl --station local/station.yaml teleop read --duration-s 30
```

*应看到：* 含 `q=`（主臂关节弧度）、`gripper=`（归一化宽度，1 开、0 闭，无通道时 `none`）、`intervening=`、触发的 `events=` 的行。逐关节检查校准姿态和若干其他姿态。`--rate-hz` 控制打印采样率（默认 3 Hz）；省略 `--duration-s` 时读到 Ctrl-C。

`intervention: always` 时 `intervening` 始终为真。配置 evdev `event_device` 且 `intervention: toggle` 时，确认切换会改变状态，标签键出现在 `events`。此读取命令不监听会话终端 `t`/`h`；它们来自下一步的事件源。

无守护进程时，直连只打开 GELLO，不打开从臂、夹爪、相机。若站点声明已停止的守护进程，在 `teleop` 前加全局 `--direct` 本地读取。`--with-follower` 仅适用于运行中的守护进程，增加 `follower_q` 和关节绝对值 `delta`，不另开机械臂连接。比较方法见 [07](07-teleop-leader-follower.md#1-check-joint-space-alignment)。读数失败或与实际关节不符，见[失败怎么办](#if-it-fails)。

<a id="4-complete-one-supervised-session"></a>

## 4. 完成一次受监督会话

在从臂 NUC 的仓库根目录，激活项目环境，确保机械臂周围畅通，核查本地预设 home 和限制：

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml teleop run --backend gello \
    --record demos/gello-check.pkl --num-successes 1
```

此命令启动自己的监督器和录制器。审查运动确认：reset 可使机械臂回 home 并操作夹爪。它选择 `leader_cartesian` 锚定笛卡尔映射，无需对齐主臂与从臂关节。见[操作顺序](../intervening.md#the-sequence)。

1. 保持接管释放，等待 reset 完成。
2. 按 `t` 将当前主臂姿态锚定到从臂命令姿态，轻柔移动。
3. 压下/释放扳机以关闭/打开夹爪。若与末次夹爪命令不一致，先匹配该状态以释放接管锁存。
4. 按 `h` 交还，保持实测姿态；`t` 使用新锚点接管。
5. 介入时按 `s` 标成功，将成功终止步纳入人工 pkl。`f` 标失败，`r` 丢弃重录，Esc 无成功结束。
6. 标签进入会话保持，冻结最后接受目标。配额允许时 `n` 请求下一回合；`q` 释放控制并退出，达到成功配额后也用它退出。

*应看到：* 上述 reset、接管和保持行为，以及输出旁的 `gello-check.session-<id>/` 报告目录。含 `resolved-config.yaml`、`leader-robot.yaml`（模拟臂及已校准 GELLO 遥操作的载体）、`leader-options.yaml`、`provenance.json` 及会话报告。保留成功回合才会生成 pkl 及契约/归档伴随文件；无保留成功则无 pkl。启动、保持或保存失败见[失败怎么办](#if-it-fails)。

回合间的中立门控要求**释放接管**，无需对齐关节或居中扳机。`--neutral-stable-s` 默认 0.2 秒。常用选项为 `--record`（必需，全新 `.pkl` 路径）、`--num-successes`（默认 20）、`--max-episodes`（可选尝试上限）和 `--events-file`（仅追加 JSONL，或 `-` 使用管道 stdin；否则用终端键）。完整数据集和归档流程见 [08 录制示范](08-record-demos.md)。

<a id="if-it-fails"></a>

## 失败怎么办

先运行 `rwrl doctor --station local/station.yaml`，在 [doctor 红灯处理](../doctor.md#red-lights-what-to-do)找到对应红色行。

- 串口访问、事件设备组或 FTDI 读取慢：[串口、输入组和 USB 延迟](../doctor.md#serial-input-groups-and-usb-latency)。
- 守护进程或其他进程占用输入/机械臂：[设备占用 (BLOCKED)](../doctor.md#busy-devices-blocked)。结束竞争进程。
- 符号错误或姿态残差：运动前重做校准和读取检查。
- `WORKSPACE HOLD`：清除请求，按 `h` 再按 `t` 重新锚定；见[机器人如何跟随](../intervening.md#how-the-robot-follows-you)。
- 缺少预设或复用输出：在 `teleop` 前提供已调试的 `--preset` 和新的 `.pkl` 路径。修改硬件权限无法修复这两者。

下一步：[06 遥操作：SpaceMouse](06-teleop-spacemouse.md)。
