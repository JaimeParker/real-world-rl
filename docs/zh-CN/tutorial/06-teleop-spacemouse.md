<!-- English source: docs/tutorial/06-teleop-spacemouse.md -->

<a id="6-teleop-spacemouse"></a>

# 6. 遥操作：SpaceMouse

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/06-teleop-spacemouse.md)

**目标：** 检查 HID 输入，调试方向配置及任务坐标系，完成一次受监督的 SpaceMouse 会话。

<a id="before-you-start"></a>

## 开始前

除非另有说明，所有命令在**连接 SpaceMouse 的 NUC** 的**仓库根目录**、**激活项目环境**后运行。

- 完成 [03](03-franka-bringup.md)、[04](04-first-motion.md)，包括审查本地笛卡尔预设、home 和安全限制。阅读[操作员页面](../intervening.md)。
- 使用 `teleop.backend` 为 `spacemouse` 的站点。从 GELLO 切换时，先单独保存已校准的 GELLO 站点再修改本地站点。无其他进程占用 HID 或从臂；运动时确保物理停止装置可及。
- 这些步骤调试输入；实际方向、时序和断连行为仍需硬件验证。默认值见 [SpaceMouse 参考](../teleop.md#spacemouse)。

<a id="1-install-and-check-usb-access"></a>

## 1. 安装并检查 USB 访问

在现有项目环境安装输入扩展：

<!-- tutorial-test: skip -->
```bash
pip install -e '.[spacemouse]'
```

然后检查并按建议修复 udev/组权限：

```bash
rwrl doctor --station local/station.yaml
rwrl doctor --station local/station.yaml --fix
```

*应看到：* 按要求重新插拔或登录后，`usb.*`、`spacemouse.*` 检查通过。扩展提供 `hidapi`（导入名 `hid`）；SDK 或权限失败见 [SpaceMouse 与 HID](../doctor.md#spacemouse-and-hid)。

未知 3Dconnexion 设备可在不移动机器人的情况下采集描述符与报告：

```bash
rwrl --station local/station.yaml teleop probe-hid --duration-s 10
```

采集时移动各轴并按每个按钮。多个候选设备时，用报告 ID 加 `--device VID:PID` 重试。`--output DIR` 选择新的证据目录。

*应看到：* `local/teleop/probes/<vid>-<pid>/` 下的证据，包括 `descriptor.hex`、`reports.jsonl`、`candidate.yaml`。审查草稿，解决所有 TODO 解码字段；将验证条目合入站点旁的 `local/spacemouse-devices.yaml`。probe 不自动安装。格式、证据和失败处理见[现场设备表](../spacemouse-commissioning.md#site-device-table)；继续前重跑 doctor。

<a id="2-diagnose-the-input-without-motion"></a>

## 2. 无运动诊断输入

```bash
rwrl --station local/station.yaml teleop diagnose --duration-s 5
```

*应看到：* 引导输入采集及描述轴、按钮、设备身份的诊断报告。检查每个请求动作符号正确且能回到静止。`--profile NAME` 应用现有配置；`--report-dir DIR` 选择证据目录。`diagnose` 仅适用于 SpaceMouse，不接受 `--backend`。无法打开设备见[失败怎么办](#if-it-fails)；未经验证的轴不要用于运动。

<a id="3-set-the-task-frame-and-inspect-the-workspace"></a>

## 3. 设置任务坐标系并检查工作区

按 [04](04-first-motion.md#5-optional-task-frame-and-workspace) 所述，依厂商手动引导流程将臂放在三个不同点：原点、+X 方向一点、X 的 +Y 侧一点。每次读取前退出引导模式。在对应点执行采集；命令仅读姿态，不移动机械臂：

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
    --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
rwrl --station local/station.yaml workspace show
```

*应看到：* 经审查的任务坐标系差异、保存的原点和轴，再是工作区及基坐标安全限制。点文件相对站点目录。已调试坐标系无需重新采集；改变它会重定位同名工作区，须重新检查边界。共线或过近点必须重采。

任务坐标系赋予 tracker 方向相对任务的含义。工作区边界约束该坐标系中的请求，并与基坐标安全盒取交集。只有坐标系不会创建工作区，也不证明内部无碰撞。运动前按[配置与工作区调试指南](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)检查边界；采集设备错误见[失败怎么办](#if-it-fails)。

<a id="4-calibrate-and-select-a-direction-profile"></a>

## 4. 校准并选择方向配置

按上文保存 `table` 后，运行交互方向采集：

```bash
rwrl --station local/station.yaml teleop calibrate --backend spacemouse \
    --profile rear --frame table --output teleop-profiles.yaml --use
rwrl --station local/station.yaml teleop profiles list
rwrl --station local/station.yaml teleop profiles show --profile rear
rwrl --station local/station.yaml teleop profiles use --profile rear
```

*应看到：* 提示方向试验、验证后的候选，再是保存的 `rear` 配置及站点 `teleop_profiles` 中的选择。审查文件与站点改动。输出配置路径相对站点目录。`profiles use` 显式选择现有配置；成功 `--use` 校准后立即执行它是多余的。

方向配置拟合控制帽相对 `table` 的偏航，记录方向/旋转验证及设备身份，保存中立阈值，不改变机器人 home。校准默认每次 3 秒、最大误差 25 度、中立阈值 0.05。基坐标配置用 `--frame base`。`--save` 仅保存，`--use` 保存并选择。`--samples`、`--capture-label`、`--append-trial`、`--report-dir` 支持可重复试验；见[实时校准](../spacemouse-commissioning.md#live-input-calibration-and-diagnosis)。验证失败则重做指定试验，不要选择该候选。

<a id="5-complete-a-supervised-tracker-session"></a>

## 5. 完成受监督 tracker 会话

```bash
rwrl --station local/station.yaml --preset local/gello_cartesian.yaml teleop run --backend spacemouse \
    --mode tracker --profile rear --record demos/spacemouse-check.pkl --num-successes 1
```

审查运动确认；reset 可使机械臂回 home 并操作夹爪。命令解析站点配置并启动自己的监督器。reset 后按按钮 **0** 接管，轻推控制帽，按按钮 **1** 切换夹爪。再次按按钮 0 交还。单独移动控制帽不会接管。释放控制帽停止添加速度；从臂仍可能向有界命令目标收敛。

驱动时用终端 `s` 标记，或按[操作员页面](../intervening.md#spacemouse)使用 `f`/`r`/Esc。终端 `t`/`h` 被忽略。标签保持最后接受目标。配额允许时 `n` 请求下一轨迹；释放控制帽/按钮到中立（配置阈值内稳定 0.2 秒）。达到配额后用 `q` 退出。

*应看到：* 选定配置、受监督 reset/保持行为，以及含解析配置和来源信息的 `demos/spacemouse-check.session-<id>/` 报告目录。保留成功会生成 pkl 和契约/归档伴随文件。工作区 HOLD 会交还；清除请求后按按钮 0 重新接管。启动或输入失败见[失败怎么办](#if-it-fails)。

默认 `tracker`：按钮接管、持久限速目标，接管时保留末次夹爪命令。可选 `--mode serl` 使用原始运动/按钮接管：六轴范数大于 0.001 即接管，释放后交回策略（录制器空闲保持）；按住按钮 0 关闭、按钮 1 打开，按钮 0 优先，均未按下时夹爪动作中立。先配置 xyz 和 RPY 安全盒。SERL 有独立动作契约，录制、训练、部署应保持同一模式。见[两种模式](../teleop.md#spacemouse)。

常用会话选项为 `--record`（必需的新 `.pkl` 路径）、`--num-successes`（默认 20）、`--max-episodes`（可选尝试上限）和 `--events-file`（JSONL 事件替代终端键）。正式数据集采集和归档检查见 [08](08-record-demos.md)。

<a id="if-it-fails"></a>

## 失败怎么办

先运行 `rwrl doctor --station local/station.yaml`，在 [doctor 红灯处理](../doctor.md#red-lights-what-to-do)找到对应红色行。

- USB/HID 访问或 Python SDK 错误：[SpaceMouse 与 HID](../doctor.md#spacemouse-and-hid)。
- 设备被另一会话占用：[设备占用 (BLOCKED)](../doctor.md#busy-devices-blocked)。
- 未知 VID/PID：安装草稿条目前先检查[现场设备表](../spacemouse-commissioning.md#site-device-table)证据。
- 坐标系缺失/不匹配、配置无效或方向反转：运动前重做坐标系/配置调试及输入诊断。选择配置无法修复 HID 访问。
- 缺少预设或输出已存在：提供已调试全局 `--preset` 和新的 pkl 路径。

下一步：[07 遥操作：主臂/从臂](07-teleop-leader-follower.md)。
