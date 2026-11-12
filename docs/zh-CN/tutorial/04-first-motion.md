<!-- English source: docs/tutorial/04-first-motion.md -->

<a id="4-home-gripper-cameras"></a>

# 4. Home、夹爪、相机

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/04-first-motion.md)

**目标：** 用 `rwrl` 命令逐一检查设备：保存 home 姿态并回到该姿态、开闭夹爪、查看相机帧，并掌握停止与恢复。

所有命令在 NUC 的仓库根目录、激活项目环境后运行。

<a id="before-you-start"></a>

## 开始前

- [03](03-franka-bringup.md) 的 doctor 为绿色，无其他进程占用机械臂。
- 工作区畅通、急停触手可及：本页会移动机械臂。
- **运动确认方式。** `home go` 和 `gripper open/close/move` 先打印目标及最大变化，再询问 `proceed? [y/N] `。只有 `y`（不区分大小写）会执行运动；其他回答或输入结束均取消，退出码为 2。无终端（管道输入、agent）时，除非提供自动确认的 `--yes`，否则拒绝运动；仍会打印目标。`--station`、`--preset`、`--yes` 等全局选项放在命令**之前**。

<a id="1-home"></a>

## 1. Home

站点 home 是向导写入的 `home_joints`。与当前机械臂位置比较（仅读取，不运动）：

```bash
rwrl --station local/station.yaml home show
```

*应看到：* `source: station home_joints`，随后是 `home (rad)`、`q (rad)`、`delta (home - q, rad)` 和 `max delta`。

录制和训练使用的 home 位于环境预设，由 `env.reset()` 读取。复制教程预设为本地文件，保留仓库文件不变（`cp` 无需环境）：

<!-- tutorial-test: skip -->
```bash
cp examples/presets/gello_cartesian.yaml local/gello_cartesian.yaml
```

用手引导机械臂至期望 home（Desk 引导模式或机械臂引导按钮），退出引导模式，再将实测关节保存到副本并检查：

```bash
rwrl --preset local/gello_cartesian.yaml home set
rwrl --preset local/gello_cartesian.yaml home show
```

`home set` 对照站点及预设关节限制检查后写入 `task.reset.home_joints`（四位小数），保留注释和键顺序，将旧文件备份为旁边的 `local/gello_cartesian.yaml.<id>.bak`。它不运动，故不询问。第 08 页将在后续更新切换到相同的 `--preset local/gello_cartesian.yaml` 形式；此前凡出现 `examples/presets/gello_cartesian.yaml` 均使用此副本。

将机械臂稍微移开，再回 home。第一条运动前询问，第二条自动确认：

```bash
rwrl --preset local/gello_cartesian.yaml home go
rwrl --preset local/gello_cartesian.yaml --yes home go
```

*应看到：* 运动前 `target home [...]; max delta ... rad`，随后 home、新 `q` 以及接近零的 `max delta`。`home go` 仅执行 reset 的 home 阶段：阻塞关节运动，再执行预设的 `task.reset.gripper`（`open` 或 `close`；`keep` 不变）。跳过抬升、途经点和稳定阶段。home 解析顺序见 [cli.md](../cli.md#home-resolve-save-and-move)。

<a id="2-gripper"></a>

## 2. 夹爪

```bash
rwrl gripper show
rwrl --yes gripper close
rwrl --yes gripper open
rwrl --yes gripper move 0.5
```

*应看到：* 每条命令均有类似 `gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=False, source FrankyGripper` 的行；每次运动前有 `target gripper close 0.0000; max delta 1.0000 norm`。宽度已归一化：`0` 闭、`1` 开（`max_width_m`）。确认实测宽度与目视一致；单凭宽度不能证明装有 Hand。

<a id="3-cameras"></a>

## 3. 相机

RealSense 相机需要项目环境中的 `realsense` 扩展：

<!-- tutorial-test: skip -->
```bash
python -m pip install -e '.[realsense]'
```

随后，确认无录制器或 actor 占用设备：

```bash
rwrl --station local/station.yaml camera show
rwrl --station local/station.yaml camera show --name wrist
rwrl --station local/station.yaml camera grab --name wrist --output local/camera-wrist
```

*应看到：* 每台相机的配置、读取帧的 `color (H, W, 3)`、30 次读取测得的帧率（可用 `--frames N` 修改）及健康状态；无帧时退出码为 1。`camera grab` 在 `local/camera-wrist/` 保存 `wrist-color.png`（启用深度时还有 `wrist-depth.png`）和 `wrist.json`，不会覆盖之前的抓取；下次请换新的 `--output` 目录。

图像的 `[H, W]` 即相机 `source_resolution`。感兴趣区域 `preprocess.<camera>.roi: [top, left, height, width]` 以像素为单位，从保存的彩色图读取。ROI、深度及缩放设置均属策略观测契约（[观测设置](10-hil-training.md#task-images-and-control-profiles)）。

<a id="4-stop-and-recover"></a>

## 4. 停止与恢复

```bash
rwrl stop
rwrl recover
```

*应看到：* `stop: ok` 和 `recover: ok`。`stop` 无需询问即停止；`recover` 清除反射停止后的错误（见 [03](03-franka-bringup.md#5-after-a-reflex-stop)）。两者均与 `status` 使用同一连接：直连时第二个 FCI 连接无法访问录制器或 actor 占用的臂，须先结束该进程或使用物理停止装置。

<a id="5-optional-task-frame-and-workspace"></a>

## 5. 可选：任务坐标系与工作区

SpaceMouse 控制和工作区保持才需要任务坐标系及其中的工作区盒。操作员定位机械臂后（这些命令不执行运动），采集三点并保存坐标系：

```bash
rwrl --station local/station.yaml workspace capture --name frame-origin --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-x --output frame-points.yaml --save
rwrl --station local/station.yaml workspace capture --name frame-y --output frame-points.yaml --save
rwrl --station local/station.yaml workspace frame set --name table \
    --from-points frame-points.yaml --origin frame-origin --x-axis frame-x --y-axis frame-y --save
rwrl --station local/station.yaml workspace frame show
```

点文件相对站点目录解析。根据采集点提出工作区边界（`rwrl workspace set --frame table --from-points ...`）及审查步骤见 [SpaceMouse 调试指南](../spacemouse-commissioning.md#spacemouse-profiles-and-task-frame-workspace)。

<details>
<summary>使用模拟站点演练这些命令</summary>

创建 `local/mock-station.yaml`，内容如下：

```yaml
type: franka
name: mock_station
arm_backend: fake
claim: false
gripper:
  backend: fake
home_joints: [0.1, 0.0, 0.0, -1.5, 0.0, 1.5, 0.7]
```

然后在 NUC（或任何具有项目环境的机器）的仓库根目录运行：

```bash
rwrl --station local/mock-station.yaml --mock status
rwrl --station local/mock-station.yaml --mock home show
rwrl --station local/mock-station.yaml --mock --yes gripper close
```

```text
connection: direct fake
robot: mock_station (franka)
arm: FakeArm, ok=True, mode=idle, errors=[]
gripper width 0.0800 m (1.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
cameras: none
teleop: none
```

```text
source: station home_joints
home (rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
q (rad): [0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000]
delta (home - q, rad): [0.1000, 0.0000, 0.0000, -1.5000, 0.0000, 1.5000, 0.7000]
max delta: 1.500000 rad
```

```text
target gripper close 0.0000; max delta 1.0000 norm
gripper width 0.0000 m (0.00 of max_width_m 0.080), grasped=unknown, source FakeGripper
```

不带 `--yes` 且无终端时，`home go` 向 stdout 打印目标，向 stderr 打印 `error: refusing to move without --yes`，退出码为 2。

</details>

<a id="if-it-fails"></a>

## 失败怎么办

- **退出码 2 且 `refusing to move without --yes`：** 无终端可询问；阅读打印目标后添加 `--yes`。
- **`home set requires --preset`：** 在命令之前提供预设。
- **`home set` 边界检查失败：** 实测关节越限；将臂移回限制内或修正限制（[02](02-setup.md#4-check-limits-and-home)）。
- **错误涉及机械臂、夹爪或相机：** 运行 `rwrl doctor --station local/station.yaml`，找到红色行，再查看 [rwrl doctor：红灯处理](../doctor.md#red-lights-what-to-do)对应章节。

下一步：[05 遥操作：GELLO](05-teleop-gello.md)。
