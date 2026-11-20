# 路线一 · 遥操作和真机数据采集

目标是留下一个可以复查的回合：人在什么时候接管、给了什么动作、机器人实际到了哪里。下面两种设备任选其一。示例文件名统一用 `demos/task.pkl`，每次新录制换一个文件名。

## 工作站和任务各保存什么

station 描述物理连接和工作区，preset 描述这次实验给策略哪些观测、动作采用什么单位、如何复位。在上位机仓库根目录创建本地副本，再填写实机地址、相机名、home、夹爪设置及经现场确认的运动范围。

```bash
mkdir -p local demos
cp -n examples/stations/robot.franka.example.yaml local/station.yaml
cp -n examples/presets/gello_cartesian.yaml local/task.yaml
```

后一个模板给出单臂笛卡尔增量和状态观测。使用相机时，在 station 声明相机，并在 preset 指定相同名字和图像尺寸。按下面的设备指南补全 station 的 teleop 块。SpaceMouse 的受监护入口会选择自己的输入源；后续训练复用会话最终保存的配置。

```bash
rwrl doctor --station local/station.yaml
rwrl --station local/station.yaml workspace show
```

核对当前机器人、夹爪、设备和工作区。真实会话开始时 reset 可能归位并操作夹爪，先让现场人员确认对应姿态、手中物体和空间。

## 使用 SpaceMouse

先推一个轴、松手，再换下一轴，看输入响应和回零。给操作位建立方向档案；例如 `rear` 是操作位名称，`table` 是已标定的桌面坐标系。转身换边操作时，原来的“向前”可能对应另一方向。

```bash
rwrl --station local/station.yaml teleop read --duration-s 10
rwrl --station local/station.yaml teleop calibrate --backend spacemouse --profile rear --frame table --output teleop-profiles.yaml --use
rwrl --station local/station.yaml teleop diagnose --duration-s 5
rwrl --station local/station.yaml --preset local/task.yaml teleop run --backend spacemouse --mode tracker --profile rear --record demos/task.pkl --num-successes 1
```

`teleop diagnose` 检查输入。进入 `teleop run` 后才开始受监护会话。tracker 模式按按钮 0 接管，再按一次交还；接管期间按钮 1 切换夹爪。松开手柄会停止继续推进目标，但接管状态依然保留。显式选择 `--mode serl` 时，移动超过阈值即介入，带夹爪时左右按钮分别闭合和打开，松开手柄和按钮交还。先用一种模式完整记录，保留对应配置。[坐标系建立、HID 与方向诊断](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/tutorial/06-teleop-spacemouse.md)

## 使用 GELLO

在 station 中指定本机串口、关节 ID、方向、偏移和夹爪标定。依次移动单个主臂关节，看对应读数，再检查夹爪从闭合到张开的读数。把主从姿态对齐后再开始接管。

```bash
rwrl --station local/station.yaml teleop calibrate --backend gello --use
rwrl --station local/station.yaml teleop read --duration-s 30
rwrl --station local/station.yaml --preset local/task.yaml teleop run --backend gello --record demos/task.pkl --num-successes 1
```

本例沿用笛卡尔 preset，主臂关节经运动学映射为从臂末端目标。若任务要求关节跟随，选用相应关节 preset，记录所用 `joint_delta` 或 `joint_abs`。两种动作即使维数相同，训练和部署也要沿用采集时的含义。[GELLO 工作站与操作说明](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/tutorial/05-teleop-gello.md) · [双臂独立接管案例](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/testing/bimanual-franka-gello-smoke.md)

## 标注和复查

受监护会话里，成功按 `s`，失败按 `f`。还有后续回合时，先保持，操作员确认后按 `n` 复位；`q` 退出。达到采集配额时会话可以结束。记下输出旁本次专属的 `.session-…/resolved-config.yaml`，后文用环境变量 `SESSION_CONFIG` 表示这份文件的路径。

```bash
rwrl archive inspect demos/task.trajectory.sqlite3
```

SQLite 留存所有尝试和逐步数据。检查 `human_action_mask` 看哪些维度由人控制，检查实际送给环境的动作，再对照相邻观测中的 TCP 位姿或关节读数，看机械臂实际移动了多少。训练用 pkl 保存符合条件的成功示范；一次失败也可以留下完整 SQLite。把 pkl、同名 `.contract.json` 和会话配置一起保存。下一步进入[行为克隆](bc.md)。
