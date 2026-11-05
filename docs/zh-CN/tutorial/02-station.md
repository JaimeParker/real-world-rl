<!-- English source: docs/tutorial/02-station.md -->

<a id="2-configure-the-station"></a>

# 2. 配置工作站

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/02-station.md)

`python -m robot_infra.setup` 是交互向导。机械臂、相机和 GELLO 插好并上电后，在机器人 PC 运行：探测硬件、提问、内联标定 GELLO、写出带注释的 `station.yaml`、运行 doctor 并打印后续命令。问题实现位于 `robot_infra/setup/wizard.py`，每项均有参数，见 `python -m robot_infra.setup --help`。本页全部命令在 NUC、仓库根、项目环境（project environment）激活时执行。

```bash
python -m robot_infra.setup
```

<a id="select-the-hardware"></a>

## 选择硬件

命令环境同上，见 [01](01-install.md#4-安装项目环境)。先停占用设备的 recorder、actor 或守护进程。未传 `--non-interactive` 时为交互模式，例如：

```bash
python -m robot_infra.setup --arms 1 --arm-names arm --robot-ip 172.16.0.2 --teleop gello --scope local
```

| 参数 | 含义与省略时默认行为 |
|---|---|
| `--arms 1` / `--arms 2` | 臂数；新交互工位建议 1 |
| `--arm-names arm` / `--arm-names left right` | 配置名称；建议 `arm` 或 `left right` |
| `--robot-ip 172.16.0.2` | 按臂顺序指定探测/选择地址 |
| `--cameras none` | 该臂无相机；否则交互选择发现的相机 |
| `--cameras SERIAL=wrist` | 实际序列号及观测名；每臂一个逗号分隔值 |
| `--teleop gello` | 选择 GELLO；单个值应用到所有臂 |
| `--no-calibrate` | 延后标定；交互默认询问，非交互默认跳过 |

未标定值不能用于运动。向导探测 Franka 主机、相机序列号和串口总线；按身份选择，不按发现顺序。
*应该看到：* 带注释的 station YAML 与 doctor 结果。*否则：* 先按 [01 的失败层](01-install.md#5-configure-once-then-run-doctor) 处理。

回答说明：

- **机器人 IP、相机、GELLO 端口**接受列表编号或值本身。未探测到的值需 `--allow-unprobed` 才允许。
- **相机**格式为逗号分隔的 `SERIAL[=NAME]` 或列表编号。`NAME` 对应后续 `obs.rgb` 的观测键 `rgb_<name>`；教程用 `wrist`。
- **介入：**actor 提供键盘接管状态时保留 `always`，事件源会覆盖该设置。目标按键契约见 [00](00-overview.md#operator-keys)。`toggle` 用于机器人 PC 的 evdev 键盘或脚踏，见 [GELLO](../gello.md#6-hil-training)。
- **未指定 `--output`/`--scope`**时最后询问保存位置：`write the station to (local = ./local/station.yaml, user = ~/.config/robot_infra/station.yaml, or a path) [local]:`。已有文件先备份为 `station.yaml.bak.<UTC timestamp>`。
- **GELLO 夹爪：**新通道设置 `gripper_binarize_threshold: 0.5`，动作严格为开 `+1`、闭 `-1`，给 [07](07-hil-training.md) 的离散夹爪头提供清晰标签。

<a id="the-inline-gello-calibration"></a>

## 内联 GELLO 标定

回答 `yes` 运行与 `python -m robot_infra.devices.backends.teleops.gello_calibrate` 相同的流程。扭矩关闭，手动移动 leader：

1. 逐个七关节朝机器人正方向移动并回车。输出 `joint N: observed sign +1 (current +1); use it? [Y/n]`，回车接受，`n` 保留旧符号。位移不足 0.1 rad 保留符号；mock 不运动，会提示 `moved less than 0.1 rad; keeping sign +1`。
2. 保持标定姿态后回车：`[0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0]`，即第 4 关节 -90°、第 6 关节 +90°，其余为零。输出 `offset residuals (rad): [...]`；超过 pi/8 表示姿态或符号可能有误，应重新标定。
3. 扳机全开后回车，再全闭后回车。转动不足 0.05 rad 会失败。

回答 `no` 写入零偏移、全 `+1` 符号及 `# TODO calibrate` 注释。失败会问 `retry the calibration? [Y/n]`。以后用以下命令补标定：

```bash
python -m robot_infra.setup --from local/station.yaml --only teleop --calibrate
```

<a id="where-the-file-lands"></a>

## 文件保存位置

默认 `--scope local` 写入执行目录的 `./local/station.yaml`（`local/` 被 git 忽略）；`--scope user` 写 `~/.config/robot_infra/station.yaml`，`--output` 指定任意路径。不带 `--config` 时，守护进程、doctor 和 discovery 按顺序取首个文件：

1. `$ROBOT_INFRA_CONFIG`
2. `./local/station.yaml`
3. `$XDG_CONFIG_HOME/robot_infra/station.yaml`，默认 `~/.config/robot_infra/station.yaml`

显式 `--config <yaml>` 优先。后续命令应从同目录执行，或设置 `ROBOT_INFRA_CONFIG`。

<a id="multiple-arms"></a>

## 多臂

每臂独立 IP、Desk 和 FCI 会话。示例**左臂 `172.16.0.2`，右臂 `172.16.0.4`**；分配相机、GELLO 和 home 前核对实体。支持共用交换机。逐页刷新、在 NUC 申请控制权，并分别激活 FCI。

NUC、仓库根、项目环境激活，无相机且延后标定的双臂交互示例：

```bash
python -m robot_infra.setup --arms 2 --arm-names left right \
    --robot-ip 172.16.0.2 172.16.0.4 --cameras none none \
    --teleop gello --no-calibrate --scope local
```

每个 GELLO 选不同串口，运动前完成两个标定。若与已配置示例一致，左臂选无夹爪、右臂选 Franka Hand；软件宽度读数不能证明 Hand 存在。不能复制另一臂 home。home 是绑定该物理臂的七关节目标；记录的 TCP 是参考测量，不是另一套独立复位约束。

*应该看到：* `robots.left`、`robots.right` 含正确 IP 和各自设备。在 NUC、仓库根、项目环境激活，逐臂检查：

```bash
python -m robot_infra.doctor --config local/station.yaml --robot left
python -m robot_infra.doctor --config local/station.yaml --robot right
```

*否则：* 在失败臂的 Desk 检查身份和 FCI。Hand 初始化失败常是开错页面或旧页状态陈旧。刷新正确 IP，确认控制权、实体 Hand 与末端电源。重新初始化会移动手指，不是只读诊断。双臂实验设置见 [bimanual.md](../bimanual.md)。

<a id="check-the-safety-limits"></a>

## 检查安全限制

向导写入 Panda 关节限位及单步上限 `joint_limits`、`max_delta_per_step: {xyz_m: 0.05, joint_rad: 0.04}`，并询问 `safety_box`，建议桌面工作区。首动前按现场核对三者：盒位于基坐标系，`tcp_pose = O_T_EE`，单位米，须包含 home、物体和箱子；`home_joints` 应在关节限位内。
两种进程布局均由设备层裁剪伺服目标并拒绝越界 `move_to_*`，见[安全规则](../robot-infra.md#safety-rules)。训练环境只能更严。`--from` 重跑保留已编辑值。

<a id="regenerating-and-rehearsing"></a>

## 重新生成和演练

```bash
python -m robot_infra.setup --from local/station.yaml                    # old values are the defaults
python -m robot_infra.setup --from local/station.yaml --only cameras     # redo one section: cameras | teleop | daemon
python -m robot_infra.setup --mock --output /tmp/station.mock.yaml       # rehearse without hardware
```

演练用 `--output`，避免覆盖真实 station。脚本化操作见 [robot_infra README](../robot-infra.md#setup-wizard) 的 `--non-interactive`。

<a id="the-env-preset-hilyaml"></a>

## 环境预设（`hil.yaml`）

默认示范录制及 HIL actor **直接在 NUC 上运行**。使用内嵌机器人配置 `real_robot.robot`、`arm_backend: franky`，不启动守护进程。另一台机器上的 actor 才用 `real_robot.remote`，见 [03](03-daemon.md)。`robot`、`remote` 必须且只能设置其一。

已提交的[单臂预设](../../../examples/presets/hil_single_arm.yaml)仍是旧的远程 10 Hz joint-delta 流程。在 NUC、仓库根、项目环境激活时复制，编辑完成后再运行：

<!-- tutorial-test: skip -->
```bash
cp examples/presets/hil_single_arm.yaml hil.yaml
```

直接连接的笛卡尔流程：

- 移除 `real_robot.remote` 和 `real_robot.token`。
- 将单臂 station 的机器人映射复制到 `real_robot.robot`，包括 `type: franka`、设备选项、已标定 `teleop`、home、安全限制；不复制 `daemon`。多臂按双臂指南分别取命名机器人映射。
- 设置 `real_robot.action_mode: cartesian_delta`、`real_robot.control_hz: 50`，在 `state_fields` 包含笛卡尔遥操作所需的 `tcp_pose`。
- `obs.rgb` 取 station 相机名（示例 `[wrist]`）；无相机则 `[]`。有图像时保留 `obs.image_size: [128, 128]`。
- 有 Hand 时保留 `gripper_pos`、`gripper_in_action: true`；无 Hand 则 `robot.gripper: null`、`gripper_in_action: false`，移除 `gripper_pos`。

`robot` 是映射，不是 station 文件路径。工位变化后同步复制的设置。GELLO 笛卡尔映射用 `leader_cartesian`；原始关节位置不能直接作为笛卡尔 twist。
目标终端按键路线尚待实现，见 [00](00-overview.md#operator-keys)；配置解析通过不等于运行路线验证。

在 NUC、仓库根、项目环境激活时无硬件检查：

```bash
python examples/train_real_world.py hil_serl --config hil.yaml --print-config
```

*应该看到：* JSON 中 `env_backend: real_robot`、`real_robot.robot` 映射，无远程地址，`action_mode: cartesian_delta`、`control_hz: 50`。
*否则：* 查 YAML 缩进并移除旧远程和 joint-delta 设置。后续教程示例仍待同步到此目标流程；录制、BC/ACT、HIL、部署必须共用动作/观测契约。

<a id="task-images-and-control-profiles"></a>

## 任务、图像与控制配置

教程预设保留 `real_robot.task.name: none`，由操作员或分类器提供成功标签。任务/复位、深度和逐相机预处理见[真机实验](../real-world-experiments.md#task-presets-and-observation-settings)，录制前配置并贯穿流水线。`preprocess.<camera>.roi` 是像素 `[top, left, height, width]`，需真实 `source_resolution: [height, width]`。RGB 双线性缩放；深度为单通道米值，最近邻缩放，无效值为零。修改会改变观测契约。

station 每臂可设 `control:` 阻抗配置，实验通过 `real_robot.control` 覆盖，须在 IDLE 配置。覆盖只能收紧声明的扭矩变化率和误差裁剪界限，SDK 须支持相应关键字。字段、有效值和向导 `--control` 见[机械臂控制配置](../control-profiles.md)。向导保留旧配置，新工位为 `control: null`；增益应适配真实工具和负载。

`real_robot.control_hz` 决定控制频率。非默认全局 `--control-hz` 可填补仍默认的机器人频率，但两者非默认且冲突时拒绝。录制、训练、部署均用最终频率，它属于实验契约。

下一步：[03 进程布局与可选守护进程](03-daemon.md)。
