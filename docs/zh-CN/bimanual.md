<a id="two-arms-as-one-env-real_robot_bimanual"></a>

# 将两条机械臂组合成一个环境（`real_robot_bimanual`）

[中文入口](../README.zh-CN.md) · [English](../bimanual.md)

`--env-backend real_robot_bimanual` 把两条 `robot_infra` 机械臂作为一个环境控制：每臂一个 `RealRobotEnv`，可连接本地机器人或独立守护进程；两臂并行步进，共用一个动作和观测。单臂 `real_robot` 保持原有行为。

[教程](tutorial/00-overview.md) 覆盖单臂从配置向导到部署的过程；下方[双臂教程步骤](#two-arms-the-tutorial-steps)逐项说明双臂工作站的修改。

<a id="config"></a>

## 配置

`--real_robot_bimanual.*` 对应 `BimanualRealRobotEnvConfig`：在 `left` 和 `right` 下分别配置一个 `RealRobotEnvConfig`，另外包含 `arm_names`、`max_episode_steps` 和 `reset_settle_s`。

| 字段 | 配置位置 | 说明 |
|---|---|---|
| `robot` / `remote` / `token`、`safety_box`、`joint_limits`、`home_joints`、`cameras` | 每臂独立 | `cameras` 名称跨臂唯一；`joint_limits` 必须两臂都设置或都不设置。 |
| `action_mode`、`num_joints`、`control_hz`、`gripper_in_action`、`xyz_scale_m`、`rot_scale_rad`、`joint_scale_rad`、`state_fields`、`image_size` | 每臂配置，但两臂必须相同 | 配置构建时不一致会抛出 `ValueError`。 |
| `arm_names` | 双臂配置 | 默认 `[left, right]`，用于观测键和动作约定中的机械臂名称。 |
| `max_episode_steps`、`reset_settle_s` | 双臂配置 | 作用于两臂；每臂的 `max_episode_steps` 必须保持未设置。 |

与 `real_robot` 相同，`joint_limits` 等逐臂列表写在通过 `--config` 指定的 YAML 预设中：

```yaml
env_backend: real_robot_bimanual
env_id: my_bimanual_task
obs:
  rgb: [left_wrist, right_wrist]   # every arm's cameras, in arm order
  image_size: [64, 64]
real_robot_bimanual:
  max_episode_steps: 200
  left:
    remote: LEFT_PC:50051
    token: T
    action_mode: joint_delta
    cameras: [left_wrist]
    home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
  right:
    remote: RIGHT_PC:50051
    token: T
    action_mode: joint_delta
    cameras: [right_wrist]
    home_joints: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
```

`--obs.rgb` 必须按左臂、右臂顺序恰好列出所有相机。后端无法从 `--obs.rgb` 推断相机属于哪条臂，因此需要显式设置 `--real_robot_bimanual.<arm>.cameras`，单臂 `real_robot` 才会自动填充相机列表。

<a id="action"></a>

## 动作

`num_arms=2` 时，`real_world_rl/action_contract.py` 定义的动作约定布局为：

```
[left arm block, right arm block, left gripper, right gripper]
```

每个机械臂块在 `cartesian_delta` 下为 6 维，关节模式下为 `num_joints` 维，含义与单臂相同，见 [README 动作约定](project-readme.md#action-contract)。启用 `gripper_in_action` 时另有两个夹爪维度。环境 `action_contract` 包含 `num_arms: 2`、`arm_names` 和逐臂 `joint_limits`，即 `[left limits, right limits]` 或 null。

<a id="observation"></a>

## 观测

环境自身的观测为 `{"state": {"<arm>_<field>": ...}, "frames": {cam: ...}}`，例如 `left_tcp_pose`、`right_joint_pos`。后端将其展平成 rl-garden 键：

- `state`：先按配置顺序拼接左臂 `state_fields`，再拼接右臂。
- `rgb_<cam>`：每台相机一个，按机械臂顺序排列。

`--convert-obs-rotation` 将两臂 `tcp_pose` 的四元数转换为旋转向量，每臂状态减少一维。

每步 `info` 包含带机械臂前缀的各项键，如 `left_tcp_pose`、`right_rtt_ms`、`left_safety_clips`，以及 `elapsed_steps` 和较慢一臂往返时间 `rtt_ms`。

<a id="running-two-daemons"></a>

## 运行两个守护进程

每臂使用独立 `robot_infra` 守护进程、机器人 YAML 和端口，见 robot_infra README 的[如何开始](robot-infra.md#how-to-start)。可以在两台机器人 PC 上运行，也可以在一台 PC 上使用两个端口：

```bash
python -m robot_infra.server --config left.yaml --bind 0.0.0.0:50051 --watchdog-s 5
python -m robot_infra.server --config right.yaml --bind 0.0.0.0:50052 --watchdog-s 5
```

含 `robots:` 列表的 robot_infra 工作站 YAML 描述两臂。`python -m robot_infra.setup` 的[配置向导](robot-infra.md#setup-wizard)会生成该文件；不传 `--config` 运行 `python -m robot_infra.server --watchdog-s 5`，会按文件中每臂的 `daemon.bind` 启动一个进程。因此仅在使用独立 YAML 文件时才需要上面的两条命令。将 `left.remote` / `right.remote` 指向对应地址。

<a id="teleop-and-demo-recording"></a>

## 遥操作与示范录制

`--teleop-device bimanual` 同时控制两臂，每臂一套主臂；`--teleop-arm-device` 选择逐臂设备。默认 `leader_arm` 读取各臂守护进程提供的机器人 `teleop:` GELLO 部件；`robot` 使用各机器人任意遥操作部件；`gello` 使用连接 actor 机器的 GELLO。操作员覆盖整个动作；仅一臂介入时，另一臂保持静止、夹爪不变。逐臂选项 YAML 和细节见 [teleop.md](teleop.md#two-arms-bimanual)。

通过两个守护进程运行 HIL actor，用一个键盘控制切换和事件：

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard
```

也可以把两套 GELLO 插在 actor 机器上，从 robot_infra 工作站 YAML 读取配置；`robots: {left, right}` 中每臂都有 `teleop:` 段：

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device gello \
    --teleop-options-yaml local/station.yaml --teleop-event-device keyboard
```

示范录制使用相同参数，见 [recording-demos.md](recording-demos.md)：

```bash
python examples/record_real_demos.py --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_abs --teleop-event-device keyboard \
    --output-path demos/bimanual_20.pkl --num-successes 20 --max-episode-steps 300
```

`--teleop-device fake` 同样声明两臂，可在无操作员时演练双臂 actor。

<a id="deploy"></a>

## 部署

使用 `examples/deploy_real_robot.py --env-backend real_robot_bimanual`，连接设置位于 `--real_robot_bimanual.{left,right}.*`。训练时的双臂配置决定 `arm_names`、`max_episode_steps` 和每臂约定字段；训练与部署必须使用同一后端。`--algo rlpd_hybrid`（hil_serl）根据部署环境约定重建双夹爪智能体；`--algo real_world_bc` 部署 `real_world_bc` 运行：

```bash
python examples/deploy_real_robot.py --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote LEFT_PC:50051 --real_robot_bimanual.right.remote RIGHT_PC:50052
```

与训练一样，补充每臂 `token`、`cameras` 和与训练一致的 `image_size`。

<a id="verified-hardware-free"></a>

## 已完成的无硬件验证

`tests/test_bimanual_station_smoke.py` 使用 fake 设备验证完整双臂流程。工作站 YAML 的 `robots: {left, right}` 为每臂配置 fake 机械臂、夹爪、两台相机和脚本化 `joint_abs` 主臂部件，其中一个守护进程设置令牌。通过 `robot_infra.discovery.autoconfig.load_station` 加载，每臂由回环地址上的独立 gRPC 守护进程服务，构建函数为 `robot_infra.server.daemon.build_daemon`，与 `python -m robot_infra.server` 每臂启动的实现相同。测试通过后端注册表构建 `real_robot_bimanual` 环境，每臂独立 `remote`，采用 `joint_delta` 和四台 `--obs.rgb` 相机，然后依次运行下列命令对应的流程：

```bash
python examples/train_real_world.py hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
python examples/record_real_demos.py --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --output-path demos/bimanual.pkl --num-successes 1
python examples/train_real_world.py residual_hil_serl --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs \
    --residual-variant hybrid --base-provider zero
python examples/train_real_world.py silri --role actor --config bimanual.yaml \
    --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_abs
python examples/train_real_world.py hil_serl --role learner --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl            # likewise residual_hil_serl
                                                       # (--allow-base-less-demos) and silri
python examples/train_real_world.py real_world_bc --config bimanual.yaml \
    --demo-dataset-paths demos/bimanual.pkl
python examples/deploy_real_robot.py --checkpoint runs/<bc run>/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual ...
```

测试检查观测键 `state` 与逐相机 `rgb_<cam>`、16 维动作和 `num_arms=2`；检查介入时跟随左主臂而右臂及夹爪保持不变；检查示范 pkl 与 `.contract.json` 中的 `num_arms`、`arm_names`、逐臂 `joint_limits` 可加载到 hil_serl、残差与 SiLRI learner；检查智能体从环境约定推断 `num_grippers=2` 并得到 9 分类夹爪头；检查 BC checkpoint 可通过两个守护进程部署。未安装 `grpc` 时跳过该测试。

<a id="two-arms-the-tutorial-steps"></a>

## 双臂教程步骤

将[教程](tutorial/00-overview.md)应用到两台 Franka、每臂一套 GELLO 时，逐步修改如下。按键、接管起始方式、奖励规则与指标保持相同。

**硬件（[01](tutorial/01-install.md)）。** 每臂需要一台 Franka Panda 与 Hand、一套 GELLO 与总线适配器，以及一台腕部相机。第二臂的 FCI 地址必须不同，向导还会探测 `172.16.1.2`。

**工作站（[02](tutorial/02-station.md)）。** 在 `number of arms` 处回答 `2`，默认名称为 `left right`。向导会对每臂提问一次，分别显示 `-- arm 'left' (1/2) --` 和 `-- arm 'right' (2/2) --`：

- 第二臂的机器人 IP、GELLO 串口和守护进程端口必须不同；重复选择会被拒绝，已选设备标记为 `(used by another arm)`。默认端口为 50051 和 50052。
- 向导为每臂分配下一台可用相机作为腕部相机，命名 `wrist_left`、`wrist_right`；最后一臂还会分配其余相机，例如 `front`。
- 依次标定每套 GELLO。

结果是含 `robots:` 映射的文件，每臂有独立相机、`teleop:` 和 `daemon:` 块。下例为缩写；带完整注释的示例见 [`station.bimanual.example.yaml`](../../robot_infra/docs/station.bimanual.example.yaml)：

```yaml
robots:
  left:
    type: franka
    robot_ip: 172.16.0.2
    cameras: [{name: wrist_left, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...LEFT..., ...}}
    daemon: {bind: 0.0.0.0:50051, token: null}
  right:
    type: franka
    robot_ip: 172.16.1.2
    cameras: [{name: wrist_right, serial: "..."}, {name: front, serial: "..."}]
    teleop: {name: gello, backend: gello, options: {port: /dev/serial/by-id/...RIGHT..., ...}}
    daemon: {bind: 0.0.0.0:50052, token: null}
```

检查两台机器人的安全限制；除任务确有需要外，两个工作区盒应保持分离，因为当前没有臂间碰撞检查。向导最后会指出双臂环境后端 `real_robot_bimanual`。`--mock` 也支持双臂工作站演练，其预设包含两台 Franka、三台相机和两套 GELLO 总线。

环境预设改用 `bimanual.yaml`，单臂使用的文件为 `hil.yaml`。`obs.rgb` 按左臂、右臂顺序列出全部相机，每臂再列自己的相机，守护进程地址取工作站端口。两臂的 `action_mode`、`state_fields` 和其他共享字段必须一致，见[配置](#config)：

```yaml
# bimanual.yaml (two arms)
env_backend: real_robot_bimanual
env_id: my_bimanual_task
obs:
  rgb: [wrist_left, wrist_right, front]
  image_size: [128, 128]
real_robot_bimanual:
  max_episode_steps: 200
  left:
    remote: ROBOT_PC:50051
    cameras: [wrist_left]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
  right:
    remote: ROBOT_PC:50052
    cameras: [wrist_right, front]
    action_mode: joint_delta
    state_fields: [joint_pos, gripper_pos]
```

**守护进程（[03](tutorial/03-daemon.md)）。** doctor 分别打印 `== robot 'left' ==`、`== robot 'right' ==` 和最终 `overall: 2 robots, all passed`；`--robot left` 只检查一臂。运行相同的 `python -m robot_infra.server --watchdog-s 5` 会启动两个守护进程，先显示 `starting 2 daemons from <path>: 'left' on 0.0.0.0:50051, 'right' on 0.0.0.0:50052`，随后各进程输出自己的预检查与 `serving ...`。端口来自文件的 `daemon.bind`。`--robot left` 只启动一臂；多机器人时 `--bind` 和 `--token` 会报错，应逐臂写入文件。无硬件演练由 `tests/test_bimanual_station_smoke.py` 完成，见[无硬件验证](#verified-hardware-free)。

**录制示范（[04](tutorial/04-record-demos.md)）。** 设备参数改为 `--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm --robot-teleop-action-mode joint_delta --teleop-event-device keyboard`。每臂读取自己的 GELLO，任一操作员介入就记录该步，另一臂保持静止。`--max-episode-steps` 覆盖 `--real_robot_bimanual.max_episode_steps`。

**基础策略（[05](tutorial/05-base-policy.md)）。** BC 使用相同命令，将配置换为 `--config bimanual.yaml`。

**HIL 训练（[07](tutorial/07-hil-training.md)）。** 仅修改 `ENV_FLAGS`。三个智能体都会从环境动作约定推断 `num_grippers=2`，无需手动传入：

```bash
ENV_FLAGS="--config bimanual.yaml --teleop-device bimanual --teleop-arm-device leader_arm \
    --robot-teleop-action-mode joint_delta --teleop-event-device keyboard --reward-from-teleop-events"
```

BC 基础策略无需额外设置，描述为 `checkpoint:...?algo=real_world_bc`。双臂 `hil_serl`（`rlpd_hybrid`）checkpoint 作为基础策略时需要显式指定夹爪数量：`checkpoint:<path>?algo=rlpd_hybrid&num_grippers=2`，见[提供器表](residual.md#base-action-providers)。

**部署（[08](tutorial/08-deploy.md)）。** 增加 `--env-backend real_robot_bimanual`，并提供每臂连接与相机；双臂相机列表不会从训练记录自动填入：

```bash
python examples/deploy_real_robot.py --checkpoint runs/bc/checkpoints/final.pt \
    --algo real_world_bc --env-backend real_robot_bimanual \
    --real_robot_bimanual.left.remote ROBOT_PC:50051 \
    --real_robot_bimanual.right.remote ROBOT_PC:50052 \
    --real_robot_bimanual.left.cameras wrist_left \
    --real_robot_bimanual.right.cameras wrist_right front
```

部署守护进程使用移除两台机器人 `teleop:` 块后的工作站 YAML 副本。

<a id="limits"></a>

## 限制

- 尚无臂间碰撞检查，每臂只按自身安全盒、关节限制和设备层限制裁剪。保持两个工作区盒分离。
- `--env-backend real_robot_bimanual` 会拒绝单臂设备 `robot`、`leader_arm`、`gello`、`leader_cartesian`、`spacemouse`、`pico`，请使用 `--teleop-device bimanual`。
