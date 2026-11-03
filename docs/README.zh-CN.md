# real-world-rl 中文文档入口

这里提供 `docs/` 全部 19 篇文档的完整中文译文，并补充项目 README、设备层 README 和 ACT 说明，共 22 篇。下方教程、专题与参考文档均链接到中文版，涵盖从配置 Franka 与 GELLO，到示范采集、基础策略、人工介入强化学习和部署的完整流程。各译文顶部的 English 链接可对照原文；命令、参数、配置键和程序输出保留原样。

项目由 `real_world_rl`（训练与环境）、`robot_infra`（设备与守护进程）和 `rl-garden`（算法依赖，Git 子模块）组成。GitHub 仓库名为 `real-world-rl`；已有本地目录可能仍叫 `rlgarden-real-world`。以下涉及的命令均从项目根目录运行。

[中文文档索引](zh-CN/README.md) · [项目说明](zh-CN/project-readme.md) · [设备层说明](zh-CN/robot-infra.md)

## 按目标选择入口

| 你要做什么 | 从哪里开始 |
|---|---|
| 第一次把单臂设备跑起来 | [00 总览](zh-CN/tutorial/00-overview.md) → [01 安装](zh-CN/tutorial/01-install.md) → [02 工作站配置](zh-CN/tutorial/02-station.md) → [03 启动与检查](zh-CN/tutorial/03-daemon.md) |
| 用 GELLO 采集示范 | 先读 [06 操作员指南](zh-CN/tutorial/06-intervening.md)，再按 [04 录制示范](zh-CN/tutorial/04-record-demos.md) 操作 |
| 从示范训练基础策略 | [05 BC 与 ACT](zh-CN/tutorial/05-base-policy.md) |
| 开始人工介入强化学习 | [06 操作员指南](zh-CN/tutorial/06-intervening.md) → [07 HIL 训练](zh-CN/tutorial/07-hil-training.md) |
| 在已有 BC 或 ACT 上学习修正 | [残差 RL](zh-CN/residual.md) → [07 HIL 训练](zh-CN/tutorial/07-hil-training.md) |
| 从不完全理想的人工介入中学习 | [SiLRI](zh-CN/silri.md) → [07 HIL 训练](zh-CN/tutorial/07-hil-training.md) |
| 使用双臂和两套 GELLO | [双臂环境](zh-CN/bimanual.md)，其中按教程步骤列出了单臂配置的改法 |
| 把训练结果部署到机器人 | [08 部署](zh-CN/tutorial/08-deploy.md) |
| 排查接管、断连、数据或训练问题 | [已知问题](zh-CN/known-issues.md)和[设备层故障排查](zh-CN/robot-infra.md#troubleshooting) |
| 了解 HIL-SERL 移植历史 | [迁移路线图](zh-CN/hil-serl-roadmap.md)，并对照[已知问题中的文档勘误](zh-CN/known-issues.md#docs-stale-items-in-hil-serl-roadmapmd) |

## 单臂教程

教程以一台 Franka Panda、一个夹爪、一套 GELLO 和腕部相机完成抓取放置为主线。默认使用 `real_robot` 后端和 `joint_delta` 动作模式；示范、基础策略、HIL 训练和部署需要保持同一套动作与观测约定。

| 步骤与译文 | 中文导读 | 完成后应得到什么 |
|---|---|---|
| [00 总览与安全](zh-CN/tutorial/00-overview.md) | 理解从安装到部署的流程、各台机器的职责、动作模式与当前硬件验证范围。 | 明确机器人 PC、actor 和 learner 的分工。 |
| [01 安装](zh-CN/tutorial/01-install.md) | 核对 Franka FCI、相机和 GELLO；配置网络、串口权限，按机器人 PC、GPU 机器和 actor 的职责安装依赖。 | 对应角色所需的软件环境。 |
| [02 配置工作站](zh-CN/tutorial/02-station.md) | 使用配置向导发现设备、标定 GELLO、生成 `station.yaml`；检查设备限制，再复制并调整训练预设 `hil.yaml`。 | 设备配置与训练预设，且 `--print-config` 检查通过。 |
| [03 启动守护进程并检查](zh-CN/tutorial/03-daemon.md) | 用 discovery 查看设备，用 doctor 检查工作站，再启动为客户端提供设备访问的 gRPC 守护进程；也可先用 fake 后端演练。 | recorder、actor 和部署程序可连接的设备服务。 |
| [04 录制示范](zh-CN/tutorial/04-record-demos.md) | 通过 GELLO 操作机器人，用键盘标记成功、失败和重录；只保留包含介入步骤的成功回合。操作前先读第 06 篇。 | `demos/pick-and-place.pkl` 及配套的 `.contract.json`。 |
| [05 基础策略 BC 或 ACT](zh-CN/tutorial/05-base-policy.md) | 用相同示范离线训练 BC 或 ACT，检查观测、动作和相机约定；了解 ACT 在人工介入后重新规划动作块的行为。 | 基础策略 checkpoint；BC 可单独部署，ACT 当前通过残差策略使用。 |
| [06 人工介入操作员指南](zh-CN/tutorial/06-intervening.md) | 掌握接管和交还控制、成功与失败标记、`arm` 与 `ramp` 接管方式，以及每种算法如何使用人工数据。 | 能完成示范录制和 HIL 训练中的人工操作。 |
| [07 HIL 训练](zh-CN/tutorial/07-hil-training.md) | 先启动 learner，再启动 actor；分别给出 `hil_serl`、BC 或 ACT 基础策略上的残差 RL、`silri` 的命令，并说明评估、日志和可选奖励分类器。 | 训练日志、checkpoint 和可用于检查进展的介入率等指标。 |
| [08 部署](zh-CN/tutorial/08-deploy.md) | 从 checkpoint 与训练时的 `config.json` 重建策略；准备移除 `teleop:` 配置块的部署工作站文件，提供机器人连接信息。 | 无 learner 的策略执行与每回合设备状态输出。 |

建议阅读顺序：**00 → 01 → 02 → 03 → 06 → 04 → 05 → 07 → 08**。第 06 篇是录制与在线训练共用的操作说明，适合提前读并在操作时保持打开。

### 机器和配置分别负责什么

| 名称 | 职责 |
|---|---|
| 机器人 PC | 连接机械臂、夹爪、相机和 GELLO，运行 `robot_infra` 守护进程。 |
| actor 机器 | 运行策略采样和示范录制，读取操作员键盘；通常可以和机器人 PC 共用。 |
| learner 机器 | 在 GPU 上训练并向 actor 发布参数；也负责离线训练 BC 或 ACT。 |
| `station.yaml` | 描述实际设备、标定、守护进程和设备侧限制；向导默认写入 `local/station.yaml`。 |
| `hil.yaml` | 描述训练环境、相机、动作模式和连接地址，通常从单臂预设复制。 |
| `.contract.json` | 与示范 pkl 配套，记录动作模式、维度、缩放、关节限制和状态字段等约定。 |
| 训练目录的 `config.json` | 保存训练配置，供部署和基础策略加载时重建模型及动作、观测约定。 |

## 算法与概念

| 方法 | 核心思路 | 适合从哪篇了解 |
|---|---|---|
| `serl` | actor 与 learner 分进程运行，通过 RLPD 学习，可使用状态或图像观测。 | [项目说明中的上机参考](zh-CN/project-readme.md#running-on-hardware) |
| `hil_serl` | 在 SERL 上加入人工接管、持续增长的示范缓冲区、可选奖励分类器和离散夹爪决策。 | [07 HIL 训练](zh-CN/tutorial/07-hil-training.md)、[迁移路线图](zh-CN/hil-serl-roadmap.md) |
| `residual_hil_serl` | 冻结基础策略，学习有界修正；执行动作由基础动作与缩放后的残差相加并裁剪得到。 | [残差 RL 与人工介入](zh-CN/residual.md) |
| `silri` | 学习人工行为模型，再用随状态变化的拉格朗日乘子约束策略与人工行为的距离；人工行为越分散，约束容许范围越大。 | [HIL-RL 与 SiLRI](zh-CN/silri.md) |
| `real_world_bc` | 直接从示范进行离线行为克隆，可作为部署基线或残差策略的基础。 | [05 基础策略](zh-CN/tutorial/05-base-policy.md) |

### 残差 RL

[residual.md](zh-CN/residual.md) 解释组合动作 `clamp(a_base + scale * u, -1, 1)`、`continuous` 与 `hybrid` 变体、`zero` / `checkpoint:` / `act:` 基础策略提供器、人工介入数据、BC 辅助损失和预热流程。需要调整修正幅度、夹爪控制或 ACT 动作块同步时读这一篇。

残差 learner 使用 actor 保存的 `base_actions` 与 `next_base_actions`。普通录制 pkl 缺少这些字段时，默认会被拒绝；`--allow-base-less-demos` 会用零基础动作补齐，使用前先读该文的 [人工介入](zh-CN/residual.md#human-intervention) 一节。

### SiLRI

[silri.md](zh-CN/silri.md) 解释 actor、critic、人工行为模型、状态相关乘子和离散夹爪头，以及损失、预训练、行为模型刷新、数据流、超参数和相对参考实现的差异。需要理解人工介入如何约束策略，或调整学习和刷新节奏时读这一篇。

### 遥操作接口

[teleop.md](zh-CN/teleop.md) 是遥操作总参考，覆盖 `TeleopSource` 协议、设备注册、介入包装器写入的 `info` 字段，以及新增设备的方式。设备包括 Pico、SpaceMouse、机器人自身的 teleop 部件、GELLO、键盘和双臂组合；还包含 `leader_cartesian` 的关节到笛卡尔动作映射、Pico 数据格式和 WSRL 数据录制。

选择 GELLO 接入方式时，`leader_arm` 从机器人 PC 的守护进程读取，`gello` 直接打开 actor 机器上的设备，`leader_cartesian` 用于 `franka_real` 的笛卡尔动作路径。具体选项见[主从臂说明](zh-CN/teleop.md#leader-arms-gello)。

### 双臂环境

[bimanual.md](zh-CN/bimanual.md) 说明如何把两个机器人组合成 `real_robot_bimanual`，包括逐臂配置、两个守护进程、观测拼接、双 GELLO 接管、示范录制、训练和部署。动作布局为 `[左臂动作, 右臂动作, 左夹爪, 右夹爪]`；两臂共用动作模式等约定，相机名称需要唯一。

单臂流程迁移到双臂时，按该文的 [双臂教程步骤](zh-CN/bimanual.md#two-arms-the-tutorial-steps) 逐步修改。当前每臂分别执行设备限制，双臂之间的碰撞检查尚待实现。

### HIL-SERL 迁移路线图

[hil-serl-roadmap.md](zh-CN/hil-serl-roadmap.md) 保留从 HIL-SERL 移植时的功能对照、已落地工作、待迁移能力及默认行为差异，适合追溯设计与移植过程。部分路径与状态保留了历史写法；录制器、双臂夹爪、主从臂以及部分默认参数的过时描述已列在[文档勘误](zh-CN/known-issues.md#docs-stale-items-in-hil-serl-roadmapmd)中，当前使用方式请结合对应专题和教程阅读。

## 硬件与数据参考

| 文档 | 中文导读 |
|---|---|
| [GELLO 硬件到 HIL 训练](zh-CN/gello.md) | 总线适配器、供电、串口权限、波特率与延迟设置，关节方向、偏移和扳机标定，机器人 YAML、录制和训练示例；末尾列出硬件待验证项与主臂跟随设想。 |
| [示范录制参数与文件格式](zh-CN/recording-demos.md) | recorder 参数、起始姿态容差、接管同步步骤的处理、成功回合筛选，以及 pkl transition 字段和 `.contract.json`；适合查参数和数据加载问题。 |
| [旧版上机入口](zh-CN/getting-started-hardware.md) | 保留旧链接，把原有上机章节映射到新教程；从旧书签进入时可用它找到对应步骤。 |
| [完整中文文档索引](zh-CN/README.md) | 完整翻译原文索引，按教程、概念与参考组织，与本页互相导航。 |

`docs/` 之外的相关参考：

- [项目 README](zh-CN/project-readme.md)：安装、各方法入口、动作约定、设备限制、部署、示范约定，以及 fake 机器人的端到端演练。
- [robot_infra README](zh-CN/robot-infra.md)：设备层、工作站向导、discovery、doctor、守护进程和故障排查。
- [单臂训练预设](../examples/presets/hil_single_arm.yaml)：复制为 `hil.yaml` 后修改连接地址和相机等配置。
- [双臂工作站示例](../robot_infra/docs/station.bimanual.example.yaml)：了解逐臂设备与守护进程配置。
- [rl-garden 子模块](../rl-garden)：算法和模型依赖；另见 [ACT 完整中文说明](zh-CN/act.md)，包含从轨迹 pkl 训练、数据约定和引用。

## 操作时常查的规则

| 场景 | 要点 | 详细说明 |
|---|---|---|
| 人工接管 | Space 切换接管；`s` 标记成功，`f` 标记失败，`r` 在 recorder 中重录，Esc 结束回合。 | [06 操作员指南](zh-CN/tutorial/06-intervening.md) |
| 录制成功示范 | 在仍然接管时按 `s`，才能将成功的终止步骤写入示范。 | [04 录制示范](zh-CN/tutorial/04-record-demos.md) |
| 训练奖励 | `real_robot` 自身返回的奖励为 0；HIL actor 需要启用 `--reward-from-teleop-events` 或配置奖励分类器。recorder 默认启用按键奖励。 | [奖励来源](zh-CN/tutorial/06-intervening.md#where-the-reward-comes-from) |
| 动作模式 | `joint_delta`、`joint_abs`、`cartesian_delta` 含义不同；示范、基础策略和训练环境的模式、缩放与状态字段需要一致。 | [动作约定](zh-CN/project-readme.md#action-contract) |
| 绝对关节动作 | `joint_abs` 需要 `joint_limits` 以及设备报告的 `max_delta_per_step.joint_rad`；关节限制成对列表放在 YAML 中配置。 | [02 工作站配置](zh-CN/tutorial/02-station.md) |
| 健康检查 | doctor 会连接设备，安排在同一机器人守护进程启动前执行，避免设备占用冲突。 | [03 启动与检查](zh-CN/tutorial/03-daemon.md) |
| watchdog | 教程使用 `--watchdog-s 5` 为 reset 后首次推理留出时间；结合设备响应和[退出问题 S6](zh-CN/known-issues.md#s6-actor-shutdown-the-arm-is-released-by-the-watchdog-queued-rows-are-lost)选择实际值。 | [03 守护进程](zh-CN/tutorial/03-daemon.md) |
| 评估与部署 | `--role eval` 评估时保持策略自主执行；部署程序从训练目录读取约定，部署 YAML 中移除 `teleop:`。 | [06 操作说明](zh-CN/tutorial/06-intervening.md)、[08 部署](zh-CN/tutorial/08-deploy.md) |

## 已知问题与验证范围

[known-issues.md](zh-CN/known-issues.md) 汇总首次真机运行可能遇到的问题，并给出症状、影响、临时处理方式和计划修复。当前条目包括：

| 编号 | 问题 | 当前处理要点 |
|---|---|---|
| S4 | learner 停止或重启时，缓冲区尾部数据丢失或重复加载。 | 缩短 `--buffer-period`；含快照的运行目录避免同时使用 `--load-replay-buffer`。 |
| S6 | actor 退出后由 watchdog 停臂，待发送数据可能丢失。 | 先结束回合并到达安全姿态，再停止进程；按设备需要设置 watchdog。 |
| S8 | 截断标记缺少直接测试；未训练的 `joint_abs` 策略可能向关节范围中点运动。 | 首次调试采用教程的 `joint_delta` 路径。 |
| S2 | 接管对齐过程的动作进入在线训练的示范缓冲区。 | 可使用 `--teleop-onset arm`，待主从臂对齐再接管。 |
| S3 | 接管时扳机状态与夹爪不同，可能立即松开物体。 | 接管前先让扳机状态匹配夹爪状态。 |
| S1 | `hil_serl --fwbw` 的介入数据没有进入示范缓冲区。 | 首次运行采用常规 HIL 流程。 |
| S5 | SiLRI 预训练或刷新占用缓冲区锁，使 actor 上传阻塞。 | 关注 `actor/dropped_transitions`，必要时减少 `--bc-pretrain-steps`。 |
| S7 | eval 中的人工介入可能计为策略成功。 | 评估期间保持策略自主执行，需要结束失败回合时用 Esc。 |

仓库当前说明的验证范围为 fake 设备测试。Franka 与 GELLO 真机链路、硬件延迟和标定、真实示范上的 ACT、真实图像上的奖励分类器，以及 `leader_cartesian` 运动学仍需上机验证。首次运行前按[教程总览](zh-CN/tutorial/00-overview.md#safety)检查工作区和设备限制，具体状态见[已知问题](zh-CN/known-issues.md)。
