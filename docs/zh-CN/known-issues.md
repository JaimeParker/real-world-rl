<!-- English source: docs/known-issues.md -->

<a id="known-issues-and-deferred-work"></a>

# 已知问题与待办工作

[中文入口](../README.zh-CN.md) · [English](../known-issues.md)

本页说明首次在带夹爪的 Franka 单臂上运行可能遇到的问题、代码位置和当前处理方式。GELLO 通过机器人守护进程读取，使用 `--teleop-device leader_arm`，按键来自 `--teleop-event-device keyboard`。条目中的 `path:line` 在编写时已对照代码，后续行号可能变化，可搜索函数名定位。“静态分析，未测量”的数字来自代码推算。

每项列出症状、影响、位置、当前处理方式、计划修复与验证测试及状态。修复后移至[已关闭](#closed)，保留对应验证记录。

<a id="a-to-fix-after-the-first-real-run"></a>

## A. 首次真机运行后待修复

按处理顺序排列。


<a id="s8-untrained-joint_abs-startup-motion"></a>

### S8. 未训练 `joint_abs` 的启动运动

接近零的归一化动作会指向关节范围中点。learner 首个快照仍可能包含未训练策略；就绪握手不会为非残差 `joint_abs` 自动替换成基于实测姿态的保持策略。调试该模式时应使用已配置的单步运动边界。自主控制前的保持或预热策略仍待实现。截断与 bootstrap 边界已有 `tests/test_partial_imitation.py` 和轨迹恢复测试覆盖。

状态：启动保持与预热行为仍未解决（S8）。

<a id="s3-joint-space-leaders-taking-over-can-open-the-gripper"></a>

### S3（关节 leader）：接管可能误开夹爪

- **症状：**leader_arm、gello 或 robot 路线，策略持物而扳机释放时 `t` 接管会立即张开。
- **影响：**物体掉落和错误介入行。
- **位置：**`real_world_rl/teleop/part.py` 的 `TeleopPartSource._gripper_action` 直接做 `2w - 1`；`leader_arm.py` 的 `_poll` onset 仅处理关节。
- **当前处理：**接管前匹配夹爪状态，持物时先捏紧。
- **计划修复：**采用 leader_cartesian 的接管锁存，见[已关闭项](#s3-leader_cartesian-taking-over-can-open-the-gripper)。
- **状态：**关节 leader 路线仍开放。

<a id="zero-twist-holds-left-on-cartesian_delta"></a>

### cartesian_delta 仍使用零 twist 的保持路线

- **症状：**保持仍发零 twist 而非固定姿态时，阻抗后端平衡点跟随实测而漂移。
- **位置：**wrapper 对人工未控制维度补位（selected-parts + idle-behavior hold，或无夹爪 teleop）；InterventionWrapper 未调用 hold 的 reset/observe_step hooks；分类帧采集器 `real_world_rl/reward/collect_real_robot.py`；双臂 cartesian_delta 的 `hold_action_fn_for` 在 `num_arms > 1` 时仍用零臂动作。
- **当前处理：**示范用固定保持的 recorder；笛卡尔分类器采帧时让操作员保持控制。
- **计划修复：**wrapper/collector 调用 hold hooks，双臂逐臂独立 tracker。
- **状态：**开放。

<a id="docs-stale-items-in-hil-serl-roadmapmd"></a>

### 文档 `hil-serl-roadmap.md` 的过时条目

文档整理时待修正：第 8 项“仍然缺失”称没有示范录制器，但已有 `examples/record_real_demos.py`；第 4 项称双臂夹爪未实现，但 `RLPDHybrid` 已有 `3 ** num_grippers` 分类头且存在 `real_robot_bimanual`；“初次迁移后落地的工作”仍把主臂遥操作后端列为缺失；“行为差异”中的 `backup_entropy`、图像增强和池化默认值已由 `HilSerlArgs` 覆盖。状态：文档问题待解决。

<a id="b-not-planned"></a>

## B. 暂无计划的工作

- **GELLO 力矩跟随**：主臂跟踪从臂需要 robot_infra 的 Dynamixel 电流控制和操作员安全评估。`ramp`、`arm` 已覆盖接管，见 [gello.md](gello.md#9-future-work-leader-tracking-wp8)，实测 GELLO 延迟后再评估。
- **放开主臂即交还控制**：缺少握持传感器，暂停操作可能误交还。使用 `h`（keyboard 或 stdin 事件源）或 GELLO `event_device` 的脚踏，见 [teleop.md](teleop.md#leader-arms-gello)。
- **遥操作读故障使 actor 停止**：属于设计行为。异常经 `real_world_rl/teleop/base.py:86` 向上传播，包装器在每步前轮询（`real_world_rl/envs/wrappers/intervention.py:147`），避免将丢读数当成策略步骤。actor 清理现在会关闭环境，强制终止仍依赖 watchdog。


<a id="c-unverified-on-hardware"></a>

## C. 硬件待验证

硬件侧仅使用 fake 设备运行；`leader_cartesian` 正向运动学未与真机比较；ACT 未用真实 pkl 训练；奖励分类器未使用真实图像。详情见[总览中的已知限制](tutorial/00-overview.md#known-limits)。

<a id="closed"></a>

## 已关闭

<a id="cartesian-hold-drift-cartesian_delta-control-v2"></a>

### 笛卡尔保持漂移（cartesian_delta，控制 v2）

real_robot 发送 `measured + action * scale`，零 hold 曾使阻抗平衡点随漂移移动，首次实机 24 s 漂移 137 mm。现在 recorder 在 reset、每次交还采样固定 hold，leader_cartesian 维护持续命令，以最多 0.05 m/s / 0.3 rad/s 推进、领先实测最多 0.05 m / 0.3 rad，多余丢弃。环境尺度须不小于上限。契约记录 `teleop_control` / `cartesian_tracker_v1`，见 [teleop](teleop.md#leader-arm-on-a-cartesian-robot-leader_cartesian)。仍用零 twist 的路线见上方开放项。

状态：recorder 与 leader_cartesian 已修复，仅离线、尚未重跑实臂。回归：`tests/test_cartesian_tracker.py`、`tests/test_leader_cartesian_source.py`、`tests/test_real_demo_recorder.py`。

<a id="s3-leader_cartesian-taking-over-can-open-the-gripper"></a>

### S3（leader_cartesian）：接管误开夹爪

接管时不匹配的扳机被忽略，持续发送 wrapper 的 `last_gripper_action_getter` 最后命令，直到扳机跨到对应状态。

状态：leader_cartesian 和下述 spacemouse 已修复；关节 leader 仍开放。回归：`tests/test_leader_cartesian_source.py` 的夹爪锁存测试。

<a id="s3-spacemouse-taking-over-can-open-the-gripper"></a>

### S3（spacemouse）：接管误开夹爪

旧实现每次 reset 将 sticky gripper 设开，策略持物时接管会发 +1 掉物。现在 SpaceMouse 每次按钮 0 接管从 wrapper 最后夹爪命令开始，包括连续值；只有按钮 1 在驱动时切换 ±1，交还时按钮 1 无效。

状态：spacemouse 已修复，仅离线、尚未跑臂。回归为 `tests/test_spacemouse_teleop_source.py`（接管夹爪、连续最后动作、未接管按钮 1）。robot YAML 的 backend spacemouse 路线仍是旧原始 twist 语义：运动即介入、按钮 0 切换 reset 后开的夹爪；模拟专用 spacemouse_twist 亦同。

<a id="s1-fwbw-intervention-tracing"></a>

### S1. FWBW 介入追踪

FWBW 每个方向现在输出独立持久化轨迹片段，包含实际动作、策略提案与真实人工掩码。人工数据通过与其他 actor 路径相同的训练投影进入在线和示范回放，片段边界防止 n-step 窗口跨越方向切换。

状态：已修复。回归覆盖为 `tests/test_p1_ingestion.py` 的 FWBW 轨迹与部分人工监督测试。


<a id="actor-parameter-versions"></a>

### actor 参数版本

actor 快照使用 learner session/version 身份，仅在身份变化时加载新状态；learner 重启后使用新 session、较低版本号也会正确刷新。状态：已修复，P0 生命周期测试覆盖。


<a id="s2-catch-up-motion-and-partial-demonstration-labels"></a>

### S2. 追赶动作与部分示范标签

接管追赶携带同步元数据：`joint_delta` 标记到主臂首次进入从臂一个 `joint_scale_rad` 范围内为止，`joint_abs` 标记 ramp 限速步骤。初次对齐后正常监督。持久化归档保留实际动作与真实人工掩码，训练投影排除同步监督；human/override 掩码也使保持补充不进入部分模仿损失。录制器启用 `--keep-syncing-steps` 后，pickle 导出将这些步骤作为普通监督保留。

状态：已修复，回归测试见 `tests/test_partial_imitation.py`、`tests/test_leader_arm_source.py` 及录制器测试。


<a id="s4-learner-replay-and-checkpoint-recovery"></a>

### S4. learner 回放与 checkpoint 恢复

持久化 journal 接收、原子 checkpoint generation、回放扩展恢复及 journal 游标现在可保留已提交数据，并避免 bundle 与快照重复加载。旧原生回放仍可读取，同时明确其来源标签限制。

状态：已修复，回归测试为 `tests/test_replay_state.py`、`tests/test_p0_process_recovery.py`。


<a id="s6-actor-lifecycle-and-pending-uploads"></a>

### S6. actor 生命周期与待上传数据

actor/eval 清理会释放环境资源，并在期限内排空上传。SQLite outbox 保留未确认消息和 SiLRI 暂存数据。强制终止进程仍依赖守护进程 watchdog；动作执行后、持久化提交前的最后结果不在持久化保证内。

状态：已修复，回归测试为 `tests/test_p0_process_recovery.py`、`tests/test_p0_lifecycle.py`。


<a id="s5-silri-pretraining-and-ingestion"></a>

### S5. SiLRI 预训练与数据接收

HTTP 接收提交 learner journal，不依赖训练缓冲区锁。稳定 transition ID 让重试幂等，漏收 ACK 或进程重启后 outbox 数据仍保留。

状态：已修复，回归测试为 `tests/test_p0_process_recovery.py`、`tests/test_p0_lifecycle.py`。


<a id="s7-autonomous-and-assisted-evaluation"></a>

### S7. 自主与辅助评估

actor、eval、deploy 和录制共用显式结果。主要成功率以全部已完成回合为分母，只计未发生动作介入的成功；任务总成功率和介入率单列。仅有正奖励或回合终止不足以判成功。

状态：已修复，回归测试为 `tests/test_episode_outcomes.py`、`tests/test_p0_lifecycle.py`、`tests/test_real_demo_recorder.py`。


操作细节与剩余恢复边界见[真机实验](real-world-experiments.md)。
