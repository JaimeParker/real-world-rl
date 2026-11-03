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

<a id="s3-taking-over-can-open-the-gripper"></a>

### S3. 接管可能打开夹爪

- **症状**：策略正抓着物体，扳机松开时按 Space，夹爪立即打开。
- **影响**：物体掉落，并产生错误的介入数据。
- **位置**：扳机直接映射为 `2w - 1`（`real_world_rl/teleop/part.py:273`）；接管策略只处理关节（`real_world_rl/teleop/leader_arm.py:161`），夹爪在 `:164` 加入。
- **当前处理**：按 Space 前使扳机匹配当前夹爪状态，抓着物体时先捏紧。
- **计划修复**：在扳机状态一致前保持最后夹爪命令。测试验证不一致接管时夹爪保持，直到扳机越过对应位置。
- **状态**：未解决（S3）。


<a id="docs-stale-items-in-hil-serl-roadmapmd"></a>

### 文档 `hil-serl-roadmap.md` 的过时条目

文档整理时待修正：第 8 项“仍然缺失”称没有示范录制器，但已有 `examples/record_real_demos.py`；第 4 项称双臂夹爪未实现，但 `RLPDHybrid` 已有 `3 ** num_grippers` 分类头且存在 `real_robot_bimanual`；“初次迁移后落地的工作”仍把主臂遥操作后端列为缺失；“行为差异”中的 `backup_entropy`、图像增强和池化默认值已由 `HilSerlArgs` 覆盖。状态：文档问题待解决。

<a id="b-not-planned"></a>

## B. 暂无计划的工作

- **GELLO 力矩跟随**：主臂跟踪从臂需要 robot_infra 的 Dynamixel 电流控制和操作员安全评估。`ramp`、`arm` 已覆盖接管，见 [gello.md](gello.md#9-future-work-leader-tracking-wp8)，实测 GELLO 延迟后再评估。
- **放开主臂即交还控制**：缺少握持传感器，暂停操作可能误交还。使用 Space 或 GELLO `event_device` 的脚踏，见 [teleop.md](teleop.md#leader-arms-gello)。
- **遥操作读故障使 actor 停止**：属于设计行为。异常经 `real_world_rl/teleop/base.py:86` 向上传播，包装器在每步前轮询（`real_world_rl/envs/wrappers/intervention.py:147`），避免将丢读数当成策略步骤。actor 清理现在会关闭环境，强制终止仍依赖 watchdog。


<a id="c-unverified-on-hardware"></a>

## C. 硬件待验证

硬件侧仅使用 fake 设备运行；`leader_cartesian` 正向运动学未与真机比较；ACT 未用真实 pkl 训练；奖励分类器未使用真实图像。详情见[总览中的已知限制](tutorial/00-overview.md#known-limits)。

<a id="closed"></a>

## 已关闭

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
