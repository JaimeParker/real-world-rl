<a id="known-issues-and-deferred-work"></a>

# 已知问题与待办工作

[中文入口](../README.zh-CN.md) · [English](../known-issues.md)

本页说明首次在带夹爪的 Franka 单臂上运行可能遇到的问题、代码位置和当前处理方式。GELLO 通过机器人守护进程读取，使用 `--teleop-device leader_arm`，按键来自 `--teleop-event-device keyboard`。条目中的 `path:line` 在编写时已对照代码，后续行号可能变化，可搜索函数名定位。“静态分析，未测量”的数字来自代码推算。

每项列出症状、影响、位置、临时处理方式、计划修复及验证测试和状态。修复后移至[已关闭](#closed)，补充提交与测试，保留历史记录。

<a id="a-to-fix-after-the-first-real-run"></a>

## A. 首次真机运行后待修复

按处理顺序排列。

<a id="s4-the-learner-loses-or-double-loads-buffer-rows-across-a-stop"></a>

### S4. learner 停止后丢失或重复加载缓冲区数据

- **症状**：以相同 `--exp-name` 重启 learner 后，`learner/buffer_size` 小于停止前的 `learner/received`；使用 `--load-checkpoint ... --load-replay-buffer` 时也可能变为约两倍。
- **影响**：数据丢失，上限为 `--buffer-period` 减 1 条，默认 999，10 Hz 下约 100 秒；重复数据会使采样偏斜。
- **位置**：`HilSerlLearnerLoop._on_transition` 仅在收满一个 `--buffer-period` 时写入恢复快照（`real_world_rl/loops/hil_serl.py:376`，`_snapshot` 位于 `:379`）。`LearnerLoop.run` 退出时保存 `final.pt`，但未刷新待保存数据（`real_world_rl/loops/learner.py:254`）。教程先停止 actor，之后没有数据触发快照，尾部因此丢失。启用 `--load-replay-buffer` 时，构建器中的 `agent.load` 从 checkpoint 加载在线缓冲区，位置为 `rl-garden/rl_garden/training/online/entrypoints/rlpd_hybrid.py:121`、`rlpd.py:108`、`real_world_rl/training/entrypoints/silri.py:168`；随后又加载同目录全部快照（`real_world_rl/loops/hil_serl.py:270`，`_reload_snapshots` 位于 `:292`）。定期 checkpoint 在释放缓冲区锁后保存（`real_world_rl/loops/learner.py:150`，`_maybe_save_periodic_checkpoint` 位于 `:153`；SiLRI 为 `real_world_rl/loops/silri.py:200`），因此 `--save-replay-buffer` 下网络线程可能在写文件期间继续添加数据，此项为静态分析，尚未测量。
- **当前处理**：首次运行将 `--buffer-period` 设小，例如 100；新 `checkpoints/buffer/transitions_<n>.pkl` 出现后立即停止 actor；包含快照的目录避免再使用 `--load-replay-buffer`。
- **计划修复**：停止时刷新待保存数据；checkpoint 已包含缓冲区时跳过快照重载；定期保存时持有缓冲区锁。测试应验证停止时尾部落盘、重启恢复恰好 `received` 条，以及 checkpoint 缓冲区与快照并存时每条仅加载一次。
- **状态**：未解决（S4）。

<a id="s6-actor-shutdown-the-arm-is-released-by-the-watchdog-queued-rows-are-lost"></a>

### S6. actor 关闭后依赖 watchdog 停臂且队列数据丢失

- **症状**：Ctrl-C 或 actor 崩溃后，机械臂等待守护进程 watchdog 触发才停止；尾部数据及 SiLRI 暂存的最后一条无法送达 learner。
- **影响**：机械臂在 `--watchdog-s` 时间内保持最后伺服目标，默认 0.5 秒；每次停止都可能丢失数据。
- **位置**：actor 入口未调用 `env.close()`，位置为 `real_world_rl/training/entrypoints/hil_serl.py:36`、`residual_hil_serl.py:95`、`silri.py:187`；eval 循环 `hil_serl.py:185`、`residual_hil_serl.py:183`、`silri.py:263` 也相同。`ActorLoop.run` 仅停止同步客户端（`real_world_rl/loops/actor.py:171`）。`env.close()` 本会向守护进程发送 `robot.disconnect`，相关位置为 `real_world_rl/envs/wrappers/intervention.py:182`、`real_world_rl/envs/real_robot/env.py:418`、`robot_infra/robot_infra/server/remote.py:440`。watchdog 默认值位于 `robot_infra/robot_infra/server/daemon.py:51`。`ActorSyncClient.stop` 在推送线程清空队列前设置停止事件，线程在每项前检查它（`real_world_rl/loops/sync.py:285`、`:323`），因此 SiLRI 在 `_on_run_end`（`real_world_rl/loops/silri.py:76`）刷新发送的数据也可能丢弃。
- **当前处理**：先按 Esc 结束回合，待机械臂到达安全姿态再 Ctrl-C；保持急停可用，守护进程的 `--watchdog-s` 按 reset 所需时间设置。
- **计划修复**：在所有 actor 与 eval 路径的 `finally` 中关闭环境；设置停止事件前，在有限期限内清空推送队列。测试验证中断后 fake 环境关闭，且 `stop()` 前排队数据均送达 fake learner。
- **状态**：未解决（S6）。

<a id="s8-small-gaps-truncated-rows-untested-joint_abs-start-up-motion"></a>

### S8. 小缺口 截断数据未测试与 `joint_abs` 启动运动

- **症状**：截断测试缺口尚无已观察症状；未训练的 `joint_abs` 策略会在 actor 启动后立即驱动机械臂向 `joint_limits` 中点靠近。
- **影响**：done 标记回归可能静默污染数据；意外运动影响安全。
- **位置**：`ActorLoop._build_transition` 保存 `done = terminated`（`real_world_rl/loops/actor.py:113`），唯一直接测试只传入 `terminated=True, truncated=False`（`tests/test_real_world_actor_loop_hooks.py:86`）。`tests/test_single_arm_station_smoke.py:461` 运行截断回合但未检查 `done`。`joint_abs` 将接近 0 的动作映射到 `joint_limits` 中点（`real_world_rl/envs/real_robot/env.py:337`），仅由 `max_delta_per_step`（`:167`）限制每步幅度。`hil_serl` 与 `silri` 从未训练策略开始，`residual_hil_serl` 叠加基础策略。
- **当前处理**：采用教程主线的 `joint_delta`；使用 `joint_abs` 时调低 `max_delta_per_step`，启动后及时按 Space 接管。
- **计划修复**：验证 `truncated=True, terminated=False` 保存 `done` 为 False；为无基础策略的 `joint_abs` 增加预热或保持阶段。
- **状态**：未解决（S8）。

<a id="s2-takeover-transients-enter-the-demo-buffer"></a>

### S2. 接管过渡步骤进入示范缓冲区

- **症状**：`joint_delta` 中按 Space 后，从臂以每步 `joint_scale_rad` 向主臂靠近，饱和步骤被当作示范用于训练。
- **影响**：示范缓冲区混入追赶动作，污染任务示范。
- **位置**：只有 `joint_abs` ramp 会标记 `teleop_syncing`（`real_world_rl/teleop/leader_arm.py:212`，在 `:167` 的 `events` 设置）；`joint_delta` 在 `:208` 返回未标记数据。actor 的 `_extra_transition_fields`（`real_world_rl/loops/hil_serl.py:145`）未转发该标记，learner 仅依据 `intervened` 路由（`:360`）。`tests/test_leader_arm_hil_loops.py:216` 固定了 ramp 数据进入示范区的行为。仅录制器丢弃这类数据（`real_world_rl/demos/record.py:381`）。
- **当前处理**：HIL 训练也使用 `--teleop-onset arm`，主从臂误差小于 `--teleop-arm-tolerance-rad` 才接管（`real_world_rl/teleop/leader_arm.py:200`），每次接管前需手工对齐 GELLO。
- **计划修复**：为 `joint_delta` 饱和起始步骤添加标记并转发，learner 增加 `--demo-skip-syncing`。测试验证两种模式开启该参数后都排除同步步骤。
- **状态**：未解决（S2）。

<a id="s3-taking-over-can-open-the-gripper"></a>

### S3. 接管可能打开夹爪

- **症状**：策略正抓着物体，扳机松开时按 Space，夹爪立即打开。
- **影响**：物体掉落，并产生错误的介入数据。
- **位置**：扳机直接映射为 `2w - 1`（`real_world_rl/teleop/part.py:273`）；接管策略只处理关节（`real_world_rl/teleop/leader_arm.py:161`），夹爪在 `:164` 加入。
- **当前处理**：按 Space 前使扳机匹配当前夹爪状态，抓着物体时先捏紧。
- **计划修复**：在扳机状态一致前保持最后夹爪命令。测试验证不一致接管时夹爪保持，直到扳机越过对应位置。
- **状态**：未解决（S3）。

<a id="s1---fwbw-drops-interventions"></a>

### S1. `--fwbw` 丢失介入标记

- **症状**：`hil_serl --fwbw` 运行中，`learner/demo_buffer_size` 不增长，也没有 `actor/*` 指标。
- **影响**：介入动作正常执行并存入在线缓冲区，但未进入示范缓冲区。
- **位置**：`hil_serl --fwbw` 使用普通 `FWBWActorLoop`（`real_world_rl/training/entrypoints/hil_serl.py:82`），其 `_extra_transition_fields` 为空（`real_world_rl/loops/actor.py:228`）。`residual_hil_serl` 和 `silri` 会拒绝 `--fwbw`（`residual_hil_serl.py:64`、`silri.py:173`）。
- **当前处理**：首次运行保持常规流程，暂不启用 `--fwbw`。
- **计划修复**：增加会标记 `intervened` 并上报回合统计的 FWBW actor，测试验证介入数据进入示范缓冲区。
- **状态**：未解决（S1）。

<a id="s5-silri-pretraining-stalls-the-actors-uploads"></a>

### S5. SiLRI 预训练阻塞 actor 上传

- **症状**：`silri` 启动时，行为模型预训练期间 actor 持续输出 `learner at ... unreachable`，随后 `actor/dropped_transitions` 可能上升。
- **影响**：阻塞时间超过 actor 队列容量时丢数据。
- **位置**：`SilriLearnerLoop._train_step` 在缓冲区锁内执行 `pretrain()` 和行为刷新（`real_world_rl/loops/silri.py:189`）；预训练由 `--bc-pretrain-steps` 控制，默认 500，定义于 `real_world_rl/training/entrypoints/silri.py:69`。同步服务请求线程在 `_on_transition` 等待同一锁（`real_world_rl/loops/silri.py:155`、`real_world_rl/loops/sync.py:133`）。actor 推送在 5 秒后超时（`sync.py:336`），即每 5 秒发一条，同时每秒产生 10 条；600 条队列约一分钟后开始丢最旧数据（`sync.py:204`、`:296`，静态分析，未测量）。请求超时的数据仍会在锁释放后加入。预训练也无法在 actor 启动前完成，因为它等待 `--learning-starts` 默认 10 条在线数据（`real_world_rl/loops/silri.py:171`）。
- **当前处理**：先启动 learner，再启动 actor，按 Space 保持人工控制直到警告结束；超过一分钟时降低 `--bc-pretrain-steps`。检查 `actor/dropped_transitions` 保持 0。
- **计划修复**：在缓冲区锁外预训练，或在服务器接收数据前预训练。测试验证预训练期间提交 transition 能在推送超时前返回。
- **状态**：未解决（S5）。

<a id="s7-eval-counts-your-interventions-as-policy-successes"></a>

### S7. eval 将人工介入计为策略成功

- **症状**：评估中按过 Space 后，`[eval] success_rate=...` 高于策略自身水平。
- **影响**：评估结果失真。
- **位置**：`--role eval` 构建含遥操作包装器的环境（`real_world_rl/training/env_building.py:373`），将所有正常终止回合计为成功，相关位置为 `real_world_rl/training/entrypoints/hil_serl.py:224`、`residual_hil_serl.py:209`、`silri.py:286`。
- **当前处理**：eval 期间保持策略自主执行，不按 Space，见 [06](tutorial/06-intervening.md#the-sequence)；用 Esc 结束无法挽回的回合，计为失败。
- **计划修复**：单独报告介入回合，并从 `success_rate` 中排除。测试验证包含介入步骤的评估回合不计为策略成功。
- **状态**：未解决（S7）。

<a id="docs-stale-items-in-hil-serl-roadmapmd"></a>

### 文档 `hil-serl-roadmap.md` 的过时条目

文档整理时待修正：第 8 项“仍然缺失”称没有示范录制器，但已有 `examples/record_real_demos.py`；第 4 项称双臂夹爪未实现，但 `RLPDHybrid` 已有 `3 ** num_grippers` 分类头且存在 `real_robot_bimanual`；“初次迁移后落地的工作”仍把主臂遥操作后端列为缺失；“行为差异”中的 `backup_entropy`、图像增强和池化默认值已由 `HilSerlArgs` 覆盖。状态：文档问题待解决。

<a id="b-not-planned"></a>

## B. 暂无计划的工作

- **GELLO 力矩跟随**，即主臂跟踪从臂：需要 robot_infra 的 Dynamixel 电流控制及操作员安全评估；现有 `ramp` 和 `arm` 已覆盖接管，见 [gello.md](gello.md#9-future-work-leader-tracking-wp8)。实测 GELLO 延迟后再评估。
- **放开主臂就交还控制**：没有握持传感器，操作暂停会误触发交还。使用 Space，或通过 GELLO 的 `event_device` 连接脚踏，见 [teleop.md](teleop.md#leader-arms-gello)。
- **遥操作读故障导致 actor 停止**：这是设计行为。读取失败向上传播（`real_world_rl/teleop/base.py:86`），包装器在每步之前轮询（`real_world_rl/envs/wrappers/intervention.py:147`），避免把丢失读数误当策略步骤。机械臂释放由 S6 的 `env.close()` 修复处理。
- **actor 每步执行 `policy.load_state_dict`**，位置为 `real_world_rl/loops/actor.py:72`：通过 `actor/param_refresh_ms_*` 测量，实测后决定是否修改。

<a id="c-unverified-on-hardware"></a>

## C. 硬件待验证

硬件侧仅使用 fake 设备运行；`leader_cartesian` 正向运动学未与真机比较；ACT 未用真实 pkl 训练；奖励分类器未使用真实图像。详情见[总览中的已知限制](tutorial/00-overview.md#known-limits)。

<a id="closed"></a>

## 已关闭

暂无。
