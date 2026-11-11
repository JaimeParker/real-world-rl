<!-- English source: docs/real-world-experiments.md -->

<a id="running-and-recovering-real-world-experiments"></a>

# 运行与恢复真机实验

[中文入口](../README.zh-CN.md) · [English](../real-world-experiments.md)

本指南介绍持久化数据接收、显式结果、按部件接管与轨迹归档。已有 CPU 和 fake 环境回归覆盖，硬件验证仍待完成。设备配置与调试从[教程](tutorial/00-overview.md)开始。

<a id="actor-identity-and-durable-uploads"></a>

## actor 身份与持久化上传

actor 先把 transition 提交到本地 SQLite outbox，才向采样循环确认 `push_transition` 成功。learner 先提交自己的 SQLite journal，才确认接收。重试保留 transition ID，相同 ID 和内容仅接收一次；同一 ID 对应不同内容会报错。

actor outbox 默认路径为：

```text
<log_dir>/<run_id or exp_name or sync_host_sync_port>/<actor_id>/outbox.sqlite3
```

`actor_id` 默认 `actor`，`--actor-outbox-path` 可显式选文件。未设 `--run-id` 时，消息运行身份采用 outbox 的持久 UUID；每个进程还生成 session ID 和序号。FWBW 客户端将 `_forward` / `_backward` 方向加到文件名与 actor 身份。

重启后续传需使用相同 outbox 路径和 actor 身份，并保留匹配的 learner journal/checkpoint。独立实验选择新 `--run-id` 或独立显式 outbox；只改实验标签却强制复用旧 outbox，仍会保留其待发送消息。每个 outbox 同时只运行一个 actor，SQLite 数据库和 WAL 放在本地存储。

并发 actor 各用不同 `actor_id` 和 outbox；同一 actor 重启复用原身份与路径。

FWBW learner 的输出路径也分离。显式 `--exp-name experiment` 变成 `experiment_forward` / `experiment_backward`，显式 checkpoint 根目录变成 `<root>/forward` / `<root>/backward`。已命名为 `forward` 或 `backward` 的目录直接使用，方向不符会拒绝。恢复 bundle 时选择对应方向的 manifest。

包含 journal 或 bundle 的旧扁平 checkpoint 目录方向归属不明确，会被拒绝。先识别记录属于哪个方向，再拆分。混有两个方向的 journal 需要逐记录识别分离，复制到方向目录不代表已有合法恢复历史。每个已核实方向的 journal、WAL/SHM、bundle generation 和快照需完整保留。程序不会自动移动或重写已有目录，独立旧 `.pt` 权重与回放仍可加载。

learner ACK 后 outbox 才删除数据。未确认数据可跨进程退出保留，重启再发。SiLRI 暂存的前驱 transition 也持久化，恢复时发送而不凭空添加后继介入起始标签。持久化覆盖上传或暂存 API 已成功提交的数据；机器人动作执行与提交之间突然终止进程，仍可能丢失该动作的内存结果。

outbox 和 learner journal 都用 SQLite WAL、`synchronous=NORMAL`，提交不会等待 fsync，控制循环无需每步承担同步落盘。已提交记录可跨 actor/learner 进程崩溃或强制终止保留；操作系统崩溃或断电则可能回滚本机最近提交，包括 actor 最新 outbox 数据、learner 最新 journal 数据。后者可能已获 ACK，actor 也可能已删除对应数据。

learner 不可达时，例如重启或断网，actor 继续运行；上传按 0.1 秒至 2 秒增长的退避重试，首个批次被确认后重置。首次及每连续失败 10 次打印警告，数据在 outbox 累积直到 learner 恢复。只有记录数或字节容量达到上限、磁盘写失败、或 learner 用 HTTP 4xx 拒绝记录时，采集才停止。`ActorSyncClient(max_consecutive_failures=N)` 可恢复有限容错次数，默认 `None` 持续重试。

<a id="startup-and-shutdown"></a>

## 启动与停止

`--sync-startup-timeout` 默认 30 s，actor 在 reset/step 前等待初始 learner 快照。快照含 session/version、训练阶段和策略就绪状态，learner 重启后即使新 session 版本更低仍可刷新。残差预热遵循就绪状态及 base/residual 预热设置。

`--sync-drain-timeout` 默认 30 s，正常/异常停止时先释放 env/provider，再 flush 暂存行、完成回合和 drain，同步写盘不继续占用控制。超时将未确认行留 outbox 并报告数量，原路径重启续传。强杀无法清理，守护 watchdog 仍是该故障路线的一环；无守护进程时见[看门狗与监督](#watchdogs-and-the-supervisor-process)。

<a id="watchdogs-and-the-supervisor-process"></a>

## 看门狗与监督进程

三个独立层可停止失去控制循环更新的机器人，都不能代替物理停止或固件保护。

| 层 | 所在位置 | 检测 | 动作 |
|---|---|---|---|
| 守护 watchdog，`robot-infra daemon --watchdog-s 0.5` | 机器人 PC 守护进程，仅该路线 | 伺服时超过 watchdog_s 无动作 | arm.stop，切 IDLE |
| Step watchdog，环境 watchdog_s，默认关闭 | 机器人进程内线程 | reset 后超过阈值无 step | stop 一次，下个 step 抛 StepWatchdogTripped |
| Supervisor，`rwrl supervise` | 独立父进程 | 超过 heartbeat-timeout-s 无心跳 | 对子进程组依次 SIGINT、SIGTERM、SIGKILL |

<a id="step-watchdog-in-process"></a>

### Step watchdog（进程内）

`RealRobotEnvConfig.watchdog_s`，即 YAML `real_robot.watchdog_s`，在 reset 完成后启动 StepWatchdog，reset 开始时暂停。值须超过一个控制周期。step 入口和命令返回后均签到，因此调用方慢、设备调用阻塞都可检测。触发后 stop 一次，后续 step 持续抛异常直到 reset。回合末步后因导出/保存静默触发时，下次 reset 只记录警告，除此无害。该值不属于实验契约。

它与控制循环共用进程/GIL，不能覆盖 SIGSTOP、整机挂起、持 GIL 扩展调用、进程被杀，或 stop 自己卡住；这些由 supervisor 处理。

<a id="supervisor-process"></a>

### Supervisor 进程

将 recorder 或 actor 作为监督子进程：

```bash
rwrl supervise --log runs/<run>/supervise.jsonl -- \
    rwrl record --config hil.yaml ...
```

| 选项 | 默认 | 含义 |
|---|---|---|
| `--heartbeat-timeout-s` | `2.0` | 静默到此开始信号升级 |
| `--grace-s` | `3.0` | SIGINT 到 SIGTERM |
| `--kill-s` | `2.0` | SIGTERM 到 SIGKILL |
| `--startup-s` | `30.0` | 首消息到达或此时间前不要求心跳 |
| `--long-op-s` | `60.0` | 子进程报告长操作（reset）后的静默宽限 |
| `--log` | 无 | jsonl：child pid、每次升级的单调/墙钟时间、退出码/信号 |

子进程独立 session，终端 Ctrl-C、SIGTERM、关闭终端仅到 supervisor；它转发 SIGINT 并一直等子进程退出，不先退出。返回子进程码，信号 N 为 128+N。发生升级后，即使子进程返回零也非零，并向 stderr 写原因。子进程退出后清理残余进程组。supervisor 死亡时内核发 SIGINT 给子进程，下一心跳抛 SupervisorLost。仅 Linux。

recorder/actor 通过 `robot_infra.supervise.Heartbeat` 每步和 reset 后发心跳，reset 前发长操作消息；无 supervisor 时无操作。startup-s 应长于首次 reset 前所有工作，actor 包括 sync-startup-timeout。

<a id="what-the-supervisor-can-and-cannot-do"></a>

### Supervisor 能做与不能做的事

单进程模式由进程内 franky 驱动，supervisor 不连接机器人，只能杀进程。libfranka 断连后固件 reflex 停止，状态为 Reflex，可作为最后保护；这不是自主选择减速曲线的受控停止，夹爪保留最后命令。

不能：

- 比固件 reflex 更温和地停机或独立调用 arm.stop；后者只在守护路线存在。
- 检测仍持续心跳但发错误动作/目标的循环。
- 在整机冻结、断电或机器人网络故障时发挥作用；此时靠机器人自身通信超时。
- 保证停止时间上界。SIGINT/SIGTERM 未被响应时约为心跳超时＋grace＋kill（默认 7 s），还要加 libfranka 通信超时。

<a id="recovering-after-a-supervisor-kill"></a>

### Supervisor 杀进程后的恢复

1. 确认机器人静止、工位安全、物理停止可达。
2. 读 stderr 原因及 log 的 escalate 事件。
3. Reflex 时释放外部激活装置，在 Desk 清错，或 `rwrl doctor --station robot.yaml --fix` 恢复并复查模式。
4. 被杀运行未完成收尾。SQLite outbox/journal 保留已提交行，但 recorder 最后回合及执行中的动作可能丢失。用同一 outbox 路径重启续传。

<a id="learner-checkpoints-and-recovery"></a>

## learner checkpoint 与恢复

learner journal 位于 `<checkpoint_dir>/journal.sqlite3`。接收提交独立于训练和回放修改，因此 SiLRI 长预训练不会让 HTTP 请求等待训练缓冲区锁。训练线程按顺序应用 journal 数据。

checkpoint bundle 包含模型、优化器、在线与示范回放、环形游标、介入扩展列、循环计数和 journal 游标、身份；还保留 FWBW 边界与蒙特卡洛状态、SiLRI 刷新计数。只有原子发布 `bundle_manifest.json` 后，新 generation 才成为当前版本；写入中断仍保留上一个 manifest。

恢复时同时提供匹配 checkpoint 目录与 manifest：

```text
--load-checkpoint /path/to/checkpoints/bundle_manifest.json \
--checkpoint-dir /path/to/checkpoints
```

匹配的 `--exp-name` / `--log-dir` 也可选中原 checkpoint 目录。仅传 manifest 路径不会迁移 journal；新运行目录会创建新 journal，其身份将被拒绝。manifest 始终恢复完整在线、示范和游标一致性单元。`--load-replay-buffer` 仅控制旧 `.pt` 加载，不会将 bundle 改为仅权重恢复。仅初始化权重时，选择 generation 的 `agent.pt`，并以空 journal 开始新运行。

bundle 默认包含完整回放，因此占用磁盘、保存时间和临时 CPU 内存。定期保存遵循 `checkpoint_freq`，最终保存遵循 `save_final_checkpoint`。Ctrl-C（`KeyboardInterrupt`）或 `SystemExit` 算正常停止，即使发生在更新中也会写 `final.pt` 和新 bundle。更新抛异常，或应用 journal 记录到回放时被中断，则跳过最终保存，保留最后完整 bundle，恢复时重放 journal 尾部。保持 manifest 引用的 `bundle_*` 与对应 journal 一起，身份不符会拒绝。恢复后只应用已保存游标之后的数据，不叠加初始示范或旧 pickle 快照。

显式旧 `.pt` checkpoint 仍支持，`--load-replay-buffer` 请求对应原生回放文件，真机适配器会实际恢复已有回放。原生回放没有 journal 游标，因此需要空 journal；避免把无关旧回放与当前 journal 混用。没有权威 bundle/回放恢复时，旧 `.pkl` transition 快照与初始示范路径仍是回退来源。保留原动作约定附属文件。

旧核心回放可能缺介入来源，仍可读取但会警告。明确标识为旧 `BC` 算法的 checkpoint，在没有介入标签与掩码时保留全维度监督意图；显式 `intervened` 及已有空或部分掩码优先。动作来源、策略提案和回合结果仍为未知。未带该算法身份的原生回放，包括来源未知在线回放，不会获得专家标签；记录动作仅作为未知策略提案的占位。缺少算法要求的基础动作或已配置惩罚列时无法重建，应使用匹配 transition 快照或完整 bundle。旧五字段示范仍可读取，调用缓冲区 `add` 前投影掉归档元数据。版本化归档把部分人工掩码与回放训练视图区分保存。

<a id="reading-success-and-intervention-metrics"></a>

## 阅读成功率与介入指标

评估默认保留人工接管。主要 `success_rate` 是 `autonomous_success_rate` 的别名：

```text
successful completed episodes with no action intervention / all completed episodes
```

`task_success_rate` 包含辅助成功；`episode_intervention_rate` 统计已完成回合中有动作介入的比例；`step_intervention_rate` 为已完成回合介入步数占这些回合总步数的比例。未知结果保留在成功率已完成分母中，并报告 `unknown_episodes`。中断、重录和丢弃尝试单独计数，不进入该分母。

成功来自显式环境、分类器、人工标签或遥操作成功事件。正奖励和 `terminated=True` 本身不足以建立成功。人工失败否决成功，同时出现成功失败事件也一样。只按结果标签键、未接管动作，不会使回合变成辅助回合。报告保留结果来源和结束原因，以区分操作员与分类器判断。回放 `done` 仍等于 `terminated`，超时截断按原算法约定继续 bootstrap。

<a id="part-takeover-and-imitation"></a>

## 按部件接管与模仿

`--teleop-selected-parts` 选择具名部件，例如双臂的 `left.arm`、`right.arm`、`left.gripper`、`right.gripper`，单臂为 `arm.arm` 和 `arm.gripper`，名称遵循环境 `arm_names`。空列表使用输入源可用部件。动作布局保持所有机械臂块在前，包含夹爪时每臂一个夹爪通道在后。

默认 `--teleop-idle-behavior hold`，部分接管期间，未选机械臂在 `joint_abs` 下保持当前从臂姿态，增量模式使用零增量；夹爪保持最后目标。`--teleop-idle-behavior policy` 让未选部件继续使用策略。`human_action_mask` 标记真实人工控制，`override_mask` 还包括保持补充。空人工掩码没有专家标签，保持补充不训练模仿损失。旧全动作介入沿用完整掩码行为。

主臂追赶会实际执行并归档。`joint_abs` 的 `ramp` 每个限速步骤都是追赶；`joint_delta` 仅将接管到主臂首次进入从臂一个 `joint_scale_rad` 范围内的初始步骤视为追赶，同次接管之后因操作员更快而产生的饱和步骤正常教学。追赶记录的 `syncing` 禁用示范监督。录制器 `--keep-syncing-steps` 将其作为普通监督保留在 pickle，归档仍保留 `syncing`，归档训练视图也继续排除。无夹爪通道主臂只保持前一目标，不提供专家夹爪标签。

BC、SiLRI 行为拟合及 actor 预训练只监督人工维度。连续 NLL 每个监督数据行采用 `D / k * sum(mask * NLL)`，`D` 是完整连续宽度，`k` 是该行监督宽度。空掩码不贡献梯度，完整掩码恢复原目标。残差 BC 还与非零残差缩放求交集。SiLRI 联合夹爪头使用已观测人工夹爪的边缘概率，对未观测夹爪选项求和；在线行为先验距离和乘子约束仍保持完整连续动作语义。

无夹爪 HIL 使用本地连续分支 `ContinuousHilRLPD`。连续残差和连续 SiLRI 也接受无夹爪约定；混合或离散夹爪分支要求至少一个夹爪，在采样前拒绝零夹爪配置。

<a id="trajectory-archives-and-training-views"></a>

## 轨迹归档与训练视图

录制在成功示范 `.pkl` 旁创建 `<output stem>.trajectory.sqlite3`。回合开始、每步和回合结束均通过与接收相同的 SQLite 持久层提交。归档含策略提案、实际动作、human/override 掩码、同步、终止与截断、奖励分量、结果来源和实验身份，保留失败与重录。迭代采用分页，重新打开不会把历史相机帧全部加载成内存列表。

突然停止后，再开同一归档会将未结束回合标为中断，后续尝试使用新回合 ID。恢复保证从 durable append 返回时开始；此前已执行但未提交的动作仍存在执行到提交的间隙。`--export-trajectory-pt` 显式生成可移植普通记录导出，SQLite 仍是默认归档和恢复来源。

`--demo-dataset-paths` 接受旧 `.pkl`、`.trajectory.sqlite3` 和显式 `.trajectory.pt` 导出。归档视图选择完整成功回合中的非同步人工步骤，保留原始 step index，过滤不会虚构时间邻接或将终止奖励转移到较早人工步骤。缺介入列的旧人工示范沿用全维度监督；显式 false 标签与来源未知在线 checkpoint 保持非专家身份。

版本化实验约定包含比较部分和来源部分。比较部分决定策略看到什么及动作如何解释：任务、控制设置、`control_hz`、`joint_limits`、深度相机、`image_size`、图像预处理、最终观测布局（键、形状、相机顺序、帧堆叠），以及遥操作选择和空闲行为。录制、训练、部署、评估需一致，不符即拒绝；仅这部分进入实验哈希。来源部分随约定保存并显示于差异错误，但不参与比较：奖励来源，包括分类器路径、内容、阈值、`reward_from_teleop_events`、`teleop_success_reward`、抓取惩罚，以及命令行管理的工作站事实 `safety_box`、`home_joints`、`reset_settle_s`、`max_episode_steps`、`cameras`。相机名称和顺序通过观测布局比较。因此按键奖励示范可用于分类器训练运行，部署也可更改归位姿态或回合上限。早于实验元数据的数据通常带警告加载，除非运行任务、控制、深度或预处理必须依赖这些元数据。双臂保留逐臂结果，一臂成功不能确认整个并行任务；全任务确认使用 `task_success` / `task_failure`。checkpoint 与 actor 参数还核对策略动作边界、残差缩放和离散动作缓冲区；动作宽度相同但含义不同会在 reset 前拒绝，分离夹爪头时连续头维持自身形状。


<a id="task-presets-and-observation-settings"></a>

## 任务预设与观测设置

无需算法训练即可运行完整本地 fake 任务与 reset 路径：

```bash
PYTHONPATH=.:rl-garden:robot_infra python examples/task_smoke.py --config examples/presets/pick_and_place_fake.yaml
PYTHONPATH=.:rl-garden:robot_infra python examples/task_smoke.py --config examples/presets/parallel_pick_and_place_fake.yaml
```

这些预设展示版本化任务、reset 路径点与扰动、控制、终止和评估覆盖。`success_source=tcp_proxy` 报告 TCP 位置代理，主要任务结果仍未知。真实抓取放置使用显式人工或分类器标签。并行分类任务要求每臂一个 `classifier_checkpoints`，可逐臂指定 `classifier_image_keys`，所有必需臂都成功才算成功。

逐相机 `preprocess` 支持像素 ROI `[top,left,height,width]`、源分辨率 `[height,width]` 与米制深度裁剪。在环境和观测中分别配置匹配的深度相机和键。RGB 用双线性，深度用最近邻、米制单通道、无效零像素。请求和实际解析的控制、安全、SDK 设置保存于运行和 generation 元数据。全局与环境频率均非默认且冲突时拒绝，actor 和部署采用解析频率。

控制频率、动作缩放、reset 时长，以及工作站和环境几何参数都需有限值。非法动作在任何机械臂命令发出前拒绝；双臂在任一臂步进前核对完整动作。环境安全限制只能收紧工作站限制。

分类器文件移动后，部署接受替换的 `classifier_checkpoint` 或任务 `classifier_checkpoints`。只覆盖任务路径时，其他任务字段可保持默认，由已保存任务补齐。分类器路径和内容属于来源信息，因此可部署重训分类器，并将新内容哈希与训练时哈希共同记录。
