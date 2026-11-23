部署本仓训练的 BC 策略
================================================================================

本章使用 :doc:`bc-check` 留下的 ``local/bc-runs/first-bc/checkpoints/final.pt``，
先恢复配置和检查保存的观测，再安排一次受监护回合。
保留同目录的 config.json、experiment.json 和 offline-check.json。
已有 openpi／RLinf 权重走入口 B：:doc:`deploy-openpi`。

本章的 BC 入口是 ``real_world_rl/deploy/checkpoint.py``，对应
``rwrl deploy --algo real_world_bc`` 和 ``real-world-rl-deploy``。
``examples/deploy_real_robot.py`` 转发到同一入口；``deploy-act`` 加载的是另一种 ACT 权重。
本入口没有 dry-run 参数。本章离线部分只恢复配置或预测保存的观测，不调用 reset／step。

命令从仓库根目录执行，使用已安装 learning 依赖的 Python；源码环境先设置：

.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start environment
   :end-before: # end environment

下方实测输出来自 rlg 环境的 CPU，合成数据与四步训练按 :doc:`bc-train` 重新生成。
合成 checkpoint 只验证加载流程，不进入真机回合。

先按训练方法选择入口；下面第 1–4 步仍以 BC 为例。

.. list-table:: checkpoint 与部署算法
   :header-rows: 1

   * - 训练方法
     - deploy 的 --algo／独立入口
     - 权重
   * - BC
     - real_world_bc
     - 对应 run 的 final.pt
   * - HIL-SERL（带教程夹爪）
     - rlpd_hybrid
     - 对应 run 的 manifest 或 .pt
   * - residual_hil_serl
     - continuous 用 residual_hil_serl；hybrid 用 residual_hil_serl_hybrid
     - 残差 run 的 manifest 或 .pt，另需基础模型
   * - SiLRI
     - silri
     - 对应 run 的 manifest 或 .pt
   * - ACT
     - rwrl deploy-act（第 6 步）
     - ACT 训练器的 final.pt 及示范 contract

manifest 解析到所引用的一代模型，部署不恢复 replay，也不连接 learner。

1. 恢复配置，对照当前工作站
--------------------------------------------------------------------------------

先把模型实际使用的配置打印出来，与上一章的输入输出记录和现场配置逐项比较。
``deploy-bc.py`` 是本章辅助脚本；默认只调用部署模块的 restore_training_args 和
resolve_deploy_robot_config，不建立设备连接。

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**你应该看到**：下面是合成 checkpoint 的实际输出。obs 决定 RGB 相机和图像尺寸；
这里 real_robot.cameras 为空并不表示模型不使用相机，构造环境时还会应用 obs。

.. literalinclude:: ../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start config
   :end-before: # end config

按 offline-check.json 核对以下项目，不能只比较动作长度：

.. list-table:: 部署前对照
   :header-rows: 1
   :widths: 30 70

   * - 模型记录
     - 当前工作站要核对什么
   * - cameras、observation_shapes、observation_dtypes
     - local/task.yaml 的 obs 与恢复结果一致；station 中有 wrist，实际视角、序列号和安装位置与示范一致。
       本例 rgb_wrist 为 [1,224,224,3] uint8；也核对 depth、preprocess、frame_stack。
   * - state_fields
     - 本例按 tcp_pose、tcp_vel、joint_pos、joint_vel 拼成 27 维；保持顺序、单位、xyzw 四元数和任务坐标系。
       核对 experiment 中是否有旋转转换。
   * - action_conventions、action_contract
     - 本例 current / cartesian_delta，七维，位置尺度 0.05 m、旋转尺度 0.3 rad、10 Hz。
       夹爪经 (a+1)/2 变为开口请求；另核对 station 的二值阈值、joint_limits、workspace、control 和 teleop 来源。
   * - station 与原 session resolved-config
     - 设备、home、工作区、单步限制和控制设置须与已审阅配置一致。连接地址可以换，不能把换相机或动作语义当成换地址。

部署从 config.json 和 checkpoint 元数据恢复训练参数，训练的观测／动作约定优先；
CLI 提供连接地址。它不会自动读取当前 local/task.yaml 或 local/station.yaml。
后面的远程部署由 daemon 加载 station，服务和隧道按 :doc:`deploy-openpi` 配置，
本章沿用那里 GPU 端的 127.0.0.1:50061。

移动目录后，用 ``--config-path`` 指定训练时的 config.json；deploy 不接受训练入口的 --config。
保留 experiment.json 与 bundle 引用的一代文件；残差基础模型另搬位置时用
``--base-provider`` 指定实际规格，不能只搬残差权重。

分类器移动后可用 ``--classifier-checkpoint`` 指向新位置；逐臂任务的分类器映射在相应 task 配置中替换。
替代分类器内容属于来源信息，但任务、控制、预处理、观测布局和接管部件仍要匹配原实验。
换一个模型路径不是解除其他契约检查。

用一个故意冲突的尺度验证拒绝路径；此命令仍然离线：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start mismatch
   :end-before: # end mismatch

**你应该看到**：进程失败，异常末尾如下，来自部署模块的真实检查：

.. literalinclude:: ../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start mismatch
   :end-before: # end mismatch

**如果不对**：

* ``contradicts the training run's real_robot config``：按列出的字段查训练记录，不把模型尺度改成现场想要的数。
  核对无误后去掉冲突覆盖，让部署恢复训练值。
* 没报错也不能跳过对照：CLI 检查只把“不同于 dataclass 默认值”的字段视为显式覆盖；
  相机实际视角、单位和安装变化也无法由这项检查确认。
* 缺 config.json／experiment.json 或记录属于另一个 checkpoint：先找回完整 run 和示范来源，返回 :doc:`bc-check`。
* 当前 task、状态顺序或相机与记录不同：先修正配置或重新准备数据和模型；不要继续执行回合。

2. 先检查一条保存的观测
--------------------------------------------------------------------------------

复用上一章的离线检查脚本，为这次部署单独保留预测结果。
只加载自己生成或可信的 checkpoint 和 pickle。

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start offline
   :end-before: # end offline

合成练习用下面这一条替代，不要把合成数据改名为真实示范：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start synthetic
   :end-before: # end synthetic

**你应该看到**：真实 CPU 输出如下；deterministic=true，模型处于 eval 模式。

.. literalinclude:: ../examples/deploy-bc-output.txt
   :language: text
   :start-after: # start offline
   :end-before: # end offline

预测 x 约 -0.1588，按本例尺度请求约 -0.0079 m；它是请求增量，不是实测位移。
模型还输出了示范没有的其他方向运动。这份四步合成模型不能用于现场任务。
自己的模型应检查多个独立回合的观测，再决定是否进入下一步。

**如果不对**：

* ``recorded action contract does not match`` 或 ``saved experiment contract/hash differs``：
  按错误字段核对数据和训练目录，不删除 contract 来绕过验证。
* ``prediction/action/mask shape mismatch`` 或 ``non-finite prediction or no supervised dimensions``：
  返回 :doc:`bc-check` 检查输入、动作和 mask。
* ``FileExistsError``：保留旧记录，给 --record 一个新文件名。

3. 运行一次受监护回合
--------------------------------------------------------------------------------

在通过现场核对后，用同一部署循环执行一个最多十步的回合，并保存逐步记录。
先查看真实 CLI 参数：

.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start help
   :end-before: # end help

标准入口的等价命令如下，只打印回合摘要，不保存逐步轨迹。
它和后面的记录命令二选一；本章选择后者。

.. warning::

   以下两个命令都会连接设备、reset 归位并开爪，然后发送策略动作。
   先确认真实模型、home 路径、工作区、夹爪和物体摆放，保持现场监护与物理停止装置可用。
   本例关闭遥操作接管，SpaceMouse 不是停止按钮。
   按 Ctrl-C 中断当前进程会进入环境清理、断开设备；必须现场确认机械臂实际停止，
   异常运动时立即使用物理停止装置。十步上限不是运动安全保证，也不包括 reset 时间。

.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start cli
   :end-before: # end cli

本章脚本调用 build_deploy_env、restore_policy 和 run_deploy，增加记录包装，
不改变策略输出。它限定 BC 那几章的 task.name=none、CPU、确定性推理、一个回合、十步上限，
并关闭遥操作；自定义任务另行核对任务配置与评价逻辑。
执行下面这一条，同时保存终端输出：

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start episode
   :end-before: # end episode

**你应该看到**：终端先打印恢复配置，正常结束后出现 ``[deploy] episode=0`` 和
``[deploy] summary``，包含 steps、return、truncated、RTT 和健康状态。
本次没有真机执行，因此不提供虚构的回合输出。BC 那几章默认没有成功判定；
达到十步通常是截断，return 为零不能解释为任务成功或失败。

**如果不对**：

* ``[deploy] device error`` （标准 CLI）或设备异常（辅助脚本）：按 hint 排查；链路与 daemon 诊断见 :doc:`deploy-openpi`。
* ``FileExistsError``：辅助脚本拒绝复用输出目录；改成新的 --output，并同步修改 tee 日志路径。
* 构造环境时报约定／权重不匹配：停止，回查完整训练配置、experiment 和第 1 节记录。
* 动作方向、速度或健康状态异常：停止回合，保存日志，重新检查约定；不要增大限制继续试。

标准 deploy CLI 捕获设备错误后向 stderr 打印 ``[deploy] device error: ...``，
有提示时再打印 ``[deploy] hint: ...``，退出码为 1；命令行解析错误退出码为 2。
契约不匹配在 reset 前抛 ValueError，不属于上面的设备错误摘要。
这些退出约定针对标准 deploy，不套用到本章辅助脚本或 deploy-act。

4. 读取策略动作、实测运动和结果
--------------------------------------------------------------------------------

用相同 step 编号对照请求和反馈，区分策略想做什么与机械臂实际做了什么。

.. tutorial-test: skip
.. literalinclude:: ../examples/deploy-bc.sh
   :language: bash
   :start-after: # start records
   :end-before: # end records

**你应该看到**：辅助脚本写入以下文件；标准 CLI 本身不生成这些 JSON 文件。

.. list-table:: 本章执行记录
   :header-rows: 1
   :widths: 40 60

   * - 路径
     - 内容与读法
   * - local/deploy-bc-episode/steps.jsonl
     - reset 行保存初始 state；每步 proposal 行保存 policy_action 和时间，随后 feedback 行保存
       state、tcp_pose、reward、terminated、truncated。每行立即 flush。
   * - local/deploy-bc-episode/result.json
     - 正常结束后的 EpisodeResult：steps、episode_return、truncated、rtt_ms_mean、health_ok、outcome。
       task_success=null 表示未知，不能用 health_ok=true 代替成功判定。
   * - local/deploy-bc-episode.log
     - tee 保存配置、部署摘要和异常。逐步记录不含图像；原 checkpoint、offline-check 和 station/task 仍须一起归档。

本例 state 前七项是 TCP 的 XYZ 和 xyzw；用 reset 的 state 与第一条 feedback，
随后用相邻 feedback 的 state 比较位置变化，单位 m。关节反馈在 state 的第 13–19 项
（从 0 开始编号），单位 rad；tcp_pose 来自环境 step 的设备反馈。
这些是采样时刻的位姿，不是整段运动轨迹或已稳定到目标的证明。
不要把四元数逐项相减当作旋转角，也不要把归一化 policy_action 当成实测位移。

本例无人工接管，proposal 是传给 env.step 的动作；环境映射、裁剪和设备限制仍可能改变目标。
现场另记任务实际成功／失败、判定依据、物体状态和停止是否正常，放在同目录的
``operator-notes.txt``，不要改写机器记录中的未知标签。

**如果不对**：

* 只有 proposal 没有同编号 feedback：step 未正常返回，动作可能已经发送；不能认定没有运动。
* 有 interrupted 行、没有 result.json：这是未完成的尝试；保留 steps 和终端异常，不补造完成回合。
* state／tcp_pose 缺失或与布局不符：停止解释位移，回查观测字段与反馈链路。
* 实测变化与请求差异大：检查裁剪、工作区、控制周期和设备反馈，不能只看累计 reward。

5. 替代：部署残差权重或保留人工接管
--------------------------------------------------------------------------------

已完成 :doc:`rl-train` 的 ACT 残差训练时，在机器人 PC 保留完整 run 和基础模型，
用下列命令替代第 3 步。BC 基础的残差运行则换成自己的 first-residual-bc 目录及 checkpoint: 基础规格。
下面示例使用训练时保存的连接配置；地址变动时只覆盖已核对的连接字段。

.. warning::

   本节命令会 reset、开爪并执行策略。先结束在线 actor，核对基础模型及残差的动作约定，
   保留现场监护和物理停止手段。没有接管不表示无人值守运行已被验证。

.. literalinclude:: ../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start residual
   :end-before: # end residual

**你应该看到**：与第 3 步相同的 [deploy] 回合和 summary 行。
**如果不对**：基础路径移动时改 --base-provider 的路径，保留 temporal_agg 等原选项；
残差 variant 错误时改用训练时算法，不通过改动作宽度来适配。

要保留 GELLO 接管，必须使用在该运行中验证过的 leader_cartesian 配置、引用文件和 stdin 事件源。
下面针对 GELLO 训练得到的 HIL-SERL 权重；SpaceMouse 运行不能直接换成这个设备名。

.. literalinclude:: ../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start assisted
   :end-before: # end assisted

enable_teleop 默认 true，但 task.name=none 时还需显式指定 teleop-device 才启用源组合；
版本化任务也会触发组合。设备名无默认值，来自显式参数或保存配置。
``--no-enable-teleop`` 则不构建介入源；station 已声明的设备仍可能在设备初始化时打开。

**你应该看到**：t 接管、h 交还，s/f 标注结果，Esc 或 q 结束当前回合；报告含介入和自主／任务成功率。
本例只有一个回合，结束后退出；通用 deploy 不检查 operator_quit 来退出多回合循环，
多个回合时仍会继续 reset。要中断整个部署进程使用 Ctrl-C。
仅标注 s/f 不算动作介入；真正接管的成功回合只计任务成功，不能算自主成功。
**如果不对**：主臂读取失败会停止部署，先检查引用文件和设备占用；
SpaceMouse 应显式选 spacemouse 并保持原 tracker／SERL 契约，参见 :doc:`rl-intervene`。

双臂模型须选择 ``--env-backend real_robot_bimanual``，恢复两臂的观测、动作和连接设置，
按 :doc:`../reference/bimanual` 核对逐臂配置。它不是把单臂 checkpoint 改一个开关就用于双臂。
双臂 reset 会涉及两臂运动，须先核对各自 home 和相互避碰空间。

6. 替代：独立运行 ACT
--------------------------------------------------------------------------------

使用 :doc:`bc-train` 第 4 步的 ACT 权重。这里明确提供录制时的任务配置和示范 contract，
不把 ACT 文件交给 real_world_bc。local/rl-task.yaml 必须由同一录制 session 生成；
保留原模型的 state、相机和动作设置，部署会在 reset 前核对示范契约与模型宽度／相机顺序。

.. warning::

   下面命令会连接设备、reset 并执行 ACT 动作，关闭人工接管。
   完成 :doc:`bc-check` 的约定检查后才可在现场监护下运行；保持物理停止装置可用。

.. literalinclude:: ../examples/deploy-alternatives.sh
   :language: bash
   :start-after: # start act
   :end-before: # end act

ACT 默认 temporal_agg=true，回合开始时清空块／集成状态；
``--no-temporal-agg`` 改为逐块回放，不应在评估中途混用两种设置。
``--demo-contract`` 要的是 :doc:`bc-train` 第 4 步 ``rwrl dataset export --format pkl`` 在 pkl 旁写出的 ``.contract.json``。

通用 deploy 与 deploy-act 共用 run_deploy，不执行训练 eval 角色的 START_NEUTRAL／
EPISODE_HOLD／n 流程，也不在循环里按 control_hz 补足等待；保存的 hold 配置不会改变这一点。
需要该中立检查、回合保持和循环节拍时，使用 :doc:`rl-resume` 的对应训练方法 eval 入口。

**你应该看到**：通过加载／约定检查后出现 [deploy] 回合及 summary。
本次未执行 ACT 真机回合，缺 torchvision 的本地环境也未完成 ACT 模型加载。
**如果不对**：缺 --checkpoint 或 --demo-contract 会拒绝；相机或宽度不匹配时找回匹配的配置和示范 contract（数据集头的 action_contract，或导出 pkl 时写出的 sidecar），
不要编辑 contract 使检查通过。部署无训练更新和数据上传，评估结果另行归档。

.. note::

   **本章哪些内容需要真机确认**

   CPU 合成训练、配置恢复、尺度冲突拒绝和保存观测推理已实跑。
   相机视角、状态单位、home／reset 路径、动作映射、实测运动、控制时序、
   受监护回合、逐步现场记录、停止效果和任务结果都需要真机确认。

检查清单：进入真机强化学习前
--------------------------------------------------------------------------------

* 已对照 offline-check 验证 actor 将使用的观测和动作配置，能解释策略怎样产生动作。
* 知道如何停止当前回合，能定位策略请求、实测反馈和回合结果，能区分未知、截断和成功。
* 保存工作站与实验配置；真机强化学习（:doc:`rl-overview` 起）复用它们，HIL-SERL 的策略初始化会单独说明。

远程服务、隧道与更完整的部署检查见 :doc:`deploy-openpi`。
接下来 :doc:`rl-overview`，沿一个时间步看在线实验如何运行。
