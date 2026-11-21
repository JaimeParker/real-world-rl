路线三 2. 准备一个可训练的任务
================================================================================

本章让 :doc:`record` 中的同一个任务有明确的 reset、结束条件、成功标签和奖励。
沿用人工事件这条已支持的路径：操作员看见任务完成后按 s，看见失败后按 f。
``env_id: pick_and_place`` 只是实验名称；它本身不会检测物体是否放好。

1. 从已验证会话生成训练配置
--------------------------------------------------------------------------------

先看配置模板，再从 ``first-demo`` 的真实 session 合入观测、动作与遥操作参数。
不要直接把下面的模板当作已经标定好的现场配置。

.. literalinclude:: ../examples/rl-task.yaml
   :language: yaml
   :start-after: # start config
   :end-before: # end config

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - 必须改的字段
     - 怎么得到这个值
   * - ``obs``、``real_robot`` 的动作、状态、尺度、工作区字段
     - 从上一章实际使用的 session/resolved-config.yaml 复制；下面的脚本整块保留。
       inspect 输出 rgb_wrist 为 [1,128,128,3] 时，对应 rgb: [wrist]、image_size: [128,128]。
   * - ``real_robot.remote``、必要时 ``token``
     - 从 station 的 daemon 监听地址／端口取得。actor 与 daemon 同机、端口为 50051 时，
       填 127.0.0.1:50051；启用 token 的站点复制匹配 token。脚本删除原直接连接 robot 块。
   * - ``teleop_device``、``teleop_options_yaml``、``teleop_input`` 等
     - 由 session 复制。SpaceMouse 应为 spacemouse；前章 GELLO 笛卡尔路径应为 leader_cartesian。
       保留 session 目录中的 leader-options.yaml、leader-robot.yaml 及其引用关系。
   * - ``real_robot.max_episode_steps``、``reset_settle_s``
     - 本路线前章分别为 200 步和 0.5 秒；脚本以真实 session 为准。用已验证频率计算任务预算，
       在现场确认等待足够后才改，不根据 learner 的运行速度定回合长度。
   * - ``reward_from_teleop_events``、``teleop_success_reward``
     - 本章选择 true、1.0；成功判据由操作员按任务定义确认，例如物体已稳定放入目标区域。
       按键测试见第 3 节，不能由回合回报反推真实成功。
   * - ``--demo-dataset-paths`` （通过 CLI 传入）
     - 使用 :doc:`inspect` 已确认的 local/first-demo.pkl，保留同名 contract.json；
       当前 YAML 的 Sequence 解析限制见下一节，因此不把路径列表写进模板。
   * - ``buffer_size``、``demo_buffer_size``
     - 模板各 1000 行，只用于短会话。用 :doc:`inspect` 得到的 pkl transitions 数调整 demo 容量，
       至少容纳全部种子示范并给在线纠正留空间；估算图像内存后再延长会话。
   * - ``log_dir``、``exp_name``、``run_id``、``actor_id``
     - 本次分别用 local/rl-runs、first-hil、first-hil、actor；独立实验换新名字及 outbox。
       恢复同一实验则保留，详见 :doc:`rl-resume`。

其余字段保持模板设置：buffer_device=cpu；命令用 CUDA_VISIBLE_DEVICES 隐藏 GPU，batch_size=32，learning_starts=100，
demo_data_ratio=0.5，train_freq=1，utd=1，publish_freq=10，log_freq=10，
checkpoint_freq／buffer_period=100，save_final_checkpoint=true。
这些是短会话的教程值，不是 HIL-SERL 全部默认值或推荐收敛参数；数据量和采集速度稳定后再调整。
classifier_checkpoint=null、use_grasp_penalty=false 保持奖励来源单一。
HIL-SERL 的 Args 没有 BC 入口的 device 字段；策略构造使用自动选设备，所以本章命令显式隐藏 GPU。

先列出上一章的产物，再把命令里的 ``<uuid>`` 换成实际目录名：

.. literalinclude:: ../examples/rl-task.sh
   :language: bash
   :start-after: # start files
   :end-before: # end files

.. literalinclude:: ../examples/rl-task.sh
   :language: bash
   :start-after: # start prepare
   :end-before: # end prepare

生成脚本保留 session 的模型输入、动作和 teleop 标定，仅改成 remote 连接与本章训练设置。
``local/station.yaml`` 和 ``local/task.yaml`` 保留原样。将生成的配置、pkl、contract 和
被引用的 teleop 文件复制到 learner 机器；路径须在那里也能解析。learner 读取标定配置但不打开设备。

**你应该看到**：脚本打印 config 路径、remote、action_mode 和 teleop_device。
应为 cartesian_delta，且与你选择的输入设备一致。用文本编辑器检查 real_robot 中不再有直接 robot 配置，
home／workspace／控制设置仍来自原会话或同一个 daemon station。

**如果不对**：FileExistsError 时检查已有 local/rl-task.yaml，不覆盖正在运行的实验；
脚本拒绝 joint 动作时选回前章笛卡尔 session；GELLO 引用文件丢失时复制完整 session；
daemon 未配置时先按 :doc:`deploy` 的设备服务流程建立连接，不猜 remote 地址。

2. 把复位、结束与奖励分开核对
--------------------------------------------------------------------------------

本路线保留 ``task.name: none``，复用前章示范的实验约定，通过事件 wrapper 提供任务标签。
reset 使用原 real_robot.home_joints，未设置时读取 station 的 home；随后开爪并等待 reset_settle_s。
home 的七个值来自 :doc:`station` 的 q 读数和已审阅起始姿态，不在这里另给通用关节角。

``episode_end_behavior: hold`` 使 actor 在回合结束后保持并等待 n，再经过中立检查进入下一次 reset。
它不改变回合内策略和人工的交还逻辑。``max_episode_steps`` 是截断上限。
成功事件令 reward 至少为 1、terminated=True；失败事件优先于同一步成功，令
task_success=False、truncated=True。本配置底层 reward 为 0，所以失败行 reward=0；
失败 wrapper 本身并不保证把任意分类器奖励清零。Esc 只结束回合，不补成功标签。

只做解析预检，确认字段及路径来源：

.. literalinclude:: ../examples/rl-task.sh
   :language: bash
   :start-after: # start preflight
   :end-before: # end preflight

**你应该看到**：解析后的 real_robot、teleop、reward 和 demo_dataset_paths。
数据路径通过 CLI 传入；当前严格 YAML 解析器不能把 list 当作该 Sequence 字段读取。
print-config 不会确认示范内容和硬件可用性；第三章 learner 真正加载时还会核对 contract。

如需自动成功检测，代码支持 classifier_checkpoint、classifier_threshold（默认 0.5）和
classifier_image_keys；必须先另行采集成功／失败图像并验证分类器。
本章不运行分类器：其构造路径使用 get_device("auto")，仅指定 buffer_device=cpu 并不能限制分类器设备。
版本化 ``real_robot.task.name: pick_and_place`` 还支持分阶段 reset、任务奖励和结束配置，
但改变 task 会改变实验契约，不能直接接上这里 task=none 的旧示范。
``tcp_proxy`` 只判断 TCP 接近位置，不能证明抓取／放置成功。

**如果不对**：``Preset field 'demo_dataset_paths' must have type 'Sequence', got list`` 时
从 YAML 删掉该项，按命令行传入；``recorded action contract does not match`` 时逐项对照 session，
不要删除 sidecar；无标签时检查 stdin 是否确实连接到 actor 终端及 reward_from_teleop_events=true。

3. 确认一条成功和一条失败
--------------------------------------------------------------------------------

先检查已有录制归档；若前章没有失败回合，回到 :doc:`record` 再录一轮明确失败与一轮成功，
换新的输出名并对应替换下面路径。失败仍在 SQLite，默认不会导出到成功人工示范 pkl。

.. literalinclude:: ../examples/rl-task.sh
   :language: bash
   :start-after: # start labels
   :end-before: # end labels

**你应该看到**：两个不同 episode_id 的 episode_end.outcome，task_success 分别为 true 和 false，
outcome_source 都为 manual；失败的 end_reason 为 manual_failure。
再读同回合最后 transition，核对 reward、terminated、truncated。
脚本以只读连接检查 SQLite；仅打开自己生成或可信的归档。

第三章提供不用硬件的 CPU 练习。以下是该练习实际输出的两条回合结束记录，
UUID 与时刻由当次运行产生；并非真实任务标签：

.. literalinclude:: ../examples/rl-intervene-output.txt
   :language: json
   :start-after: # start labels
   :end-before: # end labels

**如果不对**：只在 pkl 中查不到失败是正常筛选；unknown／timeout 表示没有明确结果，
不能当成 failure 标签；outcome_source 不是 manual 时检查是否额外启用了分类器或版本化任务。

.. note::

   **本章哪些内容需要真机确认**

   home 路径、开爪、物体布置、工作区、复位等待、回合预算和人工判据都要现场确认。
   CPU 例子只证明成功与失败字段可以区分。新分类器必须独立验证，本章没有训练或部署分类器。

检查清单：配置来自同一 session；两台机器都有数据与引用文件；成功／失败可在 SQLite 中区分；
知道 reset、hold 和截断各自何时发生。

下一章 :doc:`rl-train`。
