检查录制结果
================================================================================

本章接着 :doc:`record` 的 ``local/first-demo`` 输出，全部操作离线，不连接机器人。
先看数据集里有哪些回合、回放一遍，再用检查器量出接管区间，最后检查训练数据；
目标是说清楚“哪一回合的哪几步由我控制，控制了哪些维度”。
本章只用不修改数据集的命令；会修改数据集的 ``drop``、``undrop``、``prune`` 在 :doc:`../reference/recording-demos` 介绍。

1. 列出回合并回放
--------------------------------------------------------------------------------

先列出数据集里的所有回合，包括没有保留的：

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start list
   :end-before: # end list

**你应该看到**：每个回合一行，列出 status（kept 或 discarded）、原因、是否成功、步数、人工接管占比、时长和备注数，
末尾一行汇总回合数、保留数、丢弃数和保留的 transitions（训练会用到的行数）。例如：

.. literalinclude:: ../examples/inspect-list-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

再用 Rerun 查看器回放。这一步需要 viewer extra：``pip install -e '.[viewer]'``。

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start view
   :end-before: # end view

**你应该看到**：每个回合是查看器里独立的一条 recording，在 recording 列表里选；名字形如
``episode_000007 · kept`` 或 ``episode_000003 · discarded (rerecord)``。
每个彩色／深度相机一个画面，与关节、TCP 位置和姿态、夹爪、``action`` 对 ``policy_action`` 的曲线共用时间轴，
另有接管区间、成功／失败事件和 status 的标注，以及一块回合摘要（状态、备注、历史）。拖动时间轴，相机画面和曲线同步。

只看部分回合、在远程机器上看，或存成文件带走：

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start view-options
   :end-before: # end view-options

``--web`` 在机器人电脑上起网页查看器并打印地址（默认网页端口 9090，gRPC 端口 9876），``Ctrl-C`` 停止。
机器人电脑没有显示器时，在自己的电脑上做端口转发再打开打印出的地址；两个端口都要转发，因为地址里带着 gRPC 端口：

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start forward
   :end-before: # end forward

``--save x.rrd`` 不开查看器，只写一个 ``.rrd`` 文件；用 ``rerun x.rrd`` 打开，同样在 recording 列表里选回合。

**如果不对**：

* 提示需要 rerun-sdk：安装 viewer extra（``pip install -e '.[viewer]'``）。
* 看不到画面：回合没有相机键时只有曲线；``video/`` 缺失时查看器改用无损帧。
* 看过后想放弃某个回合：``rwrl dataset drop``，可逆，见 :doc:`../reference/recording-demos`。

2. 用检查器量化接管区间
--------------------------------------------------------------------------------

用检查器列出回合、人工标记和实测位移，并导出可逐行核对的 CSV。``archive inspect`` 直接接受数据集目录（也接受旧的 SQLite 归档）。

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**你应该看到**：JSON 汇总中的 records、episode_starts、episode_ends、transitions、
human_mask_counts、outcomes 和 episodes。steps.csv 每行对应一个环境步骤，
measurements.csv 每行对应一个区间指标。数据集以只读方式打开。
每个回合（kept 或 discarded）都算一个 episode_start 与 episode_end；``episode_id`` 来自各回合 ``meta.json``。

``human_action_mask`` 是与动作等长的布尔标记，每一位表示该动作维度是否由人控制。
检查器把它打印成 0/1 字符串，按连续相同 mask 分段：全零是 hold，有任一 1 是 intervene。
这里的 hold 是回合内无人接管区间；上一章回合结束后的 EPISODE_HOLD 不写轨迹。
``override_mask`` 表示动作被替换的维度，不一定全是人工控制；找人工区间应看 human_action_mask。

**如果不对**：

* 目录不存在或不是数据集：检查实际 --record 名；目录里应有 ``dataset.json``。
* CSV 已存在：换一个新的导出路径；检查器不覆盖已有证据。
* episode_starts 多于 episode_ends：可能是未完成的回合；检查器不会替你修复或结束它。
* ``tcp_metrics: false``：缺少可定位的单臂 TCP 状态／contract，或是双臂归档；仍可查看 mask 分段。

3. 跑一个有确定答案的离线例子
--------------------------------------------------------------------------------

复用现有测试中的合成轨迹，先熟悉输出格式，再看自己的录制。
这份轨迹的 TCP 与 mask 是测试写入的，不是模拟动力学或真机测量。
需要安装 test extra；从仓库根目录运行一次，输出目录须未使用过。

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :lines: 20-27

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :lines: 28-28

**你应该看到**：实际运行得到 48 条记录（开始、46 个 transition、结束），
21 步全零 mask、25 步人工 mask。完整输出如下；临时验证目录与本机路径会不同。

.. literalinclude:: ../examples/inspect-synthetic-output.txt
   :language: json
   :start-after: # start output
   :end-before: # end output

这个例子的答案是：``episode-0`` 的第 16–40 步，时间戳 1001.6–1004.0 秒，
由人控制六个机械臂维度，夹爪不属于人工控制；mask 为 ``1111110``。
这里 step_index 从 0 开始，区间含两端，共 25 步。
两端采样时间差为 2.4 秒，不能把它当作 25 个控制周期的总时长。

**如果不对**：

* 找不到 tests 文件：回到仓库根目录，确认检出的测试文件存在。
* 缺少 pytest 或 torch：使用安装了 learning、test 的环境。
* 重跑时导出路径已存在：另选新目录，不把多次生成的数据叠在同一个目录里。

4. 对照人工动作和实测位姿变化
--------------------------------------------------------------------------------

从步骤 CSV 找出第一条人工动作，同时计算该步前后 TCP 的实测变化。

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start rows
   :end-before: # end rows

**你应该看到**：以下为该命令的实际输出。

.. literalinclude:: ../examples/inspect-rows-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

``action`` 是实际送给环境的动作，``policy_action`` 是被接管前的候选动作；
录制器无人接管时使用保持策略，并非一个训练好的策略。
该测试故意把 action 写成全零，TCP 却按预设路径变化，所以只能验证检查器的读数和分段，
不能据此推断动作造成了这段运动。

真实录制中，把 action 按同回合的 contract 解码，再比较 state 与 next_state。
按前面的 preset，current 语义下前三维乘 0.05 m，旋转三维乘 0.3 rad；这是环境请求，
机械臂实际运动还受控制器、边界及跟踪误差影响。
教程 task 的 state_fields 从 tcp_pose 开始，TCP xyz 在 state 的前 3 项；
这个合成例子先放七维 joint_pos，故脚本取第 7–9 项。迁移脚本时先查 contract，别照抄切片。
四元数是 xyzw，姿态变化应比较相对旋转，不能直接把四元数差当作角度。

对应的 measurements.csv 实际内容如下：

.. literalinclude:: ../examples/inspect-measurements.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

``max_drift_mm`` 是区间内相对首个观测的最大距离；residual_mm 是区间首尾距离。
本例首次接管 jump_mm 为 20 mm，比较前一段最后一个 obs 与接管第一步的 next_obs。
停手时间 1002.4 秒是测试指定的，得到 settle_s=0.8、停手后残余位移 30 mm。
真实归档没有保存遥操作目标；不能自动从 action=0 推断操作者何时停手。
timestamp_s 在 step 后记录，而 xyz 取 step 前的 obs，绝对时间存在约一个周期的偏移。

``--stop-time-s`` 使用录制机器的 Unix 墙钟秒数，可重复指定多个停手时刻。
默认 ``--settle-speed-mm-s 2.0``、``--settle-hold-s 0.5``：速度须连续低于阈值达指定时长；
``--drift-window-s`` 默认 10 秒，本例显式改为 0.85 秒。这三个参数必须大于零。
未指定停手时刻时退出码为 0；指定后全部稳定为 0、有未稳定项为 3，
任一时刻缺少覆盖它的 TCP 回合为 4（优先于 3）。全局 ``--json`` 输出另有
``status`` 字段，分别为 REPORT、PASS、FAIL、BLOCKED；普通文本 JSON 汇总不带这个字段。

**如果不对**：

* mask 全零：确认回合内确实接管；不要把回合外保持时间当成人工操作。
* 数值尺度差很多：核对单位、state_fields、动作模式和 current／serl 语义。
* stop time 不在有 TCP 的回合内：检查时间来源；仅看区间时可省略 --stop-time-s。
* 双臂没有 TCP 指标：仍按左右动作块逐维检查 mask；不要套用这里的单臂切片。

5. 检查训练数据和动作约定
--------------------------------------------------------------------------------

训练直接读数据集目录，用的是 kept 回合里的人工步骤。下面用和训练同一个函数取出这些行，
并打印数据集头里的动作约定，确认筛选后的步骤和训练时需要的约定仍在。

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start training-view
   :end-before: # end training-view

**你应该看到**：transitions 是保留的训练行数，不是数据集的总步数。
每行包含 obs、next_obs、action、reward、done，也保留 episode_id、step_index、
human_action_mask 等字段。缺口前最后一步会标为 training_segment_end，避免训练时跨缺口接轨迹。
``frame_stack=1`` 时图像不叠帧；训练时图像历史会按训练运行的 ``obs.frame_stack`` 重建。
动作约定（原先的 ``.contract.json``）现在是 ``dataset.json`` 里的 ``action_contract``，
``obs_spec`` 是每个观测键的单帧形状。核对 action_mode、arm_action_dim、gripper_in_action、state_fields、
experiment 中的观测键与形状，以及动作尺度和 experiment.sources.teleop_control；与以后训练、部署保持一致。
按前面的配置应有 state 和 rgb_wrist，动作七维；GELLO 关节扩展则是八维。

``experiment.sources.teleop_control`` 中，默认笛卡尔 tracker 标识为 ``cartesian_tracker_v1``，
还记录平移／旋转领先上限和目标速度；SERL 路径标识为 ``spacemouse_serl_v1``。
这些标识区分控制规则，不是物理输入设备型号。
当前 SpaceMouse 还记录 ``axis_lock`` （默认 dominant），GELLO 的 tracker 块没有此项，
所以即使速度和领先上限相同，两者的默认控制约定也不同。须比较完整控制块及其余动作、
观测约定，不能仅凭相同版本标识就合并数据。

保留条件是成功回合且有人工步骤；默认剔除 teleop_syncing 的对齐步骤。
同一成功回合中的无人操作段也不进入训练行。成功标注发生在交还之后时，
最后人工行不一定自带成功事件；要同时查回合 ``meta.json`` 的 success 和 events。
失败、重录、超时和中断仍在数据集里，标为 discarded 并带原因（``rwrl dataset list`` 可见）。
不要因训练行里没有失败，就认为现场没有失败尝试。

**如果不对**：

* 没有训练行：上一章没有保留成功人工示范；先用 ``rwrl dataset list`` 看 status 与原因，再看 mask。
* 训练行少于数据集总步数：按成功、人工控制、syncing 条件逐层解释，通常是正常筛选。
* 动作约定不匹配任务：选回同一会话的配置与数据，不只按文件名或动作维数猜测。

没有自己的数据集时，可接着前面的合成例子，读取它写出的数据集：

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect-offline.sh
   :language: bash
   :start-after: # start training-view
   :end-before: # end training-view

**你应该看到**：以下实际输出说明数据集有一个 kept 回合，训练行 25 条（即那 25 个人工步），
最后一个人工步 ``step_index=40`` 标为 segment end；动作约定是六维笛卡尔增量加夹爪。
这份合成数据的数值是测试写入的，不能抄到机器人配置。

.. literalinclude:: ../examples/inspect-offline-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

**如果不对**：缺少测试依赖时回到安装章补齐 test extra。

.. note::

   **本章哪些内容需要真机确认**

   本章的合成数据集和输出已离线执行。真实人工接管时刻、实际发送动作与运动的对应、
   相机内容、位姿反馈、停手时刻和停止效果，需要用真机归档核对。
   mask 能说明控制来源，不能单独证明机器人按请求执行了动作。

检查清单：能说出 episode_id、起止 step_index／时间戳和 mask；能找到 action 与前后状态；
能解释训练行少了哪些步骤，以及失败尝试保存在何处（数据集里，标为 discarded）。

工作站与录制这一部分完成。接下来进入 :doc:`bc-data`，准备训练数据。
