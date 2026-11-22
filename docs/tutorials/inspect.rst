检查录制结果
================================================================================

本章接着 :doc:`record` 的 ``local/first-demo`` 输出，全部操作离线，不连接机器人。
先打开完整归档，再检查训练数据；目标是说清楚“哪一回合的哪几步由我控制，控制了哪些维度”。

1. 打开完整归档
--------------------------------------------------------------------------------

用检查器列出回合、人工标记和实测位移，并导出可逐行核对的 CSV。

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start inspect
   :end-before: # end inspect

**你应该看到**：JSON 汇总中的 records、episode_starts、episode_ends、transitions、
human_mask_counts、outcomes 和 episodes。steps.csv 每行对应一个环境步骤，
measurements.csv 每行对应一个区间指标。源 SQLite 以只读方式打开。

``human_action_mask`` 是与动作等长的布尔标记，每一位表示该动作维度是否由人控制。
检查器把它打印成 0/1 字符串，按连续相同 mask 分段：全零是 hold，有任一 1 是 intervene。
这里的 hold 是回合内无人接管区间；上一章回合结束后的 EPISODE_HOLD 不写轨迹。
``override_mask`` 表示动作被替换的维度，不一定全是人工控制；找人工区间应看 human_action_mask。

**如果不对**：

* 文件不存在：检查实际 --record 主名，归档后缀是 .trajectory.sqlite3。
* CSV 已存在：换一个新的导出路径；检查器不覆盖已有证据。
* episode_starts 多于 episode_ends：可能是未完成的回合；检查器不会替你修复或结束它。
* ``tcp_metrics: false``：缺少可定位的单臂 TCP 状态／contract，或是双臂归档；仍可查看 mask 分段。

2. 跑一个有确定答案的离线例子
--------------------------------------------------------------------------------

复用现有测试中的合成轨迹，先熟悉输出格式，再看自己的录制。
这份轨迹的 TCP 与 mask 是测试写入的，不是模拟动力学或真机测量。
需要安装 test extra；从仓库根目录运行一次，输出目录须未使用过。

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :lines: 5-13

.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :lines: 14-14

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
* 重跑时导出路径已存在：另选新目录，不把多次生成的归档叠在同一个文件上。

3. 对照人工动作和实测位姿变化
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

4. 检查训练 pkl 和 contract
--------------------------------------------------------------------------------

打开自己刚录下的 pkl，确认筛选后的步骤和训练时需要的约定仍在。
只加载自己生成或可信的 pickle 文件。

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect.sh
   :language: bash
   :start-after: # start pkl
   :end-before: # end pkl

**你应该看到**：transitions 是保留的训练行数，不是完整归档的步数。
每行包含 obs、next_obs、action、reward、done，也保留 episode_id、step_index、
human_action_mask 等归档字段。缺口前最后一步会标为 training_segment_end，避免训练时跨缺口接轨迹。
contract 中核对 action_mode、arm_action_dim、gripper_in_action、state_fields、
experiment 中的观测键与形状，以及动作尺度和 experiment.sources.teleop_control；与以后训练、部署保持一致。
按前面的配置应有 state 和 rgb_wrist，动作七维；GELLO 关节扩展则是八维。

``experiment.sources.teleop_control`` 中，默认笛卡尔 tracker 标识为 ``cartesian_tracker_v1``，
还记录平移／旋转领先上限和目标速度；SERL 路径标识为 ``spacemouse_serl_v1``。
这些标识区分控制规则，不是物理输入设备型号。
当前 SpaceMouse 还记录 ``axis_lock`` （默认 dominant），GELLO 的 tracker 块没有此项，
所以即使速度和领先上限相同，两者的默认控制约定也不同。须比较完整控制块及其余动作、
观测约定，不能仅凭相同版本标识就合并数据。

保留条件是成功回合且有人工步骤；默认剔除 teleop_syncing 的对齐步骤。
同一成功回合中的无人操作段也不进入 pkl。成功标注发生在交还之后时，
最后人工行不一定自带成功事件；要同时查归档的 episode_end.outcome。
失败、重录、超时和中断仍在完整 SQLite，outcomes 提供对应结果。
不要因 pkl 没有失败行，就认为现场没有失败尝试。

**如果不对**：

* pkl 不存在：上一章没有保留成功人工示范；先检查 SQLite 的 outcomes 与 mask。
* pkl 行数小于归档：按成功、人工控制、syncing 条件逐层解释，通常是正常筛选。
* contract 不匹配任务：选回同一会话的配置与数据，不只按文件名或动作维数猜测。

没有自己的 pkl 时，可接着前面的合成例子，运行现有录制器测试的 fake 环境：

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect-offline.sh
   :language: bash
   :start-after: # start record
   :end-before: # end record

然后读取它生成的文件：

.. tutorial-test: skip
.. literalinclude:: ../examples/inspect-offline.sh
   :language: bash
   :start-after: # start pkl
   :end-before: # end pkl

**你应该看到**：以下实际输出说明 pkl 保留了一行及其 mask，contract 是八维 joint_abs。
这个测试专门使用关节 fake 环境，不是前面那份笛卡尔轨迹的训练导出。
限位 ±2、joint_scale_rad=0.05 都是测试数据，不能抄到机器人配置。

.. literalinclude:: ../examples/inspect-offline-pkl-output.txt
   :language: text
   :start-after: # start output
   :end-before: # end output

**如果不对**：demos.pkl 已存在时另选新的离线目录；缺少测试依赖时回到安装章补齐 test extra。

.. note::

   **本章哪些内容需要真机确认**

   本章的合成归档和输出已离线执行。真实人工接管时刻、实际发送动作与运动的对应、
   相机内容、位姿反馈、停手时刻和停止效果，需要用真机归档核对。
   mask 能说明控制来源，不能单独证明机器人按请求执行了动作。

检查清单：能说出 episode_id、起止 step_index／时间戳和 mask；能找到 action 与前后状态；
能解释 pkl 少了哪些步骤，以及失败尝试保存在何处。

工作站与录制这一部分完成。接下来进入 :doc:`bc-data`，准备训练数据。
