用 GELLO 遥操作
================================================================================

本章沿用 :doc:`station` 的单臂 Franka 配置。在 GELLO 串口所在的录制机器执行。
主臂是手里移动的 GELLO，从臂是实际工作的 Franka。
本章的 ``local/task.yaml`` 输出七维笛卡尔增量动作，不是七个关节角。

.. tip::

   本章命令省略了 ``--station`` 和 ``--preset``，用的是 :doc:`station` 第 10 步 ``rwrl use`` 记住的两份文件。
   还没做这一步时，先运行 ``rwrl use preset local/task.yaml``。要临时换文件，在命令里显式写出参数即可。

1. 找串口、关节 ID 和事件设备
--------------------------------------------------------------------------------

先把每个主臂关节与总线上的 ID 对上，再开始标定。

.. tutorial-test: skip
.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start serial
   :end-before: # end serial

使用 ``/dev/serial/by-id/`` 稳定路径，沿符号链接核对实际串口节点可读写。
用 Dynamixel Wizard 扫描串口、波特率和 ID，按主臂从基座到末端的关节顺序记下 ID；
夹爪触发器的 ID 单独记录。仓库的 calibrate 不扫描 ID，也不会改 EEPROM 波特率。
例如扫描得到关节 1–7、触发器 8、57600 baud，才使用下节这组命令参数。

**你应该看到**：串口路径、七个按机械关节顺序排列的不同 ID、独立的 gripper ID，
安装章需已补齐 ``gello``；只有使用 evdev 键盘／踏板时才需要节点清单与 ``gello-events``。

**如果不对**：

* doctor 的 ``franka.os:serial`` 失败：检查 U2D2／OpenRB-150 的 USB 连接，重新枚举。
* 节点属于 dialout 而用户不在组内：执行下列命令，重新登录后再查 id。
* 能开串口但读不到伺服：核对总线供电、实际 ID 和波特率；别把示例 ID 当成扫描结果。
* ``franka.os:groups`` 失败或 evdev 打不开：检查所选 ``/dev/input`` 节点读权限；串口的 dialout 权限不覆盖它。

.. tutorial-test: skip
.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start dialout
   :end-before: # end dialout

2. 标定方向、偏移和夹爪
--------------------------------------------------------------------------------

标定把伺服原始角度转成与从臂一致的关节角，并测出夹爪触发器的两端。
若 station 曾配置 SpaceMouse，先移除其 teleop 块，避免 ``--use`` 拒绝替换其他后端。

.. warning::

   本步释放主臂伺服力矩并手动移动主臂。支撑好主臂，停止所有跟随程序。
   不要同时运行从臂录制；标定姿态只要求摆放主臂，不会命令从臂运动。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start calibrate
   :end-before: # end calibrate

用自己的串口、ID 和波特率替换示例。按提示逐关节向从臂定义的正方向转动，
确认建议的 joint_signs，再把主臂摆到提示的标定姿态。
七关节默认姿态为 ``[0, 0, 0, -pi/2, 0, pi/2, 0]``；
如果机构的已知姿态不同，通过 ``--calibration-pose`` 传入对应的七个弧度数。
最后分别完全打开、闭合触发器并回车。

需要覆盖标定输入时，``--joint-signs`` 接收按关节顺序排列的符号；
未指定时取 station 的符号，再没有才全为 +1。
``--joint-limits-yaml`` 从另一份机器人 YAML 的 ``limits.joints`` 计算 wrap center，
未指定时使用当前 station。``--skip-sign-check`` 跳过逐关节方向检查、直接使用初始符号；
只有这些符号已在本机验证过时才使用。

**你应该看到**：``offset residuals (rad):`` 后有七个残差，输出 teleop YAML，
确认后保存到 station。计算关系是 ``q = sign * (raw - offset)``，随后按 wrap center 包裹角度。
偏移在四分之一圈网格上选择；残差不是允许随意忽略的安装误差。

校准后，在生成的 teleop 块中补入以下选项，保留全部实测标定字段：

.. literalinclude:: ../examples/gello-config.txt
   :language: yaml
   :start-after: # start events
   :end-before: # end events

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``options.port``、``baudrate``、``joint_ids``、``gripper_id``
     - 用上一步串口和 Wizard 清单；例：触发器 ID 8 对应 ``gripper_id: 8``。
   * - ``joint_signs``、``joint_offsets_rad``
     - 使用刚才逐关节检查及标定姿态算出的输出，不手抄通用零值。
   * - ``gripper_open_rad``、``gripper_closed_rad``
     - 使用触发器完全开／闭时标定输出的原始弧度值。
   * - ``wrap_centers_rad``
     - 来自从臂 ``limits.joints`` 每行上下限的中点；先按当前机器人型号的规格填 station 的 ``limits.joints``，再运行标定。
   * - ``intervention``、``event_device``、``event_keys``
     - 新配置默认 always；本例设为 always，活动回合使用终端 t/h，不需要 evdev toggle。
       对已有 GELLO 块执行 ``--use`` 会保留原有 intervention、event_device、event_keys，需核对后修改。

其余字段保持生成值：``name``、``backend``；未设置的 ``hold_s``、``poll_interval_s``
默认 0，设备层 ``gripper_binarize_threshold`` 默认 null。需要设备层二值化时才调整后者；
本章 leader_cartesian 已默认用 0.5 分开闭。

**如果不对**：

* ``moved less than 0.1 rad``：这次保留旧符号，重新做足够幅度的单关节检查。
* 残差超过 pi/8：检查姿态和符号，再标定；不要直接开始跟随。
* ``gripper trigger moved only ...``：检查触发器 ID，开闭两端至少相差 0.05 rad。
* ``no limits.joints given``：上一章模板没有 ``limits.joints``，零 wrap center 对部分 Panda 关节不合适。
  按当前型号规格补上七组限位后重做标定，尤其核对跨 pi 的关节。

3. 对齐主从，先读后动
--------------------------------------------------------------------------------

先比较关节读数与夹爪状态，避免在错误映射上练习运动。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start read
   :end-before: # end read

**你应该看到**：read 的 ``q`` 是标定后的主臂角度；另一条命令的 ``q (rad)`` 是从臂角度。
在从臂已安全停止的位置手动对齐主臂，逐项比较。触发器完全打开对应 gripper=1，关闭对应 0。
默认 always 模式的 intervening 始终为 True；若另配 evdev toggle 模式，Space 才切换设备层状态。
直接连接时分别读主从即可；``teleop read --with-follower`` 只支持 daemon 路径。

**如果不对**：

* 某一项反向或相差约 pi/2：回查 joint_ids 顺序、sign、offset，不用反向扳主臂补偿。
* 跨 pi 后读数跳变：核对 wrap_centers_rad 与实际 ``limits.joints``。
* 开闭方向相反：重新采集 gripper_open_rad 和 gripper_closed_rad，不靠交换标签继续。

read 默认每秒打印 3 次；省略 ``--duration-s`` 就一直读到 Ctrl-C。
它不消费终端 t/h，设备层 intervening 不能用这两个键切换。
station 声明了 daemon、但 daemon 已停止时，把全局 ``--direct`` 放在 teleop 前：

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start read-routes
   :end-before: # end read-routes

**你应该看到**：daemon 路径多出 ``follower_q`` 和逐关节 ``delta``，
后者是主臂减从臂的弧度差。只移动主臂，在几个姿态逐项比较方向、顺序和差值；
这条读取命令不发送从臂运动。直连只打开输入，不打开从臂、夹爪或相机。

**如果不对**：daemon 不可达时先恢复服务，或去掉 --with-follower 用直连检查输入；
不要同时让 daemon 和直连进程占用同一设备。

4. 接管、操作和交还
--------------------------------------------------------------------------------

用上一章的笛卡尔 preset 做一次短练习，熟悉主臂位移如何变成从臂动作。
默认 ``anchored`` 映射在接管时记住主从位置，随后应用主臂末端的相对位移和旋转。
它用主臂正运动学，所以仍需核对机构、关节顺序和工具坐标约定。

.. warning::

   下面会 reset、归位并开爪。确认 home、路径、工作区和停止方式，留人监护。
   保持主臂稳定，先做很小的单方向动作。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start practice
   :end-before: # end practice

当前 ``teleop run`` 的活动回合由终端 ``t`` 接管、``h`` 交还。
默认 always 模式没有实体切换。reset 后会话停在 READY，按空格开始回合（ACTIVE）；
看到 ACTIVE 后按 t，移动主臂，试一次触发器开闭，再按 h。
最后按 s 标记成功，回合进入 POST_EPISODE，按 q 退出。一个人在主臂和终端之间切换即可，两人分工的完整流程见 :doc:`record`。下一章解释标注。

**你应该看到**：从臂跟随主臂末端相对变化，交还后保持；
接管时若触发器与上次夹爪状态不同，先把触发器移到该状态，再控制开闭。
会话报告的 ``resolved-config.yaml`` 写着 ``teleop_device: leader_cartesian`` 和
``action_mode: cartesian_delta``；训练动作是六维笛卡尔增量加夹爪。

**如果不对**：

* 一直停在 READY：按空格才开始回合，要在运行 ``rwrl`` 的终端里按。
* Space 能切设备状态却不接管活动回合：当前会话使用终端事件源，按 t/h；不要混用两套接管状态。
* 从臂在盒子边缘不再前进、只沿墙面滑动：目标被 station 的 ``limits.tcp`` 裁进盒内，不是故障。先交还，检查请求方向和盒子，再接管；不要通过扩大盒子掩盖错误映射。
* ``measured TCP outside TCP box``：实测 TCP 超出盒子加 ``measured_margin_m``，环境已停臂并锁存 ``SafetyViolation``；检查反馈和机器人状态，人工确认后再重新开始。

需要核对映射时，记接管瞬间的主臂末端为 ``(p_l0, R_l0)``，从臂命令为 ``(p_c, R_c)``。
随后请求位置是 ``p* = p_c + (p_l - p_l0)``，旋转是 ``R* = (R_l R_l0^T) R_c``；
这里的主臂末端由正运动学求得，tracker 再限制请求速度和领先量。
``teleop run`` 把 station 的 task_frame 合入环境；限位随机器人带来，环境在首次 reset 时继承 station 的 ``limits``。
源的显式选项优先于环境配置，环境未设置时才回退到本地机器人 YAML；
TCP 盒（轴取自 ``task_frame``）把笛卡尔目标裁进盒内，机器人关节范围同时限制关节目标。

5. 需要关节跟随时怎样切换
--------------------------------------------------------------------------------

只有希望从臂逐关节跟随主臂时，才另建关节动作配置和输出文件。
把本次 session 目录的 resolved-config.yaml 复制为 ``local/gello-joint.yaml``，
保留里面已合入的机器人连接和 leader-options 路径，修改这些字段。
扩展使用 ``python -m real_world_rl.demos``，它也是 ``rwrl record`` 调用的录制入口：

.. literalinclude:: ../examples/gello-config.txt
   :language: yaml
   :start-after: # start joint
   :end-before: # end joint

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``teleop_device``、``robot_teleop_action_mode``
     - 主臂读绝对关节角，选择 leader_arm、joint_abs。
   * - ``real_robot.action_mode``、``joint_scale_rad``
     - 本扩展按仓库双臂烟测 preset 使用 joint_delta、0.01 rad；现场确认该步长。
   * - ``teleop_onset``、``episode_end_behavior``
     - arm 要求接管时对齐；本例用 auto：回合结束后自动 reset。hold 只支持笛卡尔环境；关节环境可改用 confirm（要事件设备，如 ``--teleop-event-device stdin``），得到\ :doc:`record`\ 的两人流程，保持用实测关节，需要现场确认。

其余保留复制配置；``teleop_arm_tolerance_rad`` 默认 0.1 rad，只有重新审阅对齐容差后才改。
此时每臂七个关节增量加夹爪，共八维；别与前面的七维数据混用。

若改成 ``real_robot.action_mode: joint_abs``，还必须在 real_robot 中填写
按本机核验过的七组 ``limits.joints``（``real_robot.limits.joints``，只能比 station 的范围更窄）；绝对动作按这些上下限映射。
机器人配置也要提供 ``limits.max_delta_per_step.joint_rad``，远程连接时在 daemon 所用 station 中设置。
保留 ``state_fields`` 中的 ``joint_pos``，核对 remote、token、相机与 home/reset，不能只改动作模式。
源选项文件含多个机器人时，用 ``--teleop-robot NAME`` 选择对应机器人。

**让从臂更平滑地跟随。** station 默认的关节控制器 ``impedance`` 在每个 ``control_hz`` 周期收到一个阶跃目标。
关节跟随可以改用 ``joint_controller: trajectory_impedance``：每个目标经规划器变成连续轨迹，再用 GELLO 的关节阻抗律跟踪。
它写在\ **机器人端的 station**\ 里，需要一起构建的 franky、``serl_controller`` 和 ``joint_controller``，建议实时内核；
选择、安装和 doctor 检查按 :doc:`controller` 第 10 步做，不用重新标定。同时把配置里 ``real_robot.control_hz`` 提高到 15–30 Hz：

.. literalinclude:: ../examples/controller-joint.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

.. code-block:: yaml

   real_robot:
     control_hz: 20     # 默认 10；GELLO 关节采集可提到 15 到 30

``control_hz`` 提高后，``joint_scale_rad`` 和 ``max_delta_per_step.joint_rad`` 按\ **每步**\ 计的速度也随之提高，重新核对它们。
采集、训练和部署必须用同一个 ``control_hz``，所以这一步要在开始采集\ **之前**\ 定下来。
主臂移动时从臂会落后约一个刹车距离（默认约 0.06 rad），用 ``joint_trajectory.max_acceleration`` 调整，见 :doc:`../reference/joint-controller`。

``--teleop-onset arm`` 等到最大关节差不超过 ``--teleop-arm-tolerance-rad`` 才接管；
默认容差 0.1 rad，对齐前不算人工步骤，reset 后需重新对齐。
默认起始策略 ``ramp`` 则立即接管：joint_abs 目标相对从臂每步最多前进
``--teleop-ramp-step-rad`` （默认 0.05 rad），受限步骤标为 teleop_syncing。
joint_delta 已按 joint_scale_rad 限幅，初次追赶标记持续到主臂首次进入一个步长以内。

数据集保留这些执行动作；录制器默认从训练行排除同步步，
``--keep-syncing-steps`` 可保留，但数据集派生的训练视图仍排除同步监督。
关节路径没有笛卡尔夹爪接管锁存，接管前先让触发器匹配当前夹爪状态。
录制产物与恢复方式见 :doc:`record`；各动作模式使用独立数据集。

.. warning::

   这个关节扩展结束回合后会自动 reset，不再等待 n。先确认主从关节范围和归位路径。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start joint
   :end-before: # end joint

**你应该看到**：新数据集头 action_contract 的 action_mode 为 joint_delta，arm_action_dim 为 7。
若选择 joint_abs 环境，录制器还会在回合开始检查主从最大差值（默认 0.8 rad）；改用 confirm 后，这一项变成 READY 里的起始锁：差值超过容差时空格不起作用，终端列出差得多的关节，对齐后才能开始，不会报错退出。

**如果不对**：动作维数或关节限位不匹配时停止运行，核对主从关节顺序和对应范围；
报 ``leader arm too far`` 时先对齐，不能仅扩大 start_tolerance_rad。

6. 用第二台机械臂作笛卡尔主臂
--------------------------------------------------------------------------------

这条扩展用 ``arm_leader`` 只读取另一台机械臂的关节角，不向主臂发运动命令。
先在主臂主机复制配置，再改成实际主臂地址和已安装的后端：

.. tutorial-test: skip
.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start second-copy
   :end-before: # end second-copy

.. literalinclude:: ../examples/gello-config.txt
   :language: yaml
   :start-after: # start second-station
   :end-before: # end second-station

外层 fake 臂只作载体，真实主臂只由 teleop 打开，避免重复连接。
``arm.type`` 选择后端，其余 arm 字段传给该后端构造器。
``gripper`` 默认 false；没有主臂夹爪读数时，从臂保留末次夹爪命令。
``intervention: always`` 提供持续激活读数，实际接管由从臂录制终端 t/h 决定。

.. warning::

   连接不会启用手动引导。按主臂厂商流程安排手动操作，确认主从两侧停止方式。
   后面的录制会使从臂 reset、归位并操作夹爪；两侧都要有人确认运动范围。

在主臂主机激活带机械臂 SDK 与 daemon 依赖的环境，启动服务：

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start second-daemon
   :end-before: # end second-daemon

**你应该看到**：服务监听 50052，提供主臂读数；它不控制从臂。

**如果不对**：在主臂主机运行 ``rwrl doctor --station local/leader.yaml``，
核对真实主臂连接与 SDK，不能用外层 fake 臂正常来证明真实主臂可读。

在从臂录制机器把以下内容保存为 ``local/leader-cartesian.yaml``，
将 leader-host 换成可达主机；服务设置令牌时补上匹配的 remote_token。

.. literalinclude:: ../examples/gello-config.txt
   :language: yaml
   :start-after: # start second-source
   :end-before: # end second-source

主臂路径只能选一种：remote_address、指向本地载体配置的 robot_yaml，
或进程内 leader 部件／解析器。远程路径在源启动时连接。
内置运动学模型只有默认的 panda，要求主臂报告恰好七个关节的 joint_abs；
其他机构需注册匹配模型，kinematics 不会随 arm.type 自动改变。

按 :doc:`record` 第 6 步准备 ``local/record-cartesian.yaml``，保留已核验的从臂配置。
普通录制器不自动合并 station 的 task_frame；确认它已在 real_robot
或其本地 robot 配置中，限位（``limits``）则在 robot 配置里或随 daemon 的 station 带来。若预设直连从臂，先停止占用从臂的 daemon，主臂 daemon 保持运行。
在从臂机器的学习环境启动：

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start second-record
   :end-before: # end second-record

**你应该看到**：从臂 reset 后保持，t 接管时锚定，h 交还。
保存示范前逐项验证主臂 FK 与实际 TCP、平移和旋转方向、夹爪、盒子边缘的滑动及停止效果。
这些是现场验收目标，尚不能视为第二台实体主臂的硬件验证结果。

**如果不对**：关节数或模型不符时先修正运动学；方向或停止不符合预期时停止录制，
不要扩大边界或跳过检查继续采集。

扩展：双臂 GELLO
--------------------------------------------------------------------------------

双臂使用独立的 preset，让 left／right 各自对应一台从臂和一套主臂。

.. warning::

   两臂 reset 都会运动。确认两臂及工具之间的间距；先验证左侧单独接管，再验证右侧。
   此 preset 采用 auto 回合行为，结束后可能自动复位。

.. tutorial-test: skip
.. literalinclude:: ../examples/gello.sh
   :language: bash
   :lines: 27-27

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :lines: 28-29

先只执行复制行。检查复制文件的 ``arm_names: [left, right]`` 与两个 remote：
示例 left 为本机 50051，right 为本机 50052；分别启动已配置、已标定的 station daemon 后才运行录制。
每个 daemon 的 teleop.options.port、ID、标定和事件节点必须对应那一侧。
主臂通过服务读取时 remote_address 指服务 HOST:PORT，remote_token 与服务一致；
本地读取主臂则用 robot_yaml 指向含 teleop 块的配置。

该 preset 使用 ``bimanual`` 加 ``leader_arm``，两侧都是 joint_delta、0.01 rad，
没有统一的终端事件源。每侧在自己的 GELLO event_keys.toggle 绑定独立键，
例如左 KEY_F1、右 KEY_F2；intervention 都设 toggle，避免一个按键同时触发两侧。

**你应该看到**：左接管时右侧保持，右接管时左侧保持。
动作布局是左七关节、右七关节、左夹爪、右夹爪；仅左侧接管的 mask
示例为 ``1111111000000010``。两侧的成功／失败事件合并为整个回合的标注。

**如果不对**：左右反了就核对 arm_names、remote 和物理串口映射；
一键控制两侧就核对 event_device 与按键绑定。双臂归档没有单臂 TCP 指标，下一章仍可逐维查看 mask。

.. note::

   **本章哪些内容需要真机确认**

   串口权限、供电、ID、波特率、正方向、偏移残差、关节包裹中心、触发器两端、
   主从对齐与运动学、实体键状态、中立检查、夹爪接管、关节扩展及双臂独立接管需现场确认。
   always 模式已通过无硬件测试验证中立检查逻辑；现场仍需验证对齐阈值与运动安全。

检查清单：能逐项对上主从读数；标定参数来自实测；中立门通过；能 t 接管、h 交还；
知道当前输出是笛卡尔还是关节动作。

接下来进入共同流程 :doc:`record`。
