路线一 2B. 使用 GELLO
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
     - 来自从臂 ``joint_limits`` 每行上下限的中点；先按当前机器人型号的规格填 station.joint_limits，再运行标定。
   * - ``intervention``、``event_device``、``event_keys``
     - 保留校准默认 always；本例活动回合使用终端 t/h，不需要 evdev toggle。

其余字段保持生成值：``name``、``backend``；未设置的 ``hold_s``、``poll_interval_s``
默认 0，设备层 ``gripper_binarize_threshold`` 默认 null。需要设备层二值化时才调整后者；
本章 leader_cartesian 已默认用 0.5 分开闭。

**如果不对**：

* ``moved less than 0.1 rad``：这次保留旧符号，重新做足够幅度的单关节检查。
* 残差超过 pi/8：检查姿态和符号，再标定；不要直接开始跟随。
* ``gripper trigger moved only ...``：检查触发器 ID，开闭两端至少相差 0.05 rad。
* ``no joint_limits given``：上一章模板没有 joint_limits，零 wrap center 对部分 Panda 关节不合适。
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
* 跨 pi 后读数跳变：核对 wrap_centers_rad 与实际 joint_limits。
* 开闭方向相反：重新采集 gripper_open_rad 和 gripper_closed_rad，不靠交换标签继续。

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
默认 always 模式没有实体切换；中立门要求主臂请求位姿与从臂实测位姿的领先量在阈值内。
看到 START_NEUTRAL 通过后按 t，移动主臂，试一次触发器开闭，再按 h。
最后按 s 标记成功、q 退出保持阶段。下一章解释标注。

**你应该看到**：从臂跟随主臂末端相对变化，交还后保持；
接管时若触发器与上次夹爪状态不同，先把触发器移到该状态，再控制开闭。
会话报告的 ``resolved-config.yaml`` 写着 ``teleop_device: leader_cartesian`` 和
``action_mode: cartesian_delta``；训练动作是六维笛卡尔增量加夹爪。

**如果不对**：

* 一直停在 START_NEUTRAL：always 模式将主臂与从臂对齐并稳定保持；若另用 toggle 模式，则关闭设备 toggle。
* Space 能切设备状态却不接管活动回合：当前会话使用终端事件源，按 t/h；不要混用两套接管状态。
* ``WORKSPACE HOLD``：先交还，检查请求方向和边界，再接管。不要通过扩大工作区掩盖错误映射。

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
     - arm 要求接管时对齐；auto 用于关节环境，当前 session hold 只支持笛卡尔环境。

其余保留复制配置；``teleop_arm_tolerance_rad`` 默认 0.1 rad，只有重新审阅对齐容差后才改。
此时每臂七个关节增量加夹爪，共八维；别与前面的七维数据混用。

.. warning::

   这个关节扩展结束回合后会自动 reset，不再等待 n。先确认主从关节范围和归位路径。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start joint
   :end-before: # end joint

**你应该看到**：新归档 contract 的 action_mode 为 joint_delta，arm_action_dim 为 7。
若选择 joint_abs 环境，录制器还会在回合开始检查主从最大差值（默认 0.8 rad）。

**如果不对**：动作维数或关节限位不匹配时停止运行，核对主从关节顺序和对应范围；
报 ``leader arm too far`` 时先对齐，不能仅扩大 start_tolerance_rad。

扩展：双臂 GELLO
--------------------------------------------------------------------------------

双臂使用独立的 preset，让 left／right 各自对应一台从臂和一套主臂。

.. warning::

   两臂 reset 都会运动。确认两臂及工具之间的间距；先验证左侧单独接管，再验证右侧。
   此 preset 采用 auto 回合行为，结束后可能自动复位。

.. literalinclude:: ../examples/gello.sh
   :language: bash
   :start-after: # start bimanual
   :end-before: # end bimanual

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
