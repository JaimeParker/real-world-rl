路线一 1. 建立工作站
================================================================================

你会接通一台 Franka、Franka Hand 和一台 RealSense，再写好第一份实验配置。
先完成 :doc:`install`。本章命令都在 NUC 的仓库根目录、已激活的环境中运行。

``station`` 是一份 YAML，写这台机器接了哪些设备、怎么连。
``preset`` 是另一份 YAML，写这次实验给策略什么观测、策略输出什么动作。
本章直接连接设备。``daemon`` 是让其他程序远程使用设备的服务，这次不用启动。

1. 复制模板
--------------------------------------------------------------------------------

先复制两份模板，后面只改本地副本。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start copy
   :end-before: # end copy

模板来自仓库的 ``robot.franka.example.yaml``、``gello_cartesian.yaml`` 和
``hil_single_arm.yaml``，已经组合成单臂、单相机的例子。
下文“保持默认”指保留模板里的值。
先改 ``local/station.yaml``；``local/task.yaml`` 留到设备检查完成后再改。

**你应该看到**：``local/`` 下有 ``station.yaml`` 和 ``task.yaml``。
前者的 ``REPLACE_WITH_...`` 要换成现场值；home 和边界的数字会在后面读出来再填。

**如果不对**：

* 找不到模板：确认终端在仓库根目录。
* ``local/`` 已有自己的配置：先备份或另选文件名，并同步修改后续命令中的路径。

2. 填连接地址和相机序列号
--------------------------------------------------------------------------------

先填能让程序找到设备的两个值：Franka IP 和 RealSense 序列号。

在 ``local/station.yaml`` 找到这一段，修改 ``robot_ip``：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start arm
   :end-before: # end arm

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``robot_ip``
     - 在 NUC 浏览器打开现场 Franka Desk，取当前可访问的 ``https://实际IP/desk/`` 中的 IP。
   * - ``robot_model``
     - 在同一个 Desk 的系统设置里看系统版本：4.x 是 Franka Emika Robot，填 ``panda``；
       5.x 是 Franka Research 3，填 ``fr3``。

其余字段保持默认：``type``、``name``、``arm_backend``、``realtime``、
``relative_dynamics_factor``、``cartesian_controller``、``claim``。
``cartesian_controller: impedance`` 表示由 franky 的阻抗 tracker 执行末端目标，普通内核也能用；
另外两个选项 ``internal`` 和 ``serl`` 以后只改这一行即可切换，见 :doc:`serl-controller`。
``robot_model`` 决定 SERL 关节复位用哪张关节限位表，也是 ``rwrl setup`` 给 ``joint_limits`` 的默认来源。这里使用 ``franky``；需要强制实时内核时才把
``realtime`` 改成 ``enforce``，并先准备 PREEMPT_RT。归位速度需要调整时再审阅
``relative_dynamics_factor``，它控制 ``move_to_*`` 的速度、加速度和 jerk 比例。
``claim: true`` 保留本机设备独占锁。

插入 RealSense，运行发现命令：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start discover
   :end-before: # end discover

把列出的序列号填进相机块：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start camera
   :end-before: # end camera

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``cameras[0].serial``
     - 从刚运行的 ``rwrl discover`` 的 ``realsense`` 行复制。

其余字段保持默认：``name: wrist``、``backend: realsense``、640 × 480、30 fps、
``enable_depth: false``。改变采集模式时先核对相机支持的模式；需要深度时再打开深度。
只有一台相机时也可以留空序列号自动选择，本章填写它，方便以后扩展第二台。

**你应该看到**：发现报告里有相机，格式如下；两个文字占位符都已经替换。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start discover
   :end-before: # end discover

**如果不对**：

* ``cameras: none``：检查 ``realsense`` extra、USB 连接和线缆，再运行发现命令。
* 出现多个序列号：逐台拔插，确认哪个对应 ``wrist``，再填入。
* Desk 打不开：先修复 NUC 到控制柜的网络连接，再继续。

3. 让 doctor 检查设备
--------------------------------------------------------------------------------

现在连接设备并读反馈，确认地址和驱动可用，再去测 home。

``doctor`` 会生成检查报告，每一行是一项检查。先在 Desk 取得控制权、激活 FCI，
停止其他占用设备的程序，再运行：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start doctor
   :end-before: # end doctor

**你应该看到**：下面几项通过，另有 ``franka.feedback:gripper`` 的夹爪反馈。
这是源码格式示例，现场结果以你的报告为准。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

**如果不对**：

* ``franka.os:ping``／``franka.os:tcp`` 失败：核对 IP、NUC 网口和控制柜状态。
* ``franka.handshake`` 失败：检查 FCI、设备占用；若报告 ``libfranka mismatch``，按报告匹配
  franky wheel。``--install-sdk`` 只处理握手确认的版本不匹配。
* ``franka.feedback:arm`` 失败：检查电源、急停和 Desk 状态，修复后重跑。
* ``franka.feedback:camera:wrist`` 报 ``no frames within 2s``：检查 USB 3 带宽，换接口后重连。
  夹爪反馈异常则按后面的夹爪检查核对行程和单位。

4. 读出 home，再填回文件
--------------------------------------------------------------------------------

把机械臂放到下一次实验开始的位置，读出关节角，作为本仓 reset 使用的 home。
``reset`` 是开始新回合时的复位流程；本章配置会归位并打开夹爪。

.. warning::

   接下来会手动引导机械臂。先确认负载、停止方式、周围空间和现场监护。
   测量期间不要运行 ``home move``、录制或 reset。以后执行 reset 前，
   还要确认归位路径和开爪不会碰撞或掉落物体。

按现场手动引导流程放到安全的起始姿态，退出引导后读取：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start arm-state
   :end-before: # end arm-state

**你应该看到**：``status`` 显示 ``robot: franka (franka)`` 和 ``cameras: wrist``；
``q``、``tcp`` 的格式如下。这里的数字只是演示。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start home
   :end-before: # end home

把 ``q (rad):`` 后的七个数原样复制到 ``home_joints``。以上述示例为例，填写结果是：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start home
   :end-before: # end home

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``home_joints``
     - 用本步 ``rwrl --station local/station.yaml q`` 的七个数；同时保存 ``tcp`` 输出供复核。

其余字段保持默认；换起始姿态时重新读数并更新这一行。
保存后，这一行应与你自己的 ``q`` 输出一致。

**如果不对**：

* 读数命令连接失败：确认已退出手动引导，重新检查 Desk 控制权和 FCI。
* ``q`` 不是七个有限数：停止填写，重新运行 doctor 检查机械臂反馈。
* 修改后 YAML 不能加载：保留方括号、逗号与原有缩进，再检查。

5. 测工作区，再检查保存结果
--------------------------------------------------------------------------------

在允许区域的角点读取 TCP，得到本仓限制末端目标所需的边界。

沿用上一步的手动引导流程。先选一个已留出工具和障碍物余量的区域，
逐个引导到八个角点；每到一点，退出引导，运行一次：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start corner
   :end-before: # end corner

**你应该看到**：每次都有一行 TCP 读数。下面演示一个基座系位置盒的八个角点。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start corners
   :end-before: # end corners

从这些读数分别取 x、y、z 的最小值和最大值。
示例得到 ``low = [0.3, -0.3, 0.05]``、``high = [0.7, 0.3, 0.6]``。
现场有测量偏差时向内收紧，并再次检查范围内的工具、机械臂和路径。
角点可达并不保证整个盒子无碰撞。

把结果写入 ``local/station.yaml``：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start bounds
   :end-before: # end bounds

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``workspace.low`` / ``workspace.high``
     - 从本步每次 ``rwrl --station local/station.yaml tcp`` 的 XYZ 取下界／上界，单位 m。
   * - ``safety_box``
     - 把已审阅的 low 和 high 连在一起，顺序为 xmin、ymin、zmin、xmax、ymax、zmax。

其余字段保持默认：``workspace.frame: base`` 和 ``max_delta_per_step``。
后者把从实测状态到 servo 目标的单步变化限制为示例的 0.05 m／0.1 rad；
只有现场核对控制周期和运动后才调整。两个位置范围共同生效，取交集。

保存后检查文件里的值：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start workspace
   :end-before: # end workspace

**你应该看到**：``workspace``、``base_safety_box`` 和 ``home_joints`` 与刚才填写的一致。
下面是输出字段的格式节选。输出还会标明 m、rad、xyzw 单位。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start workspace
   :end-before: # end workspace

**如果不对**：

* low 大于或等于 high：重新按 x、y、z 分别整理下界和上界。
* 输出仍是旧值：保存文件，确认命令使用的是 ``local/station.yaml``。
* 某个角点或路径不安全：缩小、重新选择区域，不要为了凑满盒子强行到达。

6. 读夹爪开口
--------------------------------------------------------------------------------

读一次夹爪反馈，确认软件里的开口与现场一致。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start gripper-state
   :end-before: # end gripper-state

**你应该看到**：宽度、归一化比例、``grasped`` 和反馈来源。格式示例如下：

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start gripper
   :end-before: # end gripper

本章的 Franka Hand 使用模板中的夹爪块：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start gripper
   :end-before: # end gripper

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - 无；核对 ``max_width_m``
     - 用手爪规格和刚才 ``gripper show`` 的宽度核对 0.08 m；实际行程不同才修改。

其余字段保持默认：``backend: franky``、``mode: binary``、``binary_threshold: 0.5``、
``shares_arm_session: false``。Hand 独立连接；二值模式下设备动作大于 0.5 时打开。
后四个键是 HIL-SERL 的夹爪命令：闭合以 ``speed_m_s`` 0.3 m/s、``grasp_force_n`` 130 N 抓到
``close_width_m`` 0.01 m（空抓也算成功），张开到 ``open_width_m`` （``null`` 为完全张开）。
命令在后台执行、不阻塞控制步，连续按键只执行最新一次。省略这些键时取同样的值，
所以旧 station 不改也会用这些值；夹易碎物体时先调小 ``grasp_force_n`` （上限 130）。
任务确实需要连续宽度时才改成 ``continuous``，并一起核对动作约定。

**如果不对**：

* ``robot has no gripper``：检查 station 是否保留了夹爪块。
* doctor 的 ``franka.feedback:gripper`` 提示读数不可信：核对 ``max_width_m`` 和 SDK 宽度单位。
* ``grasped=unknown``：当前反馈不能判断抓取状态，按现场反馈确认，别把它记为抓取成功。

7. 读相机状态
--------------------------------------------------------------------------------

读取几帧图像，确认 ``wrist`` 对应正确设备，并且持续出帧。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start camera-state
   :end-before: # end camera-state

**你应该看到**：序列号、640 × 480 的图像形状、帧龄和健康状态。格式示例如下：

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start camera
   :end-before: # end camera

30 fps 是请求值，``measured`` 是实测值。两者不要求完全相等。

**如果不对**：

* 找不到 ``wrist``：检查 station 的相机 ``name``，命令与配置用同一个名称。
* ``failed cameras: wrist`` 或没有帧：检查 USB 3 带宽、线缆和序列号，再重连。
* 启动时报告模式不支持：把宽、高、fps 改成这台相机支持的组合。

8. 选择给策略的观测
--------------------------------------------------------------------------------

设备都能读取后，再改 ``local/task.yaml``，选择这次实验使用的相机和图像大小。

.. literalinclude:: ../examples/task.yaml
   :language: yaml
   :start-after: # start observations
   :end-before: # end observations

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``obs.rgb``
     - 从刚检查过的 station 相机 ``name`` 填入；本章是 ``[wrist]``。
   * - ``obs.image_size``
     - 按模型需要填写高、宽；本例先用 ``[128, 128]``，录制、训练和部署保持一致。

其余字段保持默认：``env_backend: real_robot``、``env_id: pick_and_place``、
``obs.state: true``、``obs.depth: []``。换实验时可改 ``env_id`` 作为标识；
它不自动提供成功判定。需要深度时才同步修改 station 的深度开关与 ``obs.depth``。

**你应该看到**：配置使用 ``wrist``，采集尺寸仍为 640 × 480，模型输入为 128 × 128。
环境会做缩放。后续策略收到的键是 ``state`` 和 ``rgb_wrist``，
RGB 张量是 ``(1, H, W, 3)`` 的 uint8。
这一节只检查文件，还没有启动策略。

**如果不对**：

* 名称与相机检查不同：把 ``obs.rgb`` 改成 station 中的实际名称。
* 增加第二台相机：先给它唯一名称和序列号，通过相机检查，再把名称加入这里。
* 训练和部署图像形状不同：统一 ``image_size`` 和预处理后再使用模型。

9. 选择动作和回合长度
--------------------------------------------------------------------------------

最后检查 preset 中的动作尺度和时间设置，让后续录制与策略使用同一套约定。
下面这段保留在 ``real_robot:`` 下。

.. literalinclude:: ../examples/task.yaml
   :language: yaml
   :start-after: # start action
   :end-before: # end action

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``action_mode`` / ``xyz_scale_m`` / ``rot_scale_rad``
     - 本路线先保留 ``cartesian_delta``、0.05 m、0.3 rad；与接下来遥操作的目标限制一起核对。
   * - ``control_hz``
     - 本例先用 10 Hz；接入后按实测循环能力调整，并在采集与部署时保持一致。
   * - ``max_episode_steps``
     - 按任务允许的回合长度填写；本例 200 步，不含 reset 时间。

其余字段保持默认：``num_joints: 7``、``gripper_in_action: true`` 和 ``state_fields``。
状态按这里的字段顺序拼接：TCP 位姿、速度、关节位置、速度。只有更换策略输入约定时才改顺序或字段。

**你应该看到**：本章配置得到七维动作。下表直接说明每一段如何执行。

.. list-table:: 七维动作
   :header-rows: 1

   * - 维度
     - 含义
     - 输入范围与执行单位
   * - 0–2
     - 位置增量
     - 各维 [-1, 1] × ``xyz_scale_m``，单位 m，相对实测 TCP，基座系
   * - 3–5
     - 旋转增量
     - 各维 [-1, 1] × ``rot_scale_rad``，旋转向量，单位 rad
   * - 6
     - 夹爪
     - [-1, 1] 经 ``(a + 1) / 2`` 变为设备宽度 [0, 1]；-1 关闭，+1 打开

默认的 ``current`` 动作语义使用上表映射。home 沿用 station，reset 等待时间保留：

.. literalinclude:: ../examples/task.yaml
   :language: yaml
   :start-after: # start reset
   :end-before: # end reset

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - 无
     - ``home_joints`` 已在第 4 步填写；这里沿用 station 的值。

其余字段保持默认：``reset_settle_s: 0.5``；现场归位后需要更久稳定时再调整。
未设置 ``real_robot.task`` 时任务名默认为 ``none``，reset 会归位、开爪，再等待。

**如果不对**：

* 现有示范用的是关节动作：先匹配示范的动作约定，再选择 preset，不能只照搬七维长度。
* 需要更小运动范围：同时审阅动作尺度、设备单步上限和下一章的遥操作目标限制。
* 回合总是提前结束：检查 ``max_episode_steps``；它不是 reset 的超时时间。
* 复位位置不对：回查 station 的实测 home，不要用模板数字继续试运动。

10. 记住 station 和 preset
--------------------------------------------------------------------------------

后面每条命令都要用这两份文件。用 ``rwrl use`` 记住它们，之后就不用每次写
``--station`` 和 ``--preset``。
``rwrl use`` 把选择记在 ``local/.rwrl-use.json``，只对从当前目录（仓库根目录）运行的命令生效。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start use
   :end-before: # end use

**你应该看到**：两条命令都先检查文件能否加载，再记住它，并打印当前的选择：

.. literalinclude:: ../examples/station-use-output.txt
   :language: text
   :start-after: # start remember
   :end-before: # end remember

之后不带参数运行 ``rwrl use`` 可以查看当前选择。省略参数的命令会在 stderr
说明自己用了哪份文件：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start use-show
   :end-before: # end use-show

.. literalinclude:: ../examples/station-use-output.txt
   :language: text
   :start-after: # start fallback
   :end-before: # end fallback

.. tip::

   后续章节的设备命令都省略 ``--station local/station.yaml`` 和 ``--preset local/task.yaml``。
   命令里显式写出的 ``--station`` 或 ``--preset`` 总是优先；设置了 ``$RWRL_STATION`` 时它也优先于记住的 station。
   ``train``、``record``、``deploy`` 不读记住的 preset，仍要显式传 ``--config``。

**如果不对**：

* ``station local/station.yaml does not exist``：命令不在仓库根目录运行，或第 1 步没有复制模板。
* ``... does not load: ...``：文件有 YAML 语法错误或字段不合法；先按报错修好再记住。
* 输出 ``station: ... ($RWRL_STATION)``：环境变量优先于记住的选择；不需要时 ``unset RWRL_STATION``。
* 记住的文件被移走或改名：命令会报 ``chosen with `rwrl use station` does not exist``；
  重新运行 ``rwrl use station <yaml>``，或用 ``rwrl use --clear station`` 清除选择。

现在两份配置已准备好。下一章 :doc:`spacemouse` 会选择输入设备，
用 ``teleop run`` 把 station 和 preset 合成一份会话配置，再开始接管练习。
本章 preset 没有独立的连接信息，因此先沿用这条合并流程。

.. note::

   **本章哪些内容需要真机确认**

   文中的硬件输出是按 CLI 格式编写的示例。FCI 与 SDK 握手、真实设备反馈、
   夹爪行程和开闭、相机模式与画面、home 与归位路径、工作区和单步上限、
   动作方向和幅度、10 Hz 控制时序、0.5 秒稳定时间，都要现场确认。
   ``workspace show`` 只显示配置，不验证现场几何、碰撞或在线 daemon。
