建立工作站
================================================================================

你会接通一台 Franka、Franka Hand 和一台 RealSense，再写好第一份实验配置。
先完成 :doc:`install`。本章命令都在 NUC 的仓库根目录、已激活的环境中运行。

``station`` 是一份 YAML，写这台机器接了哪些设备、怎么连。
``preset`` 是另一份 YAML，写这次实验给策略什么观测、策略输出什么动作。
本章直接连接设备。``daemon`` 是让其他程序远程使用设备的服务，这次不用启动。

1. 复制模板
--------------------------------------------------------------------------------

先复制两份模板，后面只改本地副本。

.. tutorial-test: skip
.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start copy
   :end-before: # end copy

模板来自仓库的 ``robot.franka.example.yaml``、``gello_cartesian.yaml`` 和
``hil_single_arm.yaml``，已经组合成单臂、单相机的例子。
下文“保持默认”指保留模板里的值。
先改 ``local/station.yaml``；``local/task.yaml`` 留到设备检查完成后再改。

**你应该看到**：``local/`` 下有 ``station.yaml`` 和 ``task.yaml``。
前者的 ``REPLACE_WITH_...`` 要换成现场值；home 和限位的数字会在后面读出来再填。

**如果不对**：

* 找不到模板：确认终端在仓库根目录。
* ``local/`` 已有自己的配置：先备份或另选文件名，并同步修改后续命令中的路径。

**已有调试好的工作站。** 复用现场 station 和项目环境，不要再复制模板覆盖它。
每次使用前，在对应机械臂的 Desk 请求控制权并刷新，确认 FCI 已激活，
结束其他占用设备的进程，再做第 3 步 doctor。浏览器控制权变了，不等于需要重新标定 GELLO。

.. _station-setup-wizard:

**用向导代替手填 station。** 新工位也可以运行 ``rwrl setup``：它先探测设备，
再逐项询问、按你的选择标定 GELLO，最后写入带注释的 station 并运行设备 doctor。
它只生成 station；实验 preset 仍用前面的 ``local/task.yaml``。
先完成下方 Control 网络检查，再运行真实向导；没有硬件时先用模拟演练。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start setup
   :end-before: # end setup

每个问题都能用选项回答。每臂选项可给一个值供所有臂使用，也可按机械臂顺序各给一个值。
地址、相机和 GELLO 串口既可输入向导列出的编号，也可输入实际值。

.. list-table:: 向导常用选项
   :header-rows: 1

   * - 选项
     - 填什么
   * - ``--arms``、``--arm-names``、``--robot-ip``
     - 1 或 2 臂、名称、各臂 FCI 地址；名称用于 doctor 的行前缀。
   * - ``--arm-backend``、``--gripper``
     - 臂为 ``franky``、``serl_http`` 或 ``fake``；夹爪还可选 ``none``。
   * - ``--robot-model``
     - ``panda`` 或 ``fr3``，决定默认关节限位；非交互创建新臂时必填。
   * - ``--cartesian-controller``、``--joint-controller``、``--realtime``
     - 控制器默认 ``impedance``，也可选 ``internal``／``serl``；实时设置为 ``ignore``／``enforce``。internal、serl 或 PREEMPT_RT 内核默认 enforce；其他情况默认 ignore。``--joint-controller`` 默认 ``impedance``，GELLO 关节遥操作可选 ``trajectory_impedance``\ （也建议 enforce）。SERL 和关节控制器的安装见 :doc:`controller`。
   * - ``--home-joints``、``--joint-limits``、``--tcp-box-low``、``--tcp-box-high``
     - 逗号分隔的关节角、各关节 lo/hi 对、TCP 盒相对 home 的下界／上界（沿桌面轴，m，如 ``'[-0.2,-0.25,-0.15]'``）；home 可填 ``none`` 留待第 4 步实测。不要拿向导建议值当工位验收结果。没有运动学模型的机器人再加 ``--tcp-origin``。
   * - ``--max-delta-xyz-m``、``--max-delta-joint-rad``、``--control``
     - 单步位置／关节上限和阻抗参数映射；完整格式见本步的 ``setup --help``。
   * - ``--cameras``
     - ``SERIAL=wrist``，多相机逗号分隔；无相机填 ``none``。
   * - ``--camera-resolution``、``--camera-fps``、``--camera-depth``
     - 如 ``1280x720``、``15``、``yes``；未提供时新相机为 640×480、30 fps、无深度，已有相机保留原值。
   * - ``--teleop``、``--gello-port``
     - ``gello``／``spacemouse``／``fake``／``none``；GELLO 填稳定的 ``/dev/serial/by-id/`` 路径。
   * - ``--gello-baudrate``、``--gello-joint-ids``、``--gello-gripper-id``
     - 波特率、逗号分隔的关节舵机 ID、扳机 ID（没有则 ``none``）。
   * - ``--intervention``、``--event-device``
     - ``always`` 或 ``toggle``；事件设备路径或 ``none``。踏板及接管设置见 :doc:`gello`。
   * - ``--daemon-port``
     - 每臂设备服务端口；双臂使用不同端口。
   * - ``--calibrate``／``--no-calibrate``
     - 现在标定 GELLO／稍后标定；交互模式会询问，非交互模式默认不标定。
   * - ``--scope local``／``--scope user``／``--output PATH``
     - 写入 ``./local/station.yaml``／``~/.config/rwrl/station.yaml``／指定路径。
   * - ``--allow-unprobed``
     - 接受探测未发现的 IP、序列号或串口；不能替代第 3 步真实检查。
   * - ``--scan-subnet``、``--port``、``--probe-timeout-s``
     - 额外探测子网、串口及 TCP 探测超时；需要时查看帮助后填写。

要让脚本遇到缺失答案立即失败，使用 ``--non-interactive``。
以下命令以已有 station 提供答案，只更新相机；创建新站点时也可把所有答案写成选项。
只要某个问题既没有选项也没有旧值，就会报错，不会悄悄接受交互建议。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start setup-noninteractive
   :end-before: # end setup-noninteractive

已有输出文件会先备份为 ``station.yaml.bak.<UTC时间戳>``（其他文件名同样加后缀）。
未标定的新 GELLO 会写入零偏移、全 ``+1`` 方向和 ``TODO calibrate`` 注释。
这些占位值不能用来带动从臂。之后按 :doc:`gello` 标定，或重新进入向导：

.. warning::

   标定会关闭 GELLO 舵机力矩，需操作员在主臂旁手动转动关节和扳机。
   保持从臂停止；不要用未验证的偏移、方向和扳机范围开始遥操作。

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start setup-calibrate
   :end-before: # end setup-calibrate

非交互模式不能同时请求 ``--calibrate``。只想演练写文件和 doctor 时，用独立输出文件：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start setup-mock
   :end-before: # end setup-mock

``--mock`` 使用预设探测结果、模拟 GELLO 和模拟 doctor；不要把演练文件当作现场配置。

**你应该看到**：写入路径、设备检查的 ``Result:`` 和后续命令。
**如果不对**：缺答案就补选项或旧 station；探测失败先修连接；doctor 的阻塞项按第 3 步处理。
文件已写出不代表设备已通过。

.. _station-multi-arm:

**双臂和局部重配。** 双臂用不同 IP、名称、串口和 daemon 端口；
先逐臂完成本章检查，再做 :doc:`gello` 的双臂练习。下面第一条启动双臂向导，后两条逐臂检查：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start setup-bimanual
   :end-before: # end setup-bimanual

只重配一部分时，带上 ``--station`` 和 ``--only cameras``、``--only teleop`` 或
``--only daemon``，其余部分沿用旧值。不要把两臂的 home 或标定互相复制。

**接通 Control 网络。** NUC 网线接控制箱的 Control 端口，不是机械臂底座的 Arm 端口。
下面假设现场控制箱是 ``172.16.0.2``，NUC 专用网口是 ``172.16.0.1/24``；
地址已改过时用现场值。先检查路由，再把 ``ROBOT_IF`` 换成实际网口名：

.. tutorial-test: skip
.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start network-check
   :end-before: # end network-check

接口缺少该地址时，可临时添加；已存在时不重复执行。重启后临时地址会消失。

.. tutorial-test: skip
.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start network-temporary
   :end-before: # end network-temporary

若这个专用接口由 NetworkManager 管理，复用它的连接配置；没有配置时新建。
先确认接口不是当前 SSH 或互联网连接使用的接口，再修改：

.. tutorial-test: skip
.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start network-persistent
   :end-before: # end network-persistent

机器人链路不设网关，不替换互联网默认路由。先检查 VPN 是否抢占机器人网段，重启后再查一次路由和 ping。
若接口是 unmanaged，改用实际管理该接口的网络配置工具，不要同时交给两个管理器。

**你应该看到**：路由从 Control 网口发出，源地址为 ``172.16.0.1``，ping 有回复。
**如果不对**：核对接口、现场子网、网线和控制箱电源；不要继续尝试机械臂运动。

2. 填连接地址和相机序列号
--------------------------------------------------------------------------------

先在 NUC 打开目标机械臂的 Desk。Desk 运行在控制箱上，不用在 NUC 安装。
确认页面对应现场控制箱后，逐项检查：

#. 页面可达、状态灯正常；先处理故障。
#. 已登录，本机持有控制权；必要时 Request control，然后刷新页面确认。
#. 关节已 Unlock joints，装了 Hand 时已 End Effector Power On。
#. System 的 Installed Features 中有 Franka Control Interface。
#. 从 Desk 菜单激活 FCI；运行 Desk 任务不能替代激活 FCI。

记录 Dashboard 的 Control 和 System image 版本。
``FCI 1.0.0`` 这类 Installed Features 条目是功能版本，不是固件或 robot server 版本，
也不是应安装的 libfranka 版本。版本对应关系和握手报错的读取方法见 :doc:`controller` 第 2 步。

然后填能让程序找到设备的两个值：Franka IP 和 RealSense 序列号。

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
``cartesian_controller: impedance`` 表示由 franky 的阻抗 tracker 执行末端目标，普通内核也能用。
先保持它：做完第 3 步后，:doc:`controller` 会帮你在 ``impedance``、``internal``、``serl`` 中选定一个，
并装好它需要的软件、填好 ``realtime``。
``robot_model`` 决定 SERL 关节复位用哪张关节限位表，也是 ``rwrl setup`` 给 ``limits.joints`` 的默认来源。
归位速度需要调整时再审阅 ``relative_dynamics_factor``，它控制 ``move_to_*`` 的速度、加速度和 jerk 比例。
``claim: true`` 保留本机设备独占锁。

插入 RealSense，运行发现命令：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start discover
   :end-before: # end discover

需要机器读取报告时运行 ``rwrl --json discover``。结果是一个 ``schema_version: 1`` 对象，
包含 ``cameras``、``teleops``、``can_interfaces``、``hosts``、``hid_devices``、``unknown_hid`` 列表；
没有发现时为空列表。``hosts`` 来自 station 中的 IP 探测，不是 FCI 控制会话。

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

doctor 会连接后断开，不执行归位或机械臂运动；设备初始化仍可能改变设备状态，例如关闭 GELLO 力矩。

.. warning::

   停止动作必须能在现场完成。直连时 ``rwrl stop`` 也要取得 FCI 连接，
   第二个连接不能停止已被录制器或 actor 占用的机械臂；先结束持有连接的程序，紧急情况用物理停止装置。
   ``BLOCKED`` 不会替你停止或抢占别的进程。
   进程内看门狗不覆盖整个进程冻结；独立监督器可终止子进程，之后靠固件处理连接丢失。
   daemon 看门狗覆盖静默的 servo 客户端，不覆盖 daemon 或 NUC 本身故障。

报告先列主机检查，再列每臂设备检查。按层查首个失败，不要把后续跳过当成通过：

.. list-table:: 六层设备检查
   :header-rows: 1

   * - 层
     - 行名（前缀是机器人名称）
     - 检查什么
   * - 0 deps
     - ``franka.deps``
     - 所需 extras
   * - 1 sdk
     - ``franka.sdk:*``
     - 各设备 SDK 能否导入
   * - 2 os
     - ``franka.os:*``
     - 网络、设备节点和权限
   * - 3 handshake
     - ``franka.handshake``
     - 独占声明和连接；失败后跳过反馈与模式检查
   * - 4 feedback
     - ``franka.feedback:*``
     - 臂、夹爪、相机和遥操作设备是否返回数据
   * - 5 mode
     - ``franka.mode:*``
     - 机器人模式、版本和实时要求

报告尾部 ``Report:`` 给出报告目录；有阻塞项时 ``Needs attention:`` 重列问题和提示。
检查结果只有 PASS、WARN、SKIPPED 时退出 0；仍有 FAIL、BLOCKED、PENDING_ACTION 时退出 1。

**你应该看到**：下面几项通过，另有 ``franka.feedback:gripper`` 的夹爪反馈。
这是源码格式示例，现场结果以你的报告为准。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

无需硬件即可演练完整报告（与机器相关的值使用占位符）：

.. code-block:: bash

   rwrl doctor --mock --station examples/stations/robot.franka.example.yaml

.. doctor-example: mock
.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start mock-doctor
   :end-before: # end mock-doctor

**如果不对**：

* ``franka.os:ping``／``franka.os:tcp`` 失败：核对 IP、NUC 网口和控制柜状态。
* ``franka.handshake`` 失败：检查 FCI、设备占用。若报告 ``libfranka mismatch: robot server N, ...``，
  记下 N，先读 :doc:`controller` 第 2 步再决定怎么装 franky：打算用 ``serl`` 的，\ **不要**\ 运行
  ``rwrl doctor --install-sdk``，按 :doc:`controller` 第 4B 步在本机一起构建 franky 和 serl_controller；
  其他控制器才用 ``rwrl doctor --station local/station.yaml --install-sdk`` 换成匹配的 franky wheel。
  ``--install-sdk`` 只处理握手确认的版本不匹配。
* ``franka.feedback:arm`` 失败：检查电源、急停和 Desk 状态，修复后重跑。
* ``franka.feedback:camera:wrist`` 报 ``no frames within 2s``：检查 USB 3 带宽，换接口后重连。
  夹爪反馈异常则按后面的夹爪检查核对行程和单位。

串口、输入组或 USB 延迟问题可用引导修复；它先预览并询问，再执行和复查：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start doctor-fix
   :end-before: # end doctor-fix

**你应该看到**：修复后的检查或需要你完成的 PENDING_ACTION。
**如果不对**：更改了组成员关系时，退出登录再重新登录，重跑普通 doctor；仅开新终端不够。

4. 读出 home，再填回文件
--------------------------------------------------------------------------------

把机械臂放到下一次实验开始的位置，读出关节角，作为本仓 reset 使用的 home。
``reset`` 是开始新回合时的复位流程；本章配置会归位并打开夹爪。

.. warning::

   接下来会手动引导机械臂。先确认负载、停止方式、周围空间和现场监护。
   测量期间不要运行 ``home go``、录制或 reset。以后执行 reset 前，
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

``limits.tcp`` 的盒子以 home 为中心，并冻结了 home 的位置。改了 ``home_joints`` 之后，加载 station 会报 ``home_joints changed since limits were frozen`` 并提示运行 ``rwrl limits rebase``，
这时运行 ``rwrl limits rebase local/station.yaml --write``（第 5 步细讲），盒子随新 home 重新居中。

**如果不对**：

* 读数命令连接失败：确认已退出手动引导，重新检查 Desk 控制权和 FCI。
* ``q`` 不是七个有限数：停止填写，重新运行 doctor 检查机械臂反馈。
* 修改后 YAML 不能加载：保留方括号、逗号与原有缩进，再检查。

**可选：把实测 home 写入实验 preset。** station 的 home 仍按上文保存。
如果这次任务需要自己的 home，在退出手动引导后运行：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start home-preset
   :end-before: # end home-preset

``home set`` 不运动、不询问；检查 station、机械臂和 preset 的关节限位后，
把实测值保留四位小数写入 ``real_robot.task.reset.home_joints``，保留注释和键顺序，
旧文件留为同目录的 ``task.yaml.<id>.bak``。

**你应该看到**：旧值、新值和备份路径；``home show`` 的来源为 preset task.reset.home_joints。
这个值优先于 station home；第 9 步“沿用 station”只适用于没有设置此覆盖值的任务。
**如果不对**：缺 preset 就显式传入路径；越限就先核对实测姿态和限位，不能为了写入而放宽限制。

5. 设定限位，再检查保存结果
--------------------------------------------------------------------------------

给机械臂定两类限位：每个关节的活动范围，和末端 TCP 的活动盒。它们都写在 ``local/station.yaml`` 的一个 ``limits:`` 块里。
模板已经带了 Panda 手册的关节范围和一个以 home 为中心的盒子，这一步是核对、按现场收紧，再检查保存结果。

盒子的设计：它\ **以 home 为中心**\ ，三个方向各给一个相对 home 的偏移（``low`` 都是负数，``high`` 都是正数，单位 m），
所以 home 一定在盒内；它沿\ **桌面轴**\ ：轴取自 ``task_frame``（第 11 步用桌上三个点标定的桌面坐标系），还没标定时沿基座轴。
机械臂侧装或倾斜安装时，盒子的边仍与桌面对齐。盒内\ **不限制姿态**\ 。完整字段和越界行为见\ :ref:`限位（limits） <devices-limits>`\ 。

先选一个已留出工具和障碍物余量的区域：以 home 为中心，向前后、左右、上下各留多少米。模板的默认值是 x ±0.20、y ±0.25、z 从 -0.15 到 +0.30。
把结果写入 ``local/station.yaml``：

.. literalinclude:: ../examples/station.yaml
   :language: yaml
   :start-after: # start bounds
   :end-before: # end bounds

.. list-table::
   :header-rows: 1

   * - 要核对的字段
     - 怎么得到这个值
   * - ``limits.joints``
     - 保持 ``rwrl setup`` 按 ``robot_model`` 写的手册值，除非现场需要更窄的范围；``home_joints`` 必须在范围内。
   * - ``limits.tcp.low`` / ``limits.tcp.high``
     - 相对 home 的偏移，沿桌面轴，单位 m。现场有障碍物时向内收紧。``low`` 必须全为负、``high`` 必须全为正。
   * - ``limits.tcp.origin_xyz`` / ``axes_R``
     - 不手填：由 ``rwrl setup`` 写入，或运行 ``rwrl limits rebase``（见下）。它们冻结 home 的位置和 ``task_frame`` 的旋转；
       以后改了 home 或 ``task_frame``，加载会提示重新运行 ``rebase``，不会悄悄移动盒子。

其余字段保持默认：``measured_margin_m: 0.003`` 是实测 TCP 超出盒子多少米以上才算越界。
``max_delta_per_step`` 把从实测状态到 servo 目标的单步变化限制为示例的 0.05 m／0.1 rad；
只有现场核对控制周期和运动后才调整。

如果你自己写（或改了）``tcp`` 块，可以只写 ``origin: home``、``low``、``high``，再让 ``rebase`` 补上两个冻结值。
先不带 ``--write`` 看会改什么，满意后再带 ``--write``：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start rebase
   :end-before: # end rebase

保存后检查文件里的值：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start workspace
   :end-before: # end workspace

**你应该看到**：``limits.tcp`` 的 ``low`` / ``high`` 与刚才填写的一致，``home_joints`` 与第 4 步一致。
``home_within_limits`` 为 true 表示 station home 在已配置关节限位内；false 表示越限，
查看 ``home_violations``。null 表示缺 home 或关节限位，不能当作通过。
下面是输出字段的格式节选。输出还会标明 m、rad、xyzw 单位。

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start workspace
   :end-before: # end workspace

再看八个角点在基座系下的坐标，确认盒子落在你想要的区域：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start limits
   :end-before: # end limits

.. literalinclude:: ../examples/station-output.txt
   :language: text
   :start-after: # start corners
   :end-before: # end corners

可选的现场核对：沿用上一步的手动引导流程，把 TCP 引导到几个角点附近（先从离障碍物最近的开始），
退出引导后运行一次 ``rwrl tcp``，读数应落在对应角点的盒内：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start corner
   :end-before: # end corner

角点在盒内不代表整个盒子无碰撞，也不代表机械臂本体不碰障碍物。

**如果不对**：

* ``tcp bounds require low < 0 < high on every axis``：``low`` 必须全是负数、``high`` 必须全是正数，home 在盒子里面。
* ``home_joints changed since limits were frozen``：改过 home；运行 ``rwrl limits rebase local/station.yaml --write``。
* ``task_frame changed since limits were frozen``：改过桌面坐标系；同样运行 ``rebase``。
* 报 ``safety_box`` / ``workspace`` / ``joint_limits`` 已删除：这是旧版站点文件，运行 ``rwrl limits migrate local/station.yaml``，核对后加 ``--write``。
* 输出仍是旧值：保存文件，确认命令使用的是 ``local/station.yaml``。
* 某个角点或路径不安全：缩小盒子，不要为了凑满盒子强行到达。

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
``camera show`` 默认读取 30 次，可用 ``--frames N`` 修改；无帧等检查失败时退出 1。
还要保存一帧，目视确认画面与相机名称对应：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start camera-grab
   :end-before: # end camera-grab

**你应该看到**：目录内有 ``wrist-color.png`` 和 ``wrist.json``；
启用了深度且返回深度帧时还有 ``wrist-depth.png``。JSON 保存相机配置、帧时间及图像形状等信息。
命令不覆盖同名抓取文件；再次采集请换新的输出目录。

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
     - 按模型需要填写高、宽；本例推荐 ``[224, 224]``，录制、训练和部署保持一致。
       尺寸越大占用越多：一帧无损 224×224 RGB 约 150 KB（每个相机每个控制步一帧，压缩前），
       录制数据集的磁盘和训练缓冲的内存都按它放大；资源紧张时可改用较小尺寸，但各阶段要一致。

其余字段保持默认：``env_backend: real_robot``、``env_id: pick_and_place``、
``obs.state: true``、``obs.depth: []``。换实验时可改 ``env_id`` 作为标识；
它不自动提供成功判定。需要深度时才同步修改 station 的深度开关与 ``obs.depth``。

**你应该看到**：配置使用 ``wrist``，采集尺寸仍为 640 × 480，模型输入为 224 × 224。
环境会做缩放。后续策略收到的键是 ``state`` 和 ``rgb_wrist``，
RGB 张量是 ``(1, H, W, 3)`` 的 uint8。
若要裁剪画面，先从保存的彩色图量像素坐标。在 ``real_robot.preprocess.wrist`` 中，
``source_resolution`` 是原图的 ``[高, 宽]``，``roi`` 是 ``[top, left, height, width]``。
ROI 必须在原图内，设置 ROI 时也必须提供 source_resolution；实际帧尺寸不匹配会报错。
处理顺序是先裁剪，再缩放到 ``obs.image_size``。也可用 ``crop_region: center_square``
或归一化的 ``[top, left, bottom, right]``（0–1，边界递增）代替像素 ROI，不能同时设置两者；
``crop_region`` 按相机原始尺寸解析，并要求提供 ``image_size``。
``resize`` 默认 ``pil_bilinear``，也可选 ``cv2_linear``；RGB 使用对应的双线性插值，深度使用最近邻。
不要把缩放后的 224×224 当原图尺寸。ROI、深度选择和缩放尺寸都是观测契约，录制、训练和部署必须一致。
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
     - 先保留 ``cartesian_delta``、0.05 m、0.3 rad；与接下来遥操作的目标限制一起核对。
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

11. 看清机械臂怎么装，测桌面坐标系
--------------------------------------------------------------------------------

这一步在第一次遥操作之前做。它决定 SpaceMouse（以及任何“按桌面方向”给出的指令）怎么变成机械臂的运动。

先看机械臂是怎么装的：**正装**\ （底座平放在桌面上）、\ **侧装**\ （例如绕自己的 x 轴转了约 45°，本实验室常见），
还是有别的倾斜或偏转。SpaceMouse 放在桌面上，它的方向是桌面的方向：前 = 桌面 +x，左 = +y，上 = 竖直 +z。
Franka 的基座坐标系跟着底座走；底座不正装时，基座的 “上” 不是竖直向上。
所以 SpaceMouse 的指令要先\ **转到基座系**\ ，再变成末端目标：

.. code-block:: text

   SpaceMouse 读数 (桌面系 table：前 +x、左 +y、上 +z；平移 xyz 和旋转 rpy 各三维)
        │   × 方向档案的 yaw（只描述 SpaceMouse 在桌上朝哪摆，见 SpaceMouse 章第 3 步）
        ▼
   base_R_table  = station task_frame.base_T_frame 的左上 3×3
        │   Δp_base = base_R_table · Δp_table ；Δω_base = base_R_table · Δω_table
        ▼   （平移和旋转用同一个矩阵）
   基座系里的增量
        │   位置：目标 = 当前位姿 + 缩放后的 Δp_base ；姿态：旋转增量左乘当前姿态
        ▼
   末端目标（基座系）
        ▼
   控制器 cartesian_controller：impedance | internal | serl （它们收到的已是基座系目标，不再旋转）

正装且桌面 x／y 轴与基座 x／y 轴对齐时，``base_R_table`` 才是单位矩阵，
这时可以不写 ``task_frame``；仅凭底座正装不能排除水平 yaw 偏差。
不正装时写好 ``task_frame``，tracker 和 serl 两种 SpaceMouse 模式、三种控制器就都按它工作：

.. list-table::
   :header-rows: 1

   * - 情况
     - SpaceMouse 指令怎么转到基座系
   * - tracker 模式，有方向档案
     - 档案的旋转：``base_R_table`` × 档案 yaw（档案按 ``--frame table`` 标定）
   * - tracker 模式，没有档案，有 ``task_frame``
     - 只用 ``base_R_table``
   * - serl 模式（默认）
     - 只用 ``base_R_table``，不加档案 yaw
   * - serl 模式，``use_profile_rotation: true``
     - 档案的旋转（``base_R_table`` × 档案 yaw）
   * - 没有 ``task_frame``，也未使用档案旋转
     - 不旋转，读数直接当基座系；需确认输入轴与基座轴对齐（tracker 有档案时仍使用档案旋转）

策略输出的动作本来就是基座系增量，不经过这条链。

**11.1 写名义值，判断正负号。** 先按安装方式写一个名义的 ``task_frame``。下面矩阵表示桌面轴相对基座轴为 ``Rx(+45°)``；
它是桌面到基座的变换，与“基座相对桌面转 +45°”的安装角互为逆变换：

.. literalinclude:: ../examples/station-frame.yaml
   :language: yaml
   :start-after: # start nominal
   :end-before: # end nominal

``base_T_frame`` 是 4×4 矩阵：左上 3×3 的三列分别是桌面的 +x、+y、+z 在基座系里的方向；
最后一列是桌面系原点在基座系里的位置（名义值先写 0）。
+45° 还是 −45°，用一次竖直抬高来判断：把 TCP 放在工作区中部，读一次 ``tcp``，
手动引导它\ **竖直**\ （按房间的上方，不是按机械臂的某个轴）抬高约 10 cm，再读一次：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start frame-sign
   :end-before: # end frame-sign

.. literalinclude:: ../examples/station-frame-output.txt
   :language: text
   :start-after: # start frame-sign
   :end-before: # end frame-sign

两次 xyz 之差就是“桌面向上”在基座系里的方向，也就是矩阵第三列（乘 0.1 m）：

.. list-table::
   :header-rows: 1

   * - xyz 的变化（约）
     - 安装
     - 名义 ``base_R_table``
   * - ``[0, 0, +0.10]``
     - z 轴对齐；仍需核对水平 x／y 轴
     - x／y 也对齐时才是单位矩阵（不写 ``task_frame``）
   * - ``[0, -0.07, +0.07]``
     - 桌面轴相对基座轴 ``Rx(+45°)``
     - 上面的 ``Rx(+45°)``
   * - ``[0, +0.07, +0.07]``
     - 桌面轴相对基座轴 ``Rx(-45°)``
     - ``Rx(-45°)``：把上面矩阵第 2 行第 3 列改成 ``+0.7071…``、第 3 行第 2 列改成 ``-0.7071…``

**11.2 测出真实的坐标系。** 名义值只是起点：实际安装总有偏差。例如本实验室一台名义 45° 的机械臂，
手眼标定测出 roll 41.8°、pitch 2.3°、yaw −6.7°；另一台是 −46.1°、−0.4°、3.9°。
用三个点测出完整的 roll／pitch／yaw：

1. 原点 ``frame-origin``：桌面上一个好找的点，例如桌角附近；
2. ``frame-x``：沿桌面的“前”方向（SpaceMouse 前方要对着的方向）离原点 20–30 cm；
3. ``frame-y``：沿桌面的“左”方向离原点 20–30 cm。

三个点离桌面的高度保持一样（例如都让指尖刚好碰到桌面上贴的标记）。沿用第 4 步的手动引导流程：
引导到一个点，退出引导，捕获；再去下一个点：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start frame-capture
   :end-before: # end frame-capture

``--output`` 相对 station 所在目录，所以写入 ``local/frame-points.yaml``。然后先不带 ``--save`` 看候选，满意后再带 ``--save`` 写入 station：

.. literalinclude:: ../examples/station.sh
   :language: bash
   :start-after: # start frame-set
   :end-before: # end frame-set

**你应该看到**：候选 ``task_frame`` 和下面这些读数（这里是合成数据的真实输出）：

.. literalinclude:: ../examples/station-frame-output.txt
   :language: text
   :start-after: # start frame-set
   :end-before: # end frame-set

.. list-table::
   :header-rows: 1

   * - 读数
     - 怎么看
   * - ``rpy_deg``
     - 桌面系相对基座的 roll／pitch／yaw（``base_R_table = Rz(yaw)·Ry(pitch)·Rx(roll)``）。应接近名义值：侧装 45° 的 roll 在 ±45° 附近，相差几度属于正常的安装偏差；差十几度先查点是否取错。
   * - ``axis_lengths_m``
     - 原点到 x 点、y 点的距离。越长越准，建议都在 0.2 m 以上。
   * - ``y_axis_angle_deg``
     - y 点偏离“与 x 垂直”的角度（y 轴会自动改成与 x 垂直）。桌边是直角时应只有几度；很大说明 y 点取歪了。

带 ``--save`` 时先显示改动、确认后才写入；``workspace frame show`` 再显示一次保存结果和 ``rpy_deg``。

**11.3 先坐标系，后档案。** 坐标系定好以后，再去 :doc:`spacemouse` 标定方向档案（``--frame table``）：
这样档案的 yaw 只描述 SpaceMouse 在桌上怎么摆，不再混入机械臂的安装角度。
改了 ``task_frame`` 之后，之前在旧坐标系下标定的档案和捕获的点都要重做。
``task_frame`` 的旋转同时决定第 5 步 TCP 盒的轴：保存新的 ``task_frame`` 后，加载 station 会报 ``task_frame changed since limits were frozen`` 并提示运行 ``rwrl limits rebase``。先运行
``rwrl limits rebase local/station.yaml``\ （不带 ``--write`` 只看改动），确认后加 ``--write``：
盒子的 ``low`` / ``high`` 不变，``axes_R`` 换成新桌面系的旋转，``origin_xyz`` 重新取 home 的位置。
然后运行 ``rwrl workspace show`` 和 ``rwrl limits show`` 核对基座系下的角点，再用
``rwrl workspace validate --pose X Y Z QX QY QZ QW`` 检查待用的基座系位姿
（位置单位 m，四元数 xyzw；用实际读数替换占位符；只检查位置是否在盒内）。``workspace frame set`` 的输出也会提醒这一点。
最后在 :doc:`spacemouse` 第 4 步用单轴推动验证：推帽向上，TCP 应竖直向上走；向前推，TCP 沿桌面前方走。

**如果不对**：

* ``origin, x-axis and y-axis points are collinear``：y 点要离开 x 方向那条线。
* ``x-axis point is ... mm from the origin``：x 点离原点太近，拉开到 20 cm 以上。
* ``rpy_deg`` 和名义值差很多：检查 x、y 点有没有取反（前／左），三个点是否同高。
* 保存 ``task_frame`` 后 ``rwrl`` 报 ``task_frame changed since limits were frozen``：按上文运行 ``rwrl limits rebase``。
* 正装机器人测出的 roll／pitch 只有零点几度：还要核对 yaw；输入轴与基座轴都对齐时才可省略 ``task_frame``。

现在两份配置和桌面坐标系都已准备好。下一章 :doc:`controller` 选定并装好 Franka 控制器；
之后 :doc:`spacemouse` 用 ``teleop run`` 把 station 和 preset 合成一份会话配置，开始接管练习。

.. note::

   **本章哪些内容需要真机确认**

   文中的硬件输出是按 CLI 格式编写的示例。FCI 与 SDK 握手、真实设备反馈、
   夹爪行程和开闭、相机模式与画面、home 与归位路径、限位盒和单步上限、
   动作方向和幅度、10 Hz 控制时序、0.5 秒稳定时间，都要现场确认。
   ``workspace show`` 和 ``limits show`` 只显示配置，不验证现场几何、碰撞或在线 daemon。
   桌面坐标系的名义角度和实测角度、单轴方向也要现场确认。
