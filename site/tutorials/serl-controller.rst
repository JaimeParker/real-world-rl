路线一 2C. 选择 Franka 控制器，启用 SERL 控制器
================================================================================

本章讲 station 里的 ``cartesian_controller``：谁把末端目标变成电机力矩。
默认的 ``impedance`` 已经能完成前两章；想让 SpaceMouse 的手感与 SERL／HIL-SERL 一致时，
按本章换成 ``serl``。本章命令在机器人主机（NUC）的仓库根目录、已激活的环境中运行，
沿用 :doc:`station` 的 ``local/station.yaml``。

.. tip::

   这里有两个容易混的开关，它们互不依赖：

   * station 的 ``cartesian_controller``：**机器人端**\ 怎么执行目标（本章）。
   * ``teleop run --mode tracker|serl``：**SpaceMouse 端**\ 怎么把推帽变成目标（见 :doc:`spacemouse`）。

   换控制器只改 station 文件，不改代码、不重新录制方向档案。

1. 先看懂三种控制器
--------------------------------------------------------------------------------

每个 servo 目标（策略或遥操作的一步）都会交给下面三者之一：

.. list-table::
   :header-rows: 1

   * - ``cartesian_controller``
     - 谁在执行
     - 需要什么
     - 适合
   * - ``impedance`` （默认）
     - franky 的扭矩级阻抗 tracker，像弹簧一样把 TCP 拉向目标；增益来自 station 的 ``control:``
     - 普通内核也能跑
     - 第一次接机器人、调试其他部分
   * - ``internal``
     - 机器人自带的控制器。每个目标变成一段基座系速度运动，0.2 s 内补完误差，
       限速 0.25 m/s、1.0 rad/s，300 ms 没有新目标就自己停下
     - PREEMPT_RT 内核和 ``realtime: enforce`` （速度接口对 1 kHz 抖动很敏感）
     - 希望机器人自己处理 IK 和负载
   * - ``serl``
     - SERL 原版的笛卡尔阻抗控制器（从 ``serl_franka_controllers`` 逐行移植，结果逐位相同），
       ``move_to_joints`` 也换成 SERL 的 10 s 关节复位；增益来自 station 的 ``serl:``
     - 同一次构建的 franky + serl_controller 两个 wheel；建议 PREEMPT_RT 和 ``realtime: enforce``
     - 想要 SERL／HIL-SERL 的操作手感

关节 servo 在三种模式下都是 franky 的关节阻抗 tracker；``move_to_pose`` 始终是 Ruckig 轨迹。

和 SpaceMouse 两种模式的搭配：

.. list-table::
   :header-rows: 1

   * - 控制器 \\ SpaceMouse
     - ``--mode tracker`` （默认）
     - ``--mode serl``
   * - ``impedance``
     - 前两章用的组合，按 button_0 接管
     - 可用；推帽即接管，但执行端不是 SERL 的控制器
   * - ``internal``
     - 可用
     - 可用
   * - ``serl``
     - 可用；推荐先用它做第一次运动
     - 最接近 HIL-SERL：推帽即接管，原始基座系输入，SERL 控制器执行

**你应该看到**：你能说出现在 station 里写的是哪一个；没写就是 ``impedance``。

2. 在 Desk 确认机器人型号
--------------------------------------------------------------------------------

``robot_model`` 告诉本仓这是哪一代 Franka。它决定 SERL 关节复位使用哪张关节限位表，
也决定 ``rwrl setup`` 给 ``joint_limits`` 提供的默认值。

在 Desk 打开系统设置，查看系统版本：

.. list-table::
   :header-rows: 1

   * - Desk 系统版本
     - 机器人
     - 填写
   * - 4.x
     - Franka Emika Robot（FER，常叫 Panda）
     - ``robot_model: panda`` （默认）
   * - 5.x
     - Franka Research 3
     - ``robot_model: fr3``

libfranka 握手时报告的 robot server version 与此对应：1–5 是 4.x，6 及以上是 5.x。
例如 Zizhu 的机器人报告 server version 9，所以很可能是 FR3；仍以 Desk 显示为准。

**如果不对**：Desk 打不开就先修网络，不要凭外观猜型号；两代的第 6、7 关节限位差别很大。

3. 装实时内核
--------------------------------------------------------------------------------

SERL 当年在 franka_control 下以 ``realtime_config: enforce`` 运行。
没有实时内核时，1 kHz 控制循环偶尔迟到，机器人会触发通信类反射并停下。
``internal`` 同样需要它。先检查当前内核：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start rt-check
   :end-before: # end rt-check

没有实时内核时，按 Franka 官方文档
`Setting up the real-time kernel <https://frankarobotics.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel>`_
安装 PREEMPT_RT 内核（Ubuntu 22.04 也可以用 Ubuntu Pro 的 ``realtime-kernel``），重启进入新内核。
然后给当前用户实时优先级，重新登录后再跑一次上面的检查：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start rt-limits
   :end-before: # end rt-limits

**你应该看到**：``uname -v`` 含 ``PREEMPT_RT``，``/sys/kernel/realtime`` 为 1，``ulimit -r`` 为 99。

**如果不对**：

* ``ulimit -r`` 仍是 0：确认已经重新登录，``id`` 里有 ``realtime`` 组。

4. 装同一次构建的两个 wheel
--------------------------------------------------------------------------------

``serl_controller`` 是一个 C++ 扩展，它的运动类直接继承 franky 的 C++ 类，
还链接 franky wheel 自带的 ``libfranky.so`` 和 ``libfranka``。所以它只能和
**同一次构建**\ 出来的 franky wheel 一起用。PyPI／GitHub 上的官方 franky wheel 不行。

两个 wheel 放在同一个目录 ``local/wheels/rwrl-franky-build-20261122-cxxpass/`` 里，
旁边有 ``BUILD_INFO.txt`` 和 ``SHA256SUMS``。``local/`` 不进 git，先把整个目录拷到机器人主机：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start copy-wheels
   :end-before: # end copy-wheels

然后在机器人主机的项目环境里安装。先卸载旧的两个包，再\ **一起**\ 装这两个文件：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start install-wheels
   :end-before: # end install-wheels

**你应该看到**：``sha256sum -c`` 每行都是 ``OK``；最后一行打印的 ``BUILD_INFO`` 里有
franky 版本 ``2.0.0+libfranka.0.17.0`` 和 build id。

**如果不对**：

* ``python --version`` 不是 3.10：这对 wheel 是 cp310，换用 3.10 的环境，不要强装。
* ``sha256sum`` 报 ``FAILED``：拷贝不完整，重新拷整个目录。
* 以后有人运行了 ``pip install franky-control`` 或 ``rwrl doctor --install-sdk``：官方 wheel 会覆盖
  配对的那个，``sdk:serl_controller`` 会失败；重新执行本步。
* 这对 wheel 里的 franky 是未修改的 v2.0.0，``impedance`` 和 ``internal`` 模式照常可用，
  所以切换控制器不需要换 wheel。

5. 改 station：控制器、型号与实时
--------------------------------------------------------------------------------

改之前先备份，回滚时直接拷回来：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start backup
   :end-before: # end backup

在 ``local/station.yaml`` 顶层（与 ``robot_ip`` 同级）修改或加入：

.. literalinclude:: ../examples/serl-station.yaml
   :language: yaml
   :start-after: # start controller
   :end-before: # end controller

.. list-table::
   :header-rows: 1

   * - 必须改的字段
     - 怎么得到这个值
   * - ``cartesian_controller``
     - 本章填 ``serl``；想回到前两章的行为填 ``impedance``，或删掉这一行。
   * - ``realtime``
     - 第 3 步检查通过才填 ``enforce``；内核不是 PREEMPT_RT 时 ``enforce`` 会让 doctor 失败。
   * - ``robot_model``
     - 第 2 步 Desk 的结果：4.x 填 ``panda``，5.x 填 ``fr3``。

``joint_limits`` 不会因为 ``robot_model`` 自动改变：station 里已经写了的表照常生效，
SERL 复位前仍会拒绝超出它的目标。新跑 ``rwrl setup`` 时，向导按 ``robot_model``
提供 Panda 或 FR3 的手册限位作为默认值。

6. 填 SERL 参数
--------------------------------------------------------------------------------

``serl:`` 是 SERL 控制器的增益和限幅，键名、默认值、范围和 SERL 的
``compliance_param.cfg`` 完全一样。下面就是默认值；全部省略效果相同。

.. literalinclude:: ../examples/serl-station.yaml
   :language: yaml
   :start-after: # start serl
   :end-before: # end serl

.. list-table::
   :header-rows: 1

   * - 字段
     - 含义（范围）
   * - ``translational_stiffness`` / ``translational_damping``
     - 平移刚度 N/m（0–6000）／阻尼（0–400）
   * - ``rotational_stiffness`` / ``rotational_damping``
     - 旋转刚度（0–300）／阻尼（0–30）
   * - ``nullspace_stiffness`` / ``joint1_nullspace_stiffness``
     - 零空间保持起始姿态的刚度（0–100），第 1 关节单独一项
   * - ``translational_clip_{x,y,z}`` / ``translational_clip_neg_{x,y,z}``
     - 每轴平移误差的正／负上限，m（0–0.1）。刚度 × 上限就是能用的最大推力，默认 2000 × 0.01 = 20 N
   * - ``rotational_clip_*``
     - 旋转误差（四元数向量部分）的正／负上限（0–0.1）
   * - ``translational_Ki`` / ``rotational_Ki``
     - 积分增益（0–100），默认 0

第一次上机保持默认，不用改。超出范围的值会被\ **夹到范围内**\ 并记录警告（和 SERL 的
dynamic_reconfigure 一样）；写错键名或写非数字会直接报错。

``control:`` 块里的笛卡尔增益（``translational_stiffness``、``rotational_stiffness``、
``cartesian_damping``、``translational_error_clip``、``rotational_error_clip``）只给 ``impedance``
用，``serl`` 模式下忽略，doctor 会报 ``config:serl_control`` WARN。不想看到它就删掉这几项；
``control:`` 的关节项仍用于关节 servo。

7. 自动恢复、负载与夹爪
--------------------------------------------------------------------------------

**自动恢复。** 推到障碍物、碰撞阈值等会触发机器人反射（reflex），SERL 控制器随之停下。
HIL-SERL 在每个 ``/pose`` 前调用 ``/clearerr``，所以操作员感觉不到中断。
本仓的 ``serl_auto_recover`` 做同样的事：下一个 servo 目标到来时清错误，
在实测位置\ **新建**\ 一个 SERL 控制器，再执行这个目标。

.. literalinclude:: ../examples/serl-station.yaml
   :language: yaml
   :start-after: # start recover
   :end-before: # end recover

.. list-table::
   :header-rows: 1

   * - 字段
     - 怎么填
   * - ``enabled``
     - 默认 ``true``。想每次反射都停下来人工确认时填 ``false``。
   * - ``max_per_episode``
     - 每个回合最多自动恢复几次，默认 5；0 等于关闭。

“回合”从进入笛卡尔 servo 时开始，环境每次 reset 也重新计数。超过次数后，
servo 报可恢复错误，需要 ``rwrl recover``。和 HIL-SERL 的一点区别：HIL-SERL 在同一个控制器
实例上重新 ``starting()``；本仓新建控制器，刚度和阻尼从 0 用约 0.2 s 爬升，
和 SERL 每次重启 ``impedance.launch`` 一样。

**负载。** ``load`` 在每次连接后、任何运动前发给机器人（libfranka 的 ``setLoad``，
相当于 HIL-SERL 的 ``/set_load``）。三种控制器都生效。

.. literalinclude:: ../examples/serl-station.yaml
   :language: yaml
   :start-after: # start load
   :end-before: # end load

只在手爪上装了 Desk 没登记的东西时才填；质心在法兰坐标系，惯量绕质心，3×3。
机器人拒绝时 ``connect`` 失败。

**夹爪。** Franka Hand 现在默认使用 HIL-SERL 的命令：闭合是以 0.3 m/s、130 N 抓到 0.01 m
（空抓也算成功），张开是以 0.3 m/s 完全打开；命令在后台线程里执行，不会卡住控制步，
连续按键时只执行最新的一次。这与控制器选择无关，已有 station 的夹爪块不写这些键时也是这些值。
要显式写出或修改：

.. literalinclude:: ../examples/serl-station.yaml
   :language: yaml
   :start-after: # start gripper
   :end-before: # end gripper

.. list-table::
   :header-rows: 1

   * - 字段
     - 怎么填
   * - ``speed_m_s``
     - 手指速度，默认 0.3。
   * - ``grasp_force_n``
     - 抓取力，默认 130，不能超过 130 N。夹易碎物体时调小。
   * - ``close_width_m``
     - 闭合目标宽度，默认 0.01。
   * - ``open_width_m``
     - 张开宽度，默认 ``null`` （完全张开）。

8. 让 doctor 检查
--------------------------------------------------------------------------------

在 Desk 取得控制权、激活 FCI 后运行：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start doctor
   :end-before: # end doctor

**你应该看到**：除了 :doc:`station` 里的那些行，还有下面几行，且都是 PASS。
这是按源码格式写的示例，现场以自己的报告为准。

.. literalinclude:: ../examples/serl-controller-output.txt
   :language: text
   :start-after: # start doctor
   :end-before: # end doctor

.. list-table::
   :header-rows: 1

   * - 行
     - 检查什么
   * - ``sdk:serl_controller``
     - 两个 wheel 是否来自同一次构建（不导入扩展，比较 build_info 与已装 franky 的 libfranky.so 指纹）
   * - ``config:serl_control``
     - ``control:`` 里有没有 serl 模式会忽略的笛卡尔增益
   * - ``mode:serl_realtime`` / ``mode:internal_realtime``
     - 用 ``serl`` ／ ``internal`` 时是否 ``realtime: enforce``
   * - ``mode:robot_model``
     - station 的 ``robot_model`` 和机器人自报的型号是否一致。franky 2.0 的 Python 接口没有
       ``Robot.server_version``，所以这里读机器人提供的 URDF（``Robot.model_urdf``）里的关节名
       （``fr3_joint`` ／ ``panda_joint``）；读不到时显示 could not be cross-checked

**如果不对**：

.. literalinclude:: ../examples/serl-controller-output.txt
   :language: text
   :start-after: # start doctor-warn
   :end-before: # end doctor-warn

* ``sdk:serl_controller`` FAIL：装的不是同一次构建，回到第 4 步。
* ``mode:serl_realtime`` WARN：回到第 3 步；临时测试可以先忽略，但要预期偶发的通信反射。
* ``mode:robot_model`` WARN：回到第 2 步在 Desk 确认，再改 ``robot_model``。

9. 第一次运动：只走 1 cm
--------------------------------------------------------------------------------

先用熟悉的 tracker 模式和 :doc:`spacemouse` 的 preset，只换执行端。

.. warning::

   首次 reset 会走 SERL 的关节复位：10 s 线性插值到 home，比 Ruckig 慢，路径也不同。
   先清空路径，手放在急停上，保持现场监护。

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start first-servo
   :end-before: # end first-servo

中立检查通过后按 button_0 接管，沿 +x 轻推约 1 cm，松帽，再按 button_0 交还。

**你应该看到**：复位约 10 s 后停在 home；接管后机械臂沿推帽方向平稳移动、松帽后停住，
没有明显的抖动或下坠。终端打印接管状态行：

.. literalinclude:: ../examples/serl-controller-output.txt
   :language: text
   :start-after: # start intervention
   :end-before: # end intervention

``INTERVENTION OFF`` 要在不接管持续 0.3 s 后才打印，所以 serl 模式下推帽时短暂松手不会刷屏；
reset 时如果仍处于接管，会打印 ``INTERVENTION OFF: episode reset``。

每次启动控制器，刚度和阻尼从 0 爬升约 0.2 s，开头一瞬间会软一点，这是 SERL 的原样行为。

**如果不对**：

* 方向不对：控制器不改变方向；回 :doc:`spacemouse` 核对方向档案和任务系。
* 日志有 ``joint reset TIMEOUT``：约 30 s 内关节没到 home 附近（每关节约 0.01 rad），复位仍会结束；
  检查是否被挡住、home 是否在限位边上。
* 一启动就报 ``cartesian_motion_generator``／``communication_constraints_violation`` 一类反射：
  回第 3 步确认实时内核。

10. 反射与自动恢复
--------------------------------------------------------------------------------

接管时把末端轻轻推向桌面或软物体，直到机器人触发反射。

**你应该看到**：日志里一行 WARNING，下一次推帽机器人从当前位置继续动：

.. literalinclude:: ../examples/serl-controller-output.txt
   :language: text
   :start-after: # start recover-log
   :end-before: # end recover-log

超过 ``max_per_episode`` 或错误清不掉时，按 Desk 和现场情况处理后运行：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start recover
   :end-before: # end recover

``rwrl recover`` 成功后，下一个 servo 目标会启动一个新的 SERL 控制器。

11. 可选：SERL 模式的 SpaceMouse 与参数档案
--------------------------------------------------------------------------------

tracker 模式没问题后，再按 :doc:`spacemouse` 第 5 步准备 ``local/task-serl.yaml``，
用 SERL 自己的 SpaceMouse 语义：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start serl-mode
   :end-before: # end serl-mode

这时推帽的原始读数直接当作\ **基座系**\ 动作，不经过方向档案的旋转（与 HIL-SERL 相同）；
详见 :doc:`spacemouse` 第 5 步。

任务需要不同的刚度或限幅时，在 preset 里写命名档案。切换一个档案是在当前参数上做
**部分更新**：档案没写的键保持之前的值，和 HIL-SERL 的 ``update_param`` 一样。

下面是 preset ``real_robot:`` 块里的片段（完整文件见 ``site/examples/serl-task.yaml``）：

.. literalinclude:: ../examples/serl-task.yaml
   :language: yaml
   :start-after: # start profiles
   :end-before: # end profiles
   :dedent: 2

档案在 preset 加载时检查键名和数值；环境不会自动切换它们，由任务代码在需要时调用
``arm.update_serl_params``。不同档案写了不同的键时会有警告：结果会依赖切换顺序。

12. 回滚
--------------------------------------------------------------------------------

切换控制器只改 station。回到原来的行为，把备份拷回来再检查：

.. literalinclude:: ../examples/serl-controller.sh
   :language: bash
   :start-after: # start rollback
   :end-before: # end rollback

也可以只把 ``cartesian_controller`` 改回 ``impedance`` 或 ``internal``。配对的 wheel 可以留着；
只有想回到官方 franky wheel 时才运行 ``rwrl doctor --install-sdk``，之后 ``serl`` 不再可用。

.. note::

   **本章哪些内容需要真机确认**

   实时内核与抖动、wheel 配对、Desk 型号、SERL 关节复位路径、1 cm 运动的方向和滞后、
   反射后的自动恢复、夹爪力度与速度都要现场确认。按顺序的上机验收步骤和通过标准见
   `SERL 控制器上机验收清单 <https://github.com/JaimeParker/real-world-rl/blob/main/docs/serl-robot-validation.md>`_
   （`中文版 <https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/serl-robot-validation.md>`_）。
   深层参考见 `docs/serl-controller.md <https://github.com/JaimeParker/real-world-rl/blob/main/docs/serl-controller.md>`_。

检查清单：doctor 的 SERL 相关行都通过；1 cm 运动方向正确；反射后能自动或手动恢复；知道怎么回滚。

接着进入 :doc:`record`；用 GELLO 的读者看 :doc:`gello`。
