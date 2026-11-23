SERL 控制器：真机验证
================================================================================

这是 ``cartesian_controller: serl`` 与两种 SpaceMouse 模式的硬件检查清单。请在机器人主机上按顺序执行（例如一台 FR3，后接 NUC，SpaceMouse 接在 NUC 上），在仓库根目录、激活项目环境后操作。每一步都写了要看什么、什么算通过。遇到第一个失败的步骤就停下，并保留它的日志。背景知识见 \ :doc:`SERL 控制器 <serl-controller>`\ 。

每个涉及运动的步骤都要注意安全：一人手握外部急停装置，单元里没有其他人；工作空间要空旷，并远在站点 ``limits.tcp`` 盒以内；机械臂从 home 位置开始。

0. 把机器人主机的工作区切换到这份代码
--------------------------------------------------------------------------------

检出目录里有本地改动时，先保存它们，再切换到包含这次工作的提交：

.. tutorial-test: skip
.. code-block:: bash

   git status --short
   git diff > ~/rwrl-lab-patch-$(date +%Y%m%d).patch    # keep the old patch, just in case
   git stash push --include-untracked -m "lab patch before SERL validation"   # or: git checkout -- . after saving
   git fetch origin && git checkout main && git pull --ff-only
   git submodule update --init rl-garden
   git log --oneline -3                                  # inspect the checked-out revision; it must include SERL support
   python -m pytest tests -q -x                          # optional: the suite needs no hardware

``local/`` 不受 git 管理，上面的操作不会动它。修改工作站前先备份：

.. tutorial-test: skip
.. code-block:: bash

   cp local/station.yaml local/station.yaml.before-serl-validation.bak
   ls local/station.yaml*.bak*      # wizard copies: station.yaml.bak.<UTC>; CLI edits: station.yaml.<id>.bak

1. PREEMPT_RT 内核与实时限制
--------------------------------------------------------------------------------

.. tutorial-test: skip
.. code-block:: bash

   uname -v                          # must contain PREEMPT_RT after the reboot
   cat /sys/kernel/realtime          # 1

.. code-block:: bash

   rwrl doctor --fix                 # writes rtprio 99 / memlock unlimited for this user; log in again

内核需要手动安装：启用了 Ubuntu Pro 的 Ubuntu 上执行 ``sudo pro enable realtime-kernel``\ ，否则按 Franka 的 "Setting up the real-time kernel" 指南操作，然后重启。可选：用 ``sudo cyclictest -m -Sp90 -i200 -h400 -D 60s``\ （来自 ``rt-tests``\ ）测量内核延迟，最大延迟应远低于 100 µs。

通过标准：\ ``rwrl doctor`` 中 ``realtime.kernel`` 和 ``realtime.limits`` 为 PASS。

2. 构建并安装 franky + serl_controller
--------------------------------------------------------------------------------

从 ``rwrl doctor`` 读出机器人服务器版本（\ ``libfranka mismatch: robot server N, ...``\ ；握手通过时看已装 franky 的 ``+libfranka.X`` 标签），按 \ :ref:`SERL 控制器 <serl-controller-installing-franky-and-serl_controller>`\  中的表选 ``LIBFRANKA_VERSION``\ 。这台工作站不要运行 ``rwrl doctor --install-sdk``\ 。然后在机器人主机上（不需要 Docker）：

.. tutorial-test: skip
.. code-block:: bash

   LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
   python -c "import franky, serl_controller as sc; print(sc.BUILD_INFO)"
   cat ~/.cache/rwrl-native-build/BUILD_INFO.txt

通过标准：脚本最后打印 ``PASS  sdk:serl_controller``\ ，CTest 通过（含 ``golden_equivalence`` 和 ``joint_reset_golden``\ ），\ ``BUILD_INFO`` 的构建 id 与 ``BUILD_INFO.txt`` 一致且 ``franky_layout: source``\ 。（在别处用 Docker 配方构建的 wheel 对也可以用；同一个构建目录的两个 wheel 一起装。）

3. robot_model：在 Desk 中确认
--------------------------------------------------------------------------------

在 Desk → Settings → System 中查看系统版本（4.x = Franka Emika Robot / ``panda``\ ，5.x = Franka Research 3 / ``fr3``\ ）和机器人型号。服务器版本 ≥ 6（系统 5.x）时预期是 ``fr3``\ 。在工作站中设置：

.. code-block:: yaml

   robot_model: fr3

4. 诊断：先用当前控制器，再用 serl
--------------------------------------------------------------------------------

先在未改动的工作站（\ ``impedance``\ ）上运行 ``rwrl doctor``\ ，确认源码构建的 franky 没有破坏任何东西：\ ``sdk:arm``\ 、\ ``handshake``\ 、\ ``feedback:*``\ 、\ ``mode:robot_mode`` 为 PASS，\ ``mode:robot_model`` 显示 ``robot_model: fr3 (robot URDF: fr3)``\ （或服务器版本）。

然后修改工作站：

.. code-block:: yaml

   cartesian_controller: serl
   realtime: enforce
   serl: null                 # SERL's .cfg defaults
   serl_auto_recover: null    # {enabled: true, max_per_episode: 5}

先停止 ``rwrl daemon`` 和录制器/actor，在空闲工作站上再次运行 ``rwrl doctor``\ 。检查通过后再启动 daemon。预期结果：

.. list-table::
   :header-rows: 1

   * - 行
     - 预期
   * - ``sdk:serl_controller``
     - PASS ``serl_controller 0.1.0 and franky_control 2.0.0+libfranka.0.17.0 from the same build (...)``
   * - ``config:serl_control``
     - PASS；或 WARN 并列出 ``control:`` 中的 Cartesian 键，这无害，它们只在 ``impedance`` 下生效
   * - ``mode:realtime_kernel``
     - PASS ``PREEMPT_RT kernel; realtime: enforce``
   * - ``mode:serl_realtime``
     - PASS ``cartesian_controller: serl with realtime: enforce``
   * - ``mode:robot_model``
     - PASS ``robot_model: fr3 (...)``\ ；WARN 说明第 3 步没做

5. 第一次 serl 伺服：+x 1 cm
--------------------------------------------------------------------------------

先用 ``spacemouse_mode: tracker``\ （它有领先量上限），以低增益启动一次平移诊断：

.. code-block:: bash

   rwrl --station local/station.yaml --preset local/task.yaml teleop run \
     --mode tracker --diagnose translation --gain 0.2 --duration-s 10 \
     --record local/serl-first-01

``--record`` 是数据集目录；这次诊断的回合也存在 ``local/serl-first-01/`` 里，可用 ``rwrl dataset view`` 回放。按 button 0 接管，轻轻向前（+x）推帽约 1 cm，然后松开。观察并记录：

-  stderr 上的 ``INTERVENTION ON: the operator drives the arm``\ ，以及交还后 0.3 s 出现的 ``INTERVENTION OFF: the operator handed back``\ ；
-  机械臂向前移动，没有侧向或竖直漂移；零点几秒内开始动（每次控制器启动后刚度在约 0.2 s 内从 0 爬升）；松开后停下；
-  会话写出的停止记录（\ ``local/serl-first-01.session-*/stop-trace-*.assessment.json``\ ）和守护进程日志（不应出现 ``SERL impedance controller stopped``\ ）。

然后分析这次会话：

.. tutorial-test: skip
.. code-block:: bash

   python scripts/analyze_teleop_direction.py local/serl-first-01.session-*/

脚本对每个纯平移段打印一行：指令方向与实测位移之间的 ``angle``\ 、\ ``speed``\ 、\ ``lead at release``\ （松手时目标减去 TCP 的距离）和 ``coast``\ （松手后 1 s 内 TCP 继续走的距离），并按 angle < 10°、coast < 3 mm 判定 PASS/FAIL。

**控制循环抖动。** SERL 与原版一样，每个 1 kHz tick 都会分配 Eigen 临时对象（为了逐位一致而保留）。在 PREEMPT_RT 内核上这还没出过问题，但要检查。整个会话期间，守护进程日志里不应出现 ``communication_constraints_violation`` reflex。要直接测量的话，先停掉守护进程，保持位姿 30 s，同时对照控制器的 tick 计数和机器人时间采样。丢 tick 表现为 ``tick`` 落后于 ``rel_time``\ ：

.. code-block:: python

   import time
   from real_world_rl.devices import Robot
   from real_world_rl.devices.arm import ControlMode
   from real_world_rl.station import load_station
   robot = Robot.from_config(load_station("local/station.yaml").select().config); robot.connect()
   arm = robot.arm
   arm.set_control_mode(ControlMode.CARTESIAN_SERVO)
   pose = arm.read_state()["tcp_pose"]
   for _ in range(300):                      # 30 s at 10 Hz, holding the pose
       arm.servo_cartesian(pose); time.sleep(0.1)
   snap = arm._handle.serl_motion.snapshot()  # internal handle: the running SerlCartesianImpedanceMotion
   print(snap["tick"], snap["rel_time"], snap["tick"] / max(snap["rel_time"] * 1000, 1))
   arm.stop(); robot.disconnect()

通过标准：比值 > 0.99（丢 tick 少于 1 %），且没有 reflex。

6. 用同一测试比较 tracker、serl 和 internal
--------------------------------------------------------------------------------

在同一现场，由同一位操作员用同样的推法，重复同一个纯平移测试：沿 ±x、±y、±z 逐轴推，每次 1–2 s，推到大约一半行程。每一行做两遍，\ ``--record`` 用不同的名字：

.. list-table::
   :header-rows: 1

   * - 运行
     - 工作站 ``cartesian_controller``
     - ``--mode``
   * - A
     - ``impedance``
     - ``tracker``
   * - B
     - ``serl``
     - ``tracker``
   * - C
     - ``serl``
     - ``serl``
   * - D
     - ``internal``
     - ``tracker``

.. code-block:: bash

   rwrl --station local/station.yaml --preset local/task.yaml teleop run \
     --mode serl --diagnose translation --gain 0.5 --duration-s 20 --record local/cmp-C-01

.. tutorial-test: skip
.. code-block:: bash

   python scripts/analyze_teleop_direction.py local/cmp-*.session-*/

每次改工作站后都要重启守护进程，并让 ``spacemouse_mode`` / ``--mode`` 与表格一致。运行 C 先按 HIL-SERL 摆放 SpaceMouse（前 = 任务系 +x，左 = +y，上 = 竖直 +z）并做六个单轴检查；基座侧装时需要测好的 ``task_frame``\ （见工作站教程）。\ ``--mode serl`` 需要 preset 中的 xyz 与 RPY 安全框。通过标准（每次运行）：所有段 angle < 10°，且 coast < 3 mm（松手后的位移，与松手时的领先量不同）。记录每次运行的角度中位数、速度和 lead at release。serl 的运行应至少和运行 A 一样好。

7. 夹爪
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl gripper close     # HIL-SERL grasp: 0.01 m, 0.3 m/s, 130 N; prints the width when done
   rwrl gripper open      # fully open at 0.3 m/s
   rwrl gripper move 0.5  # half of max_width_m

在遥操作会话中，夹爪按钮不应让机械臂卡顿：指令在工作线程中执行，新指令会替换排队中的指令。通过标准：宽度符合预期，闭合过程中的机械臂运动没有顿挫。

8. 关节复位
--------------------------------------------------------------------------------

.. code-block:: bash

   rwrl home go           # serl mode: SERL's joint reset (10 s linear blend, then hold)

机械臂在约 10 s 内插值到 home，然后保持。只有约 33 s 内没到达时，守护进程日志才会报告 ``SERL joint reset TIMEOUT``\ 。在没有会话运行时读取报告：

.. code-block:: python

   from real_world_rl.server.remote import RemoteRobot, RemoteRobotConfig
   robot = RemoteRobot(RemoteRobotConfig(address="127.0.0.1:50051")); robot.connect()
   print(robot.arm.last_joint_reset)   # reached, polls, max_abs_error_rad, soft_limited_ticks, robot_model
   robot.disconnect()

通过标准：\ ``reached: true``\ ，\ ``max_abs_error_rad`` < 0.01，\ ``robot_model: fr3``\ 。home 远在限位以内时，\ ``soft_limited_ticks`` 为 0。

9. 自动恢复
--------------------------------------------------------------------------------

在 serl 模式会话中（工作站 ``serl``\ ，\ ``--mode tracker`` 或 ``serl``\ ），慢慢开向一个软障碍物（泡沫块），或请第二个人轻推机械臂，直到触发 reflex（例如 ``cartesian_reflex``\ ）。速度要低。预期：

-  守护进程日志打印 ``SERL impedance controller stopped (<error>); auto-recovery 1/5 this episode: clearing errors and restarting the controller at the measured pose``\ ；
-  机械臂停在原地，短暂变软（约 0.2 s 的爬升），然后跟随下一次输入；
-  ``robot.arm.serl_auto_recovery``\ （同第 8 步）显示 ``episode_count: 1`` 和 ``last.error``\ 。

同一个 episode 内的第六次 reflex 应改为抛出 ``... (serl_auto_recover: max_per_episode 5 reached, ...)``\ ，之后在 ``rwrl recover`` 之前拒绝伺服。通过标准：两种行为都符合，并且恢复后机械臂只做你指令的运动。

10. 回退
--------------------------------------------------------------------------------

要撤销工作站修改，恢复备份并重启守护进程：

.. tutorial-test: skip
.. code-block:: bash

   cp local/station.yaml.before-serl-validation.bak local/station.yaml

更早的副本是 ``local/station.yaml.bak.<UTC>``\ （setup 向导）和 ``local/station.yaml.<id>.bak``\ （CLI 编辑）。要换回官方 franky wheel，先把工作站改成 ``cartesian_controller: impedance`` 或 ``internal``\ ，再执行 ``rwrl doctor --install-sdk``\ 。要回到旧代码，检出之前的提交，或应用第 0 步保存的补丁。

请交回会话目录（\ ``local/*.session-*/``\ ）、守护进程日志、分析输出和 doctor 报告（\ ``rwrl --json doctor > local/doctor-serl.json``\ ）。
