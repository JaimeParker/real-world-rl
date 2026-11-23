机械臂控制配置
================================================================================

工作站 YAML 可为每个机械臂分别声明阻抗配置：

.. code-block:: yaml

   arm_backend: franky
   control:
     translational_stiffness: 500.0
     rotational_stiffness: 30.0
     cartesian_damping: [45.0, 45.0, 45.0, 11.0, 11.0, 11.0]
     translational_error_clip: [0.02, 0.02, 0.02]
     rotational_error_clip: [0.1, 0.1, 0.1]
     max_delta_tau: 0.5

以下是示意设置。操作硬件前应按实际负载、工具、任务和已安装 SDK 选择并验证数值。自动测试覆盖 fake SDK 与本地 RPC；机器人稳定性需要上机测量。

除非指定 ``--control``\ ，\ ``rwrl setup`` 会为使用 ``cartesian_controller: impedance`` 的新 franky 机械臂写入 ``FRANKY_SETUP_CONTROL``\ ；franky 自带的 500 N/m 太软，在约 1 cm 的遥操作领先量下推不动受关节摩擦的机械臂：

.. code-block:: yaml

   control:
     translational_stiffness: 2000.0
     rotational_stiffness: 75.0                       # SERL's 150, approximate (see below)
     cartesian_damping: [89.0, 89.0, 89.0, 7.0, 7.0, 7.0]
     translational_error_clip: [0.01, 0.01, 0.01]
     rotational_error_clip: [0.10, 0.10, 0.10]        # SERL's 0.05, approximate

这些值在 franky 的 tracker 中近似 SERL 的默认柔顺参数（\ ``compliance_param.cfg``\ ）。旋转部分是近似值：franky 的旋转误差是旋转向量 θ·axis，SERL 是四元数虚部 sin(θ/2)·axis ≈ (θ/2)·axis，所以 SERL 的 150 Nm/rad 变成 75，0.05 的限幅变成 2·asin(0.05) ≈ 0.10 rad，只在小角度下相等；阻尼（对速度的 89 / 7）两者相同，这里显式给出。这些增益只在 ``cartesian_controller: impedance`` 时生效（关节相关键在所有模式下用于关节伺服），见\ :ref:`设备 <devices-station-file-reference>`\ 。

``real_world_rl.devices.control.ArmControlConfig`` 无需导入 torch 即可验证有限数值和向量大小。省略字段默认为 ``None``（SDK 默认值）。刚度和阻尼须非负；误差裁剪、\ ``max_delta_tau`` 和 ``gains_time_constant`` 须为正数；拒绝布尔值和未知键。笛卡尔增益使用 tracker 的标量平移/旋转刚度参数、六轴阻尼向量和三轴误差裁剪。关节控制具有独立 ``joint_stiffness``\ 、\ ``joint_damping`` 向量，长度与机械臂关节数一致。\ ``max_delta_tau`` 和 ``gains_time_constant`` 传给两类 tracker。

实验配置可覆盖工作站增益，但只能减小已声明的扭矩变化速率限制与误差裁剪。现有 ``limits``（TCP 盒、关节范围）和伺服步长上限继续由设备层独立执行，控制配置与这些限制共同生效。

仅在 IDLE 状态配置：

.. code-block:: python

   arm.set_control_mode(ControlMode.IDLE)
   effective = arm.configure_control({"max_delta_tau": 0.3})
   metadata = arm.control_metadata

守护进程通过 ``arm.configure_control`` 和 ``arm.control_metadata`` 提供相同操作，与机器人命令及 watchdog 串行执行。\ ``RemoteArm`` 检查已声明能力，读取守护进程生效配置。旧守护进程可在本地接受空请求；显式不支持的设置在运动前失败。即使覆盖为空，若守护进程有工作站默认值，也会在 reset 运动前核对已安装 tracker API。

Franky 应用显式配置前检查已安装 tracker 构造器签名。值直接传给 ``CartesianImpedanceTracker`` 或 ``JointImpedanceTracker``\ ；安装包缺少请求关键字时抛 ``VersionMismatch``\ 。\ ``control_metadata`` 记录后端、SDK 版本、实际传给 tracker 的请求设置，以及可从构造器读取的标量 SDK 默认值。省略且默认值不可读取的字段明确标为未知。仅包含请求值的配置不能枚举全部底层控制器默认值。

工作站向导保留现有配置；新的 franky impedance 机械臂使用上面的默认配置。切换到 ``serl`` 时移除 Cartesian tracker 键并保留关节设置。\ ``--control null`` 显式清空配置。其他机械臂后端只有声明支持时才接受非空配置。fake 后端接受相同配置以供集成测试，未模拟扭矩动力学。

配置向导还可通过可选参数接收配置，无需增加交互提示：

.. code-block:: bash

   rwrl --station local/station.yaml setup --control '{max_delta_tau: 0.5}'

可传一份双臂共用映射，或逐臂映射。不传参数会保留现有配置，\ ``--control null`` 会删除配置。现有相机宽高描述源视频流；裁剪/ROI 和策略缩放属于实验预处理配置。

SERL 柔顺参数与参数组
--------------------------------------------------------------------------------

设置 ``cartesian_controller: serl`` 时，笛卡尔控制器是 SERL 自己的，参数也是 SERL 的：工作站 ``serl:`` 块（名称、默认值和范围同 ``compliance_param.cfg``\ ，省略的键保留 ``.cfg`` 默认值）、任务 preset 中可选的命名参数组，以及运行时的部分更新：

.. code-block:: yaml

   # station
   cartesian_controller: serl
   serl: {translational_stiffness: 2000.0, rotational_stiffness: 150.0}
   # task preset
   real_robot:
     serl_profiles:
       compliance: {translational_clip_x: 0.0075, translational_clip_neg_x: 0.002}
       precision: {translational_clip_x: 0.1, translational_clip_neg_x: 0.1, rotational_stiffness: 250}

.. code-block:: python

   from real_world_rl.devices.control import serl_profile_update
   update = serl_profile_update(preset_profiles, "precision")  # the profile's keys only
   effective = arm.update_serl_params(update)   # merged into the current values (HIL-SERL update_param);
                                                # any mode; applied at the next 1 kHz tick if running
   arm.serl_params                              # the full effective set

参数组是在当前值之上的部分更新，未设置的键保留上一个参数组的值；若各参数组设置的键不同，校验时会警告（此时切换顺序有影响）。

超出 ``.cfg`` 范围的值会被钳制（与 dynamic_reconfigure 相同）；未知键报错。工作站/参数组校验记录钳制日志并保存钳制后的值。在线 ``update_serl_params`` 的钳制也记录到 ``control_metadata["serl"]["clamped"]``\ ；已在配置校验时钳制的值不会在此保留原始请求。与 ``configure_control`` 不同，SERL 参数不受工作站值约束：上限就是 ``.cfg`` 范围，与 SERL 相同。该模式下 ``control:`` 的 Cartesian tracker 键被忽略（加载工作站时的警告和 ``rwrl doctor`` 的 ``config:serl_control`` WARN 会列出它们）。完整参数表、以 HIL-SERL ``ram_insertion`` 为例的参数组以及控制器细节见 \ :ref:`SERL 控制器 <serl-controller-parameters>`\ 。
