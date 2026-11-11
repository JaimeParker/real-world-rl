<a id="arm-control-profiles"></a>

# 机械臂控制配置

[中文入口](../README.zh-CN.md) · [English](../../robot_infra/docs/control-profiles.md)

工作站 YAML 可为每个机械臂分别声明阻抗配置：

```yaml
arm_backend: franky
control:
  translational_stiffness: 500.0
  rotational_stiffness: 30.0
  cartesian_damping: [45.0, 45.0, 45.0, 11.0, 11.0, 11.0]
  translational_error_clip: [0.02, 0.02, 0.02]
  rotational_error_clip: [0.1, 0.1, 0.1]
  max_delta_tau: 0.5
```

以下是示意设置。操作硬件前应按实际负载、工具、任务和已安装 SDK 选择并验证数值。自动测试覆盖 fake SDK 与本地 RPC；机器人稳定性需要上机测量。

`robot_infra.control.ArmControlConfig` 无需导入 torch 即可验证有限数值和向量大小。笛卡尔增益使用 tracker 的标量平移/旋转刚度参数、六轴阻尼向量和三轴误差裁剪。关节控制具有独立 `joint_stiffness`、`joint_damping` 向量，长度与机械臂关节数一致。`max_delta_tau` 和 `gains_time_constant` 传给两类 tracker。

实验配置可覆盖工作站增益，但只能减小已声明的扭矩变化速率限制与误差裁剪。现有工作空间和伺服步长上限继续由 `ArmSafety` 独立执行，控制配置与这些限制共同生效。

仅在 IDLE 状态配置：

```python
arm.set_control_mode(ControlMode.IDLE)
effective = arm.configure_control({"max_delta_tau": 0.3})
metadata = arm.control_metadata
```

守护进程通过 `arm.configure_control` 和 `arm.control_metadata` 提供相同操作，与机器人命令及 watchdog 串行执行。`RemoteArm` 检查已声明能力，读取守护进程生效配置。旧守护进程可在本地接受空请求；显式不支持的设置在运动前失败。即使覆盖为空，若守护进程有工作站默认值，也会在 reset 运动前核对已安装 tracker API。

Franky 应用显式配置前检查已安装 tracker 构造器签名。值直接传给 `CartesianImpedanceTracker` 或 `JointImpedanceTracker`；安装包缺少请求关键字时抛 `VersionMismatch`。`control_metadata` 记录后端、SDK 版本、实际传给 tracker 的请求设置，以及可从构造器读取的标量 SDK 默认值。省略且默认值不可读取的字段明确标为未知。仅包含请求值的配置不能枚举全部底层控制器默认值。

工作站向导重载 YAML 时保留配置，新工作站写为 `control: null`。其他机械臂后端只有声明支持时才接受非空配置。fake 后端接受相同配置以供集成测试，未模拟扭矩动力学。

配置向导还可通过可选参数接收配置，无需增加交互提示：

```bash
rwrl --station local/station.yaml setup --control '{max_delta_tau: 0.5}'
```

可传一份双臂共用映射，或逐臂映射。不传参数会保留现有配置，`--control null` 会删除配置。现有相机宽高描述源视频流；裁剪/ROI 和策略缩放属于实验预处理配置。
