<a id="documentation-index"></a>

# 文档索引

[中文文档入口](../README.zh-CN.md) — 全部文档的完整中文译文、阅读顺序与操作速查。

<a id="tutorial"></a>

## 教程

一条 Franka 机械臂、Franka Hand、一台 NUC 和 GELLO 主臂，从空白 NUC 到部署抓取放置策略。录制器和 HIL actor 直接在 NUC 运行，每步均为 `rwrl` 命令。请按顺序阅读。

- [0. 总览与安全](tutorial/00-overview.md)
- [1. 安装](tutorial/01-install.md)
- [2. 配置工作站](tutorial/02-setup.md)
- [3. 启动 Franka](tutorial/03-franka-bringup.md)
- [4. 归位、夹爪、相机](tutorial/04-first-motion.md)
- [5. 遥操作：GELLO](tutorial/05-teleop-gello.md)：标定、读回并运行受监督笛卡尔会话
- [6. 遥操作：SpaceMouse](tutorial/06-teleop-spacemouse.md)：HID 检查、方向档案、任务坐标系和受监督会话
- [7. 遥操作：主从跟随](tutorial/07-teleop-leader-follower.md)：关节空间接管起始、笛卡尔主臂及第二条机械臂（需硬件验证）
- [8. 录制示范](tutorial/08-record-demos.md)
- [9. 基础策略：BC 或 ACT](tutorial/09-base-policy.md)
- [10. HIL 训练](tutorial/10-hil-training.md)
- [11. 部署](tutorial/11-deploy.md)

录制与训练时放在机器人旁的[操作员页](intervening.md)。Actor 位于另一台机器时见[通过机器人 daemon 运行远程 actor](remote-actor.md)。

<a id="reference"></a>

## 参考

- [rwrl：工作站命令行](cli.md)：全局选项、各命令、退出码、JSON
- [rwrl doctor](doctor.md)：检查、状态、修复、报告及红色行处理
- [录制示范：recorder 参数与 pkl 格式](recording-demos.md)
- [GELLO 主臂：硬件、标定、机器人 YAML](gello.md)
- [SpaceMouse 调试：档案、任务工作空间、会话](spacemouse-commissioning.md)
- [设备层：工作站文件、后端、安全规则](devices.md)
- [机器人守护进程：服务、RemoteRobot 与进程监督](daemon.md)
- [机械臂控制配置](control-profiles.md)：工作站增益、后端能力与生效设置
- [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml)：单臂环境预设
- [仓库 README](project-readme.md#running-on-hardware)：动作约定、安全限制、部署、示范约定附属文件、fake 端到端演练
- [真机实验](real-world-experiments.md)：持久上传、恢复、结果、局部接管、轨迹归档、任务和观测
- [已知问题与延期工作](known-issues.md)：首次真机运行可能遇到的问题、临时处理、计划修复

<a id="concepts"></a>

## 概念

方法与人工介入路径的工作原理。

- [带人工介入的残差 RL（`residual_hil_serl`）](residual.md)
- [HIL-RL（SiLRI）](silri.md)
- [遥操作：输入源、介入包装器、新增设备](teleop.md)
- [两条机械臂合为一个环境（`real_robot_bimanual`）](bimanual.md)
- [HIL-SERL 路线图：已迁移能力与差异](hil-serl-roadmap.md)

<a id="hardware-test-cases"></a>

## 真机测试案例

- [硬件会话检查表](testing/README.md)：英文主检查表、记录 CSV 约定及证据要求。硬件结果在现场填写。

GitHub Wiki 由 GitHub Action 从这些仓库文件生成，请直接修改仓库文件。
