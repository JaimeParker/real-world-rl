<a id="documentation-index"></a>

# real-world-rl 中文文档入口

[完整中文文档索引](zh-CN/README.md) — 全部文档的完整中文译文、阅读顺序与操作速查。

<a id="tutorial"></a>

## 教程

Franka + Hand + NUC + GELLO：从空白 NUC 到部署策略。录制与 HIL actor 直接在 NUC 运行，按 00–11 顺序阅读。

- [0. 总览与安全](zh-CN/tutorial/00-overview.md)
- [1. 安装](zh-CN/tutorial/01-install.md)
- [2. 配置工作站](zh-CN/tutorial/02-setup.md)
- [3. 启动 Franka](zh-CN/tutorial/03-franka-bringup.md)
- [4. 归位、夹爪、相机](zh-CN/tutorial/04-first-motion.md)
- [5. 遥操作：GELLO](zh-CN/tutorial/05-teleop-gello.md)：标定、读回并运行受监督笛卡尔会话
- [6. 遥操作：SpaceMouse](zh-CN/tutorial/06-teleop-spacemouse.md)：HID 检查、方向档案、任务坐标系和受监督会话
- [7. 遥操作：主从跟随](zh-CN/tutorial/07-teleop-leader-follower.md)：关节空间接管起始、笛卡尔主臂及第二条机械臂（需硬件验证）
- [8. 录制示范](zh-CN/tutorial/08-record-demos.md)
- [9. 基础策略：BC 或 ACT](zh-CN/tutorial/09-base-policy.md)
- [10. HIL 训练](zh-CN/tutorial/10-hil-training.md)
- [11. 部署](zh-CN/tutorial/11-deploy.md)

录制与训练时放在机器人旁的[操作员页](zh-CN/intervening.md)。Actor 位于另一台机器时见[通过机器人 daemon 运行远程 actor](zh-CN/remote-actor.md)。

<a id="reference"></a>

## 参考

- [rwrl：工作站命令行](zh-CN/cli.md)：全局选项、各命令、退出码、JSON
- [rwrl doctor](zh-CN/doctor.md)：检查、状态、修复、报告及红色行处理
- [录制示范：recorder 参数与 pkl 格式](zh-CN/recording-demos.md)
- [GELLO 主臂：硬件、标定、机器人 YAML](zh-CN/gello.md)
- [SpaceMouse 调试：档案、任务工作空间、会话](zh-CN/spacemouse-commissioning.md)
- [设备层：工作站文件、后端、安全规则](zh-CN/devices.md)
- [机器人守护进程：服务、RemoteRobot 与进程监督](zh-CN/daemon.md)
- [机械臂控制配置](zh-CN/control-profiles.md)：工作站增益、后端能力与生效设置
- [`examples/presets/hil_single_arm.yaml`](../examples/presets/hil_single_arm.yaml)：单臂环境预设
- [仓库 README](zh-CN/project-readme.md#running-on-hardware)：动作约定、安全限制、部署、示范约定附属文件、fake 端到端演练
- [真机实验](zh-CN/real-world-experiments.md)：持久上传、恢复、结果、局部接管、轨迹归档、任务和观测
- [已知问题与延期工作](zh-CN/known-issues.md)：首次真机运行可能遇到的问题、临时处理、计划修复

<a id="concepts"></a>

## 概念

方法与人工介入路径的工作原理。

- [带人工介入的残差 RL（`residual_hil_serl`）](zh-CN/residual.md)
- [HIL-RL（SiLRI）](zh-CN/silri.md)
- [遥操作：输入源、介入包装器、新增设备](zh-CN/teleop.md)
- [两条机械臂合为一个环境（`real_robot_bimanual`）](zh-CN/bimanual.md)
- [HIL-SERL 路线图：已迁移能力与差异](zh-CN/hil-serl-roadmap.md)

<a id="hardware-test-cases"></a>

## 真机测试案例

- [硬件会话检查表](zh-CN/testing/README.md)：英文主检查表、记录 CSV 约定及证据要求。硬件结果在现场填写。

GitHub Wiki 由 GitHub Action 从这些仓库文件生成，请直接修改仓库文件。
