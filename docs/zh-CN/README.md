<a id="documentation-index"></a>

# 文档索引

[中文入口](../README.zh-CN.md) · [English](../README.md)

[中文文档入口](../README.zh-CN.md) — 全部文档的完整中文译文、阅读顺序与操作速查。

<a id="tutorial"></a>

## 教程

使用一台带夹爪的 Franka 机械臂、一套由机器人守护进程读取的 GELLO 主臂和键盘，从尚未配置的机器人 PC 开始，最终部署抓取放置策略。请按顺序阅读。

- [0. 总览与安全](tutorial/00-overview.md)
- [1. 安装](tutorial/01-install.md)
- [2. 配置工作站](tutorial/02-station.md)
- [3. 启动守护进程并检查](tutorial/03-daemon.md)
- [4. 录制示范](tutorial/04-record-demos.md)
- [5. 基础策略 BC 或 ACT](tutorial/05-base-policy.md)
- [6. 人工介入操作员指南](tutorial/06-intervening.md)
- [7. HIL 训练](tutorial/07-hil-training.md)
- [8. 部署](tutorial/08-deploy.md)

<a id="concepts"></a>

## 概念

各方法与人工介入路径的工作原理。

- [带人工介入的残差 RL（`residual_hil_serl`）](residual.md)
- [HIL-RL（SiLRI）](silri.md)
- [遥操作：输入源、介入包装器与新增设备](teleop.md)
- [将两条机械臂组合成一个环境（`real_robot_bimanual`）](bimanual.md)
- [HIL-SERL 路线图：已移植的能力与差异](hil-serl-roadmap.md)

<a id="reference"></a>

## 参考

参数、文件格式和硬件细节。

- [仓库 README](project-readme.md#running-on-hardware)：动作约定、设备限制、部署、示范约定 sidecar，以及 fake 机器人端到端演练。
- [GELLO 主臂：硬件、标定与机器人 YAML](gello.md)。
- [示范录制：recorder 参数与 pkl 格式](recording-demos.md)。
- [`examples/presets/hil_single_arm.yaml`](../../examples/presets/hil_single_arm.yaml)：单臂环境预设。
- [robot_infra：设备层、配置向导、doctor 与守护进程](robot-infra.md)。
- [真机实验](real-world-experiments.md)：持久上传、恢复、结果、局部接管、轨迹归档、任务与观测。
- [机械臂控制配置](control-profiles.md)：工作站增益、后端能力与生效设置。
- [已知问题与延期工作](known-issues.md)：首次真机运行可能遇到的问题、临时处理方式和计划修复。
- [硬件入门](getting-started-hardware.md)（已迁入教程，保留旧链接）。

<a id="hardware-test-cases"></a>

## 真机测试案例

- [真机测试页索引（英文主版本）](../testing/README.md)：英文测试页、记录 CSV 约定与证据要求；本目录的 testing 页面为对应中文翻译，真机结果在现场填写。
