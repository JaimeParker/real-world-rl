<!-- English source: docs/testing/README.md -->

<a id="hardware-session-checklists"></a>

# 硬件测试清单

`docs/testing/` 中的英文页为主版本。`docs/zh-CN/testing/` 中的对应文件是翻译，并非独立测试计划。

每项检查包含一个稳定 ID、一条可复制命令、一项通过标准和一个主要记录文件。共用 shell 辅助函数可采集更多证据。每页注明机器、仓库工作目录和已激活的项目环境；运行表格命令前先执行准备代码块。

记录 CSV 列顺序为：`planned_date`、`actual_start_time`、`item_id`、`item_name`、`expected_result`、`actual_observation`、`conclusion`、`run_directory`、`issue_id`、`notes`。中文 CSV 翻译表头，保留列顺序和项目 ID。结论仅为 **PASS / FAIL / BLOCKED / NOT_RUN**。模板初始为 NOT_RUN；前提或证据缺失为 BLOCKED，实测违规为 FAIL。退出码和操作员任务成功标签独立于检查结论。

在指定控制 PC 的仓库根目录运行。创建新的运行目录，将记录 CSV 复制到该目录并编辑副本，不改模板。将软件提交、实际配置、日志、原始归档、导出步骤、测量和带时间戳的视频一并保存。归档 SQLite 前关闭写入进程；保留其 WAL/SHM 文件。填完 CSV 后归档整个运行目录；后续修改记录后再次归档。这些页面定义检查，不表示硬件已通过。

- [第二次单臂与夹爪测试](single-arm-gripper-session2.md)：GELLO / SpaceMouse 检查与证据采集。
- [SpaceMouse 桌面控制与正常停止验收](spacemouse-control-session.md)：带正负方向的标定、两种控制模式、轨迹保持及停止证据。
- [双臂 Franka / GELLO 测试](bimanual-gello-session.md)——**仅供参考**；保留供下次双臂测试使用，不随教程维护。
