<!-- English source: docs/tutorial/01-install.md -->

<a id="1-install"></a>

# 1. 安装

[中文入口](../../README.zh-CN.md) · [English](../../tutorial/01-install.md)

**目标：** 建立项目环境，使 `rwrl --help` 能列出命令，`rwrl doctor --mock` 以 `Result: no blocking findings` 结束。本页无需机器人、相机或遥操作设备。

**开始前：** NUC（或将运行 `rwrl` 的 Linux 机器）已具备支持 venv 的 Python 3.10、`git` 和供 `pip` 使用的网络连接。尚未激活 Python 环境。

<a id="1-clone-the-repository"></a>

## 1. 克隆仓库

在 NUC 上，进入准备存放检出的父目录，不激活环境：

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
```

`--recurse-submodules` 同时获取 `rl-garden/`，即本项目从源码安装的学习库。

<a id="4-install-the-project-environment"></a>

<a id="2-run-the-installer"></a>

## 2. 运行安装器

在 NUC 的仓库根目录，不激活环境：

<!-- tutorial-test: skip -->
```bash
bash scripts/install.sh
source .venv/bin/activate
```

安装器用 `python3.10` 创建（或复用）`.venv`，清除继承的 `PYTHONPATH`，以可编辑模式安装 rl-garden 和本项目，启用 `learning`、`franka-gello` 扩展（`learning` 还带上 `test` 扩展）。此 `.venv` 即后续所有页面激活的**项目环境**。最后使用机械臂和夹爪均为模拟设备的临时站点运行 `rwrl doctor --mock`；通过只证明软件正常。

| 选项 | 用途 |
|---|---|
| `--extras LIST` | 其他扩展，逗号分隔（默认 `learning,franka-gello`）；例如 `learning,franka-gello,spacemouse` 或添加 `realsense` |
| `--daemon-only` | 仅运行守护进程的机器人 PC：扩展 `franky,grpc`，不安装 `rl-garden/` 和 torch |
| `--venv PATH` | 其他 venv 路径（默认 `.venv`） |
| `--python PROGRAM` | 创建新 venv 的其他解释器（默认 `python3.10`） |
| `--torch-index URL` | 先从此索引安装 torch，例如 `https://download.pytorch.org/whl/cpu` |
| `--station YAML` | 最终 doctor 使用真实站点而非模拟站点 |
| `--no-doctor` | 跳过最终 doctor |

`bash scripts/install.sh --help` 打印同样的列表。后续页面需要的扩展也在此安装：[04](04-first-motion.md#3-cameras) 的 RealSense、[06](06-teleop-spacemouse.md) 的 SpaceMouse。

<details>
<summary>手动执行安装器的步骤</summary>

在 NUC 的仓库根目录，激活任意 venv 或非 base 的 conda 环境：

<!-- tutorial-test: skip -->
```bash
unset PYTHONPATH
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ./rl-garden -e '.[learning,franka-gello,test]'
```

</details>

<a id="3-check-the-command"></a>

## 3. 检查命令

在 NUC 的仓库根目录，激活项目环境：

<!-- tutorial-test: skip -->
```bash
rwrl --help
```

*应看到：* 用法行和命令，包括 `status`、`q`、`tcp`、`stop`、`recover`、`home`、`gripper`、`camera`、`doctor`、`setup`、`daemon`、`supervise`、`discover`、`teleop`、`workspace`、`archive`、`train`、`record`、`deploy` 和 `collect-reward`。`--station` 和 `--yes` 等全局选项放在命令**之前**；`rwrl <command> --help` 显示各命令的选项。

<a id="4-run-the-doctor-without-hardware"></a>

## 4. 无硬件运行 doctor

在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl doctor --mock
```

*应看到*（平台、解释器路径和报告目录会不同）：

<!-- doctor-example: mock -->
```text
PASS           host.platform: Linux ubuntu 22.04 x86_64; container=False; wsl=False
PASS           host.python: /home/robot/real-world-rl/.venv/bin/python
SKIPPED        station: no station configured; arm handshake not performed
    hint: run rwrl setup when ready; software/USB success is not robot acceptance
Report: /home/robot/real-world-rl/local/doctor/20261112T101500-3f2a9c1d
Result: no blocking findings; review SKIPPED/WARN coverage
```

每行格式为 `STATUS id: message`；非 `PASS` 行后附 `hint:`。`host.python` 必须是 `.venv` 的解释器；该行 `WARN` 表示项目环境未激活。尚无 `local/station.yaml`，所以 `station` 为 `SKIPPED`；第 02 页将创建它。`Report:` 指向本次 `report.json` 所在目录。没有失败时末行显示绿色。状态词说明见 [00](00-overview.md#status-words)。

不带 `--mock` 时，doctor 还检查主机：诊断工具、USB 设备、串口和输入权限、实时限制及各扩展的 Python 依赖。在 NUC 的仓库根目录，激活项目环境：

```bash
rwrl doctor
```

报告行因机器而异。完成第 02 页前，`station` 和 `coverage.*` 应为 `SKIPPED`；此时尚未验证硬件。

<a id="if-it-fails"></a>

## 失败怎么办

- **安装阶段错误：** 安装器打印 `ERROR: <stage> failed` 和从仓库根目录重试的完整命令。解决指定原因后重新运行安装器。
- **`rl-garden/` 为空**（克隆时未用 `--recurse-submodules`）：在 NUC 的仓库根目录，无需环境：

  <!-- tutorial-test: skip -->
  ```bash
  git submodule update --init
  ```

  然后重新运行安装器。`rl_garden==0.0.1` 从此子模块安装，而非公共索引。
- **`Unknown env backend 'real_robot'`：** 运行命令的解释器未安装本包；激活项目环境。
- **`franky` 导入错误：** 包名是 `franky-control`（自带 libfranka，无需 ROS），不是 `franka`。
- **doctor 红色行：** 阅读 `hint:`，再查看 [rwrl doctor：红灯处理](../doctor.md#red-lights-what-to-do) 对应章节，从 [Python 环境与扩展](../doctor.md#python-environment-and-extras)开始。

下一步：[02 设置站点](02-setup.md)。
