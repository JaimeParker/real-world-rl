# 安装与初验

先确定谁打开设备，谁使用观测。一台上位机可以直接运行采集程序，也可以运行 daemon，把设备提供给本机或远端客户端。训练服务器只运行 learner 时，只需读取数据和模型。GPU 策略服务又是另一个角色，它返回预测动作。

在需要运行本仓程序的机器上获取同一版代码。记录两个提交，方便以后重现。

```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl
git rev-parse HEAD
git submodule status
```

| 本机用途 | 从仓库根目录执行 |
|---|---|
| 仅运行 Franka 设备服务 | `bash scripts/install.sh --daemon-only` 安装 franky 和 gRPC |
| 采集、推理客户端，通过网络连设备服务 | `bash scripts/install.sh --extras learning,grpc` |
| 离线 BC 或 learner | `bash scripts/install.sh --extras learning` |
| 上位机兼任 Franka 设备端和采集端 | 例如 `bash scripts/install.sh --extras learning,franky,grpc,spacemouse`；按下表替换或补充实际设备 |

脚本默认创建 Python 3.10 的 `.venv`。用 `--python` 可以指定解释器。上位机负责读取哪些外设，就在那台机器的环境中安装对应 extra。设备端需要自选组合时，用 `--extras franky,grpc,...` 替代 `--daemon-only`。Franka SDK 版本需与控制柜版本匹配，接入后按 doctor 报告核实。

| 遥操作与输入 | 安装项和第一次检查 |
|---|---|
| SpaceMouse | `spacemouse`，检查 HID 权限和六轴读数 |
| GELLO | `gello`，检查 Dynamixel 串口；实体键盘或踏板通过 Linux 输入设备参与时加 `gello-events` |
| Pico | `pico`，确认 VR 端发布的 ZeroMQ 地址和手柄选择 |
| 实体主臂 | 按主臂型号安装驱动；读取远端主臂再加 `grpc`。选择关节跟随或笛卡尔映射 |
| 标注输入 | 桌面全局键盘使用 `reward-collection` 中的 pynput；终端 `stdin` 适合 SSH；JSONL 可提供脚本事件 |

使用 RealSense 时添加 `realsense`，ZED 的 UVC 彩色路径添加 `zed-uvc`，ZED SDK 路径按厂商 SDK 安装 Python 绑定。模型客户端连接 openpi 时添加 `openpi-client`。

未提供 station 时，脚本用临时模拟工作站运行 doctor。看到导入成功和 mock 检查通过，就完成了软件初验。打开新终端后执行：

```bash
source .venv/bin/activate
rwrl --help
```

到现场再检查真实网络、设备权限、状态和工作区。继续阅读[遥操作和真机数据采集](collect.md)。
