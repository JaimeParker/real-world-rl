<!-- English source: docs/deploy-openpi.md -->

<a id="evaluate-pi05-with-an-openpi-policy-server"></a>

# 通过 openpi 策略服务评估 Pi0.5

[中文入口](../README.zh-CN.md) · [English](../deploy-openpi.md)

openpi WebSocket 服务和 `rwrl deploy-openpi` 均运行在 GPU 主机。NUC 只运行无需 torch
的 `rwrl daemon`，通过 franky 控制机械臂和夹爪，采集相机画面，并执行工作站的安全限制。

```text
GPU 主机                                 Franka 旁的 NUC
  openpi serve_policy.py :8000
              ↕ WebSocket
  rwrl deploy-openpi (RealRobotEnv)      rwrl daemon (127.0.0.1:50051)
    └─ 127.0.0.1:50061 ── SSH/gRPC ───▶   │
                                          └─ franky 机械臂和夹爪、相机
```

<a id="start-the-daemon-tunnel-and-policy-server"></a>

## 启动守护进程、隧道和策略服务

在 NUC 上，使用装有 `franky`、`grpc` 和所需相机扩展的设备环境。完成
[Franka 启动与检查](tutorial/03-franka-bringup.md)后，为单臂工作站启动守护进程。
命令中的超时值仅为示例。配置真机前请参阅[分块执行与时序](#chunk-execution-and-timing)。

```bash
rwrl daemon --station local/station.yaml --bind 127.0.0.1:50051 --watchdog-s 2
```

在 GPU 主机的独立终端开启隧道。将 `operator@nuc` 换成 NUC 的 SSH 登录地址，
部署循环将连接转发端口 `50061`。

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:50061:127.0.0.1:50051 operator@nuc
```

在 GPU 主机的 **openpi 环境**中，用训练时的配置和归一化资产加载已导出的 openpi 检查点。
运行下方 RLinf 示例时，服务端 openpi 的配置注册表必须包含 RLinf 的 `pi05_franka_pnp`
配置及其 `LeRobotRealworldDataConfig`、`RealworldInputs` 和 `RealworldOutputs`。
使用该代码版本的 `serve_policy.py`。原版 openpi 自带其官方策略的检查点配置。

```bash
uv run scripts/serve_policy.py --port 8000 policy:checkpoint \
  --policy.config pi05_franka_pnp --policy.dir /path/to/exported-checkpoint
```

官方服务监听所有网络接口，WebSocket 无需认证。请在 GPU 主机限制对 8000 端口的访问。
若服务端或网关支持 openpi 客户端的请求头，`--api-key` 会发送 `Authorization: Api-Key <key>`。
[openpi 服务入口](https://github.com/Physical-Intelligence/openpi/blob/main/scripts/serve_policy.py)
· [openpi WebSocket 客户端](https://github.com/Physical-Intelligence/openpi/blob/main/packages/openpi-client/src/openpi_client/websocket_client_policy.py)

<a id="check-and-run-the-deployment"></a>

## 检查策略并运行部署

在 GPU 主机的 **rwrl 环境**中，从仓库根目录安装客户端和环境依赖。`jpeg` 扩展提供
RLinf 缩放所需的 OpenCV、通用档案缩放所需的 Pillow，以及 JPEG 解码支持。

```bash
pip install -e ./rl-garden -e '.[learning,grpc,openpi-client,jpeg]'
rwrl deploy-openpi --dry-run --prompt "Pick up the red block and place it in the bin"
```

试运行用合成状态和图像执行一次推理，输出返回的动作块形状和耗时。只需策略服务已启动，
可以在守护进程和隧道启动前执行。若策略服务地址并非 `127.0.0.1:8000`，用
`--policy-host` 和 `--policy-port` 指定地址。

工作站就绪后可短程评估。下方缩放值与 RLinf 的真机 SFT 示例环境一致。真机运动前，
请按检查点和工作站配置动作缩放、归位与复位姿态、控制频率、相机映射和安全限制。

```bash
rwrl deploy-openpi --profile rlinf_franka \
  --prompt "Pick up the red block and place it in the bin" \
  --real-robot.remote 127.0.0.1:50061 --real-robot.control-hz 10 \
  --real-robot.xyz-scale-m 1.0 --real-robot.rot-scale-rad 1.0 \
  --camera-keys wrist_1 observation/image wrist_2 observation/extra_view_image \
  --episodes 1 --max-steps-per-episode 30 --replan-steps 5
```

若守护进程要求令牌，请传入 `--real-robot.token`。`--camera-keys` 交替接收守护进程
相机名和策略输入键，并确定环境采集哪些彩色相机。`--image-size` 按高、宽顺序指定图像
尺寸，可覆盖所选档案的默认值。

<a id="profiles"></a>

## 观测与动作档案

- **`rlinf_franka`（默认）**匹配 `pi05_franka_pnp` 等使用 `RealworldInputs` 的
  19 维输入。顺序依次为夹爪开口宽度（1 维，米）、TCP 力（3 维）、TCP 位姿（6 维，
  相对于复位位姿的 XYZ 和弧度制 `xyz` 欧拉角）、TCP 力矩（3 维）、TCP 速度（6 维，
  经逆伴随变换到末端坐标系）。Franky 的力和力矩从基座坐标系旋转到刚度坐标系。
  默认刚度坐标系与 TCP 轴对齐。若有标定旋转，可用 `--stiffness-quat X Y Z W`
  指定其相对于 TCP 的四元数。默认将 `wrist_1` 映射到 `observation/image`，
  将 `wrist_2` 映射到 `observation/extra_view_image`。每帧默认从中心裁成正方形，
  以 OpenCV `INTER_LINEAR` 插值缩放至 128 × 128，再以 HWC、uint8、BGR 格式发送。
  之后由策略的模型变换将图像缩放至 224 × 224。可用
  `--crop-regions wrist_1 0.1 0.2 0.9 0.8` 匹配训练时的裁切区域。每组参数以守护进程的
  相机名开头，后跟上、左、下、右四条边的比例坐标。环境使用守护进程声明的原始分辨率，
  按 RLinf 的规则将各边截断为整数像素。守护进程声明支持 OpenCV 时可在传输前裁切和
  缩放，否则由 GPU 主机对原始尺寸帧执行相同的预处理。见[相机帧预处理](daemon.md#camera-frame-preprocessing)。
- **`rlinf_franka` 动作**包含七个归一化值。环境用当前位姿的伴随矩阵转换前六维，
  应用 `xyz_scale_m` 和 `rot_scale_rad`，再用欧拉角增量对应的旋转左乘实测姿态。
  守护进程执行工作站的工作空间和单步增量限制。Franka Hand 使用二值模式。乘以
  `gripper_action_scale` 后，动作值达到 +0.5 时按实测状态张开夹爪，达到 −0.5
  或更低时闭合。`--real-robot.rlinf-gripper-threshold` 可调整阈值。时序由守护
  进程的阻塞式夹爪命令决定。CLI 的平移和旋转缩放默认沿用环境值 0.02 米和 0.1 弧度。
- **`generic`**按 `--real-robot.state-fields` 的顺序拼接状态，保留环境原有的单位
  和坐标系。`--state-key`、`--prompt-key` 可重命名输入键，`--camera-keys` 可设置
  HWC、uint8、RGB 图像的策略输入键。默认用 Pillow 双线性插值缩放至 224 × 224。
  动作由环境按所配置的 `action_mode`、缩放和
  夹爪设置解释。

使用 `FrankaEEInputs` 的 RLinf 配置拥有独立的策略约定。选择档案时请检查对应检查点
的状态布局和动作语义。

<a id="chunk-execution-and-timing"></a>

## 分块执行与时序

首次复位机器人之前，用合成观测预热一次推理。复位后请求第一个实时动作块，
以 `real_robot.control_hz` 的频率每次执行至多 `--replan-steps` 步。默认执行整个
动作块。每轮回合报告守护进程平均 RTT、策略往返的平均和最大耗时、服务端平均推理耗时
（若返回该数据）以及机械臂健康状态。

`--prefetch-steps N` 将一次推理与当前动作块最后 `N` 步重叠，至少先执行该块第一步。
这可以缩短块间等待时间，但下一块动作所依赖的观测可能比最新反馈早最多 `N` 步。
可从 `N=1` 开始评估此取舍。每次复位后的首个推理仍同步使用复位观测。

推理期间，循环等待下一个动作块。动作中断时仍由 NUC 守护进程的看门狗让机械臂进入
空闲模式，默认超时 **0.5 秒**。在 NUC 上设置 `--watchdog-s`，使其高于从进入伺服
或发送一个动作起，到下一动作送达守护进程为止的实测最长间隔。要考虑相机传输、
策略往返、隧道抖动、控制周期及夹爪动作。启用 `real_robot.watchdog_s` 时，同样按此
间隔配置环境看门狗。选择能覆盖正常间隔并留有余量、且适合该工作站的最短超时值。
若守护进程的看门狗触发，部署循环会因设备错误退出。真机调试时确保可使用物理急停。
