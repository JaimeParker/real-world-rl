# 从 π0.5 checkpoint 到策略服务

部署程序同时连接两端：它通过机器人 daemon 取得相机和状态，通过 WebSocket 把观测与任务指令发给 GPU 策略服务，再把返回的动作交给环境。先明确手里的权重由哪个模型实现加载。

## 用官方 pi05_base 理解转换方向

官方 JAX / Orbax 权重目录包含 `params/`。目标是在 RLinf 中微调或评估时，进入 **RLinf** 的 OpenPI 环境，准备任务数据的归一化统计，然后运行下列转换；路径变量应指向本机实际文件。

```bash
python -m rlinf.utils.ckpt_convertor.openpi.convert --mode jax_to_openpi \
    --input-model "$JAX_CKPT" --input-norm-stats "$TASK_STATS" \
    --output-model "$OUTPUT_MODEL" --output-norm-stats "$OUTPUT_MODEL/task/norm_stats.json"
```

按目标训练配置核对 action horizon、action dim、token 长度和统计文件路径，必要时显式传给转换器。生成的目录含权重与模型配置。转换器复制传入的统计量，统计量仍对应原来的任务数据。已有官方 PyTorch 布局则查看 `openpi_pytorch_to_openpi`。要用官方服务加载它原本支持的官方权重，可以直接选择匹配的官方配置启动服务。检验转换时，用同一权重转换前后的相同观测和采样噪声比较输出。[RLinf 权重准备指南](https://rlinf.readthedocs.io/en/release-v0.4/rst_source/examples/embodied/sft_openpi.html) · [官方策略服务](https://github.com/Physical-Intelligence/openpi/blob/main/scripts/serve_policy.py)

## 用 Pick_Red 判断一份微调权重能否接入工作站

在 GPU 服务器获取 RLinf 发布的模型：

```bash
hf download RLinf/RLinf-Pi05-Pick_Red --revision f2bfb811409c19cd1d3090ec83539537c59ec63f --local-dir models/Pick_Red
```

RLinf 的真机配置名为 `pi05_franka_pnp`，动作维度为 7，预测长度为 5。先按原配置加载权重、变换和统计量。原始评估使用 RLinf 的运行环境。接入本仓还需要一个实现 openpi WebSocket 协议的策略服务；核实该服务确实载入选定模型，能接收约定的观测键并返回 actions。原始 Ray 评估入口与 WebSocket 监听地址是两个不同的接口。[模型文件](https://huggingface.co/RLinf/RLinf-Pi05-Pick_Red) · [原始评估配置](https://github.com/RLinf/RLinf/blob/main/evaluations/realworld/realworld_pnp_eval_pi05_sft_RTC.yaml)

已核实此版本含 `model.safetensors`、`metadata.pt` 和 `assets/RLinf/pick_red/norm_stats.json`。权重头含 `paligemma_with_expert.*` 参数键，属于上游 PyTorch 布局。使用 RLinf 的 JAX 对齐实现时选 `openpi_pytorch_to_openpi`；选择能读取此布局的策略服务时，保留对应的输入输出变换和原始统计量。该模型输入变换读取 19 维状态，网络内部的动作维度为 32，输出变换取前 7 维给环境。不要用网络内部维度配置机械臂。

在**已注册** `pi05_franka_pnp` 及其 `RealworldInputs/RealworldOutputs` 的 openpi 环境中，可按该环境的服务入口启动：

```bash
uv run scripts/serve_policy.py --port 8000 policy:checkpoint \
    --policy.config=pi05_franka_pnp --policy.dir="$PWD/models/Pick_Red"
```

stock openpi 的配置表不含这个 RLinf 配置。若服务提示找不到配置，先核对模型端的配置注册和变换实现；替换成 LIBERO 配置会改变策略看到的状态和相机布局。服务启动后，在部署机先发一条合成观测。下面命令需要已经运行的兼容策略服务：

```bash
rwrl deploy-openpi --dry-run --policy-host "$POLICY_HOST" --policy-port 8000 \
    --prompt "Pick up the red block and place it in the bin"
```

成功返回有限的二维动作数组，说明网络、消息格式和动作宽度通过初验。再核对主相机和副相机的视角与颜色顺序、状态相对哪一个复位姿态表示、旋转参考系和夹爪符号。`rlinf_franka` 提供专门映射，`generic` 按配置的状态字段顺序展平并原样传递动作。两者都需要与模型训练配置对应。实机连接使用 `--real-robot.remote` 指向 daemon，`--replan-steps` 控制一段预测执行几步后再次请求。完整参数见[远程推理说明](https://github.com/JaimeParker/real-world-rl/blob/main/docs/zh-CN/deploy-openpi.md)。

Pick_Red 在本仓通过 WebSocket 的实际权重推理仍待验证。双臂 [Clean Desk 模型](https://huggingface.co/RLinf/RLinf-Pi05-RealWorld-DualFranka-Clean_Desk) 可用于对照：它还有左右臂顺序、双臂相机布局和 20 步动作块要匹配。
