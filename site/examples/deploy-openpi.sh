# 本章命令分机、分环境执行；REPLACE_WITH_* 先替换。
# start inspect
CHECKPOINT=REPLACE_WITH_CHECKPOINT_DIRECTORY
rg --files --hidden --no-ignore "$CHECKPOINT"
rg -n 'pi05|norm|asset|RealworldInputs|RealworldOutputs|FrankaEEInputs|action|camera' "$CHECKPOINT" -g '*.json' -g '*.yaml' -g '*.py'
# end inspect
# start help
rwrl deploy-openpi --help
rwrl daemon --help
rwrl doctor --help
# end help
# start install
pip install -e ./rl-garden -e '.[learning,grpc,openpi-client,jpeg]'
# end install
# start serve
# GPU：在包含 RLinf 配置的 openpi checkout 及其环境中执行。
uv run scripts/serve_policy.py --port 8000 policy:checkpoint \
  --policy.config pi05_franka_pnp --policy.dir REPLACE_WITH_EXPORTED_CHECKPOINT
# end serve
# start daemon
# NUC：2 秒仅为 docs/deploy-openpi.md 的示例，须按现场测量修订。
rwrl daemon --station local/station.yaml --watchdog-s 2
# end daemon
# start tunnel
# GPU：operator@nuc 换成已验证的 NUC SSH 用户和主机。
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:50061:127.0.0.1:50051 operator@nuc
# end tunnel
# start doctor-nuc
# NUC：保留 station 才会检查 daemon.bind/listen、sshd、firewall。
rwrl doctor --station local/station.yaml --link 127.0.0.1:50051
# end doctor-nuc
# start doctor-gpu
# GPU：--ssh 会实际检查 SSH 并建立临时转发。
rwrl doctor --link 127.0.0.1:50061 --ssh operator@nuc \
  --remote-bind 127.0.0.1:50051 --policy 127.0.0.1:8000
# end doctor-gpu
# start doctor-empty
# 本章实测：没有服务，只探测回环地址，不指定远程 SSH。
rwrl doctor --link 127.0.0.1:50061 --policy 127.0.0.1:8000 \
  --report-dir /tmp/deploy-openpi-doctor-gpu
# end doctor-empty
# start dry-run
rwrl deploy-openpi --dry-run --profile rlinf_franka \
  --policy-host 127.0.0.1 --policy-port 8000 \
  --prompt "Pick up the red block and place it in the bin"
# end dry-run
# start mapping
# NUC：读出实际设备名、边界和 home。
rwrl --station local/station.yaml status
rwrl --station local/station.yaml workspace show
# end mapping
# start episode
# GPU：先按训练配置审阅这组示例值；输出同时保存在本地。
mkdir -p local
set -o pipefail
rwrl deploy-openpi --profile rlinf_franka \
  --prompt "Pick up the red block and place it in the bin" \
  --real-robot.remote 127.0.0.1:50061 --real-robot.control-hz 10 \
  --real-robot.xyz-scale-m 1.0 --real-robot.rot-scale-rad 1.0 \
  --camera-keys wrist_1 observation/image wrist_2 observation/extra_view_image \
  --episodes 1 --max-steps-per-episode 30 --replan-steps 5 \
  --reconnect-timeout-s 120 2>&1 | tee local/deploy-openpi.log
# end episode
# start log
rg '\[deploy-openpi\]' local/deploy-openpi.log
# end log
# start service
# GPU：先安装 autossh、完成 SSH 密钥登录，再替换服务文件内的 operator@nuc。
command -v autossh
mkdir -p ~/.config/systemd/user
cp site/examples/deploy-openpi.service ~/.config/systemd/user/rwrl-tunnel-50061.service
# 编辑刚复制的文件，核对 ExecStart 的 autossh 路径、SSH 目标和转发端口。
# 先结束占用 50061 的手动 ssh -N，再启用服务。
systemctl --user daemon-reload
systemctl --user enable --now rwrl-tunnel-50061.service
systemctl --user status rwrl-tunnel-50061.service
loginctl enable-linger "$USER"
# end service
# start proxy
# 按 link.proxy 的 hint 设置；若已有 no_grpc_proxy，也要同步补齐。
export no_proxy="${no_proxy:+$no_proxy,}127.0.0.1,localhost"
# end proxy
# start real-ip
# NUC：nuc 换成 doctor hint 中的真实 SSH 主机名。
getent ahostsv4 nuc
# end real-ip
