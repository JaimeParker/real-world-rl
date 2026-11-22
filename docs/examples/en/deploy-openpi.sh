# Run this chapter's commands on the indicated machine and in the indicated environment; replace REPLACE_WITH_* first.
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
# GPU: run in the openpi checkout containing the RLinf configuration and its environment.
uv run scripts/serve_policy.py --port 8000 policy:checkpoint \
  --policy.config pi05_franka_pnp --policy.dir REPLACE_WITH_EXPORTED_CHECKPOINT
# end serve
# start daemon
# NUC: 2 seconds is only the example in the deploy-openpi reference page; revise it using on-site measurements.
rwrl daemon --watchdog-s 2
# end daemon
# start tunnel
# GPU: replace operator@nuc with the verified NUC SSH user and host.
ssh -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:50061:127.0.0.1:50051 operator@nuc
# end tunnel
# start doctor-nuc
# NUC: keep station to check daemon.bind/listen, sshd, firewall.
rwrl doctor --link 127.0.0.1:50051
# end doctor-nuc
# start doctor-gpu
# GPU: --ssh actually checks SSH and establishes a temporary forward.
rwrl doctor --link 127.0.0.1:50061 --ssh operator@nuc \
  --remote-bind 127.0.0.1:50051 --policy 127.0.0.1:8000
# end doctor-gpu
# start doctor-empty
# Measured for this chapter: no service, probe only loopback addresses, without remote SSH.
rwrl doctor --link 127.0.0.1:50061 --policy 127.0.0.1:8000 \
  --report-dir /tmp/deploy-openpi-doctor-gpu
# end doctor-empty
# start dry-run
rwrl deploy-openpi --dry-run --profile rlinf_franka \
  --policy-host 127.0.0.1 --policy-port 8000 \
  --prompt "Pick up the red block and place it in the bin"
# end dry-run
# start mapping
# NUC: read actual device names, boundaries, and home.
rwrl status
rwrl workspace show
# end mapping
# start episode
# GPU: review these example values against the training configuration first; output is also saved locally.
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
# GPU: install autossh and complete SSH key login first, then replace operator@nuc in the service file.
command -v autossh
mkdir -p ~/.config/systemd/user
cp docs/examples/deploy-openpi.service ~/.config/systemd/user/rwrl-tunnel-50061.service
# Edit the file just copied; check the autossh path, SSH target, and forwarding ports in ExecStart.
# Stop the manual ssh -N using 50061 before enabling the service.
systemctl --user daemon-reload
systemctl --user enable --now rwrl-tunnel-50061.service
systemctl --user status rwrl-tunnel-50061.service
loginctl enable-linger "$USER"
# end service
# start proxy
# Set according to the link.proxy hint; if no_grpc_proxy already exists, update it too.
export no_proxy="${no_proxy:+$no_proxy,}127.0.0.1,localhost"
# end proxy
# start real-ip
# NUC: replace nuc with the real SSH hostname from the doctor hint.
getent ahostsv4 nuc
# end real-ip
