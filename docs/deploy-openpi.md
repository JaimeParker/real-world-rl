# Evaluate Pi0.5 with an openpi policy server

[中文](zh-CN/deploy-openpi.md)

Both the openpi websocket server and `rwrl deploy-openpi` run on the GPU host. The NUC runs
the torch-free `rwrl daemon` with franky. It owns the arm, Hand and cameras and enforces the
station's safety limits.

```text
GPU host                                NUC beside the Franka
  openpi serve_policy.py :8000
              ↕ websocket
  rwrl deploy-openpi (RealRobotEnv)      rwrl daemon (127.0.0.1:50051)
    └─ 127.0.0.1:50061 ── SSH/gRPC ───▶   │
                                          └─ franky arm + Hand, cameras
```

## Start the daemon, tunnel and policy server

On the NUC, use the device environment with `franky`, `grpc` and any camera extras your station
needs. Complete the [Franka bring-up](tutorial/03-franka-bringup.md), then serve the single-arm
station. The watchdog setting here is an example. See [timing](#chunk-execution-and-timing)
before using it on your station.

```bash
rwrl daemon --station local/station.yaml --bind 127.0.0.1:50051 --watchdog-s 2
```

On the GPU host, open the tunnel in its own terminal. Replace `operator@nuc` with your NUC SSH
login. The deploy loop will use the forwarded port `50061`.

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:50061:127.0.0.1:50051 operator@nuc
```

In the GPU host's **openpi environment**, serve an exported openpi checkpoint using the config
and normalization assets it was trained with. For the RLinf example, the server's openpi config
registry must include the RLinf `pi05_franka_pnp` config with `LeRobotRealworldDataConfig`,
`RealworldInputs` and `RealworldOutputs`. Use the `serve_policy.py` from that checkout. Stock
openpi's built-in checkpoint registry is for its own policy configs.

```bash
uv run scripts/serve_policy.py --port 8000 policy:checkpoint \
  --policy.config pi05_franka_pnp --policy.dir /path/to/exported-checkpoint
```

The official server listens on all interfaces and provides an unauthenticated websocket.
Restrict access to port 8000 at the GPU host. For a server or gateway configured to accept the
openpi client header, `--api-key` sends `Authorization: Api-Key <key>`.
[openpi server entrypoint](https://github.com/Physical-Intelligence/openpi/blob/main/scripts/serve_policy.py)
· [openpi websocket client](https://github.com/Physical-Intelligence/openpi/blob/main/packages/openpi-client/src/openpi_client/websocket_client_policy.py)

## Check and run the deployment

On the GPU host, in the **rwrl environment** and from the repository root, install the client
and env dependencies. The `jpeg` extra supplies OpenCV for the RLinf resize, Pillow for the
generic resize, and JPEG decoding.

```bash
pip install -e ./rl-garden -e '.[learning,grpc,openpi-client,jpeg]'
rwrl deploy-openpi --dry-run --prompt "Pick up the red block and place it in the bin"
```

Dry-run performs one inference on synthetic state and images and prints the returned chunk shape
and timing. It needs only the policy endpoint and can run before the daemon or tunnel is up.
Override `--policy-host` and `--policy-port` if the policy uses an endpoint other than
`127.0.0.1:8000`.

With the workcell ready, run a short evaluation. The scales below match the example RLinf
realworld SFT environment. Set the scales, home/reset pose, control rate, camera assignment and
station limits for your checkpoint and workcell before moving the robot.

```bash
rwrl deploy-openpi --profile rlinf_franka \
  --prompt "Pick up the red block and place it in the bin" \
  --real-robot.remote 127.0.0.1:50061 --real-robot.control-hz 10 \
  --real-robot.xyz-scale-m 1.0 --real-robot.rot-scale-rad 1.0 \
  --camera-keys wrist_1 observation/image wrist_2 observation/extra_view_image \
  --episodes 1 --max-steps-per-episode 30 --replan-steps 5
```

Supply `--real-robot.token` if the daemon requires a token. `--camera-keys` takes alternating
daemon camera names and policy keys and supplies the env's RGB camera list. `--image-size`
takes height then width and overrides the profile size.

## Profiles

- **`rlinf_franka` (default)** matches the 19-element `RealworldInputs` contract, such as
  `pi05_franka_pnp`. State order is gripper width (1, metres), TCP force (3), TCP pose (6,
  reset-relative XYZ and Euler `xyz` in radians), TCP torque (3) and TCP velocity (6,
  inverse-adjoint transformed into the end-effector frame). The Franky wrench is rotated
  from base coordinates into the stiffness frame. The default assumes its axes align with
  the TCP. Use `--stiffness-quat X Y Z W` for a calibrated stiffness rotation relative to the
  TCP. The default cameras are `wrist_1 → observation/image` and
  `wrist_2 → observation/extra_view_image`. Each frame is centre-cropped to a square and
  resized to 128 × 128 with OpenCV `INTER_LINEAR`, then sent as HWC uint8 BGR. The policy's
  model transform handles resizing to 224 × 224. To match a training crop, pass
  `--crop-regions wrist_1 0.1 0.2 0.9 0.8`. Each group is the daemon camera name followed by
  fractional top, left, bottom and right coordinates. The env uses the native resolution
  advertised by the daemon and truncates each edge to an integer pixel, as RLinf does.
  The daemon can crop and resize before transfer when it advertises OpenCV support. Otherwise
  the GPU host applies the same preprocessing to native-sized frames. See [camera frame
  preprocessing](daemon.md#camera-frame-preprocessing).
- **`rlinf_franka` actions** contain seven normalized values. The env maps the six arm
  values through RLinf's current-pose adjoint, applies `xyz_scale_m` and `rot_scale_rad`,
  and left-multiplies the measured orientation by the Euler increment. The daemon still
  enforces the station's workspace and per-step limits. The Franka Hand uses binary mode.
  After scaling by `gripper_action_scale`, values at or above +0.5 open it and values at or
  below −0.5 close it when its measured state calls for a change. The threshold is adjustable
  with `--real-robot.rlinf-gripper-threshold`. This path uses the daemon's blocking Hand
  command timing. CLI translation and rotation scales default to the env's 0.02 m and 0.1 rad.
- **`generic`** concatenates `--real-robot.state-fields` in the supplied order, using native
  env units and frames. `--state-key` and `--prompt-key` rename the inputs, and
  `--camera-keys` renames HWC uint8 RGB images. Pillow bilinear resize defaults to 224 × 224.
  The env interprets actions using its configured
  `action_mode`, scaling and gripper settings.

RLinf configs built with `FrankaEEInputs` use a separate policy contract. Check the checkpoint's
state layout and action semantics before selecting a profile by name.

## Chunk execution and timing

One synthetic warmup runs before the first robot reset. The live loop requests a new chunk after
the reset and executes up to `--replan-steps` actions per inference at `real_robot.control_hz`,
using the full horizon by default. Every episode reports mean daemon RTT, mean and max policy
round trip, mean server inference time when available, and arm health.

`--prefetch-steps N` overlaps one inference with the last `N` steps of a chunk, after at least
one action of that chunk has executed. It can reduce idle time between chunks. The next chunk
then starts from an observation that is up to `N` steps older than the latest feedback. Start
with `N=1` when testing this tradeoff. The first inference after each reset still uses the reset
observation and runs synchronously.

During an inference gap the loop waits for the next chunk, and the daemon's watchdog remains
responsible for idling the arm if actions stop. Its default is **0.5 s**. Set the NUC's
`--watchdog-s` above the worst measured gap from entering servo or sending an action to the next
action reaching the daemon. Allow for camera transfer, policy round trip, tunnel jitter,
control period and gripper motion. Apply the same budget to `real_robot.watchdog_s` if you
enable that additional env watchdog. Choose the smallest timeout that covers the observed
normal gap with margin and remains safe for the workcell. If the daemon trips, the loop exits
on the device error. Keep the physical stop available during commissioning.
