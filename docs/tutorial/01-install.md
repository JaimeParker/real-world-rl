# 1. Install

## Hardware checklist

- One Franka Panda with the Franka Hand, and the FCI feature activated in Desk.
- A Linux robot PC wired by Ethernet to the arm's Control box (not the arm base).
- One Intel RealSense camera on USB 3 at the wrist; the wizard names it `wrist`. Extra cameras are
  named `front`, `front_2`, ...
- One GELLO with its USB bus adapter (ROBOTIS U2D2 or OpenRB-150) and powered servos. A U2D2 does not
  power the Dynamixel bus from USB ([gello.md](../gello.md#1-hardware) has the hardware details,
  including the FTDI latency timer).
- A GPU machine for the learner and for BC / ACT training.
- A keyboard on the actor machine (the intervention toggle and the episode keys).
- The task: one object to pick and one bin to place it in, inside the workspace box you will set in
  [02](02-station.md#check-the-safety-limits).

## Network

The arm's FCI address is set in Desk (the default is `172.16.0.2`). Give the robot PC an address in
the same subnet on the interface wired to the Control box (this repository does not configure it).
Activate FCI in Desk before you start; the doctor cannot do it for you. The actor must reach the
daemon's gRPC port on the robot PC (50051 by default) and the learner's sync port (6000 in
[07](07-hil-training.md)).

## Linux permissions

- GELLO serial port: the user needs the `dialout` group (`sudo usermod -aG dialout $USER`, then log
  in again) or a udev rule.
- RealSense: install the librealsense udev rules (`scripts/setup_udev_rules.sh` in the librealsense
  sources) and use USB 3 ports and cables.
- Real-time kernel: optional. The default `realtime: ignore` works without a PREEMPT_RT kernel and the
  doctor then reports `control jitter possible`; `realtime: enforce` needs a PREEMPT_RT kernel (see
  the [robot_infra troubleshooting table](../../robot_infra/README.md#troubleshooting)).
- The keyboard route of this tutorial needs no `input` group: `pynput` reads the actor machine's
  desktop session. (An evdev pedal or keyboard on the robot PC does need it, see
  [gello.md](../gello.md#6-hil-training).)

**Docker.** If the daemon runs in a container it needs the USB nodes passed in before it starts
(`--device=/dev/serial/by-id/...` for the GELLO, the camera devices), network access to the robot
(the FCI link is UDP) and its gRPC port reachable from the actor (`--network host`, or publish the
port). Containers are not covered by this repository's tests; the robot_infra README lists udev and
Docker as
[unverified](../../robot_infra/README.md#known-limitations-and-needs-hardware-verification).

## Install

Clone with the rl-garden submodule, then install by role. The extras come from
`robot_infra/pyproject.toml` and `pyproject.toml`:

<!-- tutorial-test: skip -->
```bash
git clone --recurse-submodules https://github.com/JaimeParker/real-world-rl.git
cd real-world-rl

# Robot PC: wizard, doctor, discovery and the daemon.
pip install -e 'robot_infra[franky,grpc,realsense,gello]'

# GPU machine and actor machine: all three distributions (grpc is the client side of the daemon).
pip install -e rl-garden -e 'robot_infra[grpc]' -e .

# Actor machine: pynput for --teleop-event-device keyboard (needs that machine's desktop session).
pip install -e '.[reward-collection]'
```

| Extra | Package | Needed for |
|---|---|---|
| `robot_infra[franky]` | `franky-control` | the Franka arm and Hand (other robot firmware: install the matching wheel, see the [Franka troubleshooting row](../../robot_infra/README.md#troubleshooting)) |
| `robot_infra[realsense]` | `pyrealsense2` | RealSense cameras |
| `robot_infra[gello]` | `dynamixel-sdk` | the GELLO backend and its calibration |
| `robot_infra[grpc]` | `grpcio` | the daemon and every client of it |
| `reward-collection` | `pynput` | `--teleop-event-device keyboard` |

On one machine that does everything:

<!-- tutorial-test: skip -->
```bash
pip install -e rl-garden -e 'robot_infra[franky,grpc,realsense,gello]' -e '.[reward-collection]'
```

With the package installed, `real-world-rl-train` (or `python -m real_world_rl.training`) is
the same as `python examples/train_real_world.py`, `real-world-rl-record` the same as
`python examples/record_real_demos.py` and `real-world-rl-deploy` the same as
`python examples/deploy_real_robot.py`. The tutorial writes the `examples/` form.

*You should see* the install finish without errors. Check it without hardware with the fake-backed
test suite (needs pytest, the `dev` extra):

<!-- tutorial-test: skip -->
```bash
python -m pytest tests robot_infra/tests -q
```

*If not:* `Unknown env backend 'real_robot'` later means this package (`-e .`) is not installed in the
environment you run from.

Next: [02 Station](02-station.md).
