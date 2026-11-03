# Documentation Index

## Tutorial

One Franka arm with a gripper, a GELLO leader arm read through the robot daemon and the keyboard,
from an empty robot PC to a deployed pick-and-place policy. Read in order.

- [0. Overview and safety](tutorial/00-overview.md)
- [1. Install](tutorial/01-install.md)
- [2. Configure the station](tutorial/02-station.md)
- [3. Start the daemon and check](tutorial/03-daemon.md)
- [4. Record demos](tutorial/04-record-demos.md)
- [5. Base policy: BC or ACT](tutorial/05-base-policy.md)
- [6. Intervening: the operator page](tutorial/06-intervening.md)
- [7. HIL training](tutorial/07-hil-training.md)
- [8. Deploy](tutorial/08-deploy.md)

## Concepts

How the methods and the intervention path work.

- [Residual RL with human intervention (`residual_hil_serl`)](residual.md)
- [HIL-RL (SiLRI)](silri.md)
- [Teleoperation: sources, the intervention wrapper, adding a device](teleop.md)
- [Two arms as one env (`real_robot_bimanual`)](bimanual.md)
- [HIL-SERL roadmap: what was ported and how it differs](hil-serl-roadmap.md)

## Reference

Flags, file formats and hardware details.

- [Repository README](../README.md#running-on-hardware): action contract, safety limits, deploy, the
  demo contract sidecar, the end-to-end try-out with the fake robot
- [GELLO leader arm: hardware, calibration, robot yaml](gello.md)
- [Recording demos: recorder flags and the pkl format](recording-demos.md)
- [`examples/presets/hil_single_arm.yaml`](../examples/presets/hil_single_arm.yaml): the single-arm env preset
- [robot_infra: device layer, setup wizard, doctor, daemon](../robot_infra/README.md)
- [Getting started on hardware](getting-started-hardware.md) (moved into the tutorial; kept for old links)
