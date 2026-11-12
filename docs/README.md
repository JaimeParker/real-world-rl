# Documentation Index

[中文文档入口](README.zh-CN.md) — 全部文档的完整中文译文、阅读顺序与操作速查。

## Tutorial

One Franka arm with a Franka Hand, a NUC and a GELLO leader arm, from an empty NUC to a deployed
pick-and-place policy. The recorder and the HIL actor run directly on the NUC; every step is an
`rwrl` command. Read in order.

- [0. Overview and safety](tutorial/00-overview.md)
- [1. Install](tutorial/01-install.md)
- [2. Set up the station](tutorial/02-setup.md)
- [3. Bring up the Franka](tutorial/03-franka-bringup.md)
- [4. Home, gripper, cameras](tutorial/04-first-motion.md)
- [5. Teleop: GELLO](tutorial/05-teleop-gello.md): calibrate, read back and run a supervised cartesian session
- [6. Teleop: SpaceMouse](tutorial/06-teleop-spacemouse.md): HID checks, direction profiles, task frame and supervised sessions
- [7. Teleop: leader-follower](tutorial/07-teleop-leader-follower.md): joint-space onset and cartesian leaders, including a second arm (needs hardware verification)
- [8. Record demos](tutorial/08-record-demos.md)
- [9. Base policy: BC or ACT](tutorial/09-base-policy.md)
- [10. HIL training](tutorial/10-hil-training.md)
- [11. Deploy](tutorial/11-deploy.md)

Beside the robot while recording and training: the
[operator page](intervening.md). An actor on another machine:
[remote actor over the robot daemon](remote-actor.md).

## Reference

- [rwrl: station command line](cli.md): global options, every command, exit codes, JSON
- [rwrl doctor](doctor.md): checks, statuses, repairs, reports and what to do on a red row
- [Recording demos: recorder flags and the pkl format](recording-demos.md)
- [GELLO leader arm: hardware, calibration, robot yaml](gello.md)
- [SpaceMouse commissioning: profiles, task-frame workspace, sessions](spacemouse-commissioning.md)
- [Device layer: station files, backends, safety rules](devices.md)
- [Robot daemon: serving, RemoteRobot and supervision](daemon.md)
- [Arm control profiles](control-profiles.md): station gains, backend capabilities and
  effective settings
- [`examples/presets/hil_single_arm.yaml`](../examples/presets/hil_single_arm.yaml): the
  single-arm env preset
- [Repository README](../README.md#running-on-hardware): action contract, safety limits, deploy,
  the demo contract sidecar, the end-to-end try-out with the fake robot
- [Real-world experiments](real-world-experiments.md): durable uploads, recovery, outcomes,
  partial takeover, trajectory archives, tasks and observations
- [Known issues and deferred work](known-issues.md): what a first real run can hit, workarounds,
  planned fixes

## Concepts

How the methods and the intervention path work.

- [Residual RL with human intervention (`residual_hil_serl`)](residual.md)
- [HIL-RL (SiLRI)](silri.md)
- [Teleoperation: sources, the intervention wrapper, adding a device](teleop.md)
- [Two arms as one env (`real_robot_bimanual`)](bimanual.md)
- [HIL-SERL roadmap: what was ported and how it differs](hil-serl-roadmap.md)

## Hardware test cases

- [Hardware session checklists](testing/README.md): English primary checklists,
  record CSV conventions and evidence requirements. Hardware results are filled in on site.

The GitHub Wiki is generated from these repository files by a GitHub Action; edit the files in
the repository directly.
