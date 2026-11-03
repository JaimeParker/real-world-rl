# 3. Start the daemon and check

The robot daemon runs on the robot PC and serves the arm, the gripper, the camera and the GELLO over
gRPC. Every other process (recorder, actor, eval, deploy) is a client of it.

## Check the station

With nothing serving the robot yet:

```bash
python -m robot_infra.discovery     # what hardware is visible (never connects to a robot)
python -m robot_infra.doctor        # the five-layer check of the station's robot
```

*You should see* for discovery: the camera serials, the GELLO port, and
`172.16.0.2: ping=ok open ports=<list>` (for Franka, port 1337 open usually means FCI is reachable).
With nothing attached the sections read `none`
([sample output](../../robot_infra/README.md#2-plug-in-and-discover)). For the doctor: one line per
check, `[1/5] sdk:arm ... OK  ...` through `[5/5] mode:...`, then `summary: N ok, 0 failed, S skipped`;
the exit code is 1 if anything failed. The doctor connects to the robot and disconnects it.
`--fix` runs a failed check's autofix, `--json` prints machine output:

```bash
python -m robot_infra.doctor --fix
```

## Serve

With no `--config` the daemon uses the station file:

```bash
python -m robot_infra.server --watchdog-s 5
```

`--watchdog-s 5` matters for training: the daemon idles an arm that gets no action for the watchdog
time (default 0.5 s), and the first policy step after a reset can take longer. *You should see* the
same doctor lines first (the daemon's preflight), then the robot's parts and
`serving franka 'arm' on 0.0.0.0:50051`. The robot connects when the first client does. Leave the
terminal open; Ctrl-C stops the daemon without powering the robot off.

Do not run `python -m robot_infra.doctor` while the daemon serves the same robot: the doctor takes the
same device claim and reports a failed `handshake`.

## If a check fails: the three common failures

Each failing line is followed by `hint: ...`; follow it. The most common:

1. **GELLO serial permission.**
   `[2/5] os:serial ... FAIL  no read/write access to /dev/serial/by-id/...` with the hint to add the
   user to the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again) or a udev rule.
   `serial port ... not found` means a cable or the wrong port: list `/dev/serial/by-id/`.
2. **Franka not reachable.** `os:ping` FAIL and `handshake ... FAIL  cannot reach Franka at <ip>: ...`
   with the hint `check: ping <ip>; FCI activated in Desk; cable on the Control (not Arm) port;
   'UDP receive: Timeout' means a firewall`. Activate FCI in Desk, plug into the Control box, allow
   UDP from the robot through the firewall, and check the robot IP in the yaml.
3. **Camera serial ambiguous or missing.** `os:camera:<name> ... FAIL  serial unset and N devices
   attached; set serial to one of [...]`, or `camera '<name>' serial '<s>' not found`. The wizard
   writes serials, so this appears after a hand edit or a re-plugged camera: copy the serial from
   `python -m robot_infra.discovery` into `cameras[].serial`. `no frames within 2s` means USB 3
   bandwidth or udev rules; try `--fix`.

The full table, including libfranka version mismatch, Reflex mode and the real-time kernel, is the
[robot_infra troubleshooting table](../../robot_infra/README.md#troubleshooting).

## Dry run without hardware

The doctor can swap every backend for its fake, which checks the station file itself:

```bash
python -m robot_infra.doctor --config local/station.yaml --mock
```

The README's
[End-to-end try-out with the fake robot](../../README.md#end-to-end-try-out-with-the-fake-robot)
runs a daemon on fake backends, a learner, an actor and a deploy of the checkpoint on one machine.
Run it once so the commands of the next pages are familiar. `--teleop-device fake` instead of
`--teleop-device leader_arm` gives an actor without an operator (a scripted source that never
intervenes, [teleop.md](../teleop.md#devices)). `tests/test_single_arm_station_smoke.py` runs the
single-arm workflow of this tutorial on fakes:

<!-- tutorial-test: skip -->
```bash
python -m pytest tests/test_single_arm_station_smoke.py -q
```

Next: [04 Record demos](04-record-demos.md) (read [06 Intervening](06-intervening.md) first).
