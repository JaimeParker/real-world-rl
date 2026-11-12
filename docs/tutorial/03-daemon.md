# 3. Process layout and optional daemon

The main workflow runs the recorder or HIL actor on the NUC with a local
`real_robot.robot` mapping and `arm_backend: franky`. That process opens the arm,
Hand, cameras and GELLO directly. **Do not start a daemon for this route.**
A learner on a GPU machine does not by itself require a robot daemon: the actor
can remain on the NUC and send training data to the learner.

Only use the daemon route below when the **actor itself runs on another
machine**, using `real_robot.remote` instead of `real_robot.robot`.

| Property | Direct on the NUC (main workflow) | Remote actor (optional) |
|---|---|---|
| Hardware processes, one arm | one recorder or actor process | NUC daemon plus recorder or actor on the other machine |
| Connection config | `real_robot.robot` mapping | `real_robot.remote: NUC_ADDRESS:50051`; omit `robot` |
| Operator keyboard | attached to the NUC; target terminal keys in [00](00-overview.md#operator-keys) | read on the actor machine; current `pynput` route needs its desktop session |
| Action watchdog | application checks share the hardware process | daemon can stop the arm when client actions stop arriving |
| External stop | supervisor can kill a stuck process; firmware then handles lost control communication | a live daemon can issue stop independently of the actor |

An application thread cannot guarantee `stop()` if its whole process hangs or
is killed. A supervisor killing the direct process relies on the firmware's
communication-loss response; it is not an independent application stop command.
The daemon watchdog also cannot cover a hung daemon or failed NUC. Keep the
physical emergency stop available in both layouts.

## Start a daemon only for a remote actor

First finish the [real doctor handshake](01-install.md#5-configure-once-then-run-doctor).
Run every command in this section on the **NUC**, from the **repository root**,
with the **project environment active**. Ensure no other process owns these devices.
The remote route needs `robot_infra[grpc]` installed on both NUC and actor.

```bash
robot-infra daemon --station local/station.yaml --watchdog-s 5
```

`--watchdog-s` defaults to **0.5 seconds**. The explicit **5 seconds** above allows
for slow first inference after reset: if the interval exceeds the watchdog, the
daemon idles the arm and the next action fails. Choose the timeout for the cell;
5 seconds is an example, not a guarantee that a long pause is safe.

*You should see:* preflight doctor checks followed by the serving address and
robot parts. Leave the terminal open. Ctrl-C stops the daemon. A multi-arm station
starts one daemon process per configured robot, on separate ports; the launcher
can start both from the station file.

*If not:* follow the named preflight hint in [01](01-install.md). Do not run doctor
against the same devices while a recorder, actor or daemon owns them: the device
claim and handshake will conflict. Doctor lists such a process as `BLOCKED`; only
commands holding a hardware connection count, so offline `rwrl` commands such as
`workspace show`, `teleop inspect-stop` or `archive inspect` do not block it.

To check the connection after starting the daemon, open another terminal on
this **NUC**, at the **repository root**, with the **project environment active**:

<!-- tutorial-test: skip -->
```bash
rwrl status
```

With `daemon:` declared in `local/station.yaml`, expect
`connection: daemon HOST:PORT`. On the main route, with no daemon declaration
and no recorder/actor holding the arm, the same command reports
`connection: direct franky`. A declared but unreachable daemon is an error;
see [rwrl connection ownership](../cli.md#connection-ownership).

## Point the remote actor at the NUC

On the **actor machine**, use the project environment, from its repository root.
In its `hil.yaml`, remove `real_robot.robot` and set `real_robot.remote` to the
NUC's reachable address and the station's daemon port (normally 50051). Set
`real_robot.token` only if `daemon.token` is configured. With cameras, provide
`obs.image_size` because remote sizes are only known after connecting. Keep the
same action mode, 50 Hz frequency and observation contract as recording.

Install the gRPC client in the actor's project environment:

<!-- tutorial-test: skip -->
```bash
python -m pip install -e 'robot_infra[grpc]'
```

Check that configuration without connecting, on the actor machine, repository
root, project environment active:

```bash
rwrl train hil_serl --config hil.yaml --print-config
```

*You should see:* exactly one connection source (`remote`), the correct camera
names, action mode and control frequency. During the actual run, the daemon
connects the hardware when the client connects.

*If not:* remove a leftover `robot` mapping. For a runtime connection failure,
check the daemon's listening address, network/firewall reachability and token.
Keyboard events belong to the actor's event source; a keyboard attached only to
the NUC does not automatically become the remote actor's keyboard. Avoid opening
the same GELLO independently in both processes: use the daemon's teleop part.

Next: [04 Record demos](04-record-demos.md), with the workflow and key migration
limitations described in [00](00-overview.md#known-limits).
