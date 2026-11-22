.. _en-remote-actor-remote-actor-over-the-robot-daemon:

Remote actor over the robot daemon
================================================================================

For device RPCs and RemoteRobot on the GPU machine, see the \ :doc:`robot daemon <daemon>`\ .

The main workflow runs the recorder or HIL actor on the NUC with a local
``real_robot.robot`` mapping and ``arm_backend: franky``. That process opens the arm,
Hand, cameras and GELLO directly. **Do not start a daemon for this route.**
A learner on a GPU machine does not by itself require a robot daemon: the actor
can remain on the NUC and send training data to the learner.

Only use the daemon route below when the **actor itself runs on another
machine**, using ``real_robot.remote`` instead of ``real_robot.robot``.

.. list-table::
   :header-rows: 1

   * - Property
     - Direct on the NUC (main workflow)
     - Remote actor (optional)
   * - Hardware processes, one arm
     - one recorder or actor process
     - NUC daemon plus recorder or actor on the other machine
   * - Connection config
     - ``real_robot.robot`` mapping
     - ``real_robot.remote: NUC_ADDRESS:50051``; omit ``robot``
   * - Operator keyboard
     - attached to the NUC; target terminal keys in \ :doc:`00 <../index>`\
     - read on the actor machine; ``stdin`` uses its terminal (including SSH), while ``keyboard`` / ``pynput`` needs its desktop session
   * - Action watchdog
     - application checks share the hardware process
     - daemon can stop the arm when client actions stop arriving
   * - External stop
     - supervisor can kill a stuck process; firmware then handles lost control communication
     - a live daemon can issue stop independently of the actor

An application thread cannot guarantee ``stop()`` if its whole process hangs or
is killed. A supervisor killing the direct process relies on the firmware's
communication-loss response; it is not an independent application stop command.
The daemon watchdog also cannot cover a hung daemon or failed NUC. Keep the
physical emergency stop available in both layouts.

.. _en-remote-actor-start-a-daemon-only-for-a-remote-actor:

Start a daemon only for a remote actor
--------------------------------------------------------------------------------

First finish the \ :doc:`real doctor handshake <../tutorials/station>`\ .
Run every command in this section on the **NUC**, from the **repository root**,
with the **project environment active**. Ensure no other process owns these devices.
The remote route needs the ``grpc`` extra installed on both NUC and actor.

.. code-block:: bash
   :dedent: 0

   rwrl daemon --station local/station.yaml --watchdog-s 5

``--watchdog-s`` defaults to **0.5 seconds**. The explicit **5 seconds** above allows
for slow first inference after reset: if the interval exceeds the watchdog, the
daemon idles the arm and the next action fails. Choose the timeout for the cell;
5 seconds is an example, not a guarantee that a long pause is safe.

*You should see:* preflight doctor checks followed by the serving address and
robot parts. Leave the terminal open. Ctrl-C stops the daemon. A multi-arm station
starts one daemon process per configured robot, on separate ports; the launcher
can start both from the station file.

*If not:* follow the named preflight hint in \ :doc:`03 <../tutorials/station>`\ . Do not run doctor
against the same devices while a recorder, actor or daemon owns them: the device
claim and handshake will conflict. Doctor lists such a process as ``BLOCKED``; only
commands holding a hardware connection count, so offline ``rwrl`` commands such as
``workspace show``, ``teleop inspect-stop`` or ``archive inspect`` do not block it.

To check the connection after starting the daemon, open another terminal on
this **NUC**, at the **repository root**, with the **project environment active**:

.. code-block:: bash
   :dedent: 0

   rwrl status

With ``daemon:`` declared in ``local/station.yaml``, expect
``connection: daemon HOST:PORT``. On the main route, with no daemon declaration
and no recorder/actor holding the arm, the same command reports
``connection: direct franky``. A declared but unreachable daemon is an error;
see \ :ref:`rwrl connection ownership <en-cli-connection-ownership>`\ .

.. _en-remote-actor-point-the-remote-actor-at-the-nuc:

Point the remote actor at the NUC
--------------------------------------------------------------------------------

On the **actor machine**, use the project environment, from its repository root.
In its ``hil.yaml``, remove ``real_robot.robot`` and set ``real_robot.remote`` to the
NUC's reachable address and the station's daemon port (normally 50051). Set
``real_robot.token`` only if ``daemon.token`` is configured. With cameras, provide
``obs.image_size`` because remote sizes are only known after connecting. Keep the
same action mode, 50 Hz frequency and observation contract as recording.

Install the gRPC client in the actor's project environment:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python -m pip install -e '.[grpc]'

For ``leader_cartesian``, also replace the local leader ``robot_yaml`` option.
Save ``leader_remote.yaml`` on the actor machine:

.. code-block:: yaml

   remote_address: NUC_ADDRESS:50051
   # remote_token: T  # only when daemon.token is configured

The leader token is separate from ``real_robot.token``. This reads the daemon's
teleop part; it does not open a second local GELLO.

Check that configuration without connecting, on the actor machine, repository
root, project environment active:

.. code-block:: bash
   :dedent: 0

   rwrl train hil_serl --config hil.yaml --teleop-device leader_cartesian \
       --teleop-options-yaml leader_remote.yaml --teleop-event-device stdin --print-config

*You should see:* exactly one connection source (``remote``), the correct camera
names, action mode and control frequency. During the actual run, the daemon
connects the hardware when the client connects.

*If not:* remove a leftover ``robot`` mapping. For a runtime connection failure,
check the daemon's listening address, network/firewall reachability and token.
Keyboard events belong to the actor's event source; a keyboard attached only to
the NUC does not automatically become the remote actor's keyboard. Avoid opening
the same GELLO independently in both processes: use the daemon's teleop part.

Next: \ :doc:`08 Record demos <../tutorials/record>`\ , with the workflow and key migration
limitations described in \ :doc:`00 <../index>`\ .


