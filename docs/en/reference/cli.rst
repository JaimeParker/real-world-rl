.. _en-cli-rwrl-station-command-line:

rwrl: station command line
================================================================================

``rwrl`` unifies training, recording, deployment and station operations. People and agents use
the same commands; ``--json``, ``--yes`` and exit codes make the built-in primitives usable
without interaction. This page is the reference; the \ :doc:`tutorial <../index>`\ 
walks through the commands in order.

After installing the ``real-world-rl`` package with the station's device extras, ``rwrl`` is
available in that environment (\ :doc:`installation <../tutorials/install>`\ ); it is equivalent to
``python -m real_world_rl.cli``. ``train``, ``record``, ``deploy``, ``deploy-act``, ``deploy-openpi``, ``collect-reward``, ``teleop run``,
``archive inspect`` and ``dataset export --format pkl`` also need the ``learning`` extra; without it they exit 2 with the hint
``pip install -e ./rl-garden -e '.[learning]'``. Run hardware commands on the **robot host**
connected to the devices, with the **project environment active**. Relative station and preset
paths resolve from the current directory. Each primitive connects for its invocation and
disconnects afterwards, including on failure.

Place global options **before the command**, including ``--preset`` before ``home`` or ``doctor``.
``rwrl --help`` lists the commands; ``rwrl <command> --help`` shows one command's flags.

.. _en-cli-global-options:

Global options
--------------------------------------------------------------------------------

.. list-table::
   :header-rows: 1

   * - Option
     - Default and meaning
   * - ``--station PATH``
     - Unset: the \ :ref:`station lookup <en-cli-station-lookup>`\ .
   * - ``--robot NAME``
     - Unset: ``$RWRL_ROBOT`` when no ``--station`` is given, else the single robot; a multi-robot station requires a name for primitives.
   * - ``--preset PATH``
     - Unset: ``$RWRL_PRESET``, then the \ :ref:`rwrl use preset <en-cli-remembered-station-and-preset>`\  selection, else the station home. For ``home``, an env preset (a bare mapping or settings under ``real_robot``); required by ``home set``. ``doctor`` uses it to find expected devices and to compare its robot with the station. For learning commands it becomes ``--config PATH``.
   * - ``--direct``
     - Off: follow the station's daemon declaration. Force direct access only when its daemon port is unreachable.
   * - ``--mock``
     - Off: fake backends for a **direct** connection. Still requires a station file; does not bypass daemon selection.
   * - ``--json``
     - Off: built-in primitives print one JSON result on stdout; diagnostics, motion targets and prompts go to stderr.
   * - ``--yes``
     - Off: skip the motion confirmation or the doctor repair-plan confirmation. Preconditions still apply.

Commands without JSON output (``setup``, ``daemon``, ``supervise`` and the learning commands) reject
``--json`` with exit code 2. ``doctor`` also accepts ``--station``, ``--robot``, ``--preset``, ``--mock``,
``--json`` and ``--yes`` after the command; a value after the command wins.

.. _en-cli-station-lookup:

Station lookup
--------------------------------------------------------------------------------

Every command that reads a station takes the first of:

1. ``--station PATH`` (also ``rwrl setup --station``, ``rwrl daemon --station``, ``rwrl discover --station``);
2. ``$RWRL_STATION``;
3. the \ :ref:`rwrl use station <en-cli-remembered-station-and-preset>`\  selection;
4. ``./local/station.yaml``;
5. ``$XDG_CONFIG_HOME/rwrl/station.yaml`` (default ``~/.config/rwrl/station.yaml``).

Explicit and environment paths are used as given. For one release the pre-0.1.0 names are still
read after their replacements, ``$ROBOT_INFRA_CONFIG`` after ``$RWRL_STATION`` and
``~/.config/robot_infra/station.yaml`` last; ``rwrl doctor`` reports either as a ``WARN`` with the
command that moves it. For ``setup``, ``--station`` supplies the existing defaults and the default
output destination; ``--scope local|user`` or ``--output PATH`` overrides that destination.

.. _en-cli-remembered-station-and-preset:

Remembered station and preset
--------------------------------------------------------------------------------

``rwrl use station PATH`` and ``rwrl use preset PATH`` remember a file in ``./local/.rwrl-use.json``, so
commands run later from the same directory can omit ``--station`` or ``--preset``. The file is loaded
first and refused if it does not load. Relative paths are kept relative to the current directory.
``rwrl use`` alone shows the station and preset a command would use and where each comes from;
``rwrl use --clear [station|preset]`` forgets one or both.

A command that falls back to a remembered file prints ``using preset PATH (from rwrl use)`` (or
``station``) once on stderr, and its ``--json`` result gains ``from_rwrl_use: {kind: path}``. An explicit
flag always wins, and ``$RWRL_STATION`` wins over the remembered station. A remembered file that no
longer exists is a usage error (exit 2), not a silent fallback to the next lookup step. Learning
commands and ``archive inspect`` never receive the remembered preset: pass their ``--config``
explicitly.

.. _en-cli-shell:

Interactive shell
--------------------------------------------------------------------------------

``rwrl shell [PROFILE] [--task NAME]`` opens an interactive terminal for a device profile.
Without PROFILE it shows a selection list; ``--task`` requires PROFILE. Type commands with or
without the ``rwrl`` prefix. Plain ``rwrl`` commands are unchanged for agents and scripts.
The shell requires a terminal and the ``shell`` extra. Install it from the repository root:

.. tutorial-test: skip
.. code-block:: bash

   pip install -e '.[shell]'

``scripts/install.sh`` installs this extra by default. Profiles live under the current directory:

.. code-block:: text

   local/sites/<name>/
     station.yaml
     task.yaml
     tasks/
       <name>.yaml
     <teleop_profiles file>
     spacemouse-devices.yaml

Keep relative files such as the ``teleop_profiles`` file and ``spacemouse-devices.yaml`` beside
``station.yaml``. Tasks come from ``task.yaml`` and ``tasks/*.yaml``.
``local/station.yaml`` and ``~/.config/rwrl/station.yaml`` also appear as ``local`` and ``user``
(the user path follows ``XDG_CONFIG_HOME``). If a site already uses that name, the entry is named
``local/station.yaml`` or ``user/station.yaml``. A profile that does not load is listed with its
reason and cannot be selected.

.. list-table:: Built-in commands
   :header-rows: 1

   * - Command
     - Effect
   * - ``switch [PROFILE]``
     - Select a device profile; omit the name to open the list.
   * - ``task [NAME]``
     - Select a task from this profile; omit the name to open the list.
   * - ``robot [NAME|all]``
     - Select one robot or all; omit the name to open the list.
   * - ``menu``
     - Pick a command and fill the prompt for editing; it does not run yet.
   * - ``info``
     - Show the selected files, robot and connection addresses.
   * - ``default``
     - Ask before saving the station and task preset like ``rwrl use``; the robot choice is not saved.
   * - ``daemon [start|stop|log|status|run]``
     - Manage local daemons; no action means ``status``.
   * - ``help [COMMAND]``
     - List commands, or show a command's options.
   * - ``exit``
     - Leave the shell; Ctrl-D also leaves.

Selection applies only to this terminal. Children receive ``RWRL_STATION``, ``RWRL_PRESET`` and,
when one robot of a multi-robot station is selected, ``RWRL_ROBOT``. Other terminals keep their
selection unless ``default`` stores the station and preset for plain commands in this directory.
An inherited ``RWRL_STATION`` is ignored. A line with its own ``--station`` receives none of these
selection variables. Foreground commands own the terminal; Ctrl-C reaches the command and the
shell stays open. Motion confirmations are unchanged.

For ``train``, ``record``, ``collect-reward`` and ``deploy-act``, a missing ``--config`` (with no
explicit ``--preset``) is filled from the task and returned for editing before execution.
A config whose remote does not match this station's daemon needs a ``y`` confirmation.
For matching daemon addresses, the shell checks identity and refuses a daemon serving another
robot. An unreachable daemon is left for the command to report. ``deploy`` and ``deploy-openpi``
take no ``--config`` and are not checked here.

``daemon start`` starts the selected station's daemons in the background. They outlive the shell;
process records and logs live under ``local/.rwrl-shell/``. ``daemon status`` reports state and
identity, ``daemon log`` shows the latest log tail, and ``daemon run`` runs in the foreground.
Options go after ``start`` or ``run``; ``start`` does not accept ``--station`` or ``--bind`` overrides.
``daemon stop`` asks before stopping each daemon of this profile started by a shell in this
directory, including an earlier shell. It cannot stop daemons started elsewhere. It sends only
SIGTERM, never SIGKILL. Leaving the shell asks about each still-running shell-started daemon
recorded in this directory; declining leaves it running.

.. _en-cli-learning-commands:

Learning commands
--------------------------------------------------------------------------------

``train``, ``record``, ``deploy`` and ``collect-reward`` call the training, demonstration, deployment
and reward-collection entrypoints. Their arguments, including ``--help``, belong to those
entrypoints and are passed through unchanged; ``rwrl train --help`` shows the training parser.
Global ``--preset PATH`` prepends ``--config PATH``; giving both is a usage error. Global
``--station``, ``--robot``, ``--direct``, ``--mock``, ``--json`` and ``--yes`` are inapplicable and return
exit code 2. The console scripts, module entrypoints and example scripts remain available.

``deploy-act`` runs a standalone ACT checkpoint with the recording preset and demonstration contract sidecar. See the :doc:`deployment tutorial <../tutorials/deploy>`.

``deploy-openpi`` evaluates a websocket policy on the GPU host and reaches the robot through
the daemon. It uses the ``learning``, ``openpi-client`` and ``grpc`` extras for live deployment.
See the \ :doc:`openpi deployment guide <deploy-openpi>`\ .

.. _en-cli-connection-ownership:

Connection ownership
--------------------------------------------------------------------------------

If the selected station robot declares ``daemon:``, ``rwrl`` probes its bind port with a 0.5-second
TCP timeout and uses the daemon when reachable (a wildcard ``0.0.0.0`` bind becomes ``127.0.0.1``).
``status`` then starts with ``connection: daemon HOST:PORT``. A declared but unreachable daemon is
an error (exit 1) with the hint to start ``rwrl daemon`` or pass ``--direct``; there is no automatic
direct fallback.

Without ``daemon:``, access is direct: a Franka station using franky reports
``connection: direct franky``, a mocked one ``direct fake``. ``--direct`` is refused with exit 2 while
the declared daemon's port is reachable: ``error: the daemon owns the arm (hint: omit --direct)``.

Franka FCI allows one control connection. On the direct route, finish the recorder or actor
before opening another hardware connection with ``rwrl``; on the daemon route, use the same
daemon. ``stop`` and ``recover`` follow this selection too; they do not bypass ownership of a
directly held arm.

On the daemon route, the station's name, type and address (``robot_ip`` or ``can_interface``,
including environment fallbacks) must match what the daemon reports. The address is not compared
when the station has none after those fallbacks. A mismatch or an older daemon reporting no
identity is refused before any device call; restart an older daemon with the current version.
``stop`` still runs despite an identity mismatch and then warns; ``teleop run`` checks before
starting. Training, recording and deployment entry points connect to the preset's remote and
do not perform this station identity check themselves.

.. _en-cli-commands:

Commands
--------------------------------------------------------------------------------

"Confirm" means the wrapper's motion prompt (``--yes`` skips it). Unless noted, built-in commands
exit 0 on success, 1 on a runtime or device failure and 2 on a usage or configuration error or a
refused confirmation (\ :ref:`exit codes <en-cli-confirmation-exit-codes-and-json>`\ ).

**Station primitives** (each opens the robot connection)

.. list-table::
   :header-rows: 1

   * - Command
     - Effect
     - Confirm
   * - ``status``
     - connection path, robot, arm health, gripper feedback, camera and teleop names; daemon RTT when remote. ``--json`` adds ``arm_health`` details and safety clip/reject counts
     - no
   * - ``q``
     - joint positions in radians
     - no
   * - ``tcp``
     - TCP position in metres and quaternion in ``xyzw``
     - no
   * - ``home show``
     - resolved home, current joints, ``home - q`` and its largest value
     - no
   * - ``home set``
     - save the measured joints into ``--preset`` (\ :ref:`home <en-cli-home-resolve-save-and-move>`\ )
     - no
   * - ``home go``
     - blocking move to home, then the preset's reset gripper action
     - yes
   * - ``gripper show``
     - width in metres and normalized, grasp state, backend
     - no
   * - ``gripper open`` / ``close`` / ``move F``
     - normalized width 1, 0, or ``F`` with ``0 <= F <= 1``
     - yes
   * - ``stop``
     - arm ``stop()``; prints ``stop: ok``
     - no
   * - ``recover``
     - arm ``recover()``; prints ``recover: ok``
     - no
   * - ``camera show [--name NAME] [--frames N]``
     - configuration, frame rate over N reads (default 30; 0 reads none and leaves health unjudged), frame age, shapes and health; exit 1 on read errors or unhealthy cameras
     - no
   * - ``camera grab --name NAME --output DIR``
     - one frame as ``NAME-color.png``, ``NAME-depth.png`` (with depth) and ``NAME.json``; never overwrites; ``--timeout-s`` (default 2)
     - no

**Diagnosis and configuration**

.. list-table::
   :header-rows: 1

   * - Command
     - Effect
   * - ``doctor``
     - inspect the host and the station's devices; ``--fix`` repairs host and software after one confirmation. Reference: \ :doc:`rwrl doctor <doctor>`\
   * - ``setup``
     - the station wizard: discover, ask, calibrate GELLO, write the station, run the doctor (\ :doc:`tutorial 02 <../tutorials/station>`\ )
   * - ``discover``
     - print visible hardware without connecting to robot hosts (\ :ref:`details <en-cli-discover>`\ )
   * - ``daemon``
     - serve the station's robots over the network (\ :doc:`remote actor <remote-actor>`\ )
   * - ``supervise -- COMMAND``
     - run and supervise a robot process with heartbeat and stop escalation

``setup``, ``daemon``, ``supervise`` and ``discover`` are native device-layer commands
(``real_world_rl.cli.device_commands``); their exit codes are their own (see below), and
``setup``, ``daemon`` and ``supervise`` have no JSON output. Supervisor arguments after ``--`` belong to
the child.

**Teleop**

.. list-table::
   :header-rows: 1

   * - Command
     - Effect
     - Exit codes
   * - ``teleop read``
     - live samples of the station teleop (\ :ref:`details <en-cli-teleop-read>`\ )
     - 0 / 1 (device failure, unhealthy teleop) / 2
   * - ``teleop calibrate``
     - calibrate the station teleop; ``--backend spacemouse|gello`` (\ :ref:`details <en-cli-teleop-calibrate>`\ )
     - 0 / 1 / 2
   * - ``teleop profiles list/show/use``
     - SpaceMouse direction profiles (\ :doc:`guide <spacemouse-commissioning>`\ )
     - 0 / 1 / 2
   * - ``teleop diagnose``
     - SpaceMouse input-only diagnosis (\ :doc:`guide <spacemouse-commissioning>`\ )
     - 0 / 1 / 2
   * - ``teleop run``
     - supervised recording with operator-confirmed episode boundaries (``confirm``: space starts, ``n`` resets; see \ :ref:`Episode boundaries <en-recording-demos-episode-boundaries>`\ ); ``--backend gello`` uses the GELLO cartesian leader; asks before reset (\ :ref:`guide <en-spacemouse-commissioning-supervised-recording-sessions>`\ )
     - child status or wrapper error
   * - ``teleop probe-hid``
     - record a SpaceMouse-like HID device for the site device table (\ :ref:`details <en-cli-teleop-probe-hid>`\ )
     - 0 / 1 (no such device, busy) / 2
   * - ``teleop inspect-stop --trace FILE``
     - evaluate a saved stop trace offline (\ :ref:`guide <en-spacemouse-commissioning-live-input-calibration-and-diagnosis>`\ )
     - 0 PASS / 3 FAIL / 4 BLOCKED

**Limits, the table frame, selections and archives** (these read the station file only, except ``workspace capture`` and ``shell``)

.. list-table::
   :header-rows: 1

   * - Command
     - Effect
     - Exit codes
   * - ``workspace show``
     - ``task_frame``, the resolved ``limits`` (joints, TCP box, ``max_delta_per_step``), ``home_joints``, ``home_within_limits`` (``null`` without home or joint limits; ``home_violations`` per joint)
     - 0 / 1 / 2
   * - ``workspace capture --name N --output PTS --save``
     - record the current TCP as a named point; no motion
     - 0 / 1 / 2
   * - ``workspace validate --pose X Y Z QX QY QZ QW``
     - check whether the position of a base-frame pose is inside the ``limits.tcp`` box (the orientation is not checked; \ :ref:`Limits <en-devices-limits>`\ )
     - 0 / 1 / 2; text validation failure 5
   * - ``workspace frame show`` / ``frame set``
     - the station ``task_frame`` (\ :ref:`details <en-cli-workspace-frame-set>`\ )
     - 0 / 1 / 2
   * - ``limits show [STATION]``
     - the resolved limits of each arm: joints, TCP box centre, axes, ``low`` / ``high`` and the eight base-frame corners (\ :ref:`details <en-cli-limits>`\ )
     - 0 / 1 / 2
   * - ``limits migrate YAML [--write]`` / ``limits rebase YAML [--write]``
     - convert old top-level ``safety_box`` / ``workspace`` / ``joint_limits`` into ``limits:``; or recompute the frozen ``origin_xyz`` / ``axes_R`` from the current home and ``task_frame``. Without ``--write`` only the new block and the diff are printed (\ :ref:`details <en-cli-limits>`\ )
     - 0 / 1 / 2
   * - ``use [station|preset] [PATH]`` / ``use --clear [KIND]``
     - remember, show or forget the default station and preset for this directory (\ :ref:`details <en-cli-remembered-station-and-preset>`\ )
     - 0 / 2
   * - ``shell [PROFILE] [--task NAME]``
     - interactive device and task selection (\ :ref:`details <en-cli-shell>`\ )
     - 0 / 2
   * - ``archive inspect ARCHIVE``
     - read a demo dataset directory offline (an old recorder SQLite archive is also accepted) (\ :ref:`details <en-cli-archive-inspect>`\ )
     - 0; with ``--stop-time-s``: 0 PASS / 3 FAIL / 4 BLOCKED; 2
   * - ``dataset list|view|drop|undrop|prune|export DIR``
     - list, replay (Rerun), drop / restore / prune episodes and export to pkl or LeRobot, offline (\ :ref:`details <en-cli-dataset>`\ )
     - 0 / 2

**Learning:** ``train``, ``record``, ``deploy``, ``deploy-act``, ``deploy-openpi``, ``collect-reward`` (\ :ref:`above <en-cli-learning-commands>`\ ).

.. _en-cli-command-details:

Command details
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. _en-cli-discover:

discover
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--json`` prints one object with ``schema_version: 1`` and six lists, each empty when nothing is
found: ``cameras`` (``backend``, ``serial``), ``teleops`` (``backend``, ``device``), ``can_interfaces``
(``name``, ``operstate``, ``bitrate``), ``hosts`` (``ip``, ``ping_ok``, ``open_ports``; the station's robot
IPs), ``hid_devices`` (paths) and ``unknown_hid`` (``vid``, ``pid``, ``product``, ``path``, ``message``; see the
\ :ref:`site device table <en-spacemouse-commissioning-site-device-table>`\ ). The text output adds a
``warnings:`` section only when ``unknown_hid`` is not empty.

.. _en-cli-teleop-operations:

teleop operations
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The operations other than ``read`` and ``inspect-stop`` run in the device slice of a teleop backend
and follow the station teleop backend (``calibrate`` and ``run`` also take ``--backend``, and need it when the
station has no teleop). An operation the backend does not implement is a usage error naming the
backends that do, for example ``backend 'gello' does not implement teleop diagnose``.
``probe-hid`` probes the host's HID devices whatever the station teleop is.
``rwrl teleop <operation> --help`` lists the flags grouped by backend.

.. _en-cli-teleop-read:

teleop read
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Prints ``q`` or ``twist``, gripper, intervening and fired events, one line per sample at ``--rate-hz``
(default 3) until Ctrl-C or ``--duration-s``, then a summary line; ``--json`` prints JSON lines and
a closing ``{"summary": ...}``. The direct route opens only the teleop part. ``--with-follower``
(daemon route only) adds the follower ``follower_q`` and, for ``joint_abs`` leaders, the per-joint
leader-minus-follower ``delta``. No robot motion.

.. _en-cli-teleop-calibrate:

teleop calibrate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

SpaceMouse fits and validates a direction profile (``--profile`` required). GELLO runs the guided
leader calibration, takes unset ``--port``, ``--joint-ids``, ``--gripper-id``, ``--baudrate`` and
``--joint-signs`` from the station ``teleop.options`` and the wrap centres from the station
``joint_limits``, and prints the ``teleop:`` YAML block (\ :ref:`gello.md <en-gello-2-calibration>`\ ). ``--use``
selects the profile (SpaceMouse) or writes the calibrated ``teleop.options`` into the station
after a reviewed diff (GELLO). Flags of the other backend are a usage error.
``setup --only teleop --calibrate`` runs the same calibration inside the wizard.

.. _en-cli-teleop-probe-hid:

teleop probe-hid
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Records an attached 3Dconnexion HID device (or a Logitech multi-axis controller): its report
descriptor and ``--duration-s`` (default 10) of raw input reports, into a new
``<station dir>/teleop/probes/<vid>-<pid>/`` (``--output DIR`` instead; required without a station;
an existing directory is a usage error) holding ``descriptor.hex``, ``reports.jsonl`` and
``candidate.yaml``, a draft device-table entry. ``--device VID:PID`` picks one of several devices.
It never writes the device table and never records the serial number.

.. _en-cli-limits:

limits
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``limits show`` reads the station file (positional argument or ``--station``, otherwise the \ :ref:`station lookup <en-cli-station-lookup>`\ ), ``--robot`` selects one robot, and prints JSON: per arm the ``version`` (``station_limits_v1``), ``joints``, ``tcp`` (``origin_xyz``, ``axes_R``, ``low``, ``high``, ``measured_margin_m``, ``corners_base_m``), ``max_delta_per_step`` and ``kinematics``. ``limits migrate YAML`` and ``limits rebase YAML`` take that file as an argument; by default they only print warnings, the new ``limits:`` block and a unified diff, and ``--write`` saves it atomically with a backup. The whole candidate file is validated first and nothing is written if that fails. Fields, migration rules and breach behaviour are under \ :ref:`Limits <en-devices-limits>`\ .

.. _en-cli-workspace-frame-set:

workspace frame set
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``--name NAME`` with either ``--base-T-frame`` (16 row-major numbers, metres) or
``--from-points PTS --origin A --x-axis B --y-axis C`` (point names saved by
``workspace capture --save``; right-handed, y orthogonalised to x, degenerate points refused).
``--source TEXT`` records provenance; ``--save`` writes only ``task_frame`` after a reviewed diff. The rotation of ``task_frame`` sets the axes of the ``limits.tcp`` box, so after saving the output tells you to run
``rwrl limits rebase <yaml>``. The candidate (and ``workspace frame show``) also reports ``rpy_deg``, the frame's roll,
pitch and yaw about the base x, y, z axes (``base_R_frame = Rz(yaw) Ry(pitch) Rx(roll)``), with ``axis_lengths_m`` and ``y_axis_angle_deg`` added only for a candidate
built from points. ``frame show`` instead includes ``origin_base_m`` and
``axes_base``. Relative ``--from-points`` paths resolve from the station directory.

.. _en-cli-archive-inspect:

archive inspect
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reads the dataset directory (or an old SQLite archive) read-only and prints indented JSON: outcomes, mask counts, hold drift, and
take-over jump and residual per ``human_action_mask`` segment. ``--csv PATH`` and
``--measurements PATH`` write new step and measurement CSVs; ``--drift-window-s`` (default 10) sets
the hold-drift window; each ``--stop-time-s T`` (repeatable) adds a settle evaluation with
``--settle-speed-mm-s`` (default 2) and ``--settle-hold-s`` (default 0.5). ``--json`` prints the same
fields on one line, non-finite numbers as ``null``, plus ``status`` (``REPORT``, ``PASS``, ``FAIL`` or
``BLOCKED``; ``BLOCKED`` is a stop time without a spanning TCP episode). No station is loaded:
``--station``, ``--robot``, ``--preset``, ``--direct``, ``--mock`` and ``--yes`` are usage errors.
``python examples/inspect_teleop_archive.py`` is an alias whose exit code ignores the settle
status.

.. _en-cli-dataset:

dataset
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``rwrl dataset <list|view|drop|undrop|prune|export> DIR`` works offline on the demo dataset that ``rwrl record --output-dir`` writes (format and usage: \ :doc:`Recording demos <recording-demos>`\ ). ``list`` prints the episode table; ``view`` replays in the Rerun viewer (``-e N`` is repeatable, ``--web`` serves a web viewer, ``--save PATH.rrd`` only writes a file; needs the ``viewer`` extra); ``drop DIR N...`` (``--note TEXT``) and ``undrop DIR N...`` reversibly drop or restore episodes; ``prune DIR --yes`` deletes every discarded episode directory, and omitting ``--yes`` (or the global ``--yes``) is a usage error; ``export DIR --format pkl|lerobot --out PATH`` exports (``--frame-stack`` and ``--keep-syncing-steps`` apply to pkl only, ``--repo-id``, ``--fps`` and ``--vcodec`` to lerobot only; lerobot needs the ``lerobot`` extra). ``--json`` prints the matching structured result. No station is loaded: ``--station``, ``--robot``, ``--preset``, ``--direct`` and ``--mock`` are usage errors. Read errors such as an unknown episode or a directory that is not a dataset exit 2.

.. _en-cli-robot-host-diagnosis-and-repair:

Robot-host diagnosis and repair
--------------------------------------------------------------------------------

``rwrl doctor`` checks the host, the project environment and every station robot, previews
targeted repairs and applies them with ``--fix``. Its checks, statuses, repairs, report directory,
JSON and exit codes are on its own page: \ :doc:`rwrl doctor <doctor>`\ .

For deployment links, use
``rwrl doctor --link HOST:PORT [--ssh [USER@]HOST] [--remote-bind HOST:PORT] [--policy HOST:PORT] [--token TOKEN]``.
The address is the client-visible gRPC endpoint, ``--ssh`` enables SSH and forwarding checks (``--remote-bind`` names the daemon address on the NUC),
``--policy`` reads websocket metadata, and ``--token`` authenticates gRPC. See the
\ :ref:`station link schema and repairs <en-doctor-grpc-and-ssh-link-rows>`\ .

.. _en-cli-home-resolve-save-and-move:

Home: resolve, save and move
--------------------------------------------------------------------------------

Home resolution matches ``env.reset()``, in this order:

1. Preset ``task.reset.home_joints``.
2. Preset ``home_joints``.
3. Station ``home_joints`` (the daemon arm's ``home_joints`` when remote).

Null values fall through to the next source; no home is an error (exit 1). ``home go`` runs only
reset's home phase: a blocking joint move, then ``task.reset.gripper`` if it is ``open`` or ``close``
and a gripper exists (``keep`` leaves it alone). It skips lift, waypoints, jitter and settle.

``home set`` requires an existing ``--preset PATH``. It reads the measured joints, rounds them to
four decimals and writes ``task.reset.home_joints``, creating missing task/reset mappings. Both
measured and rounded values are checked against the station/arm joint limits and the preset's
``joint_limits`` and ``robot.joint_limits``; out-of-bounds values fail with exit 1 before anything is
written. YAML edits (here and for station and profile files) preserve comments, quotes, blank
lines and key order; the exact previous bytes are saved to a unique ``<file>.<id>.bak`` beside the
file first, and concurrent edits are rejected. The worked sequence is
\ :doc:`tutorial 04 <../tutorials/controller>`\ .

.. _en-cli-confirmation-exit-codes-and-json:

Confirmation, exit codes and JSON
--------------------------------------------------------------------------------

``home go`` and gripper motion print the target and the largest change first, even with ``--yes``.
Without ``--yes``, a terminal prompts ``proceed? [y/N]``; only ``y`` (any case, surrounding
whitespace ignored) proceeds. Other answers or end of input cancel with exit 2; non-terminal
input refuses motion with exit 2 (``error: refusing to move without --yes``).

.. list-table::
   :header-rows: 1

   * - Built-in exit code
     - Meaning and examples
   * - 0
     - Command completed (or help displayed).
   * - 1
     - Runtime/device failure: unreachable declared daemon, missing home/gripper, invalid home joint values, bounds failure or other exception.
   * - 2
     - Usage/config-selection error: unknown arguments, missing station, missing ``--robot`` in a multi-robot station, missing ``--preset`` for ``home set``, invalid ``gripper move F``, reachable daemon with ``--direct``, or refused/cancelled motion.

Errors print ``error: MESSAGE`` on stderr, adding ``(hint: HINT)`` when a hint exists. ``--json`` also
prints ``{"error": "MESSAGE", "hint": "HINT"}`` on stdout (the hint can be empty).
``RWRL_DEBUG=1`` additionally prints a traceback to stderr. The exit-code constants are in
``real_world_rl.cli.errors``. Native command errors also use stderr. Supervisor child statuses are
preserved; a failed launch returns 127, stop escalation 1 and an interrupted setup 130.
Stop-trace results keep 3/4 and text workspace validation keeps 5. Doctor's codes are on
\ :ref:`its page <en-doctor-exit-codes>`\ .

For successful connection primitives, ``--json`` prints an object with ``command``,
``path`` and command-specific fields. Workspace, selection, discovery and doctor
commands use their own schemas, and ``teleop read`` streams JSON lines;  for example, on the mock station of \ :doc:`tutorial 04 <../tutorials/controller>`\ :

.. code-block:: text

   $ rwrl --station local/mock-station.yaml --mock --json q
   {"command": "q", "path": "direct fake", "joint_pos": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}

.. _en-cli-calling-from-an-agent:

Calling from an agent
--------------------------------------------------------------------------------

Start with ``rwrl status`` to inspect the connection and hardware. For unattended built-in
primitives, use ``--json --yes`` for motion, or read-only commands with ``--json``. Keep global
options before the command and inspect the exit code. Read stderr's ``error: … (hint: …)`` as well
as any JSON error; ``--json`` reserves stdout for the result. ``--yes`` authorizes the requested
motion: verify the station, home and workspace before issuing it. Apply the device command's own
rules when calling doctor, setup, daemon or supervise.

Missing required arguments return exit code 2. With ``--json``, the error includes
``required_fields`` (CLI flag names, or a missing subcommand destination) alongside ``error`` and
``hint``. SpaceMouse runtime checks also report missing preset, capture output/name and
sample-file requirements. This describes missing inputs; it does not authorize motion or waive
confirmation. Calibration's ``NEEDS_SAMPLES`` result instead lists missing direction labels.

.. _en-cli-spacemouse-commissioning:

SpaceMouse commissioning
--------------------------------------------------------------------------------

Profiles, task-frame workspaces and supervised recording and diagnosis sessions are in the
\ :doc:`SpaceMouse commissioning guide <spacemouse-commissioning>`\ .


