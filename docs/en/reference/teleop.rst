.. _en-teleop-teleoperation:

Teleoperation
================================================================================

Human-in-the-loop intervention in ``real_world_rl``: a ``TeleopSource`` reads one operator
device, ``InterventionWrapper`` swaps the operator's action in for the policy's, and the demo
scripts in ``examples/`` use the same sources.

**Where to start:** follow \ :doc:`05 GELLO <../tutorials/gello>`\ ,
\ :doc:`06 SpaceMouse <../tutorials/spacemouse>`\  or
\ :doc:`07 leader-follower <../tutorials/gello>`\  for input checks and first supervised
sessions, then \ :doc:`08 Record demos <../tutorials/record>`\ . This page is the source contract
and device reference.

.. _en-teleop-the-teleopsource-contract:

The ``TeleopSource`` contract
--------------------------------------------------------------------------------

``real_world_rl.teleop`` (``teleop/core/contract.py``):

.. code-block:: python
   :dedent: 0

   from abc import ABC
   from dataclasses import dataclass, field
   from typing import Any, Mapping
   import numpy as np

   @dataclass(frozen=True)
   class TeleopReading:
       action: np.ndarray          # float32 (A,), env-normalized [-1, 1], gripper last when has_gripper
       intervened: bool            # the operator is driving; action only matters then
       episode_end: bool = False   # operator's episode-end button
       events: Mapping[str, bool] = field(default_factory=dict)   # success / failure / rerecord edges that fired, else empty
       timestamp: float = 0.0
       human_action_mask: Any = None  # actual human-controlled dimensions
       override_mask: Any = None      # overridden dimensions including hold supplements
       buttons: Mapping[str, bool] = field(default_factory=dict)
       diagnostics: Mapping[str, Any] = field(default_factory=dict)

   class TeleopSource(ABC):
       action_mode: str            # "cartesian_delta" | "joint_delta" | "joint_abs"
       arm_action_dim: int
       has_gripper: bool
       def start(self) -> None: ...     # open the device (default no-op)
       def poll(self) -> TeleopReading: ...       # abstract
       def reset(self) -> None: ...     # after an env reset: drop latched events and sticky state
       def close(self) -> None: ...     # release the device (safe to call twice)
       @classmethod
       def from_env(cls, env, **kwargs): ...      # registry factory; default cls(**kwargs)

-  ``action_mode``, ``arm_action_dim`` and ``has_gripper`` are checked against the env's action contract
   when the wrapper is built; a mismatch (for example a cartesian SpaceMouse on a ``joint_delta``
   robot) raises ``ValueError`` before the device is started.
-  ``poll()`` must let device failures propagate: swallowing one would record policy actions as
   operator demonstrations.
-  ``as_teleop_source(obj)`` returns a ``TeleopSource`` unchanged and wraps any duck-typed legacy
   object (``poll()`` returning something with ``.intervened``, ``.action``, ``.episode_end``; optional
   ``reset()``, ``close()`` and ``action_mode``/``arm_action_dim``/``has_gripper``, defaulting to
   cartesian_delta / 6 / True) in a ``LegacyTeleopAdapter``.

.. _en-teleop-devices:

Devices
--------------------------------------------------------------------------------

``TeleopInterventionWrapper(env, device=..., **kwargs)`` builds the source with
``make_teleop_source(device, env, **kwargs)``; in hil_serl the device comes from ``--teleop-device``.

In hil_serl (and residual, SiLRI) the source kwargs are derived from the args (``robot``/``leader_arm``:
the robot part declaration and limits; ``gello``/``leader_arm``: the onset flags), then
``--teleop-options-yaml PATH`` is merged on top: a yaml mapping of source kwargs that overrides
the derived ones. Use it for device options the CLI has no flags for, for example a local GELLO:

.. code-block:: yaml
   :dedent: 0

   # gello.yaml, for --teleop-device gello --teleop-options-yaml gello.yaml
   port: /dev/serial/by-id/usb-FTDI_...
   joint_ids: [1, 2, 3, 4, 5, 6, 7]
   joint_offsets_rad: [...]   # from teleop calibrate --backend gello
   joint_signs: [1, 1, -1, 1, 1, 1, 1]
   gripper_id: 8
   gripper_open_rad: ...
   gripper_closed_rad: ...
   hold_s: 0.0

or ``device_index: 1`` (and ``deadzone``, ``invert_axes``) for a second SpaceMouse, or, for
``--teleop-device leader_cartesian`` on the
``real_robot`` stack, the robot daemon serving the leader arm (see
\ :ref:`Leader arm on a cartesian robot <en-teleop-leader-arm-on-a-cartesian-robot-leader_cartesian>`\ ):

.. code-block:: yaml
   :dedent: 0

   # leader_cartesian.yaml, for --teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml
   remote_address: <robot-host>:<port>

With ``--teleop-event-device`` the file feeds the motion half of the composite.

The wrapper passes its "last gripper action sent" to ``robot``, ``leader_arm``, ``gello``,
``leader_cartesian``, ``spacemouse``, ``pico`` and ``bimanual`` (and to the motion half of a composite
built on one of them), so a leader part without a gripper holds the gripper instead of dropping a
grasped object, and the SpaceMouse and the Pico start every take-over from the gripper's current
command. The
env action ends with one gripper dim per arm (the action contract's layout), so the getter
returns one entry per arm and a single-arm source reads entry 0 (``bimanual`` hands each arm's
source its own entry); a source drives one arm unless it declares ``num_arms``, and the wrapper
rejects a source whose arm count differs from the env's, or whose ``arm_names`` (when it declares
them) are not the env's in the env's order.

.. list-table::
   :header-rows: 1

   * - ``device``
     - Source
     - Reads
     - Action space
     - Notes
   * - ``pico``
     - ``PicoTeleopSource``
     - a Pico VR controller through the device layer's ``PicoTeleop`` (a ZeroMQ subscriber), as a pose input anchored onto the cartesian command tracker
     - cartesian_delta, 6 + gripper
     - bind takes over / hands back, the trigger drives the gripper; see \ :ref:`Pico <en-teleop-pico>`\ . kwargs: ``address="tcp://localhost:7777"``, ``topic=""``, ``hand="right"``, ``bind_button="bind"``, ``poll_interval_s=0.005``, ``hold_s=0.0``, ``input_rotation``, ``gripper_binarize_threshold=0.5``, ``remote_address``, ``read_timeout_s``, and the tracker's (the ``--teleop-lead-cap-*``/``--teleop-target-speed-*`` flags). Needs ``pyzmq`` (extra ``pico``), imported at ``start()``.
   * - ``pico_twist``
     - ``PicoTwistTeleopSource``
     - the Pico as a raw twist (``EETwistTeleOpWrapper``; the ManiSkill sim examples only, not for real robots)
     - cartesian_delta, 6 + gripper
     - the former ``pico``: kwargs ``zmq_url``, ``hand``, ``pos_scale``, ``rot_scale``, ``twist_limit``, ``intervention_threshold``; the subscriber opens at construction.
   * - ``spacemouse``
     - ``SpaceMouseTeleopSource``
     - a SpaceMouse on this machine, through the device layer's ``SpaceMouseTeleop``, as a velocity input over the cartesian command tracker
     - cartesian_delta, 6 + gripper
     - button 0 takes over / hands back, button 1 toggles the gripper; see \ :ref:`SpaceMouse <en-teleop-spacemouse>`\ . kwargs: ``device_index=0``, ``deadzone=0.05``, ``invert_axes=()``, ``poll_interval_s=0.005``, ``read_timeout_s``, and the tracker's (``lead_cap_m``, ``lead_cap_rad``, ``target_speed_m_s``, ``target_speed_rad_s``: the ``--teleop-lead-cap-*``/``--teleop-target-speed-*`` flags). Needs ``hidapi==0.14.0.post4`` (import ``hid``), imported at ``start()``.
   * - ``spacemouse_twist``
     - ``SpaceMouseTwistTeleopSource``
     - the same SpaceMouse as a raw twist (the ManiSkill sim examples only; not for real robots)
     - cartesian_delta, 6 + gripper
     - the puck's twist goes to the env as is, moving the puck counts as intervening (``hold_s`` extends it), button 0 toggles the sticky gripper (open after every reset), button 1 ends the episode. kwargs: ``device_index=0``, ``deadzone=1e-3``, ``invert_axes=()``, ``poll_interval_s=0.005``, ``hold_s=0.0``, ``read_timeout_s``.
   * - ``robot``
     - ``TeleopPartSource``
     - the teleop part of the robot behind a ``real_robot`` env (robot yaml ``teleop:`` section, local or daemon-served)
     - cartesian_delta, joint_abs or joint_delta, mapped into the env space
     - kwargs: ``teleop_contract``, ``joint_limits`` (the arm's ``limits.joints``), ``joint_scale_rad``, ``read_timeout_s`` (hil_serl: ``--robot-teleop-action-mode``, ``--no-robot-teleop-has-gripper``). The robot owns the part, so ``close()`` leaves it connected.
   * - ``fake``
     - ``ScriptedTeleopSource``
     - scripted readings (tests; with no ``readings`` an operator who never intervenes)
     - follows the env's action contract, its arm count included
     - pass ``readings=[TeleopReading, ...]`` to script one; the last reading repeats.
   * - ``gello``
     - ``GelloTeleopSource``
     - a GELLO on this machine, through the device layer's ``gello`` teleop (Dynamixel bus)
     - joint_abs or joint_delta (the env's), leader joints mapped as for ``robot``
     - kwargs: the device options (``port``, ``joint_ids``, ``joint_offsets_rad``, ``joint_signs``, ...; see \ :ref:`Leader arms <en-teleop-leader-arms-gello>`\ ), ``hold_s``, ``read_timeout_s``, ``onset``, ``arm_tolerance_rad``, ``ramp_step_rad``. Needs ``dynamixel-sdk`` (real-world-rl extra ``gello``), imported at ``start()``.
   * - ``leader_arm``
     - ``LeaderArmPartSource``
     - the teleop part of the robot behind a ``real_robot`` env, read as a joint-space leader arm (a GELLO served by the robot daemon)
     - joint_abs or joint_delta (the env's)
     - kwargs: those of ``robot`` plus ``onset``, ``arm_tolerance_rad``, ``ramp_step_rad`` (hil_serl: ``--teleop-onset``, ``--teleop-arm-tolerance-rad``, ``--teleop-ramp-step-rad``).
   * - ``keyboard``
     - ``KeyboardTeleopSource``
     - this machine's keyboard via ``pynput``
     - follows the env's action contract; action always zeros
     - ``t`` takes over, ``h`` hands back, Esc ends the episode, ``s``/``f``/``r`` raise ``success``/``failure``/``rerecord``, ``q`` ends the episode and raises ``operator_quit`` (the recorder and the actor loops stop after that episode). The confirm flow also reads space (``space``, start the episode) and Enter (including keypad Enter, ``enter``), plus ``n``/``o``/``c`` between episodes. An event device: on its own it only suits event-only tests. Needs ``pynput`` (extra ``reward-collection``) and a desktop session, imported at ``start()``.
   * - ``stdin``
     - ``StdinTeleopSource``
     - the terminal running the process (cbreak mode, restored on ``close()``; works over SSH, no desktop session)
     - follows the env's action contract; action always zeros
     - the ``keyboard`` keys; the space bar is ``space`` and Return (CR or LF; a CRLF pair is one press) is ``enter``. kwargs: ``fd`` (default stdin), ``poll_interval_s=0.02``. End of input raises ``DeviceFault`` on the next poll.
   * - ``jsonl``
     - ``JsonlEventSource``
     - an append-only file, or a pipe on stdin
     - follows the single-arm env action contract; action always zeros
     - ``--teleop-event-device jsonl --teleop-events-file PATH`` (``-`` for stdin). One event per poll: ``takeover``, ``handback``, ``success``, ``failure``, ``rerecord``, ``end``, ``next``, ``quit``, and for the confirm flow ``start`` (space), ``enter``, ``open`` (o), ``close`` (c). File EOF waits; pipe EOF before quit faults. Reset preserves unread events.
   * - ``composite``
     - ``CompositeTeleopSource``
     - a motion source plus an event source
     - the motion source's
     - kwargs: ``motion``, ``events`` (registry names), ``motion_kwargs``, ``events_kwargs``, ``intervention_from="events"`` (or ``"motion"``). hil_serl builds it from ``--teleop-event-device``.
   * - ``leader_cartesian``
     - ``LeaderArmCartesianSource``
     - a joint-space leader arm turned into a cartesian twist by Panda forward kinematics (the ``real_robot`` stack)
     - cartesian_delta, 6 + gripper
     - see \ :ref:`Leader arm on a cartesian robot <en-teleop-leader-arm-on-a-cartesian-robot-leader_cartesian>`\ .
   * - ``bimanual``
     - ``BimanualTeleopSource``
     - one single-arm source per arm of a ``real_robot_bimanual`` env (``--teleop-arm-device``)
     - the env's, both arms then both grippers
     - kwargs: ``device_per_arm``, ``per_arm_kwargs``, ``common_kwargs``, ``intervention_from="any"`` (or ``"all"``). See \ :ref:`Two arms <en-teleop-two-arms-bimanual>`\ .

.. _en-teleop-spacemouse:

SpaceMouse
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``--teleop-device spacemouse`` supports two modes, selected with
``--spacemouse-mode {tracker,serl}`` (top-level YAML ``spacemouse_mode``, default
``tracker``). Training, demo recording and real-robot reward collection share this
flag. Deploy's optional override defaults to ``None``: inherit the training mode,
or use ``tracker`` when there is no training configuration.

.. list-table::
   :header-rows: 1

   * - Behavior
     - ``tracker`` (default)
     - ``serl``
   * - Takeover
     - Button 0 toggles takeover/hand-back; moving the puck alone does not intervene.
     - Raw six-axis L2 norm strictly ``> 0.001`` intervenes, including rotation. No per-axis deadzone. Every step is judged anew; releasing the puck and gripper buttons returns control on the next idle step.
   * - Gripper
     - Button 1 toggles open/closed while driving; takeover preserves the last command.
     - Hold button 0 to close: a fresh uniform sample in ``[-1, -0.9)`` each step. Otherwise hold button 1 to open: ``uniform(0.9, 1)``. Both held: button 0 wins. Neither held: neutral ``0.0``. Buttons cause takeover only when the env has a gripper action.
   * - Arm target
     - Persistent command with target-speed and measured-pose lead limits.
     - Latest measured pose + raw action × env scale each step; rotation left-multiplies the measured quaternion. Clip to the station ``limits.tcp`` box (xyz) and ``safety_box_rpy_*`` (RPY) (roll clipped by magnitude, preserving sign). No time-step rescaling or tracker target.
   * - Gripper execution
     - Existing env behavior.
     - Scaled action ``<= -0.5`` closes only when measured normalized width ``> 0.85``; ``>= 0.5`` opens only when width ``< 0.85``. Commands require elapsed time strictly greater than ``gripper_sleep_s`` (default ``0.6 s``), then sleep for that interval. Zero is neutral.
   * - Terminal/keyboard
     - ``t``/``h`` ignored with a warning on state changes; labels/end/quit still work.
     - Same keys: ``s``/``f``/``r``/Esc/``q`` provide labels/end/quit. ``t``/``h`` do not control takeover and warn on state changes.

For an inclined robot base, ``teleop_options_yaml`` may supply ``input_rotation``, a
finite proper 3×3 rotation matrix ``base_R_input``. Tracker mode rotates
translation and angular input into the robot base frame (with no profile, the station
``task_frame`` rotation alone). Serl mode rotates by the station ``task_frame`` rotation only (no
profile yaw; with no ``task_frame`` it drives the raw base frame, like HIL-SERL) and keeps the
profile rotation as provenance, unless the options set ``use_profile_rotation: true`` (see
\ :ref:`SERL controller: SpaceMouse in serl mode <en-serl-controller-spacemouse-in-serl-mode-the-task-frame>`\ ).
Axis inversions and the tracker's per-axis deadzone precede rotation; SERL takeover still uses
the raw six-axis norm. Each rotated three-vector is uniformly reduced only when needed
to fit the action cube, preserving its direction. No origin translation is
applied. The resolved matrix and profile name/file/yaw are saved in non-compared
``teleop_input`` provenance before resolved configuration is persisted. Restoring a run uses
those numeric input options without rereading a changed profile. Recorded actions are in
base coordinates, so operator recalibration does not invalidate old demos or resume contracts.
The station ``limits.tcp`` box lives in the robot base frame (its axes are the rotation of the
station ``task_frame``, see :ref:`Limits <en-devices-limits>`); input rotation alone does not move any
safety boundary. Neither changes
the robot's mounting/gravity configuration or payload model.

The SpaceMouse tracker clips the requested position into the station's ``limits.tcp`` box before it
limits speed and lead, and again on its command pose, so the command slides along a wall
instead of stopping; nothing is handed back and ``WORKSPACE HOLD`` is no longer logged. The
orientation is not limited. In training, policy targets go through the same device clip. SERL
applies its original XYZ/RPY clipping first (``safety_box_rpy_*``), then the device clip.
Measured TCP feedback outside the box plus ``measured_margin_m`` (default ``0.003``) is a latched
fault. Device and environment constraints remain authoritative for both teleoperation modes and
policy actions; joint modes (GELLO) stop at the wall in the device using forward kinematics.

For a named operator profile, use the \ :ref:`CLI calibration workflow <en-spacemouse-commissioning-spacemouse-profiles-and-task-frame-workspace>`\ .
The composed input rotation is ``base_R_task @ Rz(operator_yaw)`` for both
translation and angular vectors. The fixed HID axis map is a device convention,
not a measured mounting transform. Do not add an extra roll correction to an
already calibrated ``base_R_task``. Absolute positions use the full task transform,
including its origin; feedback observations and policy actions retain their
existing base-frame contract.

For SERL, configure the station's ``limits.tcp`` box and the RPY range first. Missing
either one is a construction error. The XYZ box is the station ``limits.tcp`` (optionally tightened by
``real_robot.limits.tcp``); the RPY range is the training option ``safety_box_rpy_low`` and
``safety_box_rpy_high`` on ``real_robot``, which belongs to the HIL-SERL action semantics and not to the
station limits; scales remain ``xyz_scale_m``/``rot_scale_rad``.
The mode selects ``cartesian_semantics: serl`` before env construction even on a
learner that opens no teleop device. An explicitly configured ``current`` conflicts
with this mode; an unset programmatic dataclass default is promoted to ``serl``.
Mode resolution finishes before training/record args are serialized, so saved
SERL configs contain both ``spacemouse_mode: serl`` and ``cartesian_semantics: serl``.
Deploy preserves the saved env semantics; a ``--spacemouse-mode serl`` override
conflicts with a saved ``cartesian_semantics: current``.

Use the same mode for recording, training and deployment. SERL records this
compared ``teleop_control`` contract (nondefault ``invert_axes`` adds that list, and options that
carry an ``input_rotation`` add ``use_profile_rotation``):

.. code-block:: json
   :dedent: 0

   {"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, "gripper": "serl_buttons", "input_frame": "base"}

``input_frame`` is ``base`` without a task frame, ``task_frame`` with one, or
``profile`` when ``use_profile_rotation: true``; diagnostic filters/gain also enter
the block when configured.

SERL requires one six-axis ``cartesian_delta`` arm and full arm/gripper selection.
Actions stay in the base frame after the fixed HID map
``[-y, x, z, -roll, -pitch, -yaw]`` followed by the task-frame rotation by default
(or the profile rotation with ``use_profile_rotation: true``); relative frames, relative observations and ``action_indices`` are not supported.
``teleop_options_yaml`` may set ``device_index`` (default ``0``), ``invert_axes``
(default ``[]``), ``poll_interval_s`` (default ``0.005``) and ``read_timeout_s``
(default ``1.0``). Tracker options, ``deadzone``, ``hold_s``, gripper-holding getters
and a nested ``mode`` key are rejected in this mode. Use the top-level mode flag.
The device layer's ``max_delta_xyz_m`` and pose-box limits still apply and may be
tighter than native HIL-SERL. Hardware behavior and installation remain
**unverified**. ``spacemouse_twist`` remains the separate sim-only source, with
different button and sticky-gripper behavior; it is not SERL mode.

.. _en-teleop-spacemouse-installation-and-linux-permissions:

SpaceMouse installation and Linux permissions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

On the NUC (or the machine with the SpaceMouse), repository root, with the
\ :doc:`project environment from tutorial step 4 <../tutorials/install>`\ 
active, install all SpaceMouse Python dependencies with:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   pip install -e '.[spacemouse]'

The ``spacemouse`` extra installs ``hidapi==0.14.0.post4``
(trezor/cython-hidapi, import ``hid``). The device layer includes its own decoder;
this path does not import ``pyspacemouse`` or ``easyhid``. Do not install the different
PyPI distribution named ``hid``; it occupies the same import name and has a different
API. A missing module raises ``SdkMissing`` with the pip command above; the wrong
``hid`` distribution is reported separately.

For a fresh environment, on the NUC, repository root, no environment active:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   bash scripts/install.sh --extras learning,franka-gello,spacemouse
   source .venv/bin/activate

The one-extra pip command adds HID support to an existing installation.
Recording and training also need the ``learning`` extra and the ``rl-garden`` submodule;
the installer command above installs both local projects. The wheel
target is Ubuntu 22.04 / CPython 3.10 / x86_64. This transport avoids easyhid's
system ``libhidapi`` lookup; the pinned wheel's ELF/system-library dependencies
still require validation on a clean Ubuntu 22.04 machine. Wheel installation,
USB permissions, device directions, timing and disconnect behavior are
**not yet verified on hardware**. No system ``libhidapi`` install is prescribed for this wheel
route; a source build on a platform without a suitable wheel is outside that target.

First run ``rwrl --station local/station.yaml doctor --fix`` on the NUC, repository
root, project environment active, with the SpaceMouse plugged in and the station
naming a ``spacemouse`` teleop. When the USB node is not readable and writable it
proposes, after one confirmation, the ``plugdev`` group and an
``/etc/udev/rules.d/99-rwrl-spacemouse.rules`` rule for the supported vendor/product
IDs; replug the SpaceMouse and log in again when it reports a pending action
(\ :ref:`cli.md <en-cli-robot-host-diagnosis-and-repair>`\ ). The manual installation below
is the alternative without the project environment.

Install the packaged vendor rules once. On the NUC, repository root, any shell
(no project environment required), with sudo permission:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   sudo cp real_world_rl/teleop/spacemouse/udev/99-spacemouse.rules /etc/udev/rules.d/
   sudo usermod -aG plugdev "$USER"
   sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=hidraw

The file covers vendor ``256f`` (3Dconnexion) and ``046d`` (older Logitech), with
``MODE="0664", GROUP="plugdev"``. Log out and log back in, then unplug and replug the
SpaceMouse. If the repository is unavailable, the alternative copy command below
works on the NUC from any directory, **with the project environment active** and
sudo permission; follow it with the same group/reload commands above:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   sudo cp "$(python -c 'import os, real_world_rl.teleop.spacemouse as slice; print(os.path.dirname(slice.__file__))')/udev/99-spacemouse.rules" /etc/udev/rules.d/

An open/permission failure points to this rule file and its installation commands.
Doctor checks the ``hid`` SDK separately from ``/dev/hidraw*`` access; inside Docker the
HID node must also be passed into the container. The legacy ``SpaceMouseExpert`` and
``legacy-bridge`` extra keep their separate ``pyspacemouse`` API.

On the NUC, repository root, with the project environment active and the station's
``hil.yaml`` configured, for a ``cartesian_delta`` env (``real_robot`` in ``cartesian_delta`` mode), with the
operator's labels on the terminal (the SpaceMouse Compact has two buttons, both taken):

.. code-block:: bash
   :dedent: 0

   rwrl record --config hil.yaml \
       --teleop-device spacemouse --teleop-event-device stdin \
       --output-dir demos/task_20 --num-successes 20

The HIL actor takes the same two flags. Add ``--spacemouse-mode serl`` to select
the native behavior in the table above, after configuring the ``limits.tcp`` box and the RPY range.
The following description is the default **tracker** mode. ``SpaceMouseTeleopSource``
(``real_world_rl/teleop/spacemouse/source.py``) opens the device with the device layer's
``SpaceMouseTeleop`` (``TeleopConfig(backend="spacemouse")``, a poll thread feeding ``read()``; the
twist is in ``[-1, 1]`` with axis map ``[-y, x, z, -roll, -pitch, -yaw]`` of the HID reader state,
``invert_axes`` flips the sign of the listed output indices 0-5) and drives the same
``CartesianTargetTracker`` as ``leader_cartesian`` (see \ :ref:`Mapping <en-teleop-mapping>`\ ):

.. list-table::
   :header-rows: 1

   * - Input
     - Effect
   * - button 0
     - take over; the next press hands back. Moving the puck alone never intervenes, so a nudge does not take over.
   * - button 1
     - while driving: toggle the gripper between open (``+1``) and closed (``-1``)
   * - puck
     - while driving: a velocity (below)
   * - ``s``/``f``/``r``, Esc, ``q`` on ``--teleop-event-device stdin`` (or ``keyboard``)
     - ``success``/``failure``/``rerecord``, episode end, quit

-  **Velocity.** Each intervened step requests the tracker's command moved by
   ``twist * target_speed * dt`` (``dt`` the time since the previous intervened step, 0 at a take-over,
   so taking over never jumps): translation ``twist[:3] * --teleop-target-speed-m-s * dt`` in the
   base frame, rotation vector ``twist[3:] * --teleop-target-speed-rad-s * dt`` left-multiplied, as
   the env's step. Full deflection is the target speed (0.05 m/s, 0.3 rad/s); the tracker limits a
   diagonal to it. Each axis within ``deadzone`` (0.05 of full scale, above the Compact's rest noise)
   counts as zero.
-  **Axis lock.** ``axis_lock: dominant`` (default) keeps whichever group has the larger peak input, translation or rotation, and zeros the other. ``axis_lock: "off"`` passes both groups. This option is part of ``teleop_control``.
-  **Lead cap.** The command never leads the measured TCP pose by more than
   ``--teleop-lead-cap-m``/``--teleop-lead-cap-rad`` (0.05 m, 0.3 rad); the excess is dropped. The
   request starts from the command, not from the previous request, so an arm that lags behind
   never lets it run ahead: release the puck and the arm stops at the command, at most one lead cap
   from where it was. The env's ``xyz_scale_m``/``rot_scale_rad`` must be at least the lead caps
   (``--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3``; the ``real_robot`` defaults
   0.02/0.1 are rejected when the source is built).
-  **Hand-back** holds the measured pose; the recorder's hold policy shares the tracker and keeps
   the arm on that pose (not a zero twist) until the next take-over. An input or tracking failure hands back as
   well (a requested position outside the ``limits.tcp`` box does not: it is clipped and the command slides along the wall): press button 0 to take over again. A stalled control loop (more than 0.3 s between two
   intervened steps) holds the measured pose and the next step continues from there.
-  **Gripper.** At each take-over the gripper action starts as the gripper's last command (the
   intervention wrapper's ``last_gripper_action_getter``, kept as is, so taking over never opens a
   closed gripper); button 1 then switches to the other state. Button 1 does nothing while handed
   back.
-  **Buttons act on press, once.** The device layer latches every press until the next read, so a
   press shorter than one env step is not lost; a held button counts once, and a button held
   through an env reset does not count.
-  **The event device's** ``t``\ **/**\ ``h`` **are ignored** (a warning says so): they set a take-over state,
   which can disagree with a take-over made by button 0. Button 0 is the only toggle.
-  The SDK is imported when ``start()`` opens the device (the wrapper calls it), so building the
   source works without ``hidapi``; a missing SDK raises ``SdkMissing`` with the install hint.

A run with ``--teleop-device spacemouse`` records the tracker in the experiment contract
(``teleop_control``: the tracker values plus ``axis_lock``, and applicable input-frame/diagnostic settings). ``--teleop-event-device spacemouse`` was
removed (the SpaceMouse no longer provides labels); an old command line or preset with it fails
with a message pointing at ``--teleop-device spacemouse --teleop-event-device stdin``.

The robot-yaml route (``--teleop-device robot`` with a ``backend: spacemouse`` teleop section) reads
the same device through the robot (and its daemon) with the device layer's own, older semantics:
the twist goes to the env as is, the operator intervenes while the puck moves or a button is held
(``hold_s`` keeps it going), and button 0 toggles the device's sticky gripper. Neither button ends the episode on these real-robot routes; episode end is Esc
on the event device. The sim-only ``spacemouse_twist`` uses button 1 for episode end.

.. _en-teleop-robot-teleop-part---teleop-device-robot:

Robot teleop part (``--teleop-device robot``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

hil_serl on ``--env-backend real_robot`` can also take interventions from the teleop part of
the robot itself (the robot yaml's ``teleop:`` section: spacemouse, the ``gello`` leader arm or the
``fake`` backend, served locally or by the robot daemon; a leader arm is better read with
``--teleop-device leader_arm``, see \ :ref:`Leader arms <en-teleop-leader-arms-gello>`\ ). ``TeleopPartSource``
(``real_world_rl/teleop/core/part.py``) adapts it, mapping cartesian or joint
(``joint_abs``/``joint_delta``) teleop actions into the env's ``[-1, 1]`` action space and forwarding
the device's ``success``/``failure``/``rerecord`` events; see the README's "Action contract" and
"Teleop (``--teleop-device robot``)" sections for the mapping, the ``--robot-teleop-action-mode``
declaration and an example yaml.

.. _en-teleop-leader-arms-gello:

Leader arms (GELLO)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The end-to-end guide (hardware, calibration, robot yaml, demo recording and the HIL commands for
``hil_serl``, ``residual_hil_serl`` and ``silri``) is \ :doc:`gello.md <gello>`\ ; the first-day walkthrough
with the setup wizard, BC, ACT and deployment is the
\ :doc:`tutorial <../index>`\ . This section describes the sources.

A leader arm reports absolute leader joint positions. ``LeaderArmPartSource``
(``real_world_rl/teleop/leader/joint.py``) maps them into the env's joint action mode like
``TeleopPartSource`` (``joint_abs``: normalized onto ``limits.joints``; ``joint_delta``:
``(q_leader - q_follower) / joint_scale_rad``, saturating at one ``joint_scale_rad`` per step) and
adds an onset policy for the moment the operator takes over. Two ways to read a GELLO:

-  ``--teleop-device leader_arm``: the GELLO is the ``teleop:`` part of the robot yaml the daemon
   serves on the robot PC (``rwrl daemon --station robot.yaml ...``), read through
   ``RemoteRobot``. ``rwrl setup`` writes the section (calibration included) into a
   station yaml; or add it to the robot yaml by hand (for example
   ``examples/stations/robot.franka.example.yaml``);
   ``rwrl --station robot.yaml teleop calibrate --backend gello --port ... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8``
   prints the ``options`` block for a unit (unset flags come from the station's ``teleop.options``;
   ``--use`` writes the block into the station, see \ :ref:`gello.md <en-gello-2-calibration>`\ ):

   .. code-block:: yaml
      :dedent: 0

      teleop:
        name: gello
        backend: gello
        options:
          port: /dev/serial/by-id/usb-FTDI_...
          joint_ids: [1, 2, 3, 4, 5, 6, 7]
          joint_offsets_rad: [...]    # from teleop calibrate --backend gello
          joint_signs: [1, 1, -1, 1, 1, 1, 1]
          gripper_id: 8
          gripper_open_rad: ...
          gripper_closed_rad: ...
          intervention: toggle        # or always (pure teleop)
          event_device: /dev/input/by-id/...   # optional evdev pedal/keyboard on the robot PC

-  ``--teleop-device gello`` (``GelloTeleopSource``): a GELLO plugged into the actor's machine, opened
   through the same device-layer backend with the options above as source kwargs; from the
   training CLI they come from ``--teleop-options-yaml`` (above).

HIL recipe over the daemon (a ``joint_delta`` follower; ``--robot-teleop-action-mode`` only has to
name a joint space for a leader arm, it declares the part before the remote robot connects):

.. code-block:: bash
   :dedent: 0

   rwrl train hil_serl --role actor --env-backend real_robot \
       --real_robot.remote ROBOT_PC:50051 --real_robot.action_mode joint_delta \
       --teleop-device leader_arm --robot-teleop-action-mode joint_delta \
       --teleop-onset ramp --teleop-event-device keyboard

Onset (``--teleop-onset``, source kwarg ``onset``):

-  ``ramp`` (default): in ``joint_abs`` the emitted target is
   ``q_follower + clip(q_leader - q_follower, +-ramp_step_rad)`` (``--teleop-ramp-step-rad``, 0.05),
   normalized onto the joint limits, so the recorded action is the executed command; those
   intervened steps carry ``info["teleop_events"]["teleop_syncing"] = True``. The dataset retains
   them, while training excludes syncing expert labels. In ``joint_delta``, initial catch-up is
   marked until the leader first comes within ``joint_scale_rad``; later saturated motion of the
   same takeover is ordinary teaching.
-  ``arm``: the toggle only takes effect once ``|q_leader - q_follower|_inf <= arm_tolerance_rad``
   (``--teleop-arm-tolerance-rad``, 0.1). Until then the step is not intervened and the per-joint
   error is logged (at most once per second): move the leader onto the follower. Once armed the
   source stays armed until the toggle releases; every env reset disarms it.

``q_follower`` is the ``joint_pos`` of the env's latest observation, so ``state_fields`` must contain
``joint_pos``. The toggle is the part's ``intervening`` (``intervention: toggle`` with an evdev
``event_device`` on the robot PC, or ``always``) unless an event device supplies it:

``--teleop-event-device keyboard`` (or ``stdin``) wraps the motion device in a
``CompositeTeleopSource``: motion from ``--teleop-device``, the intervention state and the
``episode_end``/``success``/``failure``/``rerecord``/``operator_quit`` events from the event device on the
actor's machine (keyboard and stdin: ``t`` takes over, ``h`` hands back, Esc ends the episode,
``s``/``f``/``r``, ``q`` quits). The leader source still applies its onset policy to that state; the
part's own ``intervention`` setting is then ignored. (``--teleop-device spacemouse`` keeps its own
take-over on button 0 and takes only the events from the event device, see
\ :ref:`SpaceMouse <en-teleop-spacemouse>`\ .)

.. _en-teleop-confirm-keys:

``episode_end_behavior: confirm`` (the default of the recorder and ``rwrl teleop run``) reads five more keys between episodes, handed in order by the event device to ``ConfirmSession``. Once the source receives ``enable_confirm_keys()`` it keeps the order of key presses (at most 64), and inside an episode these keys do nothing:

.. list-table::
   :header-rows: 1
   :widths: 14 18 68

   * - Key
     - Event name
     - Effect
   * - space
     - ``start_episode``
     - READY: start the episode
   * - ``n``
     - ``next_episode``
     - POST_EPISODE: ask for the reset (a partly closed gripper enters GRIPPER_CHECK first)
   * - ``o``/``c``
     - ``gripper_open``/``gripper_close``
     - POST_EPISODE, GRIPPER_CHECK: open / close the gripper (only ``o`` in GRIPPER_CHECK, which opens in place first)
   * - Enter
     - ``confirm``
     - GRIPPER_CHECK: confirm and reset now
   * - ``q``
     - ``operator_quit``
     - Quit in any phase (the existing key)

The event device is ``keyboard``, ``stdin``, ``jsonl``, or a GELLO with an ``event_device`` (evdev keys in :doc:`gello`). Without such a device confirm fails at startup. Phases, the gripper check and the options are in :ref:`Episode boundaries <en-recording-demos-episode-boundaries>`.

.. _en-teleop-leader-arm-on-a-cartesian-robot-leader_cartesian:

Leader arm on a cartesian robot (``leader_cartesian``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A leader arm reports joint angles. Envs that only accept a cartesian twist get those
angles through selectable forward kinematics (default ``kinematics: panda``): ``LeaderArmCartesianSource``
(``real_world_rl/teleop/leader/cartesian.py``, ``device="leader_cartesian"``) turns the
leader's joints into a requested end-effector pose, and a ``CartesianTargetTracker``
(``real_world_rl/teleop/core/targets/cartesian_tracker.py``) turns that into the twist that moves the follower
(see \ :ref:`Mapping <en-teleop-mapping>`\ ). ``examples/presets/gello_cartesian.yaml`` is a single-arm 50 Hz preset.

.. _en-teleop-why:

Why
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A ``real_robot`` with a ``serl_http`` arm talks to the SERL HTTP bridge, which only knows ``/pose`` (an
absolute cartesian impedance target), ``/open_gripper``/``/close_gripper``, ``/jointreset``,
``/startimp``/``/stopimp`` and ``/getstate``. There is no streaming joint command, so the joint-space
leader sources (``--teleop-device leader_arm``/``gello``, see \ :ref:`Leader arms <en-teleop-leader-arms-gello>`\ ) cannot drive it, and
``real_robot`` in cartesian_delta mode takes a 7-D action: base-frame twist (6) + gripper. The conversion therefore
happens on the actor side:

.. code-block:: text
   :dedent: 0

   GELLO joints q --PandaKinematics--> T* = (p*, R*) --vs follower TCP (p, R)--> twist in [-1, 1]

The same source serves ``real_robot`` in ``cartesian_delta`` mode (for example a daemon with
``arm_backend: serl_http``).

.. _en-teleop-pieces:

Pieces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1

   * - Piece
     - Where
     - What
   * - ``PandaKinematics``
     - ``real_world_rl/kinematics/panda.py``
     - numpy forward kinematics, Craig modified DH; ``flange_pose(q)``, ``ee_pose(q)`` (xyz, quat xyzw), ``ee_transform(q)`` (4x4). EE defaults: Franka Hand, 0.1034 m along the flange z and -pi/4 about it (``ee_offset_m``, ``ee_yaw_rad``).
   * - ``LeaderArmCartesianSource``
     - ``real_world_rl/teleop/leader/cartesian.py``
     - the teleop source; any ``joint_abs`` part matching the selected kinematics' joint count is a valid leader.
   * - ``RealRobotEnv.last_observation``
     - ``real_world_rl/envs/real_robot/env.py``
     - the measured TCP pose (``O_T_EE``, xyz + quat xyzw) of the latest reset/step; the source's follower pose.
   * - stub-robot yaml
     - ``examples/stations/robot.gello_station.example.yaml``
     - serves the GELLO from the robot host through the robot daemon.

.. _en-teleop-the-stub-robot-daemon-on-the-robot-host:

The stub-robot daemon on the robot host
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The GELLO is plugged into the robot host (the machine that runs the SERL bridge). A stub robot,
``type: franka`` with ``arm_backend: fake``, no gripper and no cameras, carries it as its ``teleop:``
part, and the robot daemon serves it:

.. tutorial-test: skip
.. code-block:: bash

   # on the robot host, next to the SERL bridge
   pip install -e ".[gello]"                    # + [gello-events] for an evdev pedal/keyboard

.. code-block:: bash

   rwrl --station robot.gello_station.yaml teleop calibrate --backend gello \
       --port /dev/serial/by-id/usb-FTDI_... --joint-ids 1 2 3 4 5 6 7 --gripper-id 8 \
       --use                                                # writes the options into the yaml
   rwrl daemon --station robot.gello_station.yaml --bind 0.0.0.0:50052

The fake arm holds no hardware and is never commanded, so the stub does not compete with the
bridge for the robot. Use a port other than any real robot daemon on the same host.

.. _en-teleop-actor-side:

Actor side
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The source needs exactly one leader:

-  ``remote_address="host:port"`` (+ optional ``remote_token``): a ``RemoteRobot`` for the stub daemon,
   connected by ``start()`` (the intervention wrapper calls it) and disconnected by ``close()``;
-  ``robot_yaml=<path>``: the same stub yaml loaded on this machine (the GELLO plugged into the
   actor's machine), connected by ``start()``;
-  ``leader=<part or resolver>``: a teleop part the caller owns (tests, custom setups).

The follower pose, the per-step scales and the ``limits.tcp`` box come from the env:
on a ``real_robot`` cartesian env, ``last_observation["state"]["tcp_pose"]``, the action
contract's ``xyz_scale_m``/``rot_scale_rad`` and the effective ``limits.tcp`` box (``None``: the source does not clip). Pass ``follower_pose_getter=``,
``pos_scale_m=``, ``rot_scale_rad=`` and ``limits=`` (a ``TcpBox``) to override them. The scales must be at least
the lead caps (``xyz_scale_m >= lead_cap_m``, ``rot_scale_rad >= lead_cap_rad``), else the source
refuses to build: the default scales (0.02 m / 0.1 rad) need either larger ones
(``--real-robot.xyz-scale-m 0.05 --real-robot.rot-scale-rad 0.3`` as in the preset) or lower caps.

On ``real_robot``, the ``task_frame`` is read from the env config, falling back to its local robot
YAML, as for Pico; the box comes from the env's effective limits. Explicit source kwargs override
them. A request outside the ``limits.tcp`` box is clipped into it, so the command slides along the
wall; there is no hand-back, no ``WORKSPACE HOLD`` log and no re-anchor.

.. code-block:: python
   :dedent: 0

   from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper

   env = TeleopInterventionWrapper(
       real_robot_env,
       device="leader_cartesian",
       remote_address="robot-host:50052",
       mapping="anchored",
   )

From the training CLI, select the source with ``--teleop-device leader_cartesian`` and pass its
kwargs as a yaml mapping with ``--teleop-options-yaml`` (merged into the motion source's kwargs):

.. code-block:: yaml
   :dedent: 0

   # station.yaml (actor side)
   remote_address: robot-host:50052   # the stub daemon above; or robot_yaml: <path> for a local GELLO
   mapping: anchored                  # or absolute
   gripper_binarize_threshold: 0.5

.. code-block:: bash
   :dedent: 0

   rwrl train hil_serl --role actor --env-backend real_robot --config hil.yaml \
       --teleop-device leader_cartesian --teleop-options-yaml station.yaml

With the default ``intervention: always`` the operator drives on every step (pure teleop or demo
collection). For HIL, set ``intervention: toggle`` with an ``event_device`` on the robot host, or
take the intervention from an actor-side event device (``--teleop-event-device stdin`` or
``keyboard``: ``t`` takes over, ``h`` hands back; see \ :ref:`Leader arms <en-teleop-leader-arms-gello>`\ ).

.. _en-teleop-a-second-arm-as-the-leader:

A second arm as the leader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The device-layer ``arm_leader`` Part reads a second arm through ``Arm.backend(arm.type)``.
It reports ``joint_abs`` joint positions without sending motion commands. This path **needs
hardware verification**, including manual guiding, FK/TCP agreement and gripper readings.
Connecting the Part does not enable a guiding mode; arrange the leader's manual operation
using its backend/vendor procedure.

Use \ |reference-link-1|_\ 
as the robot YAML on the second arm's host. Its outer ``arm_backend: fake``, ``gripper: null``
and ``cameras: []`` make the robot a carrier; only the teleop Part opens the real leader:

.. code-block:: yaml
   :dedent: 0

   type: franka
   name: franka_leader_station
   arm_backend: fake
   gripper: null
   cameras: []
   teleop:
     name: leader
     backend: arm_leader
     options:
       arm: {type: franky, robot_ip: 172.16.0.2, realtime: ignore}
       gripper: false
       intervention: always

``arm`` is required; its remaining nested keys go to that backend's constructor. ``gripper``
defaults to ``false`` (no channel). With ``true``, the matching ``Gripper`` backend supplies its
width normalized to ``[0, 1]`` (1 = open); shared constructor options such as ``robot_ip`` are
forwarded, and integrated grippers borrow the arm session. ``intervention: always`` is the
default and emits ``active=True``. ``intervention: event`` emits ``active=False``; the composite
event device's toggle decides take-over. Neither mode supplies buttons or events itself.

On the second arm's host, from the repository root in the venv with the arm SDK and daemon
dependencies installed, after editing the example's IP:

.. code-block:: bash
   :dedent: 0

   rwrl daemon --station examples/stations/leader.franka-as-leader.example.yaml --bind 0.0.0.0:50052

On the follower's actor host, save these source options as ``leader_cartesian.yaml`` in the
repository root (replace ``leader-host`` with the second arm's host):

.. code-block:: yaml
   :dedent: 0

   remote_address: leader-host:50052
   mapping: anchored
   kinematics: panda

In that host's learning venv, add ``--teleop-device leader_cartesian --teleop-options-yaml leader_cartesian.yaml --teleop-event-device stdin`` to the follower's
training or recording command. The terminal's ``t`` takes over and ``h`` hands back. A leader
without a gripper holds the follower's last commanded gripper action.

``kinematics`` accepts a registered name in YAML or a kinematics instance in Python. The
registry exports ``Kinematics`` (``NUM_JOINTS``, ``ee_transform(q)`` returning a 4×4 transform),
``register(name)``, ``kinematics_for(name)`` and ``kinematics_names()`` from
``real_world_rl.kinematics``. Only ``panda`` is built in; register a zero-argument implementation
for another arm and match its joint count. An explicitly configured name is saved as
``teleop_input.kinematics`` provenance and retained in ``source_options``; it does not enter
the compared ``teleop_control`` block. Omitting it preserves the existing default snapshot.

For the episode-boundary session gate, **toggle released = neutral**. ``session_input()``
consumes one leader reading without advancing the tracker or changing its anchor. Its six
axes are the normalized requested-pose lead while take-over is requested (position divided
by ``lead_cap_m``, rotation vector by ``lead_cap_rad``, clipped to ``[-1, 1]``), otherwise zeros;
``buttons["toggle"]`` carries the device toggle level, falling back to the request level for
leaders without an explicit toggle reading. Before the first anchor, the request uses the
leader's FK pose. A held toggle remains non-neutral even when the lead is zero. GELLO
``intervention: always`` reports no held toggle: neutral means the normalized lead is within
the session threshold. Supervised GELLO sessions use terminal ``t``/``h`` for take-over/hand-back
during an episode.

.. _en-teleop-mapping:

Mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``RealRobotEnv`` in ``cartesian_delta`` sends ``measured + action * scale``, and an impedance backend
(Franky's cartesian impedance tracker) treats that pose as its equilibrium. A twist recomputed
from the measured pose every step therefore never holds still: a zero action re-anchors the
equilibrium wherever the arm drifted (137 mm in 24 s on the first hardware session). The source
keeps the target instead, in a ``CartesianTargetTracker`` per arm:

-  a **hold pose**, sampled once at the env reset and once at every hand-back (the measured pose
   at that moment), then fixed while nobody drives;
-  a persistent **command pose**. At the take-over it is the current command (the hold pose: no
   jump); while the operator drives it moves towards the requested pose by at most
   ``--teleop-target-speed-m-s`` / ``--teleop-target-speed-rad-s`` (0.05 m/s, 0.3 rad/s) and then is
   pulled back to within ``--teleop-lead-cap-m`` / ``--teleop-lead-cap-rad`` (0.05 m, 0.3 rad) of the
   measured pose. The excess is dropped, never queued: a leader that jumps and comes back does not
   make the follower overshoot.

The emitted arm action lands the follower on the command, in the env's normalized units:

.. code-block:: text
   :dedent: 0

   a_pos = (p_cmd - p) / pos_scale_m
   a_rot = rotvec(R_cmd R^T) / rot_scale_rad

the inverse of the env's step (``p + a_pos * pos_scale_m``, and ``from_rotvec(a_rot * rot_scale_rad) * R``: the rotation is left-multiplied, so the rotation vector is in the base
frame). With the scales at least the lead caps the action never leaves ``[-1, 1]``; the tracker
raises (``CommandOutOfRange``) rather than clip one that would. A hand-back, an input or tracking
failure and an episode end put the tracker in hold at the measured pose. A stalled control loop
(more than 0.3 s between two intervened polls) counts as an input loss: the follower holds, a
warning gives the gap, and the next poll re-anchors the leader as at a take-over (a clock going
backwards raises ``MissedControlDeadline``). A requested position outside the ``limits.tcp``
box is not a hold: it is clipped into the box and the command slides along the wall.

The recorder's hold policy (``real_world_rl/teleop/core/hold.py``, ``CartesianHold``) shares the source's
tracker: while nobody drives it sends the action back onto the fixed hold pose (not a zero twist),
at most one lead cap per step. Without a tracker-providing source (``leader_cartesian``, ``spacemouse`` tracker
or ``pico``) it keeps a tracker of its own whose caps
are ``min(0.05 m, xyz_scale_m)`` / ``min(0.3 rad, rot_scale_rad)``, so an arm pushed far off the hold
gets a saturated action back, never an error.
A run with ``--teleop-device leader_cartesian`` (or ``spacemouse``) records the tracker in the
experiment contract (``teleop_control``: version ``cartesian_tracker_v1`` and the four values above,
compared): demos recorded without it, or with other values, do not load into it. SpaceMouse additionally records ``axis_lock`` and applicable input-frame/diagnostic settings.
Matching the four tracker flags alone therefore does not make GELLO and SpaceMouse
demos compatible; their complete compared contracts must match.

``mapping`` chooses the requested pose:

-  ``anchored`` (default): at each take-over (the first intervened poll after a non-intervened one,
   and the first after an env reset) the source stores the tracker's command ``(p_c0, R_c0)`` and
   the leader pose ``(p_l0, R_l0) = FK(q_0)``. Afterwards the leader's base-frame displacement is
   applied to the command, translation and rotation decoupled:

   .. code-block:: text
      :dedent: 0

      p* = p_c0 + (p_l - p_l0)
      R* = (R_l R_l0^T) R_c0

   The take-over keeps the hold action, so taking over never jerks the robot. A leader wrist
   rotation turns the follower in place about its own TCP, however far apart the two poses are
   (the composed form ``FK(q) T_l0^{-1} T_f0`` would instead swing the follower about the base
   origin, translating it by about ``|p_f0| * theta``). FK errors only enter through how they change
   since onset. Leader and follower postures can differ (re-grab the leader between
   interventions like a clutch); the elbow / null-space motion of the leader is ignored, as in any
   cartesian command.

-  ``absolute``: ``T* = FK(q)``. The follower goes to wherever the leader's end effector is, which
   needs ``PandaKinematics.ee_pose`` to match the robot's ``O_T_EE`` (verify it, below) and the leader
   to be posed near the follower before taking over, otherwise the follower travels from the hold
   pose to the leader's pose at the target speed.

Prefer ``anchored`` unless the leader is calibrated and verified against the robot.

.. _en-teleop-gripper:

Gripper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The leader's width target ``w`` in ``[0, 1]`` (1 = open) becomes ``+1`` (open) if
``w > gripper_binarize_threshold`` and ``-1`` (close) otherwise; the default 0.5 closes the gripper
once the trigger passes half travel. The exact +-1 values are clean labels for the hybrid
(discrete) gripper critic, and ``real_robot`` executes them unambiguously;
``gripper_binarize_threshold=None`` emits the continuous ``2w - 1`` instead. A leader
without a gripper channel holds the gripper action last sent to the env, which needs
``last_gripper_action_getter`` (as for ``--teleop-device robot``).

When the trigger disagrees with the gripper's last commanded state at the take-over (open:
action > 0; the intervention wrapper supplies it through ``last_gripper_action_getter``), the
source keeps sending that last action until the trigger crosses to the gripper's state, and only
then follows the trigger. Taking over with an open trigger therefore never drops a grasped
object.

.. _en-teleop-verify-the-dh-table-on-the-robot:

Verify the DH table on the robot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``PandaKinematics`` matches Franka's published reference poses (flange at ``q = 0``:
``(0.088, 0, 0.926)``, z down; flange at the ready pose ``[0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4]``:
about ``(0.307, 0, 0.590)``), but has not been compared with a real robot yet. Before relying on
``mapping="absolute"``, compare it with the bridge's own pose at a few joint configurations
(the ready pose, and two or three poses spread over the work area with the wrist rotated):

.. code-block:: python
   :dedent: 0

   import numpy as np
   from scipy.spatial.transform import Rotation

   from real_world_rl.server import RemoteRobot, RemoteRobotConfig
   from real_world_rl.kinematics import PandaKinematics

   # Use a daemon with joint feedback (e.g. a franky arm).
   robot = RemoteRobot(RemoteRobotConfig(address="robot-host:50051"))
   robot.connect()
   try:
       state = robot.get_observation()["state"]
   finally:
       robot.disconnect()
   position, quat = PandaKinematics().ee_pose(state["joint_pos"])
   pose = np.asarray(state["tcp_pose"])
   print("position error [m]:", np.linalg.norm(position - pose[:3]))
   print("rotation error [rad]:", (Rotation.from_quat(quat) * Rotation.from_quat(pose[3:]).inv()).magnitude())

Expect errors below about 1 mm and 0.01 rad. A constant offset along the tool axis or about it
means Desk's end-effector setting differs from the Franka Hand default: pass
``kinematics=PandaKinematics(ee_offset_m=..., ee_yaw_rad=...)`` to the source. Errors that change
with the configuration point at the DH table itself; ``anchored`` mapping is less sensitive to
both.

.. _en-teleop-two-arms-bimanual:

Two arms (``bimanual``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

On ``--env-backend real_robot_bimanual`` (see \ :doc:`bimanual.md <bimanual>`\ ) the action is
``[left arm, right arm, left gripper, right gripper]``, and the single-arm devices (``robot``,
``leader_arm``, ``gello``, ``leader_cartesian``, ``spacemouse``, ``spacemouse_twist``, ``pico``) are rejected. ``--teleop-device bimanual`` (``BimanualTeleopSource``, ``real_world_rl/teleop/core/bimanual.py``) builds one
single-arm source per arm over that arm's own ``RealRobotEnv``, so each one reads its own robot and
follower, and assembles their readings into the two-arm action. ``--teleop-arm-device`` picks the
per-arm device:

.. list-table::
   :header-rows: 1

   * - ``--teleop-arm-device``
     - Each arm's source
     - Notes
   * - ``leader_arm`` (default)
     - ``LeaderArmPartSource`` over that arm robot's teleop part
     - a GELLO served by that arm's daemon (its robot yaml's ``teleop:`` section); ``--robot-teleop-action-mode`` declares both parts before the remote robots connect; the ``--teleop-onset`` flags apply to every arm
   * - ``robot``
     - ``TeleopPartSource`` over that arm robot's teleop part
     - any teleop part, mapped as for ``--teleop-device robot``
   * - ``gello``
     - ``GelloTeleopSource``
     - a GELLO plugged into the actor's machine, one per arm; each needs its own ``port`` and calibration (below)

``leader_cartesian`` is not available per arm: it turns one leader into the twist of one cartesian
arm.

The bimanual source requests intervention: ``intervened`` is true while any arm's operator
intervenes (``intervention_from: any``, the default) or only while both do (``all``). Under ``any``,
the arm whose operator is not intervening is held still: its arm block is the hold action (the
follower's joints in ``joint_abs``, zero in the delta modes) and its gripper the action last sent
to it, so an idle leader never moves its follower and an idle trigger never opens a gripper.
``episode_end`` and the events are ORed over the arms; ``info["teleop_events"]`` carries each event
as ``<arm>_<event>``, ``<arm>.<event>`` and the plain ``<event>`` (``success``, ``teleop_syncing``, ...).

With ``--teleop-event-device keyboard`` (or ``stdin``) the bimanual source is the motion half of a
composite: ``t``/``h`` take over and hand back both arms at once (each arm's onset policy still
applies), and the keyboard carries the episode-end, outcome and quit keys.

``--teleop-options-yaml`` for ``bimanual`` is either a mapping with one section per arm name plus
common keys (``intervention_from`` is the source's own; every other common key goes to every arm,
the arm's section overrides it):

.. code-block:: yaml
   :dedent: 0

   # bimanual.yaml, for --teleop-device bimanual --teleop-arm-device gello
   intervention_from: any
   hold_s: 0.0
   left:
     port: /dev/serial/by-id/usb-FTDI_..._LEFT-if00-port0
     joint_ids: [1, 2, 3, 4, 5, 6, 7]
     joint_offsets_rad: [...]
     joint_signs: [...]
   right:
     port: /dev/serial/by-id/usb-FTDI_..._RIGHT-if00-port0
     joint_ids: [1, 2, 3, 4, 5, 6, 7]
     joint_offsets_rad: [...]
     joint_signs: [...]

or, with ``--teleop-arm-device gello``, the station yaml itself
(``robots: {left: ..., right: ...}``, as ``rwrl setup`` writes it): each robot's
``teleop.options`` (and ``hold_s``) become that arm's kwargs, matched by robot name to the env's
``arm_names``. A single-robot station or robot yaml works the same way for ``--teleop-device gello``
on one arm. With ``leader_arm`` or ``robot`` per arm, each daemon reads its own GELLO, so a station
yaml is refused there.

HIL over the two daemons:

.. code-block:: bash
   :dedent: 0

   rwrl train hil_serl --role actor --config bimanual.yaml \
       --teleop-device bimanual --teleop-arm-device leader_arm \
       --robot-teleop-action-mode joint_delta --teleop-event-device keyboard

``--teleop-device fake`` declares the env's arm count too, so it dry-runs the two-arm actor
without an operator.

.. _en-teleop-interventionwrapper:

``InterventionWrapper``
--------------------------------------------------------------------------------

``real_world_rl/envs/wrappers/intervention.py``. ``InterventionWrapper(env, source)`` takes a
``TeleopSource``; ``TeleopInterventionWrapper(env, teleop=None, device="pico", **kwargs)`` builds one
by device name (or adapts the object passed as ``teleop``). Both expose the source as ``.source``
(alias ``.teleop``).

``step(action)`` polls the source first and intersects the source's human mask with the selected
parts. With a nonempty human mask, it composes human and idle-part actions as a ``(1, A)`` float32
tensor on the policy device. ``idle_behavior="hold"`` holds unselected parts; ``"policy"`` keeps the
proposal there. Actual human control and hold overrides are recorded separately:

.. list-table::
   :header-rows: 1

   * - ``info`` key
     - Value
   * - ``intervene_action``
     - the executed ``(1, A)`` action; **only while intervening**
   * - ``policy_action``
     - the action the policy proposed, as received
   * - ``intervened``
     - python bool: the operator's action was executed
   * - ``intervention_onset``
     - python bool: intervened now and not on the previous step of this episode
   * - ``human_episode_end``
     - python bool: the operator's episode-end button
   * - ``teleop_events``
     - ``dict`` of fired ``success``/``failure``/``rerecord`` edges and leader catch-up ``teleop_syncing``; **only when present**
   * - ``human_action_mask``
     - dimensions actually driven by the human, including their provenance during syncing
   * - ``override_mask``
     - dimensions replacing the policy proposal, including hold supplements

An ``episode_end`` sets ``truncated=True`` without changing ``terminated``.
It does not itself make a terminal transition; an inner env may already have
terminated on that same step.

``info["syncing"]`` is always a bool. SERL button levels are exposed as ``left`` /
``right`` when supplied; per-arm success/failure events also produce
``teleop_arm_events`` when present.

Order of events:

-  Construction: validate the source against the env contract, then ``source.start()``.
-  ``reset(**kw)``: ``env.reset()`` first, then ``source.reset()``: presses while the robot homes are
   dropped. The "last gripper action" bookkeeping returns to open (+1) and the onset detector
   restarts.
-  ``close()``: ``source.close()``, then ``env.close()`` (also if the source fails to close).

For JSONL event input, reset clears the takeover latch but preserves unread
file/pipe events; producers must synchronize with session phases.

The wrapper remembers the gripper action last sent to the env (clipped to ``[-1, 1]``); a teleop
without a gripper on an env with one holds it instead of dropping a grasped object.

.. _en-teleop-adding-a-device:

Adding a device
--------------------------------------------------------------------------------

Subclass ``TeleopSource``, declare the action space, implement ``poll()``, and register it:

.. code-block:: python
   :dedent: 0

   from real_world_rl.teleop import TeleopReading, TeleopSource, register_teleop_source

   @register_teleop_source("joystick")
   class JoystickTeleopSource(TeleopSource):
       action_mode = "cartesian_delta"
       arm_action_dim = 6
       has_gripper = True

       def __init__(self, port: str = "/dev/input/js0") -> None: ...

       def start(self) -> None: ...      # open the device
       def poll(self) -> TeleopReading: ...
       def reset(self) -> None: ...      # drop latched presses
       def close(self) -> None: ...

``make_teleop_source("joystick", env, port=...)`` then calls ``JoystickTeleopSource.from_env(env, port=...)``, which defaults to ``cls(**kwargs)``; override ``from_env`` to read the env's action
contract (``real_world_rl.contracts.action.get_env_action_contract(env)``). Unknown names
raise ``ValueError`` listing the registered ones. Registration must happen before the name is used,
so a downstream package imports its module (or exposes it through its own entry point) before
building the env. Built-in sources are imported on first use of their name, so importing
``real_world_rl.teleop`` pulls in neither ``zmq`` nor ``hid``.

Actions are in the env's normalized ``[-1, 1]`` space, gripper last. For a cartesian device whose
twist is in physical units (as ``pico_twist``'s), see how the demo scripts normalize it.

.. _en-teleop-capabilities:

Capabilities
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The intervention wrapper, the hold policy, the session gate and the training env builder learn
what a source can do from its class, not from its name: they read the class through
``real_world_rl.teleop.core.registry.source_class(name, options)``, which returns it without
building a source. Set only the class attributes your source differs in:

.. list-table::
   :header-rows: 1

   * - Class attribute (default ``False``)
     - Set it to ``True`` when the source
     - Effect
   * - ``USES_LAST_GRIPPER_ACTION``
     - takes ``last_gripper_action_getter`` (holds or starts from the gripper actions last sent)
     - the wrapper passes the getter (a ``composite`` hands it on to such a motion half)
   * - ``REQUIRES_FULL_ACTION``
     - drives every arm and gripper dimension at once
     - a partial ``--teleop-selected-parts`` is rejected
   * - ``DRIVES_ONE_ARM``
     - drives exactly one arm
     - ``--env-backend real_robot_bimanual`` rejects it (use ``--teleop-device bimanual``)
   * - ``TRACKS_CARTESIAN_TARGET``
     - moves the follower through a ``CartesianTargetTracker`` and exposes it as ``cartesian_tracker``
     - gets the ``--teleop-lead-cap-*``/``--teleop-target-speed-*`` kwargs; the hold policy shares its tracker
   * - ``RECORDS_TELEOP_INPUT``
     - wants its resolved options frozen into the run's ``teleop_input``
     - resume reuses the snapshot instead of rereading the options file
   * - ``SUPPORTS_INPUT_DIAGNOSTICS``
     - accepts ``diagnostic_axes``/``diagnostic_gain``
     - ``--teleop-diagnostic-axes``/``--teleop-diagnostic-gain`` are allowed

``INPUT_FRAME`` is separate from the boolean capabilities: default ``None``,
``"twist"`` for Cartesian velocity input or ``"pose"`` for pose input.

Three classmethods answer questions asked before a source exists: ``resolved_class(options)``
(the class ``from_env`` builds for these options; the SpaceMouse's ``mode: serl`` builds the serl
source), ``options_from_station(station_path, station, robot=, profile=, provenance=)`` (the
source kwargs a station yaml configures; by default a source takes none) and
``control_spec(tracker_options=, source_options=)`` (the ``teleop_control`` block a run compares on
resume; ``None`` by default). The episode-boundary session gate (``--episode-end-behavior hold``)
needs a motion source with ``session_input()`` (the ``SessionInputSource`` protocol: one fresh
``(axes, buttons)`` reading that does not move the target).

.. _en-teleop-pico:

Pico
--------------------------------------------------------------------------------

``--teleop-device pico`` (``PicoTeleopSource``, ``real_world_rl/teleop/pico/source.py``) drives a
``cartesian_delta`` env (``real_robot`` in ``cartesian_delta`` mode) from a Pico VR
controller. The Pico is a **pose** device: the hand pose is anchored onto the same
``CartesianTargetTracker`` as ``leader_cartesian`` and ``spacemouse`` (see \ :ref:`Mapping <en-teleop-mapping>`\ ), so the
follower never jumps at a take-over, never leads its measured pose by more than the lead cap, and
moves at most at the target speed. The device itself is a device-layer teleop part, ``PicoTeleop``
(``backend: pico``, ``real_world_rl/teleop/pico/backend.py``), so it can also sit in a robot yaml's
``teleop:`` section, be served by the robot daemon, and show up in ``rwrl doctor`` (the ``pico`` extra,
``pyzmq``).

``--teleop-device`` has no default: every actor, eval, ``rwrl record``, reward-collection and deploy
command that builds a teleop source names its device (``--teleop-device {spacemouse,gello,leader_arm,leader_cartesian,pico,keyboard,robot,bimanual,fake,spacemouse_twist}``),
or its ``--config`` preset sets ``teleop_device``. Without either it stops with that usage error
instead of opening a Pico (the former default). A learner builds no source and needs none.

The former raw-twist Pico source is ``pico_twist`` (``real_world_rl/teleop/pico/twist_sim.py``),
for the ManiSkill sim examples only (see \ :ref:`Basic Usage <en-teleop-basic-usage>`\ ).

.. _en-teleop-message-format:

Message format
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Pico side publishes JSON arrays on a ZeroMQ ``PUB`` socket (for example ``tcp://*:7777``): one hand
(``13`` values) or both hands (``26`` values, left first). With a ``topic``, it sends two frames,
``[topic, json]``. Per hand:

.. list-table::
   :header-rows: 1

   * - Index
     - Meaning
   * - ``0:3``
     - hand position (m, Pico tracking frame)
   * - ``6:10``
     - hand quaternion, ``[x, y, z, w]``
   * - ``10``
     - trigger (``0`` released .. ``1`` pulled)
   * - ``11``
     - bind button
   * - ``12``
     - episode-end button

The publisher must stream continuously: the part publishes a sample per received message, and a
silence longer than ``read_timeout_s`` (1 s) makes the next read a ``DeviceFault``, as for every
teleop part.

.. _en-teleop-the-device-part-backend-pico:

The device part (``backend: pico``)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Option (``TeleopConfig.options``)
     - Default
     - Meaning
   * - ``address``
     - ``tcp://localhost:7777``
     - the Pico publisher the ``SUB`` socket connects to
   * - ``topic``
     - ``""``
     - subscription prefix; empty takes every message
   * - ``hand``
     - ``right``
     - which record of a two-hand message
   * - ``bind_button``
     - ``bind``
     - which record button takes over (``bind``, index 11, or ``episode_end``, index 12); the other one ends the episode
   * - ``poll_interval_s``
     - ``0.005``
     - the poll thread's period

``hold_s`` (``TeleopConfig.hold_s``, default 0) extends ``intervening`` past the bind button's release.
Every ``poll_interval_s`` the poll thread reads every message queued since the previous poll:
presses are taken from all of them (a press shorter than the poll interval is not lost), the pose
from the newest. Its sample (what ``rwrl teleop read`` reads) holds:

.. list-table::
   :header-rows: 1

   * - Sample field
     - Value
   * - ``diagnostics["hand_pose"]``
     - the hand pose ``xyz + quat_xyzw``, Pico tracking frame (also over the daemon)
   * - ``action``
     - the hand's displacement since the previous sample, ``[dp, rotvec]``, unscaled and clipped to ``[-1, 1]`` (keeps the ``cartesian_delta`` shape of every part; the ``pico`` source does not read it)
   * - ``gripper``
     - ``1 - trigger``: ``1`` open (trigger released) .. ``0`` closed
   * - ``buttons``
     - ``bind``, ``episode_end`` and ``trigger`` levels
   * - ``events``
     - ``bind`` and ``episode_end`` presses, latched until the next ``read()``
   * - ``intervening``
     - the bind button is held (plus ``hold_s``)

The first message after a connect or ``reset()`` only sets the button baseline, so a button held
through it is not a press. ``pyzmq`` is imported when the part connects; ``discover()`` finds nothing
(a network publisher has no local device node).

.. _en-teleop-take-over-and-the-anchored-mapping:

Take-over and the anchored mapping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1

   * - Input
     - Effect
   * - bind press
     - takes over; the next press hands back. Moving the hand alone never intervenes.
   * - hand motion while driving
     - ``p* = p_c + (p_l - p_l0)``, ``R* = (R_l R_l0^T) R_c`` (``pico_anchored_v1``): ``(p_c, R_c)`` is the tracker's command and ``(p_l0, R_l0)`` the hand pose at the take-over, ``p_l = B p_hand``, ``R_l = B R_hand`` with ``B = input_rotation``
   * - trigger
     - ``+1`` (open) when ``1 - trigger > gripper_binarize_threshold`` (0.5), else ``-1``; ``gripper_binarize_threshold: null`` emits the continuous ``2w - 1``. At a take-over a trigger that disagrees with the gripper's last command is ignored until it agrees, so taking over never drops a grasped object.
   * - episode-end press
     - ends the episode and hands back
   * - request outside the ``limits.tcp`` box
     - clipped into the box; the command slides along the wall, no hold and no hand-back
   * - control loop stall (> ``max_dt_s``)
     - holds at the measured pose; the next poll re-anchors the hand there
   * - read failure
     - propagates and hands back

A hand-back holds the measured pose, and the recorder's hold policy shares the tracker while nobody
drives. Re-taking over anchors wherever the hand is then, so hand-back, move, take-over is the
clutch. With ``--teleop-event-device stdin`` or ``keyboard`` the event device adds the outcome labels
(``s``/``f``/``r``), episode end (Esc) and quit (``q``); bind stays the only take-over (``t``/``h`` log a
warning).

``input_rotation`` (``base_R_input``, a proper 3x3 rotation, default identity) turns the Pico
tracking frame into the robot base frame. The tracking frame depends on the Pico-side app and on
where the headset was recentred; commission it before the first real take-over: with a small
``--teleop-target-speed-m-s`` (for example 0.01), take over, move the hand about 5 cm forward, then
left, then up, and check that the follower moves along base ``+x``, ``+y``, ``+z``. Each column ``i`` of
``input_rotation`` is the base-frame direction of the Pico frame's axis ``i``. Translation is 1:1
(no motion scaling).

The run compares the tracker's block (``teleop_control``, ``cartesian_tracker_v1``, from the
``--teleop-lead-cap-*``/``--teleop-target-speed-*`` flags), as for ``leader_cartesian``; the Pico's
own options are not compared. Each reading's diagnostics name the mapping
(``target_mapping: pico_anchored_v1``) and carry the raw hand pose; the ``real_robot`` env's
diagnostics JSONL records them when it is enabled. Both are provenance, never compared.

.. _en-teleop-running-it:

Running it
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

On the machine that runs the actor (or the recorder), repository root, project environment
active: install the extra once.

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   pip install -e '.[pico]'

Check the stream before touching the robot (no env, prints bind/episode-end and the hand twist of
the legacy bridge):

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python examples/run_twist_input_inspector.py --zmq-url tcp://192.168.6.2:7777

Write the Pico options to ``local/pico.yaml`` (a flat mapping of source kwargs; the tracker keys
come from the flags, not from this file):

.. code-block:: yaml
   :dedent: 0

   # local/pico.yaml, for --teleop-device pico --teleop-options-yaml local/pico.yaml
   address: tcp://192.168.6.2:7777
   hand: right
   # base_R_input from the commissioning above; identity when omitted
   # input_rotation: [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

Then, same machine, repository root, project environment active, with the station's ``hil.yaml``
configured for a ``cartesian_delta`` env (``xyz_scale_m >= 0.05`` and ``rot_scale_rad >= 0.3``, the
default lead caps), with the operator's labels on the terminal:

.. code-block:: bash
   :dedent: 0

   rwrl record --config hil.yaml \
       --teleop-device pico --teleop-options-yaml local/pico.yaml --teleop-event-device stdin \
       --output-dir demos/task_20 --num-successes 20

The HIL actor takes the same three flags (``rwrl train hil_serl --role actor --config hil.yaml --teleop-device pico --teleop-options-yaml local/pico.yaml --teleop-event-device stdin``).

**Through the robot daemon.** On a ``real_robot`` station, the Pico can be the ``teleop:`` part of
the robot itself, read where the robot runs (locally, or on the robot PC served by ``rwrl daemon``
like any other part):

.. code-block:: yaml
   :dedent: 0

   # in the station robot's yaml
   teleop:
     backend: pico
     hold_s: 0.0
     options:
       address: tcp://192.168.6.2:7777
       hand: right

and the actor's ``local/pico.yaml`` then says to read the env robot's part instead of opening its
own subscriber:

.. code-block:: yaml
   :dedent: 0

   robot_teleop: true

On the ``real_robot`` stack, a robot daemon that serves the Pico as its teleop part is read with
``remote_address: <robot-host>:<port>`` (and ``remote_token``), as for ``leader_cartesian``.

.. _en-teleop-basic-usage:

Basic Usage
--------------------------------------------------------------------------------

Visual teleoperation in the ManiSkill sim, with the raw-twist Pico source ``pico_twist`` (the
former ``pico``; sim only, on a real robot use ``--teleop-device pico``, \ :ref:`Pico <en-teleop-pico>`\ ):

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python examples/run_teleop_demo.py \
     --env-id PegInsertionSidePegOnly-v1 \
     --device pico_twist \
     --zmq-url tcp://192.168.6.2:7777

Print incoming teleop state without connecting to an environment:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python examples/run_twist_input_inspector.py \
     --device pico \
     --zmq-url tcp://192.168.6.2:7777

The same demo with a SpaceMouse (``--spacemouse-index``, ``--spacemouse-deadzone``,
``--spacemouse-invert-axes``, ``--spacemouse-hold-s`` tune it) or the idle ``fake`` source:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python examples/run_teleop_demo.py --env-id PegInsertionSidePegOnly-v1 --device spacemouse_twist

The sim examples use ``spacemouse_twist``, the raw-twist SpaceMouse source: moving the puck counts
as intervening, its twist goes to the env as is, button 0 toggles the gripper and button 1 ends
the episode. It is for the sim only; on a real robot use ``--teleop-device spacemouse``
(\ :ref:`SpaceMouse <en-teleop-spacemouse>`\ ).

``run_teleop_demo.py`` creates the ManiSkill environment, builds the source with
``make_teleop_source``, polls teleop actions, applies controller action normalization to the
``pico_twist`` physical twist (SpaceMouse actions are already normalized), and steps the environment.

.. _en-teleop-recording-demos-on-the-real-robot:

Recording demos on the real robot
--------------------------------------------------------------------------------

``examples/record_real_demos.py`` records teleoperated episodes on the ``real_robot`` or
``real_robot_bimanual`` env of a HIL run (same env and teleop flags as its actor)
into a demo dataset directory (``--output-dir``) that the learners
load with ``--demo-dataset-paths``; training takes only the intervened steps. See
\ :doc:`recording-demos.md <recording-demos>`\ ; for a GELLO, \ :ref:`gello.md <en-gello-5-recording-demos>`\ .

.. _en-teleop-recording-wsrl-data:

Recording WSRL Data
--------------------------------------------------------------------------------

``examples/record_teleop_wsrl.py`` records teleoperation
episodes into the same H5 format used by the WSRL data generator.

Example:

.. tutorial-test: skip
.. code-block:: bash
   :dedent: 0

   python examples/record_teleop_wsrl.py \
     --output-path demos/peg_teleop_wsrl.h5 \
     --env-id PegInsertionSidePegOnly-v1 \
     --obs-mode rgb \
     --control-mode pd_ee_twist \
     --device pico_twist \
     --zmq-url tcp://192.168.6.2:7777

``--output-path`` must be an H5 file path, not a directory. A ``.h5`` suffix is
recommended. The file is opened with write mode, so an existing file at the same
path will be overwritten.

Only ``intervened=True`` samples are stepped in the environment and saved. When
``intervened=False``, the script keeps polling the device and does not record a
transition.

The operator's episode-end button (the Pico's episode-end button under ``pico_twist``, ``spacemouse_twist`` button 1) ends the
current episode. After each episode, the terminal prints the episode length, ``max_episode_steps``, success flag, and
return, then asks whether to save the episode.

.. _en-teleop-important-arguments:

Important Arguments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  ``output_path``: H5 dataset file to write.
-  ``device``: ``pico_twist`` (default), ``spacemouse_twist`` or ``fake`` (with the same device-specific flags as the demo).
-  ``dt``: teleop polling period in seconds. The default is ``1 / 30``, roughly
   ``30 Hz``. Smaller values poll and step faster.
-  ``env_id``: ManiSkill environment id.
-  ``obs_mode``: observation mode, usually ``state``, ``rgb``, or ``rgbd``.
-  ``include_state``: include proprioceptive state for visual observations.
-  ``control_mode``: should match the EE twist controller, usually ``pd_ee_twist``.
-  ``sim_backend``, ``render_backend``: ManiSkill simulation and rendering backends.
-  ``reward_mode``: reward mode passed to the environment.
-  ``robot_uids``: robot id for the environment.
-  ``fix_box``, ``fix_peg_pose``, ``peg_density``, ``debug_pose_vis``: custom peg env
   options.
-  ``env_kwargs_json``: escape hatch for extra environment kwargs.
-  ``end_on_env_done``: if true, env termination, truncation, or success can end
   the episode. The default is false, so only teleop episode-end stops recording.
-  ``pos_scale``, ``rot_scale``, ``twist_limit``: optional overrides for twist mapping.
-  ``intervention_threshold``: twist norm threshold used to decide whether human
   input is active.

The saved H5 contains ``traj_*`` groups with:

.. code-block:: text
   :dedent: 0

   obs
   actions
   rewards
   terminated
   truncated

This file can be passed to WSRL, CQL, or Cal-QL offline training through
``--offline_dataset``.

For real-robot data where you only want to do offline pretraining (no sim
env, no eval), use rl-garden's ``examples/pretrain_offline.py cql`` or
``examples/pretrain_offline.py calql`` for standalone offline CQL/Cal-QL checkpoints. Use
the ``wsrl`` subcommand when the checkpoint should be resumed by the WSRL
offline→online flow. This entrypoint infers obs/action specs from the H5 and
produces pretrained checkpoints that can later be loaded into compatible
live-env training runs for online fine-tuning. See
``rl-garden/docs/design/wsrl-overview.md#offline-only-pretraining-no-sim-env`` in the
``rl-garden/`` submodule for
the full workflow.

.. _en-teleop-partial-control-and-training-provenance:

Partial control and training provenance
--------------------------------------------------------------------------------

Select parts with ``--teleop-selected-parts`` (``arm.arm`` / ``arm.gripper`` on the tutorial station,
``left.arm``, ``right.arm``, ``left.gripper``, ``right.gripper`` on two arms). Default
``--teleop-idle-behavior hold`` holds unselected parts; ``policy`` leaves them autonomous.
An empty selection uses the source's available parts. A source without a gripper channel
never supplies an expert gripper label. Continuous no-gripper HIL is supported; hybrid/discrete
branches require a gripper. Syncing rows remain executed/archived, but their training projection
has no expert supervision. See \ :doc:`Real-world experiments <real-world-experiments>`\  for masked
BC/SiLRI/residual losses, explicit outcomes and durable records.

.. |reference-link-1| replace:: ``leader.franka-as-leader.example.yaml``
.. _reference-link-1: https://github.com/JaimeParker/real-world-rl/blob/main/examples/stations/leader.franka-as-leader.example.yaml
