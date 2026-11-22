Arm control profiles
================================================================================

Station YAML can declare an impedance profile independently for each arm:

.. code-block:: yaml

   arm_backend: franky
   control:
     translational_stiffness: 500.0
     rotational_stiffness: 30.0
     cartesian_damping: [45.0, 45.0, 45.0, 11.0, 11.0, 11.0]
     translational_error_clip: [0.02, 0.02, 0.02]
     rotational_error_clip: [0.1, 0.1, 0.1]
     max_delta_tau: 0.5

These are illustrative settings. Choose and validate values for the actual
payload, tool, task and installed SDK before operating hardware. Automated tests
exercise fake SDKs and local RPC; they do not measure robot stability.

``rwrl setup`` writes ``FRANKY_SETUP_CONTROL`` for a new franky arm using
``cartesian_controller: impedance`` unless ``--control`` is given;
franky's own 500 N/m is too soft to move the arm against joint friction with a centimetre of
teleop lead:

.. code-block:: yaml

   control:
     translational_stiffness: 2000.0
     rotational_stiffness: 75.0                       # SERL's 150, approximate (see below)
     cartesian_damping: [89.0, 89.0, 89.0, 7.0, 7.0, 7.0]
     translational_error_clip: [0.01, 0.01, 0.01]
     rotational_error_clip: [0.10, 0.10, 0.10]        # SERL's 0.05, approximate

These approximate SERL's default compliance (``compliance_param.cfg``) in franky's tracker. The
rotational values are approximate: franky's rotation error is the rotation vector θ·axis, SERL's
the quaternion vector part sin(θ/2)·axis ≈ (θ/2)·axis, so SERL's 150 Nm/rad becomes 75 and its
0.05 clip becomes 2·asin(0.05) ≈ 0.10 rad, equal for small angles only; the damping (89 / 7 on
the twist) is the same in both and given explicitly. These gains only apply with
``cartesian_controller: impedance`` (and the joint keys to joint servo in every mode); see
\ :ref:`devices <en-devices-station-file-reference>`\ .

``real_world_rl.devices.control.ArmControlConfig`` validates finite values and vector sizes
without importing torch. Omitted fields default to ``None`` (SDK defaults). Gains and
damping must be nonnegative; error clips, ``max_delta_tau`` and
``gains_time_constant`` must be positive. Booleans and unknown keys are rejected.
Cartesian gains use the tracker’s scalar translational
and rotational stiffness keywords, a six-axis damping vector, and three-axis
error clips. Joint control has separate ``joint_stiffness`` and ``joint_damping``
vectors matching the arm’s joint count. ``max_delta_tau`` and
``gains_time_constant`` are passed to both tracker types.

Experiment configuration can override station gains. It can only reduce a
station's declared torque slew limit and error clips. Existing workspace and
servo step caps remain enforced separately by ``ArmSafety``; a control profile does
not replace them.

Configure only while IDLE:

.. code-block:: python

   arm.set_control_mode(ControlMode.IDLE)
   effective = arm.configure_control({"max_delta_tau": 0.3})
   metadata = arm.control_metadata

The daemon exposes the same operation through ``arm.configure_control`` and
``arm.control_metadata``, serialized with robot commands and the watchdog.
``RemoteArm`` checks advertised capabilities and retrieves the daemon’s effective
profile. An older daemon accepts an empty request locally; explicit unsupported
settings fail before movement. Empty overrides on a daemon with station defaults
still verify the installed tracker API before reset motion.

Franky checks the installed tracker constructor signatures before applying an
explicit profile. Values go directly into ``CartesianImpedanceTracker`` or
``JointImpedanceTracker``; a wheel missing the requested keywords raises
``VersionMismatch``. ``control_metadata`` records backend, SDK version, requested
settings actually passed to the tracker, and scalar SDK defaults inspectable
from its constructor. Omitted values whose defaults cannot be inspected remain
explicitly unknown. No claim is made that a requested-only profile enumerates
all low-level controller defaults.

The station wizard retains existing profiles; a new franky impedance arm gets the
default profile above. Switching to ``serl`` removes Cartesian tracker keys and keeps
joint settings. ``--control null`` explicitly clears the profile. Other arm backends reject nonempty profiles unless they
advertise support. The fake backend accepts the same profile for integration
tests without simulating torque dynamics.

The setup wizard also accepts an optional profile without adding new prompts:

.. code-block:: bash

   rwrl --station local/station.yaml setup --control '{max_delta_tau: 0.5}'

Pass one mapping for both arms or one per arm. Omit the flag to preserve the
existing profile, and use ``--control null`` to remove it. Existing camera width
and height settings describe the source stream; crop/ROI and policy resize
belong to the experiment preprocessing configuration.

SERL compliance parameters and profiles
--------------------------------------------------------------------------------

With ``cartesian_controller: serl`` the Cartesian controller is SERL's own, and its parameters
are SERL's: the station ``serl:`` block (names, defaults and ranges of ``compliance_param.cfg``,
omitted keys keep the ``.cfg`` defaults), optional named profiles in the task preset, and live
partial updates:

.. code-block:: yaml

   # station
   cartesian_controller: serl
   serl: {translational_stiffness: 2000.0, rotational_stiffness: 150.0}
   # task preset
   real_robot:
     serl_profiles:
       compliance: {translational_clip_x: 0.0075, translational_clip_neg_x: 0.002}
       precision: {translational_clip_x: 0.1, translational_clip_neg_x: 0.1, rotational_stiffness: 250}

.. code-block:: python

   from real_world_rl.devices.control import serl_profile_update
   update = serl_profile_update(preset_profiles, "precision")  # the profile's keys only
   effective = arm.update_serl_params(update)   # merged into the current values (HIL-SERL update_param);
                                                # any mode; applied at the next 1 kHz tick if running
   arm.serl_params                              # the full effective set

A profile is a partial update on top of the current values, so keys it omits keep the previous
profile's values; validation warns when profiles set different keys (the switching order then
matters).

Values outside a ``.cfg`` range are clamped (as dynamic_reconfigure does); unknown
keys fail. Station/profile validation logs clamps and stores the clamped values.
Live ``update_serl_params`` clamps are also recorded in
``control_metadata["serl"]["clamped"]``; already-clamped config values do not preserve
their original requests there. Unlike ``configure_control``, SERL
parameters are not bounded by the station values: the ``.cfg`` ranges are the bound, as in SERL.
``control:``'s Cartesian tracker keys are ignored in this mode (a warning at station load and a
``config:serl_control`` WARN in ``rwrl doctor`` name them).
The full table, HIL-SERL's ``ram_insertion`` profiles as an example and the controller details are
in \ :ref:`SERL controller <en-serl-controller-parameters>`\ .
