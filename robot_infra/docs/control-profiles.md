# Arm control profiles

Station YAML can declare an impedance profile independently for each arm:

```yaml
arm_backend: franky
control:
  translational_stiffness: 500.0
  rotational_stiffness: 30.0
  cartesian_damping: [45.0, 45.0, 45.0, 11.0, 11.0, 11.0]
  translational_error_clip: [0.02, 0.02, 0.02]
  rotational_error_clip: [0.1, 0.1, 0.1]
  max_delta_tau: 0.5
```

These are illustrative settings. Choose and validate values for the actual
payload, tool, task and installed SDK before operating hardware. Automated tests
exercise fake SDKs and local RPC; they do not measure robot stability.

`robot_infra.control.ArmControlConfig` validates finite values and vector sizes
without importing torch. Cartesian gains use the tracker’s scalar translational
and rotational stiffness keywords, a six-axis damping vector, and three-axis
error clips. Joint control has separate `joint_stiffness` and `joint_damping`
vectors matching the arm’s joint count. `max_delta_tau` and
`gains_time_constant` are passed to both tracker types.

Experiment configuration can override station gains. It can only reduce a
station's declared torque slew limit and error clips. Existing workspace and
servo step caps remain enforced separately by `ArmSafety`; a control profile does
not replace them.

Configure only while IDLE:

```python
arm.set_control_mode(ControlMode.IDLE)
effective = arm.configure_control({"max_delta_tau": 0.3})
metadata = arm.control_metadata
```

The daemon exposes the same operation through `arm.configure_control` and
`arm.control_metadata`, serialized with robot commands and the watchdog.
`RemoteArm` checks advertised capabilities and retrieves the daemon’s effective
profile. An older daemon accepts an empty request locally; explicit unsupported
settings fail before movement. Empty overrides on a daemon with station defaults
still verify the installed tracker API before reset motion.

Franky checks the installed tracker constructor signatures before applying an
explicit profile. Values go directly into `CartesianImpedanceTracker` or
`JointImpedanceTracker`; a wheel missing the requested keywords raises
`VersionMismatch`. `control_metadata` records backend, SDK version, requested
settings actually passed to the tracker, and scalar SDK defaults inspectable
from its constructor. Omitted values whose defaults cannot be inspected remain
explicitly unknown. No claim is made that a requested-only profile enumerates
all low-level controller defaults.

The station wizard retains profiles when reloading YAML and renders `control:
null` for a new station. Other arm backends reject nonempty profiles unless they
advertise support. The fake backend accepts the same profile for integration
tests without simulating torque dynamics.

The setup wizard also accepts an optional profile without adding new prompts:

```bash
python -m robot_infra.setup --from local/station.yaml --control '{max_delta_tau: 0.5}'
```

Pass one mapping for both arms or one per arm. Omit the flag to preserve the
existing profile, and use `--control null` to remove it. Existing camera width
and height settings describe the source stream; crop/ROI and policy resize
belong to the experiment preprocessing configuration.
