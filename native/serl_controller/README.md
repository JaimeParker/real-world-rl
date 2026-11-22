# serl_controller

SERL's field-tested Franka controllers, without ROS, running inside
[franky](https://github.com/TimSchneider42/franky)'s 1 kHz control loop: the
Cartesian impedance controller (a torque motion) and the joint reset controller
(a joint position motion).

SERL ships its controller as `serl_franka_controllers`, a separate ros_control
plugin next to an unmodified `franka_ros`. This package does the same for
franky: it is a separate C++/pybind11 package that links against an
**unmodified** franky and provides a `franky::Motion<franka::Torques>` and a
`franky::Motion<franka::JointPositions>`.

| Part | What it is |
| --- | --- |
| `SerlCartesianImpedanceLaw` (`include/serl_controller/law.hpp`, `src/law.cpp`) | The control law, Eigen only, no franky. State in (`O_T_EE`, `q`, `dq`, `tau_J_d`, EE zero Jacobian, coriolis), torques out. `applyParams` / `setTarget` / `starting` / `update` are SERL's `complianceParamCallback` / `equilibriumPoseCallback` / `starting` / `update`. |
| `SerlParams` (`params.hpp`) | The 20 fields of SERL's `compliance_param.cfg`, same names, defaults and ranges; partial merge + clamping like dynamic_reconfigure. |
| `SerlCartesianImpedanceMotion` (`motion.hpp`) | The law as a franky torque motion. Lock-free target/parameter mailboxes and a per-tick telemetry snapshot (zero Jacobian, state, command, filtered gains). |
| `SerlJointPositionLaw` (`joint_reset.hpp`, `src/joint_reset.cpp`) | SERL's `JointPositionController`: 10 s linear blend q = (t/10)·q_target + ((10−t)/10)·q_start, then hold. Eigen only. |
| `SerlJointPositionMotion` (`joint_reset_motion.hpp`) | The joint reset as a franky joint position motion; `finish()` ends it with `franka::MotionFinished` (a hold). |
| `serl_controller` Python package | pybind11 bindings. `SerlCartesianImpedanceMotion` is a subclass of `franky.BaseTorqueMotion`, `SerlJointPositionMotion` of `franky.BaseJointPositionMotion`. `build_info()` / `build_info.json`: which franky build it belongs to. |

## Provenance

Ported from `rail-berkeley/serl_franka_controllers` at commit `1f140ef`
(`src/cartesian_impedance_controller.cpp`, `cartesian_impedance_controller.h`,
`pseudo_inversion.h`, `cfg/compliance_param.cfg`, `src/joint_position_controller.cpp`,
`joint_position_controller.h`), MIT License, Copyright (c)
2024 RAIL Berkeley; see `LICENSE`. The ported files keep that notice and name
the original file and commit. The originals are vendored verbatim under
`tests/cpp/golden/serl_original/` (with `SHA256SUMS`) for the golden test only.

`PositionJointSoftLimits` is ported from ros_control's `joint_limits_interface`
(noetic-devel @ `633c374`, BSD-3-Clause, Copyright (C) 2013 PAL Robotics S.L. and
(c) 2008 Willow Garage; the full notice is next to the code in `src/joint_reset.cpp`);
the Panda / FR3 limit values are franka_ros @ `ddd2fff` (`franka_description`,
Apache-2.0). Their verbatim test copies are in `tests/cpp/golden/ros_control_original/`
and `tests/cpp/golden/franka_ros_original/`, each with its own `LICENSE` and
`SHA256SUMS`. `LICENSE` lists every part and its licence.

Behaviour kept from SERL, on purpose:

- translational error clipped per axis into `[-clip_neg, clip]`; orientation
  error = vector part of `q^-1 * q_d` (after flipping `q` into `q_d`'s
  hemisphere), rotated to the base frame with a sign flip, then clipped;
- `error_i` accumulates every tick even with `Ki = 0`, is clipped to ±0.1 / ±0.3
  and is reset only by a new target;
- damped pseudo-inverse (λ = 0.2) for the nullspace projector; nullspace PD
  towards `q` at start with a separate joint-1 stiffness; coriolis added, no
  gravity; torque-rate saturation ±1 Nm/tick against `tau_J_d`;
- after the torque of a tick is computed, K, D, nullspace stiffnesses, Ki and
  the position target move toward their targets with `filter_params = 0.005`,
  the orientation target by slerp;
- a new target quaternion is flipped into the previous target's hemisphere.

Lifetime. SERL's `franka_server` relaunches `impedance.launch` for every start,
so every start is a fresh controller. Here one motion object is one controller
lifetime (franky motions cannot be restarted anyway):

- construction is `init()` plus dynamic_reconfigure's first callback: K, D and
  Ki start at 0 (Ki and `error_i` are uninitialised in the original; fresh heap
  memory is zero, so 0 is what runs), nullspace stiffness and joint-1
  nullspace stiffness start at 20 (the header initialisers), and the *targets*
  come from the given parameters (cfg defaults: K 2000/150, D 89/7, nullspace
  0.2, joint 1 100, clips 0.01/0.05, Ki 0). So K and D ramp from 0 over ~200
  ticks (0.2 s) after every start, and the nullspace gains ramp 20 → 0.2 / 100;
- the first tick runs `starting()`: the pose target and the nullspace posture
  become the current state; a target sent before the start is overwritten, as
  in SERL;
- `update_params()` / `set_target()` take effect at the start of the next
  tick, the way ROS callbacks run between two `update()` calls. When several
  targets arrive within one millisecond only the newest is applied.

Deliberate differences: `set_target` rejects non-finite values and quaternions
whose norm is more than 1e-3 from 1, and parameters must be finite (SERL would
accept these and produce NaN torques). Valid inputs behave identically.

## Joint reset (`SerlJointPositionMotion`)

SERL's `JointPositionController` (`src/joint_position_controller.cpp`), run by
SERL's `joint.launch` under franka_control's position interface:

```text
starting(): initial_pose_ = q (measured);  elapsed_time_ = 0
update(period): elapsed_time_ += period; t = elapsed_time_.toSec()
                q_d = t > 10 ? q_target : (t / 10.0) * q_target + ((10 - t) / 10.0) * initial_pose_
```

`elapsed_time_` is a `ros::Duration` (int32 sec + int32 nsec) and franka_hw passes
`period = ros::Duration(franka::Duration(time_step).toSec())`, so the clock is an
exact count of whole milliseconds. The port keeps that representation (exact
nanoseconds read back with `toSec()`'s formula) and the expression order;
franky's `time_step` (whole ms, 0 on the first tick) drives it exactly like
franka_hw's period (0 on controller_manager's reset tick, where `starting()` runs).

franka_hw's settings for this interface (`franka_control_node.yaml` + `FrankaHW::setRunFunction`,
`ControlMode::JointPosition`): `robot.control(cb, internal_controller = joint_impedance,
limit_rate = rate_limiting = true, cutoff_frequency = 100)`. With franky: the robot's default
`ControllerMode.JointImpedance` and `robot.move(motion, asynchronous=True, limit_rate=True,
cutoff_frequency=100.0)` (the same `FRANKA_ROS_*` constants).

franka_hw also wraps the position command in its `PositionJointSoftLimitsInterface`
(`FrankaHW::setupLimitInterface`: one ros_control `PositionJointSoftLimitsHandle` per joint,
from the URDF's `<limit>` and `<safety_controller>`; franka_control_node calls `reset()` on the
period-0 tick and `enforceLimits(period)` right after every other controller update, between
`read()` and `write()`). `PositionJointSoftLimits` (`joint_reset.hpp`) is that handle, ported line
by line from ros_control noetic-devel @ 633c374 (`joint_limits_interface.h`), and
`SerlJointPositionMotion` applies it at the same point, so the command it returns is the one
franka_hw would write to libfranka. The limit tables are franka_description's (franka_ros @
ddd2fff) `robots/{panda,fr3}/joint_limits.yaml` through the `franka-limits` xacro macro
(soft limits = limits, `k_position` 100; `FRANKA_JOINT_LIMITS` in Python);
`SerlJointPositionMotion(target, robot_model="panda")` picks one (franka_control's default is
`robot:=panda`). The snapshot reports the controller's own command and `soft_limited_ticks`.

How franky's JointPositions derivation works (checked in franky v2.0.0 like the
torque one): `franky::Motion<ControlSignalType>` is a template explicitly
instantiated for `franka::JointPositions` in `libfranky.so` (`src/motion/motion.cpp`)
with the same virtual `initImpl` / `nextCommandImpl` hooks; the Python base
`franky.BaseJointPositionMotion` comes from `mkMotionAndReactionClasses<franka::JointPositions>(m,
"JointPosition")` (`python/bind_reactions.cpp`); `Robot::move(std::shared_ptr<Motion<franka::JointPositions>>,
async, limit_rate, cutoff_frequency)` runs it with `params_.controller_mode` (default
`kJointImpedance`), bound as `Robot.move(motion, asynchronous, limit_rate, cutoff_frequency)`.
`nextCommandImpl` receives franky's `RobotState` (whose `q` is the measured
`franka::RobotState::q`, which franka_hw's joint handle also reads) and the tick's
`time_step`; returning `franka::MotionFinished(cmd)` ends franky's control loop cleanly.

The motion never ends by itself (SERL holds until franka_server kills
`joint.launch`); `finish()` makes the next tick repeat the previous command as
`MotionFinished`. How rwrl finishes it (HIL-SERL's `franka_server.reset_joint`:
poll `np.allclose(target - q, 0, atol=1e-2, rtol=1e-2)` once per second, at most
30 more polls, then stop and clear errors) is in the [device guide](https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/devices.html).

## Python use

```python
import franky                       # the franky wheel from the SAME build
import serl_controller as sc

robot = franky.Robot("172.16.0.2")
motion = sc.SerlCartesianImpedanceMotion({"translational_stiffness": 2000.0})
robot.move(motion, asynchronous=True,
           limit_rate=sc.FRANKA_ROS_LIMIT_RATE,             # True
           cutoff_frequency=sc.FRANKA_ROS_CUTOFF_FREQUENCY)  # 100.0
motion.set_target([0.5, 0.0, 0.3], [1.0, 0.0, 0.0, 0.0])   # xyz, quat (x, y, z, w)
motion.update_params({"translational_clip_x": 0.0075})      # partial, like update_configuration
J = motion.snapshot()["zero_jacobian"]                      # 6x7, replaces SERL's /franka_jacobian

reset = sc.SerlJointPositionMotion([0, 0, 0, -1.9, -0.0, 2, 0])   # HIL-SERL's default reset target
robot.move(reset, asynchronous=True, limit_rate=sc.FRANKA_ROS_LIMIT_RATE,
           cutoff_frequency=sc.FRANKA_ROS_CUTOFF_FREQUENCY)
# ... once reset.snapshot()["elapsed_s"] > 10 and the joints are there:
reset.finish(); robot.join_motion()
```

`SerlCartesianImpedanceLaw`, `law_step(law, ...)` and `SerlJointPositionLaw` run
the laws offline, without franky or a robot.

## torque rate limiting and low-pass filter: franka_ros vs franky

libfranka applies two optional filters to every torque command
(`src/control_loop.cpp`, `spinControl`): a first-order low-pass toward the
last command `tau_J_d` when `cutoff_frequency < kMaxCutoffFrequency` (1000 Hz),
then `limitRate(kMaxTorqueRate = 1000 - 1e-3 Nm/s)` when `limit_rate` is true.

| | `limit_rate` | `cutoff_frequency` | Source |
| --- | --- | --- | --- |
| franka_ros (`franka_control.launch`, used by SERL's `impedance.launch`) | `true` | `100` Hz | `franka_control/config/franka_control_node.yaml` (`rate_limiting: true`, `cutoff_frequency: 100`), read in `franka_hw/src/franka_hw.cpp` and passed to `robot.control(torque_cb, limit_rate, cutoff_frequency)` in `FrankaHW::setRunFunction` (JointTorque case) |
| franky v2.0.0 `Robot.move(...)` defaults | `false` | `franka::kDefaultCutoffFrequency` = `100` Hz | `include/franky/robot.hpp` (`move(std::shared_ptr<Motion<franka::Torques>>, async, limit_rate = false, cutoff_frequency = kDefaultCutoffFrequency)` → `control(m, limit_rate, cutoff_frequency)`), `python/bind_robot.cpp` (`limit_rate=False`, `cutoff_frequency=kDefaultCutoffFrequency`); `libfranka/include/franka/lowpass_filter.h` (`kDefaultCutoffFrequency = 100.0`) |

So the cutoff already matches and only rate limiting differs. franky's Python
API takes both per call, so no franky change is needed: pass
`limit_rate=True, cutoff_frequency=100.0` to `Robot.move` (exported as
`FRANKA_ROS_LIMIT_RATE` / `FRANKA_ROS_CUTOFF_FREQUENCY`). With
`delta_tau_max = 1 Nm/tick` inside the law, libfranka's 1000 Nm/s limit is
almost never active; the 100 Hz low-pass is.
(franka_ros checked at `ddd2fff`; the `franka_combined_control` config uses
`cutoff_frequency: 1000`, but SERL uses the single-arm `franka_control.launch`.)

## Build

### ABI coupling rule

`SerlCartesianImpedanceMotion` derives, in C++ and in Python, from franky's
`Motion<franka::Torques>` and its pybind11 class `franky.BaseTorqueMotion`.
That only works when the two extensions share the class layouts and
pybind11's internals. **Build franky and serl_controller in the same build
(same machine or container, compiler, pybind11, libfranka and franky headers)
and deploy them together.** Never install serl_controller next to a franky
from a different build (including the official PyPI or GitHub wheels). The
pieces this rule covers:

- the franky headers come from the same franky tag as the wheel, and the
  `FRANKA_0_8 ... FRANKA_0_18` defines are derived from the libfranka version the
  same way franky's `CMakeLists.txt` does (they change `RobotState` and
  `Robot` layouts);
- pybind11 is pinned to franky's (`3.0.4`, `pyproject.toml`);
- the extension links the installed franky's own `libfranky.so` and its libfranka. Two layouts:
  **wheel** (the Docker recipe): the franky wheel's bundled `libfranka-<hash>.so.<ver>` (passed as
  `FRANKY_LINK_LIBRARIES`), found at run time through
  `RPATH=$ORIGIN/../franky:$ORIGIN/../franky_control.libs` and `auditwheel repair --exclude`d so
  nothing from franky is duplicated; **source** (`scripts/install-on-ubuntu.sh`): franky
  pip-installed from source against a system libfranka, `FRANKY_LINK_LIBRARIES` empty, so
  `Franka::Franka` is linked and its directory is appended to the extension's RUNPATH (franky's
  own `libfranky.so` has `RUNPATH=$ORIGIN` and finds the system libfranka through ld.so).

Each build records which franky it belongs to in
`serl_controller/build_info.json` (also compiled in as `serl_controller.BUILD_INFO`):
the franky_control version and franky commit, the sha256 of the
`libfranky.so` it was linked against, `franky_layout` (`wheel` | `source`), the
bundled libfranka file name (wheel) or the system libfranka's real path and
sha256 (source), the libfranka version and a build id. `rwrl doctor` (`sdk:serl_controller`) compares
these with the installed franky wheel without importing either extension. A franky
rebuild is not byte-reproducible, so always deploy the two wheels of one build
directory together.

### On the robot PC, without Docker (`scripts/install-on-ubuntu.sh`)

A Franka NUC is plain Ubuntu (20.04 / 22.04 / 24.04). With the project
environment active, from the repository root:

```bash
LIBFRANKA_VERSION=0.17.0 bash native/serl_controller/scripts/install-on-ubuntu.sh
```

It installs franky's apt build packages; CMake ≥ 3.24 (< 4), scikit-build-core
and pybind11 3.0.4 into the environment; clones franky `v2.0.0` (`e7e63c7`),
checks the commit and that the checkout is unmodified; builds Eigen, pybind11,
ruckig and libfranka `LIBFRANKA_VERSION` (poco, and pinocchio from 0.14) into
`/usr/local` with franky's own unmodified `docker/common/install-*` scripts
(reusing an installed libfranka of that version, refusing to replace another
version unless `REPLACE_LIBFRANKA=1`); pip-installs franky from a copy of the
checkout with `FRANKY_LIBFRANKA_VERSION_LABEL` (version `2.0.0+libfranka.X`);
pip-installs this package against it with `--no-build-isolation`; then runs
CTest (golden tests + `test_motion`) against the installed franky, the Python
tests, a smoke import and `rwrl doctor`'s pairing check
(`serl_build_pairing`). Git runs over HTTP/1.1 and network steps are retried
(`RETRIES`, default 3). Logs: `~/.cache/rwrl-native-build/logs/` (`WORK=`).
Pick `LIBFRANKA_VERSION` from the robot server version (table in
[SERL controller guide](https://real-world-rl.readthedocs.io/zh-cn/latest/en/reference/serl-controller.html)). Verified end to end in a clean Ubuntu 22.04
(CPython 3.10.12, x86_64, libfranka 0.17.0).

### Docker recipe (optional: wheels for identical robot PCs)

`docker/build-on-host.sh` builds both wheels for CPython 3.10,
manylinux_2_34_x86_64, libfranka 0.17.0 (all overridable by environment
variables):

1. clones franky at `v2.0.0` (`e7e63c7`), checks the commit and that the
   checkout is unmodified;
2. builds franky's **own** build image from franky's unmodified
   `docker/build/Dockerfile` (`--build-arg LIBFRANKA_VERSION=0.17.0`);
3. runs `docker/build-wheels.sh` in a container named `rwrl-franky-build`:
   franky's own `manylinux-build-wheels` (with
   `FRANKY_LIBFRANKA_VERSION_LABEL=0.17.0`, like franky's CI), then this
   package with scikit-build-core against the installed franky wheel, then
   `auditwheel`, CTest and pytest in a fresh venv holding only the two wheels.

```bash
WORK=$HOME/data0/rwrl-franky-build native/serl_controller/docker/build-on-host.sh
ls $WORK/out/wheels   # franky_control-2.0.0+libfranka.0.17.0-cp310-...whl, serl_controller-...whl,
                      # BUILD_INFO.txt, SHA256SUMS; logs in $WORK/out/logs
```

No GPU is needed or requested. `PIP_INDEX_URL`, `PIP_DEFAULT_TIMEOUT` and
`PIP_RETRIES` are forwarded into the container if set (on 6017, pypi.org timed
out and `PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple/` was used).

Note that the rebuilt franky wheel bundles its own `libfranka-<hash>.so`
(hash `ed02dacf` in the first build, `108ae256` in the official wheel), so the
serl_controller wheel cannot load next to the official franky wheel: both
wheels are always installed together.

### Installing a wheel pair on the robot PC

Do not mix the official franky wheel with this one. In the project environment:

```bash
pip uninstall -y franky-control serl-controller
W=path/to/wheels                                       # one build directory: both wheels + SHA256SUMS
(cd $W && sha256sum -c SHA256SUMS)
pip install $W/franky_control-*.whl $W/serl_controller-*.whl
rwrl doctor                                            # sdk:serl_controller must PASS
```

Then set `cartesian_controller: serl` (and `realtime: enforce`) in the station.
To go back, switch the station to `impedance` or `internal` and reinstall the
official franky wheel (`rwrl doctor --install-sdk`).

### Development builds

Core and golden test, no franky, no Python (needs Eigen 3.4 and CMake ≥ 3.24;
doctest is fetched with FetchContent unless installed):

```bash
cd native/serl_controller
cmake --preset dev && cmake --build --preset dev -j && ctest --preset dev
```

A law-only Python module: `pip wheel . -C cmake.define.SERL_CONTROLLER_WITH_FRANKY=OFF`.

### Lint (clang-tidy, clang-format)

`.clang-tidy` enables `bugprone-*`, `cppcoreguidelines-*`, `modernize-*`,
`performance-*`, `google-explicit-constructor`, a `misc-*` subset (const-correctness,
include-cleaner, ...) and a `readability-*` subset, all as errors; the file lists each
disabled check and why. `.clang-format` is the style. `tools/lint.sh <build-dir>` runs
both over this package's own code (headers, `src/`, `python/bindings.cpp`,
`tests/cpp/*`), never over the vendored originals, the ROS stand-ins or the golden
harnesses. Without franky (`dev` preset) the franky translation units are skipped;
the complete run is in franky's build image:

```bash
docker run --rm -v $WORK/src/franky:/src/franky:ro -v $PWD:/src/serl_controller:ro \
  -v $WORK/out/wheels:/wheels:ro --entrypoint /bin/bash <rwrl-franky-build image> \
  /src/serl_controller/docker/lint-in-image.sh      # clang-tidy / clang-format 18.1.8 from PyPI
```

### C++ conventions

C++17, C++ Core Guidelines. Ownership is by value or `std::shared_ptr` (franky's
motions); raw pointers are non-owning (`SerlParams::Field::name` points at a literal,
franky's `robot()`). The real-time path never locks, blocks or throws on valid input; the
cross-thread scheme (franky's `WaitFreeTripleBuffer` mailboxes, one user-side mutex, the
`finish()` flag) is documented at the top of `motion.hpp` and `joint_reset_motion.hpp`.
The one deliberate deviation is the per-tick heap allocation of
`SerlCartesianImpedanceLaw::update()` (dynamic-size Eigen temporaries and the SVD
pseudo-inverse), kept from SERL because fixed-size types change Eigen's evaluation and
break the bit-identical golden comparison (`law.hpp`, "Allocation");
`SerlJointPositionMotion` does not allocate.

## Tests

- `tests/cpp/test_golden_equivalence.cpp` compiles SERL's **original**
  `cartesian_impedance_controller.cpp` (vendored verbatim, built against the
  minimal ROS stand-ins in `tests/cpp/golden/ros_stubs/`) and drives it and
  `SerlCartesianImpedanceLaw` with the same random state sequences and events
  (target changes with hemisphere flips, partial and out-of-range parameter
  updates, asymmetric clips, near-singular and rank-deficient Jacobians,
  saturating `tau_J_d`, restarts). 7 scenarios, 220 000 ticks. Every tick it
  compares the torque and the internal state (K, D, Ki and their targets, both
  nullspace stiffnesses, position / orientation and their targets, error,
  error_i, clip bounds). Acceptance (`tests/cpp/tolerance.hpp`, the only place it is defined):
  per component `abs ≤ 1e-12` **and** `rel ≤ 1e-12`. The port is
  bit-identical (max abs and rel error 0 for every quantity). It also checks
  the parameter table against the vendored `.cfg`.
- `tests/cpp/test_joint_reset_golden.cpp` compiles SERL's **original**
  `joint_position_controller.cpp` (vendored verbatim; the ROS stand-ins'
  `ros::Duration` reproduces roscpp_core's rostime arithmetic) and drives it and
  `SerlJointPositionLaw` like franka_control does: `starting()` + `update(0)`, then
  whole-millisecond periods (1 ms, and 2–5 ms for lost packets), through and past
  the 10 s blend. 5 scenarios, 62 controller starts, 860 000 ticks; every tick
  compares the commanded joint positions and the elapsed time (`toSec()` and ns)
  with the same tolerance header. Result: bit-identical (max abs and rel error 0).
  It also checks `periodNsFromMilliseconds(ms) == ros::Duration(ms / 1000.0)` for
  0–100 000 ms. The same file runs franka_hw's whole position command path: the original
  controller plus the ORIGINAL `joint_limits_interface::PositionJointSoftLimitsHandle`
  (ros_control, vendored verbatim under `golden/ros_control_original/` with `SHA256SUMS`, built
  from the URDF data as franka_hw does, reset / enforced as franka_control_node does) against
  `SerlJointPositionLaw` + `PositionJointSoftLimits`, with the measured q following the sent
  command (plus noise and a jump on the first enforced tick). 6 scenarios (Panda and FR3; inside
  the limits as a control, target at / beyond a limit, start outside a limit, velocity cap after a
  measured-q jump), 60 starts, 500 000 ticks, 129 150 of them with the soft limits changing the
  command. Result: bit-identical (max abs and rel error 0 for both the controller and the sent
  command). The limit tables are checked against the vendored `joint_limits.yaml` files and the
  `franka-limits` macro (`golden/franka_ros_original/`).
- `tests/cpp/test_motion.cpp` (needs franky): the motions equal the bare laws
  with the same events; mailboxes under concurrent writers; `finish()` ends the
  joint reset with a hold.
- `tests/python/test_binding.py`: binding, params, the franky derivations
  (`isinstance(m, franky.BaseTorqueMotion)`, `franky.TorqueReaction(cond, m)`,
  `isinstance(j, franky.BaseJointPositionMotion)`, `franky.JointPositionReaction(cond, j)`),
  offline stepping, `build_info.json` == the compiled-in copy. Skipped with `importorskip` when the extension is not
  installed.

## Repository integration

- The main `real_world_rl` package stays pure Python: the root `pyproject.toml`
  only packages `real_world_rl*` (`[tool.setuptools.packages.find]`), so
  `native/` is never picked up.
- The repository tests (`python -m pytest tests`) do not collect
  `native/serl_controller/tests`; that directory has its own pytest config
  (`testpaths = ["tests/python"]`) and skips itself without the compiled
  module.
- Build artefacts (`build/`, `*.so`, `*.whl`, `CMakeUserPresets.json`,
  `compile_commands.json`, ...) are ignored by the root `.gitignore`.
