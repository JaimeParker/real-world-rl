# joint_controller

A 1 kHz Franka joint controller for teleoperation from sparse joint targets (a GELLO leader arm
at 10–30 Hz), running inside [franky](https://github.com/TimSchneider42/franky)'s control loop as
a `franky::Motion<franka::Torques>`. franky's own joint motions step to each target; this motion
turns the stream of targets into a smooth reference with continuous position, velocity and
acceleration inside velocity and acceleration limits, and tracks that reference with GELLO's joint
impedance law.

Like `native/serl_controller`, it is a separate C++/pybind11 package that links against an
**unmodified** franky.

| Part | What it is |
| --- | --- |
| `ReferenceGenerator`, `Quintic` (`include/joint_controller/planner.hpp`, `src/planner.cpp`) | The online reference: one quintic per joint, replanned from the current reference state on every target, with one common duration within the limits. Eigen only. |
| `TrajectoryImpedanceLaw` (`law.hpp`, `src/law.cpp`) | Reference, command timeout, velocity filter, impedance torque, torque-rate limit. Eigen only, no franky. |
| `TrajectoryImpedanceParams` (`params.hpp`) | All 12 parameters, partial update with validation (invalid values are rejected, never clamped). |
| `TrajectoryImpedanceMotion` (`motion.hpp`) | The law as a franky torque motion. Lock-free command and parameter mailboxes and a per-tick telemetry snapshot. |
| `joint_controller` Python package | pybind11 bindings. `TrajectoryImpedanceMotion` is a subclass of `franky.BaseTorqueMotion`. `build_info()` / `build_info.json` say which franky build it belongs to. |

## Provenance

The joint impedance law, its velocity filter and the default gains come from the public GELLO
controller `wuphilipp/gello_software` at commit `9fc8c26`
(`ros2/src/franka_fr3_arm_controllers/`: `src/joint_impedance_controller.cpp`,
`calculateTauDGains_`; `config/controllers.yaml`; `kJointStateTimeout` in
`include/.../joint_impedance_controller.hpp`), Apache License 2.0, Copyright (c) 2025 Franka
Robotics GmbH. The joint position limits are franka_description's (franka_ros @ `ddd2fff`,
Apache-2.0). The reference generator is our own, derived from the quintic boundary-value
problem. `LICENSE` lists every part and its licence; `LICENSE.Apache-2.0` is the Apache licence
text from the GELLO directory.

## The control law

Per tick, with `dt` the measured period (franky's `time_step`: 0 on the first tick, 1 ms
normally, 2–5 ms after lost packets):

1. **Reference.** `(q_ref, dq_ref, ddq_ref)` advances by `dt`. A new target is rejected if it is
   not finite, otherwise clipped into `[lower_limit + limit_margin, upper_limit - limit_margin]`.
   Every joint is replanned from the current reference state (so position, velocity and
   acceleration stay continuous) to `(target, 0, 0)` with one quintic per joint and a common
   duration `T` (below), then holds the target exactly. If several targets arrive within one tick,
   only the newest is used.
2. **Timeout.** No target for `command_timeout_s`: bounded stop and hold (below); `request_stop()`
   does the same at once. The next target resumes tracking.
3. **Velocity filter (GELLO).** `dq_f = (1 - alpha) dq_f + alpha dq`, `dq_f = 0` at start.
4. **Torque (GELLO).** `tau = K (q_ref - q) - D dq_f`, plus the model's coriolis torque if
   `compensate_coriolis` (never gravity: libfranka adds it), then per joint
   `|tau - tau_prev| <= max_delta_tau`.
5. **Start.** The reference starts at the measured `q`, at rest, so the spring term is 0 at tick 0.
   `tau_prev` is franky's previous torque command when a torque motion is replaced, else 0.

Upstream GELLO feeds the raw GELLO joint state into `q_goal`; here `q_goal` is `q_ref`.

### Duration selection

For each candidate `T` the quintic per joint is fixed (`b0..b5` in `planner.hpp`). A candidate is
feasible when every joint's `|v| <= max_velocity`, `|a| <= max_acceleration` and position within
`[lower_limit, upper_limit]` over the whole segment. These are checked by evaluating the
polynomial at its exact extrema: the roots of the jerk (a quadratic, closed form), then the roots
of the acceleration and of the velocity on the monotone pieces between them (bracketed Newton);
a cheap sufficient bound skips the root finding when the segment is clearly inside the limits.
`T` starts at the largest of `min_segment_duration_s` and two per-joint lower bounds that hold for
any trajectory (`|h| / v_max`, and the bang-bang minimum time of a double integrator with
`|a| <= a_max` from `(p0, v0)` to rest at the target), grows by ×1.1 per step until all joints are
feasible, and the last step is bisected 20 times; the feasible end is used. From rest the result
is the exact minimum (feasibility is then monotone in `T`).

From a moving state no common `T` may exist: one joint still accelerating has a feasible window
only at short `T`, another joint with a long way to go needs a long `T`. Then the target stays
pending, the reference brakes with the bounded stop and the plan is retried every tick (at rest
every target inside the limits is feasible). In a 300 000-tick soak with jittery targets and
large jumps (`tests/cpp/test_soak.cpp`) this affected 2.7 % of the ticks, the longest wait was
0.46 s; with a smooth leader motion at 10, 20 or 30 Hz it never happened.

### Bounded stop

Per joint, from `(p0, v0, a0)` to rest at `p0 + v0 T / 2 + a0 T² / 12` (the quintic family member
whose fifth-order coefficient is 0). Its velocity
`v(s) = v0 (1 - 3s² + 2s³) + a0 T s (1 - s)²` is unimodal: `|v|` rises only while the reference is
still accelerating, then falls monotonically to 0 without reversing. `T` is the smallest grid
point within the limits (velocity, acceleration, no reversal, position limits); a decelerating
joint can always stop with `T = 3 |v0| / |a0|` (`v = v0 (1 - s)³`). Joints stop with their own
durations. A joint with no feasible stop yet (at the velocity limit and still accelerating) keeps
its current, feasible, rest-ending segment and is retried every tick. The stop point is ahead of
the reference, not the reference position at the timeout: stopping exactly there would make every
moving joint reverse.

### Tracking lag

Every replan ends at rest at the newest target, so a moving leader arm is followed with a lag of
about the braking distance. The lag shrinks with `max_acceleration`: for a smooth leader at
~0.5 rad/s it is about 0.20 rad at 2 rad/s², 0.06 rad at the default 4 rad/s² and 0.04 rad at
8 rad/s²; at 0.19 rad/s it stays within 0.02 rad. Raise `max_acceleration` in the station's
`joint_trajectory:` block for faster teleop, lower it for gentler motion.

### Cost

`update()` is `noexcept`, allocates nothing (fixed-size Eigen) and does bounded work (at most 50
grid + 20 bisection feasibility checks per plan). On a WSL2 laptop (`-O2`): about 1 µs per tick on
average in the soak, 35 µs for an average replan; the worst tick, measured as the minimum of 5
repetitions of each soak tick to remove OS jitter, took 104 µs (a failed plan plus the start of
the brake in the same tick). Single-run maxima there reach ~0.5 ms, which is the laptop's jitter.

## Parameters

| Key | Default | Rule | Source |
| --- | --- | --- | --- |
| `stiffness` (7) | 240, 240, 240, 240, 100, 60, 20 Nm/rad | ≥ 0 | GELLO `controllers.yaml` `k_gains` |
| `damping` (7) | 20, 20, 20, 10, 10, 10, 5 Nms/rad | ≥ 0 | GELLO `d_gains` |
| `velocity_filter_alpha` | 0.99 | [0, 1] | GELLO `k_alpha` |
| `max_velocity` (7) | 0.6 (joints 1–4), 0.75 (5–7) rad/s | > 0 | spec |
| `max_acceleration` (7) | 4.0 rad/s² | > 0 | chosen: tracking lag ≈ braking distance (0.06 rad at a 0.5 rad/s leader; 0.20 rad at 2.0) |
| `min_segment_duration_s` | 0.1 | > 0 | spec |
| `lower_limit`, `upper_limit` (7) | required; `default_params(robot_model)` fills franka_description's | lower + 2·margin < upper | franka_ros `joint_limits.yaml` |
| `limit_margin` | 0.05 rad | ≥ 0 | spec |
| `command_timeout_s` | 0.5 | > 0 | GELLO `kJointStateTimeout` |
| `max_delta_tau` | 1.0 Nm per tick | > 0 | spec (as SERL's `delta_tau_max`) |
| `compensate_coriolis` | `true` | bool | spec |

All values must be finite. `update_params` merges a partial dict, validates the result and
either applies all of it (at the next tick) or raises `ValueError` and changes nothing. A change
of a reference limit replans the current target (re-clipped) from the current state.

## Python use

```python
import franky                       # the franky wheel from the SAME build
import joint_controller as jc

robot = franky.Robot("172.16.0.2")
params = jc.default_params("fr3")   # "panda" | "fr3": joint limits filled in
params["max_velocity"] = [0.5] * 7
m = jc.TrajectoryImpedanceMotion(params)   # a franky.BaseTorqueMotion
robot.move(m, asynchronous=True)           # starts at the measured q, holds it
for q in gello_stream():                   # 10 - 30 Hz
    m.set_target(q)                        # False if not finite
m.update_params({"stiffness": [200] * 4 + [80, 50, 15]})   # partial, returns the effective dict
t = m.telemetry()                          # q_ref, dq_ref, ddq_ref, target, tau_cmd, segment_t, ...
m.request_stop()                           # bounded stop, keeps running; next set_target resumes
robot.stop()
```

| Call | Returns |
| --- | --- |
| `TrajectoryImpedanceMotion(params: dict)` | defaults updated with `params` (`lower_limit`, `upper_limit` required); `ValueError` otherwise |
| `set_target(q)` | `bool`: `False` (nothing sent) if a value is not finite; a wrong shape raises `TypeError` |
| `update_params(updates: dict)` | the full effective dict (applied at the next tick); `ValueError` for unknown keys, wrong kinds, invalid values |
| `params()` | the full dict most recently requested |
| `telemetry()` | `ticks`, `rel_time`, `q`, `dq`, `dq_filtered`, `coriolis`, `q_ref`, `dq_ref`, `ddq_ref`, `target` (last accepted target, clipped), `goal` (where the reference comes to rest: target or stop point), `tau_cmd`, `segment_t`, `segment_T`, `time_since_target`, `timed_out`, `stopped`, `target_pending`, `plan_retries`, `applied_command_seq`, `applied_params_seq` |
| `request_stop()` | `None` |
| `jc.default_params(robot_model)` | the default dict for `"panda"` / `"fr3"` |
| `jc.validate_params(params)` | the effective dict of a full parameter dict, or `ValueError` (no robot needed) |
| `jc.build_info()` | `build_info.json` (no import of the extension needed) |

Vectors are returned as numpy arrays of shape (7,), scalars as `float`, `compensate_coriolis` as
`bool`. `TrajectoryImpedanceLaw(params)` (`starting`, `set_target`, `request_stop`, `step(q, dq,
coriolis, dt)`, `telemetry`, `apply_params`) runs the law offline, without franky or a robot.

franky's `Robot.move` defaults (`limit_rate=False`, `cutoff_frequency=100`) are fine: the torque
rate is already limited to `max_delta_tau` per tick inside the law.

## Build

### ABI coupling rule

`TrajectoryImpedanceMotion` derives, in C++ and in Python, from franky's `Motion<franka::Torques>`
and its pybind11 class `franky.BaseTorqueMotion`, exactly like `serl_controller`. **Build franky,
joint_controller and serl_controller in the same build (same machine or container, compiler,
pybind11 3.0.4, libfranka and franky headers) and deploy them together.** Never install
joint_controller next to a franky from a different build (including the PyPI wheels). The CMake
setup, the `FRANKA_0_*` defines, the two layouts (`wheel`: the franky wheel's bundled
`libfranka-<hash>.so`, RPATH `$ORIGIN/../franky:$ORIGIN/../franky_control.libs`; `source`: a
system libfranka) and `build_info.json` are those of `serl_controller` (see its README); the
JSON has `joint_controller_version` and `gello_commit` instead of the SERL fields.

### On the robot PC, without Docker (`scripts/install-on-ubuntu.sh`)

With the project environment active, from the repository root:

```bash
LIBFRANKA_VERSION=0.17.0 bash native/joint_controller/scripts/install-on-ubuntu.sh
```

If the environment already has franky v2.0.0 built from source against that libfranka (by
`native/serl_controller/scripts/install-on-ubuntu.sh`, same `WORK`, default
`~/.cache/rwrl-native-build`), it is reused. Otherwise the script first runs serl_controller's
script, the repository's one franky build path, which builds libfranka and franky and rebuilds
serl_controller against them. Then it pip-installs this package with `--no-build-isolation`, runs
CTest (incl. the franky motion test), the Python tests, a smoke import and a pairing check
(`build_info.json`'s libfranky sha256 vs the installed `libfranky.so`). `REBUILD_FRANKY=1` forces
the franky step. Logs: `$WORK/logs/jc_*`.

### Docker recipe (wheels for identical robot PCs)

```bash
WORK=$HOME/data0/rwrl-franky-build native/joint_controller/docker/build-on-host.sh
ls $WORK/out/wheels   # franky_control-...whl, joint_controller-...whl, serl_controller-...whl,
                      # BUILD_INFO.txt, SHA256SUMS; logs in $WORK/out/logs
```

Same steps as serl_controller's recipe (franky's own unmodified build image, franky's
`manylinux-build-wheels`, scikit-build-core against the franky wheel, `auditwheel repair` with
franky's libraries excluded, CTest and pytest in a fresh venv holding only the built wheels), but
it also builds `native/serl_controller` in the same container, so the three wheels form one set
(`WITH_SERL=0` builds franky + joint_controller only). Install them together:

```bash
pip uninstall -y franky-control serl-controller joint-controller
(cd $W && sha256sum -c SHA256SUMS)
pip install $W/franky_control-*.whl $W/serl_controller-*.whl $W/joint_controller-*.whl
```

### Development builds

Core and C++ tests, no franky, no Python (Eigen 3.4, CMake ≥ 3.24; doctest is fetched with
FetchContent unless `FETCHCONTENT_SOURCE_DIR_DOCTEST` points at a copy):

```bash
cd native/joint_controller
cmake --preset dev && cmake --build --preset dev -j && ctest --preset dev
```

A law-only Python module (`WITH_FRANKY` is `False`, no `TrajectoryImpedanceMotion`):
`pip install . -C cmake.define.JOINT_CONTROLLER_WITH_FRANKY=OFF`.

### Lint (clang-tidy, clang-format)

`.clang-tidy` and `.clang-format` are serl_controller's (the disabled checks are listed with
reasons). `tools/lint.sh <build-dir>` runs both over `include/`, `src/`, `python/bindings.cpp` and
`tests/cpp/`; `docker/lint-in-image.sh` is the complete run inside franky's build image.

### C++ conventions

C++17, C++ Core Guidelines. Ownership by value or `std::shared_ptr` (franky's motions); raw
pointers are non-owning (`TrajectoryImpedanceParams::Field::name`, franky's `robot()`). The
real-time path never locks, blocks, allocates or throws; the cross-thread scheme (franky's
`WaitFreeTripleBuffer` mailboxes, one user-side mutex) is documented at the top of `motion.hpp`.

## Tests

- `tests/cpp/test_planner.cpp`: quintic boundary values and exact rest at the end; extrema equal
  dense sampling; from rest the chosen `T` equals the closed-form minimum
  (`max(15/8 |h| / v_max, sqrt(10/√3 |h| / a_max), T_min)`); random moving start states and
  targets: limits respected at every 1 ms sample (`|v| <= v_max (1 + 1e-9)`, same for `a`), one
  common duration, `T (1 - 1e-5)` infeasible, endpoint exact; retargeting at 10–30 Hz keeps `q`,
  `dq`, `ddq` continuous; `min_segment_duration_s`; refused plans change nothing; the bounded stop
  (unimodal `|v|`, monotone to rest, no reversal, within limits); planning cost.
- `tests/cpp/test_law.cpp`: start seeds `q_ref = q` (zero torque at tick 0); clipping with margin;
  non-finite targets rejected with the state unchanged; the torque law and filter recurrence on
  scripted inputs, with and without coriolis; `max_delta_tau` per tick; the timeout stop (fires
  after 500 ticks, unimodal `|v|` to rest, holds) and resume; `request_stop`; live limit changes.
- `tests/cpp/test_params.cpp`: defaults and sources, required limits, partial updates, every
  rejection rule (negative gains, `alpha` outside [0, 1], `lower >= upper`, non-positive limits,
  non-finite values, unknown keys, wrong kinds), atomicity.
- `tests/cpp/test_soak.cpp`: 2 × 300 000 ticks (Panda, FR3) against a simulated arm with GELLO-like
  retargeting at 10–30 Hz, jumps beyond the limits, non-finite targets, pauses past the timeout,
  `request_stop`, lost packets and live gain changes, checking every invariant every tick; and a
  smooth leader arm at 10 / 20 / 30 Hz.
- `tests/cpp/test_motion.cpp` (needs franky): the motion equals the bare law with the same events;
  mailboxes under concurrent writers; `request_stop`; a command before the start.
- `tests/python/test_binding.py`: defaults, validation, the offline law, the franky derivation
  (`isinstance(m, franky.BaseTorqueMotion)`, `franky.TorqueReaction(cond, m)`), motion == law,
  `build_info.json` == the compiled-in copy. Skipped with `importorskip` when the extension is not
  installed.

## Repository integration

As for serl_controller: the root `pyproject.toml` packages only `real_world_rl*`, the repository
tests do not collect `native/joint_controller/tests` (own pytest config, `testpaths =
["tests/python"]`), and build artefacts are ignored by the root `.gitignore`.
