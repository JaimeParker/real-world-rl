"""Shared, algorithm-agnostic Args mixin + real-robot-only backend config for
real-robot training entrypoints.

``RealWorldFrankaArgs`` fields are common to every real-robot training method
(SERL, HIL-SERL, ...): actor/learner role dispatch, sync-server network
topology, and the ``ActorLoop``/``LearnerLoop`` cadence knobs. Algorithm-
specific fields (e.g. SERL's RLPD hyperparameters) belong in each method's own
``rl_garden_real_world/training/entrypoints/<method>.py``, not here.
``HilEnvArgs`` holds the env/teleop/classifier/demo fields every HIL-style
method (hil_serl, residual, SiLRI) shares.

``FrankaRealConfig`` lives here (not in ``rl_garden/common/env_args.py``,
where the other env backends' configs live) because it's exclusively used by
real-robot training methods -- online/offline/off2on never target
``env_backend="franka_real"``. Each method's Args class opts in to it as a
field explicitly (e.g. ``SerlArgs.franka_real``), the same way
``rl_garden/training/online/entrypoints/rlpd.py``'s ``RLPDArgs`` composes
``EnvBackendArgs`` itself rather than inheriting it implicitly.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Optional, Sequence

from rl_garden_real_world.envs.real_robot.config import RealRobotEnvConfig


@dataclass
class RealWorldFrankaArgs:
    role: Literal["actor", "learner"] = "actor"

    # Actor: address of the learner's sync server. Learner: bind address for
    # that same server. Both processes must agree on the port. Cross-machine
    # by default (SERL's own topology) -- point this at the learner's real,
    # network-reachable address for a two-machine deployment.
    sync_host: str = "127.0.0.1"
    sync_port: int = 6000

    control_hz: float = 10.0
    deterministic_actor: bool = False
    train_freq: int = 1
    publish_freq: int = 100

    # "franka_real" (FrankaRealEnv over the HTTP bridge, ``--franka_real.*``) or
    # "real_robot" (any robot_infra robot, ``--real_robot.*``).
    env_backend: Literal["franka_real", "real_robot"] = "franka_real"
    real_robot: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)


@dataclass
class FrankaRealConfig:
    """Real-Franka env settings. CLI prefix: ``--franka_real.<field>``"""

    bridge_url: str = "http://localhost:5000"
    action_scale_pos: float = 0.02
    action_scale_rot: float = 0.1
    gripper_threshold: float = 0.5
    max_episode_steps: int = 100
    # JSON-encoded dict forwarded verbatim to FrankaRealEnvConfig, taking
    # precedence over the named fields above. Escape hatch for cell-specific
    # settings (safety_box_low/high, camera_keys, device) that don't belong
    # as named CLI fields here (see adding-env-backend.md Step 3).
    env_kwargs_json: str = "{}"


@dataclass
class HilEnvArgs:
    """Fields shared by HIL-style real-robot entrypoints (hil_serl, residual,
    SiLRI): what ``env_building.build_hil_env`` and the demo/snapshot paths of
    ``HilSerlLearnerLoop`` read from the Args. Combine with ``RealWorldFrankaArgs``
    and an rl-garden Args class, as ``HilSerlArgs`` does."""

    # Only used when role="eval": number of deterministic episodes to run.
    eval_n_trajs: int = 10

    classifier_checkpoint: Optional[str] = None
    classifier_threshold: float = 0.5
    classifier_image_keys: Sequence[str] = field(default_factory=tuple)

    # Gripper-flip reward shaping added to RLPDHybrid's discrete critic
    # target on top of the raw env reward (HIL-SERL's grasp_penalty) --
    # see rl_garden_real_world/envs/wrappers/grasp_penalty.py.
    use_grasp_penalty: bool = False
    grasp_penalty_value: float = -0.05

    # Records each actor-side episode to <record_video_dir>/episode_*.mp4
    # via ffmpeg -- actor role only. None disables it.
    record_video_dir: Optional[str] = None

    # Replay-buffer image-frame dedup (MemoryEfficientDictReplayBuffer) --
    # only meaningful when obs.frame_stack > 1; a memory-usage optimization,
    # not a correctness change, for long image-heavy real-world sessions.
    memory_efficient_buffer: bool = False

    # "robot": the teleop part of the real_robot env's robot (robot yaml ``teleop:`` section).
    # "fake": a scripted source for dry runs without hardware.
    teleop_device: Literal["pico", "spacemouse", "robot", "fake"] = "pico"
    # Declares the robot's teleop part for --teleop-device robot over a remote robot, whose
    # part is only known after connecting (checked then; a mismatch raises DeviceFault).
    robot_teleop_action_mode: Optional[Literal["cartesian_delta", "joint_abs", "joint_delta"]] = None
    robot_teleop_has_gripper: bool = True
    # Converts FrankaRealEnv's raw quaternion state slice to a rotation
    # vector (matching the action side's own rotvec convention). Off by
    # default -- changes "state"'s observation shape (20 -> 19), so it's an
    # explicit opt-in, not a silent default change.
    convert_obs_rotation: bool = False

    # Growing human-intervention demo buffer (DemoInterventionMixin), mixed
    # into training via RLPD's existing offline_replay_buffer/
    # offline_data_ratio slot -- see rl_garden/buffers/demo_intervention.py.
    # Not supported together with --offline_dataset (RLPD's static prior
    # dataset) in this round; both use the same slot.
    demo_buffer_size: int = 100_000
    demo_data_ratio: float = 0.5
    # Crash-recovery pkl snapshot cadence (received transitions between
    # snapshots), mirroring HIL-SERL's own buffer_period.
    buffer_period: int = 1000
    # Pre-collected demo transitions (HIL-SERL's --demo_path equivalent),
    # glob patterns over pkl files in the same transition-dict format as the
    # buffer snapshots above. Loaded into the demo buffer unconditionally on
    # every start, before crash-recovery snapshot reload.
    demo_dataset_paths: Sequence[str] = field(default_factory=tuple)

    fwbw: bool = False
    # Only used when fwbw=True. Actor: builds both directions' sync clients
    # (forward on sync_port, backward on sync_port_backward). Learner: which
    # direction this particular learner process owns.
    sync_port_backward: Optional[int] = None
    fwbw_direction: Literal["forward", "backward"] = "forward"
