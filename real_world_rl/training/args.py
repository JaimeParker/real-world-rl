"""Shared, algorithm-agnostic Args mixin + real-robot-only backend config for
real-robot training entrypoints.

``RealWorldFrankaArgs`` fields are common to every real-robot training method
(SERL, HIL-SERL, ...): actor/learner role dispatch, sync-server network
topology, and the ``ActorLoop``/``LearnerLoop`` cadence knobs. Algorithm-
specific fields (e.g. SERL's RLPD hyperparameters) belong in each method's own
``real_world_rl/training/entrypoints/<method>.py``, not here.
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

from real_world_rl.envs.bimanual.config import BimanualRealRobotEnvConfig
from real_world_rl.envs.real_robot.config import RealRobotEnvConfig


@dataclass
class RealWorldFrankaArgs:
    role: Literal["actor", "learner"] = "actor"

    # Actor: address of the learner's sync server. Learner: bind address for
    # that same server. Both processes must agree on the port. Cross-machine
    # by default (SERL's own topology) -- point this at the learner's real,
    # network-reachable address for a two-machine deployment.
    sync_host: str = "127.0.0.1"
    sync_port: int = 6000
    # Durable actor identity; the outbox retains a generated run id when unset.
    run_id: Optional[str] = None
    actor_id: str = "actor"
    actor_outbox_path: Optional[str] = None
    sync_startup_timeout: float = 30.0
    sync_drain_timeout: float = 30.0
    outbox_max_records: int = 100_000
    outbox_max_bytes: int = 1_073_741_824
    sync_batch_size: int = 32
    admission_capacity: int = 10_000
    ingest_batch_size: int = 256

    control_hz: float = 10.0
    deterministic_actor: bool = False
    train_freq: int = 1
    publish_freq: int = 100

    # "franka_real" (FrankaRealEnv over the HTTP bridge, ``--franka_real.*``),
    # "real_robot" (any robot_infra robot, ``--real_robot.*``) or "real_robot_bimanual"
    # (two robot_infra arms as one env, ``--real_robot_bimanual.{left,right}.*``).
    env_backend: Literal["franka_real", "real_robot", "real_robot_bimanual"] = "franka_real"
    real_robot: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)
    real_robot_bimanual: BimanualRealRobotEnvConfig = field(default_factory=BimanualRealRobotEnvConfig)


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

    # Reward from the operator's buttons (TeleopEventRewardWrapper, on the envs that open the
    # teleop device: actor, eval, demo recorder): a ``success`` event pays at least
    # teleop_success_reward and terminates the episode, a ``failure`` event truncates it. Works
    # with or without --classifier-checkpoint (the larger reward wins).
    reward_from_teleop_events: bool = False
    teleop_success_reward: float = 1.0

    # Gripper-flip reward shaping added to RLPDHybrid's discrete critic
    # target on top of the raw env reward (HIL-SERL's grasp_penalty) --
    # see real_world_rl/envs/wrappers/grasp_penalty.py.
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
    # "leader_arm": the same part read as a joint-space leader arm (GELLO over the daemon), with
    # the --teleop-onset policy. "gello": a GELLO on this machine. "fake": a scripted source for
    # dry runs without hardware. "bimanual": one --teleop-arm-device per arm of a
    # real_robot_bimanual env. "spacemouse": a SpaceMouse over the cartesian command tracker
    # (button 0 takes over); "spacemouse_twist": the old raw twist, sim only, not for real robots.
    # See docs/teleop.md for every device.
    teleop_selected_parts: list[str] = field(default_factory=list)
    teleop_idle_behavior: Literal["hold", "policy"] = "hold"
    teleop_device: Literal[
        "pico",
        "spacemouse",
        "spacemouse_twist",
        "robot",
        "fake",
        "gello",
        "leader_arm",
        "keyboard",
        "leader_cartesian",
        "bimanual",
    ] = "pico"
    # tracker: button toggles and a persistent velocity-limited target (default).
    # serl: raw twist, per-step motion/button takeover and neutral idle gripper.
    spacemouse_mode: Literal["tracker", "serl"] = "tracker"
    # --teleop-device bimanual: the device of each arm ("robot", "leader_arm" or "gello", as the
    # single-arm devices of the same name, built over that arm's own env).
    teleop_arm_device: Literal["robot", "leader_arm", "gello"] = "leader_arm"
    # Declares the robot's teleop part for --teleop-device robot/leader_arm over a remote robot,
    # whose part is only known after connecting (checked then; a mismatch raises DeviceFault).
    robot_teleop_action_mode: Optional[Literal["cartesian_delta", "joint_abs", "joint_delta"]] = None
    robot_teleop_has_gripper: bool = True
    # A second device for the operator's buttons (take over / hand back, episode end, success/
    # failure/rerecord, quit): the env then takes its motion from --teleop-device and intervention
    # plus events from this one (CompositeTeleopSource). "keyboard" reads this machine's keyboard
    # (pynput, needs a desktop session), "stdin" the terminal running the process; both use the
    # keys t/h/s/f/r/Esc/q. None: --teleop-device alone. (The SpaceMouse is no longer an event
    # device: use --teleop-device spacemouse with --teleop-event-device stdin or keyboard.)
    teleop_event_device: Optional[Literal["keyboard", "stdin"]] = None
    # --teleop-device leader_cartesian/spacemouse: the persistent command target of the follower
    # (CartesianTargetTracker). It leads the measured TCP pose by at most lead_cap (the excess is
    # dropped) and advances towards the leader (the SpaceMouse: at full deflection) at most
    # target_speed. xyz_scale_m/rot_scale_rad of the env must be at least the lead caps. Recorded in
    # the experiment contract (teleop_control).
    teleop_lead_cap_m: float = 0.05
    teleop_lead_cap_rad: float = 0.3
    teleop_target_speed_m_s: float = 0.05
    teleop_target_speed_rad_s: float = 0.3
    # Leader-arm takeover (--teleop-device gello/leader_arm, and every arm of bimanual): "ramp"
    # rate-limits joint_abs targets to ramp_step_rad per step until the follower caught up; "arm"
    # ignores the toggle until the leader is within arm_tolerance_rad of the follower.
    teleop_onset: Literal["ramp", "arm"] = "ramp"
    teleop_arm_tolerance_rad: float = 0.1
    teleop_ramp_step_rad: float = 0.05
    # Yaml mapping merged into the --teleop-device source kwargs (after the ones derived from the
    # args above, so it overrides them): device options such as port/joint_ids/joint_offsets_rad/
    # joint_signs for gello, device_index/deadzone/invert_axes for spacemouse, hold_s, read_timeout_s. A
    # robot_infra station yaml also works for gello: its teleop: options become the source kwargs.
    # For bimanual: per-arm sections keyed by arm name (left: {...}) plus common keys, or a station
    # yaml with robots: {left, right}.
    teleop_options_yaml: Optional[str] = None
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
