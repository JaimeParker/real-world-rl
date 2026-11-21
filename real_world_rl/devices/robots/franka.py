"""Franka Panda: 7-DoF arm with a binary gripper."""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from real_world_rl.devices.robot import GripperConfig, Robot, RobotConfig

FRANKA_HOME_JOINTS = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
_REALTIME_MODES = ("ignore", "enforce")
_CARTESIAN_CONTROLLERS = ("impedance", "internal", "serl")
# franka_description's robot models (what serl_controller's joint reset soft limits are keyed by).
ROBOT_MODELS = ("panda", "fr3")
# Datasheet joint position limits q_min/q_max (rad), per robot model: Franka's "Robot and interface
# specifications" (https://frankarobotics.github.io/docs/robot_specifications.html), the FER table
# and the FR3 "Joint-space limits" table. The setup wizard offers them as the joint_limits default.
DATASHEET_JOINT_LIMITS = {
    "panda": ((-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698),
              (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973)),
    "fr3": ((-2.9007, 2.9007), (-1.8361, 1.8361), (-2.9007, 2.9007), (-3.0770, -0.1169),
            (-2.8763, 2.8763), (0.4398, 4.6216), (-3.0508, 3.0508)),
}
# ``control:`` keys only franky's CartesianImpedanceTracker reads (ArmControlConfig.for_mode).
CARTESIAN_TRACKER_CONTROL_KEYS = ("translational_stiffness", "rotational_stiffness", "cartesian_damping",
                                  "translational_error_clip", "rotational_error_clip")
logger = logging.getLogger(__name__)


def serl_ignored_control_keys(config) -> list[str]:
    """``control:`` keys that do nothing because cartesian_controller: serl runs SERL's controller
    for Cartesian servo (the joint servo keys and max_delta_tau / gains_time_constant still apply
    to the joint tracker)."""
    if getattr(config, "cartesian_controller", None) != "serl":
        return []
    return [key for key in CARTESIAN_TRACKER_CONTROL_KEYS if key in (config.control or {})]


@dataclass
class FrankaConfig(RobotConfig):
    TYPE = "franka"
    NUM_JOINTS = 7
    name: str = "franka"
    arm_backend: str = "franky"
    robot_ip: str = "172.16.0.2"
    bridge_url: str = "http://127.0.0.1:5000"
    gripper: GripperConfig | None = field(
        default_factory=lambda: GripperConfig(backend="franky", max_width_m=0.08, mode="binary")
    )
    home_joints: list[float] | None = field(default_factory=lambda: list(FRANKA_HOME_JOINTS))
    relative_dynamics_factor: float = 0.2
    realtime: str = "ignore"
    # impedance: franky's torque-level Cartesian impedance tracker (stiffness from ``control``);
    # internal: the robot's own controller follows Cartesian velocity motions (franky backend);
    # serl: SERL's Cartesian impedance controller (serl_controller wheel, parameters from ``serl``)
    # and SERL's joint reset for move_to_joints (franky backend).
    cartesian_controller: str = "impedance"
    # cartesian_controller: serl: the robot's SERL compliance parameters (compliance_param.cfg names;
    # omitted ones keep the .cfg defaults). Task presets can switch named profiles on top of these.
    serl: dict | None = None
    # Payload {mass_kg, center_of_mass_m: [x, y, z], inertia: 3x3 or 9}, set after every connect
    # (libfranka Robot::setLoad, what HIL-SERL's franka_server /set_load does). None: leave it.
    load: dict | None = None
    # panda (Franka Emika Robot / FER) | fr3 (Franka Research 3): selects franka_description's joint
    # limits for SERL's joint reset soft-limit stage (franka_control's robot:= argument; its default
    # is panda). Confirm the model in Desk; `rwrl doctor` cross-checks it against the server version.
    robot_model: str = "panda"
    # cartesian_controller: serl: {enabled, max_per_episode}; a reflex that ended SERL's controller is
    # cleared on the next servo target and the controller restarted (HIL-SERL's /clearerr before
    # every /pose), at most max_per_episode times per episode. None: the defaults (true, 5).
    serl_auto_recover: dict | None = None

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.realtime not in _REALTIME_MODES:
            raise ValueError(f"realtime '{self.realtime}' invalid; choose from {list(_REALTIME_MODES)}")
        if self.cartesian_controller not in _CARTESIAN_CONTROLLERS:
            raise ValueError(f"cartesian_controller '{self.cartesian_controller}' invalid; "
                             f"choose from {list(_CARTESIAN_CONTROLLERS)}")
        if self.cartesian_controller in ("internal", "serl") and self.arm_backend not in ("franky", "fake"):
            raise ValueError(f"cartesian_controller: {self.cartesian_controller} needs arm_backend franky")
        if self.robot_model not in ROBOT_MODELS:
            raise ValueError(f"robot_model '{self.robot_model}' invalid; choose from {list(ROBOT_MODELS)}")
        from real_world_rl.devices.control import franka_load, serl_auto_recover_config, serl_param_updates
        if self.serl_auto_recover is not None:
            self.serl_auto_recover = serl_auto_recover_config(self.serl_auto_recover)
        if self.serl is not None:
            self.serl = serl_param_updates(self.serl, where="serl")[0]
        self.load = franka_load(self.load)
        ignored = serl_ignored_control_keys(self)
        if ignored:
            logger.warning("robot '%s': cartesian_controller: serl ignores control: %s for Cartesian servo "
                           "(SERL's parameters come from serl:)", self.name, ignored)


@Robot.register_type
class FrankaRobot(Robot):
    ROBOT_TYPE = "franka"
    CONFIG = FrankaConfig

    def build(self) -> None:
        self._build_standard()
