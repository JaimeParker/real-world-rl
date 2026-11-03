"""Setup wizard questions: a pure function from a hardware probe and the flag values to a
``StationDraft``. Every prompt goes through ``ask``/``echo`` and has a flag; the values of an
existing station (``--from``) are the defaults, and ``--non-interactive`` takes them (or the flag
values) without asking. The joint limits and the servo step cap are written without a question:
their flag, else the ``--from`` value, else the wizard's default."""
from __future__ import annotations

import dataclasses
import ipaddress
import math
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Mapping, Sequence

from robot_infra.devices import (
    CameraConfig,
    DeviceError,
    GripperConfig,
    RobotConfig,
    TeleopConfig,
)
from robot_infra.devices.backends.teleops.gello import (
    DEFAULT_EVENT_KEYS,
    INTERVENTION_MODES,
    MAX_DYNAMIXEL_ID,
)
from robot_infra.devices.backends.teleops.gello_calibrate import (
    FRANKA_CALIBRATION_POSE,
    calibrate_gello,
    wrap_centers_from_joint_limits,
)
from robot_infra.devices.robots.franka import FRANKA_HOME_JOINTS, FrankaConfig
from robot_infra.discovery.autoconfig import DEFAULT_DAEMON_BIND, Station
from robot_infra.setup.probes import SerialBusProbe, SimulatedGelloBus, StationProbe

Ask = Callable[[str], str]
Echo = Callable[[str], None]
CalibrationBusFactory = Callable[[str, int, Sequence[int], "int | None"], Any]

NONE = "none"
ARM_BACKENDS = ("franky", "serl_http", "fake")
GRIPPER_CHOICES = (*ARM_BACKENDS, NONE)
TELEOP_CHOICES = ("gello", "spacemouse", "fake", NONE)
SECTIONS = ("cameras", "teleop", "daemon")
MAX_ARMS = 2
FRANKA_NUM_JOINTS = 7
FIRST_DAEMON_PORT = 50051
DEFAULT_CAMERA_NAME = "wrist"
EXTRA_CAMERA_NAME = "front"
# Panda datasheet joint limits (rad); their midpoints are the GELLO wrap centres when the yaml sets none.
PANDA_JOINT_LIMITS = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
# Offered per arm: [xmin, ymin, zmin, xmax, ymax, zmax] in m, base frame, a table-top workspace
# in front of a Panda.
SUGGESTED_SAFETY_BOX = (0.3, -0.3, 0.05, 0.7, 0.3, 0.6)
DEFAULT_MAX_DELTA_XYZ_M = 0.05
DEFAULT_MAX_DELTA_JOINT_RAD = 0.04
SAFETY_BOX_LENGTH = 6
GELLO_OPTION_ORDER = (
    "port",
    "baudrate",
    "joint_ids",
    "joint_offsets_rad",
    "joint_signs",
    "wrap_centers_rad",
    "gripper_id",
    "gripper_open_rad",
    "gripper_closed_rad",
    "gripper_binarize_threshold",
    "intervention",
    "event_device",
    "event_keys",
    "poll_interval_s",
)
_GELLO_OPTION_DEFAULTS: dict[str, Any] = {
    "baudrate": 57600,
    "gripper_id": None,
    "gripper_open_rad": None,
    "gripper_closed_rad": None,
    "gripper_binarize_threshold": None,
    "intervention": "always",
    "event_device": None,
    "poll_interval_s": 0.0,
}
# Written for a GELLO gripper channel the wizard sets up anew (an existing file keeps its value, and
# an absent key stays at the backend default, None): the Franka gripper runs in binary mode, so a
# half-pressed trigger must command open or closed rather than a width the gripper rounds anyway.
NEW_GELLO_GRIPPER_BINARIZE_THRESHOLD = 0.5
# (open, closed) written for an uncalibrated GELLO gripper; they only need to differ.
UNCALIBRATED_GRIPPER_RANGE_RAD = (0.0, 1.0)
_NAME_PATTERN = re.compile(r"[A-Za-z][A-Za-z0-9_-]*")
_DECIMALS = 4
_YES = ("y", "yes", "true", "1")
_NO = ("n", "no", "false", "0")


class WizardError(ValueError):
    """A flag value is invalid, or a non-interactive run cannot answer a question."""


# -- drafts ------------------------------------------------------------------------------


@dataclass
class ArmDraft:
    """One robot of the station: its key under ``robots:``, its config and its daemon endpoint.
    ``gello_calibrated`` is False when the GELLO options hold placeholders (rendered with a TODO)."""

    key: str
    config: RobotConfig
    bind: str = DEFAULT_DAEMON_BIND
    token: str | None = None
    gello_calibrated: bool = True


@dataclass
class StationDraft:
    arms: list[ArmDraft]

    @property
    def multi_robot(self) -> bool:
        return len(self.arms) > 1

    @classmethod
    def from_station(cls, station: Station) -> "StationDraft":
        """The draft of an existing station; gello options are completed with their defaults."""
        arms = []
        for key, robot in station.robots.items():
            config = robot.config
            if config.TYPE != FrankaConfig.TYPE:
                raise WizardError(
                    f"robot '{key}' is a {config.TYPE}; the wizard writes Franka stations, edit this one by hand"
                )
            calibrated = True
            if config.teleop is not None and config.teleop.backend == "gello":
                options = full_gello_options(config.teleop.options)
                config = dataclasses.replace(config, teleop=dataclasses.replace(config.teleop, options=options))
                calibrated = not gello_options_look_uncalibrated(options)
            arms.append(ArmDraft(key, config, robot.bind, robot.token, calibrated))
        return cls(arms)


def full_gello_options(options: Mapping[str, Any]) -> dict[str, Any]:
    """Every gello option in a fixed order, absent ones at the backend default (``event_keys``
    merged over the defaults, as the backend does)."""
    unknown = sorted(set(options) - set(GELLO_OPTION_ORDER))
    if unknown:
        raise WizardError(f"unknown gello options {unknown}; known: {list(GELLO_OPTION_ORDER)}")
    joint_ids = options.get("joint_ids")
    defaults = {
        **_GELLO_OPTION_DEFAULTS,
        "port": None,
        "joint_ids": None,
        "joint_offsets_rad": None,
        "joint_signs": None,
        "wrap_centers_rad": [0.0] * len(joint_ids or []),
    }
    full = {key: options[key] if key in options else defaults.get(key) for key in GELLO_OPTION_ORDER}
    full["event_keys"] = {**DEFAULT_EVENT_KEYS, **dict(options.get("event_keys") or {})}
    return full


def gello_options_look_uncalibrated(options: Mapping[str, Any]) -> bool:
    """Zero offsets and all +1 signs: what the wizard writes when calibration is skipped."""
    offsets = options.get("joint_offsets_rad") or []
    signs = options.get("joint_signs") or []
    return all(float(offset) == 0.0 for offset in offsets) and all(int(sign) == 1 for sign in signs)


def _rounded(value: float) -> float:
    rounded = round(float(value), _DECIMALS)
    return 0.0 if rounded == 0 else rounded


def gello_wrap_centers(config: RobotConfig) -> list[float]:
    limits = config.joint_limits if config.joint_limits is not None else PANDA_JOINT_LIMITS
    return [_rounded(center) for center in wrap_centers_from_joint_limits(limits)]


# -- answers and prompting --------------------------------------------------------------


@dataclass
class Answers:
    """Every flag value of the wizard (None = ask). Per-arm flags hold one value per arm, or one
    value for all arms. ``previous`` is the ``--from`` station, whose values are the defaults."""

    arms: int | None = None
    arm_names: list[str] | None = None
    robot_ips: list[str] | None = None
    arm_backends: list[str] | None = None
    grippers: list[str] | None = None
    home_joints: list[str] | None = None
    joint_limits: list[str] | None = None
    safety_boxes: list[str] | None = None
    controls: list[str] | None = None
    max_delta_xyz_m: list[str] | None = None
    max_delta_joint_rad: list[str] | None = None
    cameras: list[str] | None = None
    teleops: list[str] | None = None
    gello_ports: list[str] | None = None
    gello_baudrates: list[str] | None = None
    gello_joint_ids: list[str] | None = None
    gello_gripper_ids: list[str] | None = None
    interventions: list[str] | None = None
    event_devices: list[str] | None = None
    daemon_ports: list[str] | None = None
    calibrate: bool | None = None
    allow_unprobed: bool = False
    non_interactive: bool = False
    mock: bool = False
    only: str | None = None
    previous: StationDraft | None = None

    def for_arm(self, attribute: str, flag: str, index: int, count: int) -> str | None:
        values = getattr(self, attribute)
        if values is None:
            return None
        if len(values) == 1:
            return values[0]
        if len(values) != count:
            raise WizardError(f"{flag} needs one value per arm ({count}) or one for all, got {len(values)}")
        return values[index]


class Prompter:
    """Asks one question: a flag value wins; otherwise the operator answers, with the ``--from``
    value (``prior``) or the wizard's suggestion as the default. ``--non-interactive`` takes the
    ``prior`` value and fails when there is none."""

    def __init__(self, ask: Ask, echo: Echo, *, non_interactive: bool = False) -> None:
        self.ask = ask
        self.echo = echo
        self.non_interactive = non_interactive

    def answer(
        self,
        flag: str,
        given: Any,
        question: str,
        parse: Callable[[str], Any],
        *,
        prior: str | None = None,
        suggested: str | None = None,
    ) -> Any:
        if given is not None:
            return self._parse_flag(flag, given, parse)
        if self.non_interactive:
            if prior is None:
                raise WizardError(f"--non-interactive needs {flag} ({question})")
            return self._parse_prior(flag, prior, parse)
        default = prior if prior is not None else suggested
        while True:
            text = self.ask(f"{question}" + (f" [{default}]" if default is not None else "") + ": ").strip()
            if not text:
                if default is None:
                    self.echo("  an answer is needed")
                    continue
                text = default
            try:
                return parse(text)
            except ValueError as error:
                self.echo(f"  {error}")

    def unasked(
        self, flag: str, given: Any, parse: Callable[[str], Any], *, prior: str | None, default: str
    ) -> Any:
        """A value written without a question: the flag value, else the ``--from`` value
        (``prior``), else the wizard's ``default``."""
        if given is not None:
            return self._parse_flag(flag, given, parse)
        if prior is not None:
            return self._parse_prior(flag, prior, parse)
        return parse(default)

    @staticmethod
    def _parse_flag(flag: str, given: Any, parse: Callable[[str], Any]) -> Any:
        try:
            return parse(str(given).strip())
        except ValueError as error:
            raise WizardError(f"{flag} {given!r}: {error}") from None

    @staticmethod
    def _parse_prior(flag: str, prior: str, parse: Callable[[str], Any]) -> Any:
        try:
            return parse(prior)
        except ValueError as error:
            raise WizardError(f"{flag} from --from {prior!r}: {error}") from None


def _parse_choice(choices: Sequence[str]) -> Callable[[str], str]:
    def parse(text: str) -> str:
        value = text.strip().lower()
        if value not in choices:
            raise ValueError(f"choose one of {' | '.join(choices)}")
        return value

    return parse


def _parse_yes_no(text: str) -> bool:
    value = text.strip().lower()
    if value in _YES:
        return True
    if value in _NO:
        return False
    raise ValueError("answer yes or no")


def _split(text: str) -> list[str]:
    return [item for item in re.split(r"[,\s]+", text.strip()) if item]


def _parse_int_in(low: int, high: int) -> Callable[[str], int]:
    def parse(text: str) -> int:
        try:
            value = int(text)
        except ValueError:
            raise ValueError(f"{text!r} is not an integer") from None
        if not low <= value <= high:
            raise ValueError(f"{value} is outside [{low}, {high}]")
        return value

    return parse


def _floats(text: str) -> list[float]:
    """Numbers separated by commas or spaces; brackets are ignored, so a pasted YAML list works."""
    return [float(value) for value in _split(text.replace("[", " ").replace("]", " "))]


def _parse_positive(unit: str) -> Callable[[str], float]:
    def parse(text: str) -> float:
        try:
            value = float(text)
        except ValueError:
            raise ValueError(f"{text!r} is not a number") from None
        if not (math.isfinite(value) and value > 0):
            raise ValueError(f"give a positive number of {unit}, got {value}")
        return value

    return parse


def _parse_safety_box(text: str) -> list[float]:
    expected = f"give {SAFETY_BOX_LENGTH} numbers in m (xmin ymin zmin xmax ymax zmax)"
    try:
        box = _floats(text)
    except ValueError:
        raise ValueError(f"{expected}, e.g. {_float_list_text(SUGGESTED_SAFETY_BOX)}") from None
    if len(box) != SAFETY_BOX_LENGTH:
        raise ValueError(f"{expected}, got {len(box)}")
    for axis, low, high in zip("xyz", box[:3], box[3:]):
        if not low < high:
            raise ValueError(f"{axis}min {low} must be below {axis}max {high}")
    return box


def _parse_joint_limits(text: str) -> list[list[float]]:
    count = 2 * FRANKA_NUM_JOINTS
    expected = f"give {count} numbers in rad (lo,hi for each of the {FRANKA_NUM_JOINTS} joints)"
    try:
        values = _floats(text)
    except ValueError:
        raise ValueError(expected) from None
    if len(values) != count:
        raise ValueError(f"{expected}, got {len(values)}")
    limits = [values[start:start + 2] for start in range(0, count, 2)]
    for joint, (low, high) in enumerate(limits, start=1):
        if not low < high:
            raise ValueError(f"joint {joint}: lo {low} must be below hi {high}")
    return limits


def _parse_names(count: int) -> Callable[[str], list[str]]:
    def parse(text: str) -> list[str]:
        names = _split(text)
        if len(names) != count:
            raise ValueError(f"give {count} name(s), got {len(names)}")
        for name in names:
            if not _NAME_PATTERN.fullmatch(name):
                raise ValueError(f"name {name!r} must start with a letter and use letters, digits, _ or -")
        if len(set(names)) != len(names):
            raise ValueError(f"names must differ, got {names}")
        return names

    return parse


def _default_arm_names(count: int) -> list[str]:
    return ["arm"] if count == 1 else ["left", "right"]


# -- the wizard --------------------------------------------------------------------------


@dataclass
class _Taken:
    """Resources already given to an earlier arm of this station."""

    ips: set[str] = field(default_factory=set)
    camera_serials: set[str] = field(default_factory=set)
    camera_names: set[str] = field(default_factory=set)
    gello_ports: set[str] = field(default_factory=set)
    daemon_ports: set[int] = field(default_factory=set)


@dataclass
class _ArmContext:
    index: int
    count: int
    name: str
    prior: ArmDraft | None


def run_wizard(
    probe: StationProbe,
    answers: Answers,
    ask: Ask,
    echo: Echo,
    *,
    calibration_bus_factory: CalibrationBusFactory | None = None,
) -> StationDraft:
    """Ask every question (or take its flag) and return the station draft. ``--mock`` calibrates
    against ``SimulatedGelloBus`` unless ``calibration_bus_factory`` is given."""
    previous = answers.previous
    if answers.only is not None:
        if answers.only not in SECTIONS:
            raise WizardError(f"--only must be one of {list(SECTIONS)}, got {answers.only!r}")
        if previous is None:
            raise WizardError("--only reruns one section of an existing station; pass --from <station.yaml>")
    if answers.non_interactive and answers.calibrate:
        raise WizardError("calibration needs an operator at the leader arm; drop --non-interactive or pass --no-calibrate")
    if calibration_bus_factory is None and answers.mock:
        calibration_bus_factory = _simulated_bus
    prompt = Prompter(ask, echo, non_interactive=answers.non_interactive)

    if answers.only is None:
        count = prompt.answer(
            "--arms", answers.arms, f"number of arms (1 or {MAX_ARMS})", _parse_int_in(1, MAX_ARMS),
            prior=None if previous is None else str(len(previous.arms)), suggested="1",
        )
        prior_names = None
        if previous is not None and len(previous.arms) == count:
            prior_names = " ".join(arm.key for arm in previous.arms)
        names = prompt.answer(
            "--arm-names", None if answers.arm_names is None else " ".join(answers.arm_names),
            "arm name(s)", _parse_names(count), prior=prior_names, suggested=" ".join(_default_arm_names(count)),
        )
    else:
        names = [arm.key for arm in previous.arms]
        count = len(names)

    taken = _Taken()
    arms = []
    for index, name in enumerate(names):
        prior = previous.arms[index] if previous is not None and index < len(previous.arms) else None
        context = _ArmContext(index, count, name, prior)
        echo(f"-- arm '{name}' ({index + 1}/{count}) --")
        arms.append(_arm_draft(prompt, answers, probe, context, taken, calibration_bus_factory))
    return StationDraft(arms)


def _asks(answers: Answers, section: str) -> bool:
    return answers.only is None or answers.only == section


def _arm_draft(
    prompt: Prompter,
    answers: Answers,
    probe: StationProbe,
    context: _ArmContext,
    taken: _Taken,
    calibration_bus_factory: CalibrationBusFactory | None,
) -> ArmDraft:
    prior = context.prior
    if prior is None:
        config: RobotConfig = FrankaConfig(name=context.name, cameras=[], teleop=None)
    elif prior.key == context.name:
        config = prior.config
    else:
        config = dataclasses.replace(prior.config, name=context.name)
    if answers.only is None:
        config = _ask_robot(prompt, answers, probe, context, config, taken)
    control_text = answers.for_arm("controls", "--control", context.index, context.count)
    if control_text is not None:
        import yaml
        from robot_infra.control import ArmControlConfig
        try:
            values = yaml.safe_load(control_text)
            if values is not None and not isinstance(values, dict):
                raise ValueError("expected a JSON/YAML mapping or null")
            profile = None if values is None else ArmControlConfig.from_dict(values).to_dict()
            config = dataclasses.replace(config, control=profile)
        except (ValueError, TypeError, yaml.YAMLError) as exc:
            raise WizardError(f"--control: {exc}") from exc
    taken.ips.add(getattr(config, "robot_ip", ""))
    if _asks(answers, "cameras"):
        config = dataclasses.replace(config, cameras=_ask_cameras(prompt, answers, probe, context, config, taken))
    for camera in config.cameras:
        taken.camera_names.add(camera.name)
        if camera.serial:
            taken.camera_serials.add(camera.serial)
    calibrated = True if prior is None else prior.gello_calibrated
    if _asks(answers, "teleop"):
        config, calibrated = _ask_teleop(prompt, answers, probe, context, config, taken, calibration_bus_factory)
    if config.teleop is not None and config.teleop.backend == "gello":
        taken.gello_ports.add(str(config.teleop.options.get("port")))
    bind = DEFAULT_DAEMON_BIND if prior is None else prior.bind
    if _asks(answers, "daemon"):
        bind = _ask_daemon_bind(prompt, answers, context, taken)
    taken.daemon_ports.add(_bind_port(bind))
    return ArmDraft(
        key=context.name,
        config=config,
        bind=bind,
        token=None if prior is None else prior.token,
        gello_calibrated=calibrated,
    )


# -- robot section -------------------------------------------------------------------------


def _ask_robot(
    prompt: Prompter, answers: Answers, probe: StationProbe, context: _ArmContext, config: RobotConfig, taken: _Taken
) -> RobotConfig:
    index, count, prior = context.index, context.count, context.prior
    reachable = [host.ip for host in probe.reachable_hosts]
    if reachable:
        prompt.echo("franka hosts that answered:")
        for number, ip in enumerate(reachable, start=1):
            prompt.echo(f"  {number}) {ip}" + ("  (used by another arm)" if ip in taken.ips else ""))
    else:
        prompt.echo("no Franka answered on TCP 22/443/1337; type its IP (an unprobed IP needs --allow-unprobed)")
    prior_ip = None if prior is None else config.robot_ip

    def parse_ip(text: str) -> str:
        if text.isdigit():
            number = int(text)
            if not 1 <= number <= len(reachable):
                raise ValueError(f"pick a number in 1..{len(reachable)} or type an IP")
            ip = reachable[number - 1]
        else:
            ip = str(ipaddress.ip_address(text))
            if ip not in reachable and not (answers.allow_unprobed or ip == prior_ip):
                raise ValueError(f"{ip} did not answer the probe; check the cable and Desk, or pass --allow-unprobed")
        if ip in taken.ips:
            raise ValueError(f"{ip} is already used by another arm")
        return ip

    robot_ip = prompt.answer(
        "--robot-ip", answers.for_arm("robot_ips", "--robot-ip", index, count),
        f"robot IP of '{context.name}' (number or IP)", parse_ip,
        prior=prior_ip, suggested=next((ip for ip in reachable if ip not in taken.ips), None),
    )
    arm_backend = prompt.answer(
        "--arm-backend", answers.for_arm("arm_backends", "--arm-backend", index, count),
        f"arm backend ({' | '.join(ARM_BACKENDS)})", _parse_choice(ARM_BACKENDS),
        prior=None if prior is None else config.arm_backend, suggested=ARM_BACKENDS[0],
    )
    gripper_choice = prompt.answer(
        "--gripper", answers.for_arm("grippers", "--gripper", index, count),
        f"gripper backend ({' | '.join(GRIPPER_CHOICES)})", _parse_choice(GRIPPER_CHOICES),
        prior=None if prior is None else (config.gripper.backend if config.gripper else NONE),
        suggested=arm_backend,
    )
    if gripper_choice == NONE:
        gripper = None
    elif config.gripper is not None:
        gripper = dataclasses.replace(config.gripper, backend=gripper_choice)
    else:
        gripper = GripperConfig(backend=gripper_choice, max_width_m=0.08, mode="binary")
    config = dataclasses.replace(config, robot_ip=robot_ip, arm_backend=arm_backend, gripper=gripper)

    # A --from station without limits (null) gets the wizard's defaults.
    joint_limits = prompt.unasked(
        "--joint-limits", answers.for_arm("joint_limits", "--joint-limits", index, count), _parse_joint_limits,
        prior=None if config.joint_limits is None else _float_list_text(_flattened(config.joint_limits)),
        default=_float_list_text(_flattened(PANDA_JOINT_LIMITS)),
    )

    def parse_home(text: str) -> list[float] | None:
        if text.strip().lower() == NONE:
            values = None
        else:
            try:
                values = [float(value) for value in _split(text)]
            except ValueError:
                raise ValueError(f"give {FRANKA_NUM_JOINTS} numbers in rad, e.g. {_float_list_text(FRANKA_HOME_JOINTS)}") from None
            if len(values) == FRANKA_NUM_JOINTS:
                _require_home_inside_limits(values, joint_limits)
        dataclasses.replace(config, home_joints=values, joint_limits=joint_limits)  # RobotConfig's own checks
        return values

    home_joints = prompt.answer(
        "--home-joints", answers.for_arm("home_joints", "--home-joints", index, count),
        f"home joints of '{context.name}' ({FRANKA_NUM_JOINTS} values in rad, or none)", parse_home,
        prior=None if prior is None else _float_list_text(config.home_joints),
        suggested=_float_list_text(FRANKA_HOME_JOINTS),
    )
    config = dataclasses.replace(config, home_joints=home_joints, joint_limits=joint_limits)
    return _ask_safety(prompt, answers, context, config)


def _float_list_text(values: Sequence[float] | None) -> str:
    return NONE if values is None else ", ".join(repr(value) for value in values)


def _flattened(pairs: Sequence[Sequence[float]]) -> list[float]:
    return [float(value) for pair in pairs for value in pair]


def _require_home_inside_limits(home_joints: Sequence[float], joint_limits: Sequence[Sequence[float]]) -> None:
    outside = [
        f"joint {joint} = {value} is outside [{low}, {high}]"
        for joint, (value, (low, high)) in enumerate(zip(home_joints, joint_limits), start=1)
        if not low <= value <= high
    ]
    if outside:
        raise ValueError(
            f"the home pose must lie inside the joint limits: {'; '.join(outside)} "
            "(change the home joints, or the limits with --joint-limits)"
        )


def _ask_safety(prompt: Prompter, answers: Answers, context: _ArmContext, config: RobotConfig) -> RobotConfig:
    """The safety box (asked) and the per-step servo cap (flags, else ``--from``, else the defaults)."""
    index, count = context.index, context.count
    suggested_box = _float_list_text(SUGGESTED_SAFETY_BOX)
    prior_box = None if config.safety_box is None else _float_list_text(config.safety_box)
    if prior_box is None and prompt.non_interactive:
        prior_box = suggested_box  # --non-interactive takes the suggestion instead of failing
    prompt.echo(
        "safety box: the TCP position is kept inside it, in the robot base frame, in metres "
        "(servo targets are clipped, moves outside are refused); it must contain the home pose"
    )
    safety_box = prompt.answer(
        "--safety-box", answers.for_arm("safety_boxes", "--safety-box", index, count),
        f"safety box of '{context.name}' (xmin, ymin, zmin, xmax, ymax, zmax in m)", _parse_safety_box,
        prior=prior_box, suggested=suggested_box,
    )
    prior_max_delta = config.max_delta_per_step or {}
    max_delta_xyz_m = prompt.unasked(
        "--max-delta-xyz-m", answers.for_arm("max_delta_xyz_m", "--max-delta-xyz-m", index, count),
        _parse_positive("m"), prior=_optional_text(prior_max_delta.get("xyz_m")),
        default=repr(DEFAULT_MAX_DELTA_XYZ_M),
    )
    max_delta_joint_rad = prompt.unasked(
        "--max-delta-joint-rad", answers.for_arm("max_delta_joint_rad", "--max-delta-joint-rad", index, count),
        _parse_positive("rad"), prior=_optional_text(prior_max_delta.get("joint_rad")),
        default=repr(DEFAULT_MAX_DELTA_JOINT_RAD),
    )
    prompt.echo(
        f"servo step cap of '{context.name}': {max_delta_xyz_m} m, {max_delta_joint_rad} rad per step "
        "(--max-delta-xyz-m, --max-delta-joint-rad; joint limits: --joint-limits)"
    )
    return dataclasses.replace(
        config,
        safety_box=safety_box,
        max_delta_per_step={"xyz_m": max_delta_xyz_m, "joint_rad": max_delta_joint_rad},
    )


def _optional_text(value: Any) -> str | None:
    return None if value is None else repr(value)


# -- cameras -------------------------------------------------------------------------------


def _camera_names(arm_name: str, arm_count: int, taken_names: set[str]) -> Iterator[str]:
    """``wrist`` (``wrist_<arm>`` with two arms) first, then ``front``, ``front_2``, ...; never a taken name."""
    first = DEFAULT_CAMERA_NAME if arm_count == 1 else f"{DEFAULT_CAMERA_NAME}_{arm_name}"
    candidates = [first, EXTRA_CAMERA_NAME, *(f"{EXTRA_CAMERA_NAME}_{n}" for n in range(2, 100))]
    for candidate in candidates:
        if candidate not in taken_names:
            yield candidate


def _cameras_text(cameras: Sequence[CameraConfig]) -> str:
    return ",".join(f"{camera.serial}={camera.name}" for camera in cameras) or NONE


def _ask_cameras(
    prompt: Prompter, answers: Answers, probe: StationProbe, context: _ArmContext, config: RobotConfig, taken: _Taken
) -> list[CameraConfig]:
    serials = [camera.serial for camera in probe.cameras]
    prior_cameras = list(config.cameras) if context.prior is not None else []
    if serials:
        prompt.echo("realsense cameras:")
        for number, camera in enumerate(probe.cameras, start=1):
            used = "  (used by another arm)" if camera.serial in taken.camera_serials else ""
            line = f" ({camera.product_line})" if camera.product_line else ""
            prompt.echo(f"  {number}) {camera.serial}{line}{used}")
    else:
        prompt.echo("no RealSense camera found (an unprobed serial needs --allow-unprobed)")

    def parse_cameras(text: str) -> list[CameraConfig]:
        if text.strip().lower() == NONE:
            return []
        names_here: set[str] = set()
        default_names = _camera_names(context.name, context.count, taken.camera_names)
        cameras = []
        for item in _split(text):
            reference, has_name, name = item.partition("=")
            if reference.isdigit() and 1 <= int(reference) <= len(serials):
                serial = serials[int(reference) - 1]
            else:
                serial = reference
            known = {camera.serial for camera in prior_cameras}
            if serial and serial not in serials and not (answers.allow_unprobed or serial in known):
                raise ValueError(f"camera {serial} was not found by the probe; pass --allow-unprobed to keep it")
            if serial and (serial in taken.camera_serials or serial in {c.serial for c in cameras}):
                raise ValueError(f"camera {serial} is assigned twice")
            if not has_name:
                if not serial:
                    raise ValueError("an entry without a serial needs a name, e.g. =wrist")
                name = next(candidate for candidate in default_names if candidate not in names_here)
            if not _NAME_PATTERN.fullmatch(name):
                raise ValueError(f"camera name {name!r} must start with a letter and use letters, digits, _ or -")
            if name in names_here or name in taken.camera_names:
                raise ValueError(f"camera name {name!r} is used twice")
            names_here.add(name)
            cameras.append(_camera_config(serial, name, prior_cameras))
        return cameras

    return prompt.answer(
        "--cameras", answers.for_arm("cameras", "--cameras", context.index, context.count),
        f"cameras of '{context.name}': numbers or serials, each optionally =name (e.g. 1=wrist,2=front), or none",
        parse_cameras,
        prior=None if context.prior is None else _cameras_text(prior_cameras),
        suggested=_suggested_cameras(serials, context, taken),
    )


def _camera_config(serial: str, name: str, prior_cameras: Sequence[CameraConfig]) -> CameraConfig:
    """The ``--from`` camera with this serial (or, for an empty serial, this name), renamed; else a
    640x480 RealSense at 30 fps."""
    for camera in prior_cameras:
        if (serial and camera.serial == serial) or (not serial and not camera.serial and camera.name == name):
            return dataclasses.replace(camera, name=name)
    return CameraConfig(name=name, serial=serial)


def _suggested_cameras(serials: Sequence[str], context: _ArmContext, taken: _Taken) -> str:
    """Each arm takes the next free camera as its wrist camera; the last arm also takes the rest."""
    free = [serial for serial in serials if serial not in taken.camera_serials]
    chosen = free if context.index == context.count - 1 else free[:1]
    names = _camera_names(context.name, context.count, taken.camera_names)
    return ",".join(f"{serial}={next(names)}" for serial in chosen) or NONE


# -- teleop --------------------------------------------------------------------------------


def _simulated_bus(port: str, baudrate: int, joint_ids: Sequence[int], gripper_id: int | None) -> SimulatedGelloBus:
    return SimulatedGelloBus(FRANKA_CALIBRATION_POSE[: len(joint_ids)], has_gripper=gripper_id is not None)


def _suggested_servo_ids(bus: SerialBusProbe | None) -> tuple[str, str]:
    """(joint ids, gripper id): the probed ids, the last one offered as the gripper."""
    ids = [] if bus is None else list(bus.servo_ids)
    if len(ids) > FRANKA_NUM_JOINTS:
        return ",".join(map(str, ids[:FRANKA_NUM_JOINTS])), str(ids[FRANKA_NUM_JOINTS])
    if len(ids) == FRANKA_NUM_JOINTS:
        return ",".join(map(str, ids)), NONE
    return ",".join(str(i) for i in range(1, FRANKA_NUM_JOINTS + 1)), str(FRANKA_NUM_JOINTS + 1)


def _parse_joint_ids(text: str) -> list[int]:
    parse_id = _parse_int_in(0, MAX_DYNAMIXEL_ID)
    ids = [parse_id(value) for value in _split(text)]
    if len(ids) != FRANKA_NUM_JOINTS:
        raise ValueError(f"a Franka GELLO has {FRANKA_NUM_JOINTS} joint servos, got {len(ids)} ids")
    if len(set(ids)) != len(ids):
        raise ValueError(f"joint ids must differ, got {ids}")
    return ids


def _ask_teleop(
    prompt: Prompter,
    answers: Answers,
    probe: StationProbe,
    context: _ArmContext,
    config: RobotConfig,
    taken: _Taken,
    calibration_bus_factory: CalibrationBusFactory | None,
) -> tuple[RobotConfig, bool]:
    index, count, prior = context.index, context.count, context.prior
    prior_teleop = config.teleop if prior is not None else None
    free_buses = [bus for bus in probe.gello_buses if bus.port not in taken.gello_ports]
    if any(bus.servo_ids for bus in free_buses):
        suggested = "gello"
    elif probe.spacemice:
        suggested = "spacemouse"
    else:
        suggested = NONE
    choice = prompt.answer(
        "--teleop", answers.for_arm("teleops", "--teleop", index, count),
        f"teleop device of '{context.name}' ({' | '.join(TELEOP_CHOICES)})", _parse_choice(TELEOP_CHOICES),
        prior=None if prior is None else (prior_teleop.backend if prior_teleop else NONE), suggested=suggested,
    )
    if choice == NONE:
        return dataclasses.replace(config, teleop=None), True
    if choice != "gello":
        if prior_teleop is not None and prior_teleop.backend == choice:
            return config, True
        name = "spacemouse" if choice == "spacemouse" else "teleop"
        return dataclasses.replace(config, teleop=TeleopConfig(name=name, backend=choice)), True
    return _ask_gello(prompt, answers, probe, context, config, taken, calibration_bus_factory)


def _ask_gello(
    prompt: Prompter,
    answers: Answers,
    probe: StationProbe,
    context: _ArmContext,
    config: RobotConfig,
    taken: _Taken,
    calibration_bus_factory: CalibrationBusFactory | None,
) -> tuple[RobotConfig, bool]:
    index, count, prior = context.index, context.count, context.prior
    prior_teleop = config.teleop if prior is not None and config.teleop is not None else None
    prior_options = prior_teleop.options if prior_teleop is not None and prior_teleop.backend == "gello" else None
    ports = [bus.port for bus in probe.gello_buses]
    if ports:
        prompt.echo("gello / dynamixel buses:")
        for number, bus in enumerate(probe.gello_buses, start=1):
            found = f"{bus.baudrate} baud, ids {bus.servo_ids}" if bus.servo_ids else "no servo answered"
            used = "  (used by another arm)" if bus.port in taken.gello_ports else ""
            prompt.echo(f"  {number}) {bus.port}: {found}{used}")
    else:
        prompt.echo("no U2D2 / OpenRB-150 found under /dev/serial/by-id (an unprobed port needs --allow-unprobed)")
    prior_port = None if prior_options is None else str(prior_options.get("port"))

    def parse_port(text: str) -> str:
        if text.isdigit() and 1 <= int(text) <= len(ports):
            port = ports[int(text) - 1]
        else:
            port = text
            if port not in ports and not (answers.allow_unprobed or port == prior_port):
                raise ValueError(f"{port} was not found by the probe; pass --allow-unprobed to use it anyway")
        if port in taken.gello_ports:
            raise ValueError(f"{port} is already used by another arm")
        return port

    free = [bus for bus in probe.gello_buses if bus.port not in taken.gello_ports]
    suggested_bus = next((bus for bus in free if bus.servo_ids), free[0] if free else None)
    port = prompt.answer(
        "--gello-port", answers.for_arm("gello_ports", "--gello-port", index, count),
        f"GELLO serial port of '{context.name}' (number or path)", parse_port,
        prior=prior_port, suggested=None if suggested_bus is None else suggested_bus.port,
    )
    bus_probe = next((bus for bus in probe.gello_buses if bus.port == port), None)
    baudrate = prompt.answer(
        "--gello-baudrate", answers.for_arm("gello_baudrates", "--gello-baudrate", index, count),
        "GELLO baud rate", _parse_int_in(1, 10_000_000),
        prior=None if prior_options is None else str(prior_options.get("baudrate")),
        suggested=str(bus_probe.baudrate if bus_probe is not None and bus_probe.baudrate else 57600),
    )
    suggested_joints, suggested_gripper = _suggested_servo_ids(bus_probe)
    joint_ids = prompt.answer(
        "--gello-joint-ids", answers.for_arm("gello_joint_ids", "--gello-joint-ids", index, count),
        f"GELLO joint servo ids ({FRANKA_NUM_JOINTS}, in the Panda's joint order)", _parse_joint_ids,
        prior=None if prior_options is None else ",".join(str(i) for i in prior_options.get("joint_ids") or []),
        suggested=suggested_joints,
    )

    def parse_gripper_id(text: str) -> int | None:
        if text.strip().lower() == NONE:
            return None
        gripper_id = _parse_int_in(0, MAX_DYNAMIXEL_ID)(text)
        if gripper_id in joint_ids:
            raise ValueError(f"gripper id {gripper_id} is also a joint id")
        return gripper_id

    prior_gripper = None if prior_options is None else prior_options.get("gripper_id")
    gripper_id = prompt.answer(
        "--gello-gripper-id", answers.for_arm("gello_gripper_ids", "--gello-gripper-id", index, count),
        "GELLO gripper trigger servo id (or none)", parse_gripper_id,
        prior=None if prior_options is None else (NONE if prior_gripper is None else str(prior_gripper)),
        suggested=suggested_gripper,
    )
    intervention = prompt.answer(
        "--intervention", answers.for_arm("interventions", "--intervention", index, count),
        f"intervention ({' | '.join(INTERVENTION_MODES)}: toggle needs a pedal or keyboard)",
        _parse_choice(INTERVENTION_MODES),
        prior=None if prior_options is None else str(prior_options.get("intervention")), suggested="always",
    )

    def parse_event_device(text: str) -> str | None:
        device = None if text.strip().lower() == NONE else text.strip()
        if device is None and intervention == "toggle":
            raise ValueError("intervention toggle needs an event device, e.g. /dev/input/by-id/...-event-kbd")
        return device

    prior_event = None if prior_options is None else prior_options.get("event_device")
    event_device = prompt.answer(
        "--event-device", answers.for_arm("event_devices", "--event-device", index, count),
        "event device for toggle/episode keys (/dev/input/by-id/... or none)", parse_event_device,
        prior=None if prior_options is None else (NONE if prior_event is None else str(prior_event)),
        suggested=NONE,
    )

    keeps_calibration = prior_options is not None and (
        prior_options.get("port") == port
        and list(prior_options.get("joint_ids") or []) == joint_ids
        and prior_options.get("gripper_id") == gripper_id
    )
    # Settings that are not calibration results (event keys, poll interval, ...) carry over.
    options = dict(prior_options or {})
    if keeps_calibration:
        calibrated = prior.gello_calibrated
    else:
        options.update(_uncalibrated_gello_options(joint_ids, gripper_id, gello_wrap_centers(config)))
        calibrated = False
    options.update(
        port=port, baudrate=baudrate, joint_ids=joint_ids, gripper_id=gripper_id,
        intervention=intervention, event_device=event_device,
    )
    if gripper_id is None:
        options["gripper_binarize_threshold"] = None  # the backend rejects it without a gripper channel
    elif prior_gripper is None:
        options["gripper_binarize_threshold"] = NEW_GELLO_GRIPPER_BINARIZE_THRESHOLD
    if _wants_calibration(prompt, answers, port, calibrated):
        result = _calibrate(prompt, options, config, calibration_bus_factory)
        if result is not None:
            options.update(result)
            calibrated = True
    if not calibrated:
        prompt.echo(f"GELLO on {port} is not calibrated; the yaml gets placeholders and a TODO comment")
    teleop = TeleopConfig(
        name="gello" if prior_teleop is None or prior_options is None else prior_teleop.name,
        backend="gello",
        hold_s=0.0 if prior_teleop is None or prior_options is None else prior_teleop.hold_s,
        options=full_gello_options(options),
    )
    return dataclasses.replace(config, teleop=teleop), calibrated


def _uncalibrated_gello_options(joint_ids: Sequence[int], gripper_id: int | None, centers: list[float]) -> dict:
    open_rad, closed_rad = UNCALIBRATED_GRIPPER_RANGE_RAD
    return {
        "joint_offsets_rad": [0.0] * len(joint_ids),
        "joint_signs": [1] * len(joint_ids),
        "wrap_centers_rad": list(centers),
        "gripper_open_rad": None if gripper_id is None else open_rad,
        "gripper_closed_rad": None if gripper_id is None else closed_rad,
    }


def _wants_calibration(prompt: Prompter, answers: Answers, port: str, calibrated: bool) -> bool:
    if answers.calibrate is not None:
        return answers.calibrate
    if answers.non_interactive:
        return False
    return prompt.answer(
        "--calibrate", None, f"calibrate the GELLO on {port} now (yes | no)", _parse_yes_no,
        suggested="no" if calibrated else "yes",
    )


def _calibrate(
    prompt: Prompter, options: dict, config: RobotConfig, calibration_bus_factory: CalibrationBusFactory | None
) -> dict | None:
    """The calibrated options, or None when the operator gives up after a failure."""
    while True:
        bus = None
        if calibration_bus_factory is not None:
            bus = calibration_bus_factory(
                options["port"], options["baudrate"], options["joint_ids"], options["gripper_id"]
            )
        try:
            return calibrate_gello(
                port=options["port"],
                joint_ids=options["joint_ids"],
                gripper_id=options["gripper_id"],
                baudrate=options["baudrate"],
                calibration_pose=FRANKA_CALIBRATION_POSE,
                joint_signs=options["joint_signs"],
                joint_limits=config.joint_limits if config.joint_limits is not None else PANDA_JOINT_LIMITS,
                ask=prompt.ask,
                echo=prompt.echo,
                bus=bus,
            )
        except (DeviceError, ValueError) as error:
            prompt.echo(f"calibration failed: {getattr(error, 'message', None) or error}")
            if getattr(error, "hint", ""):
                prompt.echo(f"  hint: {error.hint}")
            retry = prompt.ask("retry the calibration? [Y/n]: ").strip().lower()
            if retry in _NO:
                return None


# -- daemon ----------------------------------------------------------------------------------


def _bind_parts(bind: str) -> tuple[str, int]:
    host, _, port = bind.rpartition(":")
    return host, int(port)


def _bind_port(bind: str) -> int:
    return _bind_parts(bind)[1]


def _ask_daemon_bind(prompt: Prompter, answers: Answers, context: _ArmContext, taken: _Taken) -> str:
    prior = context.prior
    host = _bind_parts(DEFAULT_DAEMON_BIND if prior is None else prior.bind)[0]

    def parse_port(text: str) -> int:
        port = _parse_int_in(1, 65535)(text)
        if port in taken.daemon_ports:
            raise ValueError(f"port {port} is already used by another arm's daemon")
        return port

    suggested = next(port for port in range(FIRST_DAEMON_PORT, 65536) if port not in taken.daemon_ports)
    port = prompt.answer(
        "--daemon-port", answers.for_arm("daemon_ports", "--daemon-port", context.index, context.count),
        f"daemon port of '{context.name}'", parse_port,
        prior=None if prior is None else str(_bind_port(prior.bind)), suggested=str(suggested),
    )
    return f"{host}:{port}"
