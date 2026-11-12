"""GELLO teleop commands: guided leader-arm calibration and supervised recording.

``real_world_rl.cli.teleop_dispatch`` finds handlers and their argument adders here by name;
unset calibration flags fall back to the station ``teleop.options``."""
from __future__ import annotations

import sys

from real_world_rl.cli.command import Result
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE
from real_world_rl.cli.station_edit import station_document, confirm_write, atomic_save

GROUP_DESCRIPTIONS = {
    "calibrate": "leader-arm flags; only with --backend gello; unset values come from the station teleop",
    "run": "leader_cartesian session flags; only with --backend gello; toggle takes over",
}
SHARED_FLAG_HELP = {"calibrate": {"--use": "write the calibrated options into the station teleop.options"}}
# Value flags default to None: a flag given for another backend is refused, and an unset one
# falls back to the station teleop.options.
_CALIBRATE_FLAGS = (
    ("--port", dict(help="/dev/serial/by-id/... of the U2D2 / OpenRB-150; default: station teleop.options.port")),
    ("--joint-ids", dict(type=int, nargs="+", help="default: station teleop.options.joint_ids")),
    ("--gripper-id", dict(type=int, help="default: station teleop.options.gripper_id (none: no gripper channel)")),
    ("--baudrate", dict(type=int, help="default: station teleop.options.baudrate, else 57600")),
    ("--calibration-pose", dict(type=float, nargs="+",
                                help="follower joint angles (rad) of the pose the leader is held in; default: "
                                     "the Franka pose 0 0 0 -pi/2 0 pi/2 0 for 7 joints")),
    ("--joint-signs", dict(type=int, nargs="+",
                           help="initial signs; default: station teleop.options.joint_signs, else all +1")),
    ("--joint-limits-yaml", dict(help="robot yaml whose joint_limits give the wrap centres; "
                                      "default: the station joint_limits")),
    ("--skip-sign-check", dict(action="store_true", help="take the joint signs as they are")),
)

# GELLO calibrate flag destination -> station teleop.options key supplying its default.
GELLO_STATION_DEFAULTS = {"port": "port", "joint_ids": "joint_ids", "gripper_id": "gripper_id",
                          "baudrate": "baudrate", "joint_signs": "joint_signs"}
GELLO_DEFAULT_BAUDRATE = 57600
# teleop.options keys that only mean something with a gripper channel.
GELLO_GRIPPER_OPTIONS = ("gripper_id", "gripper_open_rad", "gripper_closed_rad", "gripper_binarize_threshold")


def add_calibrate_arguments(parser):
    for flag, options in _CALIBRATE_FLAGS:
        parser.add_argument(flag, **options)


def add_run_arguments(parser):
    from real_world_rl.teleop.core.run_session import add_common_run_arguments
    add_common_run_arguments(parser)
    parser.add_argument("--neutral-stable-s", type=float,
                        help="seconds with takeover released before starting an episode (default: 0.2)")


def _leader_teleop_yaml(raw, document, node):
    """Copy the selected teleop stanza's text; expand aliases that need station context."""
    import textwrap
    import yaml

    def child(tree, name):
        return next((key, value) for key, value in tree.value if key.value == name)

    def end(value):
        if isinstance(value, (yaml.MappingNode, yaml.SequenceNode)) and not value.flow_style and value.value:
            last = value.value[-1]
            return end(last[1] if isinstance(value, yaml.MappingNode) else last)
        return value.end_mark.index

    source = raw.decode("utf-8")
    tree = yaml.compose(source, Loader=yaml.SafeLoader)
    try:
        if "robots" in document:
            name = next(name for name, robot in document["robots"].items() if robot is node)
            tree = child(child(tree, "robots")[1], str(name))[1]
        elif "robot" in document:
            tree = child(tree, "robot")[1]
        key, value = child(tree, "teleop")
    except StopIteration:  # The section can be inherited through a YAML merge key.
        return yaml.safe_dump({"teleop": node["teleop"]}, sort_keys=False)
    # Flow-style stations and cross-section aliases are re-emitted with identical values.
    if not tree.flow_style and value.start_mark.index >= key.start_mark.index:
        start = key.start_mark.index - key.start_mark.column
        stop = source.find("\n", end(value))
        snippet = textwrap.dedent(source[start:stop + 1 if stop >= 0 else len(source)])
        try:
            if yaml.safe_load(snippet) == {"teleop": node["teleop"]}:
                return snippet.rstrip("\n") + "\n"
        except yaml.YAMLError:
            pass
    return yaml.safe_dump({"teleop": node["teleop"]}, sort_keys=False)


def run_session(args, ctx):
    """Give leader_cartesian its own robot YAML, without any follower hardware."""
    import os
    import yaml
    from real_world_rl.teleop.core.run_session import SessionPlan, load_session_station, run_supervised_session
    from real_world_rl.teleop.core.session import validate_neutral_options

    _, raw, document, node, selected = load_session_station(args)
    teleop = selected.config.teleop
    if teleop is None or teleop.backend != "gello":
        raise CliError("teleop run --backend gello requires a station teleop backend of gello",
                       code=USAGE_EXIT_CODE)
    stable_s = .2 if args.neutral_stable_s is None else args.neutral_stable_s
    try:
        validate_neutral_options(.5, stable_s)
    except ValueError as error:
        raise CliError(str(error), code=USAGE_EXIT_CODE) from error
    teleop_yaml = _leader_teleop_yaml(raw, document, node)

    def prepare_report(report, settings, files):
        stub = report / "leader-robot.yaml"
        stub.write_text("type: franka\nname: gello-leader\narm_backend: fake\ngripper: null\ncameras: []\n" + teleop_yaml)
        options = report / "leader-options.yaml"
        options.write_text(yaml.safe_dump({"robot_yaml": str(stub)}, sort_keys=False))
        for path in (stub, options):
            os.chmod(path, 0o600)
        settings["teleop_options_yaml"] = str(options)
        files.update(leader_robot=stub, leader_options=options)

    plan = SessionPlan(
        settings={"teleop_device": "leader_cartesian", "teleop_options_yaml": None,
                  "episode_end_behavior": "hold", "neutral_threshold": .5, "neutral_stable_s": stable_s},
        confirm_detail=(f"GELLO leader_cartesian session on {selected.config.name}; toggle takes over; "
                        "s/f ends episode and holds; n next; q exits"),
        prepare_report=prepare_report)
    return run_supervised_session(args, ctx, plan)


class _UsageErrors:
    """The ``parser`` that ``gello_calibrate.validate_args`` reports through."""

    @staticmethod
    def error(message):
        raise CliError(message, code=USAGE_EXIT_CODE)


def calibrate(args, ctx):
    """Guided GELLO calibration; unset flags fall back to the station teleop options."""
    from types import SimpleNamespace
    from real_world_rl.teleop.gello import calibrate as gello_calibrate
    path, raw, document, node, selected = station_document(args)
    config = selected.config
    station_teleop = config.teleop
    if args.use and station_teleop is not None and station_teleop.backend != "gello":
        raise CliError(f"--use would replace the options of the station's {station_teleop.backend} teleop",
                       code=USAGE_EXIT_CODE)
    station_gello = station_teleop is not None and station_teleop.backend == "gello"
    station_options = dict(station_teleop.options) if station_gello else {}
    values = {name: getattr(args, name) if getattr(args, name) is not None else station_options.get(key)
              for name, key in GELLO_STATION_DEFAULTS.items()}
    missing = [flag for flag, name in (("--port", "port"), ("--joint-ids", "joint_ids")) if values[name] is None]
    if missing:
        raise CliError("GELLO calibration needs the leader port and joint ids (station teleop.options or flags)",
                       code=USAGE_EXIT_CODE, required_fields=missing)
    joint_ids = [int(joint_id) for joint_id in values["joint_ids"]]
    signs = values["joint_signs"]
    if args.joint_signs is None and signs is not None and len(signs) != len(joint_ids):
        signs = None  # The station signs belong to another joint set; the sign check starts from +1.
    checked = gello_calibrate.validate_args(SimpleNamespace(
        joint_ids=joint_ids, calibration_pose=args.calibration_pose,
        joint_signs=None if signs is None else [int(sign) for sign in signs]), _UsageErrors())
    if args.joint_limits_yaml is not None:
        joint_limits = gello_calibrate.load_joint_limits(args.joint_limits_yaml)
        source = args.joint_limits_yaml
    else:
        joint_limits, source = config.joint_limits, "the station joint_limits"
    if joint_limits is not None and len(joint_limits) != len(joint_ids):
        raise CliError(f"{source} has {len(joint_limits)} joint_limits rows for {len(joint_ids)} leader joints",
                       code=USAGE_EXIT_CODE, hint="pass --joint-limits-yaml for the follower arm")
    if not sys.stdin.isatty():
        raise CliError("GELLO calibration prompts the operator at every step and needs a terminal",
                       code=USAGE_EXIT_CODE)

    def echo(text):
        print(text, file=ctx.stderr, flush=True)

    def ask(prompt):
        print(prompt, file=ctx.stderr, end="", flush=True)
        answer = sys.stdin.readline()
        if not answer:
            raise CliError("cancelled", code=USAGE_EXIT_CODE)
        return answer.rstrip("\n")

    options = gello_calibrate.calibrate_gello(
        port=str(values["port"]), joint_ids=joint_ids, gripper_id=values["gripper_id"],
        baudrate=int(values["baudrate"] if values["baudrate"] is not None else GELLO_DEFAULT_BAUDRATE),
        calibration_pose=checked.calibration_pose, joint_signs=checked.joint_signs, joint_limits=joint_limits,
        skip_sign_check=args.skip_sign_check, ask=ask, echo=echo)
    block = gello_calibrate.format_yaml_block(**options)
    result = {"backend": "gello", "options": options, "yaml_block": block}
    if args.use:
        teleop_node = node.get("teleop")
        previous = {} if teleop_node is None else (teleop_node.get("options") or {})
        merged = {**previous, **options}
        if options["gripper_id"] is None:
            # A leader without a gripper channel must not keep the old gripper settings.
            for key in GELLO_GRIPPER_OPTIONS:
                merged.pop(key, None)
        if teleop_node is None:
            node["teleop"] = {"name": "gello", "backend": "gello", "options": merged}
        else:
            teleop_node["options"] = merged
        confirm_write(ctx, path, document, raw)
        result["station_saved"] = atomic_save(path, document, raw)
        echo(f"saved teleop.options to {path}")
    return Result(result, block.rstrip("\n"))
