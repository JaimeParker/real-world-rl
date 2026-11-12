"""GELLO teleop commands: the guided leader-arm calibration (``teleop calibrate --backend gello``).

``real_world_rl.cli.teleop_dispatch`` finds ``calibrate`` and ``add_calibrate_arguments`` here by
name; unset flags fall back to the station ``teleop.options``."""
from __future__ import annotations

import sys

from real_world_rl.cli.command import Result
from real_world_rl.cli.errors import CliError, USAGE_EXIT_CODE
from real_world_rl.cli.station_edit import station_document, confirm_write, atomic_save

GROUP_DESCRIPTIONS = {
    "calibrate": "leader-arm flags; only with --backend gello; unset values come from the station teleop",
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
