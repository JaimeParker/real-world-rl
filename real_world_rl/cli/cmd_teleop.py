"""Teleop command registration and explicit operation dispatch."""

CALIBRATE_BACKENDS = ("spacemouse", "gello")
_CAPTURE_LABELS = ("+x", "+y", "-x", "-y", "+z", "-z", "+rx", "-rx", "+ry", "-ry", "+rz", "-rz")

# Backend-specific `teleop calibrate` flags. Value flags default to None so a flag given for the
# other backend can be refused; the SpaceMouse branch applies SPACEMOUSE_CALIBRATE_DEFAULTS.
SPACEMOUSE_CALIBRATE_DEFAULTS = {"duration_s": 3., "frame": "table", "max_error_deg": 25., "neutral_threshold": .05}
SPACEMOUSE_CALIBRATE_FLAGS = (
    ("--samples", dict(help="JSON mapping +x,+y,-x,-y to normalized N x 6 samples")),
    ("--capture-label", dict(choices=_CAPTURE_LABELS, help="capture one direction; replace that group in --samples")),
    ("--append-trial", dict(action="store_true",
                            help="retain existing trials for --capture-label instead of replacing")),
    ("--profile", dict(help="profile name; required with --backend spacemouse")),
    ("--duration-s", dict(type=float, help="capture duration")),
    ("--report-dir", dict()),
    ("--frame", dict(help="station task_frame name or base")),
    ("--output", dict(help="profile YAML; defaults to station teleop_profiles.file")),
    ("--max-error-deg", dict(type=float)),
    ("--neutral-threshold", dict(type=float)),
    ("--save", dict(action="store_true", help="save validated candidate after confirmation")),
)
GELLO_CALIBRATE_FLAGS = (
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
CALIBRATE_FLAGS = {"spacemouse": SPACEMOUSE_CALIBRATE_FLAGS, "gello": GELLO_CALIBRATE_FLAGS}
_CALIBRATE_GROUP_DESCRIPTIONS = {
    "spacemouse": "direction profile flags; only with --backend spacemouse",
    "gello": "leader-arm flags; only with --backend gello; unset values come from the station teleop",
}


def _destination(flag):
    return flag[2:].replace("-", "_")


def given_calibrate_flags(args, backend):
    """The ``backend``-specific calibrate flags present on the command line."""
    return [flag for flag, _ in CALIBRATE_FLAGS[backend]
            if getattr(args, _destination(flag)) not in (None, False)]


def _register_calibrate(calibrate):
    calibrate.add_argument("--backend", choices=CALIBRATE_BACKENDS,
                           help="teleop device to calibrate; default: the station teleop backend")
    calibrate.add_argument("--use", action="store_true",
                           help="spacemouse: save and select this profile in the station; "
                                "gello: write the calibrated options into the station teleop.options")
    for backend in CALIBRATE_BACKENDS:
        group = calibrate.add_argument_group(backend, _CALIBRATE_GROUP_DESCRIPTIONS[backend])
        for flag, options in CALIBRATE_FLAGS[backend]:
            options = dict(options)
            default = SPACEMOUSE_CALIBRATE_DEFAULTS.get(_destination(flag)) if backend == "spacemouse" else None
            if default is not None:
                options["help"] = f"{options.get('help', '')} (default: {default})".strip()
            group.add_argument(flag, **options)


class TeleopCommand:
    name = "teleop"
    help = "read, calibrate and select teleop input devices"

    def register(self, parser):
        sub = parser.add_subparsers(dest="teleop_command", required=True)
        read = sub.add_parser("read", help="print live teleop samples (GELLO or SpaceMouse) without robot motion",
                              allow_abbrev=False)
        read.add_argument("--duration-s", type=float, help="stop after this many seconds; default: until Ctrl-C")
        read.add_argument("--rate-hz", type=float, default=3., help="printed samples per second (default: 3)")
        read.add_argument("--with-follower", action="store_true",
                          help="also print the follower joint_pos and the per-joint leader-minus-follower "
                               "delta for joint_abs leaders (daemon route only)")
        _register_calibrate(sub.add_parser("calibrate", help="calibrate the station teleop (SpaceMouse or GELLO)"))
        diagnose = sub.add_parser("diagnose", help="inspect SpaceMouse input without robot motion (SpaceMouse)",
                                  allow_abbrev=False)
        diagnose.add_argument("--duration-s", type=float, default=5.)
        diagnose.add_argument("--profile")
        diagnose.add_argument("--report-dir")
        run = sub.add_parser("run", help="supervised SpaceMouse recording; labels end episodes, n requests next, "
                                         "q releases control (SpaceMouse)")
        run.add_argument("--mode", choices=("tracker", "serl"), default="tracker")
        run.add_argument("--profile")
        run.add_argument("--events-file", help="append-only JSONL file, or - for piped stdin")
        run.add_argument("--record", required=True, help="new output .pkl path")
        run.add_argument("--num-successes", type=int, help="default: 20; 1 with --diagnose")
        run.add_argument("--max-episodes", type=int)
        run.add_argument("--diagnose", choices=("translation", "rotation", "all"))
        run.add_argument("--gain", type=float, default=.2, help="diagnostic input gain; ignored without --diagnose")
        run.add_argument("--duration-s", type=float, default=5., help="diagnostic duration; ignored without --diagnose")
        inspect_stop = sub.add_parser("inspect-stop", help="evaluate a saved stop trace without hardware access")
        inspect_stop.add_argument("--trace", required=True, help="schema-1 JSON stop trace")
        inspect_stop.add_argument("--thresholds", help="JSON object overriding stop threshold values")
        profiles = sub.add_parser("profiles", help="list, show and select direction profiles (SpaceMouse)")
        ps = profiles.add_subparsers(dest="action", required=True)
        ps.add_parser("list")
        for name in ("show", "use"):
            ps.add_parser(name).add_argument("--profile", required=True)

    def run(self, args, ctx):
        from .teleop_calibrate import calibrate, profiles, diagnose
        from .teleop_read import read
        from .teleop_session import run_session
        from .stop_inspect import inspect_stop
        from .cmd_learning import learning_dependencies
        operations = {"read": read, "calibrate": calibrate, "profiles": profiles, "diagnose": diagnose,
                      "inspect-stop": inspect_stop}
        if args.teleop_command == "run":
            # The supervised session records through the learning side (torch, rl_garden).
            with learning_dependencies("teleop run"):
                return run_session(args, ctx)
        return operations[args.teleop_command](args, ctx)


TELEOP = TeleopCommand()
