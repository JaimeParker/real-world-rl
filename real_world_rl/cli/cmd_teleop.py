"""Teleop command registration and explicit operation dispatch."""


class TeleopCommand:
    name = "teleop"
    help = "calibrate and select operator direction profiles"

    def register(self, parser):
        sub = parser.add_subparsers(dest="teleop_command", required=True)
        calibrate = sub.add_parser("calibrate")
        calibrate.add_argument("--samples", help="JSON mapping +x,+y,-x,-y to normalized N x 6 samples")
        calibrate.add_argument("--capture-label", choices=("+x", "+y", "-x", "-y", "+z", "-z",
                                                        "+rx", "-rx", "+ry", "-ry", "+rz", "-rz"),
                               help="capture one direction; replace that group in --samples")
        calibrate.add_argument("--append-trial", action="store_true",
                               help="retain existing trials for --capture-label instead of replacing")
        calibrate.add_argument("--profile", required=True)
        calibrate.add_argument("--duration-s", type=float, default=3.)
        calibrate.add_argument("--report-dir")
        calibrate.add_argument("--use", action="store_true", help="save and select this profile in station")
        calibrate.add_argument("--frame", default="table")
        calibrate.add_argument("--output", help="profile YAML; defaults to station teleop_profiles.file")
        calibrate.add_argument("--max-error-deg", type=float, default=25.)
        calibrate.add_argument("--neutral-threshold", type=float, default=.05)
        calibrate.add_argument("--save", action="store_true", help="save validated candidate after confirmation")
        diagnose = sub.add_parser("diagnose", help="inspect SpaceMouse input without robot motion", allow_abbrev=False)
        diagnose.add_argument("--duration-s", type=float, default=5.)
        diagnose.add_argument("--profile")
        diagnose.add_argument("--report-dir")
        run = sub.add_parser("run", help="supervised SpaceMouse recording; labels end episodes, n requests next, q releases control")
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
        profiles = sub.add_parser("profiles")
        ps = profiles.add_subparsers(dest="action", required=True)
        ps.add_parser("list")
        for name in ("show", "use"):
            ps.add_parser(name).add_argument("--profile", required=True)

    def run(self, args, ctx):
        from .teleop_calibrate import calibrate, profiles, diagnose
        from .teleop_session import run_session
        from .stop_inspect import inspect_stop
        operations = {"calibrate": calibrate, "profiles": profiles, "diagnose": diagnose,
                      "run": run_session, "inspect-stop": inspect_stop}
        return operations[args.teleop_command](args, ctx)


TELEOP = TeleopCommand()
