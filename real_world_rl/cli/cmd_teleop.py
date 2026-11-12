"""Teleop command registration: ``read`` and ``inspect-stop`` here, the device operations through
``teleop_dispatch`` (their flags come from the device slices, added when the operation is chosen).

Importing this module imports no device slice: ``rwrl`` registers every command at start-up."""

# Dispatched operations, in help order, with their help and parser options.
OPERATION_PARSERS = {
    "calibrate": dict(help="calibrate the station teleop (SpaceMouse or GELLO)"),
    "diagnose": dict(help="inspect SpaceMouse input without robot motion (SpaceMouse)", allow_abbrev=False),
    "run": dict(help="supervised SpaceMouse recording; labels end episodes, n requests next, "
                     "q releases control (SpaceMouse)", allow_abbrev=False),
    "profiles": dict(help="list, show and select direction profiles (SpaceMouse)"),
    "probe-hid": dict(help="record a SpaceMouse-like HID device's descriptor and raw reports and draft its "
                           "device table entry; no robot motion (SpaceMouse)", allow_abbrev=False),
}
# Operations that run the learning side (torch, rl_garden).
LEARNING_OPERATIONS = frozenset({"run"})


def _populate(name, parser):
    if name in OPERATION_PARSERS:
        from .teleop_dispatch import add_operation_arguments
        add_operation_arguments(parser, name)


class TeleopCommand:
    name = "teleop"
    help = "read, calibrate and select teleop input devices"

    def register(self, parser):
        from .teleop_dispatch import LazyOperations
        sub = parser.add_subparsers(dest="teleop_command", required=True, action=LazyOperations)
        sub.populate = _populate
        read = sub.add_parser("read", help="print live teleop samples (GELLO or SpaceMouse) without robot motion",
                              allow_abbrev=False)
        read.add_argument("--duration-s", type=float, help="stop after this many seconds; default: until Ctrl-C")
        read.add_argument("--rate-hz", type=float, default=3., help="printed samples per second (default: 3)")
        read.add_argument("--with-follower", action="store_true",
                          help="also print the follower joint_pos and the per-joint leader-minus-follower "
                               "delta for joint_abs leaders (daemon route only)")
        for name in ("calibrate", "diagnose", "run"):
            sub.add_parser(name, **OPERATION_PARSERS[name])
        inspect_stop = sub.add_parser("inspect-stop", help="evaluate a saved stop trace without hardware access")
        inspect_stop.add_argument("--trace", required=True, help="schema-1 JSON stop trace")
        inspect_stop.add_argument("--thresholds", help="JSON object overriding stop threshold values")
        for name in ("profiles", "probe-hid"):
            sub.add_parser(name, **OPERATION_PARSERS[name])

    def run(self, args, ctx):
        operation = args.teleop_command
        if operation == "read":
            from .teleop_read import read
            return read(args, ctx)
        if operation == "inspect-stop":
            from .stop_inspect import inspect_stop
            return inspect_stop(args, ctx)
        from .teleop_dispatch import dispatch
        if operation in LEARNING_OPERATIONS:
            from .cmd_learning import learning_dependencies
            with learning_dependencies("teleop " + operation):
                return dispatch(args, ctx, operation)
        return dispatch(args, ctx, operation)


TELEOP = TeleopCommand()
