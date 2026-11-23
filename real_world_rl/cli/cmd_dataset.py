"""Offline demo-dataset tools: ``rwrl dataset list|view|drop|undrop|prune|export`` (no station, no device)."""
from pathlib import Path

from .command import Result
from .errors import CliError, USAGE_EXIT_CODE

# Global options that select or move a robot; a dataset command uses none of them.
NOT_APPLICABLE_OPTIONS = ("station", "robot", "preset", "direct", "mock")
VIEWER_HINT = "pip install -e '.[viewer]'"
LEROBOT_HINT = "pip install -e '.[lerobot]'"


def open_dataset(path):
    from real_world_rl.dataset import Dataset
    try:
        return Dataset(path)
    except (OSError, ValueError) as error:
        raise CliError(f"cannot open dataset {path}: {error}", code=USAGE_EXIT_CODE) from error


def run_reader(operation, *arguments):
    """Reader errors (unknown episode, bad state) are usage errors."""
    try:
        return operation(*arguments)
    except (KeyError, ValueError, OSError) as error:
        message = error.args[0] if isinstance(error, KeyError) and error.args else str(error)
        raise CliError(str(message), code=USAGE_EXIT_CODE) from error


def listing(args, ctx):
    from real_world_rl.dataset.summary import episode_rows, format_episode_table
    dataset = open_dataset(args.dataset)
    rows = episode_rows(dataset)
    kept = [row for row in rows if row["status"] == "kept"]
    payload = {"dataset": str(dataset.root), "episodes": rows,
               "totals": {"episodes": len(rows), "kept": len(kept), "discarded": len(rows) - len(kept),
                          "kept_transitions": sum(row["intervened_steps"] for row in kept)}}
    return Result(payload, format_episode_table(dataset))


def view(args, ctx):
    from real_world_rl.dataset.view import view_dataset
    if args.web and args.save:
        raise CliError("--web and --save are mutually exclusive", code=USAGE_EXIT_CODE)
    mode = "web" if args.web else "save" if args.save else "spawn"
    try:
        import rerun  # noqa: F401
    except ModuleNotFoundError as error:
        raise CliError(f"rwrl dataset view needs rerun-sdk: {error}", code=USAGE_EXIT_CODE,
                       hint=VIEWER_HINT) from error
    open_dataset(args.dataset)
    url = run_reader(lambda: view_dataset(args.dataset, episodes=args.episode, mode=mode,
                                          save_path=args.save, web_port=args.web_port,
                                          grpc_port=args.grpc_port))
    if mode == "save":
        text = f"wrote {args.save}; open it with `rerun {args.save}` and pick an episode in the recording list"
    elif mode == "web":
        text = f"web viewer stopped ({url})"
    else:
        text = "streamed to the Rerun viewer: pick an episode in the viewer's recording list"
    return Result({"dataset": str(args.dataset), "mode": mode, "save": args.save, "url": url}, text)


def change(operation_name, verb):
    def operation(args, ctx):
        dataset = open_dataset(args.dataset)
        for index in args.episodes:
            run_reader(getattr(dataset, operation_name), index, getattr(args, "note", None))
        return Result({"dataset": str(dataset.root), operation_name: list(args.episodes)},
                      f"{verb} episodes " + ", ".join(map(str, args.episodes)))
    return operation


def prune(args, ctx):
    dataset = open_dataset(args.dataset)
    doomed = [info for info in dataset.episodes() if not info.kept]
    listed = "\n".join(f"  episode {info.index:06d}  {info.meta.get('discard_reason') or '-'}  "
                       f"{info.steps} steps" for info in doomed)
    if not doomed:
        return Result({"dataset": str(dataset.root), "removed": []}, "no discarded episodes to prune")
    if not (args.prune_yes or ctx.args.yes):
        raise CliError("prune deletes these discarded episodes permanently:\n" + listed
                       + "\nrerun with --yes to delete them", code=USAGE_EXIT_CODE)
    removed = run_reader(dataset.prune)
    return Result({"dataset": str(dataset.root), "removed": removed},
                  "removed episodes " + ", ".join(map(str, removed)))


def export(args, ctx):
    dataset = open_dataset(args.dataset)
    if args.format == "pkl":
        from .cmd_learning import learning_dependencies
        with learning_dependencies("dataset export"):
            from real_world_rl.dataset.transitions import export_pkl
            path = run_reader(lambda: export_pkl(dataset, args.out, frame_stack=args.frame_stack,
                                                 keep_syncing_steps=args.keep_syncing_steps))
        return Result({"dataset": str(dataset.root), "format": "pkl", "out": str(path)},
                      f"wrote {path}")
    try:
        from real_world_rl.dataset.lerobot_export import export_lerobot
        summary = run_reader(lambda: export_lerobot(dataset, args.out, repo_id=args.repo_id, fps=args.fps,
                                                    vcodec=args.vcodec))
    except ImportError as error:
        raise CliError(f"rwrl dataset export --format lerobot needs lerobot: {error}",
                       code=USAGE_EXIT_CODE, hint=LEROBOT_HINT) from error
    text = (f"wrote LeRobot dataset {summary['out']}: {summary['episodes']} episodes, "
            f"{summary['frames']} frames, {summary['fps']} fps")
    if summary["skipped_depth_keys"]:
        text += "\nskipped depth keys: " + ", ".join(summary["skipped_depth_keys"])
    return Result({"dataset": str(dataset.root), "format": "lerobot", **summary}, text)


class DatasetCommand:
    uses_selection = False  # --preset is not applicable
    name = "dataset"
    help = "list, review, edit and export recorded demo datasets offline (no station or robot)"

    def register(self, parser):
        sub = parser.add_subparsers(dest="dataset_command", required=True)

        def add(name, help, **kwargs):
            command = sub.add_parser(name, help=help, description=help, allow_abbrev=False, **kwargs)
            command.add_argument("dataset", metavar="DIR", type=Path, help="dataset directory")
            return command

        add("list", "episode table with kept/discarded status, steps and totals")
        viewer = add("view", "open episodes in the Rerun viewer (needs the viewer extra)")
        viewer.add_argument("-e", "--episode", type=int, action="append", metavar="N",
                            help="episode index (repeatable); default all")
        viewer.add_argument("--web", action="store_true", help="serve the web viewer instead of a native one")
        viewer.add_argument("--save", metavar="PATH.rrd", help="write an .rrd file instead of opening a viewer")
        viewer.add_argument("--grpc-port", type=int, metavar="PORT", help="viewer gRPC port (default 9876)")
        viewer.add_argument("--web-port", type=int, metavar="PORT", help="web viewer port (default 9090)")
        for name, help in (("drop", "mark kept episodes discarded (reversible)"),
                           ("undrop", "restore dropped episodes to kept")):
            command = add(name, help)
            command.add_argument("episodes", type=int, nargs="+", metavar="N", help="episode indices")
            if name == "drop":
                command.add_argument("--note", metavar="TEXT", help="reason recorded in the episode history")
        pruner = add("prune", "delete every discarded episode directory (needs --yes)")
        pruner.add_argument("--yes", dest="prune_yes", action="store_true", default=False,
                            help="delete without asking; the global --yes works too")
        exporter = add("export", "export kept episodes as a training pkl or a LeRobot v3.0 dataset")
        exporter.add_argument("--format", choices=("pkl", "lerobot"), required=True)
        exporter.add_argument("--out", required=True, metavar="PATH", type=Path,
                              help="pkl file, or output directory for lerobot")
        exporter.add_argument("--frame-stack", type=int, default=1, metavar="N", help="pkl: image frame stack")
        exporter.add_argument("--keep-syncing-steps", action="store_true", help="pkl: keep syncing steps")
        exporter.add_argument("--repo-id", metavar="NAME", help="lerobot: repo id (default: output directory name)")
        exporter.add_argument("--fps", type=int, metavar="N",
                              help="lerobot: frames per second (default: the header control_hz, a whole number)")
        exporter.add_argument("--vcodec", default="h264", choices=("h264", "hevc", "libsvtav1"),
                              help="lerobot: video codec (default h264; AV1 crashes on tiny frames)")

    def run(self, args, ctx):
        for option in NOT_APPLICABLE_OPTIONS:
            if getattr(args, option, None) not in (None, False):
                raise CliError(f"--{option} not applicable to rwrl dataset", code=USAGE_EXIT_CODE)
        operations = {"list": listing, "view": view, "drop": change("drop", "dropped"), "undrop": change("undrop", "restored"),
                      "prune": prune, "export": export}
        return operations[args.dataset_command](args, ctx)


DATASET = DatasetCommand()
