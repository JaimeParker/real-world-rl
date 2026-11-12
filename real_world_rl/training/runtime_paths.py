"""Durable ingestion paths shared by all real-world training entrypoints."""
from pathlib import Path


def actor_sync_kwargs(args, *, direction=None):
    run_name = (getattr(args, "run_id", None) or getattr(args, "exp_name", None)
                or f"{args.sync_host}_{args.sync_port}")
    actor_id = getattr(args, "actor_id", "actor")
    path = getattr(args, "actor_outbox_path", None)
    if path is None:
        path = str(Path(args.log_dir) / run_name / actor_id / "outbox.sqlite3")
    if direction:
        outbox = Path(path)
        path = str(outbox.with_name(f"{outbox.stem}_{direction}{outbox.suffix}"))
        actor_id = f"{actor_id}_{direction}"
    return dict(outbox_path=path, run_id=getattr(args, "run_id", None), actor_id=actor_id,
                startup_timeout=args.sync_startup_timeout, drain_timeout=args.sync_drain_timeout,
                outbox_max_records=args.outbox_max_records, outbox_max_bytes=args.outbox_max_bytes,
                batch_size=args.sync_batch_size)


def hil_learner_paths(args, default_run_name):
    """Separate both FWBW journals and checkpoints, including explicit run roots."""
    run_name = args.exp_name or default_run_name
    if not args.fwbw:
        return run_name, args.checkpoint_dir or str(Path(args.log_dir) / run_name / "checkpoints")
    direction = args.fwbw_direction
    if direction not in ("forward", "backward"):
        raise ValueError("FWBW direction must be forward or backward")
    if args.exp_name:
        run_name = f"{args.exp_name}_{direction}"
    if args.checkpoint_dir is None:
        return run_name, str(Path(args.log_dir) / run_name / "checkpoints")
    root = Path(args.checkpoint_dir)
    if root.name in ("forward", "backward"):
        if root.name != direction:
            raise ValueError("FWBW checkpoint directory belongs to the other direction")
        return run_name, str(root)
    if (root / "journal.sqlite3").exists() or (root / "bundle_manifest.json").exists():
        raise ValueError(
            "legacy flat FWBW checkpoint directory has ambiguous direction ownership; "
            "separate the complete matching journal (including WAL/SHM), bundle generations "
            "and snapshots into forward/backward directories, then select the matching "
            "direction directory with --checkpoint-dir and its manifest with --load-checkpoint. "
            "Standalone legacy .pt weights/replay remain readable."
        )
    return run_name, str(root / direction)


def cleanup_construction(env, *clients):
    """Release resources when construction fails before a loop takes ownership."""
    import logging
    from real_world_rl.training.evaluation import close_env
    for client in clients:
        if client is None:
            continue
        try:
            client.stop(flush=False)
        except BaseException:
            logging.getLogger(__name__).exception("Sync client cleanup failed")
    close_env(env)


def learner_experiment_kwargs(env, checkpoint_dir=None):
    from real_world_rl.action_contract import experiment_contract_of, experiment_hash, effective_experiment_of
    if checkpoint_dir is not None:
        from real_world_rl.action_contract import save_experiment_metadata
        save_experiment_metadata(env, Path(checkpoint_dir).parent / "experiment.json")
    return dict(experiment_contract=experiment_contract_of(env), experiment_hash=experiment_hash(env),
                effective_experiment=effective_experiment_of(env))


def actor_control_hz(args):
    """The robot env config owns pacing for device-layer experiments."""
    resolve_control_frequency(args)
    if args.env_backend == "real_robot": return args.real_robot.control_hz
    if args.env_backend == "real_robot_bimanual": return args.real_robot_bimanual.left.control_hz
    return args.control_hz


def resolve_control_frequency(args):
    """Resolve legacy global pacing into the experiment's robot config once.

    A nondefault global frequency fills a default robot frequency; a nondefault
    robot frequency wins over the default global value. Conflicting nondefaults
    are rejected before any device connection.
    """
    import dataclasses
    backend = getattr(args, "env_backend", None)
    if backend not in ("real_robot", "real_robot_bimanual"): return
    global_hz = getattr(args, "control_hz", 10.0)
    config = getattr(args, "real_robot" if backend == "real_robot" else "real_robot_bimanual", None)
    if config is None: return
    env_hz = config.control_hz if backend == "real_robot" else config.left.control_hz
    if global_hz != 10.0 and env_hz != 10.0 and global_hz != env_hz:
        raise ValueError("global control_hz conflicts with robot env control_hz")
    if global_hz != 10.0 and env_hz == 10.0:
        if backend == "real_robot": args.real_robot = dataclasses.replace(config, control_hz=global_hz)
        else:
            args.real_robot_bimanual = dataclasses.replace(config,
                left=dataclasses.replace(config.left, control_hz=global_hz),
                right=dataclasses.replace(config.right, control_hz=global_hz))
