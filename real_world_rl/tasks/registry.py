"""Small versioned registry for built-in real-world tasks."""
from real_world_rl.tasks.config import TaskConfig

_TASKS = {}


def register_task(name, version, factory):
    key = (name, version)
    if key in _TASKS: raise ValueError(f"task already registered: {key}")
    _TASKS[key] = factory


def build_task(config: TaskConfig, arm_names):
    config.__post_init__()
    try: factory = _TASKS[(config.name, config.version)]
    except KeyError: raise ValueError(f"unknown task {(config.name, config.version)}; available: {list(_TASKS)}") from None
    return factory(config, tuple(arm_names))


def _validate(config, arms, count):
    if count is not None and len(arms) != count:
        raise ValueError(f"task {config.name} requires {count} arm(s)")
    if set(config.arm_resets) - set(arms): raise ValueError("task arm_resets contain unknown arms")
    if config.name != "none" and (config.success_source == "tcp_proxy" or config.dense_distance_weight):
        if set(config.targets) != set(arms): raise ValueError("task targets must name every arm")
    from real_world_rl.tasks.runtime import TaskRuntime
    return TaskRuntime(config, arms)


register_task("none", 1, lambda config, arms: _validate(config, arms, None))
register_task("pick_and_place", 1, lambda config, arms: _validate(config, arms, 1))
register_task("parallel_pick_and_place", 1, lambda config, arms: _validate(config, arms, 2))
