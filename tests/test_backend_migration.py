"""Removed backend configuration fails before constructing hardware."""
import pytest

from real_world_rl.training.backend_migration import REMOVAL_MESSAGE


@pytest.mark.parametrize("parser", ["training", "training_args", "record", "collect", "deploy"])
@pytest.mark.parametrize("form", ["preset", "flag", "equals", "field", "underscore", "preset_field"])
def test_removed_backend_names_replacement(parser, form, tmp_path):
    from real_world_rl.training.registry import registry
    from real_world_rl.demos.record import parse_record_args
    from real_world_rl.reward.collect_real_robot import parse_collect_args
    from real_world_rl.deploy.checkpoint import main_cli

    if form in ("preset", "preset_field"):
        preset = tmp_path / "old.yaml"
        preset.write_text("env_backend: franka_real\n" if form == "preset" else "franka_real:\n  bridge_url: http://localhost:5000\n")
        argv = ["--config", str(preset)]
    else:
        argv = {
            "flag": ["--env-backend", "franka_real"],
            "equals": ["--env-backend=franka_real"],
            "field": ["--franka-real.bridge-url", "http://localhost:5000"],
            "underscore": ["--franka_real.bridge_url=http://localhost:5000"],
        }[form]
    parse = {
        "training": lambda args: registry.parse_command(["hil_serl", *args]),
        "training_args": lambda args: registry.parse_args(["hil_serl", *args]),
        "record": parse_record_args, "collect": parse_collect_args, "deploy": main_cli,
    }[parser]
    with pytest.raises(ValueError) as error:
        parse(argv)
    assert str(error.value) == REMOVAL_MESSAGE
    assert REMOVAL_MESSAGE == (
        "env_backend franka_real was removed; use env_backend: real_robot with a "
        "serl_http (SERL bridge) or franky arm in the station yaml"
    )
