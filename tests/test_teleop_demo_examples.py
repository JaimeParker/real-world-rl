"""The teleop example scripts build their source through the registry (no ManiSkill needed:
the scripts only import it inside ``main``)."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from _teleop_fakes import FakeEnv, make_hid

from real_world_rl.teleop import make_teleop_source
from real_world_rl.teleop.core.fake import ScriptedTeleopSource
from real_world_rl.teleop.spacemouse.twist_sim import SpaceMouseTwistTeleopSource

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


def _load_example(name: str):
    module_name = f"_example_{name}"
    spec = importlib.util.spec_from_file_location(module_name, EXAMPLES / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    # dataclass processing looks the defining module up in sys.modules
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    return module


@pytest.fixture(params=["run_teleop_demo", "record_teleop_wsrl"])
def example(request):
    module = _load_example(request.param)
    yield module
    sys.modules.pop(module.__name__, None)


def _args(example, **overrides):
    values = dict(device="fake")
    if "output_path" in example.Args.__dataclass_fields__:
        values["output_path"] = "unused.h5"
    values.update(overrides)
    return example.Args(**values)


def test_fake_device_builds_an_idle_source(example):
    args = _args(example)
    source = make_teleop_source(args.device, FakeEnv(), **example.build_teleop_kwargs(args))
    assert isinstance(source, ScriptedTeleopSource)
    assert not source.poll().intervened


def test_spacemouse_kwargs_reach_the_device(example, monkeypatch):
    fake_sdk = make_hid()
    monkeypatch.setitem(sys.modules, "hid", fake_sdk)
    args = _args(example, device="spacemouse_twist", spacemouse_index=2, spacemouse_invert_axes=[1])
    source = make_teleop_source(args.device, FakeEnv(), **example.build_teleop_kwargs(args))
    assert isinstance(source, SpaceMouseTwistTeleopSource)
    source.start()
    try:
        assert fake_sdk.devices[0].kwargs == {"device_index": 2}
    finally:
        source.close()


def test_pico_twist_kwargs_only_carry_the_overrides_that_were_given(example):
    args = _args(example, device="pico_twist", pos_scale=2.0)
    kwargs = example.build_teleop_kwargs(args)
    assert kwargs["pos_scale"] == 2.0 and "rot_scale" not in kwargs and "twist_limit" not in kwargs
    assert kwargs["zmq_url"] == args.zmq_url and kwargs["hand"] == args.hand
    assert example.PHYSICAL_TWIST_DEVICES == ("pico_twist",)


def test_the_sim_examples_default_to_the_raw_twist_pico(example):
    # A real robot takes --teleop-device pico (the anchored pose tracker), never the raw twist.
    assert example.Args.__dataclass_fields__["device"].default == "pico_twist"
