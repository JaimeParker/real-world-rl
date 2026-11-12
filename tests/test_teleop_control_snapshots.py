"""Golden snapshots of the compared ``teleop_control`` block and the frozen ``teleop_input`` for
every built-in teleop device, and the capability declarations that replaced the device-name
tables of the generic modules (env building, intervention wrapper, hold, session).

The expected strings were fixed from the device-name implementation, before the devices declared
capabilities; their serialised form, key order included, must not change.
"""
from __future__ import annotations

import json

import pytest

from real_world_rl.common.provenance import portable_path
from real_world_rl.envs.wrappers import intervention
from real_world_rl.envs.wrappers.intervention import TeleopInterventionWrapper
from real_world_rl.teleop.core import composite
from real_world_rl.teleop.core.composite import CompositeTeleopSource
from real_world_rl.teleop.core.registry import UnknownTeleopSourceError, source_class
from real_world_rl.teleop.core.session import is_session_source, require_session_source
from real_world_rl.training import env_building
from real_world_rl.training.entrypoints.hil_serl import HilSerlArgs

ROTATION = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
TRACKER_DEFAULT = (
    '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.05, "lead_cap_rad": 0.3, '
    '"speed_m_s": 0.05, "speed_rad_s": 0.3}'
)
SERL_DEFAULT = (
    '{"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, '
    '"gripper": "serl_buttons"}'
)
NONE = "null"


@pytest.fixture
def isolated_site(tmp_path, monkeypatch):
    """No local SpaceMouse device table of this machine (./local or the user config directory)
    applies."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    return tmp_path


def _options_yaml(directory, text: str) -> str:
    path = directory / "teleop.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


def _blocks(args) -> tuple[str, str]:
    """``(teleop_control, teleop_input)`` serialised, as an entrypoint resolves them."""
    control = json.dumps(env_building.teleop_control_spec(args))
    env_building.resolve_teleop_input(args)
    return control, json.dumps(args.teleop_input)


# -- every built-in device, default flags and no options file --------------------------------


@pytest.mark.parametrize(
    "fields,control,frozen_input",
    [
        pytest.param({"teleop_device": "spacemouse"}, TRACKER_DEFAULT, NONE, id="spacemouse-tracker"),
        pytest.param({"teleop_device": "spacemouse", "spacemouse_mode": "serl"}, SERL_DEFAULT, NONE,
                     id="spacemouse-serl"),
        pytest.param({"teleop_device": "leader_cartesian"}, TRACKER_DEFAULT, NONE, id="leader_cartesian"),
        pytest.param({"teleop_device": "gello"}, NONE, NONE, id="gello"),
        pytest.param({"teleop_device": "leader_arm"}, NONE, NONE, id="leader_arm"),
        pytest.param({"teleop_device": "robot"}, NONE, NONE, id="robot"),
        # Since the Pico drives the command tracker (pico_anchored_v1, provenance only) it compares
        # the tracker's block; it compared none as the raw-twist source.
        pytest.param({"teleop_device": "pico"}, TRACKER_DEFAULT, NONE, id="pico"),
        pytest.param({"teleop_device": "keyboard"}, NONE, NONE, id="keyboard"),
        pytest.param({"teleop_device": "spacemouse_twist"}, NONE, NONE, id="spacemouse_twist"),
        pytest.param({"teleop_device": "fake"}, NONE, NONE, id="fake"),
        pytest.param({"teleop_device": "bimanual"}, NONE, NONE, id="bimanual"),
        pytest.param({"teleop_device": "fake", "teleop_event_device": "stdin"}, NONE, NONE,
                     id="composite-fake-stdin"),
        pytest.param({"teleop_device": "gello", "teleop_event_device": "jsonl"}, NONE, NONE,
                     id="composite-gello-jsonl"),
        pytest.param({"teleop_device": "spacemouse", "teleop_event_device": "stdin"}, TRACKER_DEFAULT, NONE,
                     id="composite-spacemouse-stdin"),
        pytest.param({"teleop_device": "spacemouse", "spacemouse_mode": "serl", "teleop_event_device": "keyboard"},
                     SERL_DEFAULT, NONE, id="composite-serl-keyboard"),
        pytest.param({"teleop_device": "leader_cartesian", "teleop_event_device": "keyboard"},
                     TRACKER_DEFAULT, NONE, id="composite-leader_cartesian-keyboard"),
        pytest.param({"teleop_device": "pico", "teleop_event_device": "stdin"}, TRACKER_DEFAULT, NONE,
                     id="composite-pico-stdin"),
    ],
)
def test_every_device_with_defaults(isolated_site, fields, control, frozen_input):
    assert _blocks(HilSerlArgs(**fields)) == (control, frozen_input)


@pytest.mark.parametrize(
    "fields,control",
    [
        pytest.param(
            {"teleop_device": "leader_cartesian", "teleop_lead_cap_m": 0.02, "teleop_target_speed_rad_s": 0.5},
            '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.02, "lead_cap_rad": 0.3, '
            '"speed_m_s": 0.05, "speed_rad_s": 0.5}',
            id="leader_cartesian",
        ),
        pytest.param(
            {"teleop_device": "spacemouse", "teleop_lead_cap_rad": 0.2, "teleop_target_speed_m_s": 0.04},
            '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.05, "lead_cap_rad": 0.2, '
            '"speed_m_s": 0.04, "speed_rad_s": 0.3}',
            id="spacemouse",
        ),
        pytest.param(
            {"teleop_device": "pico", "teleop_lead_cap_m": 0.03, "teleop_target_speed_m_s": 0.02},
            '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.03, "lead_cap_rad": 0.3, '
            '"speed_m_s": 0.02, "speed_rad_s": 0.3}',
            id="pico",
        ),
    ],
)
def test_tracker_flags_enter_the_block(isolated_site, fields, control):
    assert _blocks(HilSerlArgs(**fields)) == (control, NONE)


@pytest.mark.parametrize("device", ["leader_cartesian", "gello", "leader_arm", "pico", "fake"])
def test_devices_whose_block_ignores_their_options_never_read_the_options_file(isolated_site, device):
    missing = str(isolated_site / "missing.yaml")
    expected = TRACKER_DEFAULT if device in ("leader_cartesian", "pico") else NONE
    args = HilSerlArgs(teleop_device=device, teleop_options_yaml=missing)
    assert json.dumps(env_building.teleop_control_spec(args)) == expected
    env_building.resolve_teleop_input(args)
    assert args.teleop_input is None


# -- SpaceMouse options: input rotation (provenance) and diagnostics (compared) ----------------


def test_spacemouse_tracker_with_rotation_and_diagnostics(isolated_site):
    path = _options_yaml(isolated_site, f"input_rotation: {ROTATION}\n")
    args = HilSerlArgs(teleop_device="spacemouse", teleop_options_yaml=path,
                       teleop_diagnostic_axes="translation", teleop_diagnostic_gain=0.5)
    control, frozen_input = _blocks(args)
    assert control == (
        '{"version": "cartesian_tracker_v1", "lead_cap_m": 0.05, "lead_cap_rad": 0.3, '
        '"speed_m_s": 0.05, "speed_rad_s": 0.3, "input_diagnostic": {"axes": "translation", "gain": 0.5}}'
    )
    assert frozen_input == (
        f'{{"options_file": {json.dumps(portable_path(path))}, '
        '"source_options": {"input_rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}}'
    )
    restored = HilSerlArgs(teleop_device="spacemouse", teleop_diagnostic_axes="translation",
                           teleop_diagnostic_gain=0.5, teleop_input=json.loads(frozen_input))
    assert _blocks(restored) == (control, frozen_input)


def test_spacemouse_serl_with_inverted_axes_rotation_and_diagnostics(isolated_site):
    path = _options_yaml(isolated_site, f"invert_axes: [0, 5]\ninput_rotation: {ROTATION}\n")
    args = HilSerlArgs(teleop_device="spacemouse", spacemouse_mode="serl", teleop_options_yaml=path,
                       teleop_diagnostic_axes="rotation", teleop_diagnostic_gain=0.25)
    control, frozen_input = _blocks(args)
    assert control == (
        '{"version": "spacemouse_serl_v1", "mode": "serl", "motion_threshold": 0.001, '
        '"gripper": "serl_buttons", "invert_axes": [0, 5], '
        '"input_diagnostic": {"axes": "rotation", "gain": 0.25}}'
    )
    assert frozen_input == (
        f'{{"options_file": {json.dumps(portable_path(path))}, '
        '"source_options": {"invert_axes": [0, 5], "input_rotation": [[0, -1, 0], [1, 0, 0], [0, 0, 1]]}}'
    )


def test_spacemouse_with_event_device_freezes_the_same_input(isolated_site):
    path = _options_yaml(isolated_site, "device_index: 2\n")
    args = HilSerlArgs(teleop_device="spacemouse", teleop_event_device="stdin", teleop_options_yaml=path)
    assert _blocks(args) == (
        TRACKER_DEFAULT,
        f'{{"options_file": {json.dumps(portable_path(path))}, "source_options": {{"device_index": 2}}}}',
    )


# -- the capability declarations that replaced the device-name tables -------------------------

_CAPABILITIES = (
    "USES_LAST_GRIPPER_ACTION",
    "REQUIRES_FULL_ACTION",
    "DRIVES_ONE_ARM",
    "TRACKS_CARTESIAN_TARGET",
    "RECORDS_TELEOP_INPUT",
    "SUPPORTS_INPUT_DIAGNOSTICS",
)
_BUILTIN = ("robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "spacemouse_twist", "pico",
            "pico_twist", "bimanual", "composite", "fake", "keyboard", "stdin", "jsonl")


def _declared(name, options=None):
    cls = source_class(name, options)
    return {capability for capability in _CAPABILITIES if getattr(cls, capability)}


@pytest.mark.parametrize(
    "name,options,declared",
    [
        ("robot", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM"}),
        ("leader_arm", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM"}),
        ("gello", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM"}),
        ("leader_cartesian", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM", "TRACKS_CARTESIAN_TARGET"}),
        ("spacemouse", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM", "TRACKS_CARTESIAN_TARGET",
                              "RECORDS_TELEOP_INPUT", "SUPPORTS_INPUT_DIAGNOSTICS"}),
        ("spacemouse", {"mode": "serl"}, {"REQUIRES_FULL_ACTION", "DRIVES_ONE_ARM", "RECORDS_TELEOP_INPUT",
                                          "SUPPORTS_INPUT_DIAGNOSTICS"}),
        ("spacemouse_twist", None, {"DRIVES_ONE_ARM"}),
        ("pico", None, {"USES_LAST_GRIPPER_ACTION", "DRIVES_ONE_ARM", "TRACKS_CARTESIAN_TARGET"}),
        ("pico_twist", None, {"DRIVES_ONE_ARM"}),
        ("bimanual", None, {"USES_LAST_GRIPPER_ACTION"}),
        ("composite", None, {"USES_LAST_GRIPPER_ACTION"}),
        ("fake", None, set()),
        ("keyboard", None, set()),
        ("stdin", None, set()),
        ("jsonl", None, set()),
    ],
)
def test_builtin_sources_declare_their_capabilities(name, options, declared):
    assert _declared(name, options) == declared


def test_declarations_reproduce_the_former_device_tables():
    def having(capability):
        return {name for name in _BUILTIN if getattr(source_class(name), capability)}

    # env_building._SINGLE_ARM_TELEOP_DEVICES (pico_twist: the former pico, which was in it)
    assert having("DRIVES_ONE_ARM") == {
        "robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "spacemouse_twist", "pico",
        "pico_twist"}
    # env_building._CARTESIAN_TRACKER_TELEOP_DEVICES, plus the Pico since it drives the tracker
    assert having("TRACKS_CARTESIAN_TARGET") == {"leader_cartesian", "spacemouse", "pico"}
    # intervention.GRIPPER_HOLDING_DEVICES, plus the composite that hands the getter on, plus the
    # Pico (its take-over starts from the last sent gripper action)
    assert having("USES_LAST_GRIPPER_ACTION") == {
        "robot", "leader_arm", "gello", "leader_cartesian", "spacemouse", "bimanual", "composite", "pico"}
    # the teleop_device == "spacemouse" checks (teleop_input, input diagnostics)
    assert having("RECORDS_TELEOP_INPUT") == having("SUPPORTS_INPUT_DIAGNOSTICS") == {"spacemouse"}


def test_source_class_resolves_the_spacemouse_mode_and_rejects_unknown_names():
    from real_world_rl.teleop.spacemouse.serl import SpaceMouseSerlTeleopSource
    from real_world_rl.teleop.spacemouse.source import SpaceMouseTeleopSource

    assert source_class("spacemouse") is SpaceMouseTeleopSource
    assert source_class("spacemouse", {"mode": "tracker"}) is SpaceMouseTeleopSource
    assert source_class("spacemouse", {"mode": "serl"}) is SpaceMouseSerlTeleopSource
    with pytest.raises(UnknownTeleopSourceError, match="nope"):
        source_class("nope")


class _Built(Exception):
    """Stops the wrapper right after it chose the source kwargs."""


@pytest.mark.parametrize(
    "device,kwargs,gets_getter",
    [
        ("robot", {}, True),
        ("leader_arm", {}, True),
        ("gello", {}, True),
        ("leader_cartesian", {}, True),
        ("spacemouse", {}, True),
        ("spacemouse", {"mode": "serl"}, False),
        ("bimanual", {}, True),
        ("composite", {"motion": "spacemouse", "events": "stdin"}, True),
        ("pico", {}, True),
        ("pico_twist", {}, False),
        ("spacemouse_twist", {}, False),
        ("fake", {}, False),
        ("keyboard", {}, False),
        ("stdin", {}, False),
        ("jsonl", {}, False),
        ("nope", {}, False),
    ],
)
def test_the_wrapper_passes_the_last_gripper_action_to_sources_that_use_it(monkeypatch, device, kwargs, gets_getter):
    built = {}

    def capture(name, env, **source_kwargs):
        built.update(name=name, kwargs=source_kwargs)
        raise _Built

    monkeypatch.setattr(intervention, "make_teleop_source", capture)
    with pytest.raises(_Built):
        TeleopInterventionWrapper(None, device=device, **kwargs)
    assert built["name"] == device
    assert ("last_gripper_action_getter" in built["kwargs"]) == gets_getter


@pytest.mark.parametrize(
    "motion,motion_kwargs,gets_getter",
    [
        ("spacemouse", {}, True),
        ("spacemouse", {"mode": "serl"}, False),
        ("leader_cartesian", {}, True),
        ("robot", {}, True),
        ("leader_arm", {}, True),
        ("gello", {}, True),
        ("bimanual", {}, True),
        ("pico", {}, True),
        ("pico_twist", {}, False),
        ("spacemouse_twist", {}, False),
        ("fake", {}, False),
    ],
)
def test_a_composite_hands_the_getter_to_a_motion_half_that_uses_it(monkeypatch, motion, motion_kwargs, gets_getter):
    built = []
    monkeypatch.setattr(composite, "make_teleop_source",
                        lambda name, env, **kwargs: built.append((name, kwargs)) or object())

    def getter():
        return 1.0

    CompositeTeleopSource.from_env(None, motion=motion, events="stdin", motion_kwargs=motion_kwargs,
                                   last_gripper_action_getter=getter)
    (motion_name, motion_built), (events_name, events_built) = built
    assert (motion_name, events_name) == (motion, "stdin")
    assert (motion_built.get("last_gripper_action_getter") is getter) == gets_getter
    assert {key: value for key, value in motion_built.items() if key != "last_gripper_action_getter"} == motion_kwargs
    assert "last_gripper_action_getter" not in events_built


def test_a_composite_keeps_an_explicit_motion_getter(monkeypatch):
    built = []
    monkeypatch.setattr(composite, "make_teleop_source",
                        lambda name, env, **kwargs: built.append(kwargs) or object())

    def explicit():
        return -1.0

    CompositeTeleopSource.from_env(None, motion="robot", events="stdin",
                                   motion_kwargs={"last_gripper_action_getter": explicit},
                                   last_gripper_action_getter=lambda: 1.0)
    assert built[0]["last_gripper_action_getter"] is explicit


def test_session_gate_needs_a_session_input_motion_half():
    class _Motion:
        def session_input(self):
            return [0.0] * 6, {}

    class _Plain:
        pass

    gated = CompositeTeleopSource(_Motion(), _Plain())
    assert is_session_source(gated)
    require_session_source(gated)
    for source in (CompositeTeleopSource(_Plain(), _Motion()), _Motion(), None):
        assert not is_session_source(source)
        with pytest.raises(ValueError, match="SessionInputSource"):
            require_session_source(source)
