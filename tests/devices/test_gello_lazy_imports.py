"""The gello device backend and its calibration CLI import without ``dynamixel_sdk`` or
``evdev``; the SDK is required only at ``connect()``."""
from __future__ import annotations

import importlib
import sys

import pytest

import real_world_rl.teleop.gello as gello_package
from real_world_rl.devices import SdkMissing, Teleop, TeleopConfig
from real_world_rl.teleop.gello import backend as original_gello
from real_world_rl.teleop.gello import calibrate as original_calibrate

GELLO_MODULE = "real_world_rl.teleop.gello.backend"
CALIBRATE_MODULE = "real_world_rl.teleop.gello.calibrate"
OPTIONS = {
    "port": "/dev/ttyUSB0",
    "joint_ids": [1, 2, 3, 4, 5, 6, 7],
    "joint_offsets_rad": [0.0] * 7,
    "joint_signs": [1] * 7,
}


@pytest.fixture
def without_sdks(monkeypatch):
    monkeypatch.setitem(sys.modules, "dynamixel_sdk", None)  # makes `import dynamixel_sdk` fail
    monkeypatch.setitem(sys.modules, "evdev", None)


def test_modules_import_and_register_without_sdks(without_sdks, monkeypatch):
    # A fresh import registers "gello" again, so give it an empty slot; everything is restored after.
    monkeypatch.setattr(Teleop, "_BACKENDS", {k: v for k, v in Teleop.backends().items() if k != "gello"})
    monkeypatch.setattr(gello_package, "backend", original_gello)
    monkeypatch.setattr(gello_package, "calibrate", original_calibrate)
    monkeypatch.delitem(sys.modules, GELLO_MODULE)
    monkeypatch.delitem(sys.modules, CALIBRATE_MODULE)
    module = importlib.import_module(GELLO_MODULE)
    calibrate = importlib.import_module(CALIBRATE_MODULE)
    assert module is not original_gello
    assert Teleop.backend("gello") is module.GelloTeleop
    assert callable(calibrate.main) and callable(calibrate.solve_offsets)


def test_connect_without_dynamixel_sdk_raises_sdk_missing_with_install_hint(without_sdks):
    teleop = Teleop.backend("gello")(TeleopConfig(backend="gello", options=OPTIONS))
    with pytest.raises(SdkMissing) as info:
        teleop.connect()
    assert "dynamixel-sdk>=3.7.31,<3.9" in info.value.hint and "[gello]" in info.value.hint
    assert not teleop.is_connected


def test_calibration_cli_without_dynamixel_sdk_raises_sdk_missing(without_sdks):
    from real_world_rl.teleop.gello.calibrate import main

    def never_asked(prompt):
        raise AssertionError(f"prompted before the SDK check: {prompt}")

    with pytest.raises(SdkMissing):
        main(["--port", "/dev/ttyUSB0", "--joint-ids", "1", "2", "3", "4", "5", "6", "7"], ask=never_asked, echo=print)
