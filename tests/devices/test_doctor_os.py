"""Doctor OS checks for serial/evdev devices: groups, serial ACL, brltty and the USB latency
timer, against a fake ``/sys`` tree and monkeypatched group helpers. Nothing runs sudo."""
from __future__ import annotations

import os
import subprocess

import pytest

from robot_infra.devices import TeleopConfig
from robot_infra.devices.robots.franka import FrankaConfig
from robot_infra.doctor import checks, cli
from robot_infra.doctor import probes as os_probes
from robot_infra.doctor.checks import CheckResult, exit_code


@pytest.fixture
def groups(monkeypatch):
    """Session groups and /etc/group members, editable per test."""
    state = {"session": {"users"}, "listed": {}}
    monkeypatch.setattr(os_probes, "session_group_names", lambda: set(state["session"]))
    monkeypatch.setattr(os_probes, "current_user", lambda: "operator")
    monkeypatch.setattr(os_probes, "listed_group_members", lambda group: set(state["listed"].get(group, ())))
    return state


@pytest.fixture
def sysfs(tmp_path, monkeypatch):
    """A fake /sys/bus/usb-serial/devices plus a /dev with ttyUSB0 and its by-id link."""
    usb_serial = tmp_path / "sys" / "usb-serial"
    usb_serial.mkdir(parents=True)
    monkeypatch.setattr(os_probes, "USB_SERIAL_SYSFS", usb_serial)
    usb_devices = tmp_path / "sys" / "usb"
    usb_devices.mkdir()
    monkeypatch.setattr(os_probes, "USB_DEVICES_SYSFS", usb_devices)
    dev = tmp_path / "dev"
    (dev / "by-id").mkdir(parents=True)
    (dev / "ttyUSB0").touch()
    drivers = tmp_path / "sys" / "drivers"
    drivers.mkdir()
    return {"usb_serial": usb_serial, "usb_devices": usb_devices, "dev": dev, "drivers": drivers}


def add_adapter(sysfs, by_id_name: str, driver: str, latency: str | None) -> str:
    tty_dir = sysfs["usb_serial"] / "ttyUSB0"
    tty_dir.mkdir()
    (sysfs["drivers"] / driver).mkdir()
    os.symlink(sysfs["drivers"] / driver, tty_dir / "driver")
    if latency is not None:
        (tty_dir / "latency_timer").write_text(f"{latency}\n")
    port = sysfs["dev"] / "by-id" / by_id_name
    os.symlink(sysfs["dev"] / "ttyUSB0", port)
    return str(port)


def gello_config(**options) -> FrankaConfig:
    return FrankaConfig(arm_backend="fake", gripper=None, teleop=TeleopConfig(backend="gello", options=options))


def os_check_names(config) -> list[str]:
    return [name for name, _ in checks._os_checks(config)]


# -- which checks run ----------------------------------------------------------------------------


def test_serial_device_checks_only_for_a_configured_gello():
    names = os_check_names(gello_config(port="/dev/ttyUSB9", event_device="/dev/input/event3"))
    assert {"os:serial", "os:groups", "os:brltty", "os:latency_timer"} <= set(names)
    assert not {"os:groups", "os:brltty", "os:latency_timer"} & set(
        os_check_names(FrankaConfig(arm_backend="fake", gripper=None, teleop=TeleopConfig(backend="fake")))
    )
    without_port = os_check_names(gello_config())
    assert "os:brltty" not in without_port and "os:latency_timer" not in without_port


# -- groups ------------------------------------------------------------------------------------


def test_groups_pass_when_the_session_has_them(groups, tmp_path):
    groups["session"] = {"users", "dialout", "input"}
    result = checks._group_membership_result({"dialout": str(tmp_path / "tty"), "input": str(tmp_path / "event3")})
    assert result.ok and not result.warning and result.message == "user in group(s) dialout, input"


def test_groups_fail_when_a_device_is_not_accessible(groups, tmp_path, monkeypatch):
    port, event = tmp_path / "ttyUSB0", tmp_path / "event3"
    port.touch()
    event.touch()
    monkeypatch.setattr(checks.os, "access", lambda path, mode: False)
    result = checks._group_membership_result({"dialout": str(port), "input": str(event)})
    assert not result.ok and "not in group(s) dialout, input" in result.message
    assert result.hint == "sudo usermod -aG dialout,input $USER, then log out and back in"


def test_groups_warn_when_an_acl_grants_access(groups, tmp_path):
    port = tmp_path / "ttyUSB0"
    port.touch()
    result = checks._group_membership_result({"dialout": str(port)})
    assert result.ok and result.warning and "ACL or udev rule" in result.message
    assert "sudo usermod -aG dialout $USER" in result.hint


def test_groups_added_after_login_ask_for_a_new_login(groups, tmp_path, monkeypatch):
    groups["listed"] = {"dialout": {"operator"}}
    port = tmp_path / "ttyUSB0"
    port.touch()
    monkeypatch.setattr(checks.os, "access", lambda path, mode: False)
    result = checks._group_membership_result({"dialout": str(port)})
    assert not result.ok and "added to dialout after this login" in result.message
    assert result.hint.startswith("log out and back in") and "usermod" not in result.hint


# -- serial ACL ----------------------------------------------------------------------------------


def test_denied_serial_port_suggests_an_acl_and_fix_applies_it(tmp_path, monkeypatch):
    port = tmp_path / "ttyUSB0"
    port.touch()
    access = {"granted": False}
    monkeypatch.setattr(checks.os, "access", lambda path, mode: access["granted"])
    granted: list[str] = []

    def grant(path):
        granted.append(path)
        access["granted"] = True

    monkeypatch.setattr(os_probes, "grant_user_acl", grant)
    check = checks._make_check(2, "os:serial", checks._serial_outcome(str(port)))
    results = [check()]
    assert not results[0].ok and f"setfacl -m u:$USER:rw {port}" in results[0].hint
    assert "dialout" in results[0].hint and results[0].autofix is not None
    checks._apply_fixes(results, {0: check})
    assert granted == [str(port)] and results[0].ok


def test_grant_user_acl_never_uses_sudo_and_reports_refusal(monkeypatch, tmp_path):
    calls: list[list[str]] = []

    def run(argv, **kwargs):
        calls.append(list(argv))
        return subprocess.CompletedProcess(argv, 1, stdout="", stderr="Operation not permitted")

    monkeypatch.setattr(subprocess, "run", run)
    monkeypatch.setattr(os_probes, "current_user", lambda: "operator")
    with pytest.raises(RuntimeError, match="Operation not permitted"):
        os_probes.grant_user_acl(str(tmp_path / "ttyUSB0"))
    assert calls[0][:3] == ["setfacl", "-m", "u:operator:rw"] and "sudo" not in calls[0]


# -- brltty --------------------------------------------------------------------------------------


@pytest.fixture
def brltty(monkeypatch):
    state = {"installed": True, "masked": False}
    monkeypatch.setattr(os_probes, "brltty_installed", lambda: state["installed"])
    monkeypatch.setattr(os_probes, "brltty_udev_masked", lambda: state["masked"])
    return state


def test_brltty_warns_for_an_attached_ch340(brltty, sysfs):
    port = add_adapter(sysfs, "usb-1a86_USB_Serial-if00-port0", "ch341-uart", latency=None)
    result = checks._brltty_result(port)
    assert result.ok and result.warning
    assert result.hint == "sudo systemctl mask --now brltty-udev.service brltty.service, then replug the adapter"


def test_brltty_fails_when_the_ch340_port_vanished(brltty, sysfs):
    (sysfs["usb_devices"] / "1-2").mkdir()
    (sysfs["usb_devices"] / "1-2" / "idVendor").write_text("1a86\n")
    result = checks._brltty_result(str(sysfs["dev"] / "by-id" / "usb-Missing-if00-port0"))
    assert not result.ok and "brltty has likely claimed" in result.message
    assert "systemctl mask --now brltty-udev.service brltty.service" in result.hint


def test_brltty_ok_when_masked_absent_or_not_ch340(brltty, sysfs):
    port = add_adapter(sysfs, "usb-FTDI_USB__-__Serial_Converter-if00-port0", "ftdi_sio", latency="1")
    assert checks._brltty_result(port).ok and not checks._brltty_result(port).warning
    brltty["masked"] = True
    assert checks._brltty_result(port).message.endswith("brltty-udev.service masked")
    brltty["installed"] = False
    assert checks._brltty_result(port).message == "brltty not installed"


def test_brltty_mask_is_read_from_systemd_overrides(tmp_path, monkeypatch):
    etc = tmp_path / "etc"
    etc.mkdir()
    monkeypatch.setattr(os_probes, "SYSTEMD_OVERRIDE_DIRS", (etc,))
    assert not os_probes.brltty_udev_masked()
    os.symlink(os.devnull, etc / "brltty-udev.service")
    assert os_probes.brltty_udev_masked()


# -- latency_timer -------------------------------------------------------------------------------


def test_ftdi_latency_timer_above_one_warns_with_the_udev_rule(sysfs):
    port = add_adapter(sysfs, "usb-FTDI_fake-if00-port0", "ftdi_sio", latency="16")
    result = checks._latency_timer_result(port)
    assert result.ok and result.warning and "ttyUSB0 latency_timer is 16 ms" in result.message
    assert 'DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' in result.hint
    assert str(sysfs["usb_serial"] / "ttyUSB0" / "latency_timer") in result.hint


def test_latency_timer_of_one_passes(sysfs):
    port = add_adapter(sysfs, "usb-FTDI_fake-if00-port0", "ftdi_sio", latency="1")
    result = checks._latency_timer_result(port)
    assert result.ok and not result.warning


def test_ch340_has_no_latency_timer(sysfs):
    port = add_adapter(sysfs, "usb-1a86_USB_Serial-if00-port0", "ch341-uart", latency=None)
    result = checks._latency_timer_result(port)
    assert result.ok and not result.warning and "has no latency_timer" in result.message


def test_latency_timer_skips_missing_and_non_usb_serial_ports(sysfs):
    assert checks._latency_timer_result(str(sysfs["dev"] / "ttyUSB7")).skipped
    acm = sysfs["dev"] / "ttyACM0"
    acm.touch()
    assert checks._latency_timer_result(str(acm)).skipped


# -- WARN in the report --------------------------------------------------------------------------


def test_warnings_print_with_their_hint_and_do_not_fail():
    results = [CheckResult(2, "os:latency_timer", True, "ttyUSB0 latency_timer is 16 ms", "add the udev rule", warning=True)]
    text = cli.format_text(results)
    assert "[2/5] os:latency_timer ... WARN  ttyUSB0 latency_timer is 16 ms" in text
    assert "    hint: add the udev rule" in text
    assert "summary: 1 ok (1 with warnings), 0 failed, 0 skipped" in text
    assert exit_code(results) == 0
    assert cli.format_json(results).count('"warning": true') == 1


def test_spacemouse_permission_hint_installs_packaged_vendor_rules(tmp_path, monkeypatch):
    (tmp_path / "hidraw0").touch()
    monkeypatch.setattr(checks.os, "access", lambda path, mode: False)
    ok, message, hint = checks._hid_outcome(str(tmp_path / "hidraw*"))()
    assert not ok and "no read/write access" in message
    assert "sudo cp robot_infra/setup/99-spacemouse.rules /etc/udev/rules.d/" in hint
    assert 'sudo usermod -aG plugdev "$USER"' in hint
    assert "--reload-rules" in hint and "--subsystem-match=hidraw" in hint
    assert "log in again" in hint and "replug" in hint
    assert "pip install" not in hint
