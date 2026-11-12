"""`rwrl teleop probe-hid` with a fake hidapi: candidates, the probe files and the draft entry."""
from __future__ import annotations

import datetime
import json
from collections import deque

import pytest
import yaml

from real_world_rl.cli import main
from real_world_rl.teleop.spacemouse import hid as spacemouse_hid
from real_world_rl.teleop.spacemouse import probe

TODAY = datetime.date(2026, 11, 12)
SERIAL = "SECRET-SERIAL-42"
# Report 1: X, Y, Z, Rx, Ry, Rz as int16 (bytes 1-12); report 3: two button bits and padding.
WIRELESS_DESCRIPTOR = bytes([
    0x05, 0x01, 0x09, 0x08, 0xA1, 0x01,
    0xA1, 0x00, 0x85, 0x01, 0x16, 0xA2, 0xFE, 0x26, 0x5E, 0x01,
    0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35,
    0x75, 0x10, 0x95, 0x06, 0x81, 0x02, 0xC0,
    0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x02, 0x81, 0x02, 0x95, 0x06, 0x81, 0x03,
    0xC0,
])
# Report 1: X, Y, Z; report 2: Rx, Ry, Rz; report 3: two buttons (the SpaceNavigator split).
SPLIT_DESCRIPTOR = bytes([
    0x05, 0x01, 0x09, 0x08, 0xA1, 0x01,
    0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02,
    0x85, 0x02, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x75, 0x10, 0x95, 0x03, 0x81, 0x02,
    0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02,
    0x95, 0x06, 0x81, 0x03, 0xC0,
])
# The six axes in report 4, which no built-in layout reads.
UNKNOWN_DESCRIPTOR = bytes([
    0x05, 0x01, 0x09, 0x08, 0xA1, 0x01,
    0x85, 0x04, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35,
    0x75, 0x10, 0x95, 0x06, 0x81, 0x02,
    0x85, 0x03, 0x05, 0x09, 0x19, 0x01, 0x29, 0x02, 0x75, 0x01, 0x95, 0x02, 0x81, 0x02,
    0x95, 0x06, 0x81, 0x03, 0xC0,
])
WIRELESS_REPORTS = [[1] + [0x10, 0x00] * 6, [3, 0x01], [3, 0x00], [1] + [0x00, 0x00] * 6]


def info(vid=0x256F, pid=0xC63A, path=b"/dev/hidraw3", product="SpaceMouse Wireless BT", usage=(1, 8), interface=0):
    return {"vendor_id": vid, "product_id": pid, "path": path, "product_string": product,
            "manufacturer_string": "3Dconnexion", "serial_number": SERIAL, "interface_number": interface,
            "usage_page": usage[0], "usage": usage[1]}


def fake_hid(infos, *, descriptor=None, reports=()):
    sdk = type("FakeHid", (), {})()
    sdk.enumerate = lambda: [dict(entry) for entry in infos]
    sdk.opened = []

    class Device:
        def __init__(self):
            self.queue = deque(list(report) for report in reports)
            self.closed = False
            sdk.opened.append(self)

        def open_path(self, path):
            self.path = path

        def set_nonblocking(self, enabled):
            assert enabled == 1

        def read(self, size):
            assert size == probe.READ_SIZE
            return self.queue.popleft() if self.queue else []

        def write(self, data):
            raise AssertionError("the probe must not send output reports")

        def send_feature_report(self, data):
            raise AssertionError("the probe must not send feature reports")

        def close(self):
            self.closed = True

    if descriptor is not None:
        Device.get_report_descriptor = lambda self, max_length=4096: list(descriptor)
    sdk.device = Device
    return sdk


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        self.now += 0.01
        return self.now

    def sleep(self, seconds):
        self.now += seconds


@pytest.fixture
def no_owners(monkeypatch, tmp_path):
    monkeypatch.setattr("real_world_rl.common.device_ownership.process_owners", lambda *args, **kwargs: [])
    monkeypatch.setattr("real_world_rl.devices.claims._DEFAULT_LOCK_DIR", tmp_path / "claims")


def run(sdk, tmp_path, **kwargs):
    clock = Clock()
    options = dict(duration_s=1.0, station_dir=tmp_path, sdk=sdk, sysfs=tmp_path / "sys",
                   claims_dir=tmp_path / "claims", clock=clock, sleep=clock.sleep, today=TODAY, echo=lambda text: None)
    return probe.run_probe(**{**options, **kwargs})


def test_descriptor_summary_reads_axis_bytes_and_button_bits():
    reports = probe.summarize_descriptor(WIRELESS_DESCRIPTOR)
    assert reports[1]["bytes"] == 12
    assert [(axis["name"], axis["byte1"], axis["byte2"]) for axis in reports[1]["axes"]] == [
        ("x", 1, 2), ("y", 3, 4), ("z", 5, 6), ("pitch", 7, 8), ("roll", 9, 10), ("yaw", 11, 12)]
    assert reports[3] == {"bytes": 1, "axes": [], "buttons": [{"byte": 1, "bit": 0}, {"byte": 1, "bit": 1}]}


@pytest.mark.parametrize("descriptor,layout", [(WIRELESS_DESCRIPTOR, "SpaceMouse Wireless"),
                                               (SPLIT_DESCRIPTOR, "SpaceNavigator")])
def test_a_built_in_layout_is_matched_by_report_ids_and_lengths(descriptor, layout):
    assert probe.guess_layout(probe.summarize_descriptor(descriptor), {})[0] == layout


def test_without_a_descriptor_the_captured_report_lengths_pick_the_layout():
    observed = probe.summarize_reports([{"report_id": report[0], "length": len(report)} for report in WIRELESS_REPORTS])
    assert probe.guess_layout(None, observed)[0] == "SpaceMouse Wireless"
    assert probe.guess_layout(None, {})[0] is None


def test_probe_writes_descriptor_reports_and_a_pasteable_candidate(tmp_path, no_owners):
    sdk = fake_hid([info()], descriptor=WIRELESS_DESCRIPTOR, reports=WIRELESS_REPORTS)
    payload = run(sdk, tmp_path)
    directory = tmp_path / "teleop/probes/256f-c63a"
    assert payload["probe_directory"] == str(directory) and payload["layout"] == "SpaceMouse Wireless"
    assert (directory / "descriptor.hex").read_text() == WIRELESS_DESCRIPTOR.hex() + "\n"
    rows = [json.loads(line) for line in (directory / "reports.jsonl").read_text().splitlines()]
    assert [(row["report_id"], row["length"]) for row in rows] == [(1, 13), (3, 2), (3, 2), (1, 13)]
    assert rows[0]["bytes_hex"] == bytes(WIRELESS_REPORTS[0]).hex() and set(rows[0]) == {"t", "report_id", "length", "bytes_hex"}
    document = yaml.safe_load((directory / "candidate.yaml").read_text())
    ((name, entry),) = document["devices"].items()
    assert entry == {"hid_id": "256f:c63a", "layout": "SpaceMouse Wireless", "evidence": {
        "probe": "teleop/probes/256f-c63a/", "checked": "2026-11-12", "note": entry["evidence"]["note"]}}
    table = spacemouse_hid.parse_device_table(directory / "candidate.yaml")
    assert table.specs[(0x256F, 0xC63A)].name == name and table.overrides == {}
    assert sdk.opened[0].closed and sdk.opened[0].path == b"/dev/hidraw3"
    # The serial number stays out of the payload and the files.
    assert SERIAL not in json.dumps(payload)
    assert not any(SERIAL in path.read_text() for path in directory.iterdir())


def test_without_a_fitting_layout_the_candidate_spells_out_the_decoding(tmp_path, no_owners):
    sdk = fake_hid([info(pid=0xC6FF, product="New Puck")], descriptor=UNKNOWN_DESCRIPTOR)
    payload = run(sdk, tmp_path)
    assert payload["layout"] is None
    path = tmp_path / "teleop/probes/256f-c6ff/candidate.yaml"
    document = yaml.safe_load(path.read_text())
    ((name, entry),) = document["devices"].items()
    assert entry["mappings"]["x"] == {"channel": 4, "byte1": 1, "byte2": 2, "scale": "TODO"}
    assert entry["mappings"]["yaw"] == {"channel": 4, "byte1": 11, "byte2": 12, "scale": "TODO"}
    assert entry["button_mapping"] == [{"channel": 3, "byte": 1, "bit": 0}, {"channel": 3, "byte": 1, "bit": 1}]
    # An unfinished entry rejects the table; with the signs filled in it is a valid entry.
    with pytest.raises(ValueError):
        spacemouse_hid.parse_device_table(path)
    for axis in entry["mappings"].values():
        axis["scale"] = 1
    path.write_text(yaml.safe_dump(document))
    assert spacemouse_hid.parse_device_table(path).specs[(0x256F, 0xC6FF)].name == name


def test_the_descriptor_falls_back_to_sysfs(tmp_path, no_owners):
    node = tmp_path / "sys/class/hidraw/hidraw3/device"
    node.mkdir(parents=True)
    (node / "report_descriptor").write_bytes(WIRELESS_DESCRIPTOR)
    payload = run(fake_hid([info()], reports=WIRELESS_REPORTS), tmp_path)
    assert payload["descriptor"]["source"] == "sysfs" and payload["layout"] == "SpaceMouse Wireless"


def test_a_logitech_interface_that_is_no_multi_axis_controller_is_never_opened(tmp_path, no_owners):
    keyboard = info(vid=0x046D, pid=0xC542, product="USB Receiver", usage=(1, 6))
    sdk = fake_hid([keyboard], descriptor=WIRELESS_DESCRIPTOR)
    assert probe.candidate_interfaces(sdk, sysfs=tmp_path / "sys") == []
    with pytest.raises(probe.DeviceNotFound):
        run(sdk, tmp_path, hid_id=(0x046D, 0xC542))
    assert sdk.opened == []


def test_the_multi_axis_interface_of_a_device_is_probed(tmp_path, no_owners):
    sdk = fake_hid([info(path=b"/dev/hidraw2", usage=(1, 6), interface=0), info(path=b"/dev/hidraw3", interface=1)],
                   descriptor=WIRELESS_DESCRIPTOR)
    run(sdk, tmp_path)
    assert sdk.opened[0].path == b"/dev/hidraw3"


def test_another_hardware_owner_blocks_the_probe(tmp_path, monkeypatch):
    monkeypatch.setattr("real_world_rl.common.device_ownership.process_owners", lambda *a, **k: ["pid 7: rwrl teleop run"])
    sdk = fake_hid([info()], descriptor=WIRELESS_DESCRIPTOR)
    with pytest.raises(probe.DeviceBusy):
        run(sdk, tmp_path)
    assert sdk.opened == [] and not (tmp_path / "teleop").exists()


@pytest.fixture
def isolated(tmp_path, monkeypatch, no_owners):
    """No station anywhere; the fake hidapi is what the probe imports."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    for name in ("RWRL_STATION", "ROBOT_INFRA_CONFIG"):
        monkeypatch.delenv(name, raising=False)

    def install(sdk):
        monkeypatch.setattr(spacemouse_hid, "require_sdk", lambda: sdk)
        return sdk

    return install


def test_cli_without_a_station_needs_output(isolated, capsys):
    sdk = isolated(fake_hid([info()], descriptor=WIRELESS_DESCRIPTOR))
    assert main(["--json", "teleop", "probe-hid"]) == 2
    assert json.loads(capsys.readouterr().out)["required_fields"] == ["--output"]
    assert sdk.opened == []


def test_cli_refuses_an_existing_probe_directory(isolated, tmp_path, capsys):
    sdk = isolated(fake_hid([info()], descriptor=WIRELESS_DESCRIPTOR))
    (tmp_path / "out").mkdir()
    assert main(["--json", "teleop", "probe-hid", "--output", str(tmp_path / "out")]) == 2
    assert "exists; pass --output DIR or remove it" in json.loads(capsys.readouterr().out)["error"]
    assert sdk.opened == []


def test_cli_lists_the_candidates_when_more_than_one_device_is_attached(isolated, tmp_path, capsys):
    sdk = isolated(fake_hid([info(), info(pid=0xC635, path=b"/dev/hidraw4", product="SpaceMouse Compact")]))
    assert main(["--json", "teleop", "probe-hid", "--output", str(tmp_path / "out")]) == 2
    error = json.loads(capsys.readouterr().out)
    assert error["required_fields"] == ["--device"]
    assert [row["hid_id"] for row in error["candidates"]] == ["256f:c635", "256f:c63a"]
    assert SERIAL not in json.dumps(error) and sdk.opened == []


@pytest.mark.parametrize("argv", [["--device", "256f"], ["--duration-s", "0"], ["--duration-s", "301"]])
def test_cli_rejects_bad_arguments_before_any_device_access(isolated, tmp_path, capsys, argv):
    sdk = isolated(fake_hid([info()]))
    assert main(["--json", "teleop", "probe-hid", "--output", str(tmp_path / "out"), *argv]) == 2
    assert sdk.opened == []


def test_cli_json_output_is_the_probe_payload(isolated, tmp_path, capsys):
    isolated(fake_hid([info()], descriptor=WIRELESS_DESCRIPTOR, reports=WIRELESS_REPORTS))
    assert main(["--json", "teleop", "probe-hid", "--device", "256f:c63a", "--duration-s", "0.2",
                 "--output", str(tmp_path / "out")]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["layout"] == "SpaceMouse Wireless" and payload["reports"]["count"] == 4
    assert payload["files"]["candidate"] == str(tmp_path / "out" / "candidate.yaml")
    assert payload["candidate"][next(iter(payload["candidate"]))]["evidence"]["probe"] == str((tmp_path / "out").resolve()) + "/"
    assert SERIAL not in json.dumps(payload)
