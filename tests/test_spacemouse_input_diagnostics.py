from types import SimpleNamespace

import numpy as np
import pytest

from real_world_rl.teleop.spacemouse.calibration import fit_yaw, validate_axes, yaw_rotation
from real_world_rl.teleop.spacemouse.diagnostics import capture


class Clock:
    def __init__(self):
        self.now = 10.
    def __call__(self):
        return self.now
    def sleep(self, duration):
        self.now += duration


class Reader:
    def __init__(self, clock):
        self.clock = clock
        self.report_count = 0
        self.last_report_monotonic = None
    def read(self):
        self.report_count += 1
        self.last_report_monotonic = self.clock()
        return SimpleNamespace(x=.2, y=-.4, z=.1, roll=.1, pitch=-.2, yaw=.3, buttons=[0, 0])


def test_capture_records_each_coordinate_stage_and_times():
    clock = Clock()
    reader = Reader(clock)
    rotation = yaw_rotation(.3)
    rows = capture(reader, duration_s=.02, rotation=rotation, yaw_rad=.3,
                   clock=clock, sleep=clock.sleep)
    assert 3 <= len(rows) <= 5
    np.testing.assert_allclose(rows[0]['mapped'], [.4, .2, .1, -.1, .2, -.3])
    np.testing.assert_allclose(rows[0]['base_input'][:3], rotation @ [.4, .2, .1])
    assert rows[0]['last_report_monotonic'] == rows[0]['monotonic_s']


def test_all_six_axes_have_separate_validation():
    data = {}
    yaw = .6
    for axis in ('x', 'y', 'z', 'rx', 'ry', 'rz'):
        for sign in ('+', '-'):
            vector = np.zeros(3)
            vector['xyz'.index(axis[-1])] = .4 if sign == '+' else -.4
            raw = yaw_rotation(yaw).T @ vector
            value = np.zeros(6)
            start = 3 if axis.startswith('r') else 0
            value[start:start+3] = raw
            data[sign+axis] = [value.tolist()] * 4
    fit = fit_yaw(data)
    validated = validate_axes(data, fit['yaw_rad'])
    assert validated['rotation_validated'] and validated['vertical_validated']
    data['+rz'] = data['-rz']
    with pytest.raises(ValueError, match='direction error'):
        validate_axes(data, fit['yaw_rad'])


def test_missing_rotation_data_is_not_accepted_as_validated():
    assert not validate_axes({}, .4)['rotation_validated']


def test_old_reports_do_not_satisfy_a_new_capture():
    clock = Clock()
    reader = Reader(clock)
    state = reader.read()
    reader.read = lambda: state
    with pytest.raises(RuntimeError, match='no new HID reports during this capture'):
        capture(reader, duration_s=.02, clock=clock, sleep=clock.sleep)


def test_change_only_reports_keep_cached_polls_distinct():
    clock = Clock()
    reader = Reader(clock)
    original = reader.read
    calls = []
    def read():
        if not calls:
            calls.append(original())
        return calls[0]
    reader.read = read
    rows = capture(reader, duration_s=.7, clock=clock, sleep=clock.sleep)
    assert sum(row['fresh_report'] for row in rows) == 1
    assert rows[-1]['mapped'] == rows[0]['mapped']
    assert rows[-1]['monotonic_s'] - rows[-1]['last_report_monotonic'] > .5


def test_open_input_decodes_with_the_station_device_table(tmp_path, monkeypatch):
    import sys
    import types

    import yaml

    from real_world_rl.station import load_station
    from real_world_rl.teleop.spacemouse.diagnostics import open_input

    class Handle:
        def open_path(self, path):
            self.path = path
        def set_nonblocking(self, enabled):
            pass
        def read(self, size):
            return []
        def close(self):
            pass

    sdk = types.ModuleType('hid')
    sdk.device = Handle
    sdk.enumerate = lambda: [{'vendor_id': 0x256F, 'product_id': 0xC63A, 'path': b'bt'}]
    monkeypatch.setitem(sys.modules, 'hid', sdk)
    monkeypatch.setattr('real_world_rl.common.device_ownership.process_owners', lambda *a, **k: [])
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
    station = tmp_path / 'station.yaml'
    station.write_text(yaml.safe_dump({'type': 'franka', 'name': 'arm', 'arm_backend': 'fake', 'claim': False,
                                       'teleop': {'backend': 'spacemouse'}}))
    (tmp_path / 'spacemouse-devices.yaml').write_text(
        'schema_version: 1\ndevices:\n  BT: {hid_id: [0x256F, 0xC63A], layout: SpaceMouse Wireless}\n')
    config = load_station(station).select().config
    with open_input(config) as reader:
        assert reader.identity['model'] == 'BT'
        assert reader.device_spec['source'] == 'local' and reader.device_spec['hid_id'] == '256f:c63a'
