"""Bounded SpaceMouse sampling without opening a robot or starting a controller."""
from __future__ import annotations

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

import numpy as np

from robot_infra.devices.backends.teleops.spacemouse import map_spacemouse_axes
from real_world_rl.common.json_io import write_json

from real_world_rl.teleop.calibration import yaw_rotation
from real_world_rl.teleop.input_frame import rotate_input

LABELS = ('+x', '+y', '-x', '-y', '+z', '-z', '+rx', '-rx', '+ry', '-ry', '+rz', '-rz')


def report_directory(directory=None):
    if directory is None:
        if (Path.cwd() / 'pyproject.toml').is_file() and os.access(Path.cwd(), os.W_OK):
            directory = Path.cwd() / 'local' / 'teleop'
        else:
            directory = Path(os.environ.get('XDG_STATE_HOME', str(Path.home() / '.local/state'))) / 'rwrl' / 'teleop'
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    path = root / (time.strftime('%Y%m%dT%H%M%S', time.gmtime()) + '-' + uuid.uuid4().hex[:10])
    path.mkdir(mode=0o700)
    return path



@contextmanager
def open_input(config):
    from real_world_rl.common.device_ownership import process_owners
    from robot_infra.devices.errors import DeviceBusy
    from robot_infra.devices.backends.teleops import _spacemouse_hid
    if config.teleop is None or config.teleop.backend != 'spacemouse':
        raise ValueError('station must configure a SpaceMouse teleop')
    owners = process_owners()
    if owners:
        raise DeviceBusy('exclusive input diagnostic blocked: ' + '; '.join(owners),
                         hint='finish the owning session before running diagnostics')
    reader = _spacemouse_hid.open(device_index=int(config.teleop.options.get('device_index', 0)), claim=config.claim)
    try:
        yield reader
    finally:
        reader.close()


def capture(reader, *, duration_s, label=None, rotation=None, yaw_rad=0., invert_axes=(),
            clock=time.monotonic, sleep=time.sleep, stream=None):
    """Record decoded and transformed samples; polling and report times are distinct."""
    if not math.isfinite(duration_s) or not 0 < duration_s <= 300:
        raise ValueError('duration_s must be in (0, 300]')
    if label is not None and label not in LABELS:
        raise ValueError('unknown direction label')
    start = clock()
    initial_report_count = reader.report_count
    previous_report_count = initial_report_count
    rows = []
    while clock() - start < duration_s:
        state = reader.read()
        now = clock()
        raw = np.array([state.x, state.y, state.z, state.roll, state.pitch, state.yaw])
        mapped = map_spacemouse_axes(raw, invert_axes)
        if not np.isfinite(mapped).all():
            raise ValueError('nonfinite HID sample')
        # HID empty reads preserve state. Do not interpret cached polls as new
        # physical samples or infer disconnection from a guessed report period.
        last_report = reader.last_report_monotonic
        fresh_report = reader.report_count > previous_report_count
        previous_report_count = reader.report_count
        row = {'label': label, 'monotonic_s': now, 'decoded': raw.tolist(),
               'mapped': mapped.tolist(), 'task_input': rotate_input(mapped, yaw_rotation(yaw_rad)).tolist(),
               'base_input': rotate_input(mapped, rotation).tolist(), 'buttons': list(state.buttons),
               'report_count': reader.report_count, 'last_report_monotonic': last_report,
               'fresh_report': fresh_report}
        rows.append(row)
        if stream is not None:
            stream.write(json.dumps(row, allow_nan=False) + '\n')
        sleep(.005)
    if reader.report_count <= initial_report_count:
        raise RuntimeError('no new HID reports during this capture; input evidence is unavailable')
    return rows


def collect(config, *, duration_s, directory=None, labels=None, rotation=None, yaw_rad=0., interactive=True):
    """Interactive labeled acquisition, or one bounded unlabelled diagnostic."""
    if labels is not None and interactive and not sys.stdin.isatty():
        raise ValueError('interactive calibration requires a terminal; supply --samples for unattended fitting')
    if not math.isfinite(duration_s) or not 0 < duration_s <= 300:
        raise ValueError('duration_s must be in (0, 300]')
    report = report_directory(directory)
    samples = {}
    try:
        from dataclasses import asdict
        from real_world_rl.common.provenance import runtime_provenance
        write_json(report / 'provenance.json', runtime_provenance(config=asdict(config)))
        with open_input(config) as reader:
            identity = reader.identity
            with (report / 'samples.jsonl').open('x', encoding='utf-8') as stream:
                os.chmod(report / 'samples.jsonl', 0o600)
                for label in labels if labels is not None else [None]:
                    if label is not None:
                        print(f'Task direction {label}: ' + ('release the puck, then press Enter. '
                              if interactive else 'capture starts now. ') +
                              f'For the next {duration_s:g}s, push/rotate in the direction '
                              'you want the TCP to follow. Fixed tabletop axes: forward +X, left +Y, '
                              'up +Z when standing behind the arm. Rotation signs use the right-hand rule.',
                              file=sys.stderr, flush=True)
                        if interactive and sys.stdin.readline() == '':
                            raise EOFError('operator input closed')
                    rows = capture(reader, duration_s=duration_s, label=label, rotation=rotation,
                                   yaw_rad=yaw_rad, invert_axes=config.teleop.options.get('invert_axes', ()),
                                   stream=stream)
                    samples[label or 'unlabelled'] = [row['mapped'] for row in rows if row['fresh_report']]
        write_json(report / 'labelled-samples.json', samples)
        result = {'schema_version': 1, 'report_directory': str(report), 'device_identity': identity,
                  'sample_counts': {key: len(value) for key, value in samples.items()}}
        write_json(report / 'summary.json', result)
        return samples, result
    except BaseException as error:
        write_json(report / 'error.json', {'error': str(error), 'report_directory': str(report)})
        raise
