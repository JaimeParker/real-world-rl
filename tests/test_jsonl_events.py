import os

import pytest

from real_world_rl.teleop.keyboard.jsonl import JsonlEventSource
from real_world_rl.devices.errors import DeviceFault


def test_appended_events_preserve_repeated_takeover(tmp_path):
    path = tmp_path / "events.jsonl"
    path.write_text('')
    source = JsonlEventSource(path=str(path))
    source.start()
    try:
        assert not source.poll().intervened
        with path.open('a') as stream:
            stream.write('{"event":"takeover"}\n{"event":"handback"}\n{"event":"takeover"}\n')
        assert source.poll().intervened
        assert not source.poll().intervened
        assert source.poll().intervened
        source.reset()
        assert not source.poll().intervened
        with path.open('a') as stream:
            stream.write('{"event":"quit"}\n')
        result = source.poll()
        assert result.episode_end and result.events['operator_quit']
    finally:
        source.close()


@pytest.mark.parametrize('line', ['{}', '{"event":"move"}', '{"event":"quit","x":1}', 'null', '{'])
def test_invalid_events_fail_closed(tmp_path, line):
    path = tmp_path / "events.jsonl"
    path.write_text(line + '\n')
    source = JsonlEventSource(path=str(path))
    source.start()
    try:
        with pytest.raises(DeviceFault, match='invalid JSONL'):
            source.poll()
    finally:
        source.close()


def test_pipe_partial_line_and_eof():
    read, write = os.pipe()
    source = JsonlEventSource(fd=read)
    source.start()
    try:
        os.write(write, b'{"event":')
        assert not source.poll().intervened
        os.write(write, b'"takeover"}\n')
        assert source.poll().intervened
        os.close(write)
        write = None
        with pytest.raises(DeviceFault, match='closed before quit'):
            source.poll()
    finally:
        source.close()
        os.close(read)
        if write is not None:
            os.close(write)
