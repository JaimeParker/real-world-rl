import threading
import time

from real_world_rl.devices.stop_sampling import StopTraceRecorder
from real_world_rl.devices.stop_trace import evaluate_stop_trace


def sample():
    return {'device_time_s': time.monotonic(), 'tcp_pose': [0, 0, 0, 0, 0, 0, 1],
            'joint_vel': [0]*7, 'errors': []}


def test_sampler_records_during_the_stop_interval_without_commanding():
    reached = threading.Event()
    def read():
        reached.set()
        return sample()
    recorder = StopTraceRecorder(read, period_s=.001).start()
    assert reached.wait(1.)
    recorder.mark_stop_start()
    reached.clear()
    assert reached.wait(1.)
    recorder.mark_stop_end()
    trace = recorder.finish()
    assert trace['samples']
    assert trace['stop_start_s'] <= trace['stop_end_s']
    assert all(row['read_duration_s'] >= 0 for row in trace['samples'])
    assert not trace['sampling_errors']
    assert evaluate_stop_trace(trace)['status'] == 'BLOCKED'  # too little observation


def test_failed_read_is_evidence_gap_not_synthetic_samples():
    reached = threading.Event()
    def read():
        reached.set()
        raise OSError('lost feedback')
    recorder = StopTraceRecorder(read).start()
    assert reached.wait(1.)
    recorder.mark_stop_start()
    recorder.mark_stop_end()
    trace = recorder.finish()
    assert not trace['samples']
    assert trace['sampling_errors'] == ['OSError: lost feedback']
    assert evaluate_stop_trace(trace)['status'] == 'BLOCKED'


def test_blocked_read_finish_is_bounded_and_reported():
    entered, release = threading.Event(), threading.Event()
    def read():
        entered.set()
        release.wait(2.)
        return sample()
    recorder = StopTraceRecorder(read).start()
    try:
        assert entered.wait(1.)
        recorder.mark_stop_start()
        recorder.mark_stop_end()
        trace = recorder.finish(join_timeout_s=.01)
        assert 'sample read still blocked at finish' in trace['sampling_errors']
    finally:
        release.set()
        recorder.finish()
