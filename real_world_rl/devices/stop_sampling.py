"""Bounded side-channel sampling around a caller-owned stop operation.

The sampler never commands motion, creates a robot connection, or feeds a
watchdog. Only supply a read callback whose backend supports concurrent reads.
"""
from __future__ import annotations

import copy
import math
import threading
import time


class StopTraceRecorder:
    """Collect fresh SDK samples and explicit stop boundaries without filtering.

    Start during normal control to obtain pre-stop evidence. The owner marks the
    actual stop call's start/end, keeps the connection alive for post observation,
    then finishes. A bounded join cannot certify a hung backend; it records that
    as missing evidence. No callback runs inside a real-time control callback.
    """

    def __init__(self, read_sample, *, period_s=.01, max_samples=10000,
                 workspace=None, task_frame=None, metadata=None, pre_window_s=None):
        if not math.isfinite(period_s) or period_s <= 0:
            raise ValueError('period_s must be finite and positive')
        if type(max_samples) is not int or max_samples < 2:
            raise ValueError('max_samples must be at least 2')
        if pre_window_s is not None and (not math.isfinite(pre_window_s) or pre_window_s <= 0):
            raise ValueError("pre_window_s must be finite and positive")
        self._pre_window = pre_window_s
        self._read = read_sample
        self._period = period_s
        self._maximum = max_samples
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._document = {'schema_version': 1, 'samples': [], 'sampling_errors': [],
                          'workspace': copy.deepcopy(workspace),
                          'task_frame': copy.deepcopy(task_frame),
                          'metadata': copy.deepcopy(metadata or {}),
                          'requested_period_s': period_s}

    def start(self):
        if self._thread is not None:
            raise RuntimeError('stop sampler can only be started once')
        self._thread = threading.Thread(target=self._collect, name='stop-evidence', daemon=True)
        self._thread.start()
        return self

    def mark_stop_start(self, *, control_mode=None, tracker_active=None, monotonic_s=None):
        with self._lock:
            if 'stop_start_s' in self._document:
                raise RuntimeError('stop already started')
            self._document['stop_start_s'] = time.monotonic() if monotonic_s is None else monotonic_s
            self._document['control_mode'] = control_mode
            self._document['tracker_active'] = tracker_active

    def mark_stop_end(self, error=None, *, monotonic_s=None):
        with self._lock:
            if 'stop_start_s' not in self._document or 'stop_end_s' in self._document:
                raise RuntimeError('invalid stop end boundary')
            self._document['stop_end_s'] = time.monotonic() if monotonic_s is None else monotonic_s
            if error is not None:
                self._document['stop_error'] = f'{type(error).__name__}: {error}'

    def _collect(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                row = dict(self._read())
                row['monotonic_s'] = time.monotonic()
                row['read_duration_s'] = row['monotonic_s'] - started
                with self._lock:
                    samples = self._document['samples']
                    if self._pre_window is not None and 'stop_start_s' not in self._document:
                        cutoff = row['monotonic_s'] - self._pre_window
                        while len(samples) > 1 and samples[1]['monotonic_s'] < cutoff:
                            samples.pop(0)
                    if len(samples) >= self._maximum:
                        self._document['sampling_errors'].append('sample capacity exhausted')
                        return
                    self._document['samples'].append(copy.deepcopy(row))
            except Exception as error:
                with self._lock:
                    self._document['sampling_errors'].append(f'{type(error).__name__}: {error}')
                return
            self._stop.wait(max(0., self._period - (time.monotonic() - started)))

    @property
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def finish(self, *, join_timeout_s=1.):
        if not math.isfinite(join_timeout_s) or join_timeout_s < 0:
            raise ValueError('join_timeout_s must be finite and nonnegative')
        self._stop.set()
        if self._thread is not None:
            self._thread.join(join_timeout_s)
        with self._lock:
            result = copy.deepcopy(self._document)
        if self._thread is not None and self._thread.is_alive():
            result['sampling_errors'].append('sample read still blocked at finish')
        return result
