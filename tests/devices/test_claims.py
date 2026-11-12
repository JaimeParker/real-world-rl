"""Claims cache migration, using only temporary lock files."""
from __future__ import annotations

from pathlib import Path

import filelock
import pytest

from real_world_rl.devices import claims
from real_world_rl.devices.errors import DeviceBusy


@pytest.fixture
def claim_paths(tmp_path, monkeypatch):
    monkeypatch.setattr(claims, '_DEFAULT_LOCK_DIR', None)
    monkeypatch.setattr(claims, '_LEGACY_LOCK_DIR', tmp_path / '.cache/robot_infra/claims')
    monkeypatch.setattr(Path, 'home', classmethod(lambda cls: tmp_path))
    monkeypatch.delenv('XDG_CACHE_HOME', raising=False)
    return tmp_path


def test_default_claim_directory_and_key_are_unchanged_except_cache_name(claim_paths):
    claim = claims.DeviceClaim('franka:10.0.0.2')
    assert claim.key == 'franka:10.0.0.2'
    with claim:
        assert (claim_paths / '.cache/rwrl/claims/franka_10.0.0.2.lock').exists()
    assert not claims._LEGACY_LOCK_DIR.exists()


def test_xdg_cache_home_is_resolved_for_each_claim(claim_paths, monkeypatch):
    for name in ('cache-one', 'cache-two'):
        cache = claim_paths / name
        monkeypatch.setenv('XDG_CACHE_HOME', str(cache))
        with claims.DeviceClaim('robot:address'):
            assert (cache / 'rwrl/claims/robot_address.lock').exists()
    monkeypatch.setenv('XDG_CACHE_HOME', '')
    with claims.DeviceClaim('robot:address'):
        assert (claim_paths / '.cache/rwrl/claims/robot_address.lock').exists()


def test_held_legacy_lock_refuses_claim_and_releases_new_lock(claim_paths, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(claim_paths / 'xdg-cache'))
    claims._LEGACY_LOCK_DIR.mkdir(parents=True)
    legacy_path = claims._LEGACY_LOCK_DIR / 'robot_address.lock'
    claim = claims.DeviceClaim('robot:address')
    with filelock.FileLock(legacy_path, timeout=0):
        with pytest.raises(DeviceBusy) as caught:
            claim.acquire()
        assert str(legacy_path) in caught.value.hint
        assert 'claimed by another process' in caught.value.message
        assert not claim.held
        with filelock.FileLock(claim._lock.lock_file, timeout=0):
            pass
    with claim:
        assert claim.held


def test_stale_legacy_file_is_held_until_final_release(claim_paths):
    claims._LEGACY_LOCK_DIR.mkdir(parents=True)
    legacy_path = claims._LEGACY_LOCK_DIR / 'robot_address.lock'
    legacy_path.touch()
    claim = claims.DeviceClaim('robot:address')
    claim.acquire()
    claim.acquire()
    claim.release()
    assert claim.held
    with pytest.raises(filelock.Timeout):
        filelock.FileLock(legacy_path, timeout=0).acquire()
    claim.release()
    with filelock.FileLock(legacy_path, timeout=0):
        pass
    assert not claim.held


def test_explicit_lock_directory_still_honored(claim_paths, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(claim_paths / 'xdg-cache'))
    with claims.DeviceClaim('robot:address', lock_dir=claim_paths / 'explicit'):
        assert (claim_paths / 'explicit/robot_address.lock').exists()
    assert not (claim_paths / 'xdg-cache').exists()


def test_explicit_legacy_directory_does_not_lock_itself_twice(claim_paths):
    with claims.DeviceClaim('robot:address', lock_dir=claims._LEGACY_LOCK_DIR):
        with pytest.raises(DeviceBusy):
            claims.DeviceClaim('robot:address', lock_dir=claims._LEGACY_LOCK_DIR).acquire()
