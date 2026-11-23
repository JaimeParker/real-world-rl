"""Fixture dataset written directly per the format spec (no writer involved)."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

CONTROL_HZ = 10.0
H, W = 9, 15
DEPTH_KEY = "depth_front"


def _ffmpeg() -> str | None:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


def _write_mp4(path: Path, frames: np.ndarray, ffmpeg: str) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
           "-s", f"{frames.shape[2]}x{frames.shape[1]}", "-r", str(CONTROL_HZ), "-i", "-",
           "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-pix_fmt", "yuv420p", str(path)]
    done = subprocess.run(cmd, input=frames.tobytes(), capture_output=True)
    return done.returncode == 0


def _quat_z(angle: np.ndarray) -> np.ndarray:
    return np.stack([np.zeros_like(angle), np.zeros_like(angle),
                     np.sin(angle / 2), np.cos(angle / 2)], axis=-1)


def _episode(root: Path, index: int, steps: int, *, status: str, reason, cameras: bool,
             success, ffmpeg: str | None) -> None:
    rng = np.random.default_rng(index)
    ep = root / f"episode_{index:06d}"
    ep.mkdir(parents=True)
    n = steps + 1
    yaw = np.linspace(0.0, 1.0, n)
    tcp = np.concatenate([rng.normal(size=(n, 3)), _quat_z(yaw)], axis=1)
    intervened = np.zeros(steps, bool)
    intervened[steps // 2:] = True
    data = {
        "obs_time_s": 1000.0 + np.arange(n) / CONTROL_HZ,
        "state": rng.normal(size=(n, 10)).astype(np.float32),
        "named.joint_pos": rng.normal(size=(n, 7)).astype(np.float32),
        "named.tcp_pose": tcp.astype(np.float32),
        "named.gripper_pos": rng.random((n, 1)).astype(np.float32),
        "stack_reset": np.eye(1, n, 0, dtype=bool)[0],
        "action": rng.normal(size=(steps, 7)).astype(np.float32),
        "policy_action": rng.normal(size=(steps, 7)).astype(np.float32),
        "human_action_mask": np.ones((steps, 7), bool),
        "override_mask": np.zeros((steps, 7), bool),
        "intervened": intervened,
        "reward": np.zeros(steps, np.float32),
        "terminated": np.zeros(steps, bool),
        "truncated": np.zeros(steps, bool),
        "syncing": np.zeros(steps, bool),
        "event.success": np.eye(1, steps, steps - 1, dtype=bool)[0],
    }
    np.savez_compressed(ep / "data.npz", **data)
    videos = {}
    if cameras:
        for key in ("rgb_wrist", "rgb_front"):
            frames = rng.integers(0, 255, size=(n, H, W, 3), dtype=np.uint8)
            (ep / "frames").mkdir(exist_ok=True)
            np.savez_compressed(ep / "frames" / f"{key}.npz", frames=frames)
            if ffmpeg and _write_mp4(ep / "video" / f"{key}.mp4", frames, ffmpeg):
                videos[key] = f"video/{key}.mp4"
        depth = rng.random((n, H, W, 1)).astype(np.float32)
        np.savez_compressed(ep / "frames" / f"{DEPTH_KEY}.npz", frames=depth)
    half = steps // 2
    meta = {
        "index": index, "episode_id": f"{index:032x}", "status": status,
        "discard_reason": reason, "recorded_discard_reason": reason, "success": success,
        "steps": steps, "intervened_steps": steps - half,
        "segments": [{"start": 0, "end": half, "kind": "hold"},
                     {"start": half, "end": steps, "kind": "intervene"}],
        "events": [{"step": steps - 1, "name": "success"}] if success else [],
        "started_at": "2026-01-02T03:04:05Z", "duration_s": steps / CONTROL_HZ,
        "operator_quit": False,
        "notes": [{"at": "2026-01-02T03:05:00Z", "text": "gripper slipped"}],
        "history": [], "videos": videos, "extra": {},
    }
    (ep / "meta.json").write_text(json.dumps(meta))


@pytest.fixture(scope="session")
def dataset_root(tmp_path_factory) -> Path:
    """Episode 0 kept with two rgb keys (15x9, odd) + depth, episode 3 discarded without cameras."""
    root = tmp_path_factory.mktemp("data") / "peg"
    root.mkdir()
    (root / "dataset.json").write_text(json.dumps({
        "format": "rwrl-dataset", "schema_version": 1, "created_at": "2026-01-02T03:04:05Z",
        "next_index": 4, "control_hz": CONTROL_HZ, "task_description": "peg",
        "action_contract": {}, "action_dim": 7, "recorded_frame_stack": 1, "config": {},
        "obs_spec": {"state": {"shape": [10], "dtype": "float32"},
                     "rgb_wrist": {"shape": [H, W, 3], "dtype": "uint8"},
                     "rgb_front": {"shape": [H, W, 3], "dtype": "uint8"},
                     DEPTH_KEY: {"shape": [H, W, 1], "dtype": "float32"}},
        "named_state_spec": {"joint_pos": [7], "tcp_pose": [7], "gripper_pos": [1]},
        "extra": {},
    }))
    ffmpeg = _ffmpeg()
    _episode(root, 0, 12, status="kept", reason=None, cameras=True, success=True, ffmpeg=ffmpeg)
    _episode(root, 3, 8, status="discarded", reason="rerecord", cameras=False, success=None,
             ffmpeg=None)
    return root
