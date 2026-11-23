"""Writing a dataset: ``DatasetWriter`` opens (creates or appends to) a dataset directory and
hands out one ``EpisodeWriter`` at a time.

An episode is buffered in memory while it runs and written when it ends
(``EpisodeWriter.finish``): ``data.npz``, the lossless ``frames/<key>.npz`` and a review
``video/<key>.mp4`` per RGB key go into a ``.tmp-episode_...`` directory, are fsynced, and the
directory is renamed to ``episode_{index:06d}``; only then is ``next_index`` bumped in
``dataset.json``. A crash before the rename leaves a ``.tmp-`` directory, which the next
``DatasetWriter.open`` removes. Kept and discarded episodes are both written; ``status`` in
``meta.json`` tells them apart.

``add_observation``/``add_step`` take what the recorder sees from a batch-of-1 env: numpy arrays
or torch tensors with a leading dimension of 1. An image that arrives frame-stacked,
``(1, T, H, W, C)``, is stored as its newest frame; ``stack_reset`` marks the rows where the
live frame-stack history was refilled, so the reader can rebuild the stacks.
"""
from __future__ import annotations

import logging
import os
import secrets
import shutil
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np

from real_world_rl.dataset import format as fmt
from real_world_rl.dataset.reader import EpisodeInfo, scan_episodes
from real_world_rl.dataset.video import encode_video

__all__ = ["DatasetWriter", "EpisodeWriter"]

_LOGGER = logging.getLogger(__name__)


def _to_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "detach") and hasattr(value, "cpu"):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _scalar(value: Any, name: str) -> Any:
    array = _to_numpy(value)
    if array.size != 1:
        raise ValueError(f"{name} must be one value, got shape {array.shape}")
    return array.reshape(()).item()


def _normalized_obs_spec(obs_spec: Mapping[str, Mapping[str, Any]]) -> dict:
    spec = {}
    for key, entry in obs_spec.items():
        shape = [int(size) for size in entry["shape"]]
        if key != fmt.STATE_KEY and len(shape) != 3:
            raise ValueError(f"obs_spec[{key!r}] must be one (H, W, C) frame, got shape {shape}")
        spec[str(key)] = {"shape": shape, "dtype": np.dtype(entry["dtype"]).name}
    return spec


def _clean_stale_tmp_dirs(root: Path) -> None:
    for path in sorted(root.glob(fmt.TMP_PREFIX + "*")):
        if path.is_dir():
            _LOGGER.warning("removing the unfinished episode write %s (left by an interrupted recording)", path)
            shutil.rmtree(path)


class DatasetWriter:
    """An open dataset directory being recorded into. Use ``DatasetWriter.open``."""

    def __init__(self, root: Path, header: dict) -> None:
        self.root = root
        self.header = header
        self._active: Optional[EpisodeWriter] = None

    @classmethod
    def open(
        cls,
        root: str | Path,
        *,
        control_hz: float,
        obs_spec: Mapping[str, Mapping[str, Any]],
        action_dim: int,
        action_contract: Optional[Mapping[str, Any]] = None,
        named_state_spec: Optional[Mapping[str, Any]] = None,
        task_description: str = "",
        recorded_frame_stack: int = 1,
        config: Optional[Mapping[str, Any]] = None,
        extra: Optional[Mapping[str, Any]] = None,
    ) -> "DatasetWriter":
        """Creates the dataset at ``root`` or appends to the one there.

        Appending needs equal ``action_contract``, ``obs_spec``, ``named_state_spec``,
        ``action_dim`` and ``control_hz`` (``DatasetMismatchError`` names the differing keys); the
        existing header is kept otherwise. Unfinished ``.tmp-`` episode writes are removed with a
        warning. A non-empty directory without ``dataset.json`` is refused."""
        root = Path(root)
        header = fmt.json_normalize({
            "format": fmt.FORMAT,
            "schema_version": fmt.SCHEMA_VERSION,
            "created_at": fmt.utc_now_iso(),
            "next_index": 0,
            "control_hz": float(control_hz),
            "task_description": task_description,
            "action_contract": None if action_contract is None else dict(action_contract),
            "obs_spec": _normalized_obs_spec(obs_spec),
            "named_state_spec": {str(k): [int(s) for s in v] for k, v in (named_state_spec or {}).items()},
            "action_dim": int(action_dim),
            "recorded_frame_stack": int(recorded_frame_stack),
            "config": dict(config or {}),
            "extra": dict(extra or {}),
        })
        header_path = root / fmt.DATASET_FILE
        if header_path.is_file():
            saved = fmt.read_json(header_path)
            fmt.validate_header(saved, root)
            differing = fmt.differing_header_keys(saved, header)
            if differing:
                raise fmt.DatasetMismatchError(root, differing)
            _clean_stale_tmp_dirs(root)
            writer = cls(root, saved)
            writer._reconcile_next_index()
            return writer
        if root.exists() and any(root.iterdir()):
            raise ValueError(f"{root} exists, is not empty and has no {fmt.DATASET_FILE}; pick a new dataset directory")
        root.mkdir(parents=True, exist_ok=True)
        fmt.write_json_atomic(header_path, header)
        return cls(root, header)

    @property
    def next_index(self) -> int:
        return int(self.header["next_index"])

    def episodes(self) -> list[EpisodeInfo]:
        return scan_episodes(self.root)

    @property
    def num_kept(self) -> int:
        """Kept episodes already in the dataset (a resumed recording counts them)."""
        return sum(info.kept for info in self.episodes())

    @property
    def active_episode(self) -> Optional["EpisodeWriter"]:
        """The episode begun and not yet finished or aborted, else ``None`` (a recorder that was
        interrupted finishes or aborts it)."""
        return None if self._active is None or self._active.closed else self._active

    def begin_episode(self, *, episode_id: Optional[str] = None) -> "EpisodeWriter":
        """Starts episode ``next_index``; one episode at a time (finish or abort the last one)."""
        if self._active is not None and not self._active.closed:
            raise RuntimeError(f"episode {self._active.index} is still open; finish() or abort() it first")
        self._active = EpisodeWriter(self, self.next_index, episode_id or uuid.uuid4().hex)
        return self._active

    def _reconcile_next_index(self) -> None:
        # A crash between an episode's rename and the header update leaves next_index behind.
        indices = [info.index for info in self.episodes()]
        if indices and max(indices) >= self.next_index:
            self._set_next_index(max(indices) + 1)

    def _set_next_index(self, value: int) -> None:
        self.header = dict(self.header, next_index=int(value))
        fmt.write_json_atomic(self.root / fmt.DATASET_FILE, self.header)


@dataclass
class _Buffers:
    obs_time_s: list = field(default_factory=list)
    stack_reset: list = field(default_factory=list)
    obs: dict = field(default_factory=dict)
    named: dict = field(default_factory=dict)
    steps: dict = field(default_factory=dict)
    events: dict = field(default_factory=dict)
    grasp_penalty: list = field(default_factory=list)
    num_steps: int = 0


class EpisodeWriter:
    """One episode being recorded: ``add_observation`` (T+1 times) and ``add_step`` (T times)
    alternate, starting with the first observation; then ``finish`` or ``abort``."""

    def __init__(self, dataset: DatasetWriter, index: int, episode_id: str) -> None:
        self._dataset = dataset
        self.index = index
        self.episode_id = episode_id
        self.started_at = fmt.utc_now_iso()
        self.closed = False
        self._tmp = dataset.root / f"{fmt.TMP_PREFIX}{fmt.episode_dir_name(index)}-{secrets.token_hex(4)}"
        self._tmp.mkdir()
        self._spec = dataset.header["obs_spec"]
        self._named_spec = dataset.header["named_state_spec"]
        self._action_dim = int(dataset.header["action_dim"])
        self._buffers = _Buffers(
            obs={key: [] for key in self._spec}, named={key: [] for key in self._named_spec},
            steps={key: [] for key in ("action", "policy_action", "human_action_mask", "override_mask",
                                       "intervened", "reward", "terminated", "truncated", "syncing")},
        )

    @property
    def num_steps(self) -> int:
        return self._buffers.num_steps

    @property
    def num_observations(self) -> int:
        """Observation rows added so far; ``finish`` needs ``num_steps + 1``."""
        return len(self._buffers.obs_time_s)

    # -- recording ----------------------------------------------------------------------

    def add_observation(
        self,
        policy_obs: Mapping[str, Any],
        named_state: Optional[Mapping[str, Any]] = None,
        *,
        time_s: float,
        stack_reset: bool = False,
    ) -> None:
        """Appends observation row ``t`` (the one before step ``t``). ``policy_obs`` has exactly
        the ``obs_spec`` keys, each ``(1, *shape)`` (an image may be ``(1, T, *shape)``, of which
        the newest frame is kept); ``named_state`` the ``named_state_spec`` fields (any shape of
        the right size, stored float32). ``stack_reset`` marks a refilled frame-stack history;
        row 0 is always one."""
        self._check_open()
        buffers = self._buffers
        if len(buffers.obs_time_s) != buffers.num_steps:
            raise RuntimeError("add_observation called twice without add_step in between")
        if set(policy_obs) != set(self._spec):
            raise ValueError(f"observation keys {sorted(policy_obs)} differ from obs_spec {sorted(self._spec)}")
        if set(named_state or {}) != set(self._named_spec):
            raise ValueError(f"named state fields {sorted(named_state or {})} differ from "
                             f"named_state_spec {sorted(self._named_spec)}")
        # Convert everything before appending anything, so a rejected row leaves no trace.
        obs = {key: self._unbatched_obs(key, value) for key, value in policy_obs.items()}
        named = {key: self._named_field(key, value) for key, value in (named_state or {}).items()}
        for key, value in obs.items():
            buffers.obs[key].append(value)
        for key, value in named.items():
            buffers.named[key].append(value)
        buffers.obs_time_s.append(float(time_s))
        buffers.stack_reset.append(bool(stack_reset) or not buffers.stack_reset)

    def add_step(
        self,
        action: Any,
        policy_action: Any,
        human_action_mask: Any,
        override_mask: Any,
        intervened: Any,
        reward: Any,
        terminated: Any,
        truncated: Any,
        syncing: Any,
        events: Mapping[str, Any],
        grasp_penalty: Any = None,
    ) -> None:
        """Appends step ``t`` (taken from observation ``t``). Actions and masks are ``(A,)`` or
        ``(1, A)``; the flags, reward and grasp penalty one value each. ``events`` maps the
        teleop event names of this step to their flags."""
        self._check_open()
        buffers = self._buffers
        if len(buffers.obs_time_s) != buffers.num_steps + 1:
            raise RuntimeError("add_step needs the observation it was taken from (add_observation first)")
        if buffers.num_steps and (grasp_penalty is None) != (not buffers.grasp_penalty):
            raise ValueError("grasp_penalty must be given on every step of an episode or on none")
        # Convert everything before appending anything, so a rejected step leaves no trace.
        row = {
            "action": self._action_row(action, "action", np.float32),
            "policy_action": self._action_row(policy_action, "policy_action", np.float32),
            "human_action_mask": self._action_row(human_action_mask, "human_action_mask", np.bool_),
            "override_mask": self._action_row(override_mask, "override_mask", np.bool_),
            "reward": float(_scalar(reward, "reward")),
        }
        for name, value in (("intervened", intervened), ("terminated", terminated),
                            ("truncated", truncated), ("syncing", syncing)):
            row[name] = bool(_scalar(value, name))
        penalty = None if grasp_penalty is None else float(_scalar(grasp_penalty, "grasp_penalty"))
        flags_of = {str(name): bool(_scalar(value, f"event {name!r}")) for name, value in events.items()}
        for name, value in row.items():
            buffers.steps[name].append(value)
        if penalty is not None:
            buffers.grasp_penalty.append(penalty)
        step = buffers.num_steps
        for name, flag in flags_of.items():
            buffers.events.setdefault(name, [False] * step).append(flag)
        for flags in buffers.events.values():
            if len(flags) == step:
                flags.append(False)
        buffers.num_steps += 1

    # -- ending -------------------------------------------------------------------------

    def finish(
        self,
        *,
        status: str,
        discard_reason: Optional[str] = None,
        success: Optional[bool] = None,
        events_list: Optional[list[dict]] = None,
        operator_quit: bool = False,
        extra: Optional[Mapping[str, Any]] = None,
    ) -> EpisodeInfo:
        """Writes the episode and makes it visible as ``episode_{index:06d}``.

        ``status`` is ``"kept"`` (``discard_reason`` None) or ``"discarded"`` (with one of
        ``format.DISCARD_REASONS``). ``events_list`` overrides the ``meta.events`` derived from the
        steps' event flags. Needs exactly T+1 observations for T steps."""
        self._check_open()
        _check_status(status, discard_reason)
        buffers = self._buffers
        if len(buffers.obs_time_s) != buffers.num_steps + 1:
            raise RuntimeError(f"an episode of {buffers.num_steps} steps needs {buffers.num_steps + 1} "
                               f"observations, got {len(buffers.obs_time_s)}")
        try:
            data = self._data_arrays()
            meta = self._meta(data, status, discard_reason, success, events_list, operator_quit, extra)
            self._write_files(data, meta)
            final = self._dataset.root / fmt.episode_dir_name(self.index)
            if final.exists():
                raise FileExistsError(f"{final} already exists")
            os.rename(self._tmp, final)
            fmt.fsync_dir(self._dataset.root)
        except BaseException:
            self.abort()
            raise
        self.closed = True
        self._dataset._set_next_index(self.index + 1)
        return EpisodeInfo(self.index, final, meta)

    def abort(self) -> None:
        """Drops the episode: nothing of it stays on disk and its index is not used."""
        self.closed = True
        shutil.rmtree(self._tmp, ignore_errors=True)

    # -- internals ----------------------------------------------------------------------

    def _check_open(self) -> None:
        if self.closed:
            raise RuntimeError(f"episode {self.index} is already finished or aborted")

    def _unbatched_obs(self, key: str, value: Any) -> np.ndarray:
        entry = self._spec[key]
        shape, dtype = tuple(entry["shape"]), np.dtype(entry["dtype"])
        array = _to_numpy(value)
        if array.shape == (1, *shape):
            array = array[0]
        elif key != fmt.STATE_KEY and array.ndim == len(shape) + 2 and array.shape[0] == 1 \
                and array.shape[2:] == shape:
            array = array[0, -1]
        else:
            raise ValueError(f"observation {key!r} has shape {array.shape}; obs_spec expects (1, {shape})"
                             + ("" if key == fmt.STATE_KEY else f" or a stacked (1, T, {shape})"))
        if array.dtype != dtype:
            if not np.can_cast(array.dtype, dtype, casting="safe"):
                raise ValueError(f"observation {key!r} has dtype {array.dtype}; obs_spec expects {dtype}")
            array = array.astype(dtype)
        return np.array(array, copy=True)

    def _named_field(self, key: str, value: Any) -> np.ndarray:
        shape = tuple(self._named_spec[key])
        array = _to_numpy(value)
        if array.size != int(np.prod(shape)):
            raise ValueError(f"named state {key!r} has {array.size} values; named_state_spec expects {list(shape)}")
        return array.astype(np.float32).reshape(shape)

    def _action_row(self, value: Any, name: str, dtype: type) -> np.ndarray:
        array = _to_numpy(value)
        if array.shape not in ((self._action_dim,), (1, self._action_dim)):
            raise ValueError(f"{name} has shape {array.shape}; expected ({self._action_dim},) or (1, {self._action_dim})")
        return array.reshape(self._action_dim).astype(dtype)

    def _data_arrays(self) -> dict[str, np.ndarray]:
        buffers = self._buffers
        steps = buffers.num_steps
        data: dict[str, np.ndarray] = {
            "obs_time_s": np.asarray(buffers.obs_time_s, dtype=np.float64),
            "stack_reset": np.asarray(buffers.stack_reset, dtype=bool),
        }
        if fmt.STATE_KEY in self._spec:
            data[fmt.STATE_KEY] = _stacked(buffers.obs[fmt.STATE_KEY], self._spec[fmt.STATE_KEY])
        for key, rows in buffers.named.items():
            data[fmt.NAMED_PREFIX + key] = np.stack(rows).astype(np.float32)
        step_dtypes = {"action": np.float32, "policy_action": np.float32, "human_action_mask": bool,
                       "override_mask": bool, "intervened": bool, "reward": np.float32,
                       "terminated": bool, "truncated": bool, "syncing": bool}
        for name, dtype in step_dtypes.items():
            rows = buffers.steps[name]
            shape = (steps, self._action_dim) if name in ("action", "policy_action", "human_action_mask",
                                                          "override_mask") else (steps,)
            data[name] = np.asarray(rows, dtype=dtype).reshape(shape)
        if buffers.grasp_penalty:
            data["grasp_penalty"] = np.asarray(buffers.grasp_penalty, dtype=np.float32)
        for name, flags in buffers.events.items():
            data[fmt.EVENT_PREFIX + name] = np.asarray(flags, dtype=bool)
        return data

    def _meta(self, data, status, discard_reason, success, events_list, operator_quit, extra) -> dict:
        intervened = data["intervened"]
        times = data["obs_time_s"]
        events = events_list if events_list is not None else [
            {"step": int(step), "name": name}
            for step in range(len(intervened))
            for name in sorted(self._buffers.events) if self._buffers.events[name][step]
        ]
        return fmt.json_normalize({
            "index": self.index,
            "episode_id": self.episode_id,
            "status": status,
            "discard_reason": discard_reason,
            "recorded_discard_reason": discard_reason,
            "success": None if success is None else bool(success),
            "steps": int(len(intervened)),
            "intervened_steps": int(intervened.sum()),
            "segments": segments_of(intervened),
            "events": list(events),
            "started_at": self.started_at,
            "duration_s": float(times[-1] - times[0]),
            "operator_quit": bool(operator_quit),
            "notes": [],
            "history": [],
            "videos": {},
            "extra": dict(extra or {}),
        })

    def _write_files(self, data: dict[str, np.ndarray], meta: dict) -> None:
        written = [self._tmp / fmt.DATA_FILE]
        np.savez_compressed(written[0], **data)
        frames_dir, video_dir = self._tmp / fmt.FRAMES_DIR, self._tmp / fmt.VIDEO_DIR
        keys = fmt.frame_keys(self._spec)
        if keys:
            frames_dir.mkdir()
        fps = float(self._dataset.header["control_hz"])
        for key in keys:
            frames = _stacked(self._buffers.obs[key], self._spec[key])
            path = frames_dir / f"{key}.npz"
            np.savez_compressed(path, **{fmt.FRAMES_ARRAY: frames})
            written.append(path)
            if fmt.is_rgb_key(key):
                video_dir.mkdir(exist_ok=True)
                video = video_dir / f"{key}.mp4"
                if encode_video(frames, video, fps=fps):
                    meta["videos"][key] = f"{fmt.VIDEO_DIR}/{key}.mp4"
                    written.append(video)
        meta_path = self._tmp / fmt.META_FILE
        fmt.write_json_atomic(meta_path, meta)
        for path in written:
            fmt.fsync_file(path)
        for directory in (frames_dir, video_dir):
            if directory.is_dir():
                fmt.fsync_dir(directory)
        fmt.fsync_dir(self._tmp)


def _stacked(rows: list[np.ndarray], entry: Mapping[str, Any]) -> np.ndarray:
    return np.stack(rows).astype(np.dtype(entry["dtype"]), copy=False).reshape(len(rows), *entry["shape"])


def _check_status(status: str, discard_reason: Optional[str]) -> None:
    if status == fmt.STATUS_KEPT:
        if discard_reason is not None:
            raise ValueError("a kept episode has no discard_reason")
    elif status == fmt.STATUS_DISCARDED:
        if discard_reason not in fmt.DISCARD_REASONS:
            raise ValueError(f"discard_reason must be one of {fmt.DISCARD_REASONS}, got {discard_reason!r}")
    else:
        raise ValueError(f"status must be {fmt.STATUS_KEPT!r} or {fmt.STATUS_DISCARDED!r}, got {status!r}")


def segments_of(intervened: np.ndarray) -> list[dict]:
    """Maximal runs ``[start, end)`` of equal ``intervened`` flags: kind ``intervene`` or ``hold``."""
    segments: list[dict] = []
    start = 0
    for step in range(1, len(intervened) + 1):
        if step == len(intervened) or intervened[step] != intervened[start]:
            segments.append({"start": start, "end": step,
                             "kind": "intervene" if intervened[start] else "hold"})
            start = step
    return segments
