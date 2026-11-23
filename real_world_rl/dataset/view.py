"""Review demo episodes in the Rerun viewer.

Each episode becomes its own Rerun recording (application ``rwrl-dataset``), so the viewer's
recording list lets you pick an episode named like ``episode_000007 · kept`` or
``episode_000003 · discarded (rerecord)``. Inside one episode you see:

- timelines ``step`` (observation row index) and ``time`` (seconds since the first
  observation);
- a camera view per ``rgb_*`` / ``depth_*`` key, synchronised to the timelines (the review mp4
  when present, else the lossless frames);
- plots of the joints, TCP position and orientation (roll/pitch/yaw), gripper, executed
  ``action`` against ``policy_action``, and the intervention flag with the reward;
- a text panel with the episode summary (status, notes, history) and a log of events and
  intervene start/end.

Bimanual named state (``<arm>.<field>``) is plotted under ``/<arm>/...``.

``rerun`` is imported lazily inside the functions, so importing this module needs neither
rerun nor torch.
"""
from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import TYPE_CHECKING, Any, Optional, Sequence

import numpy as np

if TYPE_CHECKING:
    import rerun as rr

APPLICATION_ID = "rwrl-dataset"
DEFAULT_GRPC_PORT = 9876
_SERVER_MEMORY_LIMIT = "4GiB"
_RPY_NAMES = ("roll", "pitch", "yaw")
_XYZ_NAMES = ("x", "y", "z")


# --------------------------------------------------------------------------------------
# Pure helpers (numpy only)
# --------------------------------------------------------------------------------------
def quat_xyzw_to_rpy(quat: np.ndarray) -> np.ndarray:
    """Roll, pitch, yaw (rad, extrinsic x-y-z, i.e. ``R = Rz(yaw) Ry(pitch) Rx(roll)``).

    ``quat`` is ``(..., 4)`` in xyzw order and need not be normalised. Pitch is clamped at
    +-pi/2 at gimbal lock.
    """
    q = np.asarray(quat, dtype=np.float64)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=-1)


def episode_title(meta: dict) -> str:
    """Recording name shown in the viewer, e.g. ``episode_000003 · discarded (rerecord)``."""
    title = f"episode_{int(meta['index']):06d} · {meta['status']}"
    if meta.get("discard_reason"):
        title += f" ({meta['discard_reason']})"
    return title


def recording_id_for(root: Path, index: int) -> str:
    """Stable recording id: dataset root directory name plus the episode index."""
    return f"{Path(root).resolve().name}_episode_{index:06d}"


def intervention_curve(segments: Sequence[dict], steps: int) -> np.ndarray:
    """``(steps,)`` float array, 1.0 over ``intervene`` segments and 0.0 elsewhere."""
    curve = np.zeros(steps, dtype=np.float64)
    for seg in segments:
        if seg.get("kind") == "intervene":
            curve[int(seg["start"]):int(seg["end"])] = 1.0
    return curve


def _split_named(key: str) -> tuple[str, str]:
    """``named.left.tcp_pose`` -> (``left``, ``tcp_pose``); ``named.tcp_pose`` -> (``""``, ...)."""
    arm, _, field = key[len("named."):].rpartition(".")
    return arm, field


def _base(arm: str) -> str:
    return f"/{arm}" if arm else ""


def _names(prefix: str, dim: int) -> list[str]:
    return [f"{prefix}{i}" for i in range(dim)]


def _scalar_series(data: Any) -> dict[str, np.ndarray]:
    """Map entity path -> ``(n,)`` curve for every scalar plotted from ``data.npz``.

    Observation-indexed curves have T+1 samples, step-indexed ones T.
    """
    series: dict[str, np.ndarray] = {}

    def add_dims(base: str, values: np.ndarray, names: Sequence[str]) -> None:
        values = np.asarray(values, dtype=np.float64).reshape(len(values), -1)
        for i in range(values.shape[1]):
            series[f"{base}/{names[i] if i < len(names) else f'd{i}'}"] = values[:, i]

    has_named = any(key.startswith("named.") for key in data)
    for key in sorted(k for k in data if k.startswith("named.")):
        arm, field = _split_named(key)
        base, values = _base(arm), np.asarray(data[key])
        if field == "joint_pos":
            add_dims(f"{base}/joint_pos", values, [f"q{i + 1}" for i in range(values.shape[1])])
        elif field == "tcp_pose" and values.shape[1] >= 7:
            add_dims(f"{base}/tcp_xyz", values[:, :3], _XYZ_NAMES)
            add_dims(f"{base}/tcp_rpy", quat_xyzw_to_rpy(values[:, 3:7]), _RPY_NAMES)
        elif field == "gripper_pos":
            add_dims(f"{base}/gripper_pos", values, _names("g", values.shape[1]))
    if not has_named and "state" in data:
        state = np.asarray(data["state"])
        add_dims("/state", state, _names("s", state.shape[1]))
    for key in ("action", "policy_action"):
        if key in data:
            act = np.asarray(data[key])
            add_dims(f"/{key}", act, _names("a", act.shape[1]))
    if "intervened" in data:
        series["/intervened"] = np.asarray(data["intervened"], dtype=np.float64)
    if "reward" in data:
        series["/reward"] = np.asarray(data["reward"], dtype=np.float64)
    return series


def _time_axis(data: Any, rows: int, control_hz: float) -> np.ndarray:
    """Seconds since the first observation for ``rows`` rows (control period as fallback)."""
    times = data.get("obs_time_s") if hasattr(data, "get") else None
    if times is not None and len(times) >= rows:
        times = np.asarray(times, dtype=np.float64)
        return times[:rows] - times[0]
    return np.arange(rows, dtype=np.float64) / control_hz


def episode_document(meta: dict) -> str:
    """Markdown shown in the episode text panel."""
    lines = [f"# {episode_title(meta)}", "",
             f"- status: {meta['status']}",
             f"- discard_reason: {meta.get('discard_reason')}",
             f"- success: {meta.get('success')}",
             f"- steps: {meta.get('steps')}",
             f"- intervened_steps: {meta.get('intervened_steps')}",
             f"- duration_s: {meta.get('duration_s')}",
             f"- started_at: {meta.get('started_at')}", "", "## Notes", ""]
    lines += [f"- {n.get('at')}: {n.get('text')}" for n in meta.get("notes") or []] or ["- none"]
    lines += ["", "## History", ""]
    lines += [f"- {h.get('at')}: {h.get('action')}" + (f" ({h['note']})" if h.get("note") else "")
              for h in meta.get("history") or []] or ["- none"]
    return "\n".join(lines)


# --------------------------------------------------------------------------------------
# Logging one episode
# --------------------------------------------------------------------------------------
def _image_keys(dataset: Any) -> tuple[list[str], list[str]]:
    spec = dataset.header.get("obs_spec", {})
    return ([k for k in spec if k.startswith("rgb_")], [k for k in spec if k.startswith("depth_")])


def _log_scalars(rec: "rr.RecordingStream", series: dict[str, np.ndarray], seconds: np.ndarray) -> None:
    import rerun as rr

    for path, values in series.items():
        n = len(values)
        rec.send_columns(
            path,
            indexes=[rr.TimeColumn("step", sequence=np.arange(n)),
                     rr.TimeColumn("time", duration=seconds[:n])],
            columns=rr.Scalars.columns(scalars=values),
        )


def _log_cameras(rec: "rr.RecordingStream", dataset: Any, episode: Any, seconds: np.ndarray) -> None:
    import rerun as rr

    hz = float(dataset.header["control_hz"])
    rgb_keys, depth_keys = _image_keys(dataset)
    n = len(seconds)
    indexes = [rr.TimeColumn("step", sequence=np.arange(n)), rr.TimeColumn("time", duration=seconds)]
    for key in rgb_keys:
        path, video = f"/camera/{key}", episode.video_path(key)
        if video is not None:
            rec.log(path, rr.AssetVideo(path=video, media_type="video/mp4"), static=True)
            nanos = np.round(np.arange(n) / hz * 1e9).astype(np.int64)
            rec.send_columns(path, indexes=indexes,
                             columns=rr.VideoFrameReference.columns(timestamp=nanos))
        else:
            _log_frames(rec, path, _frames_or_none(episode, key), seconds, depth=False)
    for key in depth_keys:
        _log_frames(rec, f"/camera/{key}", _frames_or_none(episode, key), seconds, depth=True)


def _frames_or_none(episode: Any, key: str) -> Optional[np.ndarray]:
    """The episode's lossless frames, or ``None`` when it recorded none for ``key``."""
    try:
        return episode.frames(key)
    except (FileNotFoundError, KeyError):
        return None


def _log_frames(rec: "rr.RecordingStream", path: str, frames: Optional[np.ndarray],
                seconds: np.ndarray, *, depth: bool) -> None:
    import rerun as rr

    if frames is None:
        return
    for row, frame in enumerate(frames):
        rec.set_time("step", sequence=row)
        rec.set_time("time", duration=float(seconds[min(row, len(seconds) - 1)]))
        if depth:
            rec.log(path, rr.DepthImage(frame[..., 0] if frame.ndim == 3 else frame, meter=1.0))
        else:
            rec.log(path, rr.Image(frame[..., 0] if frame.shape[-1] == 1 else frame))


def _log_text(rec: "rr.RecordingStream", path: str, step: int, seconds: np.ndarray, text: str) -> None:
    import rerun as rr

    rec.set_time("step", sequence=step)
    rec.set_time("time", duration=float(seconds[min(step, len(seconds) - 1)]))
    rec.log(path, rr.TextLog(text, level=rr.TextLogLevel.INFO))


def _log_annotations(rec: "rr.RecordingStream", meta: dict, steps: int, seconds: np.ndarray) -> None:
    import rerun as rr

    rec.log("/episode", rr.TextDocument(episode_document(meta), media_type="text/markdown"),
            static=True)
    segments = meta.get("segments") or []
    _log_scalars(rec, {"/intervention": intervention_curve(segments, steps)}, seconds)
    for seg in segments:
        if seg.get("kind") == "intervene":
            _log_text(rec, "/log/intervention", int(seg["start"]), seconds, "intervene start")
            _log_text(rec, "/log/intervention", int(seg["end"]), seconds, "intervene end")
    for event in meta.get("events") or []:
        _log_text(rec, "/log/events", int(event["step"]), seconds, str(event["name"]))


def log_episode(rec: "rr.RecordingStream", dataset: Any, episode: Any) -> None:
    """Log one episode (cameras, curves, annotations) into ``rec``.

    Does not set a blueprint; ``view_dataset`` attaches ``build_blueprint(dataset)``.
    """
    import rerun as rr

    meta, data = episode.meta, episode.data
    steps = int(meta["steps"])
    hz = float(dataset.header["control_hz"])
    rows = len(data["obs_time_s"]) if "obs_time_s" in data else steps + 1
    seconds = _time_axis(data, rows, hz)
    rr.send_recording_name(episode_title(meta), recording=rec)
    _log_cameras(rec, dataset, episode, seconds)
    _log_scalars(rec, _scalar_series(data), seconds)
    _log_annotations(rec, meta, steps, seconds)


# --------------------------------------------------------------------------------------
# Blueprint
# --------------------------------------------------------------------------------------
def _plot_groups(header: dict) -> list[tuple[str, list[str]]]:
    """(view title, entity-path content filters) for each time-series panel the dataset has."""
    spec = header.get("named_state_spec") or {}
    arms = sorted({k.rpartition(".")[0] for k in spec})
    groups: list[tuple[str, list[str]]] = []

    def per_arm(title: str, field: str, sub: str) -> None:
        if any(k.rpartition(".")[2] == field for k in spec):
            for arm in arms:
                groups.append((f"{arm} {title}".strip(), [f"{_base(arm)}/{sub}/**"]))

    per_arm("joints", "joint_pos", "joint_pos")
    per_arm("TCP xyz (m)", "tcp_pose", "tcp_xyz")
    per_arm("TCP rpy (rad)", "tcp_pose", "tcp_rpy")
    per_arm("gripper", "gripper_pos", "gripper_pos")
    if not spec:
        groups.append(("state", ["/state/**"]))
    groups.append(("action vs policy_action", ["/action/**", "/policy_action/**"]))
    groups.append(("intervention and reward", ["/intervention", "/intervened", "/reward"]))
    return groups


def build_blueprint(dataset: Any) -> "rr.blueprint.Blueprint":
    """Cameras on top (if any), plots in a grid below, episode text and event log at the bottom."""
    import rerun.blueprint as rrb

    rgb_keys, depth_keys = _image_keys(dataset)
    cameras = [rrb.Spatial2DView(origin=f"/camera/{key}", name=key) for key in rgb_keys + depth_keys]
    plots = [rrb.TimeSeriesView(origin="/", contents=contents, name=title)
             for title, contents in _plot_groups(dataset.header)]
    text = rrb.Horizontal(rrb.TextDocumentView(origin="/episode", name="episode"),
                          rrb.TextLogView(origin="/log", name="events"), column_shares=[1, 1])
    rows: list[Any] = []
    if cameras:
        rows.append(rrb.Horizontal(*cameras))
    rows += [rrb.Grid(*plots), text]
    shares = ([3] if cameras else []) + [5, 2]
    return rrb.Blueprint(rrb.Vertical(*rows, row_shares=shares), collapse_panels=False)


# --------------------------------------------------------------------------------------
# Viewing
# --------------------------------------------------------------------------------------
def _selected(dataset: Any, episodes: Optional[Sequence[int]]) -> list[int]:
    available = [info.index for info in dataset.episodes()]
    if episodes is None:
        return available
    missing = sorted(set(episodes) - set(available))
    if missing:
        raise ValueError(f"episodes not in dataset: {missing}")
    return sorted(set(episodes))


def _new_stream(dataset: Any, index: int) -> "rr.RecordingStream":
    import rerun as rr

    return rr.RecordingStream(APPLICATION_ID, recording_id=recording_id_for(dataset.root, index))


def _log_all(dataset: Any, indices: Sequence[int], attach: Any) -> list["rr.RecordingStream"]:
    """Create one stream per episode, ``attach(rec, i)`` its sink, log it, flush."""
    streams = []
    for i, index in enumerate(indices):
        rec = _new_stream(dataset, index)
        attach(rec, i)
        log_episode(rec, dataset, dataset.episode(index))
        rec.flush()
        streams.append(rec)
    return streams


def _save_all(dataset: Any, indices: Sequence[int], blueprint: Any, path: Path) -> None:
    """Write every episode recording into one rrd (per-episode files merged by ``rerun rrd merge``)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="rwrl-rrd-") as tmp:
        parts = [Path(tmp) / f"{index:06d}.rrd" for index in indices]
        streams = _log_all(dataset, indices, lambda rec, i: rec.save(
            parts[i], default_blueprint=blueprint))
        for rec in streams:
            rec.disconnect()
        # The rerun CLI (shipped with the SDK) merges stores without folding them into one.
        subprocess.run([sys.executable, "-m", "rerun", "rrd", "merge", *map(str, parts),
                        "-o", str(path)], check=True, capture_output=True)


def view_dataset(root: Any, *, episodes: Optional[Sequence[int]] = None, mode: str = "spawn",
                 save_path: Optional[Any] = None, web_port: Optional[int] = None,
                 grpc_port: Optional[int] = None) -> Optional[str]:
    """Open the dataset's episodes in Rerun; pick an episode in the viewer's recording list.

    ``mode``:

    - ``"spawn"``: start (or reuse) a native viewer on ``grpc_port`` (default 9876) and stream
      every episode to it; returns after logging, the viewer keeps running.
    - ``"web"``: serve gRPC on ``grpc_port`` and the web viewer on ``web_port`` (defaults
      9876/9090), print the URL and block until Ctrl-C. Returns the viewer URL.
    - ``"save"``: write all episodes into ``save_path`` (an ``.rrd`` file) without any viewer.

    ``episodes`` selects episode indices (default all, sorted).
    """
    import rerun as rr

    from real_world_rl.dataset.reader import Dataset

    if mode not in ("spawn", "web", "save"):
        raise ValueError(f"mode must be spawn, web or save, got {mode!r}")
    dataset = Dataset(root)
    indices = _selected(dataset, episodes)
    blueprint = build_blueprint(dataset)
    if mode == "save":
        if save_path is None:
            raise ValueError("mode='save' needs save_path")
        _save_all(dataset, indices, blueprint, Path(save_path))
        return None
    port = grpc_port or DEFAULT_GRPC_PORT
    if mode == "spawn":
        rr.spawn(port=port, connect=False)
        url = f"rerun+http://127.0.0.1:{port}/proxy"
        _log_all(dataset, indices, lambda rec, i: rec.connect_grpc(url, default_blueprint=blueprint))
        return None
    return _serve_web(dataset, indices, blueprint, port, web_port)


def _serve_web(dataset: Any, indices: Sequence[int], blueprint: Any, grpc_port: int,
               web_port: Optional[int]) -> str:
    import rerun as rr

    uris: list[str] = []

    def attach(rec: "rr.RecordingStream", i: int) -> None:
        if i == 0:
            uris.append(rec.serve_grpc(grpc_port=grpc_port, default_blueprint=blueprint,
                                       server_memory_limit=_SERVER_MEMORY_LIMIT))
        else:
            rec.connect_grpc(uris[0], default_blueprint=blueprint)

    streams = _log_all(dataset, indices, attach)  # noqa: F841  (kept alive: they own the server)
    port = web_port or 9090
    rr.serve_web_viewer(web_port=port, open_browser=False, connect_to=uris[0])
    url = f"http://localhost:{port}/?url={uris[0]}"
    print(f"Rerun web viewer: {url}\nCtrl-C to stop.", flush=True)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    return url
