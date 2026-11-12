"""Camera inspection: configuration, measured frame rate and a one-shot frame grab."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path

from .command import Result
from .errors import CliError, FAILURE_EXIT_CODE, SUCCESS_EXIT_CODE, USAGE_EXIT_CODE

DEFAULT_SHOW_FRAMES = 30
DEFAULT_GRAB_TIMEOUT_S = 2.0
CONFIG_FIELDS = ("name", "serial", "width", "height", "fps", "enable_depth", "backend")
# Float depth (metres, as camera backends deliver it) is stored in a 16-bit PNG as millimetres.
DEPTH_PNG_SCALE_M = 0.001
UINT16_MAX = 65535
OVERWRITE_HINT = "choose another --output directory"
NOT_MEASURED = "not measured"


def camera_config(camera) -> dict:
    return {field: getattr(camera.config, field) for field in CONFIG_FIELDS}


def frame_source(connection) -> str:
    return "daemon" if connection.remote else "direct"


def selected_cameras(connection, name: str | None) -> dict:
    cameras = dict(connection.cameras)
    if name is None:
        return cameras
    if name not in cameras:
        raise CliError(f"no camera named {name!r}; available: {', '.join(cameras) or 'none'}",
                       code=USAGE_EXIT_CODE, hint="pass --name with one of the available cameras")
    return {name: cameras[name]}


def array_shape(array) -> list[int] | None:
    return None if array is None else [int(size) for size in array.shape]


def optional_float(value) -> float | None:
    return None if value is None else float(value)


def measured_fps(timestamps: list[float]) -> float | None:
    """Rate over distinct capture times, so a frame delivered twice does not inflate it."""
    distinct = sorted(set(timestamps))
    if len(distinct) < 2 or distinct[-1] <= distinct[0]:
        return None
    return (len(distinct) - 1) / (distinct[-1] - distinct[0])


def camera_report(camera, frames: int, source: str, device_fault: type) -> dict:
    entry = {**camera_config(camera), "source": source,
             "image_encoding": getattr(camera, "image_encoding", None)}
    timestamps = []
    frame = None
    try:
        for _ in range(frames):
            frame = camera.read()
            timestamps.append(float(frame.timestamp))
    except device_fault as error:
        entry["error"] = error.message
        entry["hint"] = error.hint
    entry.update(
        frames_read=len(timestamps),
        measured_fps=measured_fps(timestamps),
        color_shape=None if frame is None else array_shape(frame.color),
        depth_shape=None if frame is None else array_shape(frame.depth),
        dtype=None if frame is None else str(frame.color.dtype),
    )
    if frames == 0:
        # Without a read, health only reflects connect timing; it is not judged.
        entry.update(frame_age_s=None, ok=None, detail=NOT_MEASURED)
    else:
        health = camera.health()
        entry.update(frame_age_s=optional_float(health.last_feedback_age_s), ok=bool(health.ok),
                     detail=health.detail)
    return entry


def camera_failed(entry: dict) -> bool:
    return "error" in entry or entry["ok"] is False


def camera_line(entry: dict, measured: bool) -> str:
    if not measured:
        rate = NOT_MEASURED
    elif entry["measured_fps"] is None:
        rate = "measured n/a"
    else:
        rate = f"measured {entry['measured_fps']:.1f}"
    age = "n/a" if entry["frame_age_s"] is None else f"{entry['frame_age_s']:.3f} s"
    if entry["color_shape"] is None:
        images = "color n/a, depth n/a"
    else:
        depth = "none" if entry["depth_shape"] is None else str(tuple(entry["depth_shape"]))
        images = f"color {tuple(entry['color_shape'])} {entry['dtype']}, depth {depth}"
    if "error" in entry:
        state = f"error: {entry['error']}" + (f" (hint: {entry['hint']})" if entry["hint"] else "")
    elif entry["ok"] is None:
        state = NOT_MEASURED
    elif entry["ok"]:
        state = "ok"
    else:
        state = "not ok" + (f" ({entry['detail']})" if entry["detail"] else "")
    return (f"{entry['name']}: {entry['width']}x{entry['height']}@{entry['fps']} fps ({rate}), "
            f"age {age}, {images}, serial {entry['serial'] or 'auto'}, {state}")


def show_cameras(args, connection, device_fault: type) -> tuple[dict, str, int]:
    source = frame_source(connection)
    entries = [camera_report(camera, args.frames, source, device_fault)
               for camera in selected_cameras(connection, args.name).values()]
    lines = [f"connection: {connection.path}"]
    lines += [camera_line(entry, args.frames > 0) for entry in entries] or ["cameras: none"]
    failed = [entry["name"] for entry in entries if camera_failed(entry)]
    if failed:
        lines.append("failed cameras: " + ", ".join(failed))
    return {"cameras": entries}, "\n".join(lines), FAILURE_EXIT_CODE if failed else SUCCESS_EXIT_CODE


def grab_targets(output: Path, name: str) -> dict[str, Path]:
    return {"color": output / f"{name}-color.png", "depth": output / f"{name}-depth.png",
            "metadata": output / f"{name}.json"}


def refuse_existing_targets(output: Path, targets: dict[str, Path]) -> None:
    if output.exists() and not output.is_dir():
        raise CliError(f"--output {output} is not a directory", code=USAGE_EXIT_CODE, hint=OVERWRITE_HINT)
    existing = [str(path) for path in targets.values() if path.exists()]
    if existing:
        raise CliError("refusing to overwrite " + ", ".join(existing), code=USAGE_EXIT_CODE, hint=OVERWRITE_HINT)


def write_new_file(path: Path, write) -> None:
    try:
        with open(path, "xb") as stream:
            write(stream)
    except FileExistsError as error:
        raise CliError(f"refusing to overwrite {path}", code=USAGE_EXIT_CODE, hint=OVERWRITE_HINT) from error


def color_image(color):
    import numpy as np

    color = np.ascontiguousarray(color)
    if color.dtype != np.uint8 or color.ndim != 3 or color.shape[-1] != 3:
        raise CliError(f"color frame {color.shape} {color.dtype} is not HxWx3 uint8")
    return color


def depth_image(depth) -> tuple[object, str]:
    """A 16-bit array for the depth PNG and its unit: integer depth is kept as delivered, float
    depth in metres becomes millimetres (invalid or negative values become 0)."""
    import numpy as np

    depth = np.asarray(depth)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    if depth.ndim != 2:
        raise CliError(f"depth frame {depth.shape} is not HxW")
    if np.issubdtype(depth.dtype, np.integer):
        values, unit = np.clip(depth, 0, UINT16_MAX), "raw"
    else:
        millimetres = np.nan_to_num(depth.astype(np.float64) / DEPTH_PNG_SCALE_M, nan=0.0, posinf=0.0, neginf=0.0)
        values, unit = np.rint(np.clip(millimetres, 0, UINT16_MAX)), "mm"
    return np.ascontiguousarray(values.astype(np.uint16)), unit


def grab_frame(args, connection, station_path: Path, image_module, device_fault: type) -> tuple[dict, str]:
    """Success follows the frame and the files; camera health is only reported."""
    camera = selected_cameras(connection, args.name)[args.name]
    try:
        frame = camera.read(timeout_s=args.timeout_s)
    except device_fault as error:
        raise CliError(f"camera {args.name!r}: {error.message}", hint=error.hint) from error
    health = camera.health()
    color = color_image(frame.color)
    depth, depth_unit = (None, None) if frame.depth is None else depth_image(frame.depth)
    output = Path(args.output)
    targets = grab_targets(output, args.name)
    output.mkdir(parents=True, exist_ok=True)
    written = []

    def write_png(key, array):
        write_new_file(targets[key], lambda stream: image_module.fromarray(array).save(stream, format="PNG"))
        written.append(str(targets[key]))

    write_png("color", color)
    if depth is not None:
        write_png("depth", depth)
    metadata = {
        "camera": camera_config(camera),
        "source": frame_source(connection),
        "image_encoding": getattr(camera, "image_encoding", None),
        "path": connection.path,
        "station": str(station_path.absolute()),
        "robot": args.robot,
        "timestamp": float(frame.timestamp),
        "written_at": datetime.now(timezone.utc).isoformat(),
        "frame_age_s": optional_float(health.last_feedback_age_s),
        "ok": bool(health.ok),
        "detail": health.detail,
        "color_shape": array_shape(frame.color),
        "dtype": str(frame.color.dtype),
        "depth_shape": array_shape(frame.depth),
        "depth_dtype": None if frame.depth is None else str(frame.depth.dtype),
        "depth_png_unit": depth_unit,
        "depth_png_scale_m": DEPTH_PNG_SCALE_M if depth_unit == "mm" else None,
        "files": written + [str(targets["metadata"])],
    }
    write_new_file(targets["metadata"], lambda stream: stream.write(
        (json.dumps(metadata, indent=2, allow_nan=False) + "\n").encode("utf-8")))
    written.append(str(targets["metadata"]))
    lines = [f"connection: {connection.path}", f"camera {args.name}: wrote " + ", ".join(written)]
    if not health.ok:
        lines.append(f"warning: camera {args.name} health not ok" + (f": {health.detail}" if health.detail else ""))
    data = {"camera": args.name, "output": str(output), "files": written,
            "ok": bool(health.ok), "detail": health.detail}
    return data, "\n".join(lines)


class CameraCommand:
    name = "camera"
    help = "inspect cameras or save one frame"

    def register(self, parser):
        actions = parser.add_subparsers(dest="action", required=True)
        show = actions.add_parser("show", allow_abbrev=False,
                                  help="print camera configuration, measured frame rate and health")
        show.add_argument("--name", metavar="NAME", help="only this camera (default: every station camera)")
        show.add_argument("--frames", type=int, default=DEFAULT_SHOW_FRAMES, metavar="N",
                          help=f"frames to read for the measured rate (default {DEFAULT_SHOW_FRAMES}; 0 reads none)")
        grab = actions.add_parser("grab", allow_abbrev=False,
                                  help="save one frame as PNG files plus a JSON sidecar")
        grab.add_argument("--name", required=True, metavar="NAME", help="camera to read")
        grab.add_argument("--output", required=True, metavar="DIR",
                          help="directory for NAME-color.png, NAME-depth.png (with depth) and NAME.json; "
                               "created if missing; existing files are never overwritten")
        grab.add_argument("--timeout-s", type=float, default=DEFAULT_GRAB_TIMEOUT_S, metavar="S",
                          help=f"wait at most S seconds for a frame (default {DEFAULT_GRAB_TIMEOUT_S:g})")

    def run(self, args, ctx):
        from robot_infra.devices.errors import DeviceFault

        if args.action == "show":
            if args.frames < 0:
                raise CliError("camera show --frames N requires N >= 0", code=USAGE_EXIT_CODE)
            with ctx.open_robot() as connection:
                data, text, code = show_cameras(args, connection, DeviceFault)
                return Result({"command": "camera show", "path": connection.path, **data}, text, code)
        if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
            raise CliError("camera grab --timeout-s S requires S > 0", code=USAGE_EXIT_CODE)
        try:
            from PIL import Image
        except ImportError as error:
            raise CliError("camera grab needs Pillow to write PNG files", hint="pip install pillow") from error
        from robot_infra.discovery.autoconfig import resolve_station_config_path

        refuse_existing_targets(Path(args.output), grab_targets(Path(args.output), args.name))
        station_path = resolve_station_config_path(args.station)
        with ctx.open_robot() as connection:
            data, text = grab_frame(args, connection, station_path, Image, DeviceFault)
            return Result({"command": "camera grab", "path": connection.path, **data}, text)


CAMERA = CameraCommand()
