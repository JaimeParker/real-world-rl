"""``rwrl teleop probe-hid``: read-only evidence for a SpaceMouse-like HID device and a draft entry
for the site device table (``spacemouse-devices.yaml``, see ``hid.parse_device_table``).

The probe lists the 3Dconnexion HID interfaces and the Logitech multi-axis-controller ones,
reads the selected interface's report descriptor (hidapi, else sysfs) and its raw input reports
for a while, and writes ``descriptor.hex``, ``reports.jsonl`` and ``candidate.yaml`` into a new
directory. It sends no feature or output report and never writes the device table. A Logitech
interface that is not a multi-axis controller is never opened: it may be a keyboard receiver.
The serial number is never recorded."""
from __future__ import annotations

import datetime
import errno
import json
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from real_world_rl.devices.errors import DeviceBusy, DeviceError, DeviceNotFound
from real_world_rl.teleop.spacemouse import hid as _hid

PROBES_DIRECTORY = "teleop/probes"  # under the station directory
DEFAULT_DURATION_S = 10.0
MAX_DURATION_S = 300.0
MAX_REPORTS = 10_000
READ_SIZE = 64
POLL_INTERVAL_S = 0.001
DESCRIPTOR_FILE, REPORTS_FILE, CANDIDATE_FILE = "descriptor.hex", "reports.jsonl", "candidate.yaml"
TODO = "TODO"
_HID_ID_TEXT = re.compile(r"[0-9a-fA-F]{4}:[0-9a-fA-F]{4}")
_GENERIC_DESKTOP_PAGE, _BUTTON_PAGE, _MULTI_AXIS_USAGE = 0x01, 0x09, 0x08
# Generic Desktop X..Rz under the device table's axis names: the built-in layouts read Rx as
# pitch and Ry as roll.
AXIS_USAGES = {0x30: "x", 0x31: "y", 0x32: "z", 0x33: "pitch", 0x34: "roll", 0x35: "yaw"}


def _path_text(path: Any) -> str:
    return path.decode(errors="replace") if isinstance(path, bytes) else str(path)


def parse_hid_id(text: str) -> tuple[int, int]:
    """``(vendor_id, product_id)`` of ``"vvvv:pppp"`` (hex)."""
    if not isinstance(text, str) or not _HID_ID_TEXT.fullmatch(text):
        raise ValueError(f"--device must be VID:PID in hex, e.g. 256f:c63a; got {text!r}")
    vendor, product = text.split(":")
    return int(vendor, 16), int(product, 16)


@dataclass(frozen=True)
class HidInterface:
    """One HID interface as hidapi enumerates it (the serial number left out)."""

    vendor_id: int
    product_id: int
    path: Any
    product: str = ""
    manufacturer: str = ""
    interface_number: int | None = None
    usage_page: int | None = None
    usage: int | None = None
    multi_axis: bool = False

    @property
    def hid_id(self) -> str:
        return _hid.format_hid_id((self.vendor_id, self.product_id))

    def public(self) -> dict:
        return {"hid_id": self.hid_id, "product": self.product, "manufacturer": self.manufacturer,
                "interface_number": self.interface_number, "usage_page": self.usage_page,
                "usage": self.usage, "multi_axis": self.multi_axis, "path": _path_text(self.path)}


class AmbiguousDevice(LookupError):
    """More than one candidate device and no ``--device``."""

    def __init__(self, hid_ids: Sequence[str], candidates: Sequence[Mapping[str, Any]]) -> None:
        super().__init__(f"{len(hid_ids)} candidate HID devices ({', '.join(hid_ids)}); pass --device VID:PID")
        self.hid_ids = list(hid_ids)
        self.candidates = [dict(candidate) for candidate in candidates]


def sysfs_descriptor(path: Any, sysfs: str | Path = "/sys") -> bytes | None:
    """The report descriptor of a hidraw path (``/dev/hidrawN``) from sysfs; None for another
    kind of path or when it cannot be read."""
    name = Path(_path_text(path)).name
    if not name.startswith("hidraw"):
        return None
    try:
        return (Path(sysfs) / "class/hidraw" / name / "device/report_descriptor").read_bytes() or None
    except OSError:
        return None


def candidate_interfaces(sdk: Any, *, sysfs: str | Path = "/sys") -> list[HidInterface]:
    """Every 3Dconnexion HID interface and every Logitech one that is a multi-axis controller
    (by its hidapi usage, else its sysfs descriptor), sorted by id, interface and path."""
    found = []
    for info in sdk.enumerate():
        vendor_id, product_id = info.get("vendor_id"), info.get("product_id")
        if type(vendor_id) is not int or type(product_id) is not int:
            continue
        if vendor_id not in (_hid.THREEDCONNEXION_VENDOR_ID, _hid.LOGITECH_VENDOR_ID):
            continue
        path = info.get("path", b"")
        usage_page, usage = info.get("usage_page"), info.get("usage")
        descriptor = sysfs_descriptor(path, sysfs)
        multi_axis = (usage_page, usage) == (_GENERIC_DESKTOP_PAGE, _MULTI_AXIS_USAGE) or (
            descriptor is not None and _hid.describes_multi_axis_controller(descriptor))
        if vendor_id == _hid.LOGITECH_VENDOR_ID and not multi_axis:
            continue
        interface_number = info.get("interface_number")
        found.append(HidInterface(
            vendor_id=vendor_id, product_id=product_id, path=path,
            product=str(info.get("product_string") or ""), manufacturer=str(info.get("manufacturer_string") or ""),
            interface_number=interface_number if type(interface_number) is int else None,
            usage_page=usage_page if type(usage_page) is int else None,
            usage=usage if type(usage) is int else None, multi_axis=multi_axis))
    return sorted(found, key=lambda item: (item.vendor_id, item.product_id,
                                           -1 if item.interface_number is None else item.interface_number,
                                           _path_text(item.path)))


def select_interface(interfaces: Sequence[HidInterface], hid_id: tuple[int, int] | None = None) -> HidInterface:
    """The interface to probe: of device ``hid_id``, else of the only candidate device; its
    multi-axis interface when it has one."""
    if hid_id is not None:
        matching = [item for item in interfaces if (item.vendor_id, item.product_id) == tuple(hid_id)]
        if not matching:
            raise DeviceNotFound(
                f"no 3Dconnexion or Logitech multi-axis HID device {_hid.format_hid_id(hid_id)} attached",
                hint="run rwrl teleop probe-hid without --device to list the candidates; a Logitech "
                     "device is probed only when it is a multi-axis controller")
    else:
        hid_ids = sorted({item.hid_id for item in interfaces})
        if not hid_ids:
            raise DeviceNotFound("no 3Dconnexion or Logitech multi-axis HID device attached",
                                 hint=f"plug it in; {_hid.UDEV_HINT}")
        if len(hid_ids) > 1:
            raise AmbiguousDevice(hid_ids, [item.public() for item in interfaces])
        matching = list(interfaces)
    return next((item for item in matching if item.multi_axis), matching[0])


def read_descriptor(handle: Any, interface: HidInterface, *, sysfs: str | Path = "/sys") -> tuple[bytes | None, str | None]:
    """``(descriptor, "hidapi" | "sysfs")``, or ``(None, None)`` when neither gives one."""
    reader = getattr(handle, "get_report_descriptor", None)
    if callable(reader):
        try:
            descriptor = bytes(reader())
        except (OSError, ValueError, RuntimeError, TypeError):
            descriptor = b""
        if descriptor:
            return descriptor, "hidapi"
    descriptor = sysfs_descriptor(interface.path, sysfs)
    return (descriptor, "sysfs") if descriptor else (None, None)


def summarize_descriptor(descriptor: bytes | Sequence[int]) -> dict[int, dict]:
    """The input reports a HID report descriptor declares, by report id (0: no report ids).

    Each has ``bytes`` (the data bytes after the report id), ``axes`` (each 16-bit Generic
    Desktop X..Rz field: ``name`` and ``byte1``/``byte2``, its low and high byte counted as the
    device table counts them, the report id at 0) and ``buttons`` (each Button-page bit:
    ``byte``, ``bit``). Short items (HID 1.11, 6.2.2); long items are skipped."""
    data = bytes(descriptor)
    usage_page = report_size = report_count = report_id = 0
    stack: list[tuple[int, int, int, int]] = []
    usages: list[tuple[int, int]] = []
    usage_minimum = usage_maximum = None
    bit_offsets: dict[int, int] = {}
    reports: dict[int, dict] = {}
    index = 0
    while index < len(data):
        prefix = data[index]
        if prefix == 0xFE:  # long item: data size, tag, data
            index += 3 + (data[index + 1] if index + 1 < len(data) else 0)
            continue
        size = (0, 1, 2, 4)[prefix & 0x03]
        value = int.from_bytes(data[index + 1:index + 1 + size], "little")
        index += 1 + size
        item_type, tag = (prefix >> 2) & 0x03, prefix >> 4
        if item_type == 1:  # global
            if tag == 0x0:
                usage_page = value
            elif tag == 0x7:
                report_size = value
            elif tag == 0x8:
                report_id = value
            elif tag == 0x9:
                report_count = value
            elif tag == 0xA:
                stack.append((usage_page, report_size, report_count, report_id))
            elif tag == 0xB and stack:
                usage_page, report_size, report_count, report_id = stack.pop()
        elif item_type == 2:  # local
            if tag == 0x0:
                usages.append((value >> 16, value & 0xFFFF) if size == 4 else (usage_page, value))
            elif tag == 0x1:
                usage_minimum = value
            elif tag == 0x2:
                usage_maximum = value
        elif item_type == 0:  # main
            if tag == 0x8:  # Input
                report = reports.setdefault(report_id, {"bytes": 0, "axes": [], "buttons": []})
                offset = bit_offsets.get(report_id, 0)
                fields = list(usages)
                if not fields and usage_minimum is not None and usage_maximum is not None:
                    fields = [(usage_page, usage) for usage in range(usage_minimum, usage_maximum + 1)]
                if not value & 0x01:  # Data, not Constant (padding)
                    for field in range(report_count):
                        page, usage = fields[min(field, len(fields) - 1)] if fields else (usage_page, None)
                        start = offset + field * report_size
                        byte = start // 8 + (1 if report_id else 0)
                        if (page == _GENERIC_DESKTOP_PAGE and usage in AXIS_USAGES and report_size == 16
                                and start % 8 == 0):
                            report["axes"].append({"name": AXIS_USAGES[usage], "byte1": byte, "byte2": byte + 1})
                        elif page == _BUTTON_PAGE and report_size == 1:
                            report["buttons"].append({"byte": byte, "bit": start % 8})
                offset += report_count * report_size
                bit_offsets[report_id] = offset
                report["bytes"] = (offset + 7) // 8
            usages, usage_minimum, usage_maximum = [], None, None
    return reports


def capture_reports(handle: Any, *, duration_s: float, clock: Callable[[], float] = time.monotonic,
                    sleep: Callable[[float], None] = time.sleep, limit: int = MAX_REPORTS) -> tuple[list[dict], int]:
    """The raw input reports read in ``duration_s`` (``{t, report_id, length, bytes_hex}``, ``t``
    in seconds from the start; ``report_id`` is the first byte) and how many past ``limit`` were
    dropped."""
    start = clock()
    reports: list[dict] = []
    dropped = 0
    while clock() - start < duration_s:
        report = handle.read(READ_SIZE)
        if not report:
            sleep(POLL_INTERVAL_S)
            continue
        if len(reports) >= limit:
            dropped += 1
            continue
        raw = bytes(report)
        reports.append({"t": round(clock() - start, 4), "report_id": raw[0], "length": len(raw),
                        "bytes_hex": raw.hex()})
    return reports, dropped


def summarize_reports(reports: Sequence[Mapping[str, Any]]) -> dict[int, dict]:
    """``{report id: {count, lengths}}`` of captured reports (``lengths`` include the id byte)."""
    summary: dict[int, dict] = {}
    for report in reports:
        entry = summary.setdefault(int(report["report_id"]), {"count": 0, "lengths": set()})
        entry["count"] += 1
        entry["lengths"].add(int(report["length"]))
    return {report_id: {"count": entry["count"], "lengths": sorted(entry["lengths"])}
            for report_id, entry in sorted(summary.items())}


def _spec_axes(spec: _hid.DeviceSpec) -> set[tuple[str, int, int, int]]:
    return {(name, axis.channel, axis.byte1, axis.byte2) for name, axis in spec.mappings.items()}


def _spec_buttons(spec: _hid.DeviceSpec) -> list[tuple[int, int, int]]:
    return [(button.channel, button.byte, button.bit) for button in spec.button_mapping if button.byte is not None]


def _report_lengths(positions) -> dict[int, int]:
    """``{report id: report length with the id byte}`` that reading ``positions`` needs."""
    lengths: dict[int, int] = {}
    for channel, last_byte in positions:
        lengths[channel] = max(lengths.get(channel, 0), last_byte + 1)
    return lengths


def guess_layout(descriptor_reports: Mapping[int, Mapping[str, Any]] | None,
                 observed: Mapping[int, Mapping[str, Any]]) -> tuple[str | None, list[str]]:
    """The built-in entry whose decoding fits the device, by report ids and lengths, and notes.

    With a descriptor, its 16-bit axis fields must sit exactly where the entry reads the six
    axes, and the entry's buttons among the descriptor's button bits (an entry with as many
    buttons is preferred). Without one, the captured axis and button reports must have the
    lengths the entry reads. None when no entry fits; the first fitting entry otherwise (the
    others are named in the notes)."""
    specs = list(_hid.BUILTIN_DEVICE_SPECS.values())
    notes: list[str] = []
    if descriptor_reports:
        axes = {(axis["name"], report_id, axis["byte1"], axis["byte2"])
                for report_id, report in descriptor_reports.items() for axis in report["axes"]}
        buttons = {(report_id, button["byte"], button["bit"])
                   for report_id, report in descriptor_reports.items() for button in report["buttons"]}
        fitting = [spec for spec in specs if _spec_axes(spec) == axes]
        within = [spec for spec in fitting if set(_spec_buttons(spec)) <= buttons]
        exact = [spec for spec in within if len(_spec_buttons(spec)) == len(buttons)]
        chosen = exact or within or fitting
        how = "the descriptor's axis fields and button bits"
        if fitting and not within:
            notes.append("no fitting built-in entry reads only the descriptor's button bits: check the buttons")
    else:
        lengths = {report_id: set(entry["lengths"]) for report_id, entry in observed.items()}

        def matches(spec: _hid.DeviceSpec) -> bool:
            axis_lengths = _report_lengths((axis.channel, axis.byte2) for axis in spec.mappings.values())
            return all(length in lengths.get(channel, ()) for channel, length in axis_lengths.items())

        def buttons_match(spec: _hid.DeviceSpec) -> bool:
            button_lengths = _report_lengths((channel, byte) for channel, byte, _ in _spec_buttons(spec))
            return all(length in lengths.get(channel, ()) for channel, length in button_lengths.items())

        fitting = [spec for spec in specs if matches(spec)]
        chosen = [spec for spec in fitting if buttons_match(spec)] or fitting
        how = "the captured report ids and lengths (no descriptor)"
        if fitting and chosen is fitting:
            notes.append("no button report matched a fitting entry: press every button during the capture")
    if not chosen:
        notes.append(f"no built-in layout fits {how}; candidate.yaml spells out the decoding with TODO fields")
        return None, notes
    notes.append(f"layout {chosen[0].name!r} matched by {how}")
    others = [spec.name for spec in chosen[1:]]
    if others:
        notes.append("also fits: " + ", ".join(others) + "; they differ in their buttons")
    return chosen[0].name, notes


def template_entry(descriptor_reports: Mapping[int, Mapping[str, Any]] | None) -> dict:
    """The full decoding of an entry without ``layout``: what the descriptor gives (axis report
    ids and bytes, the first two button bits) filled in, the rest ``TODO``."""
    mappings = {name: {"channel": TODO, "byte1": TODO, "byte2": TODO, "scale": TODO} for name in _hid.AXIS_NAMES}
    buttons = [{"channel": TODO, "byte": TODO, "bit": TODO} for _ in range(2)]
    if descriptor_reports:
        positions = []
        for report_id, report in sorted(descriptor_reports.items()):
            for axis in report["axes"]:
                mappings[axis["name"]].update(channel=report_id, byte1=axis["byte1"], byte2=axis["byte2"])
            positions += [(report_id, button) for button in report["buttons"]]
        for index, (report_id, button) in enumerate(positions[:len(buttons)]):
            buttons[index] = {"channel": report_id, "byte": button["byte"], "bit": button["bit"]}
    return {"axis_scale": 350.0, "led_id": None, "mappings": mappings, "button_mapping": buttons}


def _report_note(descriptor_reports, observed) -> str:
    if descriptor_reports:
        parts = [f"report {report_id}: {report['bytes']} data bytes, {len(report['axes'])} 16-bit axes, "
                 f"{len(report['buttons'])} button bits" for report_id, report in sorted(descriptor_reports.items())]
    else:
        parts = [f"report {report_id}: captured lengths {entry['lengths']}" for report_id, entry in observed.items()]
    return "; ".join(parts) or "no input report"


def candidate_entry(interface: HidInterface, *, layout: str | None, descriptor_reports, observed,
                    probe_reference: str, checked: str) -> tuple[str, dict]:
    """``(entry name, entry)`` for ``spacemouse-devices.yaml``; the name carries the id so it
    cannot reuse a built-in entry's name."""
    name = f"{interface.product or 'HID device'} {interface.hid_id}"
    entry: dict = {"hid_id": interface.hid_id}
    if layout is not None:
        entry["layout"] = layout
    else:
        entry.update(template_entry(descriptor_reports))
    entry["evidence"] = {"probe": probe_reference, "checked": checked,
                         "note": _report_note(descriptor_reports, observed)}
    return name, entry


def candidate_yaml(name: str, entry: Mapping[str, Any], *, interface: HidInterface, checked: str,
                   table_path: str) -> str:
    """``candidate.yaml``: a complete device table holding the one draft entry."""
    import yaml

    header = [f'# rwrl teleop probe-hid: {interface.hid_id} "{interface.product}", {checked} (UTC).']
    if "layout" in entry:
        header.append("# Draft: the layout was matched by report ids and lengths, not checked by moving the puck.")
    else:
        header += ["# Draft: no built-in layout fits. Replace every TODO (scale is 1 or -1; a missing button is",
                   "# {channel: null, byte: null, bit: null}); an entry with a TODO rejects the whole table."]
    header += [f"# Paste the entry under `devices:` of {table_path},",
               "# then run `rwrl doctor` and `rwrl teleop diagnose` to confirm the device and its axes."]
    document = {"schema_version": _hid.DEVICE_TABLE_SCHEMA_VERSION, "devices": {name: dict(entry)}}
    return "\n".join(header) + "\n" + yaml.safe_dump(document, sort_keys=False, allow_unicode=True)


def _probe_reference(directory: Path, station_dir: str | Path | None) -> str:
    """The probe directory as ``evidence.probe`` gives it: relative to the station directory
    (where the site table lives) when inside it, else absolute."""
    directory = directory.resolve()
    if station_dir is not None:
        try:
            return directory.relative_to(Path(station_dir).resolve()).as_posix() + "/"
        except ValueError:
            pass
    return directory.as_posix() + "/"


def probe_directory(interface: HidInterface, *, output: str | Path | None, station_dir: str | Path | None) -> Path:
    """``output``, else ``<station_dir>/teleop/probes/<vid>-<pid>``."""
    if output is not None:
        return Path(output)
    if station_dir is None:
        raise ValueError("a probe directory needs an output directory or a station directory")
    return Path(station_dir) / PROBES_DIRECTORY / f"{interface.vendor_id:04x}-{interface.product_id:04x}"


def run_probe(*, hid_id: tuple[int, int] | None = None, duration_s: float = DEFAULT_DURATION_S,
              output: str | Path | None = None, station_dir: str | Path | None = None,
              table: _hid.DeviceTable | None = None, echo: Callable[[str], None] = print, sdk: Any = None,
              sysfs: str | Path = "/sys", claims_dir=None, clock: Callable[[], float] = time.monotonic,
              sleep: Callable[[float], None] = time.sleep, today: datetime.date | None = None) -> dict:
    """Probe one device and write its probe directory; the result payload of ``probe-hid``.

    Raises ``AmbiguousDevice``, ``FileExistsError`` (the probe directory exists),
    ``DeviceNotFound``, ``DeviceBusy`` (another process owns hardware, or the device is claimed)
    and ``SdkMissing`` before reading anything."""
    sdk = _hid.require_sdk() if sdk is None else sdk
    interfaces = candidate_interfaces(sdk, sysfs=sysfs)
    interface = select_interface(interfaces, hid_id)
    directory = probe_directory(interface, output=output, station_dir=station_dir)
    if directory.exists():
        raise FileExistsError(errno.EEXIST, "probe directory exists", str(directory))
    from real_world_rl.common.device_ownership import process_owners

    owners = process_owners()
    if owners:
        raise DeviceBusy("exclusive HID probe blocked: " + "; ".join(owners),
                         hint="finish the owning session before probing")
    claim = _hid.device_claim(interface.path, claims_dir=claims_dir)
    claim.acquire()
    handle = None
    try:
        handle = sdk.device()
        handle.open_path(interface.path)
        handle.set_nonblocking(1)
        descriptor, source = read_descriptor(handle, interface, sysfs=sysfs)
        echo(f"probing {interface.hid_id}: move the puck along and around every axis and press every "
             f"button during the next {duration_s:g} s")
        reports, dropped = capture_reports(handle, duration_s=duration_s, clock=clock, sleep=sleep)
    except (OSError, RuntimeError) as error:
        if isinstance(error, DeviceError):
            raise
        raise DeviceNotFound(f"cannot read HID device {interface.hid_id}: {error}", hint=_hid.UDEV_HINT) from error
    finally:
        try:
            if handle is not None:
                handle.close()
        finally:
            claim.release()

    descriptor_reports = summarize_descriptor(descriptor) if descriptor is not None else None
    observed = summarize_reports(reports)
    layout, notes = guess_layout(descriptor_reports, observed)
    if not reports:
        notes.append("no input report arrived: move the puck during the capture")
    if dropped:
        notes.append(f"{dropped} reports after the first {MAX_REPORTS} were not kept")
    checked = (today or datetime.datetime.now(datetime.timezone.utc).date()).isoformat()
    name, entry = candidate_entry(interface, layout=layout, descriptor_reports=descriptor_reports, observed=observed,
                                  probe_reference=_probe_reference(directory, station_dir), checked=checked)
    table_path = (str(Path(station_dir) / _hid.DEVICE_TABLE_FILENAME) if station_dir is not None
                  else f"the site {_hid.DEVICE_TABLE_FILENAME}")
    known = None if table is None else table.specs.get((interface.vendor_id, interface.product_id))

    directory.parent.mkdir(parents=True, exist_ok=True)
    directory.mkdir(mode=0o700)
    files = {"descriptor": None, "reports": str(directory / REPORTS_FILE), "candidate": str(directory / CANDIDATE_FILE)}
    if descriptor is not None:
        files["descriptor"] = str(directory / DESCRIPTOR_FILE)
        (directory / DESCRIPTOR_FILE).write_text(descriptor.hex() + "\n")
    (directory / REPORTS_FILE).write_text("".join(json.dumps(report) + "\n" for report in reports))
    (directory / CANDIDATE_FILE).write_text(
        candidate_yaml(name, entry, interface=interface, checked=checked, table_path=table_path))
    return {
        "schema_version": 1,
        "candidates": [item.public() for item in interfaces],
        "selected": interface.public(),
        "known_as": None if known is None else known.name,
        "probe_directory": str(directory),
        "duration_s": duration_s,
        "descriptor": None if descriptor is None else {
            "source": source, "hex": descriptor.hex(),
            "reports": {str(report_id): report for report_id, report in sorted(descriptor_reports.items())}},
        "reports": {"count": len(reports), "dropped": dropped,
                    "by_id": {str(report_id): entry for report_id, entry in observed.items()}},
        "layout": layout,
        "notes": notes,
        "candidate": {name: entry},
        "files": files,
    }


def text_report(payload: Mapping[str, Any]) -> str:
    """The text form of a ``run_probe`` payload."""
    selected = payload["selected"]
    lines = [f'probed {selected["hid_id"]} "{selected["product"]}" (interface {selected["interface_number"]}, '
             f'{selected["path"]}) for {payload["duration_s"]:g} s: {payload["reports"]["count"]} reports']
    if payload["known_as"]:
        lines.append(f"already in the device table as {payload['known_as']!r}")
    descriptor = payload["descriptor"]
    if descriptor is None:
        lines.append("descriptor: unavailable (neither hidapi nor sysfs gave one)")
    else:
        lines.append(f"descriptor ({descriptor['source']}): " + "; ".join(
            f"report {report_id}: {report['bytes']} data bytes, {len(report['axes'])} axes, "
            f"{len(report['buttons'])} buttons" for report_id, report in descriptor["reports"].items()))
    lines.append(f"layout: {payload['layout'] or 'none fits; candidate.yaml has TODO fields'}")
    lines += [f"note: {note}" for note in payload["notes"]]
    lines.append("wrote " + ", ".join(path for path in payload["files"].values() if path))
    lines.append(f"next: paste the entry of {payload['files']['candidate']} into the site device table, "
                 "then run rwrl doctor and rwrl teleop diagnose")
    return "\n".join(lines)
