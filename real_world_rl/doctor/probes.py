"""Read-only host probes for the doctor's OS layer: session groups, USB-serial sysfs, brltty.
Module-level paths and functions so tests can point them at a fake tree; none changes anything."""
from __future__ import annotations

import getpass
import os
import shutil
from pathlib import Path

USB_SERIAL_SYSFS = Path("/sys/bus/usb-serial/devices")
USB_DEVICES_SYSFS = Path("/sys/bus/usb/devices")
SYSTEMD_OVERRIDE_DIRS = (Path("/etc/systemd/system"), Path("/run/systemd/system"))
BRLTTY_UDEV_UNIT = "brltty-udev.service"
CH340_VENDOR_ID = "1a86"


def current_user() -> str:
    return getpass.getuser()


def session_group_names() -> set[str]:
    """Groups of this process; a group added with ``usermod`` shows up only after a new login."""
    import grp

    names = set()
    for gid in {os.getgid(), *os.getgroups()}:
        try:
            names.add(grp.getgrgid(gid).gr_name)
        except KeyError:
            continue
    return names


def listed_group_members(group: str) -> set[str]:
    """Users named in ``/etc/group`` for ``group`` (empty when the group does not exist)."""
    import grp

    try:
        return set(grp.getgrnam(group).gr_mem)
    except KeyError:
        return set()


def serial_tty(port: str) -> str | None:
    """``ttyUSB0`` for ``/dev/serial/by-id/...`` or ``/dev/ttyUSB0``; None when it does not resolve to a tty."""
    name = os.path.basename(os.path.realpath(port))
    return name if name.startswith("tty") else None


def usb_serial_driver(tty: str) -> str | None:
    """The usb-serial driver of ``tty`` (``ftdi_sio``, ``ch341-uart``, ...); None for non-usb-serial ttys."""
    try:
        return os.path.basename(os.readlink(USB_SERIAL_SYSFS / tty / "driver"))
    except OSError:
        return None


def latency_timer_ms(tty: str) -> int | None:
    """The ``latency_timer`` of ``tty``; None when the driver has none (only ftdi_sio exposes it)."""
    try:
        return int((USB_SERIAL_SYSFS / tty / "latency_timer").read_text().strip())
    except (OSError, ValueError):
        return None


def usb_vendor_ids() -> set[str]:
    vendors = set()
    for path in USB_DEVICES_SYSFS.glob("*/idVendor"):
        try:
            vendors.add(path.read_text().strip().lower())
        except OSError:
            continue
    return vendors


def brltty_installed() -> bool:
    return shutil.which("brltty") is not None


def brltty_udev_masked() -> bool:
    return any(os.path.realpath(directory / BRLTTY_UDEV_UNIT) == os.devnull for directory in SYSTEMD_OVERRIDE_DIRS)
