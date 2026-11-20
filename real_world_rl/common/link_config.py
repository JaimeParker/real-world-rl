"""Station and CLI configuration for read-only daemon link diagnostics."""
from __future__ import annotations

import ipaddress
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, field

DEFAULT_REMOTE_BIND = "127.0.0.1:50051"
_HOSTNAME = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.-]*")


def endpoint(value: str) -> tuple[str, int]:
    """Parse host:port, using brackets for IPv6 literals."""
    match = re.fullmatch(r"(\[[^\]]+\]|[^:]+):([0-9]{1,5})", value)
    if match:
        host, port_text = match.groups()
        port = int(port_text)
        if 0 < port <= 65535:
            if _HOSTNAME.fullmatch(host):
                return host, port
            if host.startswith("[") and re.fullmatch(r"[0-9a-fA-F:.]+", host[1:-1]):
                try:
                    return str(ipaddress.IPv6Address(host[1:-1])), port
                except ValueError:
                    pass
    raise ValueError("expected HOST:PORT (use brackets for IPv6), with port 1–65535")


def host_port(host: str, port: int) -> str:
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}"


@dataclass(frozen=True)
class LinkConfig:
    address: str
    ssh: str | None = None
    remote_bind: str = DEFAULT_REMOTE_BIND
    policy: str | None = None
    token: str | None = field(default=None, repr=False)
    control_hz: float | None = None


def validate_link(values: object) -> LinkConfig:
    if not isinstance(values, Mapping):
        raise ValueError("link must be a mapping")
    if values.keys() - LinkConfig.__dataclass_fields__.keys():
        raise ValueError("unknown link key; expected address, ssh, remote_bind, policy, token, control_hz")
    if "address" not in values:
        raise ValueError("link.address is required")
    for key in ("address", "remote_bind", "policy"):
        if key not in values:
            continue
        value = values[key]
        if not isinstance(value, str):
            raise ValueError(f"link.{key} must be a HOST:PORT string")
        try:
            endpoint(value)
        except ValueError as error:
            raise ValueError(f"link.{key}: {error}") from None
    if "ssh" in values:
        ssh = values["ssh"]
        # USER@HOST or HOST, where HOST may be an alias from ~/.ssh/config.
        user, separator, host = ssh.rpartition("@") if isinstance(ssh, str) else ("", "@", "")
        if separator and not _HOSTNAME.fullmatch(user):
            raise ValueError("link.ssh must be USER@HOST or an ssh config host alias")
        if not _HOSTNAME.fullmatch(host):
            try:
                if not re.fullmatch(r"[0-9a-fA-F:.]+", host):
                    raise ValueError
                ipaddress.IPv6Address(host)
            except ValueError:
                raise ValueError("link.ssh must be USER@HOST or an ssh config host alias") from None
    if "token" in values and not isinstance(values["token"], str):
        raise ValueError("link.token must be a string")
    if "control_hz" in values:
        rate = values["control_hz"]
        if type(rate) not in (int, float) or not 0 < rate < math.inf:
            raise ValueError("link.control_hz must be finite and positive")
    return LinkConfig(**values)


def link_overrides(args, configured: LinkConfig | None) -> LinkConfig | None:
    overrides = {
        field_name: value
        for field_name, flag in (("address", "link"), ("ssh", "ssh"), ("policy", "policy"), ("token", "token"), ("remote_bind", "remote_bind"))
        if (value := getattr(args, flag, None)) is not None
    }
    if configured is None and not overrides:
        return None
    if configured is not None:
        values = {
            key: value for key, value in vars(configured).items() if value is not None
        }
        return validate_link({**values, **overrides})
    if "address" not in overrides:
        raise ValueError("--ssh, --remote-bind, --policy and --token require --link or a station link section")
    return validate_link(overrides)
