"""Robot identities shared by daemon and station clients, without device imports."""
import os


def robot_identity(config) -> dict:
    return {"name": config.name, "type": getattr(config, "TYPE", None),
            "address": getattr(config, "robot_ip", None) or
            getattr(config, "can_interface", None) or None}


def expected_identity(config, env=None) -> dict:
    env = os.environ if env is None else env
    identity = robot_identity(config)
    addresses = []
    for field in ("robot_ip", "can_interface"):
        value = getattr(config, field, None)
        if hasattr(config, field) and not value:
            value = env.get(field.upper())
        addresses.append(value)
    identity["address"] = addresses[0] or addresses[1] or None
    return identity


def identity_mismatches(expected, actual) -> list[str]:
    if not isinstance(actual, dict):
        return ["identity: not reported by the daemon (older version)"]
    return [f"{field}: station {expected.get(field)}, daemon {actual.get(field)}"
            for field in ("name", "type", "address")
            if not (field == "address" and expected.get(field) is None)
            and expected.get(field) != actual.get(field)]


def identity_error(address, mismatches, actual) -> tuple[str, str]:
    message = f"daemon at {address} identity mismatch: " + "; ".join(mismatches)
    hint = ("restart the daemon with the current version" if not isinstance(actual, dict) else
            "the daemon at this address serves another robot: select the matching station "
            "or restart the daemon with this one")
    return message, hint
