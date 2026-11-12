"""``rwrl teleop`` operations dispatched to the device slice that implements them.

A slice's ``cli.py`` (``TELEOP_SLICES``) implements an operation by defining its handler
(``OPERATIONS``) and ``add_<op>_arguments(parser)``, and may declare
``GROUP_DESCRIPTIONS = {op: text}`` (the help of its flag group) and
``SHARED_FLAG_HELP = {op: {flag: text}}`` (store-true flags every implementer accepts, with its own
meaning). Handlers are looked up when called, so a test may patch them by their module path.
An operation with two or more implementers takes ``--backend`` (default: the station teleop
backend) and refuses the flags of the other backends; their value flags default to None and
the handler applies its defaults. ``read`` and ``inspect-stop`` are not device specific and are
not dispatched.

The flag groups are added only when argparse selects their operation: ``rwrl`` registers every
command's parser at start-up, and importing a slice imports the teleop package (numpy) and the
device layer. ``LazyOperations`` subclasses argparse's private ``_SubParsersAction`` and relies on
its ``__call__(parser, namespace, values, option_string=None)`` contract, which looks up
``_name_parser_map[values[0]]`` and parses the rest with that parser; written against CPython
3.10-3.13 (``requires-python >= 3.10``).
"""
from __future__ import annotations

import argparse
from importlib import import_module

from .errors import CliError, USAGE_EXIT_CODE

TELEOP_SLICES = {
    "spacemouse": "real_world_rl.teleop.spacemouse.cli",
    "gello": "real_world_rl.teleop.gello.cli",
}
# Operation -> name of its handler in a slice's cli.py.
OPERATIONS = {
    "calibrate": "calibrate",
    "diagnose": "diagnose",
    "run": "run_session",
    "profiles": "profiles",
    "probe-hid": "probe_hid",
}
# Operations on host devices rather than the station teleop: their one implementer runs whatever
# the station teleop is (the station only locates their output).
HOST_OPERATIONS = frozenset({"probe-hid"})
_FLAGS_DEFAULT = "_teleop_flags"


class UnsupportedTeleopOperation(CliError):
    def __init__(self, backend: str, operation: str) -> None:
        super().__init__(f"backend '{backend}' does not implement teleop {operation}", code=USAGE_EXIT_CODE,
                         hint="implemented by: " + ", ".join(implementers(operation)))


def slice_module(backend: str):
    return import_module(TELEOP_SLICES[backend])


def arguments_adder(operation: str) -> str:
    return "add_" + operation.replace("-", "_") + "_arguments"


def implementers(operation: str) -> tuple[str, ...]:
    """The backends whose slice implements ``operation``, in ``TELEOP_SLICES`` order."""
    found = []
    for backend in TELEOP_SLICES:
        module = slice_module(backend)
        if callable(getattr(module, OPERATIONS[operation], None)) and callable(
                getattr(module, arguments_adder(operation), None)):
            found.append(backend)
    return tuple(found)


class _SliceArguments:
    """What a slice's ``add_<op>_arguments`` receives: ``add_argument`` goes to the slice's
    argument group (and is recorded), ``add_shared_argument`` adds a common flag once to the
    parser, and anything else (``add_subparsers``) goes to the parser."""

    def __init__(self, parser: argparse.ArgumentParser, group) -> None:
        self._parser = parser
        self._group = group
        self.actions: list[argparse.Action] = []

    def add_argument(self, *args, **kwargs):
        action = self._group.add_argument(*args, **kwargs)
        self.actions.append(action)
        return action

    def add_shared_argument(self, *args, **kwargs):
        """Add a common value flag once, outside backend ownership checks."""
        shared = self._parser.__dict__.setdefault("_teleop_shared_arguments", {})
        key = tuple(args)
        if key not in shared:
            shared[key] = self._parser.add_argument(*args, **kwargs)
        return shared[key]

    def __getattr__(self, name):
        return getattr(self._parser, name)


def add_operation_arguments(parser: argparse.ArgumentParser, operation: str) -> None:
    """``operation``'s flags: ``--backend`` and the shared flags when it has two or more
    implementers, then one argument group per implementer."""
    backends = implementers(operation)
    if len(backends) > 1:
        parser.add_argument("--backend", choices=backends,
                            help=f"teleop device to {operation}; default: the station teleop backend")
    shared: dict[str, list[str]] = {}
    for backend in backends:
        for flag, text in getattr(slice_module(backend), "SHARED_FLAG_HELP", {}).get(operation, {}).items():
            shared.setdefault(flag, []).append(f"{backend}: {text}")
    for flag, texts in shared.items():
        parser.add_argument(flag, action="store_true", help="; ".join(texts))
    flags = {}
    for backend in backends:
        module = slice_module(backend)
        group = parser.add_argument_group(backend, getattr(module, "GROUP_DESCRIPTIONS", {}).get(operation))
        arguments = _SliceArguments(parser, group)
        getattr(module, arguments_adder(operation))(arguments)
        flags[backend] = [(action.option_strings[0], action.dest) for action in arguments.actions
                          if action.option_strings]
    parser.set_defaults(**{_FLAGS_DEFAULT: flags})


class LazyOperations(argparse._SubParsersAction):
    """Sub-parsers whose arguments ``populate(name, parser)`` adds when ``name`` is selected."""

    populate = None

    def __call__(self, parser, namespace, values, option_string=None):
        name = values[0] if values else None
        populated = self.__dict__.setdefault("_populated", set())
        if self.populate is not None and name in self._name_parser_map and name not in populated:
            populated.add(name)
            self.populate(name, self._name_parser_map[name])
        super().__call__(parser, namespace, values, option_string)


def given_flags(args, backend: str) -> list[str]:
    """The flags of ``backend``'s group that are on the command line (None or False: absent)."""
    return [flag for flag, dest in getattr(args, _FLAGS_DEFAULT, {}).get(backend, ())
            if getattr(args, dest, None) not in (None, False)]


def station_teleop_backend(args) -> str | None:
    """The backend of the selected station robot's teleop (None: it has none); station loading
    errors propagate."""
    from .station_edit import station_document
    teleop = station_document(args)[4].config.teleop
    return None if teleop is None else teleop.backend


def resolve_backend(args, operation: str) -> str:
    """The backend that runs ``operation``: its only implementer for a host operation; else
    ``--backend``, else the station teleop backend, else (a station without a teleop) the only
    implementer. With one implementer a station that cannot be loaded is left to the handler,
    which reports it in its own order."""
    backends = implementers(operation)
    if operation in HOST_OPERATIONS:
        return backends[0]
    if len(backends) > 1:
        backend = getattr(args, "backend", None)
        if backend is not None:
            return backend
        backend = station_teleop_backend(args)
        if backend is None:
            raise CliError("station has no teleop; pass --backend", code=USAGE_EXIT_CODE,
                           required_fields=["--backend"])
    else:
        try:
            backend = station_teleop_backend(args)
        except Exception:  # the handler loads the station itself and reports why it cannot
            backend = None
        if backend is None:
            return backends[0]
    if backend not in backends:
        raise UnsupportedTeleopOperation(backend, operation)
    return backend


def dispatch(args, ctx, operation: str):
    """Run ``operation`` on the backend ``resolve_backend`` picks."""
    backend = resolve_backend(args, operation)
    misplaced = [flag for other in implementers(operation) if other != backend for flag in given_flags(args, other)]
    if misplaced:
        raise CliError(f"{', '.join(misplaced)} not applicable to --backend {backend}", code=USAGE_EXIT_CODE)
    return getattr(slice_module(backend), OPERATIONS[operation])(args, ctx)
