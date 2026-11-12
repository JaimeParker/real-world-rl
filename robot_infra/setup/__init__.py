"""Station setup wizard: ``rwrl setup`` probes the hardware, asks a few questions,
calibrates the GELLO and writes the station yaml the daemon, doctor and discovery find without
``--station``."""
from robot_infra.setup.probes import StationProbe, discover_station, mock_station_probe
from robot_infra.setup.render import (
    check_rendered_station,
    render_station_yaml,
    write_station_yaml,
)
from robot_infra.setup.wizard import (
    Answers,
    ArmDraft,
    StationDraft,
    WizardError,
    run_wizard,
)

__all__ = [
    "Answers",
    "ArmDraft",
    "StationDraft",
    "StationProbe",
    "WizardError",
    "check_rendered_station",
    "discover_station",
    "mock_station_probe",
    "render_station_yaml",
    "run_wizard",
    "write_station_yaml",
]
