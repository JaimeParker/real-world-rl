from rl_garden.common.effective_config import FieldSource
from rl_garden.training.algorithm_registry import BaseAlgorithmRegistry


class RealWorldAlgorithmRegistry(BaseAlgorithmRegistry):
    package_name = "real_world_rl.training.entrypoints"
    phase_name = "real_world"
    supports_dry_run = False

    def _normalize_runtime(
        self, parsed: object, sources: dict[str, FieldSource],
    ) -> tuple[object, dict[str, object]]:
        """Finalize SpaceMouse semantics before preflight or run config serialization."""
        normalized, derived = super()._normalize_runtime(parsed, sources)
        from real_world_rl.training.entrypoints.bc import RealWorldBCArgs

        # Offline BC accepts shared station presets without resolving teleop settings.
        if isinstance(normalized, RealWorldBCArgs):
            return normalized, derived

        from real_world_rl.training.env_building import resolve_teleop_input

        if getattr(normalized, "spacemouse_mode", "tracker") == "tracker":
            resolve_teleop_input(normalized)
            return normalized, derived

        from real_world_rl.training.env_building import resolve_spacemouse_mode

        backend = normalized.env_backend
        if backend == "real_robot":
            path = "real_robot.cartesian_semantics"
            before = normalized.real_robot.cartesian_semantics
        elif backend == "franka_real":
            path = "franka_real.env_kwargs_json"
            before = normalized.franka_real.env_kwargs_json
        else:
            # The shared resolver reports unsupported backends/devices.
            resolve_spacemouse_mode(normalized, sources=sources)
            return normalized, derived
        resolve_spacemouse_mode(normalized, sources=sources)
        after = getattr(getattr(normalized, backend), path.split(".", 1)[1])
        if before != after:
            reason = "SpaceMouse serl mode selects SERL Cartesian semantics"
            derived[path] = {"before": before, "after": after, "reason": reason}
            sources[path] = FieldSource(kind="runtime-derived", detail=reason)
        resolve_teleop_input(normalized)
        return normalized, derived


registry = RealWorldAlgorithmRegistry()
