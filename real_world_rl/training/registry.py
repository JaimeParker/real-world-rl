from rl_garden.training.algorithm_registry import BaseAlgorithmRegistry


class RealWorldAlgorithmRegistry(BaseAlgorithmRegistry):
    package_name = "real_world_rl.training.entrypoints"
    phase_name = "real_world"
    supports_dry_run = False


registry = RealWorldAlgorithmRegistry()
