"""Eval loop for a websocket policy and a separate remote robot daemon."""
from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable, Sequence
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field, replace
from statistics import mean
from typing import Literal

import numpy as np

from real_world_rl.contracts.outcomes import EpisodeMetrics, EpisodeOutcome, EpisodeOutcomeAccumulator
from real_world_rl.devices.arm import ControlMode
from real_world_rl.devices.errors import DeviceError, DeviceFault
from real_world_rl.server.transport.base import TransportError, TransportUnavailable
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig
from real_world_rl.common.openpi_client import OpenPiClient, OpenPiError
from real_world_rl.deploy.openpi.profiles import PROFILES, GenericProfile, RLinfFrankaProfile


@dataclass
class DeployOpenPiArgs:
    """Run an openpi policy against a remote rwrl daemon (eval only)."""

    prompt: str
    policy_host: str = "127.0.0.1"
    policy_port: int = 8000
    api_key: str | None = field(default=None, repr=False)
    policy_timeout_s: float = 30.0
    profile: Literal["rlinf_franka", "generic"] = "rlinf_franka"
    real_robot: RealRobotEnvConfig = field(default_factory=RealRobotEnvConfig)
    episodes: int = 1
    max_steps_per_episode: int = 300
    # None executes the entire returned chunk.
    replan_steps: int | None = None
    # Query this many steps before a chunk boundary. Zero disables prefetch.
    prefetch_steps: int = 0
    # Camera frame size as (height, width). Defaults to the env size or the profile default.
    image_size: tuple[int, int] | None = None
    # Keys are daemon camera names and values are openpi input keys.
    camera_keys: dict[str, str] = field(default_factory=dict)
    # RLinf camera name followed by fractional top, left, bottom, right.
    crop_regions: dict[str, tuple[float, float, float, float]] = field(default_factory=dict)
    # Input key for the flat policy state.
    state_key: str = "observation/state"
    # Input key for the task instruction.
    prompt_key: str = "prompt"
    # RLinf Franka only: the rotation of the stiffness frame in the TCP frame (xyzw).
    stiffness_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    # Perform a single synthetic inference without connecting to a robot.
    dry_run: bool = False
    # Timeout counted from entry into WAITING. None waits indefinitely.
    reconnect_timeout_s: float | None = None
    # Reset for the next episode without terminal confirmation after reconnecting.
    auto_resume: bool = False


@dataclass(frozen=True)
class Inference:
    actions: np.ndarray
    latency_ms: float
    server_infer_ms: float | None


@dataclass(frozen=True)
class EpisodeResult:
    steps: int
    episode_return: float
    terminated: bool
    truncated: bool
    health_ok: bool
    rtt_ms_mean: float | None
    inference_ms: tuple[float, ...]
    server_infer_ms: tuple[float, ...]
    outcome: EpisodeOutcome
    disconnects: int = 0
    disconnected_s: float = 0.0


def _make_profile(args: DeployOpenPiArgs) -> GenericProfile:
    if not args.prompt.strip() or not args.policy_host or not 1 <= args.policy_port <= 65535:
        raise ValueError("provide a prompt, policy host and valid policy port")
    if not math.isfinite(args.policy_timeout_s) or args.policy_timeout_s <= 0:
        raise ValueError("policy_timeout_s must be finite and positive")
    if min(args.episodes, args.max_steps_per_episode) <= 0 or args.prefetch_steps < 0:
        raise ValueError("episodes and max_steps_per_episode must be positive, prefetch_steps nonnegative")
    if args.replan_steps is not None and args.replan_steps <= 0:
        raise ValueError("replan_steps must be positive when supplied")
    if args.reconnect_timeout_s is not None and (
        not math.isfinite(args.reconnect_timeout_s) or args.reconnect_timeout_s <= 0
    ):
        raise ValueError("reconnect_timeout_s must be finite and positive")
    config = args.real_robot
    if not args.dry_run and (not config.remote or config.robot is not None):
        raise ValueError("live deployment requires --real-robot.remote with the daemon address")
    profile = PROFILES[args.profile]
    camera_keys = args.camera_keys or profile.default_cameras or {cam: cam for cam in config.cameras}
    keys = [args.state_key, args.prompt_key, *camera_keys.values()]
    if not all(keys) or len(set(keys)) != len(keys):
        raise ValueError("state, prompt and camera input keys must be nonempty and distinct")
    if args.profile == "rlinf_franka" and "observation/image" not in camera_keys.values():
        raise ValueError("rlinf_franka requires a camera mapped to observation/image")
    if config.cameras and set(config.cameras) != set(camera_keys):
        raise ValueError("real_robot.cameras must match the camera_keys mapping")
    image_size = args.image_size if args.image_size is not None else config.image_size or profile.default_image_size
    config = replace(config, cameras=list(camera_keys), image_size=image_size, **profile.env_overrides)
    if profile is RLinfFrankaProfile:
        config = replace(config, preprocess=profile.preprocess_config(config, args.crop_regions))
    elif args.crop_regions:
        raise ValueError("crop_regions require the rlinf_franka profile")
    profile_options = {"stiffness_quat": args.stiffness_quat} if args.profile == "rlinf_franka" else {}
    return profile(config, camera_keys, args.state_key, args.prompt_key, args.prompt, **profile_options)


def _infer(client: OpenPiClient, observation: dict, action_dim: int) -> Inference:
    start = time.monotonic()
    reply = client.infer(observation)
    latency_ms = (time.monotonic() - start) * 1000
    actions = reply.get("actions")
    if (
        not isinstance(actions, np.ndarray) or actions.ndim != 2
        or actions.shape[0] == 0 or actions.shape[1] != action_dim
        or actions.dtype.kind not in "fiu" or not np.isfinite(actions).all()
    ):
        shape = getattr(actions, "shape", None)
        raise OpenPiError(f"expected finite actions shaped (horizon, {action_dim}), got {shape}")
    timing = reply.get("server_timing")
    infer_ms = timing.get("infer_ms") if isinstance(timing, dict) else None
    if not isinstance(infer_ms, (float, int, np.floating, np.integer)) or not np.isfinite(infer_ms):
        infer_ms = None
    return Inference(actions, latency_ms, infer_ms)


def _run_episode(args: DeployOpenPiArgs, profile: GenericProfile, client: OpenPiClient,
                 env: RealRobotEnv, executor: ThreadPoolExecutor,
                 ) -> tuple[EpisodeResult, DeviceError | None, Future[Inference] | None, float | None]:
    steps = 0
    episode_return = 0.0
    terminated = truncated = health_ok = False
    rtts: list[float] = []
    latencies: list[float] = []
    server_latencies: list[float] = []
    pending: Future[Inference] | None = None
    interruption = None
    last_ok: float | None = None
    outcome = EpisodeOutcomeAccumulator()

    def request_observation() -> dict:
        return profile.observation(profile.state_vector(obs["state"]), obs["frames"])

    try:
        obs, _ = env.reset()
        last_ok = time.monotonic()
        profile.reset(obs, env)
        while not (terminated or truncated):
            if pending is None:
                inference = _infer(client, request_observation(), profile.action_dim)
            else:
                inference = pending.result()
                pending = None
            latencies.append(inference.latency_ms)
            if inference.server_infer_ms is not None:
                server_latencies.append(inference.server_infer_ms)
            remaining = args.max_steps_per_episode - steps
            actions = inference.actions[:args.replan_steps]
            window = min(len(actions), remaining)
            # Always complete at least one step of the current chunk before snapshotting.
            prefetch_at = max(1, window - args.prefetch_steps)
            for index, action in enumerate(actions[:window], start=1):
                if pending is not None and pending.done():
                    pending.result()  # Surface a failed request before continuing the tail.
                obs, reward, terminated, truncated, info = env.step(action)
                last_ok = time.monotonic()
                outcome.observe(info)
                steps += 1
                episode_return += float(reward)
                rtt = info.get("rtt_ms")
                if rtt is not None:
                    rtts.append(float(rtt))
                truncated = truncated or (steps >= args.max_steps_per_episode and not terminated)
                if terminated or truncated:
                    break
                if index == prefetch_at and index < window < remaining:
                    pending = executor.submit(_infer, client, request_observation(), profile.action_dim)

        health_ok = bool(env.robot.arm.health().ok)
        if pending is not None:
            # Early termination can leave a request in flight. Discard it before the next
            # reset, and idle the arm before waiting for the request to drain.
            if not pending.done():
                env.robot.arm.set_control_mode(ControlMode.IDLE)
            inference = pending.result()
            latencies.append(inference.latency_ms)
            if inference.server_infer_ms is not None:
                server_latencies.append(inference.server_infer_ms)
    except DeviceError as error:
        if not _can_reconnect(error):
            raise
        interruption = error
        if last_ok is None:
            last_ok = time.monotonic()
        if pending is not None:
            pending.cancel()

    result = EpisodeResult(
        steps, episode_return, terminated, truncated, health_ok,
        mean(rtts) if rtts else None, tuple(latencies), tuple(server_latencies),
        outcome.finish(interrupted=interruption is not None, terminated=terminated, truncated=truncated),
    )
    return result, interruption, pending if interruption is not None else None, last_ok


def _can_reconnect(error: DeviceError) -> bool:
    if isinstance(error, TransportError):
        return isinstance(error, TransportUnavailable) or error.recoverable
    return (isinstance(error, DeviceFault) and error.recoverable
            and error.message.startswith("the daemon watchdog stopped the arm"))


def _wait_for_robot(env: RealRobotEnv, error: DeviceError, disconnected_since: float, deadline: float) -> None:
    print(f"[deploy-openpi] WAITING outcome=interrupted for robot daemon ({error.message})", flush=True)
    delay = 1.0
    attempt = 0
    while time.monotonic() < deadline:
        time.sleep(min(delay, max(0.0, deadline - time.monotonic())))
        now = time.monotonic()
        if now >= deadline:
            break
        attempt += 1
        print(f"[deploy-openpi] reconnect attempt={attempt} disconnected_s={now - disconnected_since:.1f}",
              flush=True)
        try:
            health = env.reconnect_remote()
        except DeviceError as retry_error:
            if not _can_reconnect(retry_error):
                raise
        else:
            if time.monotonic() < deadline:
                print(f"[deploy-openpi] reconnected watchdog_trips={health['watchdog_trips']}", flush=True)
                return
            break
        delay = min(delay * 2, 10.0)
    raise error


def _terminal_confirmation() -> bool:
    if sys.stdin is not None and sys.stdin.isatty():
        try:
            input("press Enter to reset and start the next episode")
            return True
        except EOFError:
            pass
    return False


def _confirm_resume(confirm: Callable[[], bool], error: DeviceError, deadline: float) -> None:
    if confirm():
        return
    print("[deploy-openpi] WAITING for operator confirmation (no interactive stdin)", flush=True)
    while time.monotonic() < deadline:
        time.sleep(min(10.0, max(0.0, deadline - time.monotonic())))
    raise error


def _format_ms(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.1f}"


def _report_episode(episode: int, result: EpisodeResult) -> None:
    server_mean = mean(result.server_infer_ms) if result.server_infer_ms else None
    inference_mean = mean(result.inference_ms) if result.inference_ms else None
    print(
        f"[deploy-openpi] episode={episode} steps={result.steps} "
        f"return={result.episode_return:.3f} terminated={result.terminated} "
        f"truncated={result.truncated} health_ok={result.health_ok} "
        f"outcome={result.outcome.end_reason} disconnects={result.disconnects} "
        f"disconnected_s={result.disconnected_s:.1f} "
        f"rtt_ms_mean={_format_ms(result.rtt_ms_mean)} "
        f"inferences={len(result.inference_ms)} "
        f"inference_ms_mean={_format_ms(inference_mean)} "
        f"inference_ms_max={_format_ms(max(result.inference_ms, default=None))} "
        f"server_infer_ms_mean={_format_ms(server_mean)}",
        flush=True,
    )


def run_deploy(args: DeployOpenPiArgs, *,
               confirm_resume: Callable[[], bool] = _terminal_confirmation) -> list[EpisodeResult]:
    profile = _make_profile(args)
    with ThreadPoolExecutor(max_workers=1, thread_name_prefix="openpi") as executor, OpenPiClient(
        args.policy_host, args.policy_port, api_key=args.api_key, timeout_s=args.policy_timeout_s,
    ) as client:
        warmup = _infer(client, profile.synthetic_observation(), profile.action_dim)
        print(f"[deploy-openpi] {'dry-run' if args.dry_run else 'warmup'} "
              f"actions={warmup.actions.shape} inference_ms={warmup.latency_ms:.1f} "
              f"server_infer_ms={_format_ms(warmup.server_infer_ms)}", flush=True)
        if args.dry_run:
            return []
        env = RealRobotEnv(profile.config)
        results = []
        metrics = EpisodeMetrics()
        completed = 0
        try:
            while completed < args.episodes:
                result, error, abandoned, last_ok = _run_episode(args, profile, client, env, executor)
                if error is None:
                    completed += 1
                    results.append(result)
                    metrics.add(result.outcome)
                    _report_episode(len(results) - 1, result)
                    continue

                waiting_since = time.monotonic()
                assert last_ok is not None
                disconnected_since = last_ok
                deadline = waiting_since + (args.reconnect_timeout_s or math.inf)
                disconnects = 1
                disconnected_s = 0.0
                try:
                    while True:
                        try:
                            _wait_for_robot(env, error, disconnected_since, deadline)
                        finally:
                            disconnected_s += time.monotonic() - disconnected_since
                        if abandoned is not None and not abandoned.cancelled():
                            # The websocket must consume its outstanding reply before another request.
                            abandoned.result()
                        if args.auto_resume:
                            break
                        _confirm_resume(confirm_resume, error, deadline)
                        try:
                            # The daemon may have restarted again while the operator was away.
                            env.reconnect_remote()
                        except DeviceError as retry_error:
                            if not _can_reconnect(retry_error):
                                raise
                            error = retry_error
                            disconnects += 1
                            disconnected_since = time.monotonic()
                            deadline = disconnected_since + (args.reconnect_timeout_s or math.inf)
                        else:
                            break
                finally:
                    result = replace(result, disconnects=disconnects, disconnected_s=disconnected_s)
                    results.append(result)
                    metrics.add(result.outcome)
                    _report_episode(len(results) - 1, result)
        finally:
            summary = metrics.summary()
            print(f"[deploy-openpi] summary completed_episodes={summary['completed_episodes']} "
                  f"interrupted_episodes={summary['interrupted_episodes']} "
                  f"success_rate={summary['success_rate']:.3f} "
                  f"disconnects={sum(result.disconnects for result in results)} "
                  f"disconnected_s={sum(result.disconnected_s for result in results):.1f}", flush=True)
            env.close()
    return results


def main_cli(argv: Sequence[str] | None = None) -> int:
    import tyro

    from real_world_rl.cli.errors import DEVICE_EXIT_CODE

    try:
        args = tyro.cli(DeployOpenPiArgs, args=argv)
        run_deploy(args)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    except (DeviceError, OpenPiError) as error:
        if isinstance(error, DeviceError):
            print(f"[deploy-openpi] device error: {error.message}", file=sys.stderr)
            if error.hint:
                print(f"[deploy-openpi] hint: {error.hint}", file=sys.stderr)
        else:
            print(f"[deploy-openpi] {error}", file=sys.stderr)
        if args.real_robot.remote:
            print(f"run: rwrl doctor --link {args.real_robot.remote}", file=sys.stderr)
        return DEVICE_EXIT_CODE
    return 0


if __name__ == "__main__":
    sys.exit(main_cli())
