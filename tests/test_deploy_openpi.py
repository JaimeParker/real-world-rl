"""Run the deployment loop with a local websocket server and simulated robot."""
from __future__ import annotations

from _station_limits import station_limits, env_limits, tcp_box

import asyncio
import socket
from concurrent.futures import Future
from threading import Event, Thread
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_world_rl.deploy.openpi import loop as deploy_openpi
from real_world_rl.cli import main
from real_world_rl.devices.arm import ControlMode
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig
from real_world_rl.common.openpi_client import OpenPiClient, OpenPiError, packb, unpackb

serve = pytest.importorskip("websockets.asyncio.server").serve


@pytest.fixture
def policy_server(monkeypatch):
    """Start the openpi handshake on loopback and surface server startup failures."""
    stop = Event()
    ready = Future()
    requests = []
    headers = []
    thread = None

    def start(reply, *, require_tcp=False):
        nonlocal thread

        async def answer(frame):
            obs = unpackb(frame)
            requests.append(obs)
            return await asyncio.to_thread(reply, obs, len(requests))

        async def handle(connection):
            headers.append(connection.request.headers.get("Authorization"))
            await connection.send(packb({"test_server": True}))
            async for frame in connection:
                result = await answer(frame)
                await connection.send(result if isinstance(result, str) else packb(result))

        listener = None
        try:
            listener = socket.socket()
            listener.bind(("127.0.0.1", 0))
        except PermissionError:
            if listener is not None:
                listener.close()
            if require_tcp:
                pytest.skip("the sandbox forbids loopback sockets")
            # Retain codec and deployment checks with an in-memory framed connection.
            from websockets.sync import client

            class FramedConnection:
                def __init__(self, uri, *, additional_headers, **kwargs):
                    headers.append((additional_headers or {}).get("Authorization"))
                    self.request = None

                def send(self, frame):
                    obs = unpackb(frame)
                    requests.append(obs)
                    self.request = (obs, len(requests))

                def recv(self, *, timeout):
                    if self.request is None:
                        return packb({"test_server": True})
                    result = reply(*self.request)
                    return result if isinstance(result, str) else packb(result)

                def close(self):
                    pass

            monkeypatch.setattr(client, "connect", FramedConnection)
            return 8000, requests, headers

        async def run():
            async with serve(handle, sock=listener, compression=None) as server:
                ready.set_result(server.sockets[0].getsockname()[1])
                await asyncio.to_thread(stop.wait)

        def worker():
            try:
                asyncio.run(run())
            except BaseException as error:
                if ready.done():
                    raise
                ready.set_exception(error)

        thread = Thread(target=worker, daemon=True)
        thread.start()
        return ready.result(timeout=5), requests, headers

    yield start
    stop.set()
    if thread is not None:
        thread.join(timeout=5)
        assert not thread.is_alive()


def _reply(action_dim: int, request: int = 1) -> dict:
    return {
        "actions": np.full((3, action_dim), request * 0.1, dtype=np.float32),
        "server_timing": {"infer_ms": 3.0, "prev_total_ms": 5.0},
    }


def test_cli_dry_run_sends_one_rlinf_observation_and_no_robot(monkeypatch, policy_server, capsys):
    port, requests, headers = policy_server(lambda obs, request: _reply(7))
    monkeypatch.setattr(deploy_openpi, "RealRobotEnv", lambda config: pytest.fail("constructed a robot"))
    assert main([
        "deploy-openpi", "--dry-run", "--prompt", "Lift the block", "--api-key", "test",
        "--policy-port", str(port), "--image-size", "8", "12",
        "--camera-keys", "wrist", "observation/image", "side", "observation/extra_view_image",
        "--crop-regions", "wrist", "0.1", "0.2", "0.9", "0.8",
    ]) == 0
    assert headers == ["Api-Key test"]
    obs, = requests
    assert obs["prompt"] == "Lift the block"
    assert obs["observation/state"].shape == (19,)
    assert obs["observation/image"].shape == obs["observation/extra_view_image"].shape == (8, 12, 3)
    assert obs["observation/image"].dtype == np.uint8
    assert "dry-run actions=(3, 7)" in capsys.readouterr().out


@pytest.mark.parametrize("require_tcp", [False, True])
def test_client_receives_metadata_and_reports_text_errors(policy_server, require_tcp):
    port, _, _ = policy_server(lambda obs, request: "test server traceback", require_tcp=require_tcp)
    with OpenPiClient("127.0.0.1", port) as client:
        assert client.metadata == {"test_server": True}
        with pytest.raises(OpenPiError, match="test server traceback"):
            client.infer({"prompt": "test"})


class FakeEnv:
    def __init__(self, config, *, before_step=None, truncate_at=None):
        self.config = config
        self.actions = []
        self.resets = 0
        self.steps = 0
        self.closed = False
        self.modes = []
        self.before_step = before_step
        self.truncate_at = truncate_at
        self.robot = SimpleNamespace(arm=SimpleNamespace(
            health=lambda: SimpleNamespace(ok=True), set_control_mode=self.modes.append,
        ))

    def _obs(self):
        return {"state": {
            "joint_pos": np.array([10 * self.resets + self.steps, 4.0], dtype=np.float32),
            "gripper_pos": np.array([0.5], dtype=np.float32),
        }, "frames": {"wrist": np.zeros((4, 5, 3), dtype=np.uint8)}}

    def reset(self):
        self.resets += 1
        self.steps = 0
        return self._obs(), {}

    def step(self, action):
        self.steps += 1
        if self.before_step is not None:
            self.before_step(self)
        self.actions.append(action.copy())
        return self._obs(), 1.0, False, self.steps == self.truncate_at, {"rtt_ms": 1.5}

    def close(self):
        self.closed = True


def _generic_args(port, **kwargs):
    return deploy_openpi.DeployOpenPiArgs(
        prompt="test", profile="generic", policy_port=port, policy_timeout_s=3,
        real_robot=RealRobotEnvConfig(
            remote="tunnel:50051", state_fields=("gripper_pos", "joint_pos"),
            num_joints=2, action_mode="joint_delta", gripper_in_action=False,
        ),
        camera_keys={"wrist": "image"}, state_key="s", prompt_key="instruction",
        image_size=(4, 5), **kwargs,
    )


@pytest.mark.parametrize("replan,prefetch,states,actions", [
    (None, 0, [0, 10, 13], [2, 2, 2, 3, 3]),
    (2, 0, [0, 10, 12, 14], [2, 2, 3, 3, 4]),
    (2, 1, [0, 10, 11, 13], [2, 2, 3, 3, 4]),
])
def test_chunk_execution_and_prefetch(replan, prefetch, states, actions, policy_server, monkeypatch):
    prefetch_started = Event()
    tail_executed = Event()

    def reply(obs, request):
        if prefetch and obs["s"][1] == 11:
            prefetch_started.set()
            assert tail_executed.wait(timeout=2)
        return _reply(2, request)

    def before_step(env):
        if prefetch and env.steps == 2:
            assert prefetch_started.wait(timeout=2)
            tail_executed.set()

    port, requests, _ = policy_server(reply)
    args = _generic_args(port, replan_steps=replan, prefetch_steps=prefetch, max_steps_per_episode=5)
    env = FakeEnv(args.real_robot, before_step=before_step)

    def build(config):
        assert len(requests) == 1
        assert config.remote == "tunnel:50051"
        assert config.cameras == ["wrist"] and config.image_size == (4, 5)
        return env

    monkeypatch.setattr(deploy_openpi, "RealRobotEnv", build)
    result, = deploy_openpi.run_deploy(args)
    np.testing.assert_allclose(env.actions, np.repeat(np.array(actions)[:, None], 2, axis=1) * 0.1)
    assert [obs["s"][1] for obs in requests] == states
    np.testing.assert_array_equal(requests[1]["s"], [0.5, 10.0, 4.0])
    assert requests[1]["image"].shape == (4, 5, 3)
    assert requests[1]["instruction"] == "test"
    assert result.steps == result.episode_return == 5
    assert result.truncated and not result.terminated and result.health_ok
    assert result.rtt_ms_mean == 1.5
    assert len(result.inference_ms) == len(states) - 1
    assert result.server_infer_ms == (3.0,) * (len(states) - 1)
    assert env.closed


def test_prefetch_is_discarded_at_early_episode_boundary(policy_server, monkeypatch):
    idled = Event()

    def reply(obs, request):
        if request == 3:
            assert idled.wait(timeout=2)
        return _reply(2, request)

    port, requests, _ = policy_server(reply)
    args = _generic_args(port, episodes=2, replan_steps=3, prefetch_steps=2, max_steps_per_episode=8)
    env = FakeEnv(args.real_robot, truncate_at=2)

    def set_mode(mode):
        assert mode == ControlMode.IDLE
        idled.set()

    env.robot.arm.set_control_mode = set_mode
    monkeypatch.setattr(deploy_openpi, "RealRobotEnv", lambda config: env)
    results = deploy_openpi.run_deploy(args)
    assert idled.is_set()
    assert [obs["s"][1] for obs in requests] == [0, 10, 11, 20, 21]
    np.testing.assert_allclose(env.actions, [[0.2, 0.2]] * 2 + [[0.4, 0.4]] * 2)
    assert [result.steps for result in results] == [2, 2]


def test_invalid_chunk_closes_robot_without_issuing_actions(policy_server, monkeypatch):
    def reply(obs, request):
        return _reply(2, request) if request == 1 else {"actions": np.full((2, 2), np.nan)}

    port, _, _ = policy_server(reply)
    args = _generic_args(port)
    env = FakeEnv(args.real_robot)
    monkeypatch.setattr(deploy_openpi, "RealRobotEnv", lambda config: env)
    with pytest.raises(OpenPiError, match="expected finite actions"):
        deploy_openpi.run_deploy(args)
    assert env.closed and not env.actions


def test_rlinf_profile_state_and_action_against_reference():
    """Independent homogeneous/adjoint calculations from RLinf's source."""
    rot = Rotation.from_euler("xyz", [0.4, -0.5, 1.3])
    reset_rot = Rotation.from_euler("xyz", [-0.7, 0.1, 0.25])
    stiffness_rot = Rotation.from_euler("z", 0.3)
    position = np.array([0.6, -0.2, 0.5])
    pose = np.r_[position, rot.as_quat()]
    reset_pose = np.r_[[0.4, 0.2, 0.3], reset_rot.as_quat()]
    x, y, z = position
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    matrix = rot.as_matrix()
    adjoint = np.block([[matrix, np.zeros((3, 3))], [skew @ matrix, matrix]])
    current = np.eye(4)
    current[:3, :3] = matrix
    current[:3, 3] = position
    reset = np.eye(4)
    reset[:3, :3] = reset_rot.as_matrix()
    reset[:3, 3] = reset_pose[:3]
    relative = np.linalg.inv(reset) @ current
    stiffness_basis = matrix @ stiffness_rot.as_matrix()
    state = {
        "gripper_pos": np.array([0.4]), "tcp_force": np.array([3.0, 2.0, 1.0]),
        "tcp_pose": pose, "tcp_torque": np.array([6.0, 5.0, 4.0]),
        "tcp_vel": np.array([1.0, -2.0, 3.0, -4.0, 5.0, -6.0]),
    }
    args = deploy_openpi.DeployOpenPiArgs(
        prompt="test", dry_run=True, stiffness_quat=tuple(stiffness_rot.as_quat()),
    )
    profile = deploy_openpi._make_profile(args)
    assert profile.config.image_size == (128, 128)
    preprocessing = profile.config.preprocess["wrist_1"]
    assert preprocessing.resize == "cv2_linear" and preprocessing.crop_region == "center_square"
    profile.reset({"state": {"tcp_pose": reset_pose}}, SimpleNamespace(
        robot=SimpleNamespace(gripper=SimpleNamespace(max_width_m=0.08)),
    ))
    got = profile.state_vector(state)
    expected = np.r_[
        0.032, stiffness_basis.T @ state["tcp_force"], relative[:3, 3],
        Rotation.from_matrix(relative[:3, :3]).as_euler("xyz"),
        stiffness_basis.T @ state["tcp_torque"], np.linalg.inv(adjoint) @ state["tcp_vel"],
    ]
    assert got.dtype == np.float32 and got.shape == (19,)
    np.testing.assert_allclose(got, expected, rtol=1e-6)
    frames = {camera: np.array([[[1, 2, 3]]], dtype=np.uint8) for camera in profile.camera_keys}
    np.testing.assert_array_equal(profile.observation(got, frames)["observation/image"], [[[3, 2, 1]]])

    # Check the actual RealRobotEnv action path, before the daemon applies limits.
    real_env = RealRobotEnv(
        RealRobotEnvConfig(remote="test", cartesian_semantics="rlinf", xyz_scale_m=0.03, rot_scale_rad=0.25),
        robot=SimpleNamespace(),
    )
    real_env._last_raw = {"arm": {"tcp_pose": pose}}
    action = np.array([1.0, -1.0, 0.9, 1.0, 0.3, 0.8, -0.5])
    move = np.clip(adjoint @ action[:6], -1, 1)
    target = np.r_[position + move[:3] * 0.03, (Rotation.from_euler("xyz", move[3:] * 0.25) * rot).as_quat()]
    np.testing.assert_allclose(real_env._arm_command(action)["tcp_pose"], target)


def test_rlinf_env_sends_gripper_thresholds_and_daemon_limits_the_arm(tmp_path, monkeypatch):
    import real_world_rl.devices.backends  # noqa: F401
    from real_world_rl.devices import GripperConfig, claims
    from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
    from real_world_rl.server import RemoteRobot
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.transport import LocalClient

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    robot = FrankaRobot(FrankaConfig(
        arm_backend="fake", gripper=GripperConfig(backend="fake"),
        home_joints=[0] * 7, limits=station_limits(bounds=[-0.04] * 3 + [0.04] * 3, steps={"xyz_m": 0.01}),

    ))
    robot.arm._kinematics = lambda q: np.zeros(3)  # isolate Cartesian daemon limiting
    service = RobotService(robot, watchdog_s=5.0)
    env = RealRobotEnv(
        RealRobotEnvConfig(
            remote="test", cartesian_semantics="rlinf", xyz_scale_m=0.25,
            gripper_action_scale=2.0, reset_settle_s=0.0, control_hz=1000,
        ),
        robot=RemoteRobot.from_client(LocalClient(service)),
    )
    try:
        env.reset()
        action = np.zeros(7)
        action[0] = 1.0
        for gripper in (-0.24, -0.25, -1, 0, 0.24, 0.25):
            action[-1] = gripper
            env.step(action)
            if gripper == -0.24:
                assert robot.arm.last_cartesian_target[0] == pytest.approx(0.01)
        assert robot.arm.last_cartesian_target[0] <= 0.04
        assert [call for call in robot.gripper.call_log if call in ("open", "close")] == [
            "open", "close", "open",
        ]
    finally:
        env.close()
        service.close()


@pytest.fixture
def reconnect_deploy(monkeypatch):
    """Exercise retries with a fake clock, policy connection, and robot."""
    from real_world_rl.server.transport.base import TransportUnavailable

    args = _generic_args(8000, max_steps_per_episode=5, replan_steps=3, prefetch_steps=2)
    error = TransportUnavailable("tunnel dropped")
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(deploy_openpi, "time", SimpleNamespace(
        monotonic=lambda: clock.now,
        sleep=lambda seconds: setattr(clock, "now", clock.now + seconds),
    ))

    def before_step(env):
        if env.resets == 1 and env.steps == 2:
            raise error

    env = FakeEnv(args.real_robot, before_step=before_step)
    step = env.step

    def step_with_success(action):
        obs, reward, terminated, truncated, info = step(action)
        info["task_success"] = env.resets > 1
        return obs, reward, terminated, truncated, info

    env.step = step_with_success
    requests = []

    class FakePolicy:
        def __init__(self, *args, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def infer(self, observation):
            requests.append(observation)
            return _reply(2, len(requests))

    attempts = []

    def reconnect():
        assert env.resets == 1
        attempts.append(clock.now)
        if len(attempts) < 3:
            raise error
        return {"watchdog_trips": 1}

    env.reconnect_remote = reconnect
    monkeypatch.setattr(deploy_openpi, "RealRobotEnv", lambda config: env)
    monkeypatch.setattr(deploy_openpi, "OpenPiClient", FakePolicy)
    return SimpleNamespace(args=args, env=env, attempts=attempts, clock=clock, requests=requests)


def test_disconnect_waits_for_confirmation_and_reports_interruption(reconnect_deploy, capsys):
    run = reconnect_deploy

    def confirm():
        assert run.attempts == [1, 3, 7]
        assert run.env.resets == 1 and len(run.env.actions) == 1
        return True

    interrupted, completed = deploy_openpi.run_deploy(run.args, confirm_resume=confirm)
    assert run.env.resets == 2 and len(run.env.actions) == 6
    assert interrupted.steps == 1 and interrupted.outcome.end_reason == "interrupted"
    assert interrupted.outcome.interrupted and not interrupted.outcome.completed
    assert interrupted.disconnects == 1 and interrupted.disconnected_s == 7
    assert completed.outcome.completed and completed.disconnects == 0
    output = capsys.readouterr().out
    assert "reconnect attempt=3 disconnected_s=7.0" in output
    assert "watchdog_trips=1" in output
    assert "summary completed_episodes=1 interrupted_episodes=1 success_rate=1.000" in output
    assert "disconnects=1 disconnected_s=7.0" in output


def test_disconnect_duration_includes_internal_retries(reconnect_deploy, capsys):
    run = reconnect_deploy
    before_step = run.env.before_step

    def simulate_rpc_timing(env):
        if env.resets == 1:
            # The first step succeeds at t=4. The second surfaces a transport error at t=14.
            run.clock.now += 4 if env.steps == 1 else 10
        before_step(env)

    run.env.before_step = simulate_rpc_timing
    run.args.auto_resume = True
    run.args.reconnect_timeout_s = 8
    interrupted, _ = deploy_openpi.run_deploy(run.args)

    assert run.attempts == [15, 17, 21]  # All three retries fit in the WAITING budget.
    assert interrupted.disconnected_s == 17
    output = capsys.readouterr().out
    assert "reconnect attempt=1 disconnected_s=11.0" in output
    assert "reconnect attempt=3 disconnected_s=17.0" in output
    assert "disconnects=1 disconnected_s=17.0" in output


@pytest.mark.parametrize("watchdog", [False, True])
def test_disconnect_auto_resume(reconnect_deploy, watchdog):
    from real_world_rl.devices.errors import DeviceFault

    run = reconnect_deploy
    if watchdog:
        def trip(env):
            if env.resets == 1:
                raise DeviceFault("the daemon watchdog stopped the arm after 0.50s", recoverable=True)

        run.env.before_step = trip
    run.args.auto_resume = True
    interrupted, completed = deploy_openpi.run_deploy(
        run.args, confirm_resume=lambda: pytest.fail("auto-resume prompted"),
    )
    assert interrupted.outcome.interrupted and completed.outcome.completed
    assert run.env.resets == 2


@pytest.mark.parametrize("timeout,expected_attempts", [(2, [1]), (8, [1, 3, 7])])
def test_disconnect_timeout_exits_without_reset(reconnect_deploy, timeout, expected_attempts, monkeypatch, capsys):
    import tyro

    from real_world_rl.cli.errors import DEVICE_EXIT_CODE

    run = reconnect_deploy
    run.args.reconnect_timeout_s = timeout
    monkeypatch.setattr(tyro, "cli", lambda *args, **kwargs: run.args)
    monkeypatch.setattr(deploy_openpi.sys, "stdin", SimpleNamespace(isatty=lambda: False))
    assert deploy_openpi.main_cli([]) == DEVICE_EXIT_CODE
    assert run.attempts == expected_attempts and run.clock.now == timeout
    assert run.env.resets == 1 and run.env.closed
    captured = capsys.readouterr()
    assert ("no interactive stdin" in captured.out) == (timeout == 8)
    assert f"run: rwrl doctor --link {run.args.real_robot.remote}\n" in captured.err


def test_disconnect_cancels_queued_prefetch(reconnect_deploy, monkeypatch):
    run = reconnect_deploy
    pending = Future()

    class Executor:
        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def submit(self, fn, *args):
            assert run.env.resets == 1
            return pending

    monkeypatch.setattr(deploy_openpi, "ThreadPoolExecutor", Executor)

    def confirm():
        assert pending.cancelled()
        run.args.prefetch_steps = 0
        return True

    deploy_openpi.run_deploy(run.args, confirm_resume=confirm)
    assert run.requests[2]["s"][1] == 20  # First fresh inference follows the second reset.


@pytest.mark.parametrize("legacy", [False, True])
def test_changed_daemon_instance_renegotiates_capabilities(tmp_path, monkeypatch, legacy):
    from real_world_rl.devices import claims
    from real_world_rl.devices.robots.franka import FrankaConfig, FrankaRobot
    from real_world_rl.server import RemoteRobot
    from real_world_rl.server.service import RobotService
    from real_world_rl.server.transport import LocalClient

    monkeypatch.setattr(claims, "_DEFAULT_LOCK_DIR", tmp_path / "claims")
    old, new = [RobotService(FrankaRobot(FrankaConfig(limits=station_limits(),
        arm_backend="fake", gripper=None, home_joints=[home] * 7,
    ))) for home in (0.0, 0.1)]
    class ReconnectingClient(LocalClient):
        def call(self, method, **kwargs):
            result = super().call(method, **kwargs)
            if legacy and method == "robot.describe":
                result.pop("daemon_instance_id")
            return result

    client = ReconnectingClient(old)
    robot = RemoteRobot.from_client(client)
    config = RealRobotEnvConfig(remote="local", gripper_in_action=False, state_fields=("tcp_pose",))
    env = RealRobotEnv(config, robot=robot)
    try:
        env.reconnect_remote()
        old_arm = robot.arm
        assert old_arm.home_joints == [0.0] * 7
        validate = env._validate_robot
        validations = []

        def record_validation():
            validations.append(robot.arm)
            validate()

        monkeypatch.setattr(env, "_validate_robot", record_validation)
        env.reconnect_remote()
        assert len(validations) == int(legacy)
        validations.clear()
        old.close()
        client._service = new
        new.robot.arm._note_error("test fault")
        env.reconnect_remote()
        assert "recover" in new.robot.arm.call_log
        assert validations == [robot.arm] and robot.arm is not old_arm
        assert robot.arm.home_joints == [0.1] * 7
    finally:
        env.close()
        old.close()
        new.close()



@pytest.mark.parametrize("fault", ["safety", "configuration", "authentication"])
def test_nonrecoverable_device_errors_exit_immediately(reconnect_deploy, fault):
    from real_world_rl.devices.errors import DeviceFault, SafetyViolation
    from real_world_rl.server.transport.base import TransportError

    errors = {
        "safety": SafetyViolation("outside workspace"),
        "configuration": DeviceFault("camera is missing"),
        "authentication": TransportError("token mismatch"),
    }
    error = errors[fault]

    def fail(env):
        raise error

    run = reconnect_deploy
    run.env.before_step = fail
    with pytest.raises(type(error), match=error.message):
        deploy_openpi.run_deploy(run.args)
    assert not run.attempts and run.env.closed
