"""Run the deployment loop with a local websocket server and simulated robot."""
from __future__ import annotations

import asyncio
import socket
from concurrent.futures import Future
from threading import Event, Thread
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from real_world_rl import deploy_openpi
from real_world_rl.cli import main
from real_world_rl.devices.arm import ControlMode
from real_world_rl.envs.real_robot import RealRobotEnv, RealRobotEnvConfig
from real_world_rl.openpi_client import OpenPiClient, OpenPiError, packb, unpackb

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
        home_joints=[0] * 7, safety_box=[-0.04] * 3 + [0.04] * 3,
        max_delta_per_step={"xyz_m": 0.01},
    ))
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
