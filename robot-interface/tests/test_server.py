"""The robot server and its clients over TCP, with the simulated robot."""

import concurrent.futures
import socket
import time

import numpy as np
import pytest

from csc376_franky import ErrorCodes, FrankaJointTrajectoryController, Gripper, RobotError, protocol

from conftest import linear_trajectory, make_station, wait_until


def in_background(function, *args):
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    future = executor.submit(function, *args)
    executor.shutdown(wait=False)
    return future


def raises(code, function, *args, **kwargs):
    with pytest.raises(RobotError) as error:
        function(*args, **kwargs)
    assert error.value.code == code, error.value
    return error.value.message


class RawClient:
    """Speaks the protocol by hand, to send what csc376_franky never would."""

    def __init__(self, port):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.file = self.sock.makefile("rb")

    def send(self, message):
        self.sock.sendall(protocol.encode(message) if isinstance(message, dict) else message)

    def reply(self, request_id=None):
        while True:
            message = protocol.decode(self.file.readline())
            if message.get("event") != "state" and message.get("id") == request_id:
                return message

    def state(self):
        while True:
            message = protocol.decode(self.file.readline())
            if message.get("event") == "state" and "q" in message["robot"]:
                return message

    def close(self):
        self.sock.close()


def hello(raw):
    raw.send({"op": "hello", "id": 1, "protocol": protocol.PROTOCOL_VERSION, "client": {"host": "raw"}})
    assert raw.reply(1)["ok"]


def test_both_clients_see_the_robot(station):
    with station.client() as a, station.client() as b:
        assert a.server_info["backend"] == "sim" and a.session != b.session
        np.testing.assert_allclose(a.get_current_joint_positions(), b.get_current_joint_positions(), atol=1e-9)
        state = a.get_current_robot_state()
        assert len(state.q) == 7 and state.robot_mode == "idle" and len(state.O_T_EE) == 16
        assert a.gripper.state.max_width == pytest.approx(0.08)


def test_execute_from_the_current_position(station):
    with station.client() as robot:
        q0 = robot.get_current_joint_positions()
        trajectory = linear_trajectory(q0, [0.2, 0, -0.1, 0.1, 0, 0, 0], duration=1.0)
        seen = []
        robot.set_trajectory_callback(seen.append)
        assert robot.run_joint_trajectory(trajectory, 0.01) == ErrorCodes.Success
        assert seen == list(range(len(trajectory)))
        np.testing.assert_allclose(robot.get_current_joint_positions(), trajectory[-1], atol=2e-3)


def test_one_trajectory_at_a_time_across_the_station(station):
    with station.client() as a, station.client() as b:
        q0 = a.get_current_joint_positions()
        plan_a = linear_trajectory(q0, [0.3, 0, 0, 0, 0, 0, 0], duration=4.0)
        plan_b = linear_trajectory(q0, [-0.2, 0, 0, 0, 0, 0, 0], duration=2.0)
        running = in_background(a.run_joint_trajectory, plan_a, 0.01)
        assert wait_until(lambda: b.is_executing)
        message = raises(ErrorCodes.RobotBusy, b.run_joint_trajectory, plan_b, 0.01)
        assert "127.0.0.1" in message
        assert running.result(10) == ErrorCodes.Success
        # b planned from where the robot was before a moved it.
        message = raises(ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition, b.run_joint_trajectory, plan_b, 0.01)
        assert "Plan again" in message
        q1 = b.get_current_joint_positions()
        assert b.run_joint_trajectory(linear_trajectory(q1, [-0.2, 0, 0, 0, 0, 0, 0], 2.0), 0.01) == ErrorCodes.Success


def test_any_client_can_stop_the_robot(station):
    with station.client() as a, station.client() as b:
        q0 = a.get_current_joint_positions()
        trajectory = linear_trajectory(q0, [0.5, 0, 0, 0, 0, 0, 0], duration=8.0)
        running = in_background(a.run_joint_trajectory, trajectory, 0.01)
        assert wait_until(lambda: b.is_executing)
        b.stop()
        with pytest.raises(RobotError) as error:
            running.result(10)
        assert error.value.code == ErrorCodes.Interrupted and "stop() from" in error.value.message
        assert a.get_current_joint_positions()[0] < trajectory[-1][0] - 0.1


def test_disconnecting_stops_the_robot(station):
    with station.client() as watcher:
        robot = station.client()
        q0 = robot.get_current_joint_positions()
        trajectory = linear_trajectory(q0, [0.5, 0, 0, 0, 0, 0, 0], duration=8.0)
        running = in_background(robot.run_joint_trajectory, trajectory, 0.01)
        assert wait_until(lambda: watcher.is_executing)
        robot.close()
        with pytest.raises(RobotError):
            running.result(10)
        assert watcher.wait_until_idle(timeout=1.0)
        assert watcher.get_current_joint_positions()[0] < trajectory[-1][0] - 0.1


def test_a_silent_client_is_stopped(station):
    raw = RawClient(station.port)
    hello(raw)
    q0 = np.array(raw.state()["robot"]["q"])
    trajectory = linear_trajectory(q0, [0.5, 0, 0, 0, 0, 0, 0], duration=8.0)
    started = time.monotonic()
    raw.send({"op": "execute", "id": 2, "trajectory": trajectory.tolist(), "dt": 0.01})
    reply = raw.reply(2)
    # No heartbeats: the server stops the robot after owner_timeout (0.5 s here).
    assert not reply["ok"] and reply["error"]["code"] == ErrorCodes.Interrupted
    assert "stopped responding" in reply["error"]["message"]
    assert time.monotonic() - started < 1.5
    raw.close()


def test_move_home(station):
    with station.client() as a, station.client() as b:
        moving = in_background(a.move_home, 0.1)
        assert wait_until(lambda: b.is_executing)
        trajectory = np.repeat([b.get_current_joint_positions()], 2, axis=0)
        assert "executing a trajectory for" in raises(ErrorCodes.RobotBusy, b.run_joint_trajectory, trajectory, 0.01)
        assert "executing a trajectory for" in raises(ErrorCodes.RobotBusy, b.move_home)
        assert moving.result(10) == ErrorCodes.Success
        np.testing.assert_allclose(a.get_current_joint_positions(), protocol.HOME_JOINT_POSITIONS, atol=1e-4)
        assert "at most 0.3" in raises(ErrorCodes.JointVelocitiesAboveMaximumAllowed, a.move_home, 0.5)

        station.controller.sim_set_joint_positions(station.server.controller.snapshot()["state"]["q"] +
                                                   np.array([0.3, 0, 0, 0, 0, 0, 0]))
        moving = in_background(a.move_home, 0.1)
        assert wait_until(lambda: b.is_executing)
        b.stop()
        assert "stop() from" in raises(ErrorCodes.Interrupted, moving.result, 10)


def test_gripper(station):
    with station.client() as robot:
        gripper = robot.gripper
        assert gripper.move(0.04, 0.1)
        assert wait_until(lambda: abs(gripper.width - 0.04) < 1e-3)
        station.gripper.sim_set_object_width(0.03)
        assert gripper.grasp(0.03, 0.1, 20.0)
        assert wait_until(lambda: gripper.state.is_grasped)
        assert gripper.open(0.1)
        assert wait_until(lambda: abs(gripper.width - 0.08) < 1e-3)
        station.gripper.sim_set_object_width(-1.0)
        assert not gripper.grasp(0.03, 0.1, 20.0)  # nothing between the fingers
        assert robot.move_gripper(0.08)


def test_gripper_is_shared_one_command_at_a_time(station):
    with station.client() as a, station.client() as b:
        a.gripper.move(0.0, 0.1)
        moving = a.gripper.move_async(0.08, 0.02)  # 4 s at the gripper's speed, 1 s simulated here
        assert wait_until(lambda: a.gripper.state.width > 0.005)
        assert "gripper is busy" in raises(ErrorCodes.RobotBusy, b.gripper.move, 0.02, 0.1)
        q0 = b.get_current_joint_positions()
        trajectory = linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], duration=1.0)
        assert "gripper is moving" in raises(ErrorCodes.RobotBusy, b.run_joint_trajectory, trajectory, 0.01)
        assert b.gripper.stop()  # stop works for anyone
        assert moving.result(5) is False


def test_robot_not_ready(station):
    with station.client() as robot:
        station.controller.sim_set_robot_mode("user_stopped")
        assert wait_until(lambda: robot.get_current_robot_state().robot_mode == "user_stopped")
        q0 = robot.get_current_joint_positions()
        message = raises(ErrorCodes.RobotNotReady, robot.run_joint_trajectory,
                         linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], 1.0), 0.01)
        assert "user stop" in message


def test_recover_after_a_reflex(station):
    with station.client() as robot:
        q0 = robot.get_current_joint_positions()
        running = in_background(robot.run_joint_trajectory, linear_trajectory(q0, [0.3, 0, 0, 0, 0, 0, 0], 4.0), 0.01)
        assert wait_until(lambda: robot.is_executing)
        station.controller.sim_inject_reflex()
        with pytest.raises(RobotError) as error:
            running.result(10)
        assert error.value.code == ErrorCodes.ControlException
        assert wait_until(lambda: "cartesian_reflex" in robot.get_current_robot_state().current_errors)
        robot.recover()
        q1 = robot.get_current_joint_positions()
        assert robot.run_joint_trajectory(linear_trajectory(q1, [-0.1, 0, 0, 0, 0, 0, 0], 1.0), 0.01) == \
            ErrorCodes.Success


def test_pose_matches_the_reported_matrix(station):
    pytest.importorskip("spatialmath")
    with station.client() as robot:
        state = robot.get_current_robot_state()
        np.testing.assert_allclose(robot.get_current_pose().A, np.array(state.O_T_EE).reshape(4, 4).T, atol=1e-9)


def test_robot_is_released_after_the_last_client_leaves(station):
    with station.client():
        assert station.controller.snapshot()["connected"]
    assert wait_until(lambda: station.controller.snapshot()["status"] == "disconnected", timeout=3.0)


def test_client_side_checks(station):
    with station.client() as robot:
        raises(ErrorCodes.JointSizeNotSeven, robot.run_joint_trajectory, np.zeros((10, 6)), 0.01)
        raises(ErrorCodes.JointSizeNotSeven, robot.run_joint_trajectory, [[0.0] * 7, [0.0] * 6], 0.01)
        raises(ErrorCodes.NonFiniteValue, robot.run_joint_trajectory, np.full((3, 7), np.nan), 0.01)


def test_protocol_errors(station):
    raw = RawClient(station.port)
    raw.send({"op": "execute", "id": 1, "trajectory": [], "dt": 0.01})
    assert raw.reply(1)["error"]["code"] == ErrorCodes.InvalidRequest  # hello first
    raw.send(b"this is not json\n")
    assert raw.reply(None)["error"]["code"] == ErrorCodes.InvalidRequest
    raw.send({"op": "hello", "id": 2, "protocol": 999})
    assert raw.reply(2)["error"]["code"] == ErrorCodes.IncompatibleVersion
    hello(raw)
    raw.send({"op": "fly", "id": 3})
    assert raw.reply(3)["error"]["code"] == ErrorCodes.InvalidRequest
    raw.send({"op": "execute", "id": 4, "trajectory": [[0, 1], [2]], "dt": 0.01})
    assert raw.reply(4)["error"]["code"] == ErrorCodes.JointSizeNotSeven
    raw.send({"op": "execute", "id": 5, "trajectory": [[0.0] * 7] * 2, "dt": "fast"})
    assert raw.reply(5)["error"]["code"] == ErrorCodes.InvalidRequest
    raw.close()


def test_no_server(native):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    message = raises(ErrorCodes.ServerUnavailable, FrankaJointTrajectoryController, "127.0.0.1", port, timeout=1.0)
    assert "Cannot reach the robot server" in message


def test_robot_not_reachable_from_the_minipc(native):
    station, close = make_station(native)
    try:
        station.controller.sim_refuse_connections("libfranka: Connection refused (simulated)")
        message = raises(ErrorCodes.RobotNotConnected, station.client, timeout=1.0)
        assert "FCI" in message
    finally:
        close()


def test_separate_gripper_connection(station):
    with Gripper("127.0.0.1", station.port) as gripper:
        assert gripper.max_width == pytest.approx(0.08)
