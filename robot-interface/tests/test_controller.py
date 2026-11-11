"""The miniPC's controller, driving the simulated robot (no network)."""

import numpy as np
import pytest

from csc376_franky import protocol
from csc376_franky.protocol import ErrorCodes

from helpers import SIM_SPEED, linear_trajectory, wait_until


@pytest.fixture
def controller(native):
    controller = native.Controller.sim(native.Limits(), speed=SIM_SPEED)
    controller.set_connection_wanted(True)
    assert wait_until(lambda: controller.snapshot()["state"] is not None)
    yield controller
    controller.shutdown()


def q_now(controller):
    return np.array(controller.snapshot()["state"]["q"])


def result_of(controller, controller_id, timeout=10.0):
    found = []

    def finished():
        found[:] = [(ErrorCodes(code), message) for i, code, message in controller.snapshot()["results"]
                    if i == controller_id]
        return bool(found)

    assert wait_until(finished, timeout), "the command never finished"
    return found[0]


def start(controller, trajectory, dt=0.01):
    code, message, controller_id = controller.execute(np.asarray(trajectory), dt)
    assert ErrorCodes(code) == ErrorCodes.Success, message
    return controller_id


def move_home(controller, speed_factor=0.3):
    code, message, controller_id = controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, speed_factor)
    assert ErrorCodes(code) == ErrorCodes.Success, message
    return controller_id


def executing(controller):
    return wait_until(lambda: controller.snapshot()["status"] == "executing")


def test_not_connected_until_wanted(native):
    controller = native.Controller.sim(native.Limits(), speed=SIM_SPEED)
    try:
        snapshot = controller.snapshot()
        assert snapshot["status"] == "disconnected" and snapshot["state"] is None
        code, message, _ = controller.execute(np.repeat([native.READY_POSE], 2, axis=0), 0.01)
        assert ErrorCodes(code) == ErrorCodes.RobotNotConnected
    finally:
        controller.shutdown()


def test_executes_and_ends_at_the_last_point(controller):
    q0 = q_now(controller)
    trajectory = linear_trajectory(q0, [0.2, -0.1, 0, 0.1, 0, 0, 0], duration=1.0)
    assert result_of(controller, start(controller, trajectory)) == (ErrorCodes.Success, "")
    np.testing.assert_allclose(q_now(controller), trajectory[-1], atol=2e-3)
    assert controller.snapshot()["status"] == "idle"


def test_reports_progress(controller):
    trajectory = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=2.0)
    controller_id = start(controller, trajectory)
    assert wait_until(lambda: (controller.snapshot()["trajectory"] or {}).get("index", 0) > 50)
    progress = controller.snapshot()["trajectory"]
    assert progress["id"] == controller_id and progress["length"] == len(trajectory)
    assert progress["duration"] == pytest.approx(2.0) and not progress["stopping"]
    result_of(controller, controller_id)
    assert controller.snapshot()["trajectory"] is None


def test_refuses_anything_else_while_executing(controller):
    first = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=2.0)
    controller_id = start(controller, first)
    assert executing(controller)
    second = linear_trajectory(q_now(controller), [0.1, 0, 0, 0, 0, 0, 0], duration=1.0)
    code, message, _ = controller.execute(second, 0.01)
    assert ErrorCodes(code) == ErrorCodes.RobotBusy
    code, message, _ = controller.recover()
    assert ErrorCodes(code) == ErrorCodes.RobotBusy
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success


def test_refuses_a_trajectory_that_starts_elsewhere(controller):
    elsewhere = q_now(controller) + [0, 0, 0, 0.1, 0, 0, 0]
    code, message, _ = controller.execute(linear_trajectory(elsewhere, [0.1, 0, 0, 0, 0, 0, 0], 1.0), 0.01)
    assert ErrorCodes(code) == ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition


def test_refuses_a_plan_from_before_the_robot_was_moved_by_hand(controller):
    q0 = q_now(controller)
    trajectory = linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], 1.0)
    controller.sim_set_joint_positions(q0 + [0.2, 0, 0, 0, 0, 0, 0])
    assert wait_until(lambda: abs(q_now(controller)[0] - q0[0] - 0.2) < 1e-9)
    code, message, _ = controller.execute(trajectory, 0.01)
    assert ErrorCodes(code) == ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition


def test_stop_slows_to_rest_along_the_path(controller):
    q0 = q_now(controller)
    trajectory = linear_trajectory(q0, [0.5, 0, 0, 0, 0, 0, 0], duration=4.0)
    controller_id = start(controller, trajectory)
    assert wait_until(lambda: (controller.snapshot()["trajectory"] or {}).get("index", 0) > 100)
    controller.stop()
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.Interrupted
    q = q_now(controller)
    assert q0[0] < q[0] < trajectory[-1][0] - 0.1
    np.testing.assert_allclose(q[1:], q0[1:], atol=2e-3)


def test_stop_too_late_to_end_short_of_the_last_point_is_a_success(native):
    limits = native.Limits()
    limits.stop_time = 2.0  # slowing down covers half of this in trajectory time
    controller = native.Controller.sim(limits, speed=SIM_SPEED)
    try:
        controller.set_connection_wanted(True)
        assert wait_until(lambda: controller.snapshot()["state"] is not None)
        trajectory = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=3.0)
        controller_id = start(controller, trajectory)
        assert wait_until(lambda: (controller.snapshot()["trajectory"] or {}).get("time", 0) > 2.1, period=0.002)
        controller.stop()
        assert result_of(controller, controller_id) == (ErrorCodes.Success, "")
        np.testing.assert_allclose(q_now(controller), trajectory[-1], atol=2e-3)
    finally:
        controller.shutdown()


def test_a_finished_command_is_not_shown_running(controller):
    trajectory = linear_trajectory(q_now(controller), [0.05, 0, 0, 0, 0, 0, 0], duration=0.3)
    controller_id = start(controller, trajectory)
    seen = []

    def finished():
        seen[:] = [controller.snapshot()]
        return any(i == controller_id for i, _, _ in seen[0]["results"])

    # The first snapshot with the result already shows the robot idle.
    assert wait_until(finished, period=0.0)
    assert seen[0]["status"] == "idle" and seen[0]["trajectory"] is None


def test_tracking_error_stops_the_motion(controller):
    trajectory = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=2.0)
    controller_id = start(controller, trajectory)
    assert executing(controller)
    controller.sim_inject_disturbance(2, 0.08)
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.TrackingErrorTooLarge and "q[2]" in message


def test_reflex_needs_recovery(controller):
    trajectory = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=2.0)
    controller_id = start(controller, trajectory)
    assert executing(controller)
    controller.sim_inject_reflex()
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.ControlException and "reflex" in message
    snapshot = controller.snapshot()
    assert snapshot["state"]["robot_mode"] == "reflex" and snapshot["current_errors"] == ["cartesian_reflex"]

    again = linear_trajectory(q_now(controller), [-0.1, 0, 0, 0, 0, 0, 0], duration=1.0)
    code, message, _ = controller.execute(again, 0.01)
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady and "recover()" in message

    code, message, recovery_id = controller.recover()
    assert ErrorCodes(code) == ErrorCodes.Success
    assert result_of(controller, recovery_id)[0] == ErrorCodes.Success
    assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "idle")
    assert result_of(controller, start(controller, again))[0] == ErrorCodes.Success


def test_user_stop_is_reported(controller):
    controller.sim_set_robot_mode("user_stopped")
    assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "user_stopped")
    trajectory = linear_trajectory(q_now(controller), [0.1, 0, 0, 0, 0, 0, 0], 1.0)
    code, message, _ = controller.execute(trajectory, 0.01)
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady and "user stop" in message


def test_user_stop_pressed_before_connecting_is_reported(native):
    controller = native.Controller.sim(native.Limits(), speed=SIM_SPEED)
    try:
        controller.sim_set_robot_mode("user_stopped")
        controller.set_connection_wanted(True)
        assert wait_until(lambda: controller.snapshot()["state"] is not None)
        snapshot = controller.snapshot()
        assert snapshot["connected"] and snapshot["state"]["robot_mode"] == "user_stopped"
        trajectory = linear_trajectory(q_now(controller), [0.1, 0, 0, 0, 0, 0, 0], 1.0)
        code, message, _ = controller.execute(trajectory, 0.01)
        assert ErrorCodes(code) == ErrorCodes.RobotNotReady and "user stop" in message
        controller.sim_set_robot_mode("idle")
        assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "idle")
        assert result_of(controller, start(controller, trajectory))[0] == ErrorCodes.Success
    finally:
        controller.shutdown()


def test_connection_loss_during_a_trajectory(controller):
    trajectory = linear_trajectory(q_now(controller), [0.3, 0, 0, 0, 0, 0, 0], duration=2.0)
    controller_id = start(controller, trajectory)
    assert executing(controller)
    controller.sim_drop_connection()
    assert result_of(controller, controller_id)[0] == ErrorCodes.ConnectionLost
    # It reconnects by itself while a connection is wanted.
    assert wait_until(lambda: controller.snapshot()["state"] is not None)
    assert controller.snapshot()["status"] == "idle"


def test_unreachable_robot_reports_why(native):
    controller = native.Controller.sim(native.Limits(), speed=SIM_SPEED)
    try:
        controller.sim_refuse_connections("libfranka: Connection refused (simulated)")
        controller.set_connection_wanted(True)
        assert wait_until(lambda: "refused" in (controller.snapshot()["connection_error"] or ""))
        code, message, _ = controller.execute(np.repeat([native.READY_POSE], 2, axis=0), 0.01)
        assert ErrorCodes(code) == ErrorCodes.RobotNotConnected and "refused" in message
        controller.sim_refuse_connections("")
        assert wait_until(lambda: controller.snapshot()["state"] is not None)
    finally:
        controller.shutdown()


def test_release_disconnects(controller):
    controller.set_connection_wanted(False)
    assert wait_until(lambda: controller.snapshot()["status"] == "disconnected")
    assert controller.snapshot()["state"] is None


def test_end_effector_pose_matches_the_toolbox(controller):
    rtb = pytest.importorskip("roboticstoolbox")
    controller.sim_set_joint_positions([0.3, -0.5, 0.2, -2.0, 0.4, 1.8, 0.1])
    assert wait_until(lambda: q_now(controller)[0] == pytest.approx(0.3))
    state = controller.snapshot()["state"]
    pose = np.array(state["O_T_EE"]).reshape(4, 4).T
    np.testing.assert_allclose(pose, rtb.models.Panda().fkine(state["q"]).A, atol=1e-9)


def test_motion_generator_rests_at_both_ends_within_its_limits(native):
    q_start, q_goal, speed_factor = np.array(native.READY_POSE), np.array(protocol.HOME_JOINT_POSITIONS), 0.3
    generator = native.JointMotionGenerator(q_start, q_goal, speed_factor)
    t = np.arange(0.0, generator.duration + 0.002, 0.001)
    q = np.array([generator.positions(ti)[1] for ti in t])
    dq = np.diff(q, axis=0) / 0.001
    np.testing.assert_allclose(q[0], q_start, atol=1e-12)
    np.testing.assert_allclose(q[-1], q_goal, atol=1e-12)
    assert generator.positions(generator.duration)[0] and not generator.positions(generator.duration / 2)[0]
    assert np.abs(dq[0]).max() < 1e-3 and np.abs(dq[-1]).max() < 1e-3
    assert (np.abs(dq) <= speed_factor * np.array([2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]) + 1e-6).all()
    assert np.abs(np.diff(dq, axis=0) / 0.001).max() <= 5.0 * speed_factor + 1e-3


def test_moves_home_like_libfranka(controller):
    controller_id = move_home(controller)
    assert executing(controller)
    progress = controller.snapshot()["trajectory"]
    assert progress["id"] == controller_id and progress["duration"] > 0
    assert result_of(controller, controller_id) == (ErrorCodes.Success, "")
    np.testing.assert_allclose(q_now(controller), protocol.HOME_JOINT_POSITIONS, atol=1e-9)
    assert controller.snapshot()["status"] == "idle"
    # Already there: finishes right away.
    assert result_of(controller, move_home(controller)) == (ErrorCodes.Success, "")


def test_move_home_refuses_what_a_trajectory_would_be_refused_for(native, controller):
    code, message, _ = controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, 0.5)
    assert ErrorCodes(code) == ErrorCodes.JointVelocitiesAboveMaximumAllowed and "0.3" in message
    code, message, _ = controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, 0.0)
    assert ErrorCodes(code) == ErrorCodes.JointVelocitiesAboveMaximumAllowed
    code, message, _ = controller.move_to_joint_positions([0, 0, 0, 0, 0, 0, 0], 0.2)
    assert ErrorCodes(code) == ErrorCodes.JointLimitsExceeded and "q_goal[3]" in message
    controller.sim_set_robot_mode("user_stopped")
    assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "user_stopped")
    code, message, _ = controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, 0.2)
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady
    controller.sim_set_robot_mode("idle")
    assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "idle")
    controller_id = move_home(controller)
    assert executing(controller)
    code, message, _ = controller.execute(np.repeat([q_now(controller)], 2, axis=0), 0.01)
    assert ErrorCodes(code) == ErrorCodes.RobotBusy
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success


def test_stopping_move_home_rests_on_the_way(controller):
    q0 = q_now(controller)
    controller_id = move_home(controller, speed_factor=0.1)
    assert wait_until(lambda: (controller.snapshot()["trajectory"] or {}).get("time", 0) > 1.0)
    controller.stop()
    assert result_of(controller, controller_id)[0] == ErrorCodes.Interrupted
    q, home = q_now(controller), np.array(protocol.HOME_JOINT_POSITIONS)
    assert np.abs(q - q0).max() > 0.05 and np.abs(q - home).max() > 0.05
    assert np.abs(controller.snapshot()["state"]["dq"]).max() < 1e-6


def test_tracking_error_stops_move_home(controller):
    controller_id = move_home(controller, speed_factor=0.1)
    assert executing(controller)
    controller.sim_inject_disturbance(2, 0.08)
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.TrackingErrorTooLarge and "q[2]" in message
