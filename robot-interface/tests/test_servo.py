"""Velocity control (servoing): the miniPC's twist follower, its controller and the clients."""

import socket
import time

import numpy as np
import pytest

from csc376_franky import ErrorCodes, RobotError, protocol
from helpers import SIM_SPEED, linear_trajectory, make_station, wait_until

READY = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, np.pi / 4])
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
DQ_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
DDQ_MAX = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
CYCLE = 0.001


def pose(native, q):
    return np.array(native.sim_pose(q)).reshape(4, 4).T


class Drive:
    """A twist follower moving joints that follow its commands exactly, at 1 kHz."""

    def __init__(self, native, q=READY, **limits):
        self.native = native
        self.limits = native.Limits()
        for name, value in limits.items():
            setattr(self.limits, name, value)
        self.follower = native.TwistFollower(self.limits)
        self.q = np.array(q, dtype=float)
        self.dq = np.zeros(7)
        # Per cycle: the joint velocities commanded, and the twist they give the end effector.
        self.commands = []
        self.twists = []
        self.limited = set()

    def run(self, seconds, twist=None, frame="end_effector", stop=False):
        """Run for seconds; with a twist, a client sends it again every cycle."""
        for _ in range(round(seconds / CYCLE)):
            if twist is not None:
                self.follower.set_target(twist, frame)
            jacobian = self.native.sim_jacobian(self.q)
            self.follower.step(
                CYCLE, self.q, self.dq, self.native.sim_pose(self.q), jacobian, stop
            )
            self.dq = np.array(self.follower.dq_c)
            self.q = self.q + self.dq * CYCLE
            self.commands.append(self.dq)
            self.twists.append(jacobian @ self.dq)
            self.limited.update(self.follower.limited)
        return self

    @property
    def position(self):
        return pose(self.native, self.q)[:3, 3]


def test_simulated_jacobian_matches_the_toolbox(native):
    rtb = pytest.importorskip("roboticstoolbox")
    panda = rtb.models.Panda()
    rng = np.random.default_rng(376)
    for q in rng.uniform(Q_MIN, Q_MAX, size=(50, 7)):
        np.testing.assert_allclose(native.sim_jacobian(q), panda.jacob0(q), atol=1e-12)
        np.testing.assert_allclose(pose(native, q), panda.fkine(q).A, atol=1e-12)


def test_follows_a_twist_in_the_base_frame(native):
    twist = np.array([0.05, -0.02, 0.03, 0.1, 0.0, -0.2])
    drive = Drive(native)
    start = drive.position
    drive.run(1.0, twist, "base")
    # Once up to speed the end effector moves with the twist (the damping of
    # the Jacobian's inverse costs a little of it), and nothing held it back.
    np.testing.assert_allclose(drive.twists[-1], twist, atol=0.02 * np.abs(twist).max())
    np.testing.assert_allclose(drive.follower.twist, twist, atol=1e-12)
    assert drive.limited == set()
    moved = drive.position - start
    assert np.linalg.norm(moved - 0.85 * twist[:3]) < 0.1 * np.linalg.norm(twist[:3])


def test_a_twist_in_the_end_effector_frame_turns_with_the_hand(native):
    # Along the hand's z axis, which points down and forward at the ready pose,
    # while the hand turns about its own y axis.
    twist = np.array([0.0, 0.0, 0.05, 0.0, 0.3, 0.0])
    drive = Drive(native).run(1.0, twist, "end_effector")
    rotation = pose(native, drive.q)[:3, :3]
    expected = np.concatenate([rotation @ twist[:3], rotation @ twist[3:]])
    np.testing.assert_allclose(drive.twists[-1], expected, atol=3e-3)
    # It turned about 0.3 rad/s x 0.85 s: the twist was not fixed in the base frame.
    start = pose(native, READY)[:3, :3]
    angle = np.arccos((np.trace(start.T @ rotation) - 1) / 2)
    assert angle == pytest.approx(0.3 * 0.85, abs=0.03)


def test_a_twist_that_is_too_fast_is_scaled_down_and_keeps_its_direction(native):
    twist = np.array([1.0, 1.0, 0.0, 0.0, 0.0, 0.2])
    drive = Drive(native).run(1.0, twist, "base")
    followed = np.array(drive.follower.twist)
    assert np.linalg.norm(followed[:3]) == pytest.approx(
        drive.limits.servo_linear_velocity
    )
    np.testing.assert_allclose(followed, twist * followed[0] / twist[0], atol=1e-12)
    assert "speed" in drive.limited

    turning = Drive(native).run(1.0, [0, 0, 0, 0, 0, 5.0], "base")
    assert turning.follower.twist[5] == pytest.approx(
        drive.limits.servo_angular_velocity
    )


def test_speed_changes_gradually(native):
    drive = Drive(native)
    limits = drive.limits
    # From rest to the fastest twist, a reversal, and a stop.
    drive.run(0.5, [1.0, 0, 0, 0, 0, 1.0], "base")
    drive.run(0.8, [-1.0, 0, 0, 0, 0, -1.0], "base")
    drive.run(0.5, [0, 0, 0, 0, 0, 0], "base")
    twists = np.array(drive.twists)
    commands = np.array(drive.commands)
    assert np.abs(twists[:, 0]).max() == pytest.approx(
        limits.servo_linear_velocity, rel=0.02
    )
    # The twist takes servo_ramp_time from rest to its limit...
    linear = np.abs(np.diff(twists[:, 0])) / CYCLE
    angular = np.abs(np.diff(twists[:, 5])) / CYCLE
    assert linear.max() < 1.1 * limits.servo_linear_velocity / limits.servo_ramp_time
    assert angular.max() < 1.1 * limits.servo_angular_velocity / limits.servo_ramp_time
    # ...and no joint changes speed faster than half of what a trajectory may
    # on the joint that accelerates the slowest.
    acceleration = np.abs(np.diff(commands, axis=0)) / CYCLE
    assert acceleration.max() <= 0.5 * limits.acceleration_scale * DDQ_MAX.min() + 1e-9
    assert (np.abs(commands) <= limits.velocity_scale * DQ_MAX + 1e-12).all()
    assert (commands[-1] == 0.0).all()


def test_the_robot_stops_when_the_twists_stop_coming(native):
    drive = Drive(native)
    twist = [0.05, 0, 0, 0, 0, 0]
    drive.run(0.5, twist, "base")
    assert drive.twists[-1][0] == pytest.approx(0.05, rel=0.02)
    # The client goes quiet: after servo_timeout the twist is taken to be zero.
    drive.limited.clear()
    drive.run(drive.limits.servo_timeout - 0.01)
    assert drive.twists[-1][0] == pytest.approx(0.05, rel=0.02) and not drive.limited
    drive.run(drive.limits.servo_ramp_time + 0.2)
    assert (drive.commands[-1] == 0.0).all() and "timeout" in drive.limited
    assert drive.follower.phase == "running"
    # It moves again with the next twist.
    drive.run(0.5, twist, "base")
    assert drive.twists[-1][0] == pytest.approx(0.05, rel=0.02)


def test_the_end_effector_is_not_driven_below_the_minimum_height(native):
    drive = Drive(native, servo_min_height=0.3)
    height = drive.position[2]
    assert height > 0.4
    drive.run(4.0, [0.0, 0.0, -0.1, 0.0, 0.0, 0.0], "base")
    lowest = min(pose(native, drive.q)[2, 3], drive.position[2])
    assert 0.3 - 1e-3 < lowest < 0.31 and "height" in drive.limited
    assert np.abs(drive.commands[-1]).max() < 1e-3
    # Along the floor and up again it moves freely.
    drive.limited.clear()
    drive.run(1.0, [0.0, 0.05, 0.05, 0.0, 0.0, 0.0], "base")
    assert drive.twists[-1][1] == pytest.approx(0.05, rel=0.02)
    assert drive.twists[-1][2] == pytest.approx(0.05, rel=0.02) and not drive.limited


def test_the_robot_stops_before_a_joint_limit(native):
    # Turning the hand about its own z axis turns the last joint, 1.2 rad from its limit.
    q = READY.copy()
    q[6] = Q_MAX[6] - 1.2
    drive = Drive(native, q)
    margin = drive.limits.servo_joint_limit_margin
    drive.run(6.0, [0, 0, 0, 0, 0, 0.4], "end_effector")
    assert drive.limited == {"joint_position"}
    positions = q[6] + np.cumsum(np.array(drive.commands)[:, 6]) * CYCLE
    assert positions.max() <= Q_MAX[6] - margin + 1e-9
    assert drive.q[6] == pytest.approx(Q_MAX[6] - margin, abs=2e-3)
    assert np.abs(drive.commands[-1]).max() < 1e-3
    # Away from the limit it moves freely, once the twist has turned around.
    drive.run(1.0, [0, 0, 0, 0, 0, -0.4], "end_effector")
    drive.limited.clear()
    drive.run(0.5, [0, 0, 0, 0, 0, -0.4], "end_effector")
    assert drive.commands[-1][6] == pytest.approx(-0.4, rel=0.05) and not drive.limited


def test_a_joint_at_its_limit_holds_the_arm_back_rather_than_bending_the_motion(native):
    # With the elbow at its limit, pulling the hand straight in needs the elbow
    # to go further. Without it the other joints would swing the hand down, at
    # more than the speed limit: the arm stays where it is instead.
    q = np.array([0.0, 0.6, 0.0, -0.12, 0.0, 0.72, np.pi / 4])
    jacobian = native.sim_jacobian(q)
    needed = np.linalg.pinv(jacobian) @ [-0.1, 0, 0, 0, 0, 0]
    without_the_elbow = jacobian @ np.where(np.arange(7) == 3, 0.0, needed)
    assert needed[3] > 0.5 and without_the_elbow[2] < -0.2
    drive = Drive(native, q).run(1.0, [-0.1, 0, 0, 0, 0, 0], "base")
    assert "joint_position" in drive.limited
    # The elbow uses up the 0.2 mrad it has left, and that is all that moves.
    assert (
        np.abs(drive.twists).max() < 5e-3 and (np.array(drive.commands[-1]) == 0).all()
    )
    np.testing.assert_allclose(drive.position, pose(native, q)[:3, 3], atol=2e-4)
    # Pushing it out again moves the elbow away from its limit, and is followed
    # once the twist has turned around.
    drive.run(0.6, [0.02, 0, 0, 0, 0, 0], "base")
    drive.limited.clear()
    drive.run(0.4, [0.02, 0, 0, 0, 0, 0], "base")
    assert drive.twists[-1][0] > 0.01 and "joint_position" not in drive.limited


def test_joint_velocities_stay_within_their_limits_near_a_singularity(native):
    # With the arm almost stretched out, pulling the hand straight in takes
    # joint velocities far beyond the limits.
    q = np.array([0.0, 0.3, 0.0, -0.5, 0.0, 0.8, np.pi / 4])
    twist = np.array([-0.1, 0, 0, 0, 0, 0])
    jacobian = native.sim_jacobian(q)
    assert np.abs(np.linalg.pinv(jacobian) @ twist).max() > 5.0
    drive = Drive(native, q).run(0.6, twist, "base")
    commands = np.array(drive.commands)
    assert drive.limited == {"joint_velocity"}
    assert np.isfinite(commands).all()
    assert (np.abs(commands) <= drive.limits.velocity_scale * DQ_MAX + 1e-12).all()
    # The joints are slowed down together, so the hand goes the way it was
    # asked to, more slowly, and never faster than it was asked to.
    twists = np.array(drive.twists)
    assert (np.linalg.norm(twists[:, :3], axis=1) <= 0.1 + 1e-9).all()
    assert twists[-1][0] < -0.05 and np.abs(twists[-1][1:]).max() < 0.1 * abs(
        twists[-1][0]
    )


def test_near_the_end_of_its_reach_the_robot_slows_down_rather_than_going_astray(
    native,
):
    # Stretched out, the arm cannot move its hand further out: what the damped
    # inverse gives instead lets the hand sink. The robot slows down until
    # that is small, and says so.
    drive = Drive(native).run(8.0, [0.1, 0, 0, 0, 0, 0], "base")
    assert drive.position[0] > 0.75 and "singularity" in drive.limited
    twists = np.array(drive.twists)
    assert np.abs(twists[:, 1:3]).max() < 0.011
    assert np.abs(twists[-1]).max() < 0.011
    # Back in it is not held back, although the way back is as bent.
    drive.run(0.5, [-0.1, 0, 0, 0, 0, 0], "base")
    drive.limited.clear()
    drive.run(0.2, [-0.1, 0, 0, 0, 0, 0], "base")
    assert "singularity" not in drive.limited and drive.twists[-1][0] < -0.01


def test_the_minimum_height_holds_whatever_makes_the_hand_go_down(native):
    # Going down while the twist changes sideways all the time, and then far
    # out, where the hand sinks without being asked to.
    drive = Drive(native, servo_min_height=0.3)
    lowest = drive.position[2]
    for i in range(40):
        drive.run(0.15, [0.087 * (-1) ** i, 0, -0.05, 0, 0, 0], "base")
        lowest = min(lowest, drive.position[2])
    for _ in range(40):
        drive.run(1.0, [0.087, 0, -0.05, 0, 0, 0], "base")
        lowest = min(lowest, drive.position[2])
    assert 0.3 - 1e-6 < lowest < 0.301 and "height" in drive.limited


def test_a_joint_limit_holds_while_the_twist_keeps_changing(native):
    rng = np.random.default_rng(376)
    for _ in range(20):
        q = rng.uniform(Q_MIN + 0.1, Q_MAX - 0.1)
        drive = Drive(native, q)
        margin = drive.limits.servo_joint_limit_margin
        for _ in range(8):
            twist = rng.normal(size=6) * [0.2, 0.2, 0.2, 1.0, 1.0, 1.0]
            drive.run(rng.uniform(0.02, 1.0), twist, "base")
            assert (drive.q <= Q_MAX - margin + 1e-9).all()
            assert (drive.q >= Q_MIN + margin - 1e-9).all()
        commands = np.array(drive.commands)
        # A joint's velocity changes no faster than the acceleration limit
        # and what braking may add to it.
        assert np.abs(np.diff(commands, axis=0)).max() <= 1.5 * 3.75 * CYCLE + 1e-12


def test_stop_slows_down_to_rest_and_then_it_is_done(native):
    drive = Drive(native).run(0.6, [0.1, 0, 0, 0, 0, 0.5], "base")
    assert drive.follower.phase == "running"
    before = len(drive.commands)
    # Twists that still arrive after the stop are ignored.
    drive.run(1.0, [0.1, 0, 0, 0, 0, 0.5], "base", stop=True)
    assert drive.follower.phase == "done"
    commands = np.array(drive.commands[before:])
    moving = np.flatnonzero(np.abs(commands).max(axis=1) > 0)
    # At rest within the ramp time, then done once it has been still for a moment.
    assert moving[-1] * CYCLE < drive.limits.servo_ramp_time + 0.2
    assert (commands[-1] == 0.0).all()
    acceleration = np.abs(np.diff(commands, axis=0)) / CYCLE
    assert acceleration.max() <= 0.5 * DDQ_MAX.min() + 1e-9


# ---- the controller ----------------------------------------------------------


@pytest.fixture
def controller(native):
    controller = native.Controller.sim(native.Limits(), speed=SIM_SPEED)
    controller.set_connection_wanted(True)
    assert wait_until(lambda: controller.snapshot()["state"] is not None)
    yield controller
    controller.shutdown()


def result_of(controller, controller_id, timeout=10.0):
    found = []

    def finished():
        found[:] = [
            (ErrorCodes(code), message)
            for i, code, message, _ in controller.snapshot()["results"]
            if i == controller_id
        ]
        return bool(found)

    assert wait_until(finished, timeout), "the command never finished"
    return found[0]


def start_servo(controller):
    code, message, controller_id = controller.start_servo()
    assert ErrorCodes(code) == ErrorCodes.Success, message
    assert wait_until(
        lambda: (controller.snapshot()["trajectory"] or {}).get("kind") == "servo"
    )
    return controller_id


def steer(controller, controller_id, twist, frame="base", seconds=0.3):
    """Send the twist every few milliseconds for seconds of wall time."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        code, message = controller.set_servo_twist(controller_id, twist, frame)
        assert ErrorCodes(code) == ErrorCodes.Success, message
        time.sleep(0.005)


def position(controller):
    return np.array(controller.snapshot()["state"]["O_T_EE"])[12:15]


def test_velocity_control_moves_the_end_effector_until_it_is_ended(controller):
    start = position(controller)
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.0, 0.05, 0.0, 0.0, 0.0, 0.0])
    progress = controller.snapshot()["trajectory"]
    assert progress["id"] == controller_id and progress["kind"] == "servo"
    assert progress["twist"][1] == pytest.approx(0.05) and progress["limited"] == []
    assert not progress["stopping"]
    controller.end_servo(controller_id)
    assert result_of(controller, controller_id) == (ErrorCodes.Success, "")
    snapshot = controller.snapshot()
    assert snapshot["status"] == "idle" and snapshot["trajectory"] is None
    assert np.abs(snapshot["state"]["dq"]).max() == 0.0
    # 0.3 s of wall time at the simulation's speed, less the time to get up to speed.
    moved = position(controller) - start
    assert 0.03 < moved[1] < 0.08 and abs(moved[0]) < 2e-3 and abs(moved[2]) < 2e-3


def test_stop_interrupts_velocity_control(controller):
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.stop()
    code, _ = result_of(controller, controller_id)
    assert code == ErrorCodes.Interrupted
    assert np.abs(controller.snapshot()["state"]["dq"]).max() == 0.0


def test_a_stop_together_with_the_end_is_an_interruption(controller):
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.stop()
    controller.end_servo(controller_id)
    assert result_of(controller, controller_id)[0] == ErrorCodes.Interrupted


def test_refuses_a_twist_too_large_to_scale(controller):
    controller_id = start_servo(controller)
    code, message = controller.set_servo_twist(
        controller_id, [1.7e308, 0.0, 1.7e308, 0.0, 0.0, 0.0], "end_effector"
    )
    assert code == ErrorCodes.NonFiniteValue and "twist[0]" in message
    controller.end_servo(controller_id)
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success
    assert np.isfinite(controller.snapshot()["state"]["q"]).all()


def test_ending_before_a_stop_is_still_a_success(controller):
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.end_servo(controller_id)
    assert wait_until(
        lambda: (controller.snapshot()["trajectory"] or {}).get("stopping", True)
    )
    controller.stop()
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success


def test_refuses_anything_else_under_velocity_control(controller):
    controller_id = start_servo(controller)
    q = np.array(controller.snapshot()["state"]["q"])
    code, _, _ = controller.execute(
        linear_trajectory(q, [0.1, 0, 0, 0, 0, 0, 0], 1.0), 0.01
    )
    assert ErrorCodes(code) == ErrorCodes.RobotBusy
    for refused in (controller.start_servo, controller.recover):
        assert ErrorCodes(refused()[0]) == ErrorCodes.RobotBusy
    code, _, _ = controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, 0.2)
    assert ErrorCodes(code) == ErrorCodes.RobotBusy
    controller.end_servo(controller_id)
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success


def test_no_velocity_control_while_a_trajectory_runs(controller):
    q = np.array(controller.snapshot()["state"]["q"])
    code, message, trajectory_id = controller.execute(
        linear_trajectory(q, [0.3, 0, 0, 0, 0, 0, 0], duration=2.0), 0.01
    )
    assert ErrorCodes(code) == ErrorCodes.Success, message
    assert wait_until(lambda: controller.snapshot()["status"] == "executing")
    assert controller.snapshot()["trajectory"]["kind"] == "trajectory"
    assert ErrorCodes(controller.start_servo()[0]) == ErrorCodes.RobotBusy
    # A twist is not for a trajectory, even with the trajectory's id.
    code, _ = controller.set_servo_twist(trajectory_id, [0.05, 0, 0, 0, 0, 0], "base")
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady
    assert result_of(controller, trajectory_id)[0] == ErrorCodes.Success


def test_twists_are_only_taken_for_the_velocity_control_that_is_running(controller):
    code, _ = controller.set_servo_twist(1, [0.05, 0, 0, 0, 0, 0], "base")
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady
    first = start_servo(controller)
    code, message = controller.set_servo_twist(
        first, [0.05, 0, float("nan"), 0, 0, 0], "base"
    )
    assert ErrorCodes(code) == ErrorCodes.NonFiniteValue and "twist[2]" in message
    with pytest.raises(ValueError):
        controller.set_servo_twist(first, [0.05, 0, 0, 0, 0, 0], "camera")
    controller.end_servo(first)
    assert result_of(controller, first)[0] == ErrorCodes.Success
    # Neither a twist nor an end for the one that is over reaches the next one.
    second = start_servo(controller)
    code, _ = controller.set_servo_twist(first, [0.05, 0, 0, 0, 0, 0], "base")
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady
    controller.end_servo(first)
    time.sleep(0.2)
    progress = controller.snapshot()["trajectory"]
    assert progress["id"] == second and not progress["stopping"]
    assert progress["twist"] == [0.0] * 6
    controller.end_servo(second)
    assert result_of(controller, second)[0] == ErrorCodes.Success


def test_velocity_control_starts_at_rest(controller):
    first = start_servo(controller)
    steer(controller, first, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.end_servo(first)
    assert result_of(controller, first)[0] == ErrorCodes.Success
    start = position(controller)
    second = start_servo(controller)
    time.sleep(0.2)
    assert controller.snapshot()["trajectory"]["twist"] == [0.0] * 6
    np.testing.assert_allclose(position(controller), start, atol=1e-9)
    controller.end_servo(second)
    assert result_of(controller, second)[0] == ErrorCodes.Success


def test_a_reflex_ends_velocity_control(controller):
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.sim_inject_reflex()
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.ControlException and "reflex" in message
    code, message, _ = controller.start_servo()
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady and "recover()" in message
    _, _, recovery_id = controller.recover()
    assert result_of(controller, recovery_id)[0] == ErrorCodes.Success
    assert wait_until(lambda: controller.snapshot()["state"]["robot_mode"] == "idle")
    controller_id = start_servo(controller)
    controller.end_servo(controller_id)
    assert result_of(controller, controller_id)[0] == ErrorCodes.Success


def test_a_tracking_error_ends_velocity_control(controller):
    controller_id = start_servo(controller)
    steer(controller, controller_id, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
    controller.sim_inject_disturbance(2, 0.08)
    code, message = result_of(controller, controller_id)
    assert code == ErrorCodes.TrackingErrorTooLarge and "q[2]" in message


def test_connection_loss_ends_velocity_control(controller):
    controller_id = start_servo(controller)
    controller.sim_drop_connection()
    assert result_of(controller, controller_id)[0] == ErrorCodes.ConnectionLost
    assert wait_until(lambda: controller.snapshot()["status"] == "idle")


def test_no_velocity_control_while_the_user_stop_is_pressed(controller):
    controller.sim_set_robot_mode("user_stopped")
    assert wait_until(
        lambda: controller.snapshot()["state"]["robot_mode"] == "user_stopped"
    )
    code, message, _ = controller.start_servo()
    assert ErrorCodes(code) == ErrorCodes.RobotNotReady and "user stop" in message


# ---- the server and its clients ---------------------------------------------


def patient(native):
    """Limits with a timeout that the tests' own pauses do not run into."""
    limits = native.Limits()
    limits.servo_timeout = 4.0  # 1 s of wall time at the simulation's speed
    return limits


@pytest.fixture
def station(native):
    station, close = make_station(native, patient(native))
    yield station
    close()


def raises(code, function, *args, **kwargs):
    with pytest.raises(RobotError) as error:
        function(*args, **kwargs)
    assert error.value.code == code, error.value
    return error.value.message


def tcp(robot):
    """Where the end effector is, without spatialmath (the miniPC has none)."""
    return np.array(robot.get_current_robot_state().O_T_EE)[12:15]


def send(servo, twist, frame="base", seconds=0.3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        servo.set_twist(twist, frame)
        time.sleep(0.01)


def test_servo_follows_the_twists_of_its_client(station):
    with station.client() as robot:
        assert "servo" in robot.server_info["features"]
        assert robot.server_info["limits"]["servo_linear_velocity"] == pytest.approx(
            0.1
        )
        start = tcp(robot)
        with robot.start_servo() as servo:
            assert servo.active and servo.error is None and robot.is_executing
            send(servo, np.array([0.0, 0.0, -0.05, 0.0, 0.0, 0.0]))
            assert wait_until(lambda: servo.status["twist"][2] == pytest.approx(-0.05))
            assert servo.limited == ()
        assert not servo.active and servo.error is None and servo.status is None
        assert robot.wait_until_idle(timeout=5.0)
        moved = tcp(robot) - start
        assert -0.08 < moved[2] < -0.03 and np.abs(moved[:2]).max() < 2e-3
        # It is over: another twist says so, another stop is harmless.
        assert "has ended" in raises(ErrorCodes.Interrupted, servo.set_twist, [0] * 6)
        assert servo.stop() is False
        # The robot is free again, from where it stopped.
        q = robot.get_current_joint_positions()
        assert (
            robot.run_joint_trajectory(
                linear_trajectory(q, [0.05, 0, 0, 0, 0, 0, 0], 0.5), 0.01
            )
            == ErrorCodes.Success
        )


def test_twists_are_checked_before_they_are_sent(station):
    with station.client() as robot, robot.start_servo() as servo:
        raises(ErrorCodes.InvalidRequest, servo.set_twist, [0.0] * 5)
        raises(ErrorCodes.InvalidRequest, servo.set_twist, "fast")
        raises(ErrorCodes.NonFiniteValue, servo.set_twist, [0, 0, np.inf, 0, 0, 0])
        assert "end_effector, base" in raises(
            ErrorCodes.InvalidRequest, servo.set_twist, [0.0] * 6, "camera"
        )
        assert servo.active


def test_the_robot_is_busy_for_the_other_client(station):
    with station.client() as a, station.client() as b:
        with a.start_servo():
            assert wait_until(lambda: b.is_executing)
            q = b.get_current_joint_positions()
            message = raises(
                ErrorCodes.RobotBusy,
                b.run_joint_trajectory,
                np.repeat([q], 2, axis=0),
                0.01,
            )
            assert "under velocity control for" in message and "127.0.0.1" in message
            assert "under velocity control for" in raises(
                ErrorCodes.RobotBusy, b.start_servo
            )
            raises(ErrorCodes.RobotBusy, b.move_home)
            assert "already under velocity control" in raises(
                ErrorCodes.RobotBusy, a.start_servo
            )
        assert b.wait_until_idle(timeout=5.0)
        with b.start_servo() as servo:
            assert servo.active


def test_any_client_can_stop_velocity_control(station):
    with station.client() as a, station.client() as b:
        servo = a.start_servo()
        send(servo, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
        assert b.stop() is True
        assert wait_until(lambda: not servo.active)
        assert servo.error.code == ErrorCodes.Interrupted
        message = raises(ErrorCodes.Interrupted, servo.set_twist, [0.05, 0, 0, 0, 0, 0])
        assert "velocity control was stopped by stop() from" in message
        assert a.wait_until_idle(timeout=5.0)


def test_the_with_block_says_when_something_else_ended_velocity_control(station):
    with station.client() as a, station.client() as b:
        with pytest.raises(RobotError) as error, a.start_servo() as servo:
            send(servo, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
            assert b.stop() is True
            assert wait_until(lambda: not servo.active)
        assert error.value.code == ErrorCodes.Interrupted
        assert "stopped by stop() from" in error.value.message


def test_twists_are_not_sent_faster_than_the_server_takes_them(station):
    with station.client() as robot, robot.start_servo() as servo:
        start = time.monotonic()
        for _ in range(200):
            servo.set_twist([0.0] * 6)
        assert time.monotonic() - start > 0.19
        # So the twist that counts is the last one, and stop is prompt.
        start = time.monotonic()
        assert servo.stop() is True
        assert time.monotonic() - start < 1.0


def test_a_twist_of_numbers_too_large_is_refused(station):
    with station.client() as robot, robot.start_servo() as servo:
        for huge in (1.7e308, 2**70):
            message = raises(
                ErrorCodes.InvalidRequest,
                robot._connection.request,
                "servo_twist",
                twist=[huge, 0, 0, 0, 0, 0],
            )
            assert "6 finite numbers" in message
        assert servo.active


def test_only_its_client_steers_velocity_control(station):
    def request(robot, op, **fields):
        return robot._connection.request(op, **fields)

    with station.client() as a, station.client() as b:
        twist = [0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
        message = raises(
            ErrorCodes.InvalidRequest, request, a, "servo_twist", twist=twist
        )
        assert "servo_start" in message
        assert request(a, "servo_stop") == {"was_active": False}
        with a.start_servo() as servo:
            start = tcp(a)
            message = raises(
                ErrorCodes.RobotBusy, request, b, "servo_twist", twist=twist
            )
            assert "under velocity control for" in message
            assert request(b, "servo_stop") == {"was_active": False}
            time.sleep(0.2)
            assert servo.active and servo.status["twist"] == [0.0] * 6
            np.testing.assert_allclose(tcp(a), start, atol=1e-9)
            for fields in (
                {},
                {"twist": [0.0] * 7},
                {"twist": [0.0, 0.0, 0.0, 0.0, 0.0, "fast"]},
                {"twist": [0.0, 0.0, 0.0, 0.0, 0.0, True]},
                {"twist": twist, "frame": "camera"},
            ):
                raises(ErrorCodes.InvalidRequest, request, a, "servo_twist", **fields)
            assert request(a, "servo_twist", twist=twist, frame="base") == {}
            assert wait_until(lambda: servo.status["twist"][0] == pytest.approx(0.05))


def test_the_robot_stops_when_its_client_stops_sending_twists(native):
    station, close = make_station(native)  # the default timeout: 0.25 s
    try:
        with station.client() as robot, robot.start_servo() as servo:
            servo.set_twist([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], "base")
            assert wait_until(lambda: "timeout" in servo.limited)
            assert wait_until(lambda: servo.status["twist"] == [0.0] * 6)
            # Velocity control goes on, and the next twist moves the robot again.
            assert servo.active
            start = tcp(robot)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                servo.set_twist([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], "base")
                time.sleep(0.002)
            assert tcp(robot)[0] - start[0] > 0.01
    finally:
        close()


def test_disconnecting_stops_velocity_control(station):
    with station.client() as watcher:
        robot = station.client()
        servo = robot.start_servo()
        send(servo, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
        robot.close()
        assert watcher.wait_until_idle(timeout=10.0)
        assert np.abs(watcher.get_current_robot_state().dq).max() == 0.0
        assert not servo.active and servo.error.code == ErrorCodes.ConnectionLost
        raises(ErrorCodes.ConnectionLost, servo.set_twist, [0.0] * 6)


def test_velocity_control_of_a_silent_client_is_stopped(station):
    sock = socket.create_connection(("127.0.0.1", station.port), timeout=5)
    stream = sock.makefile("rb")

    def reply(request_id):
        while True:
            message = protocol.decode(stream.readline())
            if message.get("id") == request_id:
                return message

    sock.sendall(
        protocol.encode({"op": "hello", "id": 1, "protocol": protocol.PROTOCOL_VERSION})
    )
    assert reply(1)["ok"]
    # Once the server has the robot's state, the robot is connected.
    while True:
        robot = protocol.decode(stream.readline()).get("robot")
        if isinstance(robot, dict) and robot.get("q") is not None:
            break
    sock.sendall(protocol.encode({"op": "servo_start", "id": 2}))
    sock.sendall(protocol.encode({"op": "servo_twist", "twist": [0.05, 0, 0, 0, 0, 0]}))
    # No heartbeats: the server stops the robot after owner_timeout (0.5 s here).
    answer = reply(2)
    assert not answer["ok"] and answer["error"]["code"] == ErrorCodes.Interrupted
    assert "stopped responding" in answer["error"]["message"]
    sock.close()


def test_server_shutdown_stops_velocity_control(native):
    station, close = make_station(native, patient(native))
    try:
        robot = station.client()
        servo = robot.start_servo()
        send(servo, [0.05, 0.0, 0.0, 0.0, 0.0, 0.0], seconds=0.1)
        station.call(station.server.close())
        assert wait_until(lambda: not servo.active)
        assert "shutting down" in servo.error.message
        robot.close()
    finally:
        close()


def test_the_gripper_works_under_velocity_control(station):
    with station.client() as a, station.client() as b, a.start_servo() as servo:
        assert a.gripper.move(0.04, 0.1)
        assert "under velocity control for" in raises(
            ErrorCodes.RobotBusy, b.gripper.move, 0.02, 0.1
        )
        assert servo.active


def test_a_server_without_velocity_control_is_reported(station):
    with station.client() as robot:
        robot.server_info = {
            key: value for key, value in robot.server_info.items() if key != "features"
        }
        message = raises(ErrorCodes.IncompatibleVersion, robot.start_servo)
        assert "no velocity control" in message
        assert not robot.is_executing


def test_velocity_control_is_refused_while_the_robot_cannot_move(station):
    with station.client() as robot:
        station.controller.sim_set_robot_mode("user_stopped")
        assert wait_until(
            lambda: robot.get_current_robot_state().robot_mode == "user_stopped"
        )
        assert "user stop" in raises(ErrorCodes.RobotNotReady, robot.start_servo)
        assert not robot.is_executing
