"""Trajectory checks and playback in the miniPC's C++ controller."""

import numpy as np
import pytest

from csc376_franky.protocol import ErrorCodes

from helpers import linear_trajectory

READY = np.array([0.0, -0.3, 0.0, -2.2, 0.0, 2.0, np.pi / 4])


def validate(native, trajectory, dt=0.01, q_current=READY, **limits):
    options = native.Limits()
    for name, value in limits.items():
        setattr(options, name, value)
    code, message = native.validate_trajectory(np.asarray(trajectory, dtype=float), dt, q_current, options)
    return ErrorCodes(code), message


def test_smooth_trajectory_is_accepted(native):
    trajectory = linear_trajectory(READY, [0.2, 0, 0, 0, 0, 0, 0], duration=2.0)
    assert validate(native, trajectory) == (ErrorCodes.Success, "")


@pytest.mark.parametrize("change, code", [
    (lambda t: t[:, :6], ErrorCodes.JointSizeNotSeven),
    (lambda t: t[:1], ErrorCodes.TrajectorySizeLessThanTwo),
    (lambda t: np.where(np.arange(t.size).reshape(t.shape) == 30, np.nan, t), ErrorCodes.NonFiniteValue),
])
def test_malformed_trajectories_are_refused(native, change, code):
    trajectory = change(linear_trajectory(READY, [0.2, 0, 0, 0, 0, 0, 0], duration=2.0))
    assert validate(native, trajectory)[0] == code


@pytest.mark.parametrize("dt, code", [
    (0.0, ErrorCodes.DtIsNegative),
    (-0.01, ErrorCodes.DtIsNegative),
    (0.0001, ErrorCodes.DtIsSmallerThanAllowed),
    (2.0, ErrorCodes.DtIsLargerThanAllowed),
    (float("nan"), ErrorCodes.NonFiniteValue),
])
def test_bad_dt_is_refused(native, dt, code):
    trajectory = np.vstack([READY, READY])
    assert validate(native, trajectory, dt=dt)[0] == code


def test_start_far_from_the_robot_is_refused(native):
    trajectory = linear_trajectory(READY, [0.2, 0, 0, 0, 0, 0, 0], duration=2.0)
    robot = READY.copy()
    robot[3] += 0.05
    code, message = validate(native, trajectory, q_current=robot)
    assert code == ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition
    assert "q[3]" in message and "current joint positions" in message


def test_start_within_tolerance_is_accepted(native):
    trajectory = linear_trajectory(READY, [0.2, 0, 0, 0, 0, 0, 0], duration=2.0)
    robot = READY + 0.015
    assert validate(native, trajectory, q_current=robot)[0] == ErrorCodes.Success
    assert validate(native, trajectory, q_current=robot, start_tolerance=0.01)[0] == \
        ErrorCodes.JointTrajectoryDoesNotStartAtCurrentPosition


def test_joint_limits_with_margin(native):
    # q[3] may go up to -0.0698 rad, less the 0.01 rad margin.
    start = READY.copy()
    start[3] = -0.2
    trajectory = linear_trajectory(start, [0, 0, 0, 0.125, 0, 0, 0], duration=1.0)
    code, message = validate(native, trajectory, q_current=start)
    assert code == ErrorCodes.JointLimitsExceeded and "q[3]" in message
    trajectory = linear_trajectory(start, [0, 0, 0, 0.115, 0, 0, 0], duration=1.0)
    assert validate(native, trajectory, q_current=start)[0] == ErrorCodes.Success


@pytest.mark.parametrize("jump", [0.1, -0.1])
def test_jumps_are_refused_in_both_directions(native, jump):
    # The old controller compared signed differences, so a jump backwards passed.
    trajectory = np.repeat(READY[None, :], 20, axis=0)
    trajectory[10:, 2] += jump
    code, message = validate(native, trajectory, dt=0.5)
    assert code == ErrorCodes.JointDistancesAboveMaxJointDifference
    assert "q[2]" in message and "points 9 and 10" in message


def test_speed_limit(native):
    # 0.3 of the 2.175 rad/s limit is 0.6525 rad/s.
    fast = np.vstack([READY + [0.007 * i, 0, 0, 0, 0, 0, 0] for i in range(10)])
    assert validate(native, fast, dt=0.01)[0] == ErrorCodes.JointVelocitiesAboveMaximumAllowed
    slow = np.vstack([READY + [0.006 * i, 0, 0, 0, 0, 0, 0] for i in range(10)])
    assert validate(native, slow, dt=0.01)[0] == ErrorCodes.Success


def test_zig_zag_is_refused(native):
    # 0.02 rad back and forth every 40 ms is within the step and speed limits, but
    # turns the joint around at 25 rad/s^2; q[0] may do 15.
    trajectory = np.repeat(READY[None, :], 20, axis=0)
    trajectory[1::2, 0] += 0.02
    code, message = validate(native, trajectory, dt=0.04)
    assert code == ErrorCodes.JointAccelerationsAboveMaximumAllowed
    assert "q[0]" in message and "point 1;" in message
    assert validate(native, trajectory, dt=0.04, acceleration_scale=2.0)[0] == ErrorCodes.Success


def test_constant_velocity_from_the_first_point_is_accepted(native):
    # np.linspace between two joint positions: no acceleration between its points.
    trajectory = np.linspace(READY, READY + [0.3, 0, 0, 0, 0, 0, 0], 101)
    assert validate(native, trajectory, dt=0.01)[0] == ErrorCodes.Success


def test_length_limits(native):
    trajectory = np.repeat(READY[None, :], 1001, axis=0)
    assert validate(native, trajectory, dt=0.7)[0] == ErrorCodes.TrajectoryTooLong
    assert validate(native, trajectory, dt=0.01, max_points=1000)[0] == ErrorCodes.TrajectoryTooLong


def follow(native, trajectory, dt, q=None, stop_at=None, limits=None):
    """Run a TrajectoryFollower against a robot that tracks perfectly; return the commands."""
    follower = native.TrajectoryFollower(np.asarray(trajectory), dt, limits or native.Limits())
    q = np.array(trajectory[0] if q is None else q, dtype=float)
    commands = []
    period = 0.0
    for tick in range(100000):
        stop = stop_at is not None and tick >= stop_at
        follower.step(period, q, np.zeros(7), stop)
        commands.append((follower.phase, np.array(follower.q_d), np.array(follower.dq_d), follower.time))
        if follower.phase == "done":
            break
        q = np.array(follower.q_d)
        period = 0.001
    return follower, commands


def test_follower_interpolates_and_settles(native):
    trajectory = linear_trajectory(READY, [0.3, 0, 0, 0, 0, 0, 0], duration=1.0, dt=0.05)
    follower, commands = follow(native, trajectory, 0.05)
    assert follower.phase == "done" and not follower.interrupted
    assert follower.index == len(trajectory) - 1
    np.testing.assert_allclose(commands[-1][1], trajectory[-1], atol=1e-12)
    # Halfway between two points, the command is halfway too, with the segment's velocity.
    phase, q_d, dq_d, t = next(c for c in commands if abs(c[3] - 0.525) < 1e-9)
    np.testing.assert_allclose(q_d, (trajectory[10] + trajectory[11]) / 2, atol=1e-12)
    np.testing.assert_allclose(dq_d, (trajectory[11] - trajectory[10]) / 0.05, atol=1e-9)


def test_follower_blends_a_start_gap_out(native):
    trajectory = linear_trajectory(READY, [0.3, 0, 0, 0, 0, 0, 0], duration=1.0, dt=0.05)
    measured = READY + [0, 0, 0.015, 0, 0, 0, 0]
    follower, commands = follow(native, trajectory, 0.05, q=measured)
    # The first command is where the robot is, so it does not jump...
    np.testing.assert_allclose(commands[0][1], measured, atol=1e-12)
    # ...and the gap is gone after start_blend_time (0.25 s), without any step in between.
    after_blend = next(c for c in commands if c[3] >= 0.3)
    assert abs(after_blend[1][2] - trajectory[0][2]) < 1e-12
    steps = np.abs(np.diff([c[1] for c in commands], axis=0)).max()
    assert steps < 0.001


def test_follower_stops_along_the_path(native):
    trajectory = linear_trajectory(READY, [0.5, 0, 0, 0, 0, 0, 0], duration=4.0, dt=0.01)
    follower, commands = follow(native, trajectory, 0.01, stop_at=1000)
    assert follower.interrupted and follower.phase == "done"
    stop_tick = 1000
    stopped_tick = next(i for i, c in enumerate(commands) if c[0] == "settling")
    # The playback slows to rest within stop_time (0.5 s) of the stop...
    assert (stopped_tick - stop_tick) * 0.001 <= 0.5 + 0.002
    # ...and stays on the planned path (only q[0] moves) well short of its end,
    # never turning back.
    final = commands[-1][1]
    assert trajectory[0][0] < final[0] < trajectory[-1][0] - 0.1
    np.testing.assert_allclose(final[1:], READY[1:], atol=1e-12)
    positions = [c[1][0] for c in commands]
    assert all(b >= a - 1e-12 for a, b in zip(positions, positions[1:]))
    assert np.all(commands[-1][2] == 0.0)


def test_follower_reaching_the_end_while_stopping_was_not_interrupted(native):
    trajectory = linear_trajectory(READY, [0.5, 0, 0, 0, 0, 0, 0], duration=4.0, dt=0.01)
    # 0.1 s before the end: slowing down over stop_time covers more than that.
    follower, commands = follow(native, trajectory, 0.01, stop_at=3900)
    assert follower.phase == "done" and not follower.interrupted
    np.testing.assert_allclose(commands[-1][1], trajectory[-1], atol=1e-12)


def test_follower_ignores_a_stop_while_settling(native):
    trajectory = linear_trajectory(READY, [0.1, 0, 0, 0, 0, 0, 0], duration=1.0, dt=0.01)
    follower, commands = follow(native, trajectory, 0.01, stop_at=1010)
    assert commands[1010][0] == "settling"
    assert follower.phase == "done" and not follower.interrupted
