"""Planning on the GPU PC with the course toolbox, then running the plan on the station."""

import numpy as np
import pytest

rtb = pytest.importorskip("roboticstoolbox")
pytest.importorskip("ruckig")
from scipy.spatial.transform import Rotation
from spatialmath import SE3

from csc376_franky import ErrorCodes, protocol
from csc376_franky.motion_generator import (
    RuckigMotionGenerator,
    normalize_rx_angle,
    se3_to_rotation_vector,
)


@pytest.fixture(scope="module")
def panda():
    return rtb.models.Panda()


def plan(panda, q_start, motion):
    generator = RuckigMotionGenerator()
    se3_start = panda.fkine(q_start)
    se3_target = motion * se3_start
    poses, dt = generator.calculate_cartesian_pose_trajectory(se3_start, se3_target)
    q_traj = generator.cartesian_pose_to_joint_trajectory(panda, q_start, poses)
    return se3_start, se3_target, q_traj, dt


def turned(pose, axis, angle, move=(0.0, 0.0, 0.0)):
    """The pose turned on the spot by angle about an axis of the base frame, then moved."""
    turn = Rotation.from_rotvec(angle * np.array(axis) / np.linalg.norm(axis))
    return SE3.Rt(turn.as_matrix() @ pose.R, pose.t + move)


# ikine_LM stops once 0.5 |e|^2 < tol = 1e-12, so each IK solution may be up to
# about 1.4 um (and 1.4 urad) from its pose.
IK_ACCURACY = 1e-5

# The motion generator's rotation limit [rad/s], and its default factors.
ROTATION_SPEED = 2.5
FACTORS = (0.02, 0.01, 0.05)

QR = rtb.models.Panda().qr
# Joint positions with the hand pointing down, 0.01 rad to either side of half a
# turn from the base frame: the hand's rotation vector has rx = 3.13 rad at the
# first and rx = -3.13 rad at the second. The first is 0.01 rad from the Franka
# home pose. The miniPC refused the plan from one to the other in the lab.
RX_NEAR_PI = np.array([0.0013, -0.7867, 0.0064, -2.3551, -0.0054, 1.5744, 0.7984])
RX_NEAR_MINUS_PI = np.array([0.1991, -0.2329, 0.6081, -2.6061, 0.1331, 2.4719, 1.4707])
# A hand pointing forward, where rx of its rotation vector is close to 0.
RX_NEAR_ZERO = SE3(0.45, 0, 0.45) * SE3.Ry(np.pi / 2) * SE3.Rx(-0.01)

# For a move in joint space: the motion generator's joint limits [rad/s, rad/s^2,
# rad/s^3] and its default factors, and the Franka home pose.
JOINT_VELOCITY = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
JOINT_ACCELERATION = np.array([15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0])
JOINT_JERK = np.array([7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0])
JOINT_FACTORS = (0.1, 0.05, 0.001)
FRANKA_HOME = np.array(protocol.HOME_JOINT_POSITIONS)


def test_straight_line_from_the_start_to_the_target(panda):
    se3_start, se3_target, q_traj, dt = plan(panda, panda.qr, SE3.Ty(0.1))
    assert dt == pytest.approx(0.01)
    # It starts exactly where the robot is...
    np.testing.assert_allclose(q_traj[0], panda.qr, atol=1e-9)
    # ...ends at the target, and the hand keeps its orientation on a straight line.
    np.testing.assert_allclose(
        panda.fkine(q_traj[-1]).A, se3_target.A, atol=IK_ACCURACY
    )
    for q in q_traj:
        pose = panda.fkine(q)
        assert (
            abs(pose.t[0] - se3_start.t[0]) < IK_ACCURACY
            and abs(pose.t[2] - se3_start.t[2]) < IK_ACCURACY
        )
        np.testing.assert_allclose(pose.R, se3_start.R, atol=IK_ACCURACY)


@pytest.mark.parametrize(
    "start, axis, angle, move, factors",
    [
        (QR, [0, 0, 1], np.pi / 2, [0, 0, 0], FACTORS),
        (QR, [1, -2, 2], 1.2, [0.05, -0.1, 0.1], FACTORS),
        (QR, [0, 1, 0], 1.0, [0, 0, 0.1], (0.06, 0.01, 0.05)),
        (QR, [2, 1, -2], 3.1, [0, 0, 0], FACTORS),
        (QR, [0, 0, 1], 4.0, [0, 0, 0], FACTORS),
        (RX_NEAR_PI, [1, 1, 0], 0.3, [0, 0, 0], FACTORS),
        (RX_NEAR_ZERO, [0, 0, -1], 0.02, [0, 0, 0], FACTORS),
    ],
    ids=[
        "about_the_vertical",
        "about_an_oblique_axis_while_moving",
        "with_a_larger_velocity_factor",
        "nearly_half_a_turn",
        "more_than_half_a_turn",
        "across_rx_pi",
        "through_rx_zero",
    ],
)
def test_hand_turns_about_one_axis_the_short_way_round(
    panda, start, axis, angle, move, factors
):
    se3_start = start if isinstance(start, SE3) else panda.fkine(start)
    se3_target = turned(se3_start, axis, angle, move)
    poses, dt = RuckigMotionGenerator().calculate_cartesian_pose_trajectory(
        se3_start, se3_target, *factors
    )
    poses = np.array(poses)
    axis = np.array(axis) / np.linalg.norm(axis)
    if angle > np.pi:  # the short way round is the other way
        axis, angle = -axis, 2 * np.pi - angle

    # At every point the hand is turned from its start orientation about the axis...
    turns = (
        Rotation.from_rotvec(poses[:, 3:6]) * Rotation.from_matrix(se3_start.R).inv()
    ).as_rotvec()
    angles = turns @ axis
    np.testing.assert_allclose(turns, np.outer(angles, axis), atol=1e-9)
    # ...by an angle that goes from none to the whole turn, never backwards, and
    # no faster than the rotation limit, which turns as long as these reach.
    assert angles[0] == pytest.approx(0.0, abs=1e-9)
    assert angles[-1] == pytest.approx(angle, abs=1e-9)
    steps = np.diff(angles)
    assert steps.min() > -1e-12
    assert steps.max() == pytest.approx(ROTATION_SPEED * factors[0] * dt, rel=1e-6)
    # Meanwhile its position goes from the start's to the target's.
    np.testing.assert_allclose(poses[0, :3], se3_start.t, atol=1e-12)
    np.testing.assert_allclose(poses[-1, :3], se3_target.t, atol=1e-12)


def test_small_turn_is_a_short_plan():
    # Through rx = 0 of the rotation vector, a turn of 0.02 rad was planned as a
    # sweep of more than a full turn that took two minutes.
    se3_target = turned(RX_NEAR_ZERO, [0, 0, -1], 0.02)
    assert se3_to_rotation_vector(RX_NEAR_ZERO)[3] < 0.0
    assert se3_to_rotation_vector(se3_target)[3] > 0.0
    poses, dt = RuckigMotionGenerator().calculate_cartesian_pose_trajectory(
        RX_NEAR_ZERO, se3_target
    )
    assert dt * (len(poses) - 1) < 1.0


def test_hand_turns_no_faster_than_the_smallest_rotation_limit(panda):
    generator = RuckigMotionGenerator()
    generator.PANDA_VEL_LIM_CARTESIAN[5] = ROTATION_SPEED / 2
    se3_start = panda.fkine(QR)
    poses, dt = generator.calculate_cartesian_pose_trajectory(
        se3_start, turned(se3_start, [1, 0, 0], 0.5)
    )
    rotations = Rotation.from_rotvec(np.array(poses)[:, 3:6])
    steps = (rotations[:-1].inv() * rotations[1:]).magnitude()
    assert steps.max() == pytest.approx(ROTATION_SPEED / 2 * FACTORS[0] * dt, rel=1e-6)


def test_cartesian_trajectory_is_made_of_poses(panda):
    # [x, y, z, rx, ry, rz, elbow]: (rx, ry, rz) is the rotation vector of the
    # hand's orientation itself, at the start pose, the target pose and in between.
    se3_start, se3_target = panda.fkine(RX_NEAR_PI), panda.fkine(RX_NEAR_MINUS_PI)
    poses, _ = RuckigMotionGenerator().calculate_cartesian_pose_trajectory(
        se3_start, se3_target
    )
    assert all(isinstance(pose, list) and len(pose) == 7 for pose in poses)
    assert poses[0] == se3_to_rotation_vector(se3_start).tolist()
    np.testing.assert_allclose(poses[-1][:3], se3_target.t, atol=1e-12)
    np.testing.assert_allclose(
        Rotation.from_rotvec(poses[-1][3:6]).as_matrix(), se3_target.R, atol=1e-12
    )


def test_joint_trajectory_of_poses_written_by_hand(panda):
    # A pose is given as its rotation vector too: rx is not mapped, as
    # normalize_rx_angle used to do.
    se3_poses = [SE3.Tz(-0.01 * i) * panda.fkine(QR) for i in range(1, 4)]
    q_traj = RuckigMotionGenerator().cartesian_pose_to_joint_trajectory(
        panda, QR, [se3_to_rotation_vector(se3_pose) for se3_pose in se3_poses]
    )
    for q, se3_pose in zip(q_traj, se3_poses):
        np.testing.assert_allclose(panda.fkine(q).A, se3_pose.A, atol=IK_ACCURACY)


def test_normalize_rx_angle_still_maps_rx_and_warns():
    pose = np.array([0.3, 0.0, 0.5, 3.0, 0.1, -0.2, 0.0])
    with pytest.warns(DeprecationWarning, match="rotation vector"):
        mapped = normalize_rx_angle(pose)
    np.testing.assert_allclose(mapped, [0.3, 0.0, 0.5, np.pi - 3.0, 0.1, -0.2, 0.0])
    np.testing.assert_allclose(pose, [0.3, 0.0, 0.5, 3.0, 0.1, -0.2, 0.0])


def test_joints_never_stall_while_the_hand_moves(panda):
    # With a loose IK tolerance, the small steps at the start and end of a move
    # returned the previous joint positions unchanged, so the robot stuttered.
    _, _, q_traj, _ = plan(panda, panda.qr, SE3.Ty(0.1))
    steps = np.abs(np.diff(np.array(q_traj), axis=0)).max(axis=1)
    assert (steps[2:-2] > 0.0).all()


@pytest.mark.parametrize(
    "q_start",
    [
        rtb.models.Panda().qr,
        [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4],
    ],
    ids=["qr", "franka_home"],
)
def test_joints_never_jump_while_the_hand_moves(panda, q_start):
    # From the Franka home pose, ikine_LM's default damping went singular and it
    # restarted from random joint positions, so the elbow jumped mid-plan.
    _, _, q_traj, _ = plan(panda, np.array(q_start, dtype=float), SE3.Ty(0.1))
    steps = np.abs(np.diff(np.array(q_traj), axis=0)).max(axis=1)
    assert steps.max() < 0.01


def test_joints_never_jump_where_the_rotation_vector_flips(panda):
    # The components of a rotation vector are not continuous: rx of the hand's
    # goes from 3.13 rad to -3.13 rad here. Interpolating them made the hand's
    # orientation, and with it the joints, jump at one point of the plan.
    se3_start, se3_target = panda.fkine(RX_NEAR_PI), panda.fkine(RX_NEAR_MINUS_PI)
    assert se3_to_rotation_vector(se3_start)[3] > 3.0
    assert se3_to_rotation_vector(se3_target)[3] < -3.0
    generator = RuckigMotionGenerator()
    poses, _ = generator.calculate_cartesian_pose_trajectory(se3_start, se3_target)
    q_traj = generator.cartesian_pose_to_joint_trajectory(panda, RX_NEAR_PI, poses)
    steps = np.abs(np.diff(np.array(q_traj), axis=0)).max(axis=1)
    assert steps.max() < 0.01


def test_plan_passes_the_minipc_checks(native, panda):
    _, _, q_traj, dt = plan(panda, panda.qr, SE3.Tx(-0.1))
    code, message = native.validate_trajectory(
        np.array(q_traj), dt, panda.qr, native.Limits()
    )
    assert ErrorCodes(code) == ErrorCodes.Success, message


@pytest.mark.parametrize(
    "q_start, target, factors",
    [
        (RX_NEAR_PI, RX_NEAR_MINUS_PI, FACTORS),
        (RX_NEAR_PI, lambda start: turned(start, [1, 1, 0], 0.3), FACTORS),
        (
            QR,
            lambda start: turned(start, [1, -2, 2], 1.2, [0.05, -0.1, 0.1]),
            (0.1, 0.05, 0.05),
        ),
    ],
    ids=["across_rx_pi", "tilted_across_rx_pi", "oblique_turn_while_moving"],
)
def test_plan_that_turns_the_hand_passes_the_minipc_checks(
    native, panda, q_start, target, factors
):
    se3_start = panda.fkine(q_start)
    se3_target = target(se3_start) if callable(target) else panda.fkine(target)
    generator = RuckigMotionGenerator()
    poses, dt = generator.calculate_cartesian_pose_trajectory(
        se3_start, se3_target, *factors
    )
    q_traj = generator.cartesian_pose_to_joint_trajectory(panda, q_start, poses)
    code, message = native.validate_trajectory(
        np.array(q_traj), dt, q_start, native.Limits()
    )
    assert ErrorCodes(code) == ErrorCodes.Success, message
    np.testing.assert_allclose(
        panda.fkine(q_traj[-1]).A, se3_target.A, atol=IK_ACCURACY
    )


def test_unreachable_target_is_reported(panda):
    with pytest.raises(RuntimeError, match="Inverse kinematics"):
        plan(panda, panda.qr, SE3.Tx(0.6))


def test_plan_from_the_robot_and_run_it(station, panda):
    with station.client() as robot:
        q_start = robot.get_current_joint_positions()
        # The miniPC's pose and the toolbox model agree, so either can be planned from.
        np.testing.assert_allclose(
            robot.get_current_pose().A, panda.fkine(q_start).A, atol=1e-6
        )
        _, se3_target, q_traj, dt = plan(panda, q_start, SE3.Tz(-0.05))
        assert robot.run_joint_trajectory(q_traj, dt) == ErrorCodes.Success
        np.testing.assert_allclose(
            robot.get_current_pose().t, se3_target.t, atol=IK_ACCURACY + 1e-3
        )


def test_plan_a_turn_from_the_robot_and_run_it(station, panda):
    with station.client() as robot:
        q_start = robot.get_current_joint_positions()
        se3_start = panda.fkine(q_start)
        se3_target = turned(se3_start, [0, 0, 1], 0.3, [0, 0.05, 0])
        generator = RuckigMotionGenerator()
        poses, dt = generator.calculate_cartesian_pose_trajectory(
            se3_start, se3_target, 0.1, 0.05, 0.05
        )
        q_traj = generator.cartesian_pose_to_joint_trajectory(panda, q_start, poses)
        assert robot.run_joint_trajectory(q_traj, dt) == ErrorCodes.Success
        np.testing.assert_allclose(
            robot.get_current_pose().A, se3_target.A, atol=IK_ACCURACY + 1e-3
        )


def joint_peaks(q_traj, dt):
    """The largest velocity, acceleration and jerk of each joint along a trajectory."""
    q_traj = np.array(q_traj)
    return [
        np.abs(np.diff(q_traj, order, axis=0)).max(axis=0) / dt**order
        for order in (1, 2, 3)
    ]


def test_joint_move_goes_straight_from_the_start_to_the_target():
    # Joint positions come as an array, a list or a tuple.
    q_traj, dt = RuckigMotionGenerator().calculate_joint_trajectory(
        protocol.HOME_JOINT_POSITIONS, list(QR)
    )
    assert dt == pytest.approx(0.01)
    assert all(isinstance(q, np.ndarray) and q.shape == (7,) for q in q_traj)
    q_traj = np.array(q_traj)
    # It starts exactly where the robot is and ends exactly at the target...
    np.testing.assert_array_equal(q_traj[0], FRANKA_HOME)
    np.testing.assert_array_equal(q_traj[-1], QR)
    # ...and at every point each joint has gone the same fraction of its way,
    # which goes from none to all of it and never backwards.
    way = QR - FRANKA_HOME
    fraction = (q_traj[:, 1] - FRANKA_HOME[1]) / way[1]
    np.testing.assert_allclose(
        q_traj, FRANKA_HOME + np.outer(fraction, way), atol=1e-12
    )
    assert fraction[0] == 0.0 and fraction[-1] == 1.0
    assert np.diff(fraction).min() > -1e-12
    # It is at rest at both ends.
    velocity = np.abs(np.diff(q_traj, axis=0)).max(axis=1) / dt
    assert velocity[0] < 1e-3 and velocity[-1] < 1e-3


@pytest.mark.parametrize(
    "q_start, q_target, factors",
    [
        (FRANKA_HOME, QR, JOINT_FACTORS),
        (QR, QR + [np.pi / 2, 0, 0, 0, 0, 0, 0], JOINT_FACTORS),
        (FRANKA_HOME, FRANKA_HOME + [0, 0, 0, 0, 0, 0, -0.015], JOINT_FACTORS),
        (RX_NEAR_MINUS_PI, FRANKA_HOME, (0.3, 0.2, 0.01)),
        (
            [-2.5, -1.5, -2.5, -2.9, -2.5, 0.1, -2.5],
            [2.5, 1.5, 2.5, -0.2, 2.5, 3.5, 2.5],
            (0.3, 1.0, 1.0),
        ),
    ],
    ids=[
        "three_joints",
        "one_joint",
        "a_hundredth_of_a_radian",
        "all_joints_faster",
        "across_the_whole_range_at_the_limits_of_the_minipc",
    ],
)
def test_joint_move_keeps_every_joint_within_its_limits(q_start, q_target, factors):
    q_traj, dt = RuckigMotionGenerator().calculate_joint_trajectory(
        q_start, q_target, *factors
    )
    np.testing.assert_array_equal(q_traj[-1], q_target)
    limits = (JOINT_VELOCITY, JOINT_ACCELERATION, JOINT_JERK)
    for peak, limit, factor in zip(joint_peaks(q_traj, dt), limits, factors):
        assert (peak <= factor * limit * (1 + 1e-6)).all()


def test_joint_move_is_as_fast_as_the_limits_of_every_joint_allow():
    # Joint 1 has the longest way to go, so its velocity limit sets the top speed.
    # Joint 2 goes nearly as far with half the acceleration and jerk limits, so it
    # is its limits that set how fast the move speeds up and slows down.
    q_traj, dt = RuckigMotionGenerator().calculate_joint_trajectory(
        QR, QR + [1.0, 0.9, 0, 0, 0, 0, 0]
    )
    velocity, acceleration, jerk = joint_peaks(q_traj, dt)
    assert velocity[0] == pytest.approx(JOINT_FACTORS[0] * JOINT_VELOCITY[0], rel=1e-6)
    assert acceleration[1] == pytest.approx(
        JOINT_FACTORS[1] * JOINT_ACCELERATION[1], rel=1e-6
    )
    assert jerk[1] == pytest.approx(JOINT_FACTORS[2] * JOINT_JERK[1], rel=1e-6)
    # The two joints move in step, so each is slower than the other's limit asks.
    assert velocity[1] == pytest.approx(0.9 * velocity[0], rel=1e-6)
    assert acceleration[0] == pytest.approx(acceleration[1] / 0.9, rel=1e-6)


def test_joint_move_to_where_the_robot_is_holds_still():
    q_traj, dt = RuckigMotionGenerator().calculate_joint_trajectory(QR, QR)
    # Two points: the miniPC runs no trajectory with fewer.
    np.testing.assert_array_equal(q_traj, [QR, QR])
    assert dt == pytest.approx(0.01)


def test_joint_moves_do_not_depend_on_the_ones_planned_before():
    # Ruckig keeps state from one update to the next: a Cartesian plan of no
    # length, made twice in a row with one motion generator, comes out wrong.
    generator = RuckigMotionGenerator()
    first, _ = generator.calculate_joint_trajectory(FRANKA_HOME, QR)
    for q_start, q_target in ((QR, QR), (QR, QR), (QR, FRANKA_HOME)):
        generator.calculate_joint_trajectory(q_start, q_target)
    again, _ = generator.calculate_joint_trajectory(FRANKA_HOME, QR)
    np.testing.assert_array_equal(again, first)


def test_joint_move_is_sampled_at_the_dt_of_the_motion_generator():
    generator = RuckigMotionGenerator()
    q_traj, dt = generator.calculate_joint_trajectory(FRANKA_HOME, QR)
    generator.dt = dt / 2
    finer, finer_dt = generator.calculate_joint_trajectory(FRANKA_HOME, QR)
    assert finer_dt == dt / 2
    # The same move, with a point in between every two.
    np.testing.assert_allclose(finer[::2][: len(q_traj) - 1], q_traj[:-1], atol=1e-12)
    # Ruckig never comes to an end with a time step of zero.
    generator.dt = 0.0
    with pytest.raises(ValueError, match="dt must be larger than 0"):
        generator.calculate_joint_trajectory(FRANKA_HOME, QR)


@pytest.mark.parametrize(
    "arguments, message",
    [
        ((QR[:6], FRANKA_HOME), "q_start must be 7 joint positions"),
        ((QR, [FRANKA_HOME, FRANKA_HOME]), "q_target must be 7 joint positions"),
        (
            (QR, [0, -0.3, 0, -2.2, 0, 2.0, np.nan]),
            "q_target must be 7 joint positions",
        ),
        (
            (QR, np.zeros(7)),
            r"q_target\[3\] = 0 rad is outside .* limits, -3.0718 to -0.0698 rad",
        ),
        (
            (QR, np.degrees(FRANKA_HOME)),
            r"q_target\[1\] = -45 rad is outside the Panda's joint limits",
        ),
        ((QR, FRANKA_HOME, 0.0), "relative_vel_factor must be larger than 0"),
        ((QR, FRANKA_HOME, 0.1, 1.5), "relative_acc_factor must be .* at most 1"),
        ((QR, FRANKA_HOME, 0.1, 0.05, np.nan), "relative_jerk_factor must be"),
    ],
    ids=[
        "six_joints",
        "a_trajectory_for_a_target",
        "not_a_number",
        "target_outside_the_joint_limits",
        "target_in_degrees",
        "no_velocity",
        "more_than_the_robot_can_do",
        "factor_not_a_number",
    ],
)
def test_joint_move_refuses_what_it_cannot_plan_with(arguments, message):
    with pytest.raises(ValueError, match=message):
        RuckigMotionGenerator().calculate_joint_trajectory(*arguments)


def test_joint_move_goes_up_to_the_joint_limits(panda):
    generator = RuckigMotionGenerator()
    # Its joint limits are those of the toolbox's Panda.
    np.testing.assert_allclose(generator.PANDA_POS_LIM_JOINT, panda.qlim, atol=1e-12)
    for q_target in generator.PANDA_POS_LIM_JOINT:
        q_traj, _ = generator.calculate_joint_trajectory(QR, q_target)
        np.testing.assert_array_equal(q_traj[-1], q_target)
    # It plans from a start outside them too. In Swift a Panda can be anywhere:
    # at the toolbox's zero pose for example, where the real robot cannot be.
    q_traj, _ = generator.calculate_joint_trajectory(panda.qz, QR)
    np.testing.assert_array_equal(q_traj[0], panda.qz)
    np.testing.assert_array_equal(q_traj[-1], QR)


def test_joint_move_that_would_take_too_long_is_reported():
    # At a ten-thousandth of the joint velocity limits this move takes over half
    # an hour. Ruckig would plan one that takes days as readily, point by point.
    with pytest.raises(RuntimeError, match="more than 600 s"):
        RuckigMotionGenerator().calculate_joint_trajectory(FRANKA_HOME, QR, 1e-4)


def check_joint_moves_anywhere(native, panda, factors, limits):
    """Moves between random joint positions go through the miniPC's checks."""
    rng = np.random.default_rng(376)
    low, high = panda.qlim[0] + 0.02, panda.qlim[1] - 0.02
    generator = RuckigMotionGenerator()
    for _ in range(40):
        q_start, q_target = rng.uniform(low, high, (2, 7))
        q_traj, dt = generator.calculate_joint_trajectory(q_start, q_target, *factors)
        code, message = native.validate_trajectory(
            np.array(q_traj), dt, q_start, limits
        )
        assert ErrorCodes(code) == ErrorCodes.Success, message


def test_joint_moves_pass_the_minipc_checks(native, panda):
    check_joint_moves_anywhere(native, panda, JOINT_FACTORS, native.Limits())


def test_joint_moves_at_the_limits_of_the_minipc_pass_its_checks(native, panda):
    # A plan made at the miniPC's own limits is not over them for a rounding
    # error: the motion generator plans a hair inside its limits. Without that,
    # the miniPC refuses three in four of these moves, at 0.6525 rad/s where
    # 0.6525 rad/s is allowed.
    limits = native.Limits()
    factors = (limits.velocity_scale, limits.acceleration_scale, 1.0)
    check_joint_moves_anywhere(native, panda, factors, limits)


def test_plan_a_joint_move_from_the_robot_and_run_it(station):
    with station.client() as robot:
        q_start = robot.get_current_joint_positions()
        q_traj, dt = RuckigMotionGenerator().calculate_joint_trajectory(
            q_start, FRANKA_HOME, 0.3, 0.3, 0.01
        )
        assert robot.run_joint_trajectory(q_traj, dt) == ErrorCodes.Success
        # No inverse kinematics on the way: the robot is at the target's joint positions.
        np.testing.assert_allclose(
            robot.get_current_joint_positions(), FRANKA_HOME, atol=2e-3
        )
