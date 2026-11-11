"""Planning on the GPU PC with the course toolbox, then running the plan on the station."""

import numpy as np
import pytest

rtb = pytest.importorskip("roboticstoolbox")
pytest.importorskip("ruckig")
from spatialmath import SE3  # noqa: E402

from csc376_franky import ErrorCodes  # noqa: E402
from csc376_franky.motion_generator import RuckigMotionGenerator  # noqa: E402


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


# ikine_LM stops once 0.5 |e|^2 < tol = 1e-12, so each IK solution may be up to
# about 1.4 um (and 1.4 urad) from its pose.
IK_ACCURACY = 1e-5


def test_straight_line_from_the_start_to_the_target(panda):
    se3_start, se3_target, q_traj, dt = plan(panda, panda.qr, SE3.Ty(0.1))
    assert dt == pytest.approx(0.01)
    # It starts exactly where the robot is...
    np.testing.assert_allclose(q_traj[0], panda.qr, atol=1e-9)
    # ...ends at the target, and the hand keeps its orientation on a straight line.
    np.testing.assert_allclose(panda.fkine(q_traj[-1]).A, se3_target.A, atol=IK_ACCURACY)
    for q in q_traj:
        pose = panda.fkine(q)
        assert abs(pose.t[0] - se3_start.t[0]) < IK_ACCURACY and abs(pose.t[2] - se3_start.t[2]) < IK_ACCURACY
        np.testing.assert_allclose(pose.R, se3_start.R, atol=IK_ACCURACY)


def test_joints_never_stall_while_the_hand_moves(panda):
    # With a loose IK tolerance, the small steps at the start and end of a move
    # returned the previous joint positions unchanged, so the robot stuttered.
    _, _, q_traj, _ = plan(panda, panda.qr, SE3.Ty(0.1))
    steps = np.abs(np.diff(np.array(q_traj), axis=0)).max(axis=1)
    assert (steps[2:-2] > 0.0).all()


@pytest.mark.parametrize("q_start", [rtb.models.Panda().qr, [0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4]],
                         ids=["qr", "franka_home"])
def test_joints_never_jump_while_the_hand_moves(panda, q_start):
    # From the Franka home pose, ikine_LM's default damping went singular and it
    # restarted from random joint positions, so the elbow jumped mid-plan.
    _, _, q_traj, _ = plan(panda, np.array(q_start, dtype=float), SE3.Ty(0.1))
    steps = np.abs(np.diff(np.array(q_traj), axis=0)).max(axis=1)
    assert steps.max() < 0.01


def test_plan_passes_the_minipc_checks(native, panda):
    _, _, q_traj, dt = plan(panda, panda.qr, SE3.Tx(-0.1))
    code, message = native.validate_trajectory(np.array(q_traj), dt, panda.qr, native.Limits())
    assert ErrorCodes(code) == ErrorCodes.Success, message


def test_unreachable_target_is_reported(panda):
    with pytest.raises(RuntimeError, match="Inverse kinematics"):
        plan(panda, panda.qr, SE3.Tx(0.6))


def test_plan_from_the_robot_and_run_it(station, panda):
    with station.client() as robot:
        q_start = robot.get_current_joint_positions()
        # The miniPC's pose and the toolbox model agree, so either can be planned from.
        np.testing.assert_allclose(robot.get_current_pose().A, panda.fkine(q_start).A, atol=1e-6)
        _, se3_target, q_traj, dt = plan(panda, q_start, SE3.Tz(-0.05))
        assert robot.run_joint_trajectory(q_traj, dt) == ErrorCodes.Success
        np.testing.assert_allclose(robot.get_current_pose().t, se3_target.t, atol=IK_ACCURACY + 1e-3)
