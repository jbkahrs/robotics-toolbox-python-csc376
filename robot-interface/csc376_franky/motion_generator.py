import copy
import warnings

import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation
from spatialmath import SE3

# Ruckig Motion Generator code taken mostly from: https://github.com/Patrick15Yao/franky_toolbox_bridge/blob/main/franky_toolbox_bridge/rtb_backend.py,
# and tailored in a more calculate traj -> then visualize way,
# also made more functional programming style.
# Please credit Patrick Yao, github id Patrick15Yao for this work.


def normalize_rx_angle(rv_vector):
    """
    Not used any more, and kept so that programs that import it still run.

    Maps rx of a pose [x, y, z, rx, ry, rz, elbow] to pi - rx (for rx > 0) or to
    -pi - rx; the mapping is its own inverse. The motion generator used to plan
    with rx mapped this way, and its Cartesian trajectories held the mapped rx.
    Since csc376_franky 2.4 they hold the rotation vector of each pose itself, so
    give cartesian_pose_to_joint_trajectory poses whose rx is not mapped.
    """
    warnings.warn(
        "normalize_rx_angle is not used any more: since csc376_franky 2.4 a "
        "Cartesian trajectory holds the rotation vector of each pose itself, so "
        "do not map rx before cartesian_pose_to_joint_trajectory",
        DeprecationWarning,
        stacklevel=2,
    )
    rv_normalized = rv_vector.copy()  # Make a copy to avoid modifying original
    # Apply the rule to the 4th element (index 3)
    rx = rv_normalized[3]
    if rx > 0:
        rv_normalized[3] = np.pi - rx
    else:
        rv_normalized[3] = -np.pi - rx
    return rv_normalized


def se3_to_rotation_vector(se3_matrix, elbow_angle=None):
    if hasattr(se3_matrix, "A"):
        se3_array = se3_matrix.A  # SE3 object
    else:
        se3_array = np.array(se3_matrix)  # Already numpy array

    translation = se3_array[:3, 3]
    rotation_matrix = se3_array[:3, :3]

    rotation = Rotation.from_matrix(rotation_matrix)
    rotvec = rotation.as_rotvec()  # [rx, ry, rz] in radians
    if elbow_angle is None:
        rotation_vector = np.concatenate([translation, rotvec, [0.0]])
    else:
        rotation_vector = np.concatenate([translation, rotvec, [elbow_angle]])
    return rotation_vector


def rotation_vector_to_joint_config(model, q_current, RV_new):
    """
    Convert 7D rotation vector back to 7 joint configuration.

    Args:
        RV_new: 7D vector [x, y, z, rx, ry, rz, elbow_angle]

    Returns:
        q_new: 7D joint configuration

    Raises:
        RuntimeError: if inverse kinematics finds no solution
    """
    translation = RV_new[:3]  # [x, y, z]
    rotvec = RV_new[3:6]  # [rx, ry, rz]
    # elbow_angle = RV_new[6]  # elbow_angle

    rotation = Rotation.from_rotvec(rotvec)
    rotation_matrix = rotation.as_matrix()

    se3_matrix = np.eye(4)
    se3_matrix[:3, :3] = rotation_matrix
    se3_matrix[:3, 3] = translation

    se3_pose = SE3(se3_matrix)
    # The default "chan" damping shrinks with the error, so near the solution the
    # step solve for the redundant Panda goes singular and ikine_LM restarts from a
    # random configuration (the elbow jumps). Sugihara damping keeps a floor of k,
    # and slimit=1 reports a failure instead of restarting somewhere random.
    solution = model.ikine_LM(
        se3_pose, q0=q_current, tol=1e-12, method="sugihara", k=1e-4, slimit=1
    )
    if not solution.success:
        raise RuntimeError(
            f"Inverse kinematics found no solution for the pose at {np.round(translation, 4)} m "
            f"({solution.reason}). Is the target out of reach?"
        )
    q_new = solution.q
    # q_new[2] = elbow_angle

    return q_new


class RuckigMotionGenerator:
    def __init__(self):
        self.dt = 0.01
        self.ruckig = Ruckig(dofs=7, delta_time=self.dt)
        # The Panda's Cartesian limits from its datasheet, for [x, y, z, rx, ry, rz, elbow]:
        # translation, rotation and elbow. A move turns the end effector about one
        # axis, and the smallest of the three rotation limits applies to that turn.
        self.PANDA_VEL_LIM_CARTESIAN = np.array(
            [1.7, 1.7, 1.7, 2.5, 2.5, 2.5, 2.175]
        )  # m/s, rad/s
        self.PANDA_ACC_LIM_CARTESIAN = np.array(
            [13.0, 13.0, 13.0, 25.0, 25.0, 25.0, 10.0]
        )  # m/s², rad/s²
        self.PANDA_JERK_LIM_CARTESIAN = np.array(
            [6500.0, 6500.0, 6500.0, 12500.0, 12500.0, 12500.0, 5000.0]
        )  # m/s³, rad/s³

    def calculate_cartesian_pose_trajectory(
        self,
        se3_start: SE3,
        se3_target: SE3,
        relative_vel_factor: float = 0.02,
        relative_acc_factor: float = 0.01,
        relative_jerk_factor: float = 0.05,
    ) -> tuple[list[list], float]:
        """Interpolates and creates a smooth cartesian trajectory (smooth vel, acc, jerk maybe)
        starting from se3_start and ending at se3_target. The trajectory also follows
        the given cartesian constraints: relative_vel_factor, relative_acc_factor, and relative_jerk_factor
        which implicitly converts to a joint constraint :).

        The end effector goes from the position of se3_start to that of
        se3_target, and turns from the orientation of se3_start to that of
        se3_target about one fixed axis, the short way round: by at most half a
        turn. Where the two orientations are exactly half a turn apart, either
        way round is as short, and rounding decides which one it takes. The
        rotation limits apply to the angle turned about that axis.

        Parameters
        ----------
        se3_start:  SE3
            The start end-effector pose of the trajectory. Usually the current ee pose of the robot.
        se3_target: SE3
            The end end-effector pose of the trajectory.
        relative_vel_factor : float
            0-1 multiplication factor to multiply PANDA_VEL_LIM_CARTESIAN.
            Limits the cartesian velocity during the entire trajectory.
        relative_acc_factor : float
            0-1 multiplication factor to multiply PANDA_ACC_LIM_CARTESIAN.
            Limits the cartesian acceleration during the entire trajectory.
        relative_jerk_factor: float
            0-1 multiplication factor to multiply PANDA_JERK_LIM_CARTESIAN.
            Limits the cartesian jerk during the entire trajectory.

        Returns
        ----------
        cartesian_pose_traj: list[list]
            List of a List: [[x, y, z, rx, ry, rz, elbow_angle], ... ].
            Cartesian pose trajectory of the end effector, from se3_start (at time 0)
            to se3_target: its position [m] and the rotation vector of its
            orientation [rad], in the base frame. A rotation vector is not
            continuous: where the end effector is half a turn from the base frame
            (a hand pointing down, for example), it can change sign from one point
            to the next although the orientation hardly changes.
        self.dt: float
            Delta time between trajectory points of the cartsian_pose_traj.
        """

        pose_start = se3_to_rotation_vector(se3_start)
        pose_target = se3_to_rotation_vector(se3_target)

        # The orientation is planned as one number, the angle turned about a fixed
        # axis: the rotation that takes the start orientation to the target's, in
        # the base frame. Interpolating the components of the rotation vector
        # instead makes the orientation jump, because they are not continuous:
        # with the hand pointing down, rx flips between pi and -pi.
        rotation_start = Rotation.from_rotvec(pose_start[3:6])
        turn = (
            Rotation.from_rotvec(pose_target[3:6]) * rotation_start.inv()
        ).as_rotvec()
        angle = np.linalg.norm(turn)
        axis = turn / angle if angle > 0.0 else turn

        def ruckig_limits(panda_limits, factor):
            limits = panda_limits * factor
            limits[3] = limits[3:6].min()  # of the angle, whichever way its axis points
            return limits

        # Ruckig's degrees of freedom are [x, y, z, angle, unused, unused, elbow].
        inp = InputParameter(7)
        out = OutputParameter(7)
        inp.current_position = [*pose_start[:3], 0.0, 0.0, 0.0, pose_start[6]]
        inp.current_velocity = [0, 0, 0, 0, 0, 0, 0]
        inp.current_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.target_position = [*pose_target[:3], angle, 0.0, 0.0, pose_target[6]]
        inp.target_velocity = [0, 0, 0, 0, 0, 0, 0]
        inp.target_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.max_velocity = ruckig_limits(
            self.PANDA_VEL_LIM_CARTESIAN, relative_vel_factor
        )
        inp.max_acceleration = ruckig_limits(
            self.PANDA_ACC_LIM_CARTESIAN, relative_acc_factor
        )
        inp.max_jerk = ruckig_limits(
            self.PANDA_JERK_LIM_CARTESIAN, relative_jerk_factor
        )
        inp.enabled = [True, True, True, True, True, True, True]
        res = Result.Working

        positions = [inp.current_position]
        while res == Result.Working:
            res = self.ruckig.update(
                inp, out
            )  # interpolates to robot vel, acc, and jerk constraints
            if res not in (Result.Working, Result.Finished):
                raise RuntimeError(f"Ruckig could not plan the trajectory: {res}")
            positions.append(out.new_position)
            out.pass_to_input(inp)

        # The orientation of each point is the start orientation turned by its angle.
        poses = np.array(positions)
        rotations = Rotation.from_rotvec(np.outer(poses[:, 3], axis)) * rotation_start
        poses[:, 3:6] = rotations.as_rotvec()
        # Start at the start pose itself, so the joint trajectory starts exactly
        # where the robot is (the miniPC refuses one that does not).
        poses[0] = pose_start
        return poses.tolist(), self.dt

    def cartesian_pose_to_joint_trajectory(
        self, model, q_start: np.ndarray, cartesian_pose_traj: list[list]
    ) -> list[np.ndarray]:
        """Converts the cartesian pose trajectory to a joint trajectory using inverse kinematics
        from the model.

        Parameters
        ----------
        model: robotics toolbox model type
            Robot model for computing IK.
        q_start : np.ndarray
            The robot's current joint positions to determine redundant solutions in the IK.
            The size is joint_dof of robot.
        cartesian_pose_traj:  list[list]
            List of a List: [[x, y, z, rx, ry, rz, elbow_angle], ... ].
            Cartesian pose trajectory of the end effector: its position [m] and
            the rotation vector of its orientation [rad], in the base frame, as
            calculate_cartesian_pose_trajectory returns it.

        Returns
        ----------
        q_traj: list[np.ndarray]
            List of np.ndarray of joint positions with size of joint_dof of robot.

        Raises
        ----------
        RuntimeError
            If inverse kinematics finds no solution for one of the poses.
        """

        q_current = copy.deepcopy(q_start)
        q_traj = []
        for cartesian_pose in cartesian_pose_traj:
            q_current = rotation_vector_to_joint_config(
                model, q_current, cartesian_pose
            )  # convert to joint traj
            q_traj.append(q_current)
        return q_traj
