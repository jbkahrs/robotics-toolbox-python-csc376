import copy

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
    Map the rx component of the rotation vector so that it is continuous where rx itself jumps.

    With the hand pointing down, rx is close to pi or to -pi and flips between the
    two from one pose to the next. The mapped value is close to 0 on both sides, so
    Ruckig can interpolate it. The mapping is its own inverse:
    calculate_cartesian_pose_trajectory applies it to the two ends of a move, and
    cartesian_pose_to_joint_trajectory applies it again to every interpolated pose
    to get the rotation vector back. Both calls are needed.
    """
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
        # translation, rotation and elbow.
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

        The rotation vector's components are interpolated linearly, which only
        follows a rotation about a fixed axis when the start and target
        orientations are close. Plan a large reorientation as several short moves.

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
            to se3_target.
        self.dt: float
            Delta time between trajectory points of the cartsian_pose_traj.
        """

        RV_start = se3_to_rotation_vector(se3_start)
        RV_target = se3_to_rotation_vector(se3_target)

        # Interpolate the mapped rx (see normalize_rx_angle); the mapping is
        # undone in cartesian_pose_to_joint_trajectory.
        N_RV_start = normalize_rx_angle(RV_start)
        N_RV_target = normalize_rx_angle(RV_target)

        inp = InputParameter(7)
        out = OutputParameter(7)
        inp.current_position = N_RV_start
        inp.current_velocity = [0, 0, 0, 0, 0, 0, 0]
        inp.current_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.target_position = N_RV_target
        inp.target_velocity = [0, 0, 0, 0, 0, 0, 0]
        inp.target_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.max_velocity = self.PANDA_VEL_LIM_CARTESIAN * relative_vel_factor
        inp.max_acceleration = self.PANDA_ACC_LIM_CARTESIAN * relative_acc_factor
        inp.max_jerk = self.PANDA_JERK_LIM_CARTESIAN * relative_jerk_factor
        inp.enabled = [True, True, True, True, True, True, True]
        res = Result.Working

        # Start at the start pose itself, so the joint trajectory starts exactly
        # where the robot is (the miniPC refuses one that does not).
        cartesian_pose_traj = [list(N_RV_start)]
        while res == Result.Working:
            res = self.ruckig.update(
                inp, out
            )  # interpolates to robot vel, acc, and jerk constraints
            if res not in (Result.Working, Result.Finished):
                raise RuntimeError(f"Ruckig could not plan the trajectory: {res}")
            cartesian_pose_traj.append(out.new_position)
            out.pass_to_input(inp)
        return cartesian_pose_traj, self.dt

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
            Cartesian pose trajectory of the end effector.

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
            # normalize_rx_angle here undoes the mapping the trajectory was planned in.
            q_current = rotation_vector_to_joint_config(
                model, q_current, normalize_rx_angle(cartesian_pose)
            )  # convert to joint traj
            q_traj.append(q_current)
        return q_traj
