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

# The longest trajectory the miniPC runs [s]. calculate_joint_trajectory plans none
# that is longer.
MAX_TRAJECTORY_DURATION = 600.0


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
        # The Panda's joint limits from its datasheet, for joints 1 to 7: the lowest
        # and the highest position of each joint, then how fast it may move. The
        # miniPC checks every trajectory against the positions, velocities and
        # accelerations too.
        self.PANDA_POS_LIM_JOINT = np.array(
            [
                [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
                [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
            ]
        )  # rad
        self.PANDA_VEL_LIM_JOINT = np.array(
            [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]
        )  # rad/s
        self.PANDA_ACC_LIM_JOINT = np.array(
            [15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0]
        )  # rad/s²
        self.PANDA_JERK_LIM_JOINT = np.array(
            [7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0]
        )  # rad/s³

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

    def calculate_joint_trajectory(
        self,
        q_start: np.ndarray,
        q_target: np.ndarray,
        relative_vel_factor: float = 0.1,
        relative_acc_factor: float = 0.05,
        relative_jerk_factor: float = 0.001,
    ) -> tuple[list[np.ndarray], float]:
        """Plans a move in joint space: a smooth joint trajectory (limited vel, acc and jerk)
        starting from q_start and ending at q_target, at rest at both ends.

        The joints go straight from q_start to q_target in joint space. They
        start together and arrive together, and at every moment each joint has
        gone the same fraction of its way, so no joint leaves the range between
        its start and its target. The move is as fast as the velocity,
        acceleration and jerk limits of every joint allow, scaled by
        relative_vel_factor, relative_acc_factor and relative_jerk_factor.

        No inverse kinematics is involved, so the robot ends at q_target itself.
        The end effector's path in between is whatever the joints make it, not a
        straight line: preview the trajectory before it runs on the robot.

        Parameters
        ----------
        q_start: np.ndarray
            The joint positions the trajectory starts at [rad], 7 numbers.
            Usually the current joint positions of the robot.
        q_target: np.ndarray
            The joint positions the trajectory ends at [rad], 7 numbers inside
            the Panda's joint limits (PANDA_POS_LIM_JOINT).
        relative_vel_factor : float
            0-1 multiplication factor to multiply PANDA_VEL_LIM_JOINT.
            Limits the velocity of every joint during the entire trajectory.
            The miniPC refuses a trajectory that is faster than its velocity
            scale, 0.3 unless it was set up otherwise.
        relative_acc_factor : float
            0-1 multiplication factor to multiply PANDA_ACC_LIM_JOINT.
            Limits the acceleration of every joint during the entire trajectory.
        relative_jerk_factor: float
            0-1 multiplication factor to multiply PANDA_JERK_LIM_JOINT.
            Limits the jerk of every joint during the entire trajectory.

        Returns
        ----------
        q_traj: list[np.ndarray]
            List of np.ndarray of joint positions with size of joint_dof of robot,
            from q_start (at time 0) to q_target: what the visualizer and the
            robot run. Where q_target is q_start, it is those two points.
        self.dt: float
            Delta time between trajectory points of the q_traj.

        Raises
        ----------
        ValueError
            If q_start or q_target is not 7 finite numbers, q_target is outside
            the Panda's joint limits, or a factor is not larger than 0 and at
            most 1.
        RuntimeError
            If the move would take longer than the miniPC runs a trajectory for,
            10 minutes: the factors are too small for it.
        """

        dof = len(self.PANDA_VEL_LIM_JOINT)

        def joint_positions(name, value):
            try:
                q = np.array(value, dtype=float)
            except (TypeError, ValueError):
                q = None
            if q is None or q.shape != (dof,) or not np.isfinite(q).all():
                raise ValueError(
                    f"{name} must be {dof} joint positions [rad], all finite numbers"
                )
            return q

        q_start = joint_positions("q_start", q_start)
        q_target = joint_positions("q_target", q_target)
        # The robot cannot go there, however the plan looks in the preview. Joint
        # positions given in degrees end here too.
        lowest, highest = self.PANDA_POS_LIM_JOINT
        outside = np.flatnonzero((q_target < lowest) | (q_target > highest))
        if outside.size > 0:
            j = outside[0]
            raise ValueError(
                f"q_target[{j}] = {q_target[j]:g} rad is outside the Panda's joint "
                f"limits, {lowest[j]:g} to {highest[j]:g} rad"
            )
        for name, factor in (
            ("relative_vel_factor", relative_vel_factor),
            ("relative_acc_factor", relative_acc_factor),
            ("relative_jerk_factor", relative_jerk_factor),
        ):
            if not 0.0 < factor <= 1.0:  # not a number (NaN) either
                raise ValueError(
                    f"{name} must be larger than 0 and at most 1, not {factor}"
                )
        # Ruckig never finishes with a time step of zero, and crashes with NaN.
        if not (np.isfinite(self.dt) and self.dt > 0.0):
            raise ValueError(f"dt must be larger than 0, not {self.dt}")

        delta = q_target - q_start
        distance = np.abs(delta)
        longest = distance.max()
        if longest == 0.0:
            return [q_start, q_target], self.dt  # nowhere to go: two points at rest

        # The move is planned as one number: how far the joint with the longest
        # way has gone. Every joint goes its own way in step with that, so the
        # path is a straight line in joint space. A joint with a shorter way is
        # slower in proportion, and lets that number change faster than its own
        # limits by as much.
        moving = distance > 0.0
        share = distance[moving] / longest
        # A hair inside the limits: rounding would otherwise put some point of a
        # plan made at a limit of the miniPC (a relative_vel_factor of 0.3) over it.
        inside = 1.0 - 1e-9

        def ruckig_limit(panda_limits, factor):
            return [inside * (factor * panda_limits[moving] / share).min()]

        # Ruckig keeps its state from one update to the next, as a controller
        # needs it to. A Ruckig of its own keeps each plan apart from the others.
        ruckig = Ruckig(dofs=1, delta_time=self.dt)
        inp = InputParameter(1)
        out = OutputParameter(1)
        inp.current_position = [0.0]
        inp.current_velocity = [0.0]
        inp.current_acceleration = [0.0]
        inp.target_position = [longest]
        inp.target_velocity = [0.0]
        inp.target_acceleration = [0.0]
        inp.max_velocity = ruckig_limit(self.PANDA_VEL_LIM_JOINT, relative_vel_factor)
        inp.max_acceleration = ruckig_limit(
            self.PANDA_ACC_LIM_JOINT, relative_acc_factor
        )
        inp.max_jerk = ruckig_limit(self.PANDA_JERK_LIM_JOINT, relative_jerk_factor)
        res = Result.Working

        travelled = [0.0]
        while res == Result.Working:
            res = ruckig.update(inp, out)  # one dt further along the move
            if res not in (Result.Working, Result.Finished):
                raise RuntimeError(f"Ruckig could not plan the trajectory: {res}")
            travelled.append(out.new_position[0])
            out.pass_to_input(inp)
            # Ruckig plans a move however long it takes, and with factors that
            # are far too small that is days: its points would fill the memory.
            if (len(travelled) - 1) * self.dt > MAX_TRAJECTORY_DURATION:
                raise RuntimeError(
                    f"The move would take more than {MAX_TRAJECTORY_DURATION:g} s, "
                    "longer than the miniPC runs a trajectory for. Use larger factors."
                )

        fraction = np.clip(np.array(travelled) / longest, 0.0, 1.0)
        q_traj = q_start + np.outer(fraction, delta)
        # Start and end at the given joint positions themselves: the miniPC
        # refuses a trajectory that does not start where the robot is.
        q_traj[0] = q_start
        q_traj[-1] = q_target
        return list(q_traj), self.dt
