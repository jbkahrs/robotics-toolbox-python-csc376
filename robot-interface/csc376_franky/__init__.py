"""CSC376 robot interface, for the GPU PCs of a robot station.

Plan trajectories (csc376_franky.motion_generator), preview them in Swift
(csc376_franky.vizualizer), and run them on the robot through the robot
server on the station's miniPC (FrankaJointTrajectoryController).
"""

__version__ = "2.0.0"

from .client import FrankaJointTrajectoryController, Gripper, GripperState, RobotError, RobotState  # noqa: E402
from .protocol import DEFAULT_HOST, DEFAULT_PORT, ErrorCodes  # noqa: E402

__all__ = [
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "ErrorCodes",
    "FrankaJointTrajectoryController",
    "Gripper",
    "GripperState",
    "RobotError",
    "RobotState",
]
