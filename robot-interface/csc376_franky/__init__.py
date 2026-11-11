"""CSC376 robot interface, for the GPU PCs of a robot station.

Plan trajectories (csc376_franky.motion_generator), preview them in Swift
(csc376_franky.visualizer), and run them on the robot through the robot
server on the station's miniPC (FrankaJointTrajectoryController).
"""

__version__ = "2.1.0"

from .client import (
    FrankaJointTrajectoryController,
    Gripper,
    GripperState,
    RobotError,
    RobotState,
)
from .protocol import DEFAULT_HOST, DEFAULT_PORT, ErrorCodes

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
