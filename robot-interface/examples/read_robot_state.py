import numpy as np

from csc376_franky import FrankaJointTrajectoryController

# Connect to the robot server on this station's miniPC
controller = FrankaJointTrajectoryController()

# Get current state
current_pos = controller.get_current_joint_positions()
robot_state = controller.get_current_robot_state()

print(f"Current joint positions: {current_pos}")
print(f"Current joint velocities: {robot_state.dq}")
print(f"End-effector pose: {np.array(robot_state.O_T_EE).reshape(4, 4).transpose()}")
# Read RobotState in csc376_franky/client.py for the robot state elements
