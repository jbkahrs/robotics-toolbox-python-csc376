from csc376_franky import Gripper

# Connect to the gripper through the robot server on this station's miniPC
csc376_gripper = Gripper()
gripper_state = csc376_gripper.state

print(f"Current width: {gripper_state.width}")
print(f"Is gripper grasped: {gripper_state.is_grasped}")
