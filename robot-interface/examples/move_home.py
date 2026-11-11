import numpy as np
from csc376_franky import FrankaJointTrajectoryController, protocol

def main():
    np.set_printoptions(precision=4, suppress=True)

    # Fraction of libfranka's joint speed and acceleration limits (at most 0.3)
    speed_factor = 0.2

    csc376_franky_robot = FrankaJointTrajectoryController() # The robot server on this station's miniPC

    print("q_current", csc376_franky_robot.get_current_joint_positions())
    print("q_home   ", np.array(protocol.HOME_JOINT_POSITIONS))

    # The miniPC moves the robot like libfranka's examples do (their MotionGenerator),
    # straight in joint space from wherever it is, so check that the way is clear
    yes_or_else = input("To move the real robot home, type [yes], then press enter\n")
    if yes_or_else != "yes":
        print("User did not type [yes], will not move the real robot")
        return

    csc376_franky_robot.move_home(speed_factor)
    print("q_current", csc376_franky_robot.get_current_joint_positions())

if __name__ == "__main__":
    main()
