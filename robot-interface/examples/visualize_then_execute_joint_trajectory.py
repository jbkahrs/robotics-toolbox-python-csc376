import numpy as np
import roboticstoolbox as rtb

from csc376_franky import FrankaJointTrajectoryController, protocol
from csc376_franky.motion_generator import RuckigMotionGenerator
from csc376_franky.visualizer import RtbVisualizer


def main():
    np.set_printoptions(precision=4, suppress=True)

    # I. Speed factor settings: fractions of the Panda's joint velocity, acceleration
    # and jerk limits (the miniPC refuses a velocity factor above 0.3)
    relative_vel_factor = 0.1
    relative_acc_factor = 0.05
    relative_jerk_factor = 0.001

    # II. RTB, Ruckig, csc376_franky, and Visualizer setup
    panda_rtb_model = rtb.models.Panda()
    motion_generator = RuckigMotionGenerator()

    csc376_franky_robot = (
        FrankaJointTrajectoryController()
    )  # The robot server on this station's miniPC

    q_start = csc376_franky_robot.get_current_joint_positions()
    # The translucent blue Panda shows where the real robot is
    visualizer = RtbVisualizer(panda_rtb_model, q_start, real_robot=csc376_franky_robot)

    # III. Choose your goal: the joint positions to move to [rad]. Here the toolbox's
    # ready pose, or libfranka's home pose if the robot is at the ready pose already
    q_target = panda_rtb_model.qr
    if np.allclose(q_start, q_target, atol=0.01):
        q_target = np.array(protocol.HOME_JOINT_POSITIONS)
    print("q_start", q_start)
    print("q_target", q_target)

    # IV. Visualize the trajectory in simulation
    q_traj, dt = motion_generator.calculate_joint_trajectory(
        q_start,
        q_target,
        relative_vel_factor,
        relative_acc_factor,
        relative_jerk_factor,
    )  # Straight in joint space, without inverse kinematics: it ends at q_target itself
    print(f"The move takes {dt * (len(q_traj) - 1):.2f} s")
    input("Press enter, to run in visualizer\n")
    visualizer.run_joint_trajectory(q_traj, dt)

    # V. Run on real robot
    yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
    if yes_or_else != "yes":
        print("User did not type [yes], will not run on real robot")
        return visualizer

    # The miniPC refuses the trajectory if the robot moved since q_start was read
    csc376_franky_robot.run_joint_trajectory(q_traj, dt)
    return visualizer


if __name__ == "__main__":
    visualizer = main()
    visualizer.stop()  # Makes sure render thread ends
