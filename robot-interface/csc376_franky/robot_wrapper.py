from typing import Optional

import numpy as np


class RobotWrapper:
    """Runs a joint trajectory in the Swift preview, on the real robot, or on
    both one after the other, asking before each."""

    def __init__(self, has_visualizer: bool, run_on_real: bool,
                 panda_rtb_model, q_start: np.ndarray, ip_address: Optional[str] = None):
        # ip_address is the station's miniPC (the robot server), not the robot;
        # None means the default, 192.168.1.100 or $CSC376_ROBOT_HOST.
        self.real_robot = None
        if run_on_real:
            from csc376_franky.client import FrankaJointTrajectoryController
            self.real_robot = FrankaJointTrajectoryController(ip_address)
            q_start = self.real_robot.get_current_joint_positions() # start the preview where the robot is
        self.visualizer = None
        if has_visualizer:
            from csc376_franky.visualizer import RtbVisualizer
            self.visualizer = RtbVisualizer(panda_rtb_model, q_start, real_robot=self.real_robot)

    @property
    def vizualizer(self):
        """The old, misspelled name of visualizer."""
        return self.visualizer

    def get_current_joint_positions(self):
        if self.real_robot:
            return self.real_robot.get_current_joint_positions()
        if self.visualizer:
            return self.visualizer.q

    def run_joint_trajectory(self, q_traj: np.ndarray, dt: float):
        if self.visualizer:
            input("To run on the sim robot, press enter\n")
            self.visualizer.run_joint_trajectory(q_traj, dt)
        if self.real_robot:
            yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
            if yes_or_else != "yes":
                print("User did not type [yes], will not run on sim/real robot")
                return False
            self.real_robot.run_joint_trajectory(q_traj, dt)
        return True

    def stop(self):
        if self.visualizer:
            self.visualizer.stop()
        if self.real_robot:
            self.real_robot.close()
