import asyncio
import atexit
import swift
import os
import threading
import weakref
import numpy as np
import time
from typing import List

import roboticstoolbox as rtb
import spatialgeometry as sg
from spatialmath import SE3

# How the Panda that follows the real robot is drawn.
REAL_ROBOT_COLOR = (0.2, 0.6, 1.0)
REAL_ROBOT_OPACITY = 0.35

def translucent_panda(base_pose):
    """ A Panda that Swift can draw translucent. Swift ignores the opacity of
    Collada (.dae) meshes, which the Panda's links are, so its links are drawn
    with the toolbox's STL meshes of the Panda instead (same link frames). The
    base is left out: it never moves, and it would flicker inside the other Panda's. """
    model = rtb.models.Panda()
    model.base = base_pose
    model.links[0].geometry = []
    stl_dir = rtb.rtb_path_to_datafile("meshes/FRANKA-EMIKA/Panda")
    for link in list(model.links[1:]) + list(model.grippers[0].links):
        meshes = []
        for shape in link.geometry:
            name = os.path.basename(getattr(shape, "filename", None) or "")
            stl = os.path.join(stl_dir, name.replace(".dae", ".stl"))
            if name.endswith(".dae") and os.path.exists(stl):
                shape = sg.Mesh(stl, pose=shape._T, color=REAL_ROBOT_COLOR)
            meshes.append(shape)
        link.geometry = meshes
    return model

# Code taken mostly from: https://github.com/Patrick15Yao/franky_toolbox_bridge,
# Please credit Patrick Yao, github id Patrick15Yao for this work.
class RtbVisualizer:
    def __init__(self, rtb_robot_model, q_start,
                 base_pose = SE3(0, 0, 0), gripper_q = [0.01, 0.01], real_robot = None):
        """ Shows a Panda in Swift, to preview joint trajectories before they run on the robot.

        Parameters
        ----------
        rtb_robot_model:
            Unused: the visualizer shows a Panda model of its own.
        q_start: np.ndarray
            Joint positions to show the Panda in. Usually the real robot's current joint positions.
        real_robot: FrankaJointTrajectoryController, optional
            If given, a translucent blue Panda follows the real robot live,
            so the preview can be compared with where the robot really is.
        """
        # self.rtb_robot_model = copy.deepcopy(rtb_robot_model) # Copy makes robot destroyed in visualizer? odd.
        self.rtb_robot_model = rtb.models.Panda()
        self.rtb_robot_model.base = base_pose
        self.rtb_robot_model.grippers[0].q = gripper_q
        self.real_robot = real_robot
        self.real_robot_model = None
        self._real_handle = None
        self._is_render_running = False
        self._rtb_rendering_thread = None
        self._swift_closed = False
        self.env = swift.Swift()
        try:
            self.env.launch()
            # Swift leaves a thread waiting on the browser that keeps Python from
            # exiting, so close Swift at exit if stop() was never called (an
            # exception, Ctrl+C). threading's exit hooks run before Python waits
            # for that thread; atexit's would run after.
            def stop_at_exit(visualizer=weakref.ref(self)):
                if visualizer() is not None:
                    visualizer().stop()
            getattr(threading, "_register_atexit", atexit.register)(stop_at_exit)
            # Swift 2 keeps each robot's joint positions in a handle; readonly
            # so that Swift only shows the q it is given.
            self.handle = self.env.add_robot(self.rtb_robot_model, readonly=True)
            self.handle.q = np.array(q_start, dtype=float)
            if real_robot is not None:
                self.real_robot_model = translucent_panda(base_pose)
                self._real_handle = self.env.add_robot(self.real_robot_model, robot_alpha=REAL_ROBOT_OPACITY,
                                                       readonly=True)
                self._real_handle.q = np.array(q_start, dtype=float)
        except Exception as e:
            print(f"[RTB Process {os.getpid()}] Error launching Swift: {e}")
            self.__close_swift()
            raise

        self.render_period = 0.02 # 50 hz, should be faster than run_joint_traj dt
        self._is_render_running = True
        self._rtb_rendering_thread = threading.Thread(target=self.__run_render_loop, daemon=True)
        self._rtb_rendering_thread.start()

        self.trajectory_callback = lambda index: None

    @property
    def q(self) -> np.ndarray:
        """ Joint positions the previewed Panda is shown in. """
        return np.array(self.handle.q)

    def move_gripper(self, target_width: float, speed: float):
        """ Moves the simulations Franka gripper to the target_width.

        Parameters
        ----------
        target_width: float
            Target width of the Franka gripper.
        speed: float
            Delay to when the gripper finishes moving
        """
        # TODO, Radian: I am too lazy to make this propely move
        # For now the gripper will just jump

        # Get current gripper state (assuming gripper width is stored in first gripper)
        if hasattr(self.rtb_robot_model, 'grippers') and len(self.rtb_robot_model.grippers) > 0:
            # Current gripper finger positions (each finger is half the total width)
            current_finger_pos = self.rtb_robot_model.grippers[0].q[0] if len(self.rtb_robot_model.grippers[0].q) > 0 else 0.02
            current_width = current_finger_pos * 2.0

            # Calculate movement distance and time
            distance = abs(target_width - current_width)
            movement_time = distance / speed if speed > 0 else 0.1
            movement_time +=0.5

            # Update gripper directly to target width
            finger_pos = target_width / 2.0
            self.rtb_robot_model.grippers[0].q = [finger_pos, finger_pos]

            # Sleep for the calculated movement time to simulate realistic timing
            time.sleep(movement_time)
            return True
        else:
            return False
    def set_trajectory_callback(self, callback):
        """ Sets a callback function that is called with the current trajectory index during execution.

        Parameters
        ----------
        callback: function
            A function that takes an integer index as input.
        """
        self.trajectory_callback = callback

    def run_joint_trajectory(self, q_trajectory: List[np.ndarray], dt: float):
        """ Updates the rtb_robot_model's (that was passed to the env) joint positions
        with the joint trajectory (q_trajectory) with a time delay of (dt),
        mimicking what will happen on the real robot control.

        Parameters
        -----------
        q_trajectory: List[np.ndarray]
            List of np.ndarray of joint positions with size of joint_dof of robot.
        self.dt: float
            Delta time between trajectory points of the cartsian_pose_traj.
        """
        for i, q in enumerate(q_trajectory):
            self.handle.q = np.array(q, dtype=float)
            self.trajectory_callback(i)
            time.sleep(dt)

    def stop(self):
        """ Cleanly stops the rendering thread and closes Swift. """
        self._is_render_running = False
        if self._rtb_rendering_thread is not None:
            self._rtb_rendering_thread.join()
        self.__close_swift()

    def __close_swift(self):
        # Swift's close() stops its event loop while the browser is still
        # connected, and on exit Python then complains about every websocket task
        # left behind ("Task was destroyed but it is pending!"). Closing the
        # websocket server first ends the connection cleanly.
        if self._swift_closed:
            return
        self._swift_closed = True
        socket = getattr(self.env, "socket", None)
        if socket is None and not self.env.headless:
            return  # launch() failed before Swift was connected: nothing to close
        try:
            server = getattr(socket, "_server", None)
            if server is not None:
                async def close_server():
                    server.close()
                    await server.wait_closed()
                asyncio.run_coroutine_threadsafe(close_server(), socket.loop).result(timeout=2)
            self.env.close()
        except Exception as e:
            print(f"[RTB Process] Error closing Swift: {e!r}")

    def __update_real_robot(self):
        status = self.real_robot.get_status()
        robot = status["robot"] if status else {}
        if "q" in robot:
            self._real_handle.q = np.array(robot["q"], dtype=float)
        gripper = status["gripper"] if status else {}
        if "width" in gripper:
            self.real_robot_model.grippers[0].q = [gripper["width"] / 2.0] * 2

    def __run_render_loop(self):
        while self._is_render_running:
            try:
                if self._real_handle is not None:
                    self.__update_real_robot()
                self.env.step(self.render_period)
                time.sleep(self.render_period)
            except Exception as e:
                print(f"[RTB Process] Error in rendering loop: {e}")
                break
