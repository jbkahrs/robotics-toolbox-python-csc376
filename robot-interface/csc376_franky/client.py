"""The station's robot, seen from a GPU PC.

The miniPC next to the robot runs the controller (csc376_franky_server); this
PC plans and visualizes. FrankaJointTrajectoryController and Gripper here are
network connections to that server, with the methods of the old
csc376_bind_franky classes, which ran on the miniPC itself.

Both GPU PCs of a station can be connected at the same time. Each receives
the robot's state 50 times a second, so it can plan from where the robot is.
The robot runs one trajectory at a time: the miniPC refuses a trajectory while
another one is running, and one that does not start where the robot is.
"""

import concurrent.futures
import itertools
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np

from . import __version__, protocol
from .protocol import ErrorCodes

# A state older than this is not trusted: the server sends one every 20 ms.
STATE_TIMEOUT = 1.0


class RobotError(RuntimeError):
    """A request to the robot failed; code (an ErrorCodes) says why."""

    def __init__(self, code, message):
        try:
            self.code = ErrorCodes(code)
        except ValueError:
            self.code = ErrorCodes.InvalidRequest
        self.message = message
        super().__init__(f"{self.code.name}: {message}")


@dataclass(frozen=True)
class RobotState:
    """The robot's state, measured on the miniPC. Names follow franka::RobotState.

    O_T_EE is the end-effector (Franka Hand TCP) pose in the base frame as 16
    numbers in column-major order: np.array(state.O_T_EE).reshape(4, 4).T is
    the 4 x 4 matrix.
    """

    q: tuple
    dq: tuple
    tau_J: tuple
    tau_ext_hat_filtered: tuple
    O_T_EE: tuple
    O_F_ext_hat_K: tuple
    elbow: tuple
    robot_mode: str
    time: float
    control_command_success_rate: float
    current_errors: tuple
    last_motion_errors: tuple


@dataclass(frozen=True)
class GripperState:
    width: float
    max_width: float
    is_grasped: bool
    temperature: float
    time: float


def _default_host():
    return os.environ.get(protocol.HOST_ENV) or protocol.DEFAULT_HOST


def _default_port():
    return int(os.environ.get(protocol.PORT_ENV) or protocol.DEFAULT_PORT)


def _map_future(future, convert):
    mapped = concurrent.futures.Future()

    def done(f):
        try:
            mapped.set_result(convert(f.result()))
        except BaseException as error:
            mapped.set_exception(error)

    future.add_done_callback(done)
    return mapped


class _Connection:
    """One TCP connection to the robot server, with a reader and a heartbeat thread."""

    def __init__(self, host, port, timeout):
        self.host, self.port = host, port
        try:
            self._sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as error:
            raise RobotError(ErrorCodes.ServerUnavailable, self._unavailable(error)) from None
        self._sock.settimeout(None)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._file = self._sock.makefile("rb")
        self._send_lock = threading.Lock()
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self._pending = {}
        self._closed = False
        self._close_reason = ""
        self._state = None
        self._state_time = 0.0
        self._state_changed = threading.Condition()
        threading.Thread(target=self._read_loop, name="csc376-robot-reader", daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, name="csc376-robot-heartbeat", daemon=True).start()

    def _unavailable(self, error):
        text = f"Cannot reach the robot server at {self.host}:{self.port} ({error})."
        if self.host == protocol.ROBOT_IP:
            text += (f" {protocol.ROBOT_IP} is the robot itself: the robot server runs on the station's miniPC, "
                     f"{protocol.DEFAULT_HOST}.")
        else:
            text += (" Check that this PC is on the station's robot network and that the robot server is running "
                     "on the miniPC.")
        return text

    @property
    def closed(self):
        return self._closed

    def closed_error(self):
        return RobotError(ErrorCodes.ConnectionLost, f"Not connected to the robot server: {self._close_reason}")

    def request_async(self, op, **fields):
        future = concurrent.futures.Future()
        with self._lock:
            if self._closed:
                future.set_exception(self.closed_error())
                return future
            request_id = next(self._ids)
            self._pending[request_id] = future
        future.request_id = request_id
        try:
            self._send({"op": op, "id": request_id, **fields})
        except BaseException as error:
            with self._lock:
                self._pending.pop(request_id, None)
            if not future.done():
                future.set_exception(error)
        return future

    def request(self, op, timeout=10.0, **fields):
        future = self.request_async(op, **fields)
        try:
            return future.result(timeout)
        except concurrent.futures.TimeoutError:
            raise RobotError(ErrorCodes.Timeout, f"The robot server did not answer {op!r} within {timeout} s") from None

    def notify(self, op, **fields):
        self._send({"op": op, **fields})

    def _send(self, message):
        try:
            data = protocol.encode(message)
        except ValueError:
            raise RobotError(ErrorCodes.NonFiniteValue, "Values sent to the robot must be finite numbers") from None
        with self._send_lock:
            if self._closed:
                raise self.closed_error()
            try:
                self._sock.sendall(data)
            except OSError as error:
                self._close(f"connection lost ({error})")
                raise self.closed_error() from None

    def latest_state(self):
        """The last state message and how many seconds ago it arrived."""
        with self._state_changed:
            return self._state, time.monotonic() - self._state_time

    def wait_for_state(self, predicate, timeout):
        """The first state message, new or already received, that predicate accepts; None on timeout."""
        with self._state_changed:
            ready = self._state_changed.wait_for(
                lambda: self._closed or (self._state is not None and predicate(self._state)), timeout)
            if ready and not self._closed:
                return self._state
            return None

    def _read_loop(self):
        reason = "the robot server closed the connection"
        try:
            for line in self._file:
                message = protocol.decode(line)
                if message.get("event") == "state":
                    with self._state_changed:
                        self._state = message
                        self._state_time = time.monotonic()
                        self._state_changed.notify_all()
                elif "id" in message:
                    with self._lock:
                        future = self._pending.pop(message["id"], None)
                    if future is None or future.done():
                        continue
                    if message.get("ok"):
                        future.set_result(message.get("result") or {})
                    else:
                        error = message.get("error") or {}
                        future.set_exception(RobotError(error.get("code", ErrorCodes.InvalidRequest),
                                                        error.get("message", "")))
        except (OSError, ValueError) as error:
            reason = f"connection lost ({error})"
        finally:
            self._close(reason)
            self._file.close()

    def _heartbeat_loop(self):
        while True:
            time.sleep(protocol.HEARTBEAT_INTERVAL)
            try:
                self.notify("heartbeat")
            except RobotError:
                return

    def _close(self, reason):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._close_reason = reason
            pending = list(self._pending.values())
            self._pending.clear()
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()
        for future in pending:
            if not future.done():
                future.set_exception(self.closed_error())
        with self._state_changed:
            self._state_changed.notify_all()

    def close(self):
        self._close("closed by this program")


class FrankaJointTrajectoryController:
    """The station's Panda, through the robot server on the miniPC.

    host and port default to $CSC376_ROBOT_HOST and $CSC376_ROBOT_PORT, or the
    miniPC's address on the station's robot network (192.168.1.100:7376).
    The constructor waits until the miniPC is connected to the robot, so FCI
    must be active in Desk. Use it as a context manager, or call close().
    """

    def __init__(self, host=None, port=None, *, timeout=10.0, wait_for_robot=True):
        self.host = host or _default_host()
        self.port = int(port or _default_port())
        self._connection = _Connection(self.host, self.port, timeout)
        try:
            self.server_info = self._connection.request(
                "hello", timeout=timeout, protocol=protocol.PROTOCOL_VERSION,
                client={"host": socket.gethostname(), "program": os.path.basename(sys.argv[0]) or "python",
                        "library": f"csc376_franky {__version__}"})
            if wait_for_robot:
                self.wait_for_robot(timeout)
        except BaseException:
            self._connection.close()
            raise
        self.session = self.server_info["session"]
        self.gripper = Gripper(_connection=self._connection) if self.server_info.get("gripper") else None
        self._trajectory_callback = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        """Disconnect. A trajectory this program started is stopped."""
        self._connection.close()

    # ---- state --------------------------------------------------------------

    def wait_for_robot(self, timeout=10.0):
        """Wait until the miniPC is connected to the robot and reports its state."""
        if self._connection.wait_for_state(lambda s: "q" in s["robot"], timeout) is None:
            if self._connection.closed:
                raise self._connection.closed_error()
            state, _ = self._connection.latest_state()
            error = state["robot"].get("error") if state else None
            raise RobotError(
                ErrorCodes.RobotNotConnected,
                "The miniPC cannot reach the robot" + (f" ({error})" if error else "") +
                ". Check that the robot is on, its brakes are open and FCI is active in Desk.")

    def _robot(self):
        if self._connection.closed:
            raise self._connection.closed_error()
        state, age = self._connection.latest_state()
        if state is None or age > STATE_TIMEOUT:
            raise RobotError(ErrorCodes.ConnectionLost, "No robot state from the robot server for over a second")
        robot = state["robot"]
        if "q" not in robot:
            error = robot.get("error")
            raise RobotError(ErrorCodes.RobotNotConnected,
                             "The miniPC is not connected to the robot" + (f": {error}" if error else ""))
        return robot

    def get_current_robot_state(self):
        """The latest RobotState from the miniPC (at most 20 ms old)."""
        robot = self._robot()
        fields = {name: robot[name] for name in RobotState.__dataclass_fields__}
        return RobotState(**{name: tuple(value) if isinstance(value, list) else value
                             for name, value in fields.items()})

    def get_current_joint_positions(self):
        """The measured joint positions [rad], as an array of 7."""
        return np.array(self._robot()["q"], dtype=float)

    def get_current_pose(self):
        """The measured end-effector pose (Franka Hand TCP) in the base frame, as an SE3.

        This is the robot's own forward kinematics. It agrees with
        rtb.models.Panda().fkine(q) when Desk's end effector is the Franka Hand.
        """
        from spatialmath import SE3
        from spatialmath.base import trnorm

        return SE3(trnorm(np.array(self._robot()["O_T_EE"], dtype=float).reshape(4, 4).T))

    def get_status(self):
        """The server's latest state message as a dict: robot, trajectory and gripper."""
        state, _ = self._connection.latest_state()
        return state

    @property
    def is_executing(self):
        """True while the robot runs a trajectory, for this or any other client."""
        state, _ = self._connection.latest_state()
        return bool(state and state.get("trajectory"))

    def wait_until_idle(self, timeout=None):
        """Wait until no trajectory is running; False on timeout."""
        state = self._connection.wait_for_state(
            lambda s: not s.get("trajectory") and s["robot"].get("status") == "idle", timeout)
        if state is None and self._connection.closed:
            raise self._connection.closed_error()
        return state is not None

    # ---- motion -------------------------------------------------------------

    def set_trajectory_callback(self, trajectory_callback):
        """Call trajectory_callback(index) for each point of a running trajectory.

        It runs in the thread that called run_joint_trajectory, as the robot
        passes each point (a few points at a time, at the state rate).
        """
        self._trajectory_callback = trajectory_callback

    def run_joint_trajectory(self, joint_trajectory, dt):
        """Execute a joint trajectory on the robot and wait until the robot has stopped.

        joint_trajectory is N x 7 joint positions [rad], dt seconds apart. The
        first must be the robot's current joint positions: plan from
        get_current_joint_positions(). Returns ErrorCodes.Success, and raises
        RobotError if the miniPC refused the trajectory or it did not finish
        (e.code says why). Ctrl+C stops the robot along its path, then raises
        KeyboardInterrupt.
        """
        try:
            trajectory = np.asarray(joint_trajectory, dtype=float)
        except (TypeError, ValueError):
            raise RobotError(ErrorCodes.JointSizeNotSeven,
                             "joint_trajectory must be a list of joint positions, each 7 numbers") from None
        if trajectory.ndim != 2 or trajectory.shape[1] != 7:
            raise RobotError(ErrorCodes.JointSizeNotSeven,
                             f"joint_trajectory must be N x 7, not {'x'.join(map(str, trajectory.shape))}")
        if not np.isfinite(trajectory).all() or not np.isfinite(dt):
            raise RobotError(ErrorCodes.NonFiniteValue, "joint_trajectory and dt must be finite numbers")

        future = self._connection.request_async("execute", trajectory=trajectory.tolist(), dt=float(dt))
        index = -1
        try:
            while True:
                try:
                    future.result(timeout=0.02)
                    break
                except concurrent.futures.TimeoutError:
                    index = self._report_progress(future.request_id, index)
        except KeyboardInterrupt:
            self._stop_after_interrupt(future)
            raise
        self._report_progress(future.request_id, index, end=len(trajectory) - 1)
        return ErrorCodes.Success

    def _report_progress(self, request_id, index, end=None):
        if self._trajectory_callback is None:
            return index
        if end is None:
            state, _ = self._connection.latest_state()
            trajectory = state.get("trajectory") if state else None
            if not trajectory or trajectory["session"] != self.session or trajectory["request"] != request_id:
                return index
            end = trajectory["index"]
        for i in range(index + 1, end + 1):
            self._trajectory_callback(i)
        return max(index, end)

    def move_home(self, speed_factor=0.2):
        """Move to libfranka's home pose (protocol.HOME_JOINT_POSITIONS) and wait until the robot is there.

        The miniPC moves the robot as libfranka's examples do: their MotionGenerator,
        followed by the robot's own joint position controller, from wherever the
        robot is. Nothing is planned on this PC. speed_factor scales libfranka's
        joint velocity and acceleration limits, up to the server's velocity scale
        (0.3 by default). Returns ErrorCodes.Success and raises RobotError like
        run_joint_trajectory; Ctrl+C stops the robot, then raises KeyboardInterrupt.
        """
        if not np.isfinite(speed_factor):
            raise RobotError(ErrorCodes.NonFiniteValue, "speed_factor must be a finite number")
        future = self._connection.request_async("move_home", speed_factor=float(speed_factor))
        try:
            while True:
                try:
                    future.result(timeout=0.1)
                    break
                except concurrent.futures.TimeoutError:
                    pass
        except KeyboardInterrupt:
            self._stop_after_interrupt(future)
            raise
        return ErrorCodes.Success

    def _stop_after_interrupt(self, future):
        print("\nStopping the robot...", file=sys.stderr)
        try:
            self._connection.request("stop", timeout=2.0)
            future.result(timeout=5.0)
        except (RobotError, concurrent.futures.TimeoutError):
            pass

    def stop(self):
        """Stop the running trajectory, whichever client started it: the robot
        slows to rest along its path within about half a second."""
        self._connection.request("stop")

    def recover(self):
        """Clear the robot's error after a reflex (e.g. a collision) so it can move
        again. Check the robot and its surroundings first."""
        self._connection.request("recover", timeout=30.0)

    def move_gripper(self, width, speed=0.1):
        """Move the gripper fingers to width [m] at speed [m/s]."""
        if self.gripper is None:
            raise RobotError(ErrorCodes.GripperNotConnected, "The robot server runs without a gripper")
        return self.gripper.move(width, speed)

    # Kept so programs written for csc376_bind_franky still run.
    @staticmethod
    def setupSignalHandler():
        """Not needed any more: Ctrl+C during run_joint_trajectory stops the robot."""

    def join(self):
        """Not needed any more: the control loop runs on the miniPC."""


class Gripper:
    """The station's Franka Hand, through the robot server on the miniPC.

    Usually reached as FrankaJointTrajectoryController(...).gripper. Created on
    its own, it opens a connection of its own (same host and port defaults).
    """

    def __init__(self, host=None, port=None, *, timeout=10.0, _connection=None):
        self._owns_connection = _connection is None
        if _connection is None:
            _connection = _Connection(host or _default_host(), int(port or _default_port()), timeout)
            try:
                info = _connection.request(
                    "hello", timeout=timeout, protocol=protocol.PROTOCOL_VERSION,
                    client={"host": socket.gethostname(), "program": os.path.basename(sys.argv[0]) or "python",
                            "library": f"csc376_franky {__version__}"})
                if not info.get("gripper"):
                    raise RobotError(ErrorCodes.GripperNotConnected, "The robot server runs without a gripper")
                _connection.wait_for_state(lambda s: "width" in s["gripper"], timeout)
            except BaseException:
                _connection.close()
                raise
        self._connection = _connection

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        if self._owns_connection:
            self._connection.close()

    def _command_async(self, op, **fields):
        return _map_future(self._connection.request_async(op, **fields), lambda result: result["success"])

    def _command(self, op, **fields):
        future = self._command_async(op, **fields)
        try:
            return future.result(timeout=60.0)
        except concurrent.futures.TimeoutError:
            raise RobotError(ErrorCodes.Timeout, f"The gripper did not finish {op} within 60 s") from None

    def homing(self):
        """Calibrate the fingers by opening and closing them fully."""
        return self._command("gripper_homing")

    def move(self, width, speed):
        """Move the fingers to width [m] at speed [m/s]; True when done."""
        return self._command("gripper_move", width=width, speed=speed)

    def grasp(self, width, speed, force, epsilon_inner=0.005, epsilon_outer=0.005):
        """Close on an object of width [m] with force [N]. True if the object was
        grasped: the fingers stopped between width - epsilon_inner and width + epsilon_outer."""
        return self._command("gripper_grasp", width=width, speed=speed, force=force,
                             epsilon_inner=epsilon_inner, epsilon_outer=epsilon_outer)

    def open(self, speed):
        """Open the fingers fully."""
        return self.move(self.max_width, speed)

    def stop(self):
        """Stop the fingers, whichever client moved them."""
        return self._command("gripper_stop")

    def homing_async(self):
        return self._command_async("gripper_homing")

    def move_async(self, width, speed):
        return self._command_async("gripper_move", width=width, speed=speed)

    def grasp_async(self, width, speed, force, epsilon_inner=0.005, epsilon_outer=0.005):
        return self._command_async("gripper_grasp", width=width, speed=speed, force=force,
                                   epsilon_inner=epsilon_inner, epsilon_outer=epsilon_outer)

    def open_async(self, speed):
        return self.move_async(self.max_width, speed)

    def stop_async(self):
        return self._command_async("gripper_stop")

    @property
    def state(self):
        """The latest GripperState from the miniPC."""
        if self._connection.closed:
            raise self._connection.closed_error()
        state, age = self._connection.latest_state()
        gripper = state["gripper"] if state is not None and age <= STATE_TIMEOUT else {}
        if "width" not in gripper:
            error = gripper.get("error")
            raise RobotError(ErrorCodes.GripperNotConnected,
                             "The miniPC is not connected to the gripper" + (f": {error}" if error else ""))
        return GripperState(**{name: gripper[name] for name in GripperState.__dataclass_fields__})

    @property
    def width(self):
        return self.state.width

    @property
    def is_grasped(self):
        return self.state.is_grasped

    @property
    def max_width(self):
        return self.state.max_width
