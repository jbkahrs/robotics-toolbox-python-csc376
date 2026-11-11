"""The wire protocol between the GPU PCs (csc376_franky) and the robot server on
the station's miniPC (csc376_franky_server).

Each client holds one TCP connection. Every message is a JSON object on one line.

A request, client to server::

    {"op": "execute", "id": 7, "trajectory": [[q0, ..., q6], ...], "dt": 0.03}

A request without an "id" is a notification and gets no reply (the heartbeat).
The server answers each request once, when it has finished::

    {"id": 7, "ok": true, "result": {...}}
    {"id": 7, "ok": false, "error": {"code": -7, "name": "...", "message": "..."}}

and sends the robot's state to every client, state_rate times a second::

    {"event": "state", "time": ..., "robot": {...}, "trajectory": ..., "gripper": {...}}

Requests: hello (first), state, ping, heartbeat, execute, move_home, stop,
recover, gripper_homing, gripper_move, gripper_grasp, gripper_stop.
"""

import enum
import json
import math

PROTOCOL_VERSION = 1

# Every station's robot network has the same addresses: the miniPC is
# 192.168.1.100, the GPU PCs 192.168.1.110 and .120, and the Panda itself
# 192.168.1.107. Only the miniPC talks to the robot.
DEFAULT_HOST = "192.168.1.100"
DEFAULT_PORT = 7376
ROBOT_IP = "192.168.1.107"
HOST_ENV = "CSC376_ROBOT_HOST"
PORT_ENV = "CSC376_ROBOT_PORT"

# Large enough for a 10 minute trajectory sampled every 5 ms.
MAX_MESSAGE_BYTES = 32 * 1024 * 1024
# The client sends a heartbeat this often; a trajectory is stopped when its
# client has been silent for the server's owner timeout (2 s by default).
HEARTBEAT_INTERVAL = 0.5

# The home pose of libfranka's examples, where move_home goes.
HOME_JOINT_POSITIONS = (0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4)


class ErrorCodes(enum.IntEnum):
    """Why a request failed. Codes -1 to -8 keep the values of the old
    csc376_bind_franky.ErrorCodes; -1 to -20 and -25 mirror the C++ enum in
    server/include/csc376_franky/common.hpp."""

    Success = 1
    JointSizeNotSeven = -1
    JointDistancesAboveMaxJointDifference = -2
    JointVelocitiesAboveMaximumAllowed = -3
    DtIsNegative = -4
    DtIsSmallerThanAllowed = -5
    Interrupted = -6
    JointTrajectoryDoesNotStartAtCurrentPosition = -7
    TrajectorySizeLessThanTwo = -8
    NonFiniteValue = -9
    JointLimitsExceeded = -10
    TrajectoryTooLong = -11
    DtIsLargerThanAllowed = -12
    RobotBusy = -13
    RobotNotConnected = -14
    RobotNotReady = -15
    ControlException = -16
    TrackingErrorTooLarge = -17
    ConnectionLost = -18
    GripperNotConnected = -19
    GripperCommandFailed = -20
    # Raised by the server or the client only.
    InvalidRequest = -21
    IncompatibleVersion = -22
    ServerUnavailable = -23
    Timeout = -24
    JointAccelerationsAboveMaximumAllowed = -25


def encode(message):
    """One message as a line of JSON. NaN and infinity are refused (ValueError)."""
    return (json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n").encode()


def decode(line):
    """The message in one line of JSON; ValueError if it is not a JSON object."""
    message = json.loads(line)
    if not isinstance(message, dict):
        raise ValueError("a message must be a JSON object")
    return message
