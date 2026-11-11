"""The robot server's network side: one session per client connection.

Any number of clients (normally the station's two GPU PCs) can connect at
once. All of them receive the robot's state; one trajectory runs at a time.
A trajectory is refused while another one, or another client's gripper
command, is running, and when it does not start where the robot is. Any
client can stop the running trajectory. A trajectory is also stopped when its
client disconnects or stops sending heartbeats.

The robot connection is opened when the first client connects and closed
idle_release seconds after the last one leaves, so the robot is free for
other FCI programs (Desk tasks, for example) the rest of the time.
"""

import asyncio
import itertools
import logging
import socket
import time
import uuid
from dataclasses import dataclass, field

import numpy as np
from csc376_franky import protocol
from csc376_franky.protocol import ErrorCodes

from . import __version__

log = logging.getLogger("csc376_franky_server")

MONITOR_PERIOD = 0.01
MAX_STATE_RATE = 100.0
# Drop state messages, but never replies, for a client that is this far behind.
MAX_BUFFERED_STATE = 256 * 1024


class RequestError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = ErrorCodes(code)
        self.message = message


@dataclass(eq=False)
class Session:
    id: int
    peer: str
    writer: asyncio.StreamWriter
    state_period: float
    host: str = "?"
    hello: bool = False
    last_seen: float = field(default_factory=time.monotonic)
    next_state: float = 0.0

    @property
    def label(self):
        return f"{self.host} ({self.peer})"


@dataclass(eq=False)
class Execution:
    session: Session
    request: object
    points: int
    duration: float
    future: asyncio.Future
    # The task that answers the request once the motion has ended.
    task: asyncio.Task
    # Why it was stopped, if it was, for the reply's message.
    stopped_by: str = ""


def _configure_socket(sock):
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    # Notice a client that vanished (power cut, cable pulled) within about 10 s.
    for option, value in (("TCP_KEEPIDLE", 4), ("TCP_KEEPINTVL", 2), ("TCP_KEEPCNT", 3)):
        if hasattr(socket, option):
            sock.setsockopt(socket.IPPROTO_TCP, getattr(socket, option), value)


def _number(message, name, default=None, minimum=None):
    value = message.get(name, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise RequestError(ErrorCodes.InvalidRequest, f"{name} must be a finite number")
    if minimum is not None and value < minimum:
        raise RequestError(ErrorCodes.InvalidRequest, f"{name} must be at least {minimum}")
    return float(value)


class RobotServer:
    def __init__(self, controller, gripper=None, *, state_rate=50.0, idle_release=30.0, owner_timeout=2.0):
        self.controller = controller
        self.gripper = gripper
        self.state_rate = min(float(state_rate), MAX_STATE_RATE)
        self.idle_release = idle_release
        self.owner_timeout = owner_timeout
        self._sessions = {}
        self._session_ids = itertools.count(1)
        self._executions = {}
        self._recoveries = {}
        self._gripper_owner = None
        self._idle_since = float("-inf")
        self._wanted = False
        self._snapshot = controller.snapshot()
        self._gripper_snapshot = gripper.snapshot() if gripper is not None else None
        self._last_connected = False
        self._last_connection_error = None
        self._server = None
        self._monitor_task = None
        self._closed = False

    async def start(self, host, port):
        self._server = await asyncio.start_server(self._handle, host, port, limit=protocol.MAX_MESSAGE_BYTES)
        self._monitor_task = asyncio.create_task(self._monitor())
        return self._server

    @property
    def port(self):
        return self._server.sockets[0].getsockname()[1]

    async def close(self):
        """Stop serving, stop any motion and disconnect from the robot."""
        if self._closed:
            return
        self._closed = True
        if self._server is not None:
            self._server.close()
        # Stop a running motion and tell its client how it ended before the
        # connections close.
        executions = list(self._executions.values())
        for execution in executions:
            if not execution.stopped_by:
                execution.stopped_by = "because the robot server is shutting down"
        self.controller.stop()
        if executions and self._monitor_task is not None:
            limits = self.controller.limits
            await asyncio.wait([execution.task for execution in executions],
                               timeout=limits.stop_time + limits.settle_timeout + 1.0)
        for session in list(self._sessions.values()):
            session.writer.close()
        if self._monitor_task is not None:
            self._monitor_task.cancel()
        await asyncio.to_thread(self.controller.shutdown)
        if self.gripper is not None:
            await asyncio.to_thread(self.gripper.shutdown)

    # ---- connections --------------------------------------------------------

    async def _handle(self, reader, writer):
        peer = writer.get_extra_info("peername")
        sock = writer.get_extra_info("socket")
        if sock is not None:
            _configure_socket(sock)
        session = Session(next(self._session_ids), peer[0] if peer else "?", writer, 1.0 / self.state_rate)
        self._sessions[session.id] = session
        log.info("session %d: connected from %s", session.id, session.peer)
        tasks = set()
        try:
            while True:
                try:
                    line = await reader.readline()
                except ValueError:
                    log.warning("session %d: message over %d bytes, closing", session.id, protocol.MAX_MESSAGE_BYTES)
                    self._write(session, self._error_reply(None, RequestError(
                        ErrorCodes.InvalidRequest, f"Message larger than {protocol.MAX_MESSAGE_BYTES} bytes")))
                    break
                except ConnectionError:
                    break
                if not line:
                    break
                session.last_seen = time.monotonic()
                try:
                    message = protocol.decode(line)
                except ValueError:
                    self._write(session, self._error_reply(
                        None, RequestError(ErrorCodes.InvalidRequest, "Each message must be one JSON object per line")))
                    continue
                task = asyncio.create_task(self._dispatch(session, message))
                tasks.add(task)
                task.add_done_callback(tasks.discard)
        finally:
            del self._sessions[session.id]
            self._session_closed(session)
            for task in tasks:
                task.cancel()
            writer.close()
            try:
                await writer.wait_closed()
            except ConnectionError:
                pass
            log.info("session %d: %s disconnected", session.id, session.label)

    def _session_closed(self, session):
        if self._gripper_owner is session:
            # The gripper command finishes on its own; it is short and harmless.
            self._gripper_owner = None
        for execution in self._executions.values():
            if execution.session is session and not execution.stopped_by:
                execution.stopped_by = f"because {session.label} disconnected"
                log.warning("session %d: disconnected during its trajectory, stopping the robot", session.id)
                self.controller.stop()

    def _write(self, session, message):
        if not session.writer.is_closing():
            session.writer.write(protocol.encode(message))

    @staticmethod
    def _error_reply(request_id, error):
        return {"id": request_id, "ok": False,
                "error": {"code": int(error.code), "name": error.code.name, "message": error.message}}

    async def _dispatch(self, session, message):
        request_id = message.get("id")
        op = message.get("op")
        handler = getattr(self, f"_op_{op}", None) if isinstance(op, str) else None
        try:
            if handler is None:
                raise RequestError(ErrorCodes.InvalidRequest, f"Unknown request {op!r}")
            if not session.hello and op not in ("hello", "heartbeat"):
                raise RequestError(ErrorCodes.InvalidRequest, "The first request must be hello")
            result = await handler(session, message)
            reply = {"id": request_id, "ok": True, "result": result or {}}
        except RequestError as error:
            reply = self._error_reply(request_id, error)
        except Exception:
            reference = uuid.uuid4().hex[:8]
            log.exception("session %d: %r request failed (ref %s)", session.id, op, reference)
            reply = self._error_reply(request_id, RequestError(
                ErrorCodes.InvalidRequest, f"The robot server could not handle the request (ref {reference})"))
        if request_id is not None:
            self._write(session, reply)
            try:
                await session.writer.drain()
            except ConnectionError:
                pass

    # ---- requests -----------------------------------------------------------

    async def _op_hello(self, session, message):
        version = message.get("protocol")
        if version != protocol.PROTOCOL_VERSION:
            raise RequestError(
                ErrorCodes.IncompatibleVersion,
                f"This robot server speaks protocol {protocol.PROTOCOL_VERSION} but the client speaks {version}. "
                "Use the csc376_franky version that matches the server (ask your TA).")
        client = message.get("client")
        if isinstance(client, dict):
            session.host = str(client.get("host", "?"))[:64]
        if "state_rate" in message:
            rate = _number(message, "state_rate", minimum=0.0)
            session.state_period = 1.0 / min(rate, MAX_STATE_RATE) if rate > 0 else 0.0
        session.hello = True
        self._update_connection_wanted(time.monotonic())
        log.info("session %d: %s says hello", session.id, session.label)
        limits = self.controller.limits
        return {
            "protocol": protocol.PROTOCOL_VERSION,
            "server": f"csc376_franky_server {__version__}",
            "session": session.id,
            "backend": self.controller.backend,
            "robot": self.controller.address,
            "gripper": self.gripper is not None,
            "state_rate": 1.0 / session.state_period if session.state_period > 0 else 0.0,
            "owner_timeout": self.owner_timeout,
            "limits": {
                "start_tolerance": limits.start_tolerance,
                "velocity_scale": limits.velocity_scale,
                "acceleration_scale": limits.acceleration_scale,
                "max_joint_step": limits.max_joint_step,
                "joint_limit_margin": limits.joint_limit_margin,
                "tracking_error_limit": limits.tracking_error_limit,
                "min_dt": limits.min_dt,
                "max_dt": limits.max_dt,
                "max_duration": limits.max_duration,
                "max_points": limits.max_points,
                "stop_time": limits.stop_time,
            },
        }

    async def _op_heartbeat(self, session, message):
        return None

    async def _op_ping(self, session, message):
        return {"time": time.time()}

    async def _op_state(self, session, message):
        self._refresh()
        state = self._state_event()
        del state["event"]
        return state

    async def _op_execute(self, session, message):
        dt = _number(message, "dt")
        try:
            points = np.asarray(message.get("trajectory"), dtype=float)
        except (TypeError, ValueError):
            raise RequestError(ErrorCodes.JointSizeNotSeven,
                               "The trajectory must be a list of joint positions, each a list of 7 numbers") from None
        if points.ndim != 2 or points.shape[1] != 7:
            raise RequestError(ErrorCodes.JointSizeNotSeven,
                               "The trajectory must be a list of joint positions, each a list of 7 numbers")
        self._require_gripper_idle(session)
        execution = await self._run_motion(session, message, self.controller.execute(points, dt),
                                           len(points), dt * (len(points) - 1))
        return {"points": execution.points, "duration": execution.duration}

    async def _op_move_home(self, session, message):
        """libfranka's examples' move to their home pose, with their MotionGenerator."""
        speed_factor = _number(message, "speed_factor")
        self._require_gripper_idle(session)
        submission = self.controller.move_to_joint_positions(protocol.HOME_JOINT_POSITIONS, speed_factor)
        await self._run_motion(session, message, submission, 2, 0.0, what="move home")
        return {}

    async def _op_stop(self, session, message):
        self._refresh()
        trajectory = self._snapshot["trajectory"]
        # Past its last point a trajectory only waits for the robot to come to
        # rest: there is nothing left to stop, and it ends as a success.
        settling = bool(trajectory and trajectory["settling"])
        executions = [] if settling else list(self._executions.values())
        for execution in executions:
            if not execution.stopped_by:
                execution.stopped_by = "by stop()" if execution.session is session else f"by stop() from {session.label}"
            log.info("session %d: stop requested by %s", execution.session.id, session.label)
        self.controller.stop()
        return {"was_executing": bool(executions)}

    async def _op_recover(self, session, message):
        code, text, controller_id = self.controller.recover()
        code = ErrorCodes(code)
        if code != ErrorCodes.Success:
            raise RequestError(code, self._busy_message(session) if code == ErrorCodes.RobotBusy else text)
        future = asyncio.get_running_loop().create_future()
        self._recoveries[controller_id] = future
        log.info("session %d: error recovery requested by %s", session.id, session.label)
        code, text = await future
        if code != ErrorCodes.Success:
            raise RequestError(code, text)
        return {}

    async def _op_gripper_homing(self, session, message):
        return await self._gripper_command(session, "homing")

    async def _op_gripper_move(self, session, message):
        width = _number(message, "width", minimum=0.0)
        speed = _number(message, "speed", minimum=0.0)
        return await self._gripper_command(session, "move", width, speed)

    async def _op_gripper_grasp(self, session, message):
        width = _number(message, "width", minimum=0.0)
        speed = _number(message, "speed", minimum=0.0)
        force = _number(message, "force", minimum=0.0)
        epsilon_inner = _number(message, "epsilon_inner", 0.005, minimum=0.0)
        epsilon_outer = _number(message, "epsilon_outer", 0.005, minimum=0.0)
        return await self._gripper_command(session, "grasp", width, speed, force, epsilon_inner, epsilon_outer)

    async def _op_gripper_stop(self, session, message):
        self._require_gripper()
        code, success, text = await asyncio.to_thread(self.gripper.stop)
        if ErrorCodes(code) != ErrorCodes.Success:
            raise RequestError(code, text)
        return {"success": bool(success)}

    # ---- helpers ------------------------------------------------------------

    def _require_gripper_idle(self, session):
        if self._gripper_owner is not None and self._gripper_owner is not session:
            raise RequestError(ErrorCodes.RobotBusy,
                               f"The gripper is moving for {self._gripper_owner.label}; wait until it finishes")

    async def _run_motion(self, session, message, submission, points, duration, what="trajectory"):
        """Wait for a motion the controller accepted (submission: code, text, id) to end."""
        code, text, controller_id = submission
        code = ErrorCodes(code)
        if code != ErrorCodes.Success:
            if code == ErrorCodes.RobotBusy:
                text = self._busy_message(session)
            log.info("session %d: %s refused: %s: %s", session.id, what, code.name, text)
            raise RequestError(code, text)
        execution = Execution(session, message.get("id"), points, duration,
                              asyncio.get_running_loop().create_future(), asyncio.current_task())
        self._executions[controller_id] = execution
        if what == "trajectory":
            log.info("session %d: executing trajectory %d for %s (%d points, %.2f s)",
                     session.id, controller_id, session.label, execution.points, execution.duration)
        else:
            log.info("session %d: executing %s %d for %s", session.id, what, controller_id, session.label)
        code, text = await execution.future
        if code == ErrorCodes.Interrupted and execution.stopped_by:
            text = f"The {what} was stopped {execution.stopped_by}"
        log.info("session %d: %s %d: %s%s", session.id, what, controller_id, code.name, f": {text}" if text else "")
        if code != ErrorCodes.Success:
            raise RequestError(code, text)
        return execution

    def _require_gripper(self):
        if self.gripper is None:
            raise RequestError(ErrorCodes.GripperNotConnected, "This robot server runs without a gripper")

    async def _gripper_command(self, session, name, *args):
        self._require_gripper()
        if self._gripper_owner is not None:
            raise RequestError(ErrorCodes.RobotBusy, f"The gripper is busy with a command from {self._gripper_owner.label}")
        owner = self._arm_owner()
        if owner is not None and owner is not session:
            raise RequestError(ErrorCodes.RobotBusy, f"The robot is executing a trajectory for {owner.label}")
        self._gripper_owner = session
        log.info("session %d: gripper %s%s", session.id, name, args)
        try:
            code, success, text = await asyncio.to_thread(getattr(self.gripper, name), *args)
        finally:
            if self._gripper_owner is session:
                self._gripper_owner = None
        if ErrorCodes(code) != ErrorCodes.Success:
            raise RequestError(code, text)
        return {"success": bool(success)}

    def _arm_owner(self):
        # The controller runs one trajectory at a time, so there is at most one.
        return next(iter(self._executions.values())).session if self._executions else None

    def _busy_message(self, session):
        owner = self._arm_owner()
        if owner is session:
            return "The robot is already executing a trajectory for this client"
        if owner is not None:
            return (f"The robot is executing a trajectory for {owner.label}. Wait until it finishes, then plan "
                    "again from the robot's current position.")
        if self._snapshot["status"] == "recovering":
            return "The robot is recovering from an error; try again in a moment"
        return "The robot is busy; try again in a moment"

    # ---- state --------------------------------------------------------------

    def _refresh(self):
        self._snapshot = self.controller.snapshot()
        if self.gripper is not None:
            self._gripper_snapshot = self.gripper.snapshot()

    async def _monitor(self):
        while True:
            try:
                now = time.monotonic()
                self._refresh()
                self._log_status()
                self._resolve()
                self._watch_owners(now)
                self._broadcast(now)
                self._update_connection_wanted(now)
            except Exception:
                log.exception("monitor failed")
            await asyncio.sleep(MONITOR_PERIOD)

    def _log_status(self):
        connected, error = self._snapshot["connected"], self._snapshot["connection_error"]
        if connected and not self._last_connected:
            log.info("connected to the robot")
        elif self._last_connected and not connected:
            log.info("disconnected from the robot")
        if error and error != self._last_connection_error:
            log.warning("robot: %s", error)
        self._last_connected, self._last_connection_error = connected, error

    def _resolve(self):
        for controller_id, code, text in self._snapshot["results"]:
            execution = self._executions.pop(controller_id, None)
            future = execution.future if execution is not None else self._recoveries.pop(controller_id, None)
            if future is not None and not future.done():
                future.set_result((ErrorCodes(code), text))

    def _watch_owners(self, now):
        trajectory = self._snapshot["trajectory"]
        execution = self._executions.get(trajectory["id"]) if trajectory else None
        if execution is not None and not execution.stopped_by \
                and now - execution.session.last_seen > self.owner_timeout:
            execution.stopped_by = f"because {execution.session.label} stopped responding"
            log.warning("session %d: no heartbeat for %.1f s during its trajectory, stopping the robot",
                        execution.session.id, now - execution.session.last_seen)
            self.controller.stop()

    def _state_event(self):
        s = self._snapshot
        robot = {"connected": s["connected"], "status": s["status"], "error": s["connection_error"]}
        if s["connected"] and s["state"] is not None:
            robot.update(s["state"])
            robot.update(age=s["age"], seq=s["seq"], current_errors=s["current_errors"],
                         last_motion_errors=s["last_motion_errors"])
        trajectory = None
        if s["trajectory"] is not None:
            t = s["trajectory"]
            execution = self._executions.get(t["id"])
            trajectory = {key: t[key] for key in ("index", "length", "time", "duration", "stopping", "settling",
                                                  "q_d")}
            trajectory.update(
                session=execution.session.id if execution else None,
                request=execution.request if execution else None,
                owner=execution.session.label if execution else None,
            )
        gripper = {"available": self.gripper is not None, "connected": False, "error": None, "busy": False,
                   "session": self._gripper_owner.id if self._gripper_owner else None}
        g = self._gripper_snapshot
        if g is not None:
            gripper.update(connected=g["connected"], error=g["connection_error"], busy=g["busy"])
            if g["connected"] and g["state"] is not None:
                gripper.update(g["state"])
        return {"event": "state", "time": time.time(), "robot": robot, "trajectory": trajectory, "gripper": gripper}

    def _broadcast(self, now):
        data = None
        for session in list(self._sessions.values()):
            if not session.hello or session.state_period <= 0 or now < session.next_state:
                continue
            session.next_state = max(session.next_state + session.state_period, now)
            transport = session.writer.transport
            if transport.is_closing() or transport.get_write_buffer_size() > MAX_BUFFERED_STATE:
                continue
            if data is None:
                data = protocol.encode(self._state_event())
            session.writer.write(data)

    def _update_connection_wanted(self, now):
        if any(session.hello for session in self._sessions.values()):
            self._idle_since = None
            wanted = True
        else:
            if self._idle_since is None:
                self._idle_since = now
            wanted = now - self._idle_since < self.idle_release
        if wanted != self._wanted:
            self._wanted = wanted
            if wanted:
                log.info("connecting to the robot at %s", self.controller.address)
            else:
                log.info("no clients for %.0f s, releasing the robot", self.idle_release)
            self.controller.set_connection_wanted(wanted)
            if self.gripper is not None:
                self.gripper.set_connection_wanted(wanted)
