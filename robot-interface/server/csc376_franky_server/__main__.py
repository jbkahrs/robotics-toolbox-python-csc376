"""Run the CSC376 robot server: python -m csc376_franky_server [options]

On a station's miniPC this normally runs as the csc376-robot-server systemd
service. With --sim it drives a simulated robot instead, on any
machine, which is how to try the clients without a robot.

With --log-dir (which the service sets) every motion is recorded at the
control rate to a CSV file of its own, for tuning the controller's gains
(--stiffness, --damping, --joint-impedance) to each robot.
"""

import argparse
import asyncio
import logging
import math
import os
import resource
import signal
import sys

from csc376_franky import protocol

from . import __version__

log = logging.getLogger("csc376_franky_server")


# No gain may be larger than this: far above what any robot of the lab wants,
# and what libfranka accepts for the robot's own joint impedance.
MAX_GAIN = 10000.0
MAX_JOINT_IMPEDANCE = 14250.0


def gains(maximum):
    """A parser of seven numbers in (0, maximum] separated by commas, one per joint."""

    def parse(text):
        try:
            values = [float(value) for value in text.split(",")]
        except ValueError:
            values = []
        if len(values) != 7 or not all(0.0 < v <= maximum for v in values):
            raise argparse.ArgumentTypeError(
                f"expected 7 numbers above 0 and at most {maximum:g} separated by commas, one per joint"
            )
        return values

    return parse


def build_parser():
    p = argparse.ArgumentParser(
        prog="csc376-robot-server", description=__doc__.splitlines()[0]
    )
    p.add_argument(
        "--robot-ip",
        default=protocol.ROBOT_IP,
        help=f"FCI address of the robot (default: {protocol.ROBOT_IP})",
    )
    p.add_argument(
        "--bind",
        default="0.0.0.0",
        help="address to listen on (default: all; the miniPC firewall only admits the station network)",
    )
    p.add_argument(
        "--port",
        type=int,
        default=protocol.DEFAULT_PORT,
        help=f"TCP port to listen on (default: {protocol.DEFAULT_PORT})",
    )
    p.add_argument(
        "--sim",
        action="store_true",
        help="drive a simulated robot and gripper instead of the real ones",
    )
    p.add_argument(
        "--sim-speed",
        type=float,
        default=1.0,
        help="simulation speed relative to real time (default: 1)",
    )
    p.add_argument(
        "--no-gripper", action="store_true", help="do not connect to the Franka Hand"
    )
    p.add_argument(
        "--start-tolerance",
        type=float,
        default=None,
        help="how far [rad] a trajectory may start from the measured joint positions (default: 0.02)",
    )
    p.add_argument(
        "--tracking-error-limit",
        type=float,
        default=None,
        help="stop a motion when a joint is this far [rad] from where it is commanded to be (default: 0.05)",
    )
    p.add_argument(
        "--velocity-scale",
        type=float,
        default=None,
        help="fraction of the Panda's joint velocity limits a trajectory may use (default: 0.3)",
    )
    p.add_argument(
        "--acceleration-scale",
        type=float,
        default=None,
        help="fraction of the Panda's joint acceleration limits the velocity may change by between "
        "trajectory points (default: 1.0)",
    )
    p.add_argument(
        "--servo-linear-velocity",
        type=float,
        default=None,
        help="fastest the end effector may be asked to move under velocity control [m/s] (default: 0.1)",
    )
    p.add_argument(
        "--servo-angular-velocity",
        type=float,
        default=None,
        help="fastest the end effector may be asked to turn under velocity control [rad/s] (default: 0.5)",
    )
    p.add_argument(
        "--servo-timeout",
        type=float,
        default=None,
        help="under velocity control, stop the robot when no new twist has come for this long [s] (default: 0.25)",
    )
    p.add_argument(
        "--servo-min-height",
        type=float,
        default=None,
        help="height above the robot's base below which velocity control does not drive the end effector [m] "
        "(default: 0.02)",
    )
    p.add_argument(
        "--stiffness",
        type=gains(MAX_GAIN),
        default=None,
        metavar="K0,...,K6",
        help="joint stiffness of the trajectory controller [Nm/rad], 7 numbers "
        "(default: 600,900,600,300,300,200,100)",
    )
    p.add_argument(
        "--damping",
        type=gains(MAX_GAIN),
        default=None,
        metavar="D0,...,D6",
        help="joint damping of the trajectory controller [Nms/rad], 7 numbers "
        "(default: 50,70,70,60,30,15,10)",
    )
    p.add_argument(
        "--joint-impedance",
        type=gains(MAX_JOINT_IMPEDANCE),
        default=None,
        metavar="K0,...,K6",
        help="joint impedance of the robot's own controller [Nm/rad], which follows move_home and velocity "
        "control, 7 numbers up to 14250 (default: 3000,3000,3000,2500,2500,2000,2000)",
    )
    p.add_argument(
        "--log-dir",
        default=os.environ.get("LOGS_DIRECTORY") or None,
        metavar="DIR",
        help="record every motion at the control rate to a CSV file in DIR, for tuning the controller "
        "(default: $LOGS_DIRECTORY, which the csc376-robot-server service sets to "
        "/var/log/csc376-robot-server; off otherwise)",
    )
    p.add_argument(
        "--no-log",
        action="store_true",
        help="record no motions, even with $LOGS_DIRECTORY set",
    )
    p.add_argument(
        "--log-max-size",
        type=float,
        default=2048.0,
        metavar="MIB",
        help="remove the oldest motion logs once they take more than this together [MiB] (default: 2048)",
    )
    p.add_argument(
        "--state-rate",
        type=float,
        default=50.0,
        help="robot state messages per second (default: 50)",
    )
    p.add_argument(
        "--idle-release",
        type=float,
        default=30.0,
        help="seconds after the last client leaves before the robot is released (default: 30)",
    )
    p.add_argument(
        "--owner-timeout",
        type=float,
        default=2.0,
        help="stop a trajectory when its client has been silent this long [s] (default: 2)",
    )
    p.add_argument(
        "--no-realtime",
        action="store_true",
        help="let libfranka run without real-time scheduling (testing only; motions may be rough)",
    )
    p.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    p.add_argument("--version", action="version", version=f"%(prog)s {__version__}")
    return p


def check_realtime():
    """Warn about the two things libfranka refuses to run without."""
    try:
        with open("/sys/kernel/realtime") as f:
            realtime_kernel = f.read().strip() == "1"
    except OSError:
        realtime_kernel = False
    if not realtime_kernel:
        log.warning(
            "this kernel is not PREEMPT_RT; libfranka will refuse to connect (boot the RT kernel)"
        )
    soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
    if soft != resource.RLIM_INFINITY and soft < 99:
        log.warning(
            "real-time priority is limited to %d; libfranka needs 99. Run the csc376-robot-server "
            "service (LimitRTPRIO=99) or log in again as a member of group 'realtime'",
            soft,
        )


def make_limits(control, args):
    limits = control.Limits()
    if args.start_tolerance is not None:
        if not math.isfinite(args.start_tolerance):
            sys.exit("error: --start-tolerance must be a number")
        limits.start_tolerance = args.start_tolerance
    if args.tracking_error_limit is not None:
        if not 0.0 < args.tracking_error_limit < math.inf:
            sys.exit("error: --tracking-error-limit must be positive")
        limits.tracking_error_limit = args.tracking_error_limit
    if args.velocity_scale is not None:
        if not 0.0 < args.velocity_scale <= 1.0:
            sys.exit("error: --velocity-scale must be in (0, 1]")
        limits.velocity_scale = args.velocity_scale
    if args.acceleration_scale is not None:
        if not args.acceleration_scale > 0.0:
            sys.exit("error: --acceleration-scale must be positive")
        limits.acceleration_scale = args.acceleration_scale
    for name in ("servo_linear_velocity", "servo_angular_velocity", "servo_timeout"):
        value = getattr(args, name)
        if value is not None:
            if not 0.0 < value < math.inf:
                sys.exit(f"error: --{name.replace('_', '-')} must be positive")
            setattr(limits, name, value)
    if args.servo_min_height is not None:
        if not math.isfinite(args.servo_min_height):
            sys.exit("error: --servo-min-height must be a number")
        limits.servo_min_height = args.servo_min_height
    for name in ("stiffness", "damping", "joint_impedance"):
        value = getattr(args, name)
        if value is not None:
            setattr(limits, name, value)
    return limits


def make_motion_log(control, args, limits):
    """The motion log of --log-dir, or None."""
    if args.no_log or not args.log_dir:
        return None
    if not (math.isfinite(args.log_max_size) and args.log_max_size > 0.0):
        sys.exit("error: --log-max-size must be a positive number")
    directory = os.path.abspath(args.log_dir)
    try:
        os.makedirs(directory, exist_ok=True)
    except OSError as error:
        sys.exit(
            f"error: cannot create the motion log directory {directory}: {error.strerror}"
        )
    if not os.access(directory, os.W_OK):
        sys.exit(f"error: the motion log directory {directory} is not writable")
    from .server import motion_log_preamble

    preamble = motion_log_preamble(
        limits,
        backend="sim" if args.sim else "franka",
        robot="simulation" if args.sim else args.robot_ip,
        libfranka=control.LIBFRANKA_VERSION,
    )
    return control.MotionLog(directory, int(args.log_max_size * 1024 * 1024), preamble)


async def serve(server, host, port):
    loop = asyncio.get_running_loop()
    stop = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    await server.start(host, port)
    log.info(
        "csc376_franky_server %s listening on %s:%d, %s robot at %s",
        __version__,
        host,
        server.port,
        server.controller.backend,
        server.controller.address,
    )
    if server.motion_log is not None:
        log.info("recording every motion in %s", server.motion_log.directory)
    await stop.wait()
    log.info("shutting down: stopping any motion and releasing the robot")
    await server.close()


def main(argv=None):
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        # journald adds its own timestamps
        format="%(levelname)-7s %(message)s"
        if os.environ.get("INVOCATION_ID")
        else "%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    if args.state_rate <= 0:
        sys.exit("error: --state-rate must be positive")
    try:
        from . import _control
    except ImportError as error:
        sys.exit(
            f"error: the compiled controller is missing ({error}). Reinstall the server; see server/README.md"
        )
    from .server import RobotServer

    limits = make_limits(_control, args)
    motion_log = make_motion_log(_control, args, limits)
    if args.sim:
        controller = _control.Controller.sim(
            limits, speed=args.sim_speed, log=motion_log
        )
        gripper = (
            None
            if args.no_gripper
            else _control.GripperController.sim(speed=args.sim_speed)
        )
    else:
        if not _control.HAS_LIBFRANKA:
            sys.exit(
                "error: this build has no libfranka, so it can only run with --sim. "
                "The miniPCs' build has it: the course staff install that one."
            )
        if not args.no_realtime:
            check_realtime()
        controller = _control.Controller.franka(
            args.robot_ip, limits, enforce_realtime=not args.no_realtime, log=motion_log
        )
        gripper = (
            None
            if args.no_gripper
            else _control.GripperController.franka(args.robot_ip)
        )

    server = RobotServer(
        controller,
        gripper,
        state_rate=args.state_rate,
        idle_release=args.idle_release,
        owner_timeout=args.owner_timeout,
        motion_log=motion_log,
    )
    try:
        asyncio.run(serve(server, args.bind, args.port))
    except OSError as error:
        controller.shutdown()
        if gripper is not None:
            gripper.shutdown()
        if motion_log is not None:
            motion_log.shutdown()
        sys.exit(f"error: cannot listen on {args.bind}:{args.port}: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
