"""Run the CSC376 robot server: python -m csc376_franky_server [options]

On a station's miniPC this normally runs as the csc376-robot-server systemd
service. With --sim it drives a simulated robot instead, on any
machine, which is how to try the clients without a robot.
"""

import argparse
import asyncio
import logging
import os
import resource
import signal
import sys

from csc376_franky import protocol

from . import __version__

log = logging.getLogger("csc376_franky_server")


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
        limits.start_tolerance = args.start_tolerance
    if args.velocity_scale is not None:
        if not 0.0 < args.velocity_scale <= 1.0:
            sys.exit("error: --velocity-scale must be in (0, 1]")
        limits.velocity_scale = args.velocity_scale
    if args.acceleration_scale is not None:
        if not args.acceleration_scale > 0.0:
            sys.exit("error: --acceleration-scale must be positive")
        limits.acceleration_scale = args.acceleration_scale
    return limits


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
    if args.sim:
        controller = _control.Controller.sim(limits, speed=args.sim_speed)
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
            args.robot_ip, limits, enforce_realtime=not args.no_realtime
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
    )
    try:
        asyncio.run(serve(server, args.bind, args.port))
    except OSError as error:
        controller.shutdown()
        if gripper is not None:
            gripper.shutdown()
        sys.exit(f"error: cannot listen on {args.bind}:{args.port}: {error}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
