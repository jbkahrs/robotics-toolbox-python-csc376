import asyncio
import threading
import time
from dataclasses import dataclass

import numpy as np
import pytest

# The simulated robot runs this many times faster than real time.
SIM_SPEED = 4.0


@pytest.fixture(scope="session")
def native():
    return pytest.importorskip("csc376_franky_server._control", reason="the robot server is not built")


@dataclass
class Station:
    """A robot server with a simulated robot, on localhost."""

    server: object
    controller: object
    gripper: object
    loop: asyncio.AbstractEventLoop
    port: int

    def client(self, **kwargs):
        from csc376_franky import FrankaJointTrajectoryController

        robot = FrankaJointTrajectoryController("127.0.0.1", self.port, **kwargs)
        assert wait_until(lambda: "width" in robot.get_status()["gripper"])
        return robot

    def call(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop).result(10)


def make_station(native, limits=None, **server_options):
    from csc376_franky_server.server import RobotServer

    controller = native.Controller.sim(limits or native.Limits(), speed=SIM_SPEED)
    gripper = native.GripperController.sim(speed=SIM_SPEED)
    options = dict(idle_release=0.5, owner_timeout=0.5)
    options.update(server_options)
    server = RobotServer(controller, gripper, **options)
    loop = asyncio.new_event_loop()
    started = threading.Event()

    def run():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(server.start("127.0.0.1", 0))
        started.set()
        loop.run_forever()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    assert started.wait(10)
    station = Station(server, controller, gripper, loop, server.port)

    def close():
        station.call(server.close())
        loop.call_soon_threadsafe(loop.stop)
        thread.join(10)

    return station, close


@pytest.fixture
def station(native):
    station, close = make_station(native)
    yield station
    close()


def wait_until(predicate, timeout=5.0, period=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period)
    return False


def linear_trajectory(q_start, offset, duration, dt=0.01):
    """A joint trajectory from q_start to q_start + offset, at rest at both ends."""
    q_start = np.asarray(q_start, dtype=float)
    s = np.linspace(0.0, 1.0, int(round(duration / dt)) + 1)
    s = s * s * (3.0 - 2.0 * s)
    return q_start + np.outer(s, offset)
