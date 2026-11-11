"""The Swift preview, headless (SWIFT_HEADLESS), alone and following the station's robot."""

import numpy as np
import pytest

pytest.importorskip("swift")
rtb = pytest.importorskip("roboticstoolbox")

from csc376_franky.robot_wrapper import RobotWrapper  # noqa: E402
from csc376_franky.vizualizer import RtbVisualizer  # noqa: E402

from conftest import linear_trajectory, wait_until  # noqa: E402


@pytest.fixture(autouse=True)
def headless(monkeypatch):
    monkeypatch.setenv("SWIFT_HEADLESS", "1")


@pytest.fixture(scope="module")
def panda():
    return rtb.models.Panda()


def test_preview_and_gripper(panda):
    visualizer = RtbVisualizer(panda, panda.qr)
    try:
        q_traj = [panda.qr + [0.01 * i, 0, 0, 0, 0, 0, 0] for i in range(5)]
        seen = []
        visualizer.set_trajectory_callback(seen.append)
        visualizer.run_joint_trajectory(q_traj, 0.001)
        assert seen == list(range(5))
        np.testing.assert_allclose(visualizer.q, q_traj[-1])
        assert visualizer.move_gripper(0.06, 1.0)
        assert list(visualizer.rtb_robot_model.grippers[0].q) == pytest.approx([0.03, 0.03])
    finally:
        visualizer.stop()


def test_transparent_panda_follows_the_real_robot(station, panda):
    with station.client() as robot:
        visualizer = RtbVisualizer(panda, robot.get_current_joint_positions(), real_robot=robot)
        try:
            moved = robot.get_current_joint_positions() + [0.1, 0, 0, 0, 0, 0, 0]
            station.controller.sim_set_joint_positions(moved)
            assert wait_until(lambda: np.allclose(visualizer._real_handle.q, moved, atol=1e-9))
        finally:
            visualizer.stop()


def test_robot_wrapper_previews_where_the_robot_is_then_runs(station, panda, monkeypatch):
    monkeypatch.setenv("CSC376_ROBOT_PORT", str(station.port))
    answers = iter(["", "yes"])
    monkeypatch.setattr("builtins.input", lambda prompt="": next(answers))
    wrapper = RobotWrapper(True, True, panda, panda.qr, "127.0.0.1")
    try:
        q0 = wrapper.get_current_joint_positions()
        np.testing.assert_allclose(wrapper.vizualizer.q, q0)
        trajectory = linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], 0.5)
        assert wrapper.run_joint_trajectory(trajectory, 0.01)
        np.testing.assert_allclose(wrapper.get_current_joint_positions(), trajectory[-1], atol=2e-3)
    finally:
        wrapper.stop()
