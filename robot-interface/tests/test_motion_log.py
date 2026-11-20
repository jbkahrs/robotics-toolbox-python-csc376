"""The motion log: one CSV file per motion, written beside the control loop."""

import json
import os
import re
import time

import numpy as np
import pytest

from csc376_franky import ErrorCodes, RobotError, protocol
from csc376_franky_server.server import LIMIT_NAMES, motion_log_preamble
from helpers import SIM_SPEED, linear_trajectory, make_station, wait_until

NAME = re.compile(r"motion-\d{8}-\d{6}-(\d{6})-(trajectory|joint_motion|servo)\.csv")


def read_log(path):
    """A motion log as (header, columns, result): the '# key: {...}' lines, the
    samples by column name, and the result line."""
    header, result, rows, names = {}, None, [], None
    with open(path) as file:
        for line in file:
            line = line.rstrip("\n")
            if line.startswith("# "):
                key, separator, rest = line[2:].partition(": ")
                if key == "result":
                    result = json.loads(rest)
                elif separator and rest[:1] in "{[":
                    header[key] = json.loads(rest)
                else:
                    header.setdefault("comments", []).append(line[2:])
            elif names is None:
                names = line.split(",")
            else:
                rows.append([float(x) for x in line.split(",")])
    data = np.array(rows, dtype=float).reshape(-1, len(names))
    return header, dict(zip(names, data.T)), result


def logs_in(directory):
    return sorted(name for name in os.listdir(directory) if NAME.fullmatch(name))


def make_log(native, directory, max_bytes=10**9):
    limits = native.Limits()
    preamble = motion_log_preamble(
        limits, backend="sim", robot="simulation", libfranka=None
    )
    return native.MotionLog(str(directory), max_bytes, preamble), limits


@pytest.fixture
def logged(native, tmp_path):
    """A controller with a simulated robot that logs every motion into tmp_path."""
    log, limits = make_log(native, tmp_path)
    controller = native.Controller.sim(limits, speed=SIM_SPEED, log=log)
    controller.set_connection_wanted(True)
    assert wait_until(lambda: controller.snapshot()["state"] is not None)
    yield controller, log
    controller.shutdown()
    log.shutdown()


def q_now(controller):
    return np.array(controller.snapshot()["state"]["q"])


def result_of(controller, controller_id, timeout=10.0):
    found = []

    def finished():
        found[:] = [
            (ErrorCodes(code), message, log_file)
            for i, code, message, log_file in controller.snapshot()["results"]
            if i == controller_id
        ]
        return bool(found)

    assert wait_until(finished, timeout), "the command never finished"
    return found[0]


def run_trajectory(controller, offset, duration):
    trajectory = linear_trajectory(q_now(controller), offset, duration)
    code, message, controller_id = controller.execute(trajectory, 0.01)
    assert ErrorCodes(code) == ErrorCodes.Success, message
    return trajectory, controller_id


def written(log, count):
    """True once the log has finished count files."""
    return wait_until(lambda: log.files == count)


def test_a_trajectory_is_logged_at_the_control_rate(logged, tmp_path):
    controller, log = logged
    trajectory, controller_id = run_trajectory(
        controller, [0.2, -0.1, 0, 0.1, 0, 0, 0], duration=1.0
    )
    code, _, log_file = result_of(controller, controller_id)
    assert code == ErrorCodes.Success
    assert written(log, 1) and log.dropped == 0 and log.error is None
    assert logs_in(tmp_path) == [os.path.basename(log_file)]
    assert NAME.fullmatch(os.path.basename(log_file)).groups() == (
        f"{controller_id:06d}",
        "trajectory",
    )

    header, columns, result = read_log(log_file)
    assert header["server"]["backend"] == "sim" and "version" in header["server"]
    assert set(header["limits"]) == set(LIMIT_NAMES)
    assert header["limits"]["stiffness"] == [600, 900, 600, 300, 300, 200, 100]
    assert header["motion"]["kind"] == "trajectory"
    assert header["motion"]["id"] == controller_id
    assert header["motion"]["points"] == len(trajectory)
    assert header["motion"]["dt"] == 0.01 and header["motion"]["duration"] == 1.0
    assert header["motion"]["started"][:2] == "20"
    assert result["code"] == 1 and result["name"] == "Success"
    assert result["dropped"] == 0

    samples = len(columns["time"])
    assert result["samples"] == samples
    # One sample per millisecond, from the start to the end of settling.
    assert 1000 < samples < 2100
    assert columns["time"][0] == 0.0
    np.testing.assert_allclose(np.diff(columns["time"]), 0.001, atol=1e-9)
    assert columns["period"][0] == 0.0 and (columns["period"][1:] == 0.001).all()
    assert (columns["mode"] == 2).all()  # franka::RobotMode::kMove
    assert len(columns) == 6 + 7 * 7 + 3 + 6 + 1
    q = np.column_stack([columns[f"q{j}"] for j in range(7)])
    q_d = np.column_stack([columns[f"q_d{j}"] for j in range(7)])
    np.testing.assert_allclose(q_d[0], trajectory[0], atol=1e-6)
    np.testing.assert_allclose(q_d[-1], trajectory[-1], atol=1e-6)
    assert np.abs(q_d - q).max() < 0.05  # within the tracking error limit
    assert columns["index"][0] == 0 and columns["index"][-1] == len(trajectory) - 1
    assert set(columns["phase"]) == {0, 2}  # running, then settling
    assert (columns["limited"] == 0).all() and (columns["vx"] == 0).all()
    # The end effector's position is the forward kinematics of q.
    pose = np.array(controller.snapshot()["state"]["O_T_EE"]).reshape(4, 4).T
    np.testing.assert_allclose(
        [columns["x"][-1], columns["y"][-1], columns["z"][-1]], pose[:3, 3], atol=1e-4
    )


def test_move_home_and_velocity_control_are_logged_too(logged, tmp_path):
    controller, log = logged
    code, message, home_id = controller.move_to_joint_positions(
        protocol.HOME_JOINT_POSITIONS, 0.3
    )
    assert ErrorCodes(code) == ErrorCodes.Success, message
    assert result_of(controller, home_id)[0] == ErrorCodes.Success

    code, message, servo_id = controller.start_servo()
    assert ErrorCodes(code) == ErrorCodes.Success, message
    assert wait_until(lambda: controller.snapshot()["status"] == "executing")
    for _ in range(30):
        controller.set_servo_twist(servo_id, [0.0, 0.0, 0.05, 0.0, 0.0, 0.0], "base")
        time.sleep(0.01)
    controller.end_servo(servo_id)
    assert result_of(controller, servo_id)[0] == ErrorCodes.Success
    assert written(log, 2)

    home_log, servo_log = (os.path.join(tmp_path, name) for name in logs_in(tmp_path))
    header, columns, result = read_log(home_log)
    assert header["motion"]["kind"] == "joint_motion"
    assert header["motion"]["speed_factor"] == 0.3
    np.testing.assert_allclose(
        header["motion"]["q_goal"], protocol.HOME_JOINT_POSITIONS
    )
    assert result["name"] == "Success" and result["samples"] == len(columns["time"])
    q_d = np.column_stack([columns[f"q_d{j}"] for j in range(7)])
    np.testing.assert_allclose(q_d[-1], protocol.HOME_JOINT_POSITIONS, atol=1e-6)

    header, columns, result = read_log(servo_log)
    assert header["motion"]["kind"] == "servo" and "points" not in header["motion"]
    assert result["name"] == "Success" and result["samples"] == len(columns["time"])
    # The twist followed, in the base frame, and the hand rising with it.
    assert columns["vz"].max() == pytest.approx(0.05, rel=0.05)
    assert columns["z"][-1] > columns["z"][0] + 0.01
    assert set(columns["phase"]) == {0, 1}  # running, then stopping


def test_an_interrupted_motion_logs_how_it_ended(logged, tmp_path):
    controller, log = logged
    _, controller_id = run_trajectory(controller, [0.5, 0, 0, 0, 0, 0, 0], duration=4.0)
    assert wait_until(
        lambda: (controller.snapshot()["trajectory"] or {}).get("index", 0) > 50
    )
    controller.stop()
    code, _, log_file = result_of(controller, controller_id)
    assert code == ErrorCodes.Interrupted
    assert written(log, 1)
    _, columns, result = read_log(log_file)
    assert result["code"] == int(ErrorCodes.Interrupted)
    assert result["name"] == "Interrupted" and "stopped" in result["message"]
    assert set(columns["phase"]) == {0, 1, 2}  # running, stopping, settling
    assert columns["index"][-1] < 300


def test_old_logs_are_removed_to_stay_under_the_size_limit(native, tmp_path):
    limit = 300_000
    log, limits = make_log(native, tmp_path, max_bytes=limit)
    controller = native.Controller.sim(limits, speed=SIM_SPEED, log=log)
    try:
        controller.set_connection_wanted(True)
        assert wait_until(lambda: controller.snapshot()["state"] is not None)
        names = []
        for i in range(4):
            offset = [0.1 if i % 2 == 0 else -0.1, 0, 0, 0, 0, 0, 0]
            _, controller_id = run_trajectory(controller, offset, duration=0.3)
            code, _, log_file = result_of(controller, controller_id)
            assert code == ErrorCodes.Success
            names.append(os.path.basename(log_file))
            assert written(log, i + 1)
        kept = logs_in(tmp_path)
        sizes = {name: os.path.getsize(os.path.join(tmp_path, name)) for name in kept}
        # Each file is over a third of the limit, so some were removed, the
        # oldest first, and the newest is always kept.
        assert all(size > limit / 3 for size in sizes.values())
        assert len(kept) < 4 and kept == names[-len(kept) :]
        assert len(kept) == 1 or sum(sizes.values()) <= limit
    finally:
        controller.shutdown()
        log.shutdown()


def test_a_log_directory_that_cannot_be_written_is_reported(native, tmp_path):
    (tmp_path / "file").write_text("not a directory")
    log, limits = make_log(native, tmp_path / "file" / "logs")
    assert "cannot create" in log.error
    controller = native.Controller.sim(limits, speed=SIM_SPEED, log=log)
    try:
        controller.set_connection_wanted(True)
        assert wait_until(lambda: controller.snapshot()["state"] is not None)
        _, controller_id = run_trajectory(controller, [0.1, 0, 0, 0, 0, 0, 0], 0.3)
        code, _, log_file = result_of(controller, controller_id)
        # The motion itself is not affected; the log says what it could not do.
        assert code == ErrorCodes.Success and log_file.endswith("-trajectory.csv")
        assert wait_until(lambda: "trajectory.csv" in (log.error or ""))
        assert "cannot create" in log.error and log.files == 0
    finally:
        controller.shutdown()
        log.shutdown()


def test_the_server_names_the_log_file(native, tmp_path, caplog):
    log, limits = make_log(native, tmp_path)
    station, close = make_station(native, limits, motion_log=log)
    try:
        with (
            caplog.at_level("INFO", logger="csc376_franky_server"),
            station.client() as robot,
        ):
            q0 = robot.get_current_joint_positions()
            trajectory = linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], 0.5)
            assert robot.run_joint_trajectory(trajectory, 0.01) == ErrorCodes.Success
    finally:
        close()
    (name,) = logs_in(tmp_path)
    assert f"(log: {name})" in caplog.text
    # Closing the server finished the log.
    _, _, result = read_log(tmp_path / name)
    assert result["name"] == "Success"


def test_a_motion_cut_short_by_the_server_shutting_down_is_logged(native, tmp_path):
    log, limits = make_log(native, tmp_path)
    station, close = make_station(native, limits, motion_log=log)
    with station.client() as robot:
        q0 = robot.get_current_joint_positions()
        trajectory = linear_trajectory(q0, [0.5, 0, 0, 0, 0, 0, 0], 4.0)
        import concurrent.futures

        executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        running = executor.submit(robot.run_joint_trajectory, trajectory, 0.01)
        assert wait_until(lambda: robot.is_executing)
        close()
        with pytest.raises(RobotError):
            running.result(10)
        executor.shutdown(wait=False)
    (name,) = logs_in(tmp_path)
    _, columns, result = read_log(tmp_path / name)
    assert result["name"] == "Interrupted" and len(columns["time"]) > 100


def test_the_server_warns_when_the_log_cannot_write(native, tmp_path, caplog):
    (tmp_path / "file").write_text("not a directory")
    log, limits = make_log(native, tmp_path / "file" / "logs")
    station, close = make_station(native, limits, motion_log=log)
    try:
        with (
            caplog.at_level("WARNING", logger="csc376_franky_server"),
            station.client() as robot,
        ):
            q0 = robot.get_current_joint_positions()
            trajectory = linear_trajectory(q0, [0.1, 0, 0, 0, 0, 0, 0], 0.3)
            assert robot.run_joint_trajectory(trajectory, 0.01) == ErrorCodes.Success
            assert wait_until(lambda: "motion log: cannot create" in caplog.text)
    finally:
        close()


def test_command_line_gains_and_log_options(native, tmp_path, monkeypatch):
    from csc376_franky_server.__main__ import build_parser, make_limits, make_motion_log

    monkeypatch.delenv("LOGS_DIRECTORY", raising=False)
    args = build_parser().parse_args(["--sim"])
    assert (
        args.log_dir is None and make_motion_log(native, args, native.Limits()) is None
    )
    limits = make_limits(native, args)
    assert list(limits.stiffness) == [600, 900, 600, 300, 300, 200, 100]
    assert list(limits.joint_impedance) == [3000, 3000, 3000, 2500, 2500, 2000, 2000]
    assert limits.tracking_error_limit == 0.05

    args = build_parser().parse_args(["--sim", "--tracking-error-limit", "0.1"])
    assert make_limits(native, args).tracking_error_limit == 0.1
    for bad in ("0", "-0.1", "inf", "nan"):
        with pytest.raises(SystemExit):
            make_limits(
                native,
                build_parser().parse_args(["--sim", "--tracking-error-limit", bad]),
            )

    args = build_parser().parse_args(
        [
            "--sim",
            "--stiffness",
            "1,2,3,4,5,6,7",
            "--damping",
            "7,6,5,4,3,2,1",
            "--joint-impedance",
            "1000,1000,1000,1000,1000,1000,1000",
            "--log-dir",
            str(tmp_path / "logs"),
            "--log-max-size",
            "1",
        ]
    )
    limits = make_limits(native, args)
    assert list(limits.stiffness) == [1, 2, 3, 4, 5, 6, 7]
    assert list(limits.damping) == [7, 6, 5, 4, 3, 2, 1]
    assert list(limits.joint_impedance) == [1000] * 7
    log = make_motion_log(native, args, limits)
    assert log.directory == str(tmp_path / "logs") and os.path.isdir(log.directory)
    assert log.error is None
    log.shutdown()

    # The service sets $LOGS_DIRECTORY; --no-log overrides it.
    monkeypatch.setenv("LOGS_DIRECTORY", str(tmp_path / "service"))
    args = build_parser().parse_args(["--sim"])
    assert args.log_dir == str(tmp_path / "service")
    args = build_parser().parse_args(["--sim", "--no-log"])
    assert make_motion_log(native, args, native.Limits()) is None

    for bad in (
        "1,2,3",
        "1,2,3,4,5,6,-7",
        "0,2,3,4,5,6,7",
        "1,2,3,4,5,6,10001",
        "a,b,c,d,e,f,g",
        "1,2,3,4,5,6,nan",
        "1,2,3,4,5,6,inf",
    ):
        with pytest.raises(SystemExit):
            build_parser().parse_args(["--sim", "--stiffness", bad])
    with pytest.raises(SystemExit):
        build_parser().parse_args(["--sim", "--joint-impedance", "14251,1,1,1,1,1,1"])
    for size in ("0", "inf", "nan"):
        args = build_parser().parse_args(
            ["--sim", "--log-dir", str(tmp_path), "--log-max-size", size]
        )
        with pytest.raises(SystemExit):
            make_motion_log(native, args, native.Limits())
