# CSC376 robot interface

Python trajectory control of the teaching lab's Franka Emika Panda robots. You plan
and preview a trajectory on a station's GPU PC with the course's Robotics Toolbox,
and the station's miniPC runs it on the robot.

```
 GPU PC (two per station)                        miniPC 192.168.1.100               Panda 192.168.1.107
 ┌─────────────────────────────────┐   TCP 7376   ┌──────────────────────────────┐ FCI ┌──────────────┐
 │ your script or notebook         │ trajectories │ csc376-robot-server          │     │              │
 │  csc376_franky.motion_generator │ ───────────▶ │  checks every trajectory     │ ──▶ │ 1 kHz torque │
 │  csc376_franky.visualizer       │              │  1 kHz joint impedance       │     │ control      │
 │  FrankaJointTrajectoryController│ ◀─────────── │  one trajectory at a time    │ ◀── │              │
 │                                 │ state, 50 Hz │  PREEMPT_RT, libfranka 0.9.2 │     │ Franka Hand  │
 └─────────────────────────────────┘              └──────────────────────────────┘     └──────────────┘
```

* **GPU PC**: `csc376_franky`, pure Python. Plans with the toolbox from this repository
  and Ruckig, previews in Swift, and talks to the miniPC. No libfranka, no real-time
  kernel, no compiler: it runs on any lab PC.
* **miniPC**: `server/`, the robot server. It is the only program that talks to the
  robot. It checks every trajectory, runs the joint impedance controller of the
  old `csc376_franky` at 1 kHz, and sends the robot's state to the GPU PCs.

Both GPU PCs of a station can work at the same time. Each one sees the robot's
state, so each can plan and preview from where the robot actually is. The robot
runs one trajectory at a time.

The full documentation, with the API reference, is [`docs.html`](docs.html) in this
folder. GitHub shows it as source: open your copy of the file in a browser.

## Quick start

On a station's GPU PC, with FCI active in Desk and the user stop in reach:

```python
import roboticstoolbox as rtb
from spatialmath import SE3

from csc376_franky import FrankaJointTrajectoryController
from csc376_franky.motion_generator import RuckigMotionGenerator
from csc376_franky.visualizer import RtbVisualizer

robot = FrankaJointTrajectoryController()  # the robot server on this station's miniPC
panda = rtb.models.Panda()
motion_generator = RuckigMotionGenerator()

q_start = robot.get_current_joint_positions()
se3_start = panda.fkine(q_start)
se3_target = SE3.Ty(0.1) * se3_start  # 10 cm along the base y axis

cartesian_traj, dt = motion_generator.calculate_cartesian_pose_trajectory(
    se3_start, se3_target
)
q_traj = motion_generator.cartesian_pose_to_joint_trajectory(
    panda, q_start, cartesian_traj
)

visualizer = RtbVisualizer(panda, q_start, real_robot=robot)  # see the plan first
visualizer.run_joint_trajectory(q_traj, dt)

robot.run_joint_trajectory(q_traj, dt)  # then run it on the robot
robot.move_gripper(0.0)  # close the gripper
```

`examples/` has the scripts from the practical, ported:

| Script | What it does |
|---|---|
| `read_robot_state.py` | Prints the joint positions, velocities and end-effector pose |
| `read_gripper_state.py` | Prints the gripper width |
| `visualize_then_execute_trajectory.py` | Moves 10 cm: preview in Swift, then on the robot after you type `yes` |
| `visualize_and_move_gripper.py` | Closes and opens the gripper, in Swift and then on the robot |
| `sim_or_real_trajectory.py` | A 10 cm square, in Swift (`simulation = True`) or on the robot |
| `sim_or_real_trajectory_with_wrapper.py` | The square in Swift, then on the robot, through `RobotWrapper` |
| `move_home.py` | Moves the robot to libfranka's home pose after you type `yes` (see below) |

`robot.move_home(speed_factor=0.2)` moves the robot the way libfranka's examples do,
without Ruckig or the toolbox. The miniPC runs libfranka's example `MotionGenerator`
through the robot's own joint position controller, going straight in joint space from
wherever the robot is to `protocol.HOME_JOINT_POSITIONS` (0, −π/4, 0, −3π/4, 0, π/2, π/4).
Nothing is planned or previewed on the GPU PC, so check that the way is clear first.
`speed_factor` can be at most the miniPC's velocity scale (0.3). `stop()`, Ctrl+C and
the other checks work as they do for a trajectory.

## Setup

**Lab GPU PCs**: nothing to install. The shared `csc376` environment has the
toolbox, Ruckig and `csc376_franky`, and the examples, to copy into your home:

```bash
conda activate csc376
cp -r "$CONDA_PREFIX/share/csc376_franky/examples" ~/csc376_franky_examples
cd ~/csc376_franky_examples && python visualize_then_execute_trajectory.py
```

**Dev container**: nothing to install either; it installs `csc376_franky` from this
folder in editable mode.

**Anywhere else**: install the toolbox from this repository as its README says, then
`pip install -e ".[planning]"` from this folder.

The robot server's address defaults to 192.168.1.100:7376, the miniPC on every
station's robot network. `FrankaJointTrajectoryController(host, port)`, or the
environment variables `CSC376_ROBOT_HOST` and `CSC376_ROBOT_PORT`, change it.

## What the miniPC checks

The miniPC never trusts a trajectory. It refuses one, with a message saying what
to change, unless all of these hold (the defaults are set on the miniPC):

| Check | Default | `ErrorCodes` when it fails |
|---|---|---|
| No trajectory is running, and no other client's gripper command | | `RobotBusy` |
| The robot can move: not user-stopped, guided, or stopped by a reflex | | `RobotNotReady` |
| The first point is where the robot is, checked again on the first 1 kHz cycle | within 0.02 rad per joint | `JointTrajectoryDoesNotStartAtCurrentPosition` |
| Every point is inside the joint limits | 0.01 rad margin | `JointLimitsExceeded` |
| No joint jumps between consecutive points (an IK solution flipping) | 0.025 rad | `JointDistancesAboveMaxJointDifference` |
| No joint moves faster than a fraction of its limit | 0.3 × (2.175 / 2.61 rad/s) | `JointVelocitiesAboveMaximumAllowed` |
| No joint's velocity changes abruptly from one segment to the next | 1.0 × (15 / 7.5 / 10 / 12.5 / 15 / 20 / 20 rad/s²) | `JointAccelerationsAboveMaximumAllowed` |
| Sensible timing | 1 ms ≤ dt ≤ 1 s, ≤ 600 s, ≤ 120 000 points | `DtIs...`, `TrajectoryTooLong` |
| While moving, every joint stays near its command | 0.05 rad | `TrackingErrorTooLarge` (the motion stops) |

`run_joint_trajectory` returns `ErrorCodes.Success` once the robot has finished and
settled, and raises `RobotError` otherwise; `error.code` is the `ErrorCodes` value.
Between the points of a trajectory the miniPC interpolates at 1 kHz. A small gap
between the first point and the robot (within the tolerance) is blended out over
0.25 s, so the robot never jumps at the start.

The velocity check compares consecutive points, and the acceleration check compares
consecutive segments: the change of a joint's step from one segment to the next,
divided by dt². A trajectory from Ruckig passes easily. One that reverses or turns a
corner at a single point may not: spread the change of velocity over several points.
The first and last segments are not compared with rest, so a constant-velocity
trajectory (`np.linspace` between two joint positions) is accepted as before.

**Stopping.** `robot.stop()` and Ctrl+C slow the robot to rest *along its path* within
0.5 s. Any client can stop any trajectory. `stop()` returns whether there was one to
stop; a stop that comes too late to end the trajectory short of its last point does
not interrupt it, and `run_joint_trajectory` returns `ErrorCodes.Success`. The miniPC also stops a trajectory when the
program that started it disconnects or has not been heard from for 2 s. The user stop
on the robot always works and does not depend on any of this.

## Two GPU PCs, one robot

* Both receive the robot's state; `get_current_joint_positions()` is at most 20 ms old.
* While one PC's trajectory runs, the other's `run_joint_trajectory` raises `RobotBusy`,
  naming the PC that has the robot. `robot.is_executing` and
  `robot.wait_until_idle()` let you wait for it.
* Afterwards, a plan made before the robot moved no longer starts where the robot is,
  so the miniPC refuses it. Plan again from `get_current_joint_positions()`.
* The gripper is shared the same way: one command at a time, and not while the other
  PC's trajectory runs.
* `RtbVisualizer(..., real_robot=robot)` adds a translucent blue Panda that follows the real
  robot, so each PC sees when the other one moves it.

## Changes from the old `csc376_franky`

| Before | Now |
|---|---|
| Everything ran on the miniPC | Planning and Swift on the GPU PC, control on the miniPC |
| `csc376_bind_franky.FrankaJointTrajectoryController("192.168.1.107")` | `csc376_franky.FrankaJointTrajectoryController()` (connects to the miniPC) |
| `csc376_bind_franky.Gripper(ip)` | `robot.gripper`, or `csc376_franky.Gripper()` |
| `run_joint_trajectory` returned an error code, also on failure | Returns `ErrorCodes.Success`, raises `RobotError` on failure |
| `stop()` ended the controller | `stop()` stops the running trajectory; `close()` disconnects |
| `setupSignalHandler()` for Ctrl+C | Not needed: Ctrl+C stops the robot along its path |
| Trajectory checks compared signed differences (backward jumps passed) | Absolute differences, plus joint limits |
| `calculate_cartesian_pose_trajectory` started one step after the start | Starts at the start pose (t = 0) |
| IK failures silently gave wrong joint positions | `RuntimeError` |
| numpy 1.26.4, websockets 13 and the swift-csc376 fork | The course toolbox fork (1.4.2) and Swift 2 |
| An exception in the 1 kHz loop killed the program | The miniPC reports it; `recover()` clears a reflex |
| Joint speed limit 0.3 × (3.0 ×3, 2.5 ×3, 2.62) rad/s | 0.3 × the datasheet's (2.175 ×4, 2.61 ×3) rad/s: 0.65 instead of 0.9 rad/s on joints 1 to 3, 0.65 instead of 0.75 rad/s on joint 4 |
| dt of at least 0.01 s | dt of at least 0.001 s |
| No acceleration check | The velocity may not change abruptly between segments (see above) |
| The planner's Cartesian limits were 3.0 m/s, 9 m/s², 4500 m/s³ | The datasheet's 1.7 m/s, 13 m/s², 6500 m/s³ (rotation 2.5 rad/s, 25 rad/s², 12 500 rad/s³), so the same `relative_vel_factor` plans a slower move |
| `csc376_franky.vizualizer`, `RobotWrapper.vizualizer` | `csc376_franky.visualizer`, `RobotWrapper.visualizer` (the old names still work) |

A trajectory that the old controller accepted can be refused now: the speed limit on
joints 1 to 4 is lower, and the acceleration check is new.

`get_current_robot_state()` keeps the old field names (`q`, `dq`, `O_T_EE`, ...),
and the old methods `setupSignalHandler()` and `join()` still exist (they do nothing).

## Trying it without a robot

The robot server can drive a simulated Panda and gripper, on any machine:

```bash
pip install . ./server                          # builds the controller (needs a C++17 compiler)
python -m csc376_franky_server --sim --bind 127.0.0.1
CSC376_ROBOT_HOST=127.0.0.1 python examples/sim_or_real_trajectory_with_wrapper.py
```

## Credits

The motion generator and the Swift visualizer are mostly Patrick Yao's
(`Patrick15Yao/franky_toolbox_bridge`, no longer online).
The joint trajectory controller is from [radiang/csc376](https://github.com/radiang/csc376),
used with its author's permission. The `MotionGenerator` behind `move_home` is a port
of the one in libfranka's examples (Apache License 2.0).

[`LICENSE`](LICENSE) and [`NOTICE`](NOTICE) have the licence and the details.
