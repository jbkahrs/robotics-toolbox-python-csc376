.. _robot-interface:

**********************
CSC376 robot interface
**********************

``csc376_franky`` is the Python interface to the CSC376 teaching lab's Franka Emika
Panda robots. You plan and preview a trajectory on a station's GPU PC with the
Toolbox, and the station's miniPC runs it on the robot. It lives in the
``robot-interface/`` folder of the
`course fork <https://github.com/jbkahrs/robotics-toolbox-python-csc376>`_ and is
not part of the upstream Toolbox.

::

    GPU PC (two per station)
    ┌──────────────────────────────────────────────────┐
    │ your script or notebook                          │
    │   csc376_franky.motion_generator    plan         │
    │   csc376_franky.visualizer          preview      │
    │   FrankaJointTrajectoryController   run          │
    └──────────────────────────────────────────────────┘
          │ trajectories              ▲ state, 50 Hz
          ▼ TCP 7376                  │
    miniPC 192.168.1.100
    ┌──────────────────────────────────────────────────┐
    │ csc376-robot-server                              │
    │   checks every trajectory                        │
    │   1 kHz joint impedance control                  │
    │   one trajectory at a time                       │
    │   PREEMPT_RT, libfranka 0.9.2                    │
    └──────────────────────────────────────────────────┘
          │ FCI                       ▲
          ▼ 1 kHz torque control      │
    Panda 192.168.1.107, with its Franka Hand

- **GPU PC**: ``csc376_franky``, pure Python. It plans with the Toolbox and Ruckig,
  previews in Swift, and talks to the miniPC. It needs no libfranka, no real-time
  kernel and no compiler, so it runs on any lab PC.
- **miniPC**: the robot server (``robot-interface/server/``). It is the only program
  that talks to the robot. It checks every trajectory, runs a joint impedance
  controller at 1 kHz, and sends the robot's state to the GPU PCs.

Both GPU PCs of a station can work at the same time. Each one sees the robot's
state, so each can plan and preview from where the robot actually is. The robot
runs one trajectory at a time.


Quick start
===========

On a station's GPU PC, with FCI active in Desk and the user stop in reach:

.. code-block:: python

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

    visualizer.stop()  # ends the render thread, so that Python can exit


Setup
=====

**Lab GPU PCs**: nothing to install. The shared ``csc376`` environment has the
Toolbox, Ruckig and ``csc376_franky``, and the examples, to copy into your home::

    conda activate csc376
    cp -r "$CONDA_PREFIX/share/csc376_franky/examples" ~/csc376_franky_examples
    cd ~/csc376_franky_examples && python visualize_then_execute_trajectory.py

**Dev container**: nothing to install either; it installs ``csc376_franky`` from
``robot-interface/`` in editable mode.

**Anywhere else**: install the Toolbox from the course fork as its README says,
then, from ``robot-interface/``::

    pip install -e ".[planning]"

The robot server's address defaults to 192.168.1.100:7376, the miniPC on every
station's robot network. ``FrankaJointTrajectoryController(host, port)``, or the
environment variables ``CSC376_ROBOT_HOST`` and ``CSC376_ROBOT_PORT``, change it.

There is no robot at home, but the robot server can drive a simulated one: see
:ref:`robot-interface-sim`.


Examples
========

``robot-interface/examples/`` has the scripts from the practical:

.. list-table::
   :class: wrap-literals
   :header-rows: 1
   :widths: 46 54

   * - Script
     - What it does
   * - ``read_robot_state.py``
     - Prints the joint positions, velocities and end-effector pose
   * - ``read_gripper_state.py``
     - Prints the gripper width
   * - ``visualize_then_execute_trajectory.py``
     - Moves 10 cm: preview in Swift, then on the robot after you type ``yes``
   * - ``visualize_and_move_gripper.py``
     - Closes and opens the gripper, in Swift and then on the robot
   * - ``sim_or_real_trajectory.py``
     - A 10 cm square, in Swift (``simulation = True``) or on the robot
   * - ``sim_or_real_trajectory_with_wrapper.py``
     - The square in Swift, then on the robot, through
       :class:`~csc376_franky.robot_wrapper.RobotWrapper`
   * - ``move_home.py``
     - Moves the robot to libfranka's home pose after you type ``yes``
       (see :ref:`robot-interface-home`)


Planning and previewing
=======================

Planning a trajectory
---------------------

:class:`~csc376_franky.motion_generator.RuckigMotionGenerator` plans a straight
move of the end effector between two poses, in two steps:

1. :meth:`~csc376_franky.motion_generator.RuckigMotionGenerator.calculate_cartesian_pose_trajectory`
   interpolates from ``se3_start`` to ``se3_target`` with
   `Ruckig <https://github.com/pantor/ruckig>`_, which limits the velocity,
   acceleration and jerk. It returns the Cartesian trajectory and the time between
   its points, ``dt`` (0.01 s).
2. :meth:`~csc376_franky.motion_generator.RuckigMotionGenerator.cartesian_pose_to_joint_trajectory`
   solves the inverse kinematics of every point with the Toolbox, each one starting
   from the solution of the point before it. It returns the joint trajectory that
   the visualizer and the robot run.

The three factors of ``calculate_cartesian_pose_trajectory`` scale the Panda's
Cartesian limits from its datasheet, for translation and for rotation:

.. list-table::
   :class: wrap-literals
   :header-rows: 1
   :widths: 35 15 50

   * - Factor
     - Default
     - Limits
   * - ``relative_vel_factor``
     - 0.02
     - 1.7 m/s, 2.5 rad/s
   * - ``relative_acc_factor``
     - 0.01
     - 13 m/s², 25 rad/s²
   * - ``relative_jerk_factor``
     - 0.05
     - 6500 m/s³, 12 500 rad/s³

Three things to know when planning:

- Plan from where the robot is. The trajectory starts at ``se3_start`` itself, and
  the miniPC refuses a joint trajectory that does not start at the robot's measured
  joint positions, so take ``q_start`` from
  :meth:`~csc376_franky.FrankaJointTrajectoryController.get_current_joint_positions`.
- The orientation is interpolated as a rotation vector, component by component.
  That only follows a rotation about a fixed axis when the start and target
  orientations are close: plan a large reorientation as several short moves.
- ``cartesian_pose_to_joint_trajectory`` raises ``RuntimeError`` when the inverse
  kinematics finds no solution for a pose, for example a target out of reach.

A trajectory does not have to come from the motion generator. The robot runs any
N x 7 array of joint positions that passes the miniPC's checks
(see :ref:`robot-interface-checks`).

Previewing in Swift
-------------------

:class:`~csc376_franky.visualizer.RtbVisualizer` opens Swift in the browser and
shows a Panda at ``q_start``.
:meth:`~csc376_franky.visualizer.RtbVisualizer.run_joint_trajectory` plays a joint
trajectory on it with the timing it will have on the robot, and
:meth:`~csc376_franky.visualizer.RtbVisualizer.move_gripper` moves its fingers.

With ``real_robot=robot``, a translucent blue Panda follows the real robot live, so
the preview can be compared with where the robot really is.

Call :meth:`~csc376_franky.visualizer.RtbVisualizer.stop` when the program is done:
it ends the render thread and closes Swift.

Preview, then robot
-------------------

:class:`~csc376_franky.robot_wrapper.RobotWrapper` puts the two together. Its
``run_joint_trajectory`` plays the trajectory in Swift after you press enter, then
runs it on the robot after you type ``yes``:

.. code-block:: python

    from csc376_franky.robot_wrapper import RobotWrapper

    robot = RobotWrapper(
        has_visualizer=True, run_on_real=True, panda_rtb_model=panda, q_start=panda.qr
    )
    q_start = robot.get_current_joint_positions()  # the real robot's, if there is one
    ...
    robot.run_joint_trajectory(q_traj, dt)
    robot.stop()

With ``run_on_real=False`` nothing connects to the robot server, and the same
program runs in Swift only. ``ip_address`` is the address of the station's miniPC
(the robot server), not of the robot; leave it out for the default.


Running on the robot
====================

:class:`~csc376_franky.FrankaJointTrajectoryController` is a network connection to
the robot server. Its constructor waits until the miniPC is connected to the robot,
so FCI must be active in Desk. Close it with
:meth:`~csc376_franky.FrankaJointTrajectoryController.close`, or use it as a context
manager:

.. code-block:: python

    with FrankaJointTrajectoryController() as robot:
        print(robot.get_current_joint_positions())

Reading the robot's state
-------------------------

The miniPC sends the robot's state 50 times a second, so what these return is at
most 20 ms old:

- :meth:`~csc376_franky.FrankaJointTrajectoryController.get_current_joint_positions`:
  the measured joint positions, an array of 7 [rad].
- :meth:`~csc376_franky.FrankaJointTrajectoryController.get_current_pose`: the
  end-effector pose (the Franka Hand's TCP) in the base frame, as an ``SE3``. It is
  the robot's own forward kinematics, and agrees with
  ``rtb.models.Panda().fkine(q)`` when Desk's end effector is the Franka Hand.
- :meth:`~csc376_franky.FrankaJointTrajectoryController.get_current_robot_state`: a
  :class:`~csc376_franky.RobotState`, with the field names of libfranka's
  ``franka::RobotState`` (``q``, ``dq``, ``tau_J``, ``O_T_EE``, ...).

Running a trajectory
--------------------

:meth:`~csc376_franky.FrankaJointTrajectoryController.run_joint_trajectory` sends an
N x 7 joint trajectory and the time between its points, and waits until the robot
has finished and settled. It returns ``ErrorCodes.Success``, and raises
:class:`~csc376_franky.RobotError` if the miniPC refused the trajectory or the
motion did not finish. ``error.code`` is the
:class:`~csc376_franky.ErrorCodes` value and ``error.message`` says what
to change:

.. code-block:: python

    from csc376_franky import ErrorCodes, RobotError

    try:
        robot.run_joint_trajectory(q_traj, dt)
    except RobotError as error:
        if error.code == ErrorCodes.RobotBusy:  # the other GPU PC has the robot
            robot.wait_until_idle()
            ...  # then plan again from get_current_joint_positions()
        else:
            raise

Between the points of a trajectory the miniPC interpolates at 1 kHz. A small gap
between the first point and the robot (within the start tolerance) is blended out
over 0.25 s, so the robot never jumps at the start.

:meth:`~csc376_franky.FrankaJointTrajectoryController.set_trajectory_callback`
registers a function that is called with the index of each point as the robot
passes it, for example to log or to act part of the way through.

Stopping
--------

:meth:`~csc376_franky.FrankaJointTrajectoryController.stop` and Ctrl+C slow the
robot to rest *along its path* within 0.5 s. Any client can stop any trajectory.
``stop()`` returns whether there was one to stop; a stop that comes too late to end
the trajectory short of its last point does not interrupt it, and
``run_joint_trajectory`` returns ``ErrorCodes.Success``.

The miniPC also stops a trajectory when the program that started it disconnects or
has not been heard from for 2 s.

.. warning:: The user stop on the robot always works and does not depend on any of
    this. Keep it in reach whenever the robot can move.

After a reflex (usually a collision) the robot refuses to move, and requests fail
with ``RobotNotReady``. Check the robot and its surroundings, then call
:meth:`~csc376_franky.FrankaJointTrajectoryController.recover`.

.. _robot-interface-home:

Moving home
-----------

:meth:`~csc376_franky.FrankaJointTrajectoryController.move_home` moves the robot the
way libfranka's examples do, without Ruckig or the Toolbox. The miniPC runs
libfranka's example ``MotionGenerator`` through the robot's own joint position
controller, going straight in joint space from wherever the robot is to
``protocol.HOME_JOINT_POSITIONS``, (0, −π/4, 0, −3π/4, 0, π/2, π/4).

.. warning:: Nothing is planned or previewed on the GPU PC, so check that the way
    is clear first.

``speed_factor`` (0.2 by default) can be at most the miniPC's velocity scale, 0.3.
``stop()``, Ctrl+C and the miniPC's other checks work as they do for a trajectory.

The gripper
-----------

``robot.gripper`` is the station's Franka Hand, a :class:`~csc376_franky.Gripper`
(``None`` if the robot server runs without one).
:meth:`~csc376_franky.FrankaJointTrajectoryController.move_gripper` is a shortcut
for its ``move``:

.. code-block:: python

    robot.move_gripper(0.08)  # open to 8 cm, at 0.1 m/s
    grasped = robot.gripper.grasp(width=0.03, speed=0.1, force=20.0)
    print(robot.gripper.width, robot.gripper.is_grasped)

``homing``, ``move``, ``grasp``, ``open`` and ``stop`` wait until the gripper has
finished and return whether it succeeded. Each has an ``_async`` variant that
returns a ``concurrent.futures.Future`` of the same result instead of waiting.
``Gripper()`` on its own opens a connection of its own, for a program that only
uses the gripper.


.. _robot-interface-checks:

What the miniPC checks
======================

The miniPC never trusts a trajectory. It refuses one, with a message saying what
to change, unless all of these hold. The defaults are set on the miniPC.

.. list-table::
   :class: wrap-literals
   :header-rows: 1
   :widths: 30 20 50

   * - Check
     - Default
     - ``ErrorCodes`` when it fails
   * - No trajectory is running, and no other client's gripper command
     -
     - ``RobotBusy``
   * - The robot can move: not user-stopped, guided, or stopped by a reflex
     -
     - ``RobotNotReady``
   * - The first point is where the robot is, checked again on the first 1 kHz cycle
     - within 0.02 rad per joint
     - ``JointTrajectoryDoesNotStartAtCurrentPosition``
   * - Every point is inside the joint limits
     - 0.01 rad margin
     - ``JointLimitsExceeded``
   * - No joint jumps between consecutive points (an IK solution flipping)
     - 0.025 rad
     - ``JointDistancesAboveMaxJointDifference``
   * - No joint moves faster than a fraction of its limit
     - 0.3 × (2.175 / 2.61 rad/s)
     - ``JointVelocitiesAboveMaximumAllowed``
   * - No joint's velocity changes abruptly from one segment to the next
     - 1.0 × (15 / 7.5 / 10 / 12.5 / 15 / 20 / 20 rad/s²)
     - ``JointAccelerationsAboveMaximumAllowed``
   * - Sensible timing
     - 1 ms ≤ dt ≤ 1 s, ≤ 600 s, ≤ 120 000 points
     - ``DtIs...``, ``TrajectoryTooLong``
   * - While moving, every joint stays near its command
     - 0.05 rad
     - ``TrackingErrorTooLarge`` (the motion stops)

The velocity check compares consecutive points, and the acceleration check compares
consecutive segments: the change of a joint's step from one segment to the next,
divided by dt². A trajectory from Ruckig passes easily. One that reverses or turns a
corner at a single point may not: spread the change of velocity over several points.
The first and last segments are not compared with rest, so a constant-velocity
trajectory (``np.linspace`` between two joint positions) is accepted.


Two GPU PCs, one robot
======================

- Both receive the robot's state; ``get_current_joint_positions()`` is at most
  20 ms old.
- While one PC's trajectory runs, the other's ``run_joint_trajectory`` raises
  ``RobotBusy``, naming the PC that has the robot.
  :attr:`~csc376_franky.FrankaJointTrajectoryController.is_executing` and
  :meth:`~csc376_franky.FrankaJointTrajectoryController.wait_until_idle` let you
  wait for it.
- Afterwards, a plan made before the robot moved no longer starts where the robot
  is, so the miniPC refuses it. Plan again from ``get_current_joint_positions()``.
- The gripper is shared the same way: one command at a time, and not while the
  other PC's trajectory runs.
- ``RtbVisualizer(..., real_robot=robot)`` adds a translucent blue Panda that
  follows the real robot, so each PC sees when the other one moves it.


.. _robot-interface-sim:

Trying it without a robot
=========================

The robot server can drive a simulated Panda and gripper, on any Linux or macOS
machine with a C++17 compiler. From ``robot-interface/``::

    pip install . ./server                          # builds the controller
    python -m csc376_franky_server --sim --bind 127.0.0.1
    CSC376_ROBOT_HOST=127.0.0.1 python examples/sim_or_real_trajectory_with_wrapper.py

The simulated robot goes through the same checks as the real one, so a trajectory
that it refuses would be refused in the lab too. ``--sim-speed 4`` runs it four
times faster than real time.


Troubleshooting
===============

.. list-table::
   :class: wrap-literals
   :header-rows: 1
   :widths: 50 50

   * - Message
     - Meaning / fix
   * - ``Cannot reach the robot server at 192.168.1.100:7376``
     - The robot server is not running on the miniPC, or the GPU PC's robot network
       cable or ``franka`` network profile is down
   * - ``The miniPC cannot reach the robot (...)``
     - FCI is not active in Desk, the brakes are locked, another FCI program holds
       the robot, or the robot is off; the message says which libfranka saw
   * - ``RobotNotReady: ... user stop``
     - Release the user stop
   * - ``RobotNotReady: ... call recover()``
     - A reflex (usually a collision) stopped the robot. Check the robot and its
       surroundings, then ``robot.recover()``
   * - ``RobotBusy``
     - The other GPU PC's trajectory or gripper command is running
   * - ``JointTrajectoryDoesNotStartAtCurrentPosition``
     - The robot moved since the plan was made: plan again from
       ``get_current_joint_positions()``
   * - ``JointAccelerationsAboveMaximumAllowed``
     - The trajectory changes a joint's velocity too abruptly at one point (a
       reversal, a sharp corner). Spread the change over more points
   * - ``TrackingErrorTooLarge``
     - A joint could not follow its command (contact, someone pushed the robot) and
       the motion was stopped
   * - ``IncompatibleVersion``
     - The GPU PC's ``csc376_franky`` and the robot server are from different
       versions of the repository


API reference
=============

The robot and its gripper
-------------------------

.. currentmodule:: csc376_franky

.. autoclass:: FrankaJointTrajectoryController
   :members:

.. autoclass:: Gripper(host=None, port=None, *, timeout=10.0)
   :members:
   :undoc-members:

.. autoclass:: RobotState
   :members:
   :undoc-members:

.. autoclass:: GripperState
   :members:
   :undoc-members:

.. autoexception:: RobotError

.. autoclass:: ErrorCodes(value)
   :members:
   :undoc-members:

Motion generator
----------------

.. autoclass:: csc376_franky.motion_generator.RuckigMotionGenerator
   :members:

Visualizer
----------

.. autoclass:: csc376_franky.visualizer.RtbVisualizer
   :class-doc-from: init
   :members:

.. autoclass:: csc376_franky.robot_wrapper.RobotWrapper
   :members:
   :undoc-members:

Wire protocol
-------------

.. automodule:: csc376_franky.protocol


Credits and licence
===================

The motion generator and the Swift visualizer are mostly Patrick Yao's
(``Patrick15Yao/franky_toolbox_bridge``, no longer online). The joint trajectory
controller is from `radiang/csc376 <https://github.com/radiang/csc376>`_, used with
its author's permission. The ``MotionGenerator`` behind ``move_home`` is a port of
the one in libfranka's examples (Apache License 2.0).

The robot interface is released under the MIT License.
`LICENSE <https://github.com/jbkahrs/robotics-toolbox-python-csc376/blob/main/robot-interface/LICENSE>`_
and
`NOTICE <https://github.com/jbkahrs/robotics-toolbox-python-csc376/blob/main/robot-interface/NOTICE>`_
have the licence and the details.
