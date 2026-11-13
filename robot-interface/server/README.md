# CSC376 robot server (miniPC)

The robot server runs on each station's miniPC, the only machine on the station that
talks to the robot. It executes the trajectories that the station's GPU PCs plan
(`csc376_franky`, one directory up), and sends them the robot's state.

* `src/`, `include/`: the C++ controller. It checks trajectories, runs the 1 kHz
  joint impedance loop of the old `csc376_franky` controller through libfranka, and
  has a simulated robot for testing.
* `csc376_franky_server/`: the Python server around it. One TCP session per client,
  JSON lines (`csc376_franky/protocol.py` describes the messages).

In the lab it is already installed and running on every station's miniPC: there is
nothing to set up. What you can do with this folder is run the server with a
simulated robot, to try your programs without one.

## Without a robot

`--sim` drives a simulated robot and gripper instead (`--sim-speed 4` runs four times
faster than real time). The build skips libfranka when it is not installed, so this
works on any Linux or macOS machine with a C++17 compiler:

```bash
pip install .. .                                # from this directory: the protocol, then the server
python -m csc376_franky_server --sim --bind 127.0.0.1
```

Then point your program at it, for example from the directory above this one:

```bash
CSC376_ROBOT_HOST=127.0.0.1 python examples/sim_or_real_trajectory_with_wrapper.py
```
