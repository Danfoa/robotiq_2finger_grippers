# Robotiq 2F Gripper Examples and Tutorials

Step-by-step examples for controlling a Robotiq 2F-85 / 2F-140 gripper from ROS (Noetic, Python 3).
Every example works in **simulation** (`sim:=true`), so you can try them without hardware.

## Prerequisites

1. Build the workspace and source it in every terminal you use:
   ```bash
   cd ~/catkin_ws && catkin build robotiq_2f_gripper_examples
   source ~/catkin_ws/devel/setup.bash
   ```
2. For a real gripper, give your user access to the serial port (log out and in afterwards):
   ```bash
   sudo adduser $USER dialout
   dmesg | grep tty        # find the port, usually /dev/ttyUSB0
   ```

The examples are the ROS package `robotiq_2f_gripper_examples`, so `roslaunch` and `rosrun` find them by name.

## Tutorials

| # | Folder | What you learn |
|---|--------|----------------|
| 1 | [`01_sim_quickstart`](01_sim_quickstart) | Start the action server and see the gripper move in RViz |
| 2 | [`02_python_client`](02_python_client) | Send goals from Python: raw goals, helper functions, feedback |
| 3 | [`03_gui`](03_gui) | Open/close the gripper with a small Tkinter GUI |

Follow them in order. Each folder has its own README.

## Gripper command reference

All examples talk to the same action, `robotiq_2f_gripper_msgs/CommandRobotiqGripperAction`,
advertised as `command_robotiq_action`.

| Goal field | Unit | Valid range | Notes |
|------------|------|-------------|-------|
| `position` | m | 0.0 - 0.085 (2F-85), 0.0 - 0.140 (2F-140) | Distance between the fingers. 0 is fully closed |
| `speed` | m/s | 0.013 - 0.1 | Values outside the range are clamped by the driver |
| `force` | % | 0 - 100 | Values outside the range are clamped by the driver |
| `stop` | bool | | Stops the motion (real gripper only) |
| `emergency_release` | bool | | Releases the fingers (real gripper only) |

Feedback and result share the same fields: `position`, `requested_position`, `is_moving`,
`obj_detected`, `fault_status`, `is_ready`, `is_reset`, `current`.

## Simulation limits

The simulated gripper only models finger motion. It never reports `obj_detected`, and it ignores
`stop`, `emergency_release` and `force`. Test those features on a real gripper.
