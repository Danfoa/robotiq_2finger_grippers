# Tutorial 4: Multiple grippers

Run one action server per gripper, each in its own ROS namespace, and command them from one script.

## Run

Start two simulated grippers (the built-in test client is disabled with `run_test:=false`):

```bash
roslaunch robotiq_2f_gripper_control robotiq_dual_action_server.launch sim:=true run_test:=false
```

In a second terminal:

```bash
python3 dual_client.py
```

Both grippers close and open together. The goals are sent without blocking, so the grippers move
at the same time, then the script waits for both results.

## Namespaces

| Gripper | Action | Joint name |
|---------|--------|------------|
| right | `/right_gripper/command_robotiq_action` | `right_gripper_finger_joint` |
| left | `/left_gripper/command_robotiq_action` | `left_gripper_finger_joint` |

## Real grippers

Each gripper needs its own serial port. Defaults are `/dev/ttyRobotiq1` (right) and `/dev/ttyRobotiq0` (left):

```bash
roslaunch robotiq_2f_gripper_control robotiq_dual_action_server.launch \
  right_gripper_comport:=/dev/ttyUSB0 left_gripper_comport:=/dev/ttyUSB1 run_test:=false
```

Create persistent names with a udev rule so the ports do not swap after a reboot.
