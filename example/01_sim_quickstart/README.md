# Tutorial 1: Simulation quickstart

Start a simulated 2F-85 gripper and watch it in RViz. No hardware is needed.

## Run

```bash
roslaunch ~/catkin_ws/src/robotiq_2finger_grippers/example/01_sim_quickstart/sim_quickstart.launch
```

The launch file starts:

- `robotiq_2f_action_server.py` in simulation mode. It advertises the action `/command_robotiq_action`
  and publishes the gripper joint on `/joint_states`.
- `robot_state_publisher` and `joint_state_publisher`, which turn the joint state into TF frames.
- RViz, showing the gripper model.

## Send a command from the terminal

Open a second terminal (sourced) and publish a goal directly:

```bash
rostopic pub --once /command_robotiq_action/goal robotiq_2f_gripper_msgs/CommandRobotiqGripperActionGoal \
  "goal: {position: 0.0, speed: 0.05, force: 50.0}"
```

The fingers in RViz close. Use `position: 0.085` to open them again.

Useful checks:

```bash
rostopic list | grep robotiq        # action topics
rostopic echo /joint_states         # finger_joint position [rad]
rostopic echo /command_robotiq_action/feedback
```

## Use a real gripper

```bash
roslaunch .../sim_quickstart.launch sim:=false comport:=/dev/ttyUSB0
```

## Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `true` | Simulated gripper (`true`) or real gripper (`false`) |
| `comport` | `/dev/ttyUSB0` | Serial port of the real gripper |
| `baud` | `115200` | Must match the gripper configuration |

For the 2F-140, change `stroke` to `0.140` and use the `robotiq_2f_140_gripper_visualization` model.

Next: [Tutorial 2, Python client](../02_python_client).
