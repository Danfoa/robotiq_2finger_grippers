# Tutorial 2: Python action client

Three scripts, from the lowest level to the most convenient. Keep the action server running
(for example Tutorial 1) and run the scripts with `rosrun` from a sourced terminal:

```bash
rosrun robotiq_2f_gripper_examples 01_open_close.py
rosrun robotiq_2f_gripper_examples 02_helpers.py
rosrun robotiq_2f_gripper_examples 03_feedback.py _position:=0.04 _speed:=0.05
```

## 01_open_close.py: raw goals

Creates a `CommandRobotiqGripperGoal`, fills in `position`, `speed` and `force`, and sends it with a
`SimpleActionClient`. `wait_for_result()` blocks until the server finishes the goal, and
`get_result()` returns the final gripper state.

## 02_helpers.py: helper functions

`Robotiq2FingerGripperDriver` has static helpers that build and send the goal for you:

```python
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

Robotiq.close(client, speed=0.1, force=50, block=True)
Robotiq.open(client, speed=0.1, force=50, block=True)
Robotiq.goto(client, pos=0.04, speed=0.02, force=10, block=False)  # returns immediately
```

With `block=False` your code keeps running while the gripper moves. Call `client.wait_for_result()`
later if you need to wait.

## 03_feedback.py: monitor the motion

Sends a goal with `feedback_cb` and `done_cb`. The server publishes feedback while the gripper moves,
and the final message tells you whether an object was detected between the fingers.

## Tips

- Always call `client.wait_for_server()` first, otherwise goals sent before the server starts are lost.
- Keep `speed` in 0.013 - 0.1 m/s and `force` in 0 - 100 %. Other values are clamped silently.
- To grasp an object, close with a moderate force and check `result.obj_detected`.

Next: [Tutorial 3, GUI](../03_gui).
