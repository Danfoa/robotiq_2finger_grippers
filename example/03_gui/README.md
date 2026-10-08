# Tutorial 3: Open/close GUI

A small Tkinter window that sends open and close goals to the gripper.

## Run

Simulation:

```bash
roslaunch robotiq_2f_gripper_examples gripper_gui.launch sim:=true
```

Real gripper:

```bash
roslaunch robotiq_2f_gripper_examples gripper_gui.launch comport:=/dev/ttyUSB0
```

The launch file (`example/03_gui/gripper_gui.launch`) starts the action server and the GUI together.

## Using the window

- **Speed** and **Force** sliders set the values used by the next command.
- **OPEN** moves the fingers to the full stroke, **CLOSE** moves them to 0.
- The status line shows the gripper state (`Ready`, `Moving`, `Holding object`, `Idle`, `Fault: 0x..`)
  and the current finger width in mm.
- The buttons are disabled until the action server is available.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `action_name` | `command_robotiq_action` | Action to command |
| `stroke` | `0.085` | Full-open finger distance [m] |
| `default_speed` | `0.1` | Initial speed slider value [m/s] |
| `default_force` | `50` | Initial force slider value |

The script is `example/03_gui/robotiq_2f_gripper_gui.py`. It needs the `tkinter`
package (`sudo apt install python3-tk`) and a display.
