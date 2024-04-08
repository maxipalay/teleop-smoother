# teleop-smoother

A very simple package to smooth the output velocity of the famous `teleop_twist_keyboard`.

When developing a robot, I came across the issue that the `teleop_twist_keyboard` generate sudden jumps in velocity (infinite acceleration). This causes unnecessary strain on the robot joints and generates a loud popping noise as the actuators are high-torque.

This package is a simple approach at smoothing the output, effectively creating a trapezoidal profile.

## `Smoother` Node

The core of this package is the `Smoother` node. This node subscribes to a velocity topic, and publishes acceleration-limited velocity commands on a separate topic.

Input args:
- `linear_acceleration` - linear, per axis acceleration (m/s^2)
- `angular_acceleration` - angular, per axis acceleration (degrees/s^2)
- `input_vel_topic` - name of topic to use as input velocity commands
- `output_vel_topic` - name of topic to publish smoother output
- `frequency` - loop frequency (Hz) (default: 50)

## Launch

A simple launchfile is provided. To run just build the workspace and run `ros2 launch teleop-smoother smoother.launch.py`
