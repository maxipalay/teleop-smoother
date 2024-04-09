# teleop-smoother

![demo_image](https://github.com/maxipalay/teleop-smoother/assets/41023326/53232134-3828-4628-8f34-20d79baa9587)

A very simple package to smooth the output velocity of the famous `teleop_twist_keyboard`. The demo image shows the raw linear `x` velocity (as commanded by `teleop_twist_keyboard`) and the processed velocity command.

When developing a robot, I came across the issue that the `teleop_twist_keyboard` generates sudden jumps in velocity (infinite acceleration). This causes unnecessary strain on the robot joints and generates a loud popping noise as the actuators are high-torque.

This package is a simple approach at smoothing the output, effectively creating a trapezoidal profile. The acceleration is at the maximum at all times.

Note: if you intend to stop the robot rapidly, I would NOT recommend this, as it introduces delays.

## Nodes

The core of this package is the `Smoother` node. This node subscribes to a velocity topic, and publishes acceleration-limited velocity commands on a separate topic.

- `smoother` - processes the velocity commands
- `plotter` - plots velocities for two topics (raw vs processed)
### Smoother
Input args:
- `linear_acceleration` - linear, per axis acceleration (m/s^2)
- `angular_acceleration` - angular, per axis acceleration (degrees/s^2)
- `input_vel_topic` - name of topic to use as input velocity commands
- `output_vel_topic` - name of topic to publish smoother output
- `frequency` - loop frequency (Hz) (default: 50)
### Plotter
Input args:
- `raw_vel_topic` - raw velocity commands as they come out of `teleop_twist_keyboard`
- `smooth_vel_topic` - processed velocity commands topic
  
## Launch

A simple launchfile is provided. To run just build the workspace and run `ros2 launch teleop-smoother smoother.launch.py`. By default, the plotting node is not launched. To launch it use `os2 launch teleop-smoother smoother.launch.py plot:=true`.

## Plotting

The plotting node offers two services: `/capture_data` and `/plot_data`. To plot:
- `ros2 service call /capture_data std_srvs/srv/Empty` - this will start capturing data from both topics
- `ros2 service call /capture_data std_srvs/srv/Empty` - this will stop data capturing
- `ros2 service call /plot_data std_srvs/srv/Empty` - this will plot the captured data

# TODO

- separate accel/decel control?
- jerk control?
- multi-axis composite acceleration control?
