# robot_simulation

ROS 2 (Jazzy) workspace containing the `robot_simulation` C++ package.

## Build

```bash
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Run

```bash
ros2 run robot_simulation simulation_node
```

## Robot description

Visualize [`N1-Robot.urdf`](src/robot_description/urdf/N1-Robot.urdf) in RViz:

```zsh
ros2 launch robot_description display.launch.py
```
