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

## Gazebo simulation

`sim.launch.py` starts Gazebo, spawns the robot, bridges all topics, and opens
RViz in one command:

```zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch robot_description sim.launch.py
```

The Gazebo model [`N1-Robot.sdf`](src/robot_description/gazebo/N1-Robot.sdf)
contains a 100 Hz IMU sensor at the same pose as the URDF `imu_frame`:
`xyz="0.06 0 0.08"`, `rpy="0 0 0"`, and an RGB-D camera on the `DCW2` frame
(see [`docs/dcw_rgbd_camera_simulation.md`](docs/dcw_rgbd_camera_simulation.md)
for the full sensor and TF breakdown). The
[`n1_imu.world`](src/robot_description/worlds/n1_imu.world) world loads
physics, IMU, and rendering-sensor systems, a ground plane, and a free
`target_cylinder` for grasp-planning tests.

### Driving the base

Start keyboard control in a separate interactive terminal while
`sim.launch.py` is running:

```zsh
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 run robot_description keyboard_teleop.py
```

The keys are `w` forward, `x` reverse, `a` left, `d` right, and `s` stop. This
publishes `geometry_msgs/msg/Twist` on `/cmd_vel`, which the four-wheel
skid-steer `DiffDrive` plugin consumes (`wheel_separation=0.1955 m`,
`wheel_radius=0.04 m`, linear speed capped at 0.3 m/s, angular at 1.0 rad/s).

### Commanding the arm

Each arm joint takes a `std_msgs/msg/Float64` position target in radians:

```zsh
ros2 topic pub --once /arm1/cmd_pos std_msgs/msg/Float64 "{data: 0.3}"
```

Topics: `/arm1/cmd_pos` … `/arm5/cmd_pos`, limits `±1.5708 rad` (±90°) on each
joint. The home pose on spawn is `arm2=1.20`, `arm3=-1.45`, `arm4=-1.32`,
`arm1=arm5=0.0`.

### Commanding the gripper

Send a single signed command to `/gripper/cmd_pos`
(`std_msgs/msg/Float64`, range `±0.95 rad`):

```zsh
ros2 topic pub --once /gripper/cmd_pos std_msgs/msg/Float64 "{data: 0.6}"
```

`gripper_command_router.py` fans this out to the six finger joints
(`llink1..3`, `rlink1..3`) with the sign multipliers that used to be URDF
`mimic` joints, now applied in software since Gazebo's
`JointPositionController` plugin does not support `mimic`.

### Verifying topics

```zsh
ros2 topic hz /imu
ros2 topic echo /imu --once
ros2 topic hz /dcw/rgbd/points
```

`/imu`'s `header.frame_id` is `imu_frame`, matching the URDF TF frame:
`imu_frame_relay.py` rewrites the scoped sensor name that
`ros_gz_bridge` publishes (e.g. `n1_robot/base_footprint/imu_sensor`) on the
intermediate `/imu/raw` topic.

### Manual startup (debugging)

To bring the pieces up individually instead of via `sim.launch.py`, use
separate terminals:

```zsh
# 1. Gazebo
export GZ_SIM_RESOURCE_PATH="$(ros2 pkg prefix robot_description)/share:${GZ_SIM_RESOURCE_PATH}"
gz sim -r "$(ros2 pkg prefix robot_description)/share/robot_description/worlds/n1_imu.world"

# 2. Spawn (after the Gazebo window is rendering)
ros2 run ros_gz_sim create \
  -world n1_imu_world \
  -file "$(ros2 pkg prefix robot_description)/share/robot_description/gazebo/N1-Robot.sdf" \
  -name n1_robot \
  -z 0.10

# 3. Bridge the IMU (the single quotes are required in zsh: '[' is a glob character)
ros2 run ros_gz_bridge parameter_bridge '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
```
