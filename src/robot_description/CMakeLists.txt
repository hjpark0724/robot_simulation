cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  gazebo
  meshes
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/gripper_command_router.py
  scripts/gz_pointcloud_to_optical.py
  scripts/imu_frame_relay.py
  scripts/keyboard_teleop.py
  scripts/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
