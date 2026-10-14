cmake_minimum_required(VERSION 3.8)
project(piper_arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn highgui imgproc)
find_package(onnxruntime_vendor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(color_object_pose_node src/color_object_pose_node.cpp)
ament_target_dependencies(color_object_pose_node
  geometry_msgs message_filters rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(color_object_pose_node ${OpenCV_LIBRARIES})
target_link_libraries(color_object_pose_node onnxruntime::onnxruntime)
target_include_directories(color_object_pose_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(color_object_pose_node PRIVATE cxx_std_17)

add_executable(red_box_camera_view_node src/red_box_camera_view_node.cpp)
ament_target_dependencies(red_box_camera_view_node
  geometry_msgs rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(red_box_camera_view_node ${OpenCV_LIBRARIES})
target_include_directories(red_box_camera_view_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(red_box_camera_view_node PRIVATE cxx_std_17)

add_executable(move_left_tcp_pregrasp_node src/move_left_tcp_pregrasp_node.cpp)
ament_target_dependencies(move_left_tcp_pregrasp_node
  geometry_msgs moveit_msgs moveit_ros_planning_interface rclcpp shape_msgs
  tf2 tf2_geometry_msgs tf2_ros)
target_compile_features(move_left_tcp_pregrasp_node PRIVATE cxx_std_17)

install(TARGETS color_object_pose_node red_box_camera_view_node move_left_tcp_pregrasp_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch model world
  DESTINATION share/${PROJECT_NAME})

ament_package()
