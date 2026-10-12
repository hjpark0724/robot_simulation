from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_description'))
    package_share_parent = package_share.parent
    robot_file = package_share / 'urdf' / 'N1-Robot.urdf'
    model_file = package_share / 'gazebo' / 'N1-Robot.sdf'
    world_file = package_share / 'worlds' / 'n1_imu.world'
    rviz_file = package_share / 'rviz' / 'robot.rviz'
    robot_description = robot_file.read_text()
    joint_state_gz_topic = '/world/n1_imu_world/model/n1_robot/joint_state'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items())

    # Gazebo needs time to start rendering (ogre2) and load the meshes before
    # it can accept a spawn request. This fixed delay is a pragmatic margin,
    # not a guaranteed sync point; raise it further on slower machines.
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'n1_imu_world',
                    '-file', str(model_file),
                    '-name', 'n1_robot',
                    # base_footprint is ground-referenced: wheel centers are
                    # already at z=0.04 m with 0.04 m radii.  Spawning at
                    # z=0.10 drops the robot onto its wheels and can tip it.
                    # Keep a small initial clearance so the contact solver can
                    # establish the four wheel contacts without penetration.
                    '-z', '0.002',
                ],
                output='screen'),
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_arm_gui', default_value='true',
            description='Start the Qt arm/gripper test GUI.'),
        # model://robot_description/... URIs in the converted SDF resolve here.
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                str(package_share_parent),
                ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ]),
        gazebo,
        spawn_robot,
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/dcw/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/dcw/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/dcw/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/dcw/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'{joint_state_gz_topic}@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/arm1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/arm5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/llink1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/llink2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/llink3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/rlink1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/rlink2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/rlink3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/gripper/attach_target@std_msgs/msg/Empty]gz.msgs.Empty',
                '/gripper/detach_target@std_msgs/msg/Empty]gz.msgs.Empty',
            ],
            remappings=[
                (joint_state_gz_topic, '/joint_states'),
                ('/dcw/rgbd/points', '/dcw/rgbd/points_raw'),
                # The bridged topic keeps Gazebo's scoped sensor name as
                # header.frame_id; imu_frame_relay.py republishes it on
                # /imu with the URDF's imu_frame name instead.
                ('/imu', '/imu/raw'),
            ],
            output='screen'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            output='screen'),
        Node(
            package='robot_description',
            executable='gripper_command_router.py',
            output='screen'),
        Node(
            package='robot_description',
            executable='imu_frame_relay.py',
            output='screen'),
        Node(
            package='robot_description',
            executable='odom_tf_broadcaster.py',
            parameters=[{'use_sim_time': True}],
            output='screen'),
        Node(
            package='robot_description',
            executable='gz_pointcloud_to_optical.py',
            parameters=[{'use_sim_time': True}],
            output='screen'),
        Node(
            package='robot_description',
            executable='arm_test_gui.py',
            condition=IfCondition(LaunchConfiguration('start_arm_gui')),
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_file)],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
