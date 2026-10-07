from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
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

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'n1_imu_world',
                    '-file', str(model_file),
                    '-name', 'n1_robot',
                    '-z', '0.10',
                ],
                output='screen'),
        ])

    return LaunchDescription([
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
            ],
            remappings=[
                (joint_state_gz_topic, '/joint_states'),
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
            executable='odom_tf_broadcaster.py',
            parameters=[{'use_sim_time': True}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_file)],
            parameters=[{'use_sim_time': True}],
            output='screen'),
    ])
