from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_end_effector = LaunchConfiguration('left_end_effector')
    right_end_effector = LaunchConfiguration('right_end_effector')
    arm_spacing = LaunchConfiguration('arm_spacing')
    description_file = PathJoinSubstitution([
        FindPackageShare('piper_arm_description'),
        'urdf',
        'piper_description.urdf.xacro',
    ])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', description_file,
            ' ', 'left_end_effector:=', left_end_effector,
            ' ', 'right_end_effector:=', right_end_effector,
            ' ', 'arm_spacing:=', arm_spacing,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_end_effector',
            default_value='none',
            description='Dual mode left arm end effector: none, gripper, revo2, or pika.',
        ),
        DeclareLaunchArgument(
            'right_end_effector',
            default_value='none',
            description='Dual mode right arm end effector: none, gripper, revo2, or pika.',
        ),
        DeclareLaunchArgument(
            'arm_spacing',
            default_value='0.60',
            description='Y-axis distance in metres between the two arm base links.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('piper_arm_description'), 'rviz', 'default.rviz',
            ])],
            output='screen',
        ),
    ])
