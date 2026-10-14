"""Move the provisional left TCP to 3 cm before the detected red box."""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder('piper', package_name='piper_arm_moveit_config')
        .robot_description(
            file_path='config/piper_dual_gripper.urdf.xacro',
            mappings={
                'left_end_effector': 'gripper',
                'right_end_effector': 'gripper',
                'left_with_camera': 'true',
                'right_with_camera': 'true',
                'use_gazebo': 'true',
            },
        )
        .robot_description_semantic(file_path='config/piper_dual_gripper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .to_moveit_configs()
    )
    return LaunchDescription([
        Node(
            package='piper_arm_moveit_config', executable='move_left_tcp_pregrasp_node',
            parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
            output='screen',
        ),
    ])
