"""Planning-only MoveIt demo for the dual Piper arm with two grippers.

This launch is intentionally not connected to hardware controllers.  It is for
validating the robot model, TF, planning groups, and IK before ros2_control or
the physical Piper drivers are integrated.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    description = Path(get_package_share_directory('piper_arm_moveit_config')) / 'config' / 'piper_dual_gripper.urdf.xacro'
    rviz_config = Path(get_package_share_directory('piper_arm_moveit_config')) / 'config' / 'moveit.rviz'
    moveit_config = (
        MoveItConfigsBuilder('piper', package_name='piper_arm_moveit_config')
        .robot_description(
            file_path=str(description),
            mappings={
                'left_end_effector': 'gripper',
                'right_end_effector': 'gripper',
                'left_with_camera': 'true',
                'right_with_camera': 'true',
            },
        )
        .robot_description_semantic(file_path='config/piper_dual_gripper.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[moveit_config.robot_description], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[str(Path(get_package_share_directory('piper_arm_moveit_config')) / 'config' / 'ros2_controllers.yaml')],
             remappings=[('/controller_manager/robot_description', '/robot_description')], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='controller_manager', executable='spawner',
             arguments=['left_arm_controller', 'right_arm_controller', 'left_gripper_controller', 'right_gripper_controller', '--controller-manager', '/controller_manager'], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group',
             parameters=[moveit_config.to_dict()], output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(rviz_config)],
             parameters=[moveit_config.to_dict()], output='screen'),
    ])
