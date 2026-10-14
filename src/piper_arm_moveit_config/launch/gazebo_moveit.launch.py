"""Run the dual Piper MoveIt configuration against Gazebo Sim controllers."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(get_package_share_directory('piper_arm_moveit_config'))
    description = package_share / 'config' / 'piper_dual_gripper.urdf.xacro'
    rviz_config = package_share / 'config' / 'moveit.rviz'
    world = package_share / 'world' / 'piper.world'
    moveit_config = (
        MoveItConfigsBuilder('piper', package_name='piper_arm_moveit_config')
        .robot_description(
            file_path=str(description),
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
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world}', 'on_exit_shutdown': 'true'}.items(),
    )
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', 'piper', '-topic', '/robot_description', '-name', 'piper'],
        output='screen',
    )
    controller_manager = '/controller_manager'
    joint_state_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', controller_manager],
        output='screen',
    )
    trajectory_spawners = [
        Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', controller_manager],
            output='screen',
        )
        for name in (
            'left_arm_controller', 'right_arm_controller',
            'left_gripper_controller', 'right_gripper_controller',
        )
    ]

    return LaunchDescription([
        # Gazebo resolves package:// mesh paths relative to this directory.
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                str(Path(get_package_share_directory('piper_arm_description')).parent),
                ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        gazebo,
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/left_camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/left_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/left_camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/left_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/right_camera/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/right_camera/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/right_camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/right_camera/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            output='screen',
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[moveit_config.robot_description, {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='piper_arm_moveit_config', executable='color_object_pose_node',
            parameters=[{
                'use_sim_time': True,
                'model_path': str(package_share / 'model' / 'red_target_yolov8s.onnx'),
                'target_class_id': 0,
                'confidence_threshold': 0.5,
                'object_size_x_m': 0.10,
                'object_yaw_rad': 0.0,
            }],
            output='screen',
        ),
        spawn_robot,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_spawner, *trajectory_spawners],
        )),
        Node(
            package='moveit_ros_move_group', executable='move_group',
            parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', str(rviz_config)],
            parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
            output='screen',
        ),
    ])
