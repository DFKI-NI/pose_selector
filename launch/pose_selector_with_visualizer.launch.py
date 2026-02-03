import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    mesh_config_file_path = LaunchConfiguration('mesh_config_file_path')
    
    visualizer_node = Node(
        package='pose_selector',
        executable='pose_selector_visualizer_node',
        name='pose_selector_visualizer_node',
        output='screen',
        parameters=[{'mesh_config_file_path': mesh_config_file_path}],
        remappings=[
            ('/pose_selector_get_all_service', '/pose_selector_get_all')
        ]
    )

    pose_selector_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_selector'), 'launch', 'pose_selector_demo.launch.py')
        ),
        launch_arguments={
            'config_file': PathJoinSubstitution([
                get_package_share_directory('pose_selector'),
                'config',
                'pose_selector_visualizer_demo.yaml'
            ]),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mesh_config_file_path',
            default_value=os.path.join(
                get_package_share_directory('pose_selector'),
                'config',
                'example_mesh_config.yaml'
            ),
            description='Path to mesh objects config file'
        ),
        visualizer_node,
        pose_selector_demo_launch
    ])