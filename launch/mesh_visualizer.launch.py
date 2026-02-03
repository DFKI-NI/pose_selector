import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    mesh_config = LaunchConfiguration('mesh_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mesh_config',
            default_value=os.path.join(
                get_package_share_directory('pose_selector'),
                'config',
                'example_mesh_config.yaml'
            ),
            description='Path to mesh objects config file'
        ),

        Node(
            package='pose_selector',
            executable='pose_selector_visualizer_node',
            name='pose_selector_visualizer_node',
            output='screen',
            parameters=[{'mesh_config_file_path': mesh_config}],
            remappings=[
                ('/pose_selector_get_all_service', '/pose_selector_get_all')
            ]
        )
    ])