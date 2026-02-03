import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    debug = LaunchConfiguration('debug')
    config_path = LaunchConfiguration('config_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value=os.path.join(
                get_package_share_directory('pose_selector'),
                'config',
                'dope_converter_test.yaml'
            ),
            description='Path to DOPE class id config file'
        ),

        Node(
            package='pose_selector',
            executable='dope_converter_node',
            name='dope_converter_node',
            output='screen',
            parameters=[{'debug': debug, 'config_path': config_path}],
            remappings=[
                ('/dope_output', '/mobipick/dope/detected_objects')
            ]
        )
    ])