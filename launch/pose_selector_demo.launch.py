import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    debug = LaunchConfiguration('debug')
    config_file = LaunchConfiguration('config_file')
    objects_of_interest = LaunchConfiguration('objects_of_interest')
    global_reference_frame = LaunchConfiguration('global_reference_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'objects_of_interest',
            default_value='["can", "bottle", "screwdriver"]',
            description='List of objects of interest'
        ),
        DeclareLaunchArgument(
            'global_reference_frame',
            default_value='map',
            description='Global reference frame'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('pose_selector'),
                                       'config',
                                       'pose_selector_demo.yaml'),
            description='Path to the pose selector configuration file'
        ),
        Node(
            package='pose_selector',
            executable='pose_selector_node',
            name='pose_selector_node',
            output='screen',
            parameters=[
                {
                    'debug': debug,
                    'config_file': config_file,
                    'objects_of_interest': objects_of_interest,
                    'global_reference_frame': global_reference_frame
                }
            ]
        )
    ])