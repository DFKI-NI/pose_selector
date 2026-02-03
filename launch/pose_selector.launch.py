import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file = LaunchConfiguration('config_file')
    objects_of_interest = LaunchConfiguration('objects_of_interest')
    global_reference_frame = LaunchConfiguration('global_reference_frame')
    logical_image_topic = LaunchConfiguration('logical_image_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'objects_of_interest',
            default_value='["multimeter"]',
            description='List of objects of interest'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(get_package_share_directory('pose_selector'),
                                        'config',
                                        'pose_selector_default.yaml'),
            description='Name of the config file'
        ),
        DeclareLaunchArgument(
            'global_reference_frame',
            default_value='map',
            description='Global reference frame'
        ),
        DeclareLaunchArgument(
            'logical_image_topic',
            default_value='/mobipick/eef_main_cam/rgb/logical_image',
            description='Topic for logical image'
        ),

        Node(
            package='pose_selector',
            executable='pose_selector_node',
            name='pose_selector_node',
            output='screen',
            parameters=[
                {
                    'config_file': config_file,
                    'objects_of_interest': objects_of_interest,
                    'global_reference_frame': global_reference_frame
                }
            ],
            remappings=[
                ('/logical_image', logical_image_topic)
            ]
        )
    ])