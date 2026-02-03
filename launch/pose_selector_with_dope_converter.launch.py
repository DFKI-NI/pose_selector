from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    dope_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_selector'), 'launch', 'dope_converter.launch.py')
        )
    )

    pose_selector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('pose_selector'), 'launch', 'pose_selector.launch.py')
        ),
        launch_arguments={'logical_image_topic': '/dope_converter_poses'}.items()
    )

    return LaunchDescription([
        dope_converter_launch,
        pose_selector_launch
    ])