import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'pose_estimation'
    
    transform_generator = Node(
        name='odom2tf',
        package=pkg_name,
        executable='no_noise',
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        transform_generator
    ])