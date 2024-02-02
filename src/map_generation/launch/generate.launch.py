import  os
import sys
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the launch description and populate
    config = os.path.join(get_package_share_directory('map_generation'), 'config', 'params.yaml')
    print(f"Config is: {config}")
    MapGen = Node(
        name='map_generation',
        package='map_generation',
        executable='map_generation',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([MapGen])