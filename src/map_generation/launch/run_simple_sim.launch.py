import  os
import sys
import json

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create the launch description and populate
    config = os.path.join(get_package_share_directory('map_generation'), 'config', 'params.yaml')

    frameInit = Node(
        name="frame_initialization",
        package="map_generation",
        executable="frame_initialization",
        output="screen",
        parameters=[config]
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gz_classic'),
                'launch',
                'simple_room.launch.py'
            ])
        ]),
        launch_arguments={}.items()
    )

    rviz = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([frameInit, simulation, rviz])