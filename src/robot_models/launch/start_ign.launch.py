import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction


from launch_ros.actions import Node
import xacro

import yaml

def generate_launch_description():

    pkg_robot_models = get_package_share_directory('robot_models')
    #######################################################################################################################################
    # Start gazebo

    world_file = os.path.join(pkg_robot_models, 'worlds', 'test_world.sdf')

    world_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': world_file + ' -r -v 4'
            }.items()
        )

    #######################################################################################################################################
    # Run the node
    return LaunchDescription([
        world_launch
    ])


