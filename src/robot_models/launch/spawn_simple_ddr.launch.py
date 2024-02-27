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
    # Spawn robot system

    robot_filepath = os.path.join(pkg_robot_models, 'models', 'simple_DDR', 'model.sdf')

    ROBOT_MODEL = 'simple_DDR'

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ROBOT_MODEL ,
            '-file', robot_filepath,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3'
        ],
        output='screen',
    )

    return LaunchDescription([
        start_gazebo_ros_spawner_cmd
    ])


