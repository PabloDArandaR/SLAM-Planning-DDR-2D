import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'apartment_sim'

    # Use xacro to process the file
    urdf = os.path.join(get_package_share_directory(pkg_name), 'robot_models', 'bugbot', 'desc', 'bugbot.urdf')

    # State Publisher node
        
    robot_state_publisher = Node(
              package='robot_state_publisher',
              executable='robot_state_publisher',
              name='robot_state_publisher',
              output='screen',
              parameters=[{'use_sim_time': True}],
              arguments=[urdf])

    # Spawn entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'bugbot'],
                    output='screen')

    # Use sim time
    sim_time = DeclareLaunchArgument(
              'use_sim_time',
              default_value='True',
              description='Use simulation (Gazebo) clock if true')

    # Run the node
    return LaunchDescription([
        sim_time,
        robot_state_publisher,
        spawn_entity,
    ])