import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'apartment_sim'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'robot_models', 'toycar', 'desc', 'toycar.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # State Publisher node
    params = {'robot_description': robot_description_raw, 'verbose': True, 'use_sim_time': True}
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[params])

    # Spawn entity
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'toycar'],
                    output='screen')

    # Run the node
    return LaunchDescription([
        rsp,
        spawn_entity,
    ])


