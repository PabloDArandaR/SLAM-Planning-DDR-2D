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
    pkg_name = 'toycar'
    file_subpath = 'urdf/toy.urdf.xacro'

    # Set the path to this package.
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Use xacro to process the file
    xacro_file = os.path.join(pkg_share,file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Node declarations
    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'toycar', "-x", "0.0", "-y", "0.0", "-z", "2.0"],
                    output='screen')

    # Run the node
    return LaunchDescription([
        node_robot_state_publisher,
        spawn_entity
    ])