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
    pkg_name = 'toycar2D'
    file_subpath = 'urdf/toy.urdf.xacro'

    # Set the path to this package.
    pkg_share = FindPackageShare(package='toycar2D').find('toycar2D')


    # Path of the world file
    world_file_name = 'cafe.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    world = LaunchConfiguration('world')
 

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    joint_state_publisher = Node(
        name='joint_state_publisher',
        package='joint_state_publisher',
        executable='joint_staste_publsiher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}]
    )
    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
            launch_arguments={'world': os.path.join(get_package_share_directory(pkg_name), "worlds", "world.world")}.items()
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot', "-x", "0.0", "-y", "1.0", "-z", "2.0"],
                    output='screen')


    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])