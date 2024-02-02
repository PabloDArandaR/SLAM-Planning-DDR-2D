import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


from launch_ros.actions import Node
import xacro

import yaml

def generate_launch_description():

    #######################################################################################################################################
    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gz_classic'
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_share_directory(pkg_name), "models")
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    #######################################################################################################################################
    # Start gazebo

    pkg_gazebo =  get_package_share_directory('gazebo_ros')
    pkg_sim = get_package_share_directory(pkg_name)
    world_path = os.path.join(pkg_sim, 'worlds', 'simple_room.sdf')

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo, 'launch'), '/gzserver.launch.py']),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
    )
    
    #######################################################################################################################################
    # Robot State Publisher node

    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'models', 'robot_models', 'toycar', 'desc', 'toycar.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    params = {'robot_description': robot_description_raw, 'verbose': True, 'use_sim_time': True}
    rsp = Node(package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])

    #######################################################################################################################################
    # Spawn entity
    
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'toycar', '-x' , '0', '-y', '0', '-z', '1'],
        remappings=[
            ('/camera_R/image_raw', 'image_R'),
            ('/camera_L/image_raw', 'image_L'),
            ('/laser_controller/out', 'laser_scan')],
        output='screen')

    #######################################################################################################################################
    # Run the node
    return LaunchDescription([
        start_gazebo,
        rsp,
        spawn_robot,
    ])


