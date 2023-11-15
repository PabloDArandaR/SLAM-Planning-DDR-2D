import os
import ament_index_python.packages
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

import launch_ros.actions
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = "simulation_models"

    # Use xacro to process the file
    xacro_file = os.path.join(
        ament_index_python.packages.get_package_share_directory(pkg_name),
        "robot_models",
        "toycar",
        "desc",
        "toycar.xacro",
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    spawn_ignition = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', xacro_file,
            '-topic', 'robot_description', ],
        output='screen',
    )

    # Run the node
    return launch.LaunchDescription(
        [
            spawn_ignition,
            # origin_frame,
        ]
    )
