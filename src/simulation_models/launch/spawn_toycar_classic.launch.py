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
    pkg_name = "robot_models"
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(
        ament_index_python.packages.get_package_share_directory(pkg_name), "models"
    )
    os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"

    # Use xacro to process the file
    xacro_file = os.path.join(
        ament_index_python.packages.get_package_share_directory(pkg_name),
        "models",
        "toycar",
        "desc",
        "toycar.xacro",
    )
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # State Publisher node
    params = {
        "robot_description": robot_description_raw,
        "verbose": True,
        "use_sim_time": True,
    }
    rsp = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Spawn entity
    spawn_robot = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "toycar",
            "-x",
            "1",
            "-y",
            "1",
            "-z",
            "0.25",
        ],
        remappings=[
            ("/camera_R/image_raw", "image_R"),
            ("/camera_L/image_raw", "image_L"),
            ("/laser_controller/out", "/laser/scan"),
        ],
        output="screen",
    )

    # Run the node
    return launch.LaunchDescription(
        [
            rsp,
            spawn_robot,
            # origin_frame,
        ]
    )
