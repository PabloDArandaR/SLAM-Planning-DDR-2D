import os
import pathlib
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
#from webots_ros2_driver.urdf_spawner import get_webots_driver_node
#from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher


def generate_launch_description():
    package_dir = get_package_share_directory("toycar")
    urdf_path = os.path.join(package_dir, 'models', 'toycar.urdf')
    robot_description = pathlib.Path(urdf_path).read_text()

    webots_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_ROBOT_NAME': 'myRobot'},
        parameters=[
            {'robot_description': robot_description},
        ],
    )

    return LaunchDescription([
        webots_driver
    ])