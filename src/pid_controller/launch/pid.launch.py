import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    toycar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('toycar'), 'launch')
            , '/spawn.launch.py'
        ])
    )
    pid_controller = Node(package='pid_controller', executable='pid', name='pid')
    send_poses = Node(package='send_poses', executable='send_poses', name='send_poses')

    # Run the node
    return LaunchDescription([
        toycar, pid_controller, send_poses
    ])