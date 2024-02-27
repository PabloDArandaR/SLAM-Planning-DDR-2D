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

    bridge_laser = Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='lidar',
            arguments=["/lidar/points@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan", "--ros-args", "-r", "/lidar/points:=/laser_scan_cloud"]
        )
    
    bridge_clock = Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='clock',
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        )


    bridge_tf = Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='tf',
            arguments=["/model/simple_DDR/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"]
        )
    
    bridge_odom = Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='odom',
            arguments=["/model/simple_DDR/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"]
        )

    bridge_cmd_vel = Node(
            package='ros_gz_bridge',
            namespace='',
            executable='parameter_bridge',
            name='cmd_vel',
            arguments=["/model/simple_DDR/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist"]
        )

    #######################################################################################################################################
    # Run the node
    return LaunchDescription([
        bridge_laser,
        bridge_clock,
        bridge_odom,
        bridge_tf,
        bridge_cmd_vel
    ])


