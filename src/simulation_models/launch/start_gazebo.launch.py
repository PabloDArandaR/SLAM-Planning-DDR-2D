from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    run_node = Node(
        package='gazebo_ros',
        executable='gazebo',
        name='gazebo',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so']
    )

    return LaunchDescription([
        run_node
    ])