import  os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable 

def generate_launch_description():

    pkg_gazebo = FindPackageShare(package="gazebo_ros").find('gazebo_ros')
    pkg_share = FindPackageShare(package='apartment').find('apartment')

    ## Set the path to the world file
    world_file = 'apartment.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file)
    
    ## Modify environment variables
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, 'models')

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_gazebo, 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path}.items(),
    )
 
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_gazebo)

    return ld