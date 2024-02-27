# Mapping and planning with 2D data

The aim of this project is to develop a simulation environment (currently in Gazebo) and test perception and navigation algorithms, both self-implemented or from other packages.

The current status of the project is:
- A simple environment has been created to perform some initial tests.
- A line-of-sight algorithm has been implemented and tested. Some bugs have been noticed and source is still to-be-found.
- No probabilistic update algorithm has been implemented yet.
- No navigation method has been implemented yet.

## Dependencies

This packages have been developed and testes using the following package versions:
- ROS 2 Iron
- Gazebo Harmonic

## Data formats

 - Occupancy grid (CSV)
   - Values in the range [0, 100]
 - 2d laser scaner recordings (CSV):
   - 3 translation
   - 4 rotation
   - n numbers of readings
 - Scaner metadata (YAML)
   - min_angle
   - max_angle
   - min_range
   - max_range
   - resolution
   - samples

## Interesting tools:
- URDF visualization: https://github.com/openrr/urdf-viz

## Sources of data:
- UR5e:
  - DH: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
  - Inertia properties and meshes: https://github.com/culurciello/pybullet_ur5_gripper/blob/master/robots/urdf/ur5e.urdf

## Common commands for debugging 

### Robot models
- Required sourcing:
  - ```source /opt/ros/iron/setup.bash && source /home/pablo/ws_ros/install/setup.bash && source /home/pablo/Projects/2d_DDR/install/setup.bash```
- Build and start Gazebo:
  - ```colcon build --packages-select robot_models && ros2 launch robot_models start_ign.launch.py```
- Spawn the robot:
  - ```ros2 launch robot_models spawn_simple_ddr.launch.py```
- Spawn the transport messages
  - ```ros2 launch robot_models simple_ddr_bridge.launch.py```
- GZ topic of lidar