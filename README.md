# Mapping and planning with 2D data

The aim of this project is to develop a simulation environment (currently in Gazebo) and test perception and navigation algorithms, both self-implemented or from other packages.

The current status of the project is:
- A simple environment has been created to perform some initial tests.
- A line-of-sight algorithm has been implemented and tested. Some bugs have been noticed and source is still to-be-found.
- No probabilistic update algorithm has been implemented yet.
- No navigation method has been implemented yet.

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
