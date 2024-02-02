#include <Eigen/Dense>

#include "mapping/mapping.hpp"
#include "mapping/no_noise.hpp"

#include "mapping/occupancy_grid.hpp"

int main() {
    noNoiseMethod mapping_method;
    scanerMetadata scaner;

    // Set the scaner metadata
    scaner.set_angledata(1.1, 1.1, 0.0);
    scaner.set_rangedata(0.1, 15);

    // Set occupancy grid
    int width{3};
    int height{3};
    double resolution{1};
    float initial_probability{0.5};
    occupancyGrid gridmap(width, height, initial_probability, resolution);

    // Set the mapping method
    mapping_method.set_scaner_metadata(scaner);
    mapping_method.set_map(&gridmap);

    // Set location data
    Eigen::Vector3d relative_position;
    relative_position << 0.1, 0.1, 0;
    Eigen::Vector4d relative_orientation;
    relative_orientation << 1, 0, 0, 0;
    std::vector<float> measurements(1, 2.2);
    mapping_method.update(relative_position, relative_orientation, measurements);
}