#include "mapping/mapping.hpp"

mappingOccupancyGrid::mappingOccupancyGrid(){};

void mappingOccupancyGrid::update([[maybe_unused]] Eigen::Vector3d relative_position,
                                  [[maybe_unused]] Eigen::Vector4d relative_rotation,
                                  [[maybe_unused]] std::vector<float> laser_measurements){};

occupancyGrid* mappingOccupancyGrid::get_map() { return this->map; }

void mappingOccupancyGrid::set_map(occupancyGrid* new_map) { this->map = new_map; };