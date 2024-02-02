#include "mapping/occupancy_grid.hpp"
#include "types.hpp"

#ifndef MAPPING_GUARD
#define MAPPING_GUARD

class mappingOccupancyGrid {

  public:
    mappingOccupancyGrid();

    virtual void update([[maybe_unused]] Eigen::Vector3d relative_position,
                        [[maybe_unused]] Eigen::Vector4d relative_rotation,
                        [[maybe_unused]] std::vector<float> laser_measurements);

    occupancyGrid* get_map();

    void set_map(occupancyGrid* new_map);

  protected:
    occupancyGrid* map;
};

#endif