#include "mapping/mapping.hpp"

occupancyGrid* mappingOccupancyGrid::get_map() { return this->map; }

void mappingOccupancyGrid::set_map(occupancyGrid* new_map) { this->map = new_map; };