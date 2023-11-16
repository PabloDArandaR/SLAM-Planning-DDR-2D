#include "mapping/occupancy_grid.hpp"
#include "mapping/types.hpp"

class mappingOccupancyGrid {

  public:
    virtual void update();

    occupancyGrid* get_map();

    void set_map(occupancyGrid* new_map);

  protected:
    occupancyGrid* map;
};