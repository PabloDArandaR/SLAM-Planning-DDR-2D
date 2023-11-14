#include "mapping/occupancy_grid.hpp"

class mappingOccupancyGrid{

    public:
        virtual void update();

        occupancyGrid * get_map();
        
        void set_map(occupancyGrid * new_map);

    private:
        occupancyGrid * map;

};