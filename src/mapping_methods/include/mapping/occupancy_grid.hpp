#include <cassert>
#include <cstdint>
#include <tuple>
#include <vector>

#include "types.hpp"

#ifndef OCCUPANCY_GRID_GUARD
#define OCCUPANCY_GRID_GUARD

class occupancyGrid {
  public:
    occupancyGrid();
    occupancyGrid(int width, int height, int initial_value, float resolution);
    occupancyGrid(std::vector<int8_t>* new_map, int width, int height, int initial_value, float resolution);

    std::vector<int8_t>* get_map();
    float get_resolution();
    int get_width();
    int get_height();
    int get_probability(int& x, int& y);

    void set_probability(int& row, int& column, int& new_probability);
    void set_map_by_reference(std::vector<int8_t>*);

    inline std::tuple<int, int> calculate_cell_x_y(float& x, float& y);

    bool is_valid_cell(Cell check_cell);

    void store_map(std::string filename);
    int return_index(int row, int column);

  private:
    int width, height; // cells
    float resolution;  // m/cells
    std::vector<int8_t>* map;

};

#endif