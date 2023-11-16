#include <cassert>
#include <cstdint>
#include <tuple>
#include <vector>

class occupancyGrid {
    public:
      occupancyGrid(int width, int height, int initial_value);

      std::vector<int8_t>* get_map();
      float get_resolution();
      int get_width();
      int get_height();
      int get_probability(int& x, int& y);

      void occupancyGrid::set_probability(int& row, int& column, int& new_probability);

      inline std::tuple<int, int> calculate_cell_x_y(float& x, float& y);

    private:
      int width, height; // cells
      float resolution;  // m/cells
      std::vector<int8_t> map;

      inline int _return_index(int row, int column);
};