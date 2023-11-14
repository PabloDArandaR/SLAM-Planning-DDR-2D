#include <cassert>
#include <cstdint>
#include <tuple>
#include <vector>

class occupancyGrid {
  public:
    occupancyGrid(int width, int height, int initial_value);

    std::vector<int8_t>* get_map();
    int get_probability(float& x, float& y);

    void occupancyGrid::set_probability(float& row, float& column, int& new_probability);

    inline std::tuple<int, int> calculate_cell_x_y(float& x, float& y);

  private:
    int width, height; // cells
    float resolution;  // m/cells
    std::vector<int8_t> map;

    inline int _return_index(int row, int column);
};