#include "map_generation/occupancy_grid.hpp"

occupancyGrid::occupancyGrid(int width, int height, int initial_value) : width{width}, height{height} {

    assert(((initial_value >= 0) && (initial_value <= 100),
            "Initial probability value is not between limits (must be between "
            "0 and 100)"));
    this->map.resize(this->width * this->height, initial_value);
}

// Returns a pointer to the vector that contains the map
std::vector<int8_t>* occupancyGrid::get_map() { return &this->map; }

// Return probability in the cell given the x,y in meters in the frame of the
// map
int occupancyGrid::get_probability(float& x, float& y) {

    int row{0};
    int column{0};

    std::tie(row, column) = this->calculate_cell_x_y(x, y);

    return this->map[(row - 1) * width - 1 + column];
}

// Set the probability for certain row, column
void occupancyGrid::set_probability(float& row, float& column, int& new_probability) {
    this->map[this->_return_index(row, column)];
}

inline std::tuple<int, int> occupancyGrid::calculate_cell_x_y(float& x, float& y) {
    return std::tuple<int, int>(int(y / this->resolution), int(x / this->resolution));
}

inline int occupancyGrid::_return_index(int row, int column) { return (row - 1) * this->width - 1 + column; }