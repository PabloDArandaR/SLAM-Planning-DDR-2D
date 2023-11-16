#include "mapping/occupancy_grid.hpp"

occupancyGrid::occupancyGrid(int width, int height, int initial_value) : width{width}, height{height} {

    assert(((initial_value >= 0) && (initial_value <= 100),
            "Initial probability value is not between limits (must be between "
            "0 and 100)"));
    this->map.resize(this->width * this->height, initial_value);
}

// Returns a pointer to the vector that contains the map
std::vector<int8_t>* occupancyGrid::get_map() { return &this->map; }

float occupancyGrid::get_resolution() { return this->resolution; };

int occupancyGrid::get_width() { return this->width; };

int occupancyGrid::get_height() { return this->height; };

// Return probability in the cell given the x,y in meters in the frame of the
// map
int occupancyGrid::get_probability(int& x, int& y) {

    return this->map[(y - 1) * width - 1 + x];
}

// Set the probability for certain row, column
void occupancyGrid::set_probability(int& row, int& column, int& new_probability) {
    if (new_probability > 100){
        new_probability = 100;
    }
    else if (new_probability < 0){
        new_probability = 0;
    }
    this->map[this->_return_index(row, column)] = new_probability;
}

inline std::tuple<int, int> occupancyGrid::calculate_cell_x_y(float& x, float& y) {
    return std::tuple<int, int>(int(y / this->resolution), int(x / this->resolution));
}

inline int occupancyGrid::_return_index(int row, int column) { return (row - 1) * this->width - 1 + column; }