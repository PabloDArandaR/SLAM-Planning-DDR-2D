#include "mapping/occupancy_grid.hpp"
#include "types.hpp"

#include <fstream>
#include <iostream>

occupancyGrid::occupancyGrid() : width{0}, height{0} {};

occupancyGrid::occupancyGrid(int width, int height, int initial_value, float resolution)
    : width{width}, height{height}, resolution{resolution}, map{new std::vector<int8_t>(0)} {

    assert((((initial_value >= 0) & (initial_value <= 100)),
            "Initial probability value is not between limits (must be between "
            "0 and 100)"));
    this->map->resize(this->width * this->height, initial_value);
}

occupancyGrid::occupancyGrid(std::vector<int8_t>* new_map, int width, int height, int initial_value, float resolution)
    : width{width}, height{height} {
    this->map = new_map;
    std::fill(this->map->begin(), this->map->end(), initial_value);
}

// Returns a pointer to the vector that contains the map
std::vector<int8_t>* occupancyGrid::get_map() { return this->map; }

float occupancyGrid::get_resolution() { return this->resolution; };

int occupancyGrid::get_width() { return this->width; };

int occupancyGrid::get_height() { return this->height; };

// Return probability in the cell given the x,y in meters in the frame of the
// map
int occupancyGrid::get_probability(int& row, int& column) { return (*this->map)[this->return_index(row, column)]; }

// Set the probability for certain row, column
void occupancyGrid::set_probability(int& row, int& column, int& new_probability) {
    if (new_probability > 100) {
        new_probability = 100;
    } else if (new_probability < 0) {
        new_probability = 0;
    }
    (*this->map)[this->return_index(row, column)] = new_probability;
}

inline std::tuple<int, int> occupancyGrid::calculate_cell_x_y(float& x, float& y) {
    return std::tuple<int, int>(int(y / this->resolution), int(x / this->resolution));
}

bool occupancyGrid::is_valid_cell(Cell check_cell){
    if ((check_cell.x >= this->width) | (check_cell.x < 0) | (check_cell.y >= this->height) | (check_cell.y < 0)){
        return false;
    }
    return true;
}

inline int occupancyGrid::return_index(int row, int column) {
    return row * this->width + column; }


void occupancyGrid::store_map(std::string filename){
    std::ofstream file(filename);
    for (int i {0}; i < this->height; ++i){
        for (int j{0}; j < this->width - 1; ++j){
            file << std::to_string((*this->map)[j + i*this->width]) << ", ";
        }
        file << std::to_string((*this->map)[this->width - 1 + i*this->width]);
        file << std::endl;
    }
    file.close();
};