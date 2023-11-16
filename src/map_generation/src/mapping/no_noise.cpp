#include <cmath>
#include <list>
#include <utility>

#include "types.hpp"
#include "mapping/no_noise.hpp"
#include "math/grid.hpp"
#include "math/linalg.hpp"

noNoiseMethod::noNoiseMethod(){};

noNoiseMethod::noNoiseMethod(occupancyGrid* map) { this->map = map; };

void noNoiseMethod::set_scaner_metadata(scanerMetadata metadata) { this->metadata = metadata; }

void noNoiseMethod::update(Eigen::Vector3f relative_position, Eigen::Vector4f relative_rotation,
                           std::vector<float> laser_measurements) {
    float angle{this->metadata.get_anglemin()};

    // Initial transformation
    Eigen::Matrix3f transformation;
    transformation = linalg::quaternionToRotation(relative_rotation);

    for (float val : laser_measurements) {
        // Obtain the initial and final point of the line in the reference of the map
        Eigen::Vector3f r_initial, r_final;

        r_initial << this->metadata.get_rangemin() * std::cos(angle), this->metadata.get_rangemin() * std::sin(angle),
            0;
        r_initial = transformation * r_initial + relative_position;

        r_final << val * std::cos(angle), val * std::sin(angle), 0;
        r_final = transformation * r_final + relative_position;

        // Translate into grid coordinates (Assumption: 0,0 is the origin)
        Point initial_grid_point (r_initial[0] / this->map->get_resolution(), r_initial[1] / this->map->get_resolution());
        Point final_grid_point (r_final[0] / this->map->get_resolution(), r_final[1] / this->map->get_resolution());

        // Find the cells in which the ray is passing by
        std::list<Cell> crossing_cells(grid::crossing_line_cells(initial_grid_point, final_grid_point));

        // Update the probability of every point
        while (crossing_cells.size() != 1){
            Cell checking_cell = crossing_cells.front();
            int new_probability = this->map->get_probability(checking_cell.x, checking_cell.y) + 50;
            this->map->set_probability(checking_cell.x, checking_cell.y, new_probability);

            crossing_cells.pop_front();
        }

        // Update variables for the next step
        angle += this->metadata.get_angleincrement();
    }
}