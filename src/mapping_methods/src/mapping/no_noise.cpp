#include <cmath>
#include <list>
#include <utility>
#include <iostream>

#include "mapping/no_noise.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/scanner_metadata.hpp"
#include "math/grid.hpp"
#include "math/linalg.hpp"
#include "types.hpp"

noNoiseMethod::noNoiseMethod(){};

noNoiseMethod::noNoiseMethod(occupancyGrid* map) { this->map = map; };

void noNoiseMethod::set_scaner_metadata(scanerMetadata metadata) { this->metadata = metadata; }

void noNoiseMethod::update(Eigen::Vector3d relative_position, Eigen::Vector4d relative_rotation,
                           std::vector<float> laser_measurements) {
    float angle{this->metadata.get_anglemin()};
    std::cout << "Angle min: " << this->metadata.get_anglemin() << std::endl;
    std::cout << "Angle max: " << this->metadata.get_anglemax() << std::endl;
    std::cout << "Angle increment: " << this->metadata.get_angleincrement() << std::endl;
    // Initial transformation
    Eigen::Matrix3d transformation{linalg::quaternionToRotation(relative_rotation)};

    Eigen::Vector3d r_initial, r_final;

    int i {0};

    for (float val : laser_measurements) {
        // TODO: Need to filter if the message is good or not (number of values should be adequate)
        ++i;

        // Obtain the initial and final point of the line in the reference of the map)
        r_initial << this->metadata.get_rangemin() * std::cos(angle), this->metadata.get_rangemin() * std::sin(angle),
            0;
        r_initial = linalg::transformPoint(r_initial, relative_position, transformation);

        r_final << val * std::cos(angle), val * std::sin(angle), 0;
        r_final = linalg::transformPoint(r_final, relative_position, transformation);
        //std::cout << "r_initial is: [" << r_initial.coeff(0) << ", " << r_initial.coeff(1) << ", " << r_initial.coeff(2) << "]" << std::endl;
        //std::cout << "r_final is: [" << r_final.coeff(0) << ", " << r_final.coeff(1) << ", " << r_final.coeff(2) << "]" << std::endl;

        // Translate into grid coordinates (Assumption: 0,0 is the origin)
        Point initial_grid_point(r_initial[0] / this->map->get_resolution(),
                                 r_initial[1] / this->map->get_resolution());
        Point final_grid_point(r_final[0] / this->map->get_resolution(), r_final[1] / this->map->get_resolution());
        //std::cout << "Initial grid point is: [" << initial_grid_point.x << ", " << initial_grid_point.y << "]" << std::endl;
        //std::cout << "Final grid point is: [" << final_grid_point.x << ", " << final_grid_point.y << "]" << std::endl;

        // Find the cells in which the ray is passing by
        std::list<Cell> crossing_cells(grid::crossing_line_cells(initial_grid_point, final_grid_point));
        //std::cout << "Found all cells" << std::endl;
        this->fill_cells(crossing_cells, val);
        //std::cout << "Right after fill_cells" << std::endl;

        // Update variables for the next step
        angle += this->metadata.get_angleincrement();
        //std::cout << "Increment of angle performed" << std::endl;
    }
}

void noNoiseMethod::fill_cells(std::list<Cell>& cell_list, float range_val){
        while (cell_list.size() != 1) {
            Cell checking_cell = cell_list.front();
            //std::cout << "Checking cell " << checking_cell.x << ", " << checking_cell.y << std::endl;
            //std::cout << "Index is: " << this->map->return_index(checking_cell.y, checking_cell.x) << std::endl;
            if (this->map->is_valid_cell(checking_cell)){
                //std::cout << "Valid cell" << std::endl;
                int new_probability = 0.0;
                try{
                    this->map->set_probability(checking_cell.y, checking_cell.x, new_probability);
                }
                catch (int err_num){
                    //std::cout << "Error number received is: " << err_num << std::endl;
                }
            }
            //std::cout << " Finished cell" << std::endl;
            cell_list.pop_front();
        }
        //std::cout << " Finished main cells" << std::endl;

        Cell last_cell = cell_list.front();
        if (this->map->is_valid_cell(last_cell)){
            if (range_val == this->metadata.get_rangemax()) {
                int new_probability = 0;
                this->map->set_probability(last_cell.y, last_cell.x, new_probability);
            } else {
                int new_probability = 100;
                this->map->set_probability(last_cell.y, last_cell.x, new_probability);
            }
        }
        //std::cout << " Finished all cells" << std::endl;
};