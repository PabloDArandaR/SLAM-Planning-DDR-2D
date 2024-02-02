#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

#include "mapping/mapping.hpp"
#include "mapping/no_noise.hpp"

#include "mapping/occupancy_grid.hpp"

int main() {
    noNoiseMethod mapping_method;
    scanerMetadata scaner;

    // Set the scaner metadata
    float angle_max{1}, angle_min{-1};
    float n_measures {150};
    float angle_increment {(angle_max - angle_min)/n_measures};
    scaner.set_angledata(angle_increment, angle_min, angle_max);
    scaner.set_rangedata(0.3, 15);

    // Set occupancy grid
    int width{300};
    int height{300};
    double resolution{0.1};
    float initial_probability{50.0};
    occupancyGrid gridmap(width, height, initial_probability, resolution);

    // Set the mapping method
    mapping_method.set_scaner_metadata(scaner);
    mapping_method.set_map(&gridmap);

    // Setup recorded data
    std::string data_line;
    std::ifstream data_file("../recording_4.csv");
    while(std::getline(data_file, data_line)){
        std::istringstream iss(data_line);
        char comma;
        float posX, posY, posZ, q0, qX, qY, qZ;
        iss >> posX >> comma >> std::ws >> posY >> comma >> std::ws >> posZ >> comma >> std::ws;
        iss >> q0 >> comma >> std::ws >> qX >> comma >> std::ws >> qY >> comma >> std::ws >> qZ/*  >> comma >> std::ws */;

        std::vector<float> measurements(150, 0.0);
        float value{0};
        int i{0};
        while (iss >> std::ws >> comma >> value){
            measurements[i] = value;
            ++i;
        }

        Eigen::Vector3d relative_position;
        relative_position << posX, posY, posZ;
        Eigen::Vector4d relative_orientation;
        relative_orientation << q0, qX, qY, qZ;
        mapping_method.update(relative_position, relative_orientation, measurements);
        mapping_method.get_map()->store_map("map.csv");
    }

    // Set location data
}