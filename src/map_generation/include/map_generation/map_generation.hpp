#include <rclcpp/rclcpp.hpp>

#include "mapping/occupancy_grid.hpp"

class mapGenerator : public rclcpp::Node {
  public:
    mapGenerator() : rclcpp::Node("map_generator") {};

  private:
    int width, height; // cells
    float resolution;  // m/cells
    occupancyGrid map;
    // Vars
};