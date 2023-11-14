#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

class scan_writter : public rclcpp::Node {
  public:
    scan_writter() : rclcpp::Node("map_generator") {
        this->subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 10, std::bind(&scan_writter::save_callback, this, _1));
    };

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;

    void save_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->store_scan_data(msg);
        this->store_map_data();
    }

    void store_scan_data(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::cout << "Recording data" << std::endl;
        std::ofstream f;
        f.open("resources/laser_scan.txt");
        f << "Angle min: " << msg->angle_min << std::endl;
        f << "Angle max: " << msg->angle_max << std::endl;
        f << "Angle increment: " << msg->angle_increment << std::endl;
        f << "Time increment: " << msg->time_increment << std::endl;
        f << "Scan time: " << msg->scan_time << std::endl;
        f << "Range max: " << msg->range_max << std::endl;
        f << "Range min: " << msg->range_min << std::endl;
        f << "Size of ranges vector: " << msg->ranges.size() << std::endl;
        f << "Size of intensities vector: " << msg->intensities.size() << std::endl;
        f << "Values in the vector are: " << std::endl;
        for (float val : msg->ranges) {
            f << "    - " << val << std::endl;
        }
    }

    void store_map_data() {}
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::cout << "Creating node" << std::endl;
    auto node = std::make_shared<scan_writter>();
    std::cout << "Creating executor" << std::endl;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::cout << "Adding node to executor" << std::endl;
    executor.add_node(node);
    std::cout << "Spinning" << std::endl;
    executor.spin();

    return 0;
}