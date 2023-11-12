#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "map_msgs/msg/occupancy_grid_update.hpp"

using std::placeholders::_1;


class MapStorage : public rclcpp::Node
{
    public:
        MapStorage() : rclcpp::Node("store_map"){
            std::cout << "Creating subscription" << std::endl;
            this->subscriber = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>("map_updates", 10, std::bind(&MapStorage::save_callback, this, _1));
        }
    
    private:
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr subscriber;

        void save_callback(const map_msgs::msg::OccupancyGridUpdate & msg){
            RCLCPP_INFO(this->get_logger(), "Saving received map");
            std::ofstream data_file;
            //auto current_time = std::chrono::high_resolution_clock::now();
            data_file.open("resources/latest_map.map");
            for (uint64_t i {0}; i < msg.height; i++){
                data_file << msg.data.at(i*msg.width);
                for (uint64_t j {1}; j < msg.width; j++){
                    data_file << ", " << msg.data.at(j + i*msg.width);
                }
                data_file << "\n";
            }
            data_file.close();
            rclcpp::shutdown();
        };
};


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    std::cout << "Creating node" << std::endl;
    auto node = std::make_shared<MapStorage>();
    std::cout << "Creating executor" << std::endl;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::cout << "Adding node to executor" << std::endl;
    executor.add_node(node);
    std::cout << "Spinning" << std::endl;
    executor.spin();

    return 0;
}