#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"

class mapRecorder : public rclcpp::Node {
    public:
        mapRecorder() : rclcpp::Node("map_recorder") {

            this->copy = false;

            // Transforms
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Map frame parameters
            this->declare_parameter("map_topic", "/map_updates");

            // Variable def
            this->filename = "map.csv";

            // Subscription
            this->map_subscriber = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
                this->get_parameter("map_topic").as_string(), 10, std::bind(&mapRecorder::map_callback, this, std::placeholders::_1));
            this->copy_subscriber = this->create_subscription<std_msgs::msg::Bool>(
                "/record_map", 10, std::bind(&mapRecorder::copy_callback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr map_subscriber;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr copy_subscriber;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::ofstream file_output;

        std::string laser_frame, map_frame, filename;

        bool copy;

        void initial_file_setup(){
            // Add config of file
        };

        void copy_callback(const std_msgs::msg::Bool msg_copy){
            this->copy = !this->copy;
        }

        void map_callback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg_grid){
            if (this->copy){
                rclcpp::Time timeout = rclcpp::Time(0);
                geometry_msgs::msg::TransformStamped t = 
                    this -> tf_buffer_->lookupTransform(this->map_frame, this->laser_frame, timeout);
                
                this->file_output.open(this->filename, std::ios_base::out);
                for (int i {0}; i < 300; ++i){
                    for (int j{0}; j < 299; ++j){
                        this->file_output << msg_grid->data[i*300 + j] << ", ";
                    }
                    this->file_output << msg_grid->data[i*300 + 299] << std::endl;
                }
            }
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mapRecorder>());
    rclcpp::shutdown();

    return 0;
}