#include <chrono>
#include <iostream>
#include <fstream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/laser_scan.hpp"

class laserRecorder : public rclcpp::Node {
    public:
        laserRecorder() : rclcpp::Node("laser_recorder") {

            this->copy = false;

            // Transforms
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // Map frame parameters
            this->declare_parameter("map_frame", "map");
            this->declare_parameter("laser_frame", "laser_frame");
            this->declare_parameter("scan_topic", "/laser_scan");

            // Variable def
            this->map_frame = this->get_parameter("map_frame").as_string();
            this->laser_frame = this->get_parameter("laser_frame").as_string();
            this->filename = "redording.csv";
            this->file_output.open(this->filename, std::ios_base::app);

            // Subscription
            this->laser_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
                this->get_parameter("scan_topic").as_string(), 10, std::bind(&laserRecorder::laser_callback, this, std::placeholders::_1));
            this->copy_subscriber = this->create_subscription<std_msgs::msg::Bool>(
                "/record_data", 10, std::bind(&laserRecorder::copy_callback, this, std::placeholders::_1));
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber;
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

        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_laser){
            if (this->copy){
                rclcpp::Time timeout = rclcpp::Time(0);
                geometry_msgs::msg::TransformStamped t = 
                    this -> tf_buffer_->lookupTransform(this->map_frame, this->laser_frame, timeout);

                this->file_output << t.transform.translation.x << ", " << t.transform.translation.y << ", " << t.transform.translation.z << ", ";
                this->file_output << t.transform.rotation.w << ", " << t.transform.rotation.x << ", " << t.transform.rotation.y << ", " << t.transform.rotation.z;
                for (float val: msg_laser->ranges){
                    this->file_output << ", " << val;
                }
                this->file_output << std::endl;
            }
        }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laserRecorder>());
    rclcpp::shutdown();

    return 0;
}