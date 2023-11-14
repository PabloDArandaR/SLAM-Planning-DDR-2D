#include <chrono>
#include <vector>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class DummyMapPublisher : public rclcpp::Node {
    public:
        DummyMapPublisher()
        : rclcpp::Node("dummy_map_publisher") {
            this->publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dummy_map", 10);
            this->publisher_update = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/dummy_map_updates", 10);
            this->timer_ = this -> create_wall_timer(
                100ms, std::bind(&DummyMapPublisher::timer_callback, this)
                );

            // Transform
            this -> tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            this -> publish_static_transforms();

            this -> generate_initial_map();
            this -> generate_initial_update_map();
            this -> send_initial_map();
        }
    
    private:
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
        rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr publisher_update;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        int counter_ {0};
        nav_msgs::msg::OccupancyGrid::SharedPtr dummy_message;
        map_msgs::msg::OccupancyGridUpdate::SharedPtr dummy_message_update;

        void timer_callback(){
            RCLCPP_INFO(this->get_logger(), "Updating and sending new message. Counter is");
            std::cout << "Counter is: " << this -> counter_ << std::endl;
            std::cout << "Value before editing is: " << this -> dummy_message_update ->  data [this->counter_ % 10000]  << std::endl;
            this -> dummy_message_update -> data [this->counter_ % 10000] = 100;
            std::cout << "Value after editing is: " << this -> dummy_message_update ->  data [this->counter_ % 10000]  << std::endl;
            this -> dummy_message_update -> header.stamp = this -> get_clock() -> now();
            this->counter_ ++;
            std::cout << "Checkpoint" << std::endl;
            this->publisher_update->publish(*this->dummy_message_update);
        }

        void send_initial_map(){
            this -> dummy_message -> header.stamp = this -> get_clock() -> now();
            this->publisher_->publish(*this->dummy_message);
        }

        void publish_static_transforms(){
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this -> get_clock() -> now();
            t.header.frame_id = "world";
            t.child_frame_id = "base";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            this -> tf_static_broadcaster_ -> sendTransform(t);
        }

        void generate_initial_map(){

            this -> dummy_message = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            this -> dummy_message -> data = std::vector<int8_t>(10000,0);
            this -> dummy_message -> info.resolution = 0.1;
            this -> dummy_message -> info.width = 100;
            this -> dummy_message -> info.height = 100;

            geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            pose.orientation.w = 0.0;
            pose.orientation.w = 0.0;
            pose.orientation.w = 0.0;
            this -> dummy_message -> info.origin = pose;

            this -> dummy_message -> header.frame_id = "world";
        }

        void generate_initial_update_map(){

            this -> dummy_message_update = std::make_shared<map_msgs::msg::OccupancyGridUpdate>();
            this -> dummy_message_update -> x = 0;
            this -> dummy_message_update -> y = 0;
            this -> dummy_message_update -> width = 100;
            this -> dummy_message_update -> height = 100;
            this -> dummy_message_update -> data = std::vector<int8_t>(10000,0);

            geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            pose.orientation.w = 0.0;
            pose.orientation.w = 0.0;
            pose.orientation.w = 0.0;
            this -> dummy_message -> info.origin = pose;
            this -> dummy_message -> header.frame_id = "world";
        }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyMapPublisher>());
    return 0;
}