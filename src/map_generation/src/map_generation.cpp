#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "mapping/no_noise.hpp"
#include "mapping/occupancy_grid.hpp"
#include "math/linalg.hpp"

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;

class mapGenerator : public rclcpp::Node {
  public:
    mapGenerator() : rclcpp::Node("map_generation") {

        RCLCPP_INFO(this->get_logger(), "Setup");
        // Set parameters
        this->declare_parameter("width", 150);
        this->declare_parameter("height", 150);
        this->declare_parameter("resolution", 0.1);
        this->declare_parameter("laser_frame", "laser_frame");
        this->declare_parameter("map_frame", "map");

        // Transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->laser_frame = this->get_parameter("laser_frame").as_string();
        this->map_frame = this->get_parameter("map_frame").as_string();

        // Occupancy grid shape definition
        this->width = this->get_parameter("width").as_int();
        this->height = this->get_parameter("height").as_int();
        this->resolution = this->get_parameter("resolution").as_double();
        this->map = occupancyGrid(this->width, this->height, 50, this->resolution);
        this->map_updater = noNoiseMethod(&this->map);

        scanerMetadata metadata;
        metadata.set_rangedata(0.3, 15);
        metadata.set_angledata((1.0+1.0)/150.0, -1, 1);
        this->map_updater.set_scaner_metadata(metadata);

        // Publisher definition
        RCLCPP_INFO(this->get_logger(), "Creeating publishers");
        this->publisher_initial_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        this->publisher_update_map = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/map_updates", 10);
        this->timer_map = this->create_wall_timer(1s, std::bind(&mapGenerator::timer_callback, this));

        // Subscriber definition
        RCLCPP_INFO(this->get_logger(), "Creating subscribers");
        this->subscriber_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_scan", 10, std::bind(&mapGenerator::laser_callback, this, std::placeholders::_1));

        // Map message setup
        RCLCPP_INFO(this->get_logger(), "Map message setup");
        this->map_initial_message = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        this->map_update_message = std::make_shared<map_msgs::msg::OccupancyGridUpdate>();
        this->generate_initial_map(this->map.get_map());
        this->generate_update_map(this->map.get_map());
        RCLCPP_INFO(this->get_logger(), "Initial messages generated and sending initial map");

        this->send_initial_map();
        RCLCPP_INFO(this->get_logger(), "Sending initial map");
    };

  private:
    int width, height; // cells
    float resolution;  // m/cells
    occupancyGrid map;
    noNoiseMethod map_updater;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_initial_map;
    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr publisher_update_map;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laser;

    rclcpp::TimerBase::SharedPtr timer_map;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_initial_message;
    map_msgs::msg::OccupancyGridUpdate::SharedPtr map_update_message;

    std::string laser_frame, map_frame;

    void timer_callback() {
        this->update_map_message();
        this->publisher_update_map->publish(*this->map_update_message);
        RCLCPP_INFO(this->get_logger(), "Published updated map");
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_laser) {
        RCLCPP_INFO(this->get_logger(), "Processing laser call");
        rclcpp::Time timeout = rclcpp::Time(0);
        geometry_msgs::msg::TransformStamped t =
            this -> tf_buffer_->lookupTransform(this->map_frame, this->laser_frame, timeout);
        Eigen::Vector3d position;
        Eigen::Vector4d quaternion;
        position << t.transform.translation.x, t.transform.translation.y, t.transform.translation.z;
        quaternion << t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z;
        this->map_updater.update(position, quaternion, msg_laser->ranges);

        auto map_update = map_msgs::msg::OccupancyGridUpdate();
        map_update.x = 0;
        map_update.y = 0;
        map_update.width = this->map.get_width();
        map_update.height = this->map.get_height();
        map_update.data = *(this->map_updater.get_map()->get_map());
        this->map_updater.get_map()->store_map("map_2.csv");
        map_update.header.frame_id = "map";
        this->publisher_update_map->publish(*this->map_update_message);
        RCLCPP_INFO(this->get_logger(), "Finished laser call");
    }

    void update_map_message() {
        this->map_update_message->data = *this->map.get_map();
        this->map_update_message->header.stamp = this->get_clock()->now();
    }

    void send_initial_map() {
        this->map_initial_message->header.stamp = this->get_clock()->now();
        this->publisher_initial_map->publish(*this->map_initial_message);
    }

    void generate_initial_map(std::vector<int8_t>* new_map) {

        this->map_initial_message->info.resolution = this->resolution;
        this->map_initial_message->info.width = this->width;
        this->map_initial_message->info.height = this->height;
        this->map_initial_message->data = *new_map;

        Eigen::Matrix3d map_rotation = linalg::rotationZ(0.0);

        Eigen::Vector4d quat = linalg::rotationToQuaternion(map_rotation);  

        geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.0;
        pose.orientation.w = quat.coeff(0);
        pose.orientation.w = quat.coeff(1);
        pose.orientation.w = quat.coeff(2);
        pose.orientation.w = quat.coeff(3);
        this->map_initial_message->info.origin = pose;

        this->map_initial_message->header.frame_id = "map";
    }

    void generate_update_map(std::vector<int8_t>* new_map) {
        this->map_update_message->x = 0;
        this->map_update_message->y = 0;
        this->map_update_message->width = this->map.get_width();
        this->map_update_message->height = this->map.get_height();
        this->map_update_message->data = *new_map;
        this->map_update_message->header.frame_id = "map";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mapGenerator>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}