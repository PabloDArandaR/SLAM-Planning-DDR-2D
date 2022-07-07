#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "map_generation/occupancy_map.hpp"

#define LASER_MAX_RANGE 15
#define LASER_MIN_RANGE 0.05

int a = 10;

using std::placeholders::_1;

class MapGenerator: public rclcpp::Node
{
    public:
        MapGenerator(): rclcpp::Node("map_generator")
        {
            // Parameters for the node
            this->declare_parameter<float>("resolution", 0.1);
            this->declare_parameter<float>("width", 10);
            this->declare_parameter<float>("height", 10);

            // Map related parameters and publisher
            this->get_parameter("resolution", this->resolution);
            this->get_parameter("width", this->width);
            this->get_parameter("height", this->height);
            
            this->map_height = this->height/this->resolution;
            this->map_width = this->width/this->resolution;
            
            this->prob_map = occupancyMap();
            
            this->prob_map.setupMap(this->map_width, this->map_height);;
            this->prob_map.setupResolution(0.05f);
            this->prob_map.setupOrigin(0, 0);

            this->prob_map.setupSensorRange(0.05, 15);
            this->prob_map.setupSensorAngle(-3.141592, 3.141592);
            this->prob_map.setupSensorAngleIncrement(0.2);
            this->prob_map.initializeVectors();

            this->prob_map.setupProbabilities(0.5, 0.8, 0.9);

            this->publisher_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

            // Generate the transform from the map to the robot
            this->map_origin.position.x = 0.0; this->map_origin.position.y = 0.0; this->map_origin.position.z = 0.0;
            this->map_origin.orientation.x = 0.0; this->map_origin.orientation.y = 0.0; this->map_origin.orientation.z = 0.0; this->map_origin.orientation.w = 1.0;

            this->map_transform.transform.translation.x = 0.0; this->map_transform.transform.translation.y = 0.0; this->map_transform.transform.translation.z = 0.0;
            this->map_transform.transform.rotation.x = 0.0; this->map_transform.transform.rotation.y = 0.0; this->map_transform.transform.rotation.z = 0.0; this->map_transform.transform.rotation.w = 1.0;
            this->map_transform.header.stamp = this->now();
            this->map_transform.header.frame_id = "map";
            this->map_transform.child_frame_id = "body";

            this->map_metadata.resolution = this->resolution;
            this->map_metadata.width = this->width;
            this->map_metadata.height = this->height;
            this->map_metadata.origin = this->map_origin;

            // Publisher of transforms
            this->tf_publisher = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            this->tf_publisher->sendTransform(this->map_transform);

            // Add listener of transforms
            this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            std::cout << "List of frames:\n" << this->tf_buffer->allFramesAsString() << std::endl;

            // Subcribers
            this->subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("/out/laser", 10, std::bind(&MapGenerator::laser_callback, this, _1));

            RCLCPP_INFO(this->get_logger(), "ROS 2 logger");
        }

    private:

        // Vars
        float resolution, width, height, map_width, map_height;
        occupancyMap prob_map;
        geometry_msgs::msg::TransformStamped map_transform;
        geometry_msgs::msg::Pose map_origin;
        nav_msgs::msg::MapMetaData map_metadata;
        
        // Publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map;

        // tf related
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser;

        // Callback functions
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            geometry_msgs::msg::TransformStamped robot_pose = tf_buffer->lookupTransform("body", "map",tf2::TimePointZero);
        }

};

int main(int argc, char** argv){
    std::cout << "A is: " << a << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGenerator>());
    std::cout << "Shutting done" << std::endl;
    rclcpp::shutdown();
    return 0;
}