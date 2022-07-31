#include <memory>
#include <iostream>
#include <chrono>

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
using namespace std::chrono_literals;

class MapGenerator: public rclcpp::Node
{
    public:
        MapGenerator(): rclcpp::Node("map_generator")
        {
            std::cout << "Pre-declaration" << std::endl;
            this->declare_parameter("resolution");
            this->declare_parameter("width");
            this->declare_parameter("height");
            std::cout << "After declaration" << std::endl;

            // Map related parameters and publisher
            rclcpp::Parameter param_resolution = this->get_parameter("resolution");
            rclcpp::Parameter param_width = this->get_parameter("width");
            rclcpp::Parameter param_height = this->get_parameter("height");
            std::cout << "After getting the parameters" << std::endl;
            this->resolution = (float)param_resolution.as_double();
            this->height = (float)param_height.as_double();
            this->width = (float)param_width.as_double();
            std::cout << "After writing the parameters" << std::endl;
            RCLCPP_INFO(this->get_logger(), "resolution: %s, width: %s, height: %s",
                param_resolution.value_to_string().c_str(),
                param_width.value_to_string().c_str(),
                param_height.value_to_string().c_str());

            
            this->map_height = this->height/this->resolution;
            this->map_width = this->width/this->resolution;
            std::cout << "Input height: " << this->height << std::endl;
            std::cout << "Input width: " << this->width << std::endl;
            std::cout << "Input resolution: " << this->resolution << std::endl;
            
            this->prob_map = occupancyMap();
            
            this->prob_map.setupMap(this->map_width, this->map_height);
            this->prob_map.setupResolution(0.05f);
            this->prob_map.setupOrigin(0, 0);

            this->prob_map.setupSensorRange(0.05, 15);
            this->prob_map.setupSensorAngle(-3.141592, 3.141592);
            this->prob_map.setupSensorAngleIncrement(0.2);
            this->prob_map.initializeVectors();

            this->prob_map.setupProbabilities(0.5, 0.8, 0.9);

            // Map publisher
            this->publisher_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

            // Generate the transform from the map to the robot
            this->map_origin.position.x = 0.0; this->map_origin.position.y = 0.0; this->map_origin.position.z = 0.0;
            this->map_origin.orientation.x = 0.0; this->map_origin.orientation.y = 0.0; this->map_origin.orientation.z = 0.0; this->map_origin.orientation.w = 1.0;

            this->map_metadata.resolution = this->resolution;
            this->map_metadata.width = this->width;
            this->map_metadata.height = this->height;
            this->map_metadata.origin = this->map_origin;

            // Add listener of transforms
            this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Subcribers
            this->subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan/out", 10, std::bind(&MapGenerator::laser_callback, this, _1));

            std::cout << "Finished setting up." << std::endl;
        };

    private:

        // Vars
        float resolution, width, height;
        int map_width, map_height;
        occupancyMap prob_map;
        geometry_msgs::msg::Pose map_origin;
        nav_msgs::msg::MapMetaData map_metadata;
        
        // Publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map;

        // tf related
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser;

        // Callback functions
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            nav_msgs::msg::OccupancyGrid::SharedPtr msg_out = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            std::cout << "Laser scan received" << std::endl;
            geometry_msgs::msg::TransformStamped robot_pose = tf_buffer->lookupTransform("map_origin", "body",tf2::TimePointZero, 100ms);
            std::cout << "Updating map" << std::endl;
            this->prob_map.updateMap(robot_pose, msg);
            std::cout << "Filling message" << std::endl;
            msg_out->header.frame_id = "map_origin";
            std::cout << "Took the message" << std::endl;
            this->publisher_map->publish(*msg_out);
            std::cout << "Message sent" << std::endl;
        }

        // Auxiliar functions
        nav_msgs::msg::OccupancyGrid::SharedPtr fillMapMessage(){
            nav_msgs::msg::OccupancyGrid::SharedPtr msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            msg->header.frame_id = "map_origin";
            msg->info = this->map_metadata;
            msg->data = *this->prob_map.getProbMap();
            std::cout << "Returning" << std::endl;
            return msg;
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapGenerator>());
    std::cout << "Shutting done" << std::endl;
    rclcpp::shutdown();
    return 0;
}