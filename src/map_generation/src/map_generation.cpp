#include <memory>
#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
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
using namespace std::chrono;


class MapGenerator: public rclcpp::Node
{
    public:
        MapGenerator(): rclcpp::Node("map_generator")
        {
            //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Map parameters declaration

            // Declaring the parameters
            RCLCPP_INFO(this->get_logger(), "Declaring the parameters");
            this->declare_parameter("resolution", 0.01);
            this->declare_parameter("width", 10.0);
            this->declare_parameter("height", 10.0);

            // Map related parameters and publisher
            RCLCPP_INFO(this->get_logger(), "Gettinging the parameters");
            rclcpp::Parameter param_resolution = this->get_parameter("resolution");
            rclcpp::Parameter param_width = this->get_parameter("width");
            rclcpp::Parameter param_height = this->get_parameter("height");
            RCLCPP_INFO(this->get_logger(), "resolution: %s, width: %s, height: %s",
                param_resolution.value_to_string().c_str(),
                param_width.value_to_string().c_str(),
                param_height.value_to_string().c_str());

            RCLCPP_INFO(this->get_logger(), "converting parameters to values");
            this->resolution = (float)param_resolution.as_double();
            this->height = (float)param_height.as_double();
            this->width = (float)param_width.as_double();
            RCLCPP_INFO(this->get_logger(), "resolution: %s, width: %s, height: %s",
                param_resolution.value_to_string().c_str(),
                param_width.value_to_string().c_str(),
                param_height.value_to_string().c_str());

            
            this->map_height = this->height/this->resolution;
            this->map_width = this->width/this->resolution;
            

            //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Occupancy map declaration

            this->prob_map = occupancyMap();
            
            this->prob_map.setupMap(this->map_width, this->map_height);
            this->prob_map.setupResolution(0.05f);
            this->prob_map.setupOrigin(0, 0);

            this->prob_map.setupSensorRange(0.05, 15);
            this->prob_map.setupSensorAngle(-3.141592, 3.141592);
            this->prob_map.setupSensorAngleIncrement(0.2);
            this->prob_map.initializeVectors();

            this->prob_map.setupProbabilities(0, 0.8, 0.9);


            //////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Publishers and subscribers

            // Publishers
            this->publisher_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
            this->publisher_update = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>("/map_updates", 10);

            // Add listener of transforms
            this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            // Subcribers
            this->subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_controller/out", 10, std::bind(&MapGenerator::laser_callback, this, _1));

            // Timers
            this->timer_ = this->create_wall_timer(500ms, std::bind(&MapGenerator::send_callback, this));

            // Generate the transform from the map to the robot
            this->map_origin.position.x = 0.0; this->map_origin.position.y = 0.0; this->map_origin.position.z = 0.0;
            this->map_origin.orientation.x = 0.0; this->map_origin.orientation.y = 0.0; this->map_origin.orientation.z = 0.0; this->map_origin.orientation.w = 1.0;

            this->map_metadata.map_load_time = this->get_clock().get()->now();
            this->map_metadata.resolution = this->resolution;
            this->map_metadata.width = this->width;
            this->map_metadata.height = this->height;
            this->map_metadata.origin = this->map_origin;

            nav_msgs::msg::OccupancyGrid::SharedPtr initial_map = this->fillMapMessage();
            this->publisher_map->publish(*initial_map);

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
        rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr publisher_update;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // tf related
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser;

        // Callback functions
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Laser reading received");
            geometry_msgs::msg::TransformStamped robot_pose = tf_buffer->lookupTransform("world", "base_link",tf2::TimePointZero, 100ms);
            geometry_msgs::msg::TransformStamped robot_pose_2 = tf_buffer->lookupTransform("base_link", "body", tf2::TimePointZero, 100ms);
            //this->prob_map.updateMap(robot_pose, msg);
            
        }

        void send_callback(){
            map_msgs::msg::OccupancyGridUpdate::SharedPtr msg_out = this->fillUpdateMessage();
            RCLCPP_INFO(this->get_logger(), "Sending map");
            this->publisher_update->publish(*msg_out);
        }


        void send_initial(){
            nav_msgs::msg::OccupancyGrid::SharedPtr msg_out = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            RCLCPP_INFO(this->get_logger(), "Sending map");
            this->publisher_map->publish(*msg_out);
        }

        // Auxiliar functions
        map_msgs::msg::OccupancyGridUpdate::SharedPtr fillUpdateMessage(){
            map_msgs::msg::OccupancyGridUpdate::SharedPtr msg = std::make_shared<map_msgs::msg::OccupancyGridUpdate>();
            msg->width = this->width;
            msg->height = this->height;
            msg->x = 0;
            msg->y = 0;
            msg->data = *this->prob_map.getProbMap();
            msg->header.frame_id = "world";
            msg->header.stamp = this->get_clock().get()->now();
            return msg;
        }

        nav_msgs::msg::OccupancyGrid::SharedPtr fillMapMessage(){
            nav_msgs::msg::OccupancyGrid::SharedPtr msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
            msg->info = this->map_metadata;
            msg->data = *this->prob_map.getProbMap();
            msg->header.frame_id = "world";
            msg->header.stamp = this->get_clock().get()->now();
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