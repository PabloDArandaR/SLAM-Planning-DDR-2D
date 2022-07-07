#include <iostream>
#include <cmath>


#define PI 3.14159265

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class poseNode: public rclcpp::Node{
    public:
        poseNode():
            Node("pose_handler"){
            // Ref pose
            float ref_angle = -PI/6;
            ref_pose.orientation.x = 0; ref_pose.orientation.y = 0; ref_pose.orientation.z = cos(ref_angle); ref_pose.orientation.x = sin(ref_angle);
            ref_pose.position.x = 1; ref_pose.position.y = 1; ref_pose.position.z = 0;
            ref_pose_stamped.pose.orientation.x = 0; ref_pose_stamped.pose.orientation.y = 0; ref_pose_stamped.pose.orientation.z = cos(ref_angle); ref_pose_stamped.pose.orientation.x = sin(ref_angle);
            ref_pose_stamped.pose.position.x = 1; ref_pose_stamped.pose.position.y = 1; ref_pose_stamped.pose.position.z = 0;
            //ref_pose_stamped.header->seq = 0;
            //ref_pose_stamped.frame_id = "Goal1";

            
            // Subscribers
            this->subscriber_poses = this->create_subscription<nav_msgs::msg::Odometry>("/p3d/odom", 10, std::bind(&poseNode::pose_callback, this, _1));
            this->publisher_goal = this->create_publisher<geometry_msgs::msg::Pose>("/goal", 10);
            this->publisher_current = this->create_publisher<geometry_msgs::msg::Pose>("/current_pose", 10);
        }
    private:
        ///////////////////////////////////////////////////////////////////////////////////
        // ROS sub/pub
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_poses; 
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_goal, publisher_current;

        ///////////////////////////////////////////////////////////////////////////////////
        // Messages
        geometry_msgs::msg::Pose ref_pose;
        geometry_msgs::msg::PoseStamped ref_pose_stamped;

        ///////////////////////////////////////////////////////////////////////////////////
        // Callback functions
        void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
            geometry_msgs::msg::Pose current_pose = msg->pose.pose;

            this->publisher_current->publish(current_pose);
            this->publisher_goal->publish(ref_pose);
        }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<poseNode>());
    rclcpp::shutdown();

    return 0;
}