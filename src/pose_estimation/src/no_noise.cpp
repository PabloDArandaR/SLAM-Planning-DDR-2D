#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class NoNoise: public rclcpp::Node
{
public:

    // Constructor
    NoNoise(): rclcpp::Node("no_noise"){
        this->odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&NoNoise::odom_callback, this, _1));
        this->tf_publisher = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:

    // Subcriptions and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        geometry_msgs::msg::TransformStamped dummy;
        dummy.header = msg->header;
        dummy.header.frame_id = "map_origin";
        dummy.child_frame_id = "body";
        dummy.transform.rotation = msg->pose.pose.orientation;
        dummy.transform.translation.x = msg->pose.pose.position.x;
        dummy.transform.translation.y = msg->pose.pose.position.y;
        dummy.transform.translation.z = msg->pose.pose.position.z;
        this->tf_publisher->sendTransform(dummy);
    }
};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NoNoise>());
    std::cout << "Shutting done" << std::endl;
    rclcpp::shutdown();
    return 0;
}