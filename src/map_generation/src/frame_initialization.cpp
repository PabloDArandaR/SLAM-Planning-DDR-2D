#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class mapFrameIntializer : public rclcpp::Node {
    public:
        mapFrameIntializer() : rclcpp::Node("map_frame_initializer") {

            // Transforms
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // Map frame parameters
            this->declare_parameter("x", -10.0);
            this->declare_parameter("y", -10.0);
            this->declare_parameter("z", 0.0);
            this->declare_parameter("q0", 1.0);
            this->declare_parameter("qx", 0.0);
            this->declare_parameter("qy", 0.0);
            this->declare_parameter("qz", 0.0);
            this->declare_parameter("parent_frame", "odom");
            this->declare_parameter("child_frame", "map");

            // Define the transform
            geometry_msgs::msg::TransformStamped t;
            
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = this->get_parameter("parent_frame").as_string();
            t.child_frame_id = this->get_parameter("child_frame").as_string();

            t.transform.translation.x = this->get_parameter("x").as_double();
            t.transform.translation.y = this->get_parameter("y").as_double();
            t.transform.translation.z = this->get_parameter("z").as_double();
            t.transform.rotation.w = this->get_parameter("q0").as_double();
            t.transform.rotation.x = this->get_parameter("qx").as_double();
            t.transform.rotation.y = this->get_parameter("qy").as_double();
            t.transform.rotation.z = this->get_parameter("qz").as_double();

            RCLCPP_INFO(this->get_logger(), "Value of position is %f %f %f", this->get_parameter("x").as_double(), this->get_parameter("y").as_double(), this->get_parameter("z").as_double());
            RCLCPP_INFO(this->get_logger(), "Value of quaternion is %f %f %f %f", this->get_parameter("q0").as_double(), this->get_parameter("qx").as_double(), this->get_parameter("qy").as_double(), this->get_parameter("qz").as_double());

            this->tf_static_broadcaster_->sendTransform(t);
        }

        ~mapFrameIntializer(){};

    private:
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mapFrameIntializer>());
    rclcpp::shutdown();

    return 0;
}