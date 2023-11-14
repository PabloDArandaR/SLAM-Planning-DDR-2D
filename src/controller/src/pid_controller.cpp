#include "src/controller/src/aux.cpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class pid_controller : public rclcpp::Node {
  public:
    pid_controller() : rclcpp::Node("pid_controller") {
        this->int_p = 0;
        this->int_beta = 0;
        this->current_p = 0;
        this->current_beta = 0;
        this->previous_p = 0;
        this->previous_beta = 0;
        this->Kp_p = this->get_parameter("Kp_p").get_parameter_value().get<float>();
        this->Ki_p = this->get_parameter("Ki_p").get_parameter_value().get<float>();
        this->Kd_p = this->get_parameter("Kd_p").get_parameter_value().get<float>();
        this->Kp_beta = this->get_parameter("Kp_beta").get_parameter_value().get<float>();
        this->Ki_beta = this->get_parameter("Ki_beta").get_parameter_value().get<float>();
        this->Kd_beta = this->get_parameter("Kd_beta").get_parameter_value().get<float>();
        this->t_set = false;

        // Message for speed
        this->cmd_vel.angular.x = 0.f;
        this->cmd_vel.angular.y = 0.f;
        this->cmd_vel.angular.z = 0.f;
        this->cmd_vel.linear.x = 0.f;
        this->cmd_vel.linear.y = 0.f;
        this->cmd_vel.linear.z = 0.f;

        // Declare subscribers and publishers
        this->cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        this->ref_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ref", 10, std::bind(&pid_controller::update_ref, this, _1));
        this->current_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose", 10, std::bind(&pid_controller::update_vel, this, _1));
    }

  private:
    // Vars
    float Kp_p, Ki_p, Kd_p, Kp_beta, Ki_beta, Kd_beta;
    float int_p, int_beta, current_p, current_beta, previous_p, previous_beta, der_p, der_beta;
    float t_current, t_previous, delta_t;
    bool t_set;
    geometry_msgs::msg::PoseStamped current, reference;
    geometry_msgs::msg::Twist cmd_vel;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_subscription, current_subscription;

    // Private func
    void update_ref(const geometry_msgs::msg::PoseStamped::SharedPtr msg) { this->reference = *msg; }

    void update_output(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->current = *msg;
        this->update_vel();
        this->cmd_vel_publisher->publish(this->cmd_vel);
    }

    void update_vel() {
        // Time update
        if (!this->t_set) {
            this->t_current = this->current.header.stamp.sec + this->current.header.stamp.nanosec * 10e-9;
            this->t_previous = this->t_current;
        } else {
            this->t_previous = this->t_current;
            this->t_current = this->current.header.stamp.sec + this->current.header.stamp.nanosec * 10e-9;
        }
        this->delta_t = this->t_current - this->t_previous;

        // Proportional
        this->previous_p = this->current_p;
        this->current_p = sqrt(pow(this->reference.pose.position.x - this->current.pose.position.x, 2) +
                               pow(this->reference.pose.position.y - this->current.pose.position.y, 2));
        this->previous_beta = current_beta;
        this->current_beta = this->reference.pose.orientation.z - this->current.pose.orientation.z;
        correctAngle<float>(&this->current_beta);

        // Integration
        this->int_p +=
            sign<float>(M_PI / 2 - abs(this->current_beta)) * (this->current_p - this->previous_p) * this->delta_t;
        this->int_beta += (this->current_beta - this->previous_beta) * this->delta_t;

        // Derivative
        this->der_p = (this->current_p - this->previous_p) / this->delta_t;
        this->der_beta = (this->current_beta - this->previous_beta) / this->delta_t;

        // Orientation parameters update
        this->cmd_vel.linear.x = this->Kp_p * current_p + this->Ki_p * this->int_p + this->Kd_p * this->der_p;
        this->cmd_vel.angular.z =
            this->Kp_beta * current_beta + this->Ki_beta * this->int_beta + this->Kd_beta * this->der_beta;
    }
};