#include <iostream>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PIDNode: public rclcpp::Node{
    public: 
        PIDNode(float Kp_lin,float Ki_lin,float Kd_lin,
                float Kp_alpha,float Ki_alpha,float Kd_alpha,
                float Kp_beta,float Ki_beta,float Kd_beta,
                float max_speed,float max_angular):
                Node("PID_controller"){
            // Controller parameters
            this->Kp_lin = Kp_lin;
            this->Ki_lin = Ki_lin;
            this->Kd_lin = Kd_lin;

            this->Kp_alpha = Kp_alpha;
            this->Ki_alpha = Ki_alpha;
            this->Kd_alpha = Kd_alpha;

            this->Kp_beta = Kp_beta;
            this->Ki_beta = Ki_beta;
            this->Kd_beta = Kd_beta;

            this->max_speed = max_speed;
            this->max_angular = max_angular;

            // Measurements
            this->t = std::chrono::high_resolution_clock::now();
            this->t_1 = this->t;
            this->theta = 0.0; this->beta_1 = 0.0; this->beta = 0.0; this->alpha = 0.0; this->alpha_1 = 0.0; this->rho_1 = 0.0; this->rho = 0.0; 
            this->int_rho = 0.0; this->int_alpha = 0.0; this->int_beta = 0.0; 
            this->time_diff = 0.0;

            // Commanded outputs
            this -> lin_vel = 0.0;
            this -> ang_vel = 0.0;

            // Diff drive control
            std::chrono::time_point<std::chrono::high_resolution_clock> t, t_1;
            this->time_diff = 0;

            // Subscribers
            this->subscriber_goal = this->create_subscription<geometry_msgs::msg::Pose>("/goal", 10, std::bind(&PIDNode::goal_callback, this, _1));
            this->subscriber_pose = this->create_subscription<geometry_msgs::msg::Pose>("/current_pose", 10, std::bind(&PIDNode::pose_callback, this, _1));

            // Publishers
            this->publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            this->publisher_lin = this->create_publisher<std_msgs::msg::Float64>("/lin", 10);
            this->publisher_ang = this->create_publisher<std_msgs::msg::Float64>("/ang", 10);

            RCLCPP_INFO(this->get_logger(), "Initializing node");
        }

    private:
        ///////////////////////////////////////////////////////////////////////////////////
        // ROS variables

        // Sub
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_goal, subscriber_pose;

        // Pub
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_lin, publisher_ang;

        // Timer
        rclcpp::TimerBase::SharedPtr timer_;

        // Storage variables
        geometry_msgs::msg::Pose current, current_1, ref;

        ///////////////////////////////////////////////////////////////////////////////////
        // Control variables

        // Controller constants
        double Kp_lin, Ki_lin, Kd_lin;
        double Kp_alpha, Ki_alpha, Kd_alpha;
        double Kp_beta, Ki_beta, Kd_beta;

        // Saturation
        double max_speed, max_angular;

        // Commanded outputs
        double lin_vel, ang_vel;

        // Diff drive control
        double theta, beta, beta_1, alpha, alpha_1, rho, rho_1;
        double int_rho, int_alpha, int_beta;
        std::chrono::time_point<std::chrono::high_resolution_clock> t, t_1;
        double time_diff;

        ///////////////////////////////////////////////////////////////////////////////////
        // Controller callback

        void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg){
            this->ref = *msg;
        }

        void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg){
            // Update time
            this->t_1 = this->t;
            this->t = std::chrono::high_resolution_clock::now();
            this->time_diff = std::chrono::duration_cast<std::chrono::seconds>(t - t_1).count();

            // Update poses
            this->current_1 = this->current;
            this->current = *msg;

            // Update aux;
            this->calculate_aux();

            // Calculate commands and send
            this->calculate_velocity_PI();
            this->saturate_speeds();
            this->send_velocity();
        }

        void vel_callback(){
        }

        ///////////////////////////////////////////////////////////////////////////////////
        // Aux functions

        float yaw_from_quat(geometry_msgs::msg::Quaternion input){
            double delta {acos(input.w)*2};
            if (abs(delta) < 1e-3){return 0;}
            else{return acos(input.z/sin(delta/2));}
        }

        void calculate_aux(){
            ////////////////////////////////////////////////////////////////
            // Angles
            this->theta = yaw_from_quat(ref.orientation);

            // Alpha
            this->alpha_1 = this->alpha;
            this->alpha = -this->theta + atan2(ref.position.y - current.position.y, ref.position.x - current.position.x);
            this->int_alpha += (this->alpha - this->alpha_1)*this->time_diff;

            // Beta
            this->beta_1 = this->beta;
            this->beta = -this->theta - this->alpha;
            this->int_beta += (this->beta - this->beta_1)*this->time_diff;

            ////////////////////////////////////////////////////////////////
            // Distance
            this->rho_1 = this->rho;
            this->rho = sqrt(pow(current.position.x - ref.position.x, 2) + pow(current.position.y - ref.position.y, 2) + pow(current.position.z - ref.position.z, 2));
            if (this->rho < 0.05){
                this->rho = 0;
            }
            this->int_rho += (this->rho - this->rho_1)*this->time_diff;
            RCLCPP_INFO(this->get_logger(), "Angles-> beta: '%f';      alpha: '%f';      theta: '%f';      time_diff: '%f'", this->beta, this->alpha, this->theta, this->time_diff);
        }

        void calculate_velocity_P(){
            this->lin_vel = this->Kp_lin*this->rho;
            this->ang_vel = this->Kp_alpha*this->alpha + this->Kp_beta*this->beta ;
            //this->ang_vel = -this->ang_vel;
        }

        void calculate_velocity_PI(){
            this->lin_vel = this->Kp_lin*this->rho + this->Ki_lin*this->int_rho;
            this->ang_vel = this->Kp_alpha*this->alpha + this->Kp_beta*this->beta + this->Ki_alpha*this->int_alpha + this->Ki_beta*this->int_beta ;
            //this->ang_vel = -this->ang_vel;
        }

        void calculate_velocity_PID(){
            this->lin_vel = this->Kp_lin*this->rho + this->Ki_lin*this->int_rho + this->Kd_lin*(this->rho - this->rho_1)/this->time_diff;
            this->ang_vel = this->Kp_alpha*this->alpha + this->Kp_beta*this->beta + this->Ki_alpha*this->int_alpha + this->Ki_beta*this->int_beta  + this->Kd_alpha*(this->alpha - this->alpha_1)/this->time_diff + this->Kd_beta*(this->beta - this->beta_1)/this->time_diff;
            //this->ang_vel = -this->ang_vel;
        }

        void send_velocity(){
            // MSGS variables
            geometry_msgs::msg::Twist msg_vel;
            std_msgs::msg::Float64 msg_aux;

            // Send the velocity variables
            msg_vel.linear.x = this->lin_vel;
            msg_vel.linear.y = 0.0;
            msg_vel.linear.z = 0.0;
            msg_vel.angular.x = 0.0;
            msg_vel.angular.y = 0.0;
            msg_vel.angular.z = this->ang_vel;
            this->publisher_vel->publish(msg_vel);

            // Send the auxiliar variables
            msg_aux.data = this->lin_vel;
            this->publisher_lin->publish(msg_aux);
            msg_aux.data = this->ang_vel;
            this->publisher_ang->publish(msg_aux);
            RCLCPP_INFO(this->get_logger(), "Linear velocity: '%f';    Angular velocity: '%f'", lin_vel, ang_vel);
        }

        void saturate_speeds(){

            if (this->lin_vel > this->max_speed){
                this->lin_vel = max_speed;
            }
            else if (this->lin_vel < -this->max_speed)
            {
                this->lin_vel = -this->max_speed;
            }

            if (this->ang_vel > this->max_angular){
                this->ang_vel = this->max_angular;
            }
            else if (this->ang_vel < -this->max_angular)
            {
                this->ang_vel = -this->max_angular;
            }
        }
};

int main(int argc, char ** argv){

    float   Kp_lin {0.2},       Ki_lin {0.1},     Kd_lin {1},
            Kp_alpha {0.3},     Ki_alpha {0.2},   Kd_alpha {1},
            Kp_beta {-0.2},     Ki_beta {-0.1},   Kd_beta {-1},
            max_speed {0.2},
            max_angular {0.5};

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDNode>(Kp_lin, Ki_lin, Kd_lin, Kp_alpha, Ki_alpha, Kd_alpha,Kp_beta, Ki_beta, Kd_beta,max_speed, max_angular));
    rclcpp::shutdown();

    return 0;
}