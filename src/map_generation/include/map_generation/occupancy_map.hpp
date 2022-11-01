#include <vector>
#include <eigen3/Eigen/Eigen>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <math.h>

using vectorType = Eigen::Vector4d;
using matrix4 = Eigen::Matrix4d;
using matrix3 = Eigen::Matrix3d;

class occupancyMap{

    public:
        occupancyMap();
        occupancyMap(float map_width, float map_height, float resolution, int origin_x, int origin_y, float max_range, float min_range, float max_angle, float min_angle, float angle_increment, float initial_probability, float true_positive, float true_negative);
        
        // Setup functions:
        void setupMap(int map_width, int map_height);
        void setupResolution(float resolution);
        void setupOrigin(int origin_x, int origin_y);
        void setupSensorRange(float min_range, float max_range);
        void setupSensorAngle(float min_angle, float max_angle);
        void setupSensorAngleIncrement(float angle_increment);
        void initializeVectors();
        void setupProbabilities(float initial_probability, float true_positive, float true_negative);

        void updateMap(geometry_msgs::msg::TransformStamped robot_pose, sensor_msgs::msg::LaserScan::SharedPtr measurements);
        std::vector<float> * getMap();
        std::vector<int8_t> * getProbMap(); 

    private:
        float resolution, max_range, min_range, max_angle, min_angle, angle_increment, true_positive, true_negative;
        int map_width, map_height;
        int origin_x, origin_y;
        std::vector<float> * probability_map;
        std::vector<int8_t> * int_probability_map;
        std::vector<float> angles;
        std::vector<vectorType> min_range_points, max_range_points;

        // Processing functions
        void transformation(geometry_msgs::msg::Quaternion rotation, geometry_msgs::msg::Vector3, Eigen::Matrix4d * output);
        void fillPoints(vectorType start, vectorType end, bool endpoint);
        void addOccupiedOdd(int );
        void addNonOccupiedOdd(int );
        
};