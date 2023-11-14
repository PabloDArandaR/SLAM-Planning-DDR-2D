#include "map_generation/occupancy_map.hpp"
#include "map_generation/math_aux.hpp"
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup the map constructors

occupancyMap::occupancyMap() {}

occupancyMap::occupancyMap(float map_width, float map_height, float resolution, int origin_x, int origin_y,
                           float max_range, float min_range, float max_angle, float min_angle, float angle_increment,
                           float initial_probability, float true_positive, float true_negative) {
    this->setupMap(map_width, map_height);
    this->setupResolution(resolution);
    this->setupOrigin(origin_x, origin_y);
    this->setupSensorRange(min_range, max_range);
    this->setupSensorAngle(min_angle, max_angle);
    this->setupSensorAngleIncrement(angle_increment);
    this->setupProbabilities(initial_probability, true_positive, true_negative);
}

void occupancyMap::setupMap(int map_width, int map_height) {
    this->map_width = map_width;
    this->map_height = map_height;
}

void occupancyMap::setupResolution(float resolution) { this->resolution = resolution; }

void occupancyMap::setupOrigin(int origin_x, int origin_y) {
    this->origin_x = origin_x;
    this->origin_y = origin_y;
}

void occupancyMap::setupSensorRange(float min_range, float max_range) {
    this->max_range = max_range;
    this->min_range = min_range;
}

void occupancyMap::setupSensorAngle(float min_angle, float max_angle) {
    this->max_angle = max_angle;
    this->min_angle = min_angle;
}

void occupancyMap::setupSensorAngleIncrement(float angle_increment) {
    this->angle_increment = angle_increment;
    this->n_rays = (this->max_angle - this->min_angle) / this->angle_increment + 1;
}

void occupancyMap::initializeVectors() {}

void occupancyMap::setupProbabilities(float initial_probability, float true_positive, float true_negative) {
    this->true_positive = true_positive;
    this->true_negative = true_negative;
    this->probability_map = new std::vector<float>(this->map_height * this->map_width, initial_probability);
    this->int_probability_map =
        new std::vector<int8_t>(this->map_height * this->map_width, (int8_t)(initial_probability * 100));
}

// Auxiliar functions

float inline occupancyMap::intersection(float b, float m, float query) { return m * query + b; }

int inline occupancyMap::getIndex(double x, double y) { return this->map_width * x + y; }

/*
    Updating map
*/
void occupancyMap::updateMap(geometry_msgs::msg::TransformStamped laser_scanner_to_map,
                             sensor_msgs::msg::LaserScan::SharedPtr measurements) {
    vectorType start_pos_vector, end_pos_vector;
    matrix4 T;

    // TODO initialize angle
    float angle{measurements->angle_max};

    // Iterate through each laser ray

    for (unsigned i = 0; i <= measurements->ranges.size(); i++) {
        std::cout << "------------------------------------------------------------------" << std::endl;

        // Initialize some variables to be used during the iteration
        double m{0.f}, b{0.f};
        double x_corner_start{0.f}, x_corner_end{0.f};
        double y_corner_start{0.f}, y_corner_end{0.f};
        double x_end{0.f}, x_start{0.f}, x_eval{0.f}, x_eval_prev{0.f};
        double y_end{0.f}, y_start{0.f}, y_eval{0.f} /*,  y_eval_prev{0.f} */;
        int dir_x{0}, dir_y{0};
        int number_of_y_crosses{0};
        bool endpoint_occupied{true};

        std::cout << "Setup hehe" << std::endl;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Identify the location of the starting and ending point in the map region

        // Find the endpoint (check if it is the value or it is out of range)
        start_pos_vector(0) = measurements->range_min * cos(angle);
        start_pos_vector(1) = measurements->range_min * sin(angle);
        start_pos_vector(0) = 0;
        start_pos_vector(0) = 1;

        if (!((measurements->ranges[i] > this->max_range) | (measurements->ranges[i] < this->min_range))) {
            end_pos_vector(0) = measurements->ranges[i] * cos(angle);
            end_pos_vector(1) = measurements->ranges[i] * sin(angle);
            end_pos_vector(2) = 0;
            end_pos_vector(3) = 1;
        } else {
            end_pos_vector(0) = measurements->range_max * cos(angle);
            end_pos_vector(1) = measurements->range_max * sin(angle);
            end_pos_vector(2) = 0;
            end_pos_vector(3) = 1;
        }

        std::cout << "The value of range is: " << measurements->ranges[i] << std::endl;
        std::cout << "The number of values is: " << measurements->ranges.size() << std::endl;
        std::cout << "The location of the endpoint is:" << end_pos_vector(0) << ", " << end_pos_vector(1) << std::endl;
        std::cout << "The value of i is: " << i << std::endl;
        std::cout << "The value of the angles is:" << angle << std::endl;

        // Transform the points to the global (map) frame
        this->transformation(laser_scanner_to_map.transform.rotation, laser_scanner_to_map.transform.translation, &T);
        start_pos_vector = T * start_pos_vector;
        end_pos_vector = T * end_pos_vector;
        std::cout << "The transform is: " << T << std::endl;

        // Convert points with the resolution to the real space
        x_start = start_pos_vector(0) * this->resolution;
        y_start = start_pos_vector(1) * this->resolution;
        x_end = end_pos_vector(0) * this->resolution;
        y_end = end_pos_vector(1) * this->resolution;

        if ((floor(x_start) == floor(x_end)) & (floor(x_start) == floor(x_end))) {
            std::cout << "Here we are" << std::endl;
            this->addOccupiedOdd(this->getIndex(floor(x_start), floor(y_start)));
            continue;
        }

        // Check if point at the end is occupied
        endpoint_occupied = false ? measurements->ranges[i] > max_range * 10 : true;

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Solve the line intersection problem

        // Line parameters
        m = (y_end - y_start) / (x_end - x_start);
        b = y_start - m * x_start;

        // Directions of the movement
        dir_x = direction(x_start, x_end);
        dir_y = direction(y_start, y_end);
        std::cout << "Directions are: " << dir_x << ", " << dir_y << std::endl;

        // Variables to track the starting and ending cells corners in the direction of the line
        x_corner_start = (dir_x) ? ceil(x_start) : floor(x_start);
        y_corner_start = (dir_y) ? ceil(y_start) : floor(y_start);
        x_corner_end = (dir_x) ? floor(x_start) : ceil(x_start);
        y_corner_end = (dir_y) ? floor(y_start) : ceil(y_start);

        // Define variables to track the progress
        x_eval = x_corner_start;
        x_eval_prev = x_start; // It is needed to not do extra operations when calculating the first interesection
        y_eval = floor(y_start);

        int n_rows{(int)abs(floor(x_corner_end) - floor(x_corner_start))};

        for (int j = 0; j < n_rows; j++) {
            number_of_y_crosses =
                abs(floor(this->intersection(b, m, x_eval) - floor(this->intersection(b, m, x_eval_prev))));
            for (int k = 0; k < number_of_y_crosses + 1; k++) {
                this->addNonOccupiedOdd(this->getIndex(x_eval_prev, y_eval));
                y_eval += dir_y;
            }

            x_eval_prev = x_eval;
            x_eval += dir_x;
            y_eval -= dir_y;
        }

        // Last iteration works differently
        number_of_y_crosses =
            abs(floor(this->intersection(b, m, x_eval) - floor(this->intersection(b, m, x_eval_prev))));
        for (int k = 0; k < number_of_y_crosses; k++) {
            this->addNonOccupiedOdd(this->getIndex(x_eval_prev, y_eval));
            y_eval += dir_y;
        }
        if (endpoint_occupied) {
            this->addOccupiedOdd(this->getIndex(x_eval_prev, y_eval));
            std::cout << "This is occupied! " << std::endl;
        }
        std::cout << "The index of the endoint is: " << this->getIndex(x_eval_prev, y_eval) << std::endl;
        std::cout << "The location of the endpoint is:" << end_pos_vector(0) << ", " << end_pos_vector(1) << std::endl;

        angle -= measurements->angle_increment;
    }
}

void occupancyMap::addNonOccupiedOdd(int index) {
    this->probability_map->at(index) = ((1 - this->true_positive) * this->probability_map->at(index)) /
                                       ((1 - this->true_positive) * this->probability_map->at(index) +
                                        this->true_negative * (1 - this->probability_map->at(index)));

    this->int_probability_map->at(index) = ((1 - this->true_positive) * this->int_probability_map->at(index)) /
                                           ((1 - this->true_positive) * this->int_probability_map->at(index) +
                                            this->true_negative * (1 - this->int_probability_map->at(index)));
}

void occupancyMap::addOccupiedOdd(int index) {
    this->probability_map->at(index) = (this->true_positive * this->probability_map->at(index)) /
                                       (this->true_positive * this->probability_map->at(index) +
                                        (1 - this->true_negative) * (1 - this->probability_map->at(index)));

    this->int_probability_map->at(index) = (this->true_positive * this->int_probability_map->at(index)) /
                                           (this->true_positive * this->int_probability_map->at(index) +
                                            (1 - this->true_negative) * (1 - this->int_probability_map->at(index)));
}

void occupancyMap::transformation(geometry_msgs::msg::Quaternion q, geometry_msgs::msg::Vector3 t, matrix4* output) {
    matrix4 T = matrix4::Zero();

    // Rotation
    T(0, 0) = 2 * (q.w * q.w + q.x * q.x) - 1;
    T(1, 1) = 2 * (q.w * q.w + q.y * q.y) - 1;
    T(2, 2) = 2 * (q.w * q.w + q.z * q.z) - 1;
    T(0, 1) = 2 * (q.x * q.y - q.w * q.z);
    T(0, 2) = 2 * (q.x * q.z + q.w * q.y);
    T(1, 2) = 2 * (q.z * q.y - q.w * q.x);
    T(1, 0) = 2 * (q.x * q.y + q.w * q.z);
    T(2, 0) = 2 * (q.x * q.z - q.w * q.y);
    T(2, 1) = 2 * (q.z * q.y + q.w * q.x);

    // Translation
    T(0, 3) = t.x;
    T(1, 3) = t.y;
    T(2, 3) = t.z;
    T(3, 3) = 1;

    *output = T;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieve data
std::vector<float>* occupancyMap::getMap() { return this->probability_map; }

std::vector<int8_t>* occupancyMap::getProbMap() { return this->int_probability_map; }