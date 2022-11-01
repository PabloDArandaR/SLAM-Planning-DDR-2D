#include "map_generation/occupancy_map.hpp"
#include <algorithm>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup the map constructors

occupancyMap::occupancyMap(){}

occupancyMap::occupancyMap(float map_width, float map_height, float resolution, int origin_x, int origin_y, float max_range, float min_range, float max_angle, float min_angle, float angle_increment, float initial_probability, float true_positive, float true_negative){
    this->setupMap(map_width, map_height);
    this->setupResolution(resolution);
    this->setupOrigin(origin_x, origin_y);
    this->setupSensorRange(min_range, max_range);
    this->setupSensorAngle(min_angle, max_angle);
    this->setupSensorAngleIncrement(angle_increment);
    this->setupProbabilities(initial_probability, true_positive, true_negative);
}

void occupancyMap::setupMap(int map_width, int map_height)
{
    this->map_width = map_width;
    this->map_height = map_height;
}
void occupancyMap::setupResolution(float resolution)
{
    this->resolution = resolution;
}
void occupancyMap::setupOrigin(int origin_x, int origin_y)
{
    this->origin_x = origin_x;
    this->origin_y = origin_y;
}
void occupancyMap::setupSensorRange(float min_range, float max_range)
{
    this->max_range = max_range;
    this->min_range = min_range;
}
void occupancyMap::setupSensorAngle(float min_angle, float max_angle)
{
    this->max_angle = max_angle;
    this->min_angle = min_angle;
}
void occupancyMap::setupSensorAngleIncrement(float angle_increment)
{
    this->angle_increment = angle_increment;
}
void occupancyMap::initializeVectors(){
    this->min_range_points.resize((int)(1 + (this->max_angle - this->min_angle)/this->angle_increment));
    this->max_range_points.resize((int)(1 + (this->max_angle - this->min_angle)/this->angle_increment));
    this->angles.resize((int)(1 + (this->max_angle - this->min_angle)/this->angle_increment));
    for (int i = 0; i < (int)(1 + (this->max_angle - this->min_angle)/this->angle_increment); i++){
        this->angles[i] = this->min_angle + this->angle_increment*i;

        this->min_range_points[i](0) = this->min_range*cos(this->angles[i]);
        this->min_range_points[i](1) = - this->min_range*sin(this->angles[i]);
        this->min_range_points[i](2) = 0;
        this->min_range_points[i](3) = 1;

        this->max_range_points[i](0) = this->max_range*cos(this->angles[i]);
        this->max_range_points[i](1) = - this->max_range*sin(this->angles[i]);
        this->max_range_points[i](2) = 0;
        this->max_range_points[i](3) = 1;
    }
}
void occupancyMap::setupProbabilities(float initial_probability, float true_positive, float true_negative)
{
    this->true_positive = true_positive;
    this->true_negative = true_negative;
    std::cout << "Size of the probability map (widht, height): " << this->map_width << ", " << this->map_height << std::endl;
    this->probability_map = new std::vector<float>(this->map_height*this->map_width, initial_probability);
    this->int_probability_map = new std::vector<int8_t>(this->map_height*this->map_width, (int8_t)(initial_probability*100));

}

// Auxiliar functions

float inline occupancyMap::intersection(float b, float m, float query){
    return m*query + b;
}

int inline occupancyMap::getIndex(int x, int y){
    return this->map_width*x + y;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Updating map

void occupancyMap::updateMap(geometry_msgs::msg::TransformStamped laser_scanner_pose, sensor_msgs::msg::LaserScan::SharedPtr measurements){
    bool endpoint_occupied {false};
    vectorType start_pos_vector, end_pos_vector;
    matrix4 T;

    // Iterate through each laser ray
    for (unsigned i = 0; i <= sizeof(measurements->ranges)/sizeof(measurements->ranges[0]); i++){
        // Check if the endpoint of that ray is occupied or not
        endpoint_occupied = !((measurements->ranges[i] > this->max_range) | (measurements->ranges[i] < this->min_range));

        // Take start and end points of the ray
        start_pos_vector = this->min_range_points[i];
        // If the endpoint is occupied then the enpoint needs to be calculated and not taken from the stored
        if (endpoint_occupied) {
            end_pos_vector(0) = measurements->ranges[i]*cos(this->angles[i]);
            end_pos_vector(1) = measurements->ranges[i]*sin(this->angles[i]);
            end_pos_vector(2) = 0;
            end_pos_vector(3) = 1;
        }
        else{
            end_pos_vector = this->max_range_points[i];
        }

        // Transform the points to the global (map) frame
        this->transformation(laser_scanner_pose.transform.rotation, laser_scanner_pose.transform.translation, &T);
        start_pos_vector = T*start_pos_vector;
        end_pos_vector = T*end_pos_vector;

        this->fillPoints(start_pos_vector, end_pos_vector, endpoint_occupied);
    }
}

void occupancyMap::fillPoints(vectorType start, vectorType end, bool endpoint_occupied){
    double x_start {start(0)}, y_start {start(1)}, x_end {end(0)}, y_end {end(1)};
    double m {(y_end - y_start)/((x_end - x_start) * this->resolution)}, b {(y_end/this->resolution - m*x_end)};
    int x_direction = (x_start < x_end) ? 1 : -1;
    //int y_direction = (y_start < y_end) ? 1 : -1;

    // Find the corners that delimit the starting and ending cells in the grid
    // TODO: initialize based on the center
    int corner_x_start = (int)(x_start/this->resolution + this->origin_x);
    int corner_x_end = (int)(x_end/this->resolution + this->origin_x);
    int corner_y_end = (int)(y_end/this->resolution + this->origin_y);
    int corner_y_start = (int)(y_start/this->resolution + this->origin_y);

    int n_elements {abs((int)ceil(std::fmax<double>(corner_x_start, corner_x_end)) -
                        (int)floor(std::fmin<double>(corner_x_start, corner_x_end)))};
    std::vector<double> x_vals(n_elements, 0.0f);
    x_vals[0] = corner_x_start; x_vals[n_elements-1] = corner_x_end;
    std::vector<double> y_vals(n_elements, 0.0f);
    y_vals[0] = corner_y_start; y_vals[n_elements-1] = corner_y_end;

    int loop_start = corner_x_end > corner_x_start ? ceil(corner_x_start) : floor(corner_x_start);
    int loop_end = corner_x_end > corner_x_start ? floor(corner_x_end) : ceil(corner_x_end);
    int count{0};
    for (int i = loop_start ; i != loop_end; i = i + x_direction){
        x_vals[count + 1] = i;
        y_vals[count + 1] = this->intersection(b, m, i);
    }
    
    // Fill the map with non occupied points and occipied points (until the last - 1 point of x)
    for (int i = 0; i < n_elements - 2; i ++){
        for (int j = floor(y_vals[i]); j <= floor(y_vals[i + 1]); j++){
            this->addNonOccupiedOdd(this->getIndex(floor(x_vals[i]), j));
        }
    }
    if(endpoint_occupied){
        for (int j = floor(y_vals[n_elements-2]); j <= floor(y_vals[n_elements-1]) - 1; j++) {
            this->addNonOccupiedOdd(this->getIndex(floor(x_vals[n_elements - 1]), j));
        }
        this->addOccupiedOdd(this->getIndex(floor(x_vals[n_elements-1]), floor(y_vals[n_elements-1])));
    }
    else{
        for (int j = floor(y_vals[n_elements-2]); j <= floor(y_vals[n_elements-1]); j++) {
            this->addNonOccupiedOdd(this->getIndex(floor(x_vals[n_elements - 1]), j));
        }
    }
}

void occupancyMap::addNonOccupiedOdd(int index){
    this->probability_map->at(index) =  ((1 - this->true_positive)*this->probability_map->at(index))/
                                        ((1 - this->true_positive)*this->probability_map->at(index)
                                        + this->true_negative*(1 - this->probability_map->at(index)));
}

void occupancyMap::addOccupiedOdd(int index){
    this->probability_map->at(index) =  (this->true_positive*this->probability_map->at(index))/
                                        (this->true_positive*this->probability_map->at(index)
                                        + (1 - this->true_negative)*(1 - this->probability_map->at(index)));
}

void occupancyMap::transformation(geometry_msgs::msg::Quaternion q, geometry_msgs::msg::Vector3 t, matrix4 * output){
    matrix4 T = matrix4::Zero();

    // Rotation
    T(0,0) = 2*(q.w*q.w + q.x*q.x) - 1;
    T(1,1) = 2*(q.w*q.w + q.y*q.y) - 1;
    T(2,2) = 2*(q.w*q.w + q.z*q.z) - 1;
    T(0,1) = 2*(q.x*q.y - q.w*q.z);
    T(0,2) = 2*(q.x*q.z + q.w*q.y);
    T(1,2) = 2*(q.z*q.y - q.w*q.x);
    T(1,0) = 2*(q.x*q.y + q.w*q.z);
    T(2,0) = 2*(q.x*q.z - q.w*q.y);
    T(2,1) = 2*(q.z*q.y + q.w*q.x);

    // Translation
    T(0,3) = t.x;
    T(1,3) = t.y;
    T(2,3) = t.z;
    T(3,3) = 1;

    *output = T;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Retrieve data
std::vector<float> * occupancyMap::getMap(){
    return this->probability_map;
}


std::vector<int8_t> * occupancyMap::getProbMap(){
    return this->int_probability_map;
} 