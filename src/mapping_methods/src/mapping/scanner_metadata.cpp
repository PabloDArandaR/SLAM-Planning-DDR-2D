#include "mapping/scanner_metadata.hpp"

scanerMetadata::scanerMetadata(){};

void scanerMetadata::set_angledata(float angle_increment, float angle_min, float angle_max) {
    this->angle_increment = angle_increment;
    this->angle_min = angle_min;
    this->angle_max = angle_max;
}

void scanerMetadata::set_rangedata(float range_min, float range_max) {
    this->range_min = range_min;
    this->range_max = range_max;
}

float scanerMetadata::get_angleincrement() { return this->angle_increment; };

float scanerMetadata::get_anglemin() { return this->angle_min; };

float scanerMetadata::get_anglemax() { return this->angle_max; };

float scanerMetadata::get_rangemin() { return this->range_min; };

float scanerMetadata::get_rangemax() { return this->range_max; };
