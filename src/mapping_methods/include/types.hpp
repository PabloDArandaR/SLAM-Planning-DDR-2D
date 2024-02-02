#include <eigen3/Eigen/Dense>

#ifndef TYPES_GUARD
#define TYPES_GUARD

struct Point {
    float x, y;

    Point() {
        this->x = 0;
        this->y = 0;
    }

    Point(float x, float y) {
        this->x = x;
        this->y = y;
    }
};

struct Cell {
    int x, y;

    Cell() {
        this->x = 0;
        this->y = 0;
    }

    Cell(float x, float y) {
        this->x = x;
        this->y = y;
    }
};

#endif