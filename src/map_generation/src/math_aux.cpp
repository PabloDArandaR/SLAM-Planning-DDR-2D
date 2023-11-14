#include "map_generation/math_aux.hpp"

int direction(double start, double end) {
    if (start > end)
        return -1;
    else
        return 1;
};