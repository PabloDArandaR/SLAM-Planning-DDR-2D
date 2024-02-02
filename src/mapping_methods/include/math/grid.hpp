#include <list>
#include <utility>

#include "types.hpp"

namespace grid {
std::tuple<float, float> line_parameters(Point, Point);

std::list<Cell> crossing_line_cells(Point, Point);
} // namespace grid