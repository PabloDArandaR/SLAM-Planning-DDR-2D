#include <cmath>

#include "math/grid.hpp"
#include "types.hpp"

std::tuple<float, float> grid::line_parameters(Point starting_point, Point ending_point) {
    float m{(starting_point.y - ending_point.y) / (starting_point.x - ending_point.x)};
    return std::tuple<float, float>(m, starting_point.y - m * starting_point.x);
};

std::list<Cell> grid::crossing_line_cells(Point starting_point, Point ending_point) {

    Cell ending_cell(floor(ending_point.x), floor(ending_point.y));
    Cell starting_cell(floor(starting_point.x), floor(starting_point.y));

    // Line equation: y = m*x + b
    float m, b;
    std::tie(m, b) = grid::line_parameters(starting_point, ending_point);

    // Setup cell search parameters
    bool check_x{true}; // If true, it evaluates the intersection with the x lines, false checks the y lines

    float deltaX{ending_point.x - starting_point.x}, deltaY(ending_point.y - starting_point.y);
    if (abs(deltaX) >= abs(deltaY)) {
        check_x = false;
    }

    // Direction in each of the axis
    int8_t direction_x{1}, direction_y{1};
    if (deltaX < 0) {
        direction_x = -1;
    }
    if (deltaY < 0) {
        direction_y = -1;
    }

    // Initialize the pivot depending on the direction in which the line is going
    int current_pivot{0};
    if (check_x) {
        if (direction_x > 0) {
            current_pivot = starting_cell.x + 1;
        } else {
            current_pivot = starting_cell.x;
        }
    } else {
        if (direction_y > 0) {
            current_pivot = starting_cell.y + 1;
        } else {
            current_pivot = starting_cell.y;
        }
    }

    // Cell limit is the limit value that we need to check when evaluating the cells (it represents the limit value in
    // which the iteration is going to run through the minimum checks direction)
    int* cell_limit = new int;
    int* iteration_limit = new int;
    if (check_x) {
        cell_limit = &ending_cell.x;
        iteration_limit = &ending_cell.y;
    } else {
        cell_limit = &ending_cell.y;
        iteration_limit = &ending_cell.x;
    }

    Cell current_cell(starting_cell);
    std::list<Cell> output;
    output.push_back(current_cell);
    while (current_pivot != *cell_limit) {
        int intersection{0};

        if (check_x) {
            intersection = floor(m * current_pivot + b);
            for (int i{current_cell.y}; i != intersection; i += direction_y) {
                Cell new_cell(current_pivot, i);
                output.push_back(new_cell);
            }
            output.push_back(Cell(current_pivot, intersection));
            current_pivot += direction_x;
        } else {
            intersection = (current_pivot - b) / m;
            for (int i{current_cell.x}; i != intersection; i += direction_x) {
                Cell new_cell(i, current_pivot);
                output.push_back(new_cell);
            }
            output.push_back(Cell(intersection, current_pivot));
            current_pivot += direction_y;
        }
    }

    // Evaluate last column/row independently
    if (check_x) {
        for (int i{current_cell.x}; i != *iteration_limit; i += direction_x) {
            Cell new_cell(i, current_pivot);
            output.push_back(new_cell);
        }
        output.push_back(Cell(current_pivot, *cell_limit));
        current_pivot += direction_x;
    } else {
        for (int i{current_cell.x}; i != *iteration_limit; i += direction_x) {
            Cell new_cell(i, current_pivot);
            output.push_back(new_cell);
        }
        output.push_back(Cell(*cell_limit, current_pivot));
        current_pivot += direction_y;
    }

    delete cell_limit, iteration_limit;

    return output;
};