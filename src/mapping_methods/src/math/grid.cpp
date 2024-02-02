#include <cmath>
#include <iostream>

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

    // Find the best direction of evaluation (x or y)
    float deltaX{ending_point.x - starting_point.x}, deltaY(ending_point.y - starting_point.y);
    bool evaluate_rows = abs(deltaX) > abs(deltaY);

    // Direction in which each axis is moving
    int8_t direction_x = deltaX >= 0 ? 1 : -1;
    int8_t direction_y = deltaY >= 0 ? 1 : -1;
    int8_t direction_main = evaluate_rows ? direction_y : direction_x;

    // Initialize the pivot depending on the direction in which the line is going
    int current_pivot_iteration {};
    if (evaluate_rows){
        if (direction_y == 1){
            current_pivot_iteration = starting_cell.y + direction_y;
        }
        else {
            current_pivot_iteration = starting_cell.y;
        }
    }
    else {
        if (direction_x == 1){
            current_pivot_iteration = starting_cell.x + direction_x;
        }
        else {
            current_pivot_iteration = starting_cell.x;
        }
    }
    //int current_pivot_iteration = evaluate_rows ? starting_cell.y : starting_cell.x - direction_x;

    // Cell limit is the limit value that we need to check when evaluating the cells (it represents the limit value in
    // which the iteration is going to run through the minimum checks direction)

    // TODO: evaluate if the iteration limit depends on the direction (positive or negative)
    int iteration_limit {};
    if (evaluate_rows){
        if (direction_y == 1){
            iteration_limit = ending_cell.y + direction_y;
        }
        else {
            iteration_limit = ending_cell.y;
        }
    }
    else {
        if (direction_x == 1){
            iteration_limit = ending_cell.x + direction_x;
        }
        else {
            iteration_limit = ending_cell.x;
        }
    }
    //int iteration_limit = evaluate_rows ? ending_cell.y + direction_y: ending_cell.x + direction_x;
    int final_limit = evaluate_rows ? ending_cell.x : ending_cell.y;

    Cell current_cell(starting_cell);
    std::list<Cell> output;
    int i {0};
    //std::cout << "Iterating through " << (evaluate_rows ? "y" : "x") << std::endl;
    while (current_pivot_iteration != iteration_limit) {
        int intersection{0};
        i++;
        if (i > 100){
            std::cout << "Hey" << std::endl;
            std::cout << "iteration_limit: " << iteration_limit << std::endl;
            std::cout << "final_limit: " << final_limit << std::endl;
            std::cout << "current_pivot_iteration: " << std::to_string(current_pivot_iteration) << std::endl;
            std::cout << "starting_cell.x: " << starting_cell.x << std::endl;
            std::cout << "starting_cell.y: " << starting_cell.y << std::endl;
            std::cout << "ending_cell.x: " << ending_cell.x << std::endl;
            std::cout << "ending_cell.y: " << ending_cell.y << std::endl;
        }
        if (evaluate_rows) {
            intersection = (current_pivot_iteration - b) / m;

            for (int i{current_cell.x}; i != intersection + direction_x; i += direction_x) {
                Cell new_cell(i, current_cell.y);
                output.push_back(new_cell);
            }

            // Update for next iteration
            current_cell.x = intersection;
            current_cell.y = current_pivot_iteration;
            current_pivot_iteration += direction_main;
        } else {
            intersection = floor(m * current_pivot_iteration + b);

            for (int i{current_cell.y}; i != intersection + direction_y; i += direction_y) {
                Cell new_cell(current_cell.x, i);
                output.push_back(new_cell);
            }
            
            // Update for next iteration
            current_cell.x = current_pivot_iteration;
            current_cell.y = intersection;
            current_pivot_iteration += direction_main;
        }
    }

    // Evaluate last column/row independently
    if (evaluate_rows) {
        for (int i{current_cell.x}; i != final_limit + direction_x; i += direction_x) {
            Cell new_cell(i, current_cell.y);
            output.push_back(new_cell);
        }
    } else {
        for (int i{current_cell.y}; i != final_limit + direction_y; i += direction_y) {
            Cell new_cell(current_cell.x, i);
            output.push_back(new_cell);
        }
    }

    return output;
};