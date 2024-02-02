cmake_minimum_required(VERSION 3.5)
project(mapping_methods)

include_directories(include)

find_package(Eigen3)
set(HEADERS include/mapping/mapping.hpp include/mapping/no_noise.hpp include/mapping/occupancy_grid.hpp include/mapping/scanner_metadata.hpp
            include/math/grid.hpp include/math/linalg.hpp
            include/types.hpp)

add_executable(debug_case1 src/main.cpp
                src/mapping/no_noise.cpp src/mapping/occupancy_grid.cpp src/mapping/scanner_metadata.cpp src/mapping/mapping.cpp
                src/math/linalg.cpp src/math/grid.cpp)

target_include_directories(debug_case1 PRIVATE include)

add_executable(debug_case2 src/main2.cpp
                src/mapping/no_noise.cpp src/mapping/occupancy_grid.cpp src/mapping/scanner_metadata.cpp src/mapping/mapping.cpp
                src/math/linalg.cpp src/math/grid.cpp
                ${HEADERS})

add_executable(recorded src/recorded_data.cpp
                src/mapping/no_noise.cpp src/mapping/occupancy_grid.cpp src/mapping/scanner_metadata.cpp src/mapping/mapping.cpp
                src/math/linalg.cpp src/math/grid.cpp
                ${HEADERS})

target_link_libraries(debug_case1 Eigen3::Eigen)
target_link_libraries(debug_case2 Eigen3::Eigen)
target_link_libraries(recorded Eigen3::Eigen)

message("${CMAKE_MODULE_PATH}")