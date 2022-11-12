cmake_minimum_required(VERSION 3.5)
project(map_generation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(map_msgs REQUIRED)

# include additional files
set(HEADERS include/map_generation/occupancy_map.hpp include/map_generation/math_aux.hpp)
include_directories(include)
add_executable(map_generation src/map_generation.cpp src/occupancy_map.cpp src/math_aux.cpp ${HEADERS})
target_link_libraries(map_generation Eigen3::Eigen)
ament_target_dependencies(map_generation rclcpp tf2 tf2_ros tf2_eigen nav_msgs geometry_msgs sensor_msgs map_msgs)

install(TARGETS
    map_generation
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
      launch config
      DESTINATION share/${PROJECT_NAME})

ament_package()
