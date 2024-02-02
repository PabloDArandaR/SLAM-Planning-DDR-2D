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

include_directories(../mapping_methods/src)
add_compile_options(-g)

# include additional files
set(HEADERS 
    ../mapping_methods/include/mapping/mapping.hpp 
    ../mapping_methods/include/mapping/no_noise.hpp 
    ../mapping_methods/include/mapping/occupancy_grid.hpp 
    ../mapping_methods/include/mapping/scanner_metadata.hpp 
    ../mapping_methods/include/math/grid.hpp 
    ../mapping_methods/include/math/linalg.hpp 
    ../mapping_methods/include/types.hpp)

include_directories(include)

include_directories(../mapping_methods/include)

add_executable(map_generation
    src/map_generation.cpp
    ${HEADERS})

add_executable(frame_initialization
    src/frame_initialization.cpp
    )

add_executable(laser_recorder
    src/laser_recorder.cpp
    )

add_executable(map_recorder
    src/map_recorder.cpp
    )

add_library(mapping_lib
    ../mapping_methods/src/mapping/mapping.cpp 
    ../mapping_methods/src/mapping/no_noise.cpp 
    ../mapping_methods/src/mapping/occupancy_grid.cpp
    ../mapping_methods/src/mapping/scanner_metadata.cpp
    ../mapping_methods/src/math/linalg.cpp
    ../mapping_methods/src/math/grid.cpp)

target_link_libraries(map_generation Eigen3::Eigen mapping_lib)

ament_target_dependencies(map_generation rclcpp tf2 tf2_ros tf2_eigen nav_msgs geometry_msgs sensor_msgs map_msgs)
ament_target_dependencies(frame_initialization rclcpp tf2 tf2_ros tf2_eigen)
ament_target_dependencies(laser_recorder rclcpp tf2 tf2_ros tf2_eigen nav_msgs geometry_msgs sensor_msgs map_msgs)
ament_target_dependencies(map_recorder rclcpp tf2 tf2_ros tf2_eigen nav_msgs geometry_msgs sensor_msgs map_msgs)

install(TARGETS
    map_generation
    frame_initialization
    laser_recorder
    map_recorder
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
      launch config
      DESTINATION share/${PROJECT_NAME})

# Tests
find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_linalg_test 
  test/test_linalg.cpp)

  
target_include_directories(${PROJECT_NAME}_linalg_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

target_link_libraries(${PROJECT_NAME}_linalg_test Eigen3::Eigen mapping_lib)

ament_package()
