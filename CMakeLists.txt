cmake_minimum_required(VERSION 3.0.2)
project(vehicle_body)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL  REQUIRED COMPONENTS common io visualization segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(vehicle_body vehicle_body.cpp)
target_link_libraries(vehicle_body  ${VTK_LIBRARIES} 
${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} 
${PCL_VISUALIZATION_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})
