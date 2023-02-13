cmake_minimum_required(VERSION 3.0.2)
project(flight_info)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 geographic_msgs
 sensor_msgs
 mavros_msgs
 common_pkg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs geographic_msgs sensor_msgs common_pkg
  INCLUDE_DIRS include 
)

add_executable(flight_info src/flight_info.cpp)
target_link_libraries(flight_info ${catkin_LIBRARIES})