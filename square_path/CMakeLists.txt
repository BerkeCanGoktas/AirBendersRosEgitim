cmake_minimum_required(VERSION 3.0.2)
project(square_path)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 geometry_msgs
 mavros_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_ctrl
  CATKIN_DEPENDS roscpp geometry_msgs mavros_msgs
#  DEPENDS system_lib
)


include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(square_path src/square_path.cpp)
target_link_libraries(square_path ${catkin_LIBRARIES})
