cmake_minimum_required(VERSION 3.0.2)
project(radar_unit)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_msgs
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(radar_unit_node src/radar_unit_node.cpp)

target_link_libraries(radar_unit_node
  ${catkin_LIBRARIES}
)