cmake_minimum_required(VERSION 3.0.2)
project(maze_control)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(constant_twist src/constant_twist.cpp)
target_link_libraries(constant_twist ${catkin_LIBRARIES})