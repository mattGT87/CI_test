cmake_minimum_required(VERSION 2.8.3)
project(waypoint_writer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  roslib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waypoint_writer
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/waypoint_writer.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


