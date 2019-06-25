cmake_minimum_required(VERSION 2.8.3)
project(demo_map_markers)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  demo_state_machine
  roscpp
  visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS demo_state_machine roscpp visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(demo_map_markers src/demo_map_markers.cpp src/main.cpp)
add_dependencies(demo_map_markers ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo_map_markers ${catkin_LIBRARIES})

