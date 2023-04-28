cmake_minimum_required(VERSION 3.0.2)
project(hybrid_a_star)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  costmap_2d
  base_local_planner
  nav_core
  pluginlib
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hybrid_a_star
 CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf costmap_2d base_local_planner nav_core pluginlib
)

find_package(ompl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hybrid_a_star_lib
  src/hybrid_a_star/planner_manager.cpp
  src/hybrid_a_star/hybrid_a_star.cpp
  src/hybrid_a_star/node3d.cpp
  src/hybrid_a_star/reeds_shepp.cpp
)
add_dependencies(hybrid_a_star_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hybrid_a_star_lib
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/try_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
