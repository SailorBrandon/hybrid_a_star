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
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES hybrid_a_star
 CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf costmap_2d
)

find_package(ompl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hybrid_a_star/hybrid_a_star.cpp
  src/hybrid_a_star/tf_broadcaster.cpp
  src/hybrid_a_star/path.cpp
  src/hybrid_a_star/node3d.cpp
  src/hybrid_a_star/space.cpp
  src/hybrid_a_star/reeds_shepp.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/hybrid_a_star_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  scripts/try_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
