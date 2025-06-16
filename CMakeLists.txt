cmake_minimum_required(VERSION 3.0.2)
project(mpc_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  pluginlib
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
find_package(CasADi REQUIRED)
link_directories(/home/emir/casadi/build/lib)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_local_planner
#  CATKIN_DEPENDS costmap_2d geometry_msgs nav_core pluginlib roscpp tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CasADi_INCLUDE_DIRS}
)
include_directories(/home/emir/casadi/build/include)

add_library(mpc_local_planner
  src/mpc_local_planner.cpp
)

target_link_libraries(mpc_local_planner
  ${catkin_LIBRARIES}
  casadi
)

install(TARGETS mpc_local_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES mpc_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)