cmake_minimum_required(VERSION 3.0.2)
project(minimum_snap_global_planner)

add_compile_options(-std=c++14 -Wall -O3 -march=native)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  global_planner
  rviz
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    minimum_snap_global_planner
    trajectory_generator_lib
    path_searcher_lib
    waypoint_tool
  CATKIN_DEPENDS
    roscpp
    nav_core
    costmap_2d
    pluginlib
    geometry_msgs
    nav_msgs
    tf
    tf2_ros
    global_planner
    rviz
    sensor_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(WAYPOINT_TOOL_MOC
  rviz_tools/waypoint_tool/src/waypoint_tool.h
)

add_library(trajectory_generator_lib
  src/trajectory/minimum_snap/minimum_snap.cpp
  src/trajectory/minimum_snap/minimum_snap_2d_flow.cpp
  src/trajectory/subscriber/path_subscriber.cpp
  src/trajectory/subscriber/pointcloud_subscriber.cpp
)
target_link_libraries(trajectory_generator_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(path_searcher_lib
  src/path_searcher/a_star/a_star_2d.cpp
  src/path_searcher/a_star/a_star_2d_flow.cpp
  src/path_searcher/subscriber/init_pose_subscriber.cpp
  src/path_searcher/subscriber/pointcloud_subscriber.cpp
)
target_link_libraries(path_searcher_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(waypoint_tool
  rviz_tools/waypoint_tool/src/waypoint_tool.cpp
  ${WAYPOINT_TOOL_MOC}
)
target_include_directories(waypoint_tool
  PRIVATE rviz_tools/waypoint_tool/src
)
target_link_libraries(waypoint_tool
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
)

add_library(minimum_snap_global_planner
  src/planner/minimum_snap_planner.cpp
)
target_link_libraries(minimum_snap_global_planner
  ${catkin_LIBRARIES}
  trajectory_generator_lib
)

install(TARGETS
  minimum_snap_global_planner
  trajectory_generator_lib
  path_searcher_lib
  waypoint_tool
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY plugins/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins
)

install(DIRECTORY rviz_tools/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz_tools
)

