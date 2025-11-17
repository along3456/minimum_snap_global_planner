cmake_minimum_required(VERSION 3.0.2)
project(path_searcher)

message(STATUS "This is a library for searching path")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES PATH_SEARCHER_LIB
    CATKIN_DEPENDS nav_msgs roscpp tf
    DEPENDS EIGEN3 PCL
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(
    PATH_SEARCHER_LIB SHARED
    src/a_star/a_star_2d.cpp
    src/a_star/a_star_2d_flow.cpp
    src/subscriber/pointcloud_subscriber.cpp
    src/subscriber/init_pose_subscriber.cpp
)
target_link_libraries(
    PATH_SEARCHER_LIB
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)