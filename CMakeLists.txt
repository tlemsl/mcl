cmake_minimum_required(VERSION 3.0.2)
project(mcl)

# Set C++ standard and optimization flags
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -ggdb -Wall -Wextra")

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_eigen
  sensor_msgs
  nav_msgs
  geometry_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

# Catkin package settings
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf2
    tf2_ros
    tf2_eigen
    sensor_msgs
    nav_msgs
    geometry_msgs
  DEPENDS
    PCL
    EIGEN3
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Library settings
add_library(${PROJECT_NAME}
  src/mcl.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Executable settings
add_executable(${PROJECT_NAME}_node src/mcl_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.pgm"
)
