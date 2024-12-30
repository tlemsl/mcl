cmake_minimum_required(VERSION 3.0.2)
project(mcl)

# Add core library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/core 
                 ${CMAKE_CURRENT_BINARY_DIR}/core)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -fPIC")
# ROS1 wrapper (conditional)
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    pcl_ros
  )

  # Find other dependencies
  find_package(Eigen3 REQUIRED)
  find_package(Sophus REQUIRED)
  find_package(yaml-cpp REQUIRED)

  catkin_package(
    INCLUDE_DIRS
      include
      core/include
    LIBRARIES
      mcl_ros1
      mcl_core
    CATKIN_DEPENDS 
      roscpp 
      sensor_msgs 
      nav_msgs 
      geometry_msgs 
      tf2 
      tf2_ros
  )

  # Include directories
  include_directories(
    ros1_wrapper/include
    core/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
  )

  add_library(mcl_ros1
    ros1_wrapper/src/mcl_ros1_wrapper.cpp
  )

  target_link_libraries(mcl_ros1
    mcl_core
    ${catkin_LIBRARIES}
    yaml-cpp
  )

  add_executable(mcl_ros1_node
    ros1_wrapper/src/mcl_ros1_node.cpp
  )

  target_link_libraries(mcl_ros1_node
    mcl_ros1
    ${catkin_LIBRARIES}
  )
endif()

# ROS2 wrapper (future implementation)
if("$ENV{ROS_VERSION}" STREQUAL "2")
  message(ERROR "ROS2 wrapper is not implemented yet")
endif()
