cmake_minimum_required(VERSION 3.0.2)
project(mcl_core)

# Set C++ standard and optimization flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create core library
add_library(${PROJECT_NAME}
  src/particle_filter.cpp
  src/motion_model.cpp
  src/measurement_model.cpp
)

# Specify include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    Sophus::Sophus
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
