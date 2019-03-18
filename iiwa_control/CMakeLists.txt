cmake_minimum_required(VERSION 3.1.0)
project(iiwa_control)

# Tell CMake where to find "Find<LIB>.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  hardware_interface
  controller_manager
  sensor_msgs
  urdf
  iiwa_tools)

## Find control_stack
find_package(ControlStack REQUIRED COMPONENTS
  ControlStack
)

## Find Eigen
find_package (Eigen3 3.3 REQUIRED)

##Needed for ros packages
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs tf hardware_interface controller_manager sensor_msgs urdf iiwa_tools
  DEPENDS ControlStack
)

# set(iiwa_control_HEADERS
#   include/iiwa_control/custom_effort_interface.hpp
# )

set(iiwa_control_SOURCES
  src/custom_effort_interface.cpp
)

add_library(CustomEffortInterface ${iiwa_control_SOURCES})

target_include_directories(CustomEffortInterface PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${ControlStack_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(CustomEffortInterface PUBLIC
  ${catkin_LIBRARIES}
  ${ControlStack_LIBRARIES}
  ${EIGEN3_LIBRARIES})

# Require C++11
set_property(TARGET CustomEffortInterface PROPERTY CXX_STANDARD 11)
set_property(TARGET CustomEffortInterface PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS CustomEffortInterface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})