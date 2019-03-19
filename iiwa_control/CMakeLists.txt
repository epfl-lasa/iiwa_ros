cmake_minimum_required(VERSION 3.1.0)
project(iiwa_control)

# Tell CMake where to find "Find<LIB>.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
  controller_interface
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
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_interface controller_manager urdf iiwa_tools
  DEPENDS ControlStack
  LIBRARIES CustomEffortController
)

set(iiwa_control_SOURCES
  src/custom_effort_controller.cpp
)

add_library(CustomEffortController ${iiwa_control_SOURCES})

target_include_directories(CustomEffortController PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${ControlStack_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(CustomEffortController PUBLIC
  ${catkin_LIBRARIES}
  ${ControlStack_LIBRARIES}
  ${EIGEN3_LIBRARIES})

# Require C++11
set_property(TARGET CustomEffortController PROPERTY CXX_STANDARD 11)
set_property(TARGET CustomEffortController PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS CustomEffortController
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES plugins/custom_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})