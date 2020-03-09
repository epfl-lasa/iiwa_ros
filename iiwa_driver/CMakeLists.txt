#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Bernardo Fichera
#|              Walid Amanhoud
#|    email:   konstantinos.chatzilygeroudis@epfl.ch
#|             bernardo.fichera@epfl.ch
#|             walid.amanhoud@epfl.ch
#|    website: lasa.epfl.ch
#|
#|    This file is part of iiwa_ros.
#|
#|    iiwa_ros is free software: you can redistribute it and/or modify
#|    it under the terms of the GNU General Public License as published by
#|    the Free Software Foundation, either version 3 of the License, or
#|    (at your option) any later version.
#|
#|    iiwa_ros is distributed in the hope that it will be useful,
#|    but WITHOUT ANY WARRANTY; without even the implied warranty of
#|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#|    GNU General Public License for more details.
#|
cmake_minimum_required(VERSION 3.1.0)
project(iiwa_driver)

# Tell CMake where to find "FindFRI.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
  geometry_msgs
  tf
  hardware_interface
  controller_manager
  sensor_msgs
  urdf
  realtime_tools
)

find_package(FRI REQUIRED COMPONENTS
    kuka_fri
)

# Message files
add_message_files(
  FILES
  AdditionalOutputs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp message_runtime geometry_msgs tf std_msgs sensor_msgs hardware_interface controller_manager urdf realtime_tools)

add_executable(iiwa_driver src/iiwa.cpp src/iiwa_driver.cpp)

# Require C++11
set_property(TARGET iiwa_driver PROPERTY CXX_STANDARD 11)
set_property(TARGET iiwa_driver PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(iiwa_driver PUBLIC include ${catkin_INCLUDE_DIRS} ${FRI_INCLUDE_DIRS})
target_link_libraries(iiwa_driver ${catkin_LIBRARIES} ${FRI_LIBRARIES})
add_dependencies(iiwa_driver ${catkin_EXPORTED_TARGETS})

add_dependencies(iiwa_driver iiwa_driver_generate_messages_cpp)


install(TARGETS iiwa_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})