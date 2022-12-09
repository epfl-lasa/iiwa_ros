#|
#|    Copyright (C) 2019-2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Matthias Mayr
#|              Bernardo Fichera
#|    email:    costashatz@gmail.com
#|              matthias.mayr@cs.lth.se
#|              bernardo.fichera@epfl.ch
#|    Other contributors:
#|              Yoan Mollard (yoan@aubrune.eu)
#|              Walid Amanhoud (walid.amanhoud@epfl.ch)
#|    website:  lasa.epfl.ch
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
project(iiwa_gazebo)

# Find catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_ros_control
  iiwa_tools
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_gazebo
 CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control iiwa_tools
)

add_library(iiwa_gazebo_gravity_compensation_hw_sim src/gravity_compensation_hw_sim.cpp)

# Require C++11
set_property(TARGET iiwa_gazebo_gravity_compensation_hw_sim PROPERTY CXX_STANDARD 11)
set_property(TARGET iiwa_gazebo_gravity_compensation_hw_sim PROPERTY CXX_STANDARD_REQUIRED ON)

target_include_directories(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_dependencies(iiwa_gazebo_gravity_compensation_hw_sim ${iiwa_tools_EXPORTED_TARGETS})

# Install
install(TARGETS iiwa_gazebo_gravity_compensation_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES iiwa_robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)