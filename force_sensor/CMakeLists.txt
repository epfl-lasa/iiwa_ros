#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Patrick Sgrò (maintainer)
#|    email:    patrick.sgro@hotmail.com
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
project(force_sensor)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY meshes/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY urdf/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)