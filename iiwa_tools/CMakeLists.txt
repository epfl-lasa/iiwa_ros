#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Bernardo Fichera
#|              Walid Amanhoud
#|    email:    costashatz@gmail.com
#|              bernardo.fichera@epfl.ch
#|              walid.amanhoud@epfl.ch
#|    Other contributors:
#|              Yoan Mollard (yoan@aubrune.eu)
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
project(iiwa_tools)

option(ENABLE_SIMD "Build with all SIMD instructions on the current local machine" ON)

find_package(PkgConfig)

# Find RBDyn library and dependencies
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Boost 1.49 REQUIRED)

pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(RBDyn REQUIRED RBDyn)
pkg_check_modules(mc_rbdyn_urdf REQUIRED mc_rbdyn_urdf)

# Find catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

add_service_files(
  FILES
  GetFK.srv
  GetIK.srv
  GetJacobian.srv
  GetJacobians.srv
  GetGravity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs geometry_msgs
 DEPENDS Boost tinyxml2 SpaceVecAlg RBDyn mc_rbdyn_urdf
 LIBRARIES iiwa_tools
)

add_library(iiwa_tools SHARED src/iiwa_tools.cpp)
target_compile_options(iiwa_tools PUBLIC -std=c++11)
target_include_directories(iiwa_tools PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(iiwa_tools PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES})

add_executable(iiwa_service src/iiwa_service.cpp src/iiwa_service_node.cpp)

if(ENABLE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(iiwa_service PUBLIC -march=native)
    target_compile_options(iiwa_tools PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(iiwa_service PUBLIC -faligned-new)
      target_compile_options(iiwa_tools PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(iiwa_service PUBLIC -march=native -faligned-new)
    target_compile_options(iiwa_tools PUBLIC -march=native -faligned-new)
  endif()
endif()

target_compile_options(iiwa_service PUBLIC -std=c++11)
target_include_directories(iiwa_service PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(iiwa_service PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES} iiwa_tools)

add_dependencies(iiwa_service iiwa_tools_generate_messages_cpp)

# Install
install(TARGETS iiwa_service iiwa_tools
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
