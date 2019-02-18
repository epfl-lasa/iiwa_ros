cmake_minimum_required(VERSION 2.8.3)
project(iiwa_description)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
 
install(DIRECTORY meshes/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
 
install(DIRECTORY urdf/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)