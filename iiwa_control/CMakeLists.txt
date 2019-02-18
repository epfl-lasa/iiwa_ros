cmake_minimum_required(VERSION 2.8.3)
project(iiwa_control)

## Find catkin
find_package(catkin REQUIRED)

##Needed for ros packages
catkin_package()

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})