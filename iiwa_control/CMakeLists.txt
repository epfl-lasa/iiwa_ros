cmake_minimum_required(VERSION 3.1.0)
project(iiwa_control)

option(ENABLE_SIMD "Build with all SIMD instructions on the current local machine" ON)

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
  iiwa_tools
  dynamic_reconfigure)

## Find control_stack
find_package(RobotControllers REQUIRED COMPONENTS
  RobotControllers
)

find_package(PkgConfig)
## Find Eigen
pkg_search_module(Eigen3 REQUIRED eigen3)

# Find Corrade
find_package(Corrade REQUIRED Utility PluginManager)


find_package(RobotControllers REQUIRED PassiveDSController)


generate_dynamic_reconfigure_options(
    cfg/ds_impedance_params.cfg
)

##Needed for ros packages
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs hardware_interface controller_interface controller_manager urdf iiwa_tools
  DEPENDS RobotControllers CorradeUtility CorradePluginManager
  LIBRARIES CustomEffortController
)

set(iiwa_control_SOURCES
  src/custom_effort_controller.cpp
  src/custom_joint_state_controller.cpp
  src/ds_impedance_controller.cpp
)

add_library(CustomEffortController ${iiwa_control_SOURCES})

if(ENABLE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(CustomEffortController PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(CustomEffortController PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(CustomEffortController PUBLIC -march=native -faligned-new)
  endif()
endif()

target_include_directories(CustomEffortController PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${RobotControllers_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

target_link_libraries(CustomEffortController PUBLIC
  ${catkin_LIBRARIES}
  ${RobotControllers_LIBRARIES}
  ${Eigen3_LIBRARIES}
  Corrade::PluginManager)

add_dependencies(CustomEffortController iiwa_service)

# Require C++11
set_property(TARGET CustomEffortController PROPERTY CXX_STANDARD 17)
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