cmake_minimum_required(VERSION 3.1)
project(iiwa_gazebo)

option(USE_SIMD "Enable native flags for optimizations" ON)

find_package(PkgConfig)

# Find RBDyn library and dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.49 REQUIRED)

pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(SpaceVecAlg REQUIRED SpaceVecAlg)
pkg_check_modules(RBDyn REQUIRED RBDyn)
pkg_check_modules(mc_rbdyn_urdf REQUIRED mc_rbdyn_urdf)

# Find catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_ros_control
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
# Gazebo cxx flags should have all the required C++ flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_gazebo
 CATKIN_DEPENDS roscpp gazebo_ros gazebo_ros_control
 DEPENDS Boost tinyxml2 SpaceVecAlg RBDyn mc_rbdyn_urdf
)

add_library(iiwa_gazebo_gravity_compensation_hw_sim src/gravity_compensation_hw_sim.cpp)

if(USE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC -march=native -faligned-new)
  endif()
endif()
target_include_directories(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(iiwa_gazebo_gravity_compensation_hw_sim PUBLIC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Eigen3::Eigen ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES})


add_library(iiwa_gazebo_gravity_compensation_plugin src/gravity_compensation_plugin.cpp)
if(USE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(iiwa_gazebo_gravity_compensation_plugin PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(iiwa_gazebo_gravity_compensation_plugin PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(iiwa_gazebo_gravity_compensation_plugin PUBLIC -march=native -faligned-new)
  endif()
endif()
target_include_directories(iiwa_gazebo_gravity_compensation_plugin PUBLIC include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(iiwa_gazebo_gravity_compensation_plugin PUBLIC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} Eigen3::Eigen ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES})


# Install
install(TARGETS iiwa_gazebo_gravity_compensation_plugin iiwa_gazebo_gravity_compensation_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES iiwa_robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)