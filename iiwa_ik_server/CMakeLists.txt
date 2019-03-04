cmake_minimum_required(VERSION 3.1.0)
project(iiwa_ik_server)

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
  GetIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iiwa_gazebo
 CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs geometry_msgs
 DEPENDS Boost tinyxml2 SpaceVecAlg RBDyn mc_rbdyn_urdf
)

add_executable(iiwa_ik_server src/iiwa_ik_server.cpp src/iiwa_ik_server_node.cpp)

if(ENABLE_SIMD)
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    set(CXX_COMPILER_VERSION ${GCC_VERSION})
    target_compile_options(iiwa_ik_server PUBLIC -march=native)
    if(GCC_VERSION VERSION_GREATER 7.0)
      target_compile_options(iiwa_ik_server PUBLIC -faligned-new)
    endif()
  elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    target_compile_options(iiwa_ik_server PUBLIC -march=native -faligned-new)
  endif()
endif()
target_compile_options(iiwa_ik_server PUBLIC -std=c++11)
target_include_directories(iiwa_ik_server PUBLIC include ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${tinyxml2_INCLUDE_DIRS} ${SpaceVecAlg_INCLUDE_DIRS} ${RBDyn_INCLUDE_DIRS} ${mc_rbdyn_urdf_INCLUDE_DIRS})
target_link_libraries(iiwa_ik_server PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${tinyxml2_LIBRARIES} ${SpaceVecAlg_LIBRARIES} ${RBDyn_LIBRARIES} ${mc_rbdyn_urdf_LIBRARIES})


# Install
install(TARGETS iiwa_ik_server
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
