# - Try to find RobotControllers
# Once executed this script will define the following:
#  RobotControllers_FOUND - RobotControllers was succesfully found
#  RobotControllers_INCLUDE_DIRS - RobotControllers's include directories
#  RobotControllers_LIBRARIES - RobotControllers's libraries
#  RobotControllers_DIR - Directory where RobotControllers was found
#------------------------------------------------------------------------------
# Users can set RobotControllers_DIR to force CMake to look in a particular location,
# setting the RobotControllers_DIR environment variable will have a similar effect.

cmake_minimum_required(VERSION 2.8.3)

#These are RobotControllers's known components (ie. libraries)
set(RobotControllers_COMPONENTS RobotControllers)


#Set INCLUDE hints
set(RobotControllers_INCLUDE_HINTS
    "${RobotControllers_DIR}/include"
    "$ENV{RobotControllers_DIR}/include" )

# Set LIBRARY hints
set(RobotControllers_LIBRARY_HINTS
    "${RobotControllers_DIR}/lib"
    "$ENV{RobotControllers_DIR}/lib" )

# Find include directories
find_path(RobotControllers_INCLUDE_DIR robot_controllers/AbstractController.hpp HINTS ${RobotControllers_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
set(RobotControllers_FILTERED_COMPONENTS ${RobotControllers_FIND_COMPONENTS})

if ( RobotControllers_FIND_COMPONENTS )
    foreach(comp ${RobotControllers_FIND_COMPONENTS})
        list(FIND RobotControllers_COMPONENTS ${comp} ${comp}_KNOWN)
        if (${comp}_KNOWN EQUAL -1)
            list(REMOVE_ITEM RobotControllers_FILTERED_COMPONENTS ${comp})
            message(STATUS "Unknown RobotControllers component ${comp}")
        endif()
    endforeach()
endif()

list(LENGTH RobotControllers_FILTERED_COMPONENTS RobotControllers_NUMBER_OF_COMPONENTS)
set(RobotControllers_FOUND_COMPONENTS TRUE)

# Look for components (ie. libraries)
if( ${RobotControllers_NUMBER_OF_COMPONENTS}  )
    foreach(comp ${RobotControllers_FILTERED_COMPONENTS})
        #Look for the actual library here
        find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${RobotControllers_LIBRARY_HINTS})
        if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
            message(STATUS "Could not find RobotControllers's ${comp}")
            set(RobotControllers_FOUND_COMPONENTS FALSE)
        else()
            #If everything went well append this component to list of libraries
            list(APPEND RobotControllers_LIBRARY ${${comp}_LIBRARY})
        endif()
    endforeach()
else()
    message(STATUS "No RobotControllers components specified")
endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    RobotControllers #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    RobotControllers_LIBRARY
    RobotControllers_INCLUDE_DIR
    RobotControllers_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
set(RobotControllers_LIBRARIES ${RobotControllers_LIBRARY} )
set(RobotControllers_INCLUDE_DIRS ${RobotControllers_INCLUDE_DIR} )
set(RobotControllers_FOUND ${RobotControllers_FOUND})

# If RobotControllers was found, update RobotControllers_DIR to show where it was found
if ( RobotControllers_FOUND )
  get_filename_component(RobotControllers_NEW_DIR "${RobotControllers_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(RobotControllers_DIR ${RobotControllers_NEW_DIR} CACHE FILEPATH "RobotControllers root directory" FORCE)

#Hide these variables
mark_as_advanced(RobotControllers_INCLUDE_DIR RobotControllers_LIBRARY RobotControllers_FOUND)