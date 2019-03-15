# - Try to find ControlStack
# Once executed this script will define the following:
#  ControlStack_FOUND - ControlStack was succesfully found
#  ControlStack_INCLUDE_DIRS - ControlStack's include directories
#  ControlStack_LIBRARIES - ControlStack's libraries
#  ControlStack_DIR - Directory where ControlStack was found
#------------------------------------------------------------------------------
# Users can set ControlStack_DIR to force CMake to look in a particular location,
# setting the ControlStack_DIR environment variable will have a similar effect.

cmake_minimum_required(VERSION 2.8.3)

#These are ControlStack's known components (ie. libraries)
set(ControlStack_COMPONENTS ControlStack)


#Set INCLUDE hints
set(ControlStack_INCLUDE_HINTS
    "${ControlStack_DIR}/include"
    "$ENV{ControlStack_DIR}/include" )

# Set LIBRARY hints
set(ControlStack_LIBRARY_HINTS
    "${ControlStack_DIR}/lib"
    "$ENV{ControlStack_DIR}/lib" )

# Find include directories
find_path(ControlStack_INCLUDE_DIR control_stack/control_stack.hpp HINTS ${ControlStack_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
set(ControlStack_FILTERED_COMPONENTS ${ControlStack_FIND_COMPONENTS})

if ( ControlStack_FIND_COMPONENTS )
    foreach(comp ${ControlStack_FIND_COMPONENTS})
        list(FIND ControlStack_COMPONENTS ${comp} ${comp}_KNOWN)
        if (${comp}_KNOWN EQUAL -1)
            list(REMOVE_ITEM ControlStack_FILTERED_COMPONENTS ${comp})
            message(STATUS "Unknown ControlStack component ${comp}")
        endif()
    endforeach()
endif()

list(LENGTH ControlStack_FILTERED_COMPONENTS ControlStack_NUMBER_OF_COMPONENTS)
set(ControlStack_FOUND_COMPONENTS TRUE)

# Look for components (ie. libraries)
if( ${ControlStack_NUMBER_OF_COMPONENTS}  )
    foreach(comp ${ControlStack_FILTERED_COMPONENTS})
        #Look for the actual library here
        find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${ControlStack_LIBRARY_HINTS})
        if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
            message(STATUS "Could not find ControlStack's ${comp}")
            set(ControlStack_FOUND_COMPONENTS FALSE)
        else()
            #If everything went well append this component to list of libraries
            list(APPEND ControlStack_LIBRARY ${${comp}_LIBRARY})
        endif()
    endforeach()
else()
    message(STATUS "No ControlStack components specified")
endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    ControlStack #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    ControlStack_LIBRARY
    ControlStack_INCLUDE_DIR
    ControlStack_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
set(ControlStack_LIBRARIES ${ControlStack_LIBRARY} )
set(ControlStack_INCLUDE_DIRS ${ControlStack_INCLUDE_DIR} )
set(ControlStack_FOUND ${ControlStack_FOUND})

# If ControlStack was found, update ControlStack_DIR to show where it was found
if ( ControlStack_FOUND )
  get_filename_component(ControlStack_NEW_DIR "${ControlStack_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(ControlStack_DIR ${ControlStack_NEW_DIR} CACHE FILEPATH "ControlStack root directory" FORCE)

#Hide these variables
mark_as_advanced(ControlStack_INCLUDE_DIR ControlStack_LIBRARY ControlStack_FOUND)