# - Try to find FRI
# Once executed this script will define the following:
#  FRI_FOUND - FRI was succesfully found
#  FRI_INCLUDE_DIRS - FRI's include directories
#  FRI_LIBRARIES - FRI's libraries
#  FRI_DIR - Directory where FRI was found
#------------------------------------------------------------------------------
# Users can set FRI_DIR to force CMake to look in a particular location,
# setting the FRI_DIR environment variable will have a similar effect.

cmake_minimum_required(VERSION 2.8.3)

#These are FRI's known components (ie. libraries)
set(FRI_COMPONENTS kuka_fri)


#Set INCLUDE hints
set(FRI_INCLUDE_HINTS
    "${FRI_DIR}/include"
    "$ENV{FRI_DIR}/include" )

# Set LIBRARY hints
set(FRI_LIBRARY_HINTS
    "${FRI_DIR}/lib"
    "$ENV{FRI_DIR}/lib" )

# Find include directories
find_path(FRI_INCLUDE_DIR kuka/fri/LBRClient.h HINTS ${FRI_INCLUDE_HINTS} )

# Verify we know about all the components requested
# and remove those we don't know about
set(FRI_FILTERED_COMPONENTS ${FRI_FIND_COMPONENTS})

if ( FRI_FIND_COMPONENTS )
    foreach(comp ${FRI_FIND_COMPONENTS})
        list(FIND FRI_COMPONENTS ${comp} ${comp}_KNOWN)
        if (${comp}_KNOWN EQUAL -1)
            list(REMOVE_ITEM FRI_FILTERED_COMPONENTS ${comp})
            message(STATUS "Unknown FRI component ${comp}")
        endif()
    endforeach()
endif()

list(LENGTH FRI_FILTERED_COMPONENTS FRI_NUMBER_OF_COMPONENTS)
set(FRI_FOUND_COMPONENTS TRUE)

# Look for components (ie. libraries)
if( ${FRI_NUMBER_OF_COMPONENTS}  )
    foreach(comp ${FRI_FILTERED_COMPONENTS})
        #Look for the actual library here
        find_library(${comp}_LIBRARY NAMES ${comp} HINTS ${FRI_LIBRARY_HINTS})
        if ( ${${comp}_LIBRARY} STREQUAL ${comp}_LIBRARY-NOTFOUND)
            message(STATUS "Could not find FRI's ${comp}")
            set(FRI_FOUND_COMPONENTS FALSE)
        else()
            #If everything went well append this component to list of libraries
            list(APPEND FRI_LIBRARY ${${comp}_LIBRARY})
        endif()
    endforeach()
else()
    message(STATUS "No FRI components specified")
endif()


# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    FRI #Package name
    DEFAULT_MSG
    # Variables required to evaluate as TRUE
    FRI_LIBRARY
    FRI_INCLUDE_DIR
    FRI_FOUND_COMPONENTS)

# Copy the values of the advanced variables to the user-facing ones
set(FRI_LIBRARIES ${FRI_LIBRARY} )
set(FRI_INCLUDE_DIRS ${FRI_INCLUDE_DIR} )
set(FRI_FOUND ${FRI_FOUND})

# If FRI was found, update FRI_DIR to show where it was found
if ( FRI_FOUND )
  get_filename_component(FRI_NEW_DIR "${FRI_INCLUDE_DIRS}/../" ABSOLUTE)
endif()
set(FRI_DIR ${FRI_NEW_DIR} CACHE FILEPATH "FRI root directory" FORCE)

#Hide these variables
mark_as_advanced(FRI_INCLUDE_DIR FRI_LIBRARY FRI_FOUND)