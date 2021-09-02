# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.5)
   message(FATAL_ERROR "CMake >= 2.6.0 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.6...3.18)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_targetsDefined)
set(_targetsNotDefined)
set(_expectedTargets)
foreach(_expectedTarget osqp::osqpstatic osqp::osqp)
  list(APPEND _expectedTargets ${_expectedTarget})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  unset(_targetsDefined)
  unset(_targetsNotDefined)
  unset(_expectedTargets)
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)


# Create imported target osqp::osqpstatic
add_library(osqp::osqpstatic STATIC IMPORTED)

set_target_properties(osqp::osqpstatic PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/emmanuel/rlss_ws/src/rlss_ros/third_party/rlss/third_party/splx/third_party/qp_wrappers/third_party/osqp/include"
)

# Create imported target osqp::osqp
add_library(osqp::osqp SHARED IMPORTED)

set_target_properties(osqp::osqp PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "/home/emmanuel/rlss_ws/src/rlss_ros/third_party/rlss/third_party/splx/third_party/qp_wrappers/third_party/osqp/include"
)

# Import target "osqp::osqpstatic" for configuration "Release"
set_property(TARGET osqp::osqpstatic APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osqp::osqpstatic PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "/home/emmanuel/rlss_ws/src/rlss_ros/build/third_party/rlss/third_party/splx/third_party/qp_wrappers/third_party/osqp/out/libosqp.a"
  )

# Import target "osqp::osqp" for configuration "Release"
set_property(TARGET osqp::osqp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osqp::osqp PROPERTIES
  IMPORTED_LOCATION_RELEASE "/home/emmanuel/rlss_ws/src/rlss_ros/build/third_party/rlss/third_party/splx/third_party/qp_wrappers/third_party/osqp/out/libosqp.so"
  IMPORTED_SONAME_RELEASE "libosqp.so"
  )

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
