# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sensor_bridge_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sensor_bridge_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sensor_bridge_FOUND FALSE)
  elseif(NOT sensor_bridge_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sensor_bridge_FOUND FALSE)
  endif()
  return()
endif()
set(_sensor_bridge_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sensor_bridge_FIND_QUIETLY)
  message(STATUS "Found sensor_bridge: 0.0.0 (${sensor_bridge_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sensor_bridge' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${sensor_bridge_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sensor_bridge_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sensor_bridge_DIR}/${_extra}")
endforeach()
