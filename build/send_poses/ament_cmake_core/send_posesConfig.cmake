# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_send_poses_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED send_poses_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(send_poses_FOUND FALSE)
  elseif(NOT send_poses_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(send_poses_FOUND FALSE)
  endif()
  return()
endif()
set(_send_poses_CONFIG_INCLUDED TRUE)

# output package information
if(NOT send_poses_FIND_QUIETLY)
  message(STATUS "Found send_poses: 0.0.0 (${send_poses_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'send_poses' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${send_poses_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(send_poses_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${send_poses_DIR}/${_extra}")
endforeach()
