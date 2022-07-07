# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_toycar3D_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED toycar3D_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(toycar3D_FOUND FALSE)
  elseif(NOT toycar3D_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(toycar3D_FOUND FALSE)
  endif()
  return()
endif()
set(_toycar3D_CONFIG_INCLUDED TRUE)

# output package information
if(NOT toycar3D_FIND_QUIETLY)
  message(STATUS "Found toycar3D: 0.0.0 (${toycar3D_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'toycar3D' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${toycar3D_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(toycar3D_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${toycar3D_DIR}/${_extra}")
endforeach()
