# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_apartment_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED apartment_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(apartment_FOUND FALSE)
  elseif(NOT apartment_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(apartment_FOUND FALSE)
  endif()
  return()
endif()
set(_apartment_CONFIG_INCLUDED TRUE)

# output package information
if(NOT apartment_FIND_QUIETLY)
  message(STATUS "Found apartment: 0.0.0 (${apartment_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'apartment' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${apartment_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(apartment_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${apartment_DIR}/${_extra}")
endforeach()
