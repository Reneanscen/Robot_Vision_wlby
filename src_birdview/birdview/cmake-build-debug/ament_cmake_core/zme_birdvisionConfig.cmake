# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_zme_birdvision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED zme_birdvision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(zme_birdvision_FOUND FALSE)
  elseif(NOT zme_birdvision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(zme_birdvision_FOUND FALSE)
  endif()
  return()
endif()
set(_zme_birdvision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT zme_birdvision_FIND_QUIETLY)
  message(STATUS "Found zme_birdvision: 0.0.0 (${zme_birdvision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'zme_birdvision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${zme_birdvision_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(zme_birdvision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${zme_birdvision_DIR}/${_extra}")
endforeach()
