# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_roboracer_localization_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED roboracer_localization_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(roboracer_localization_FOUND FALSE)
  elseif(NOT roboracer_localization_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(roboracer_localization_FOUND FALSE)
  endif()
  return()
endif()
set(_roboracer_localization_CONFIG_INCLUDED TRUE)

# output package information
if(NOT roboracer_localization_FIND_QUIETLY)
  message(STATUS "Found roboracer_localization: 0.0.0 (${roboracer_localization_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'roboracer_localization' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${roboracer_localization_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(roboracer_localization_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${roboracer_localization_DIR}/${_extra}")
endforeach()
