if(DEFINED INCLUDED_LIMESUITENG_CONFIG_CMAKE)
  return()
endif()
set(INCLUDED_LIMESUITENG_CONFIG_CMAKE TRUE)
set(limesuiteng_FOUND FALSE)

########################################################################
# LimeSuiteNGConfig - cmake project configuration for client clibraries
#
# The following will be set after find_package(LimeSuite):
# limesuiteng_LIBRARIES    - development libraries
# limesuiteng_INCLUDE_DIRS - development includes
# limesuiteng_FOUND        - library is found
#
# Or link with the import library target limesuiteng
########################################################################

########################################################################
## installation root
########################################################################
get_filename_component(LIMESUITENG_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

#Or support when the LIB_SUFFIX is an additional directory (ex debian)
if (NOT EXISTS ${LIMESUITENG_ROOT}/include AND EXISTS ${LIMESUITENG_ROOT}/../include)
    get_filename_component(LIMESUITENG_ROOT "${LIMESUITENG_ROOT}/.." ABSOLUTE)
endif ()

########################################################################
## locate the library
########################################################################
find_library(
  LIMESUITENG_LIBRARY limesuiteng
  PATHS ${LIMESUITENG_ROOT}/lib${LIB_SUFFIX}
  PATH_SUFFIXES ${CMAKE_LIBRARY_ARCHITECTURE}
  NO_DEFAULT_PATH
  )
if(NOT LIMESUITENG_LIBRARY)
  message(FATAL_ERROR "cannot find limesuiteng library in ${LIMESUITENG_ROOT}/lib${LIB_SUFFIX}")
endif()
set(limesuiteng_LIBRARIES ${LIMESUITENG_LIBRARY})

########################################################################
## locate the includes
########################################################################
find_path(
  LIMESUITENG_INCLUDE_DIR limesuite/SDRDevice.h
  PATHS ${LIMESUITENG_ROOT}/include
  NO_DEFAULT_PATH
)
if(NOT LIMESUITENG_INCLUDE_DIR)
  message(FATAL_ERROR "cannot find limesuiteng includes in ${LIMESUITENG_ROOT}/include")
endif()
set(limesuiteng_INCLUDE_DIRS ${LIMESUITENG_INCLUDE_DIR})
set(limesuiteng_FOUND TRUE)

########################################################################
## create import library target
########################################################################
add_library(limesuiteng SHARED IMPORTED)
set_property(TARGET limesuiteng PROPERTY IMPORTED_LOCATION ${LIMESUITENG_LIBRARY})
set_property(TARGET limesuiteng PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${LIMESUITENG_INCLUDE_DIR})
