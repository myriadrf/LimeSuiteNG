if(DEFINED INCLUDED_LIMESUITENG_CONFIG_CMAKE)
  return()
endif()
set(INCLUDED_LIMESUITENG_CONFIG_CMAKE TRUE)

########################################################################
# LimeSuiteNGConfig - cmake project configuration for client clibraries
#
# The following will be set after find_package(LimeSuite):
# limesuiteng_LIBRARIES    - development libraries
# limesuiteng_INCLUDE_DIRS - development includes
#
# Or link with the import library target limesuiteng
########################################################################

########################################################################
## installation root
########################################################################
get_filename_component(LIMESUITE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

#Or support when the LIB_SUFFIX is an additional directory (ex debian)
if (NOT EXISTS ${LIMESUITE_ROOT}/include AND EXISTS ${LIMESUITE_ROOT}/../include)
    get_filename_component(LIMESUITE_ROOT "${LIMESUITE_ROOT}/.." ABSOLUTE)
endif ()

########################################################################
## locate the library
########################################################################
find_library(
  LIMESUITENG_LIBRARY limesuiteng
  PATHS ${LIMESUITE_ROOT}/lib${LIB_SUFFIX}
  PATH_SUFFIXES ${CMAKE_LIBRARY_ARCHITECTURE}
  NO_DEFAULT_PATH
  )
if(NOT LIMESUITENG_LIBRARY)
  message(FATAL_ERROR "cannot find LimeSuite library in ${LIMESUITE_ROOT}/lib${LIB_SUFFIX}")
endif()
set(limesuiteng_LIBRARIES ${LIMESUITENG_LIBRARY})

########################################################################
## locate the includes
########################################################################
find_path(
  LIMESUITENG_INCLUDE_DIR limesuite/SDRDevice.h
  PATHS ${LIMESUITE_ROOT}/include
  NO_DEFAULT_PATH
)
if(NOT LIMESUITENG_INCLUDE_DIR)
  message(FATAL_ERROR "cannot find LimeSuite includes in ${LIMESUITE_ROOT}/include")
endif()
set(limesuiteng_INCLUDE_DIRS ${LIMESUITENG_INCLUDE_DIR})

########################################################################
## create import library target
########################################################################
add_library(LimeSuite SHARED IMPORTED)
set_property(TARGET LimeSuite PROPERTY IMPORTED_LOCATION ${LIMESUITENG_LIBRARY})
set_property(TARGET LimeSuite PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${LIMESUITENG_INCLUDE_DIR})
