find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_LIMESUITENG gnuradio-limesuiteng)

FIND_PATH(
    GR_LIMESUITENG_INCLUDE_DIRS
    NAMES gnuradio/limesuiteng/api.h
    HINTS $ENV{LIMESUITENG_DIR}/include
        ${PC_LIMESUITENG_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_LIMESUITENG_LIBRARIES
    NAMES gnuradio-limesuiteng
    HINTS $ENV{LIMESUITENG_DIR}/lib
        ${PC_LIMESUITENG_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-limesuitengTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_LIMESUITENG DEFAULT_MSG GR_LIMESUITENG_LIBRARIES GR_LIMESUITENG_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_LIMESUITENG_LIBRARIES GR_LIMESUITENG_INCLUDE_DIRS)
