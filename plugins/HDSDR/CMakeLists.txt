########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.12)

# HDSDR is Windows only
if (NOT CMAKE_HOST_WIN32)
    return()
endif()

cmake_dependent_option(ENABLE_HDSDR "Enable HDSDR plugin" ON "ENABLE_LIBRARY;MSVC;CMAKE_SIZEOF_VOID_P EQUAL 4" OFF)
add_feature_info(HDSDR ENABLE_HDSDR "HDSDR (32bit only) plugin for limesuiteng")
if (NOT ENABLE_HDSDR)
    return()
endif()

# Only compile on Windows
if(NOT MSVC)
    message(FATAL_ERROR "HDSDR build is available on Microsoft Visual Studio")
endif()

# Only compile if 32-bit
if(NOT (CMAKE_SIZEOF_VOID_P EQUAL 4))
    message(FATAL_ERROR "HDSDR build is only supported for 32-bit builds")
endif()

# Check if using correct Visual Studio version
if(MSVC_TOOLSET_VERSION LESS 141)
    message(FATAL_ERROR "MSVC version should be atleast v141 (Visual Studio 2017)")
endif()

project(ExtIO_LimeSDR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # Select the release build by default
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release")
        message(STATUS "Build type not specified: defaulting to release.")
    endif(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE
        ${CMAKE_BUILD_TYPE}
        CACHE STRING "")

    option(BUILD_SHARED_LIBS "Build shared library" ON)

    # Include modules for finding CyAPI
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

    message(STATUS "Make sure that BUILD_SHARED_LIBS option matches BUILD_SHARED_LIBS option when LimeSuite was built")
    message(STATUS "Only LimeSuite source and build directories should be needed, everything else is found automatically")

    ########################################################################
    # Find LimeSuite
    ########################################################################
    find_path(
        LIMESUITE_BUILD_DIR
        NAMES limesuiteng
        PATHS
        NO_DEFAULT_PATH)
    find_path(
        LIMESUITE_SOURCE_DIR
        PATHS
        NO_DEFAULT_PATH)

    find_path(
        LIMESUITE_INCLUDE_DIR
        NAMES LimeSuite.h
        HINTS ${LIMESUITE_SOURCE_DIR}/src/lime)

    find_path(
        LIMESUITENG_INCLUDE_DIR
        NAMES limesuiteng/SDRDevice.h
        HINTS ${LIMESUITE_SOURCE_DIR}/src/include)

    find_library(
        LIMESUITE_LIB
        NAMES LimeSuiteNG limesuiteng
        HINTS ${LIMESUITE_BUILD_DIR}/src)
else()
    set(LIMESUITE_LIB limesuiteng)
endif()

########################################################################
# Add shared dll
########################################################################
add_library(
    ExtIO_LimeSDR SHARED
    ${PROJECT_SOURCE_DIR}/src/ExtIO_LimeSDR.def
    ${PROJECT_SOURCE_DIR}/src/ExtIO_LimeSDR.cpp
    ${PROJECT_SOURCE_DIR}/src/ExtIO_LimeSDR.h
    ${PROJECT_SOURCE_DIR}/src/ExtIO_LimeSDR.rc
    ${PROJECT_SOURCE_DIR}/src/LC_ExtIO_Types.h
    ${PROJECT_SOURCE_DIR}/src/resource.h)
# Add Limesuite libraries
target_link_libraries(ExtIO_LimeSDR PUBLIC ${LIMESUITE_LIB})
target_include_directories(ExtIO_LimeSDR PUBLIC ${LIMESUITE_INCLUDE_DIR} ${LIMESUITENG_INCLUDE_DIR})

target_compile_definitions(ExtIO_LimeSDR PRIVATE NOMINMAX)
target_compile_options(ExtIO_LimeSDR PRIVATE /Wall /permissive-)
########################################################################
# Check if linking the dll statically and find required libraries
########################################################################
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    if(BUILD_SHARED_LIBS STREQUAL "OFF")
        # Find FTD3XX
        find_path(
            FTD3XX_INCLUDE_DIR
            NAMES FTD3XX.h
            HINTS ${LIMESUITE_SOURCE_DIR}/external/FTD3XXLibrary)
        find_library(
            FTD3XX_LIB_STATIC
            NAMES FTD3XX.lib
            HINTS ${FTD3XX_INCLUDE_DIR}/Win32/Static)

        # Find CyAPI
        include(FindCyAPI)

        # Add libraries
        target_include_directories(ExtIO_LimeSDR PUBLIC ${FTD3XX_INCLUDE_DIR})
        target_link_libraries(ExtIO_LimeSDR PUBLIC ${FTD3XX_LIB_STATIC} ${CYAPI_LIBRARY})
    endif()
endif()
