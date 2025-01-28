cmake_minimum_required(VERSION 3.15)
include(FeatureSummary)
include(CMakeDependentOption)

# limesuiteng is meta project, that encompasses all subcomponents.
# it's version could be used for packaging versioning.
# Subprojects like liblimesuiteng, limesuiteng-cli...  have their own individual versions
# but those can't really be used to set individual package versions, when all packages
# are being built from the same source, as source package can have only one version.
project(
    "limesuiteng"
    VERSION 25.1.0
    DESCRIPTION "Meta project encompassing all LimeSuiteNG components"
    LANGUAGES C CXX)
message(STATUS "cmake version: ${CMAKE_VERSION}")

set(CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

#include modules for finding CyAPI
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

include(CheckDebianChangelogVersion)

set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release;RelWithDebInfo"
    CACHE STRING INTERNAL FORCE)

### Require out-of-source builds
file(TO_CMAKE_PATH "${PROJECT_BINARY_DIR}/CMakeLists.txt" LOC_PATH)
if(EXISTS "${LOC_PATH}")
    message(
        FATAL_ERROR "You cannot build in a source directory (or any directory with a CMakeLists.txt file)."
                    "Please make a build subdirectory. Feel free to remove CMakeCache.txt and CMakeFiles.")
endif()

# Help IDEs with code completion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build the shared library" ON)

list(FIND CMAKE_CONFIGURATION_TYPES "${CMAKE_BUILD_TYPE}" index)
if(${index} EQUAL -1)
    set(CMAKE_BUILD_TYPE "Release")
endif()

#LIME_SUITE_ROOT is compiled into the library to locate the install base.
#By default, the LIME_SUITE_ROOT is set to the CMAKE_INSTALL_PREFIX.
#However users may overload this by specifying -DLIME_SUITE_ROOT=<path>.
set(LIME_SUITE_ROOT
    "${CMAKE_INSTALL_PREFIX}"
    CACHE PATH "Installation root for lime::getLimeSuiteRoot()")
file(TO_CMAKE_PATH "${LIME_SUITE_ROOT}" LIME_SUITE_ROOT)

########################################################################
## compiler flags
########################################################################

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#is this processor x86? set variable X86
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*|amd64.*|AMD64.*")
    set(X86 TRUE)
else()
    set(X86 FALSE)
endif()

if(CMAKE_SYSTEM_PROCESSOR MATCHES "riscv64")
    set(RV64 TRUE)
else()
    set(RV64 FALSE)
endif()

# Don't enable LTO/IPO, it generates bugs in PCIe DMA controls
# if(NOT (CMAKE_BUILD_TYPE STREQUAL "Debug"))
#     # Optional IPO. Do not use IPO if it's not supported by compiler.
#     include(CheckIPOSupported)
#     check_ipo_supported(RESULT result OUTPUT output)
#     if(result)
#         set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
#         if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#             add_link_options(-flto=auto)
#         endif()
#     else()
#         message(WARNING "IPO is not supported: ${output}")
#     endif()
# endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    #enable warnings
    add_compile_options(-Wall -Wpedantic -fmax-errors=5)

    #symbols are only exported from libraries/modules explicitly
    add_compile_options(-fvisibility=hidden)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)

    # enable C-style cast warnings in C++
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>)

    #disable gcc caller-saves flag for O2-O3 optimizations
    #workaround fix for gcc 9.3+
    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.3 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 9.9)
        add_compile_options($<$<NOT:$<CONFIG:Debug>>:-fno-caller-saves>)
    endif()

    if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64") # limit CPU extensions usage to generate portable binaries
        add_compile_options(-march=x86-64)
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    #enable warnings
    add_compile_options(-Wall -Wpedantic)
endif()

if(MSVC)
    # Check if using correct Visual Studio version
    # VS2015 does not support some of the used C++17 features (<string_view>, <filesystem>, ...)
    if(MSVC_TOOLSET_VERSION LESS 141)
        message(FATAL_ERROR "MSVC version should be atleast v141 (Visual Studio 2017)")
    endif()

    add_compile_definitions(_CRT_SECURE_NO_WARNINGS NOMINMAX)
    add_compile_options("/MP" "/Zc:__cplusplus")

    set(ENABLE_SIMD_FLAGS
        "SSE2"
        CACHE STRING "Set compiler SIMD flags")
    set_property(CACHE ENABLE_SIMD_FLAGS PROPERTY STRINGS none SSE2 AVX AVX2)

    if(${ENABLE_SIMD_FLAGS} MATCHES "AVX2")
        add_compile_options(/arch:AVX2)
        message(STATUS "Enabling AVX2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX")
        add_compile_options(/arch:AVX)
        message(STATUS "Enabling AVX instructions")
        # MSVC default mode already uses SSE2
        # elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE2")
        #     add_compile_options(/arch:SSE2)
        #     message(STATUS "Enabling SSE2 instructions")
    endif()
endif(MSVC)

if(APPLE)
    add_compile_definitions(__unix__=1) #we use this for unix detection, but clang does not define it
endif(APPLE)

###########################################################################################
# rpath setup - https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
###########################################################################################
# use, i.e. don't skip the full RPATH for the build tree
option(CMAKE_SKIP_BUILD_RPATH "skip rpath build" FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
option(CMAKE_BUILD_WITH_INSTALL_RPATH "build with install rpath" FALSE)

# the RPATH to be used when installing, but only if it's not a system directory
option(CMAKE_AUTOSET_INSTALL_RPATH TRUE)
if(CMAKE_AUTOSET_INSTALL_RPATH)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")
    endif("${isSystemDir}" STREQUAL "-1")
endif(CMAKE_AUTOSET_INSTALL_RPATH)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
option(CMAKE_INSTALL_RPATH_USE_LINK_PATH "build with automatic rpath" TRUE)

if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
endif()

set(CPP_STL_PCH
    <array>
    <atomic>
    <algorithm>
    <cassert>
    <chrono>
    <cmath>
    <cstdint>
    <cstring>
    <filesystem>
    <fstream>
    <functional>
    <iostream>
    <list>
    <limits>
    <random>
    <set>
    <string>
    <string_view>
    <map>
    <memory>
    <mutex>
    <thread>
    <unordered_map>
    <vector>
    <queue>)

########################################################################
# uninstall target
########################################################################
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE
    @ONLY)

#only add uninstall target if this is the top project
if(TARGET uninstall)
    message(AUTHOR_WARNING "Parent project has already defined 'uninstall' target, limesuiteng will not have it's own uninstall.")
elseif(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()

#########################################################################
# externals
#########################################################################
add_subdirectory(external)

#########################################################################
# drivers
#########################################################################
option(BUILD_DRIVERS "Build device drivers" ON)
add_feature_info("BUILD_DRIVERS" BUILD_DRIVERS "Build device drivers")
if(BUILD_DRIVERS)
    add_subdirectory(drivers)
endif()

########################################################################
## lime suite ng build
########################################################################
option(BUILD_CORE "Build limesuiteng core library" ON)
add_feature_info("BUILD_CORE" BUILD_CORE "limesuiteng core library")
if(BUILD_CORE)
    add_subdirectory(embedded)
    add_subdirectory(src)
endif()

########################################################################
# Lime command line tools
########################################################################
cmake_dependent_option(BUILD_CLI "Build command line interface applications" ON "BUILD_CORE" OFF)
add_feature_info("BUILD_CLI" BUILD_CLI "command line interface applications")
if(BUILD_CLI)
    add_subdirectory(cli)
endif()

########################################################################
## graphical interface
########################################################################
cmake_dependent_option(BUILD_GUI "Build limeGUI application" ON "BUILD_CORE" OFF)
add_feature_info("BUILD_GUI" BUILD_GUI "graphical user interface application (limeGUI)")
if(BUILD_GUI)
    add_subdirectory(GUI)
endif()

#########################################################################
# GTest tests
#########################################################################
option(ENABLE_UNIT_TESTS "Use GoogleTest to test the library" OFF)
if(ENABLE_UNIT_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

########################################################################
## plugins for external programs
########################################################################
cmake_dependent_option(BUILD_PLUGINS "Build LimeSuiteNG plugins for external software packages" ON "BUILD_CORE" OFF)
add_feature_info("BUILD_PLUGINS" BUILD_PLUGINS "plugins for external software packages")
if(BUILD_PLUGINS)
    add_subdirectory(plugins)
endif()

########################################################################
## udev rules for linux usb
########################################################################
cmake_dependent_option(INSTALL_UDEV_RULES "Install Linux udev rules for LimeSDR devices" ON "UNIX" OFF)
add_feature_info("INSTALL_UDEV_RULES" INSTALL_UDEV_RULES "Install Linux udev rules for LimeSDR devices")
if(INSTALL_UDEV_RULES)
    add_subdirectory(udev-rules)
endif()

########################################################################
## Doxygen
########################################################################
option(BUILD_DOCS "Generate documentation" ON)
add_feature_info("BUILD_DOCS" BUILD_DOCS "Build documentation")
if(BUILD_DOCS)
    add_subdirectory(docs/doxygen)
endif()

#########################################################################
# summary
#########################################################################
message(STATUS "")
message(STATUS "######################################################")
feature_summary(WHAT PACKAGES_FOUND DESCRIPTION "Packages found:")

message(STATUS "######################################################")
feature_summary(WHAT REQUIRED_PACKAGES_NOT_FOUND DESCRIPTION "Missing required packages:")

message(STATUS "######################################################")
feature_summary(WHAT RECOMMENDED_PACKAGES_NOT_FOUND DESCRIPTION "Missing recommended packages:")

message(STATUS "######################################################")
feature_summary(WHAT OPTIONAL_PACKAGES_NOT_FOUND DESCRIPTION "Missing optional packages:")

message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} enabled features")
message(STATUS "######################################################")
feature_summary(WHAT ENABLED_FEATURES)
message(STATUS "######################################################")
message(STATUS "## ${PROJECT_NAME} disabled features")
message(STATUS "######################################################")
feature_summary(WHAT DISABLED_FEATURES)
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Build timestamp: ${BUILD_TIMESTAMP}")
message(STATUS "Lime Suite version: ${LIME_SUITE_VERSION}")
message(STATUS "ABI/so version: ${LIME_SUITE_SOVER}")

# Common packaging info, must be set before 'include(CPack)'
include(packages/CPackProperties.cmake)

include(CPack)

# Uses cpack_* functions, that are available only after 'include(CPack)'
include(packages/CPackComponentsMeta.cmake)
