########################################################################
## Create LimeSuiteNG project
########################################################################
cmake_minimum_required(VERSION 3.15)
set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release;RelWithDebInfo"
    CACHE STRING INTERNAL FORCE)
project("LimeSuiteNG" LANGUAGES C CXX)

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
option(ENABLE_NEW_GAIN_BEHAVIOUR "Enable new gain functionality" OFF)

#include modules for finding CyAPI
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

list(FIND CMAKE_CONFIGURATION_TYPES "${CMAKE_BUILD_TYPE}" index)
if(${index} EQUAL -1)
    set(CMAKE_BUILD_TYPE "Release")
endif()

########################################################################
# Gather version information from the changelog
# LIMESUITENG_API_VERSION in VersionInfo.h reflects the major/minor
########################################################################
file(READ "${PROJECT_SOURCE_DIR}/Changelog.txt" changelog_txt)
string(REGEX MATCH "Release ([0-9]+)\\.([0-9]+)\\.([0-9]+) \\(" CHANGELOG_MATCH "${changelog_txt}")
if(NOT CHANGELOG_MATCH)
    message(FATAL_ERROR "Failed to extract version number from Changelog.txt")
endif(NOT CHANGELOG_MATCH)
set(VERSION_MAJOR "${CMAKE_MATCH_1}")
set(VERSION_MINOR "${CMAKE_MATCH_2}")
set(VERSION_PATCH "${CMAKE_MATCH_3}")

# verify that VersionInfo.h LIMESUITENG_API_VERSION define matches changelog
file(READ "${PROJECT_SOURCE_DIR}/src/include/limesuiteng/VersionInfo.h" version_info_header_txt)
string(REGEX MATCH "LIMESUITENG_API_VERSION 0x([A-Fa-f0-9]*)" VERSION_DEFINE_MATCH "${version_info_header_txt}")
if(NOT VERSION_DEFINE_MATCH)
    message(FATAL_ERROR "Failed to extract API version from VersionInfo.h")
endif()
math(EXPR VERSION_MAJOR_IN_HEADER "(0x${CMAKE_MATCH_1} >> 24) & 0xFF" OUTPUT_FORMAT DECIMAL)
math(EXPR VERSION_MINOR_IN_HEADER "(0x${CMAKE_MATCH_1} >> 16) & 0xFF" OUTPUT_FORMAT DECIMAL)
math(EXPR VERSION_PATCH_IN_HEADER "(0x${CMAKE_MATCH_1}) & 0xFFFF" OUTPUT_FORMAT DECIMAL)
if(NOT ${VERSION_MAJOR} EQUAL ${VERSION_MAJOR_IN_HEADER}
   OR NOT ${VERSION_MINOR} EQUAL ${VERSION_MINOR_IN_HEADER}
   OR NOT ${VERSION_PATCH} EQUAL ${VERSION_PATCH_IN_HEADER})
    message(
        FATAL_ERROR
            "Changelog release version (${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH})"
            " don't match header API version (${VERSION_MAJOR_IN_HEADER}.${VERSION_MINOR_IN_HEADER}.${VERSION_PATCH_IN_HEADER})")
endif()

# the ABI compatibility number should be incremented when the ABI changes
# the format is to use the same major and minor, but to have an incrementing
# number if there are changes within the major.minor release series
set(LIME_SUITE_SOVER "${VERSION_MAJOR}.${VERSION_MINOR}-1")

# packagers may specify -DLIME_SUITE_EXTVER="foo" to replace the git hash
if(NOT LIME_SUITE_EXTVER)
    include(${PROJECT_SOURCE_DIR}/cmake/GetGitRevisionDescription.cmake)
    get_git_head_revision(GITREFSPEC GITHASH)
    if(GITHASH)
        string(SUBSTRING "${GITHASH}" 0 8 GITHASH)
        set(LIME_SUITE_EXTVER "g${GITHASH}")
    else(GITHASH)
        set(LIME_SUITE_EXTVER "unknown")
    endif(GITHASH)
endif()

set(LIME_SUITE_LIBVER "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")
set(LIME_SUITE_VERSION "${LIME_SUITE_LIBVER}-${LIME_SUITE_EXTVER}")
string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d")

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

if(NOT (CMAKE_BUILD_TYPE STREQUAL "Debug"))
    # Optional IPO. Do not use IPO if it's not supported by compiler.
    include(CheckIPOSupported)
    check_ipo_supported(RESULT result OUTPUT output)
    if(result)
        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
        if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            add_link_options(-flto=auto)
        endif()
    else()
        message(WARNING "IPO is not supported: ${output}")
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    #enable warnings in Debug configurations
    add_compile_options(-Wall -Wpedantic)

    #symbols are only exported from libraries/modules explicitly
    add_compile_options(-fvisibility=hidden)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fvisibility-inlines-hidden>)

    # enable C-style cast warnings in C++
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>)

    #disable gcc caller-saves flag for O2-O3 optimizations
    #workaround fix for gcc 9.3+
    if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
        if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.3 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 9.9)
            add_compile_options(-fno-caller-saves)
        endif()
    endif()

    #default SIMD configuration uses native build flags
    #when packaging and x86, use sse3 so the binaries work across multiple x86 variants
    if(NOT DEFAULT_SIMD_FLAGS)
        set(DEFAULT_SIMD_FLAGS "native")
    endif()
    if("${CMAKE_INSTALL_PREFIX}" STREQUAL "/usr" AND X86)
        set(DEFAULT_SIMD_FLAGS "SSE3")
    endif()
    if(RV64)
        set(DEFAULT_SIMD_FLAGS "rv64")
    endif()

    set(ENABLE_SIMD_FLAGS
        "${DEFAULT_SIMD_FLAGS}"
        CACHE STRING "Set compiler SIMD flags")
    set_property(CACHE ENABLE_SIMD_FLAGS PROPERTY STRINGS none native SSE3 SSE4.2 AVX AVX2)

    if(${ENABLE_SIMD_FLAGS} MATCHES "native")
        add_compile_options(-march=native)
        if(X86)
            add_compile_options(-mfpmath=sse)
        endif()
        message(STATUS "Enabling native optimizations")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX2")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfma -mavx2 -mfpmath=sse)
        message(STATUS "Enabling AVX2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfpmath=sse)
        message(STATUS "Enabling AVX instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE4.2")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mfpmath=sse)
        message(STATUS "Enabling SSE4.2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE3")
        add_compile_options(-mmmx -msse -msse2 -msse3 -mfpmath=sse)
        message(STATUS "Enabling SSE3 instructions")
    endif()

endif()

if(MSVC)
    # Check if using correct Visual Studio version
    # VS2015 does not support some of the used C++17 features (<string_view>, <filesystem>, ...)
    if(MSVC_TOOLSET_VERSION LESS 141)
        message(FATAL_ERROR "MSVC version should be atleast v141 (Visual Studio 2017)")
    endif()

    include_directories(${PROJECT_SOURCE_DIR}/external/msvc)
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

if(ENABLE_NEW_GAIN_BEHAVIOUR)
    add_compile_definitions(NEW_GAIN_BEHAVIOUR)
endif()

########################################################################
# GTest
########################################################################
include(CMakeDependentOption)
include(FeatureSummary)
cmake_dependent_option(ENABLE_TESTING "Use GoogleTest to test the library" ON "NOT BUILD_SHARED_LIBS" OFF)
add_feature_info(TESTING ENABLE_TESTING "Build GoogleTest tests to test the library")
cmake_dependent_option(ENABLE_CODE_COVERAGE "Use gcov/lcov to generate a code coverage report" ON "ENABLE_TESTING" OFF)

if(ENABLE_TESTING)
    include(CTest)
    include(FetchContent)
    fetchcontent_declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG eff443c6ef5eb6ab598bfaae27f9427fdb4f6af7 # latest main at the time of writing this
    )

    set(INSTALL_GMOCK OFF) # Don't install the GMock headers
    set(INSTALL_GTEST OFF) # Don't install the GTest headers

    if(MSVC)
        set(gtest_force_shared_crt
            ON
            CACHE BOOL "" FORCE)
    endif()

    fetchcontent_makeavailable(googletest)
    enable_testing()
endif()

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

#########################################################################
# externals
#########################################################################
add_subdirectory(external)

########################################################################
## lime suite ng build
########################################################################
add_subdirectory(src)

########################################################################
## plugins for external programs
########################################################################
add_subdirectory(plugins)

########################################################################
## udev rules for linux usb
########################################################################
add_subdirectory(udev-rules)

########################################################################
## Doxygen
########################################################################
find_package(Doxygen OPTIONAL_COMPONENTS dot mscgen dia)
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_API_DOXYGEN "Enable LMS API Doxygen documentation" ON "DOXYGEN_FOUND" OFF)
add_feature_info(API_DOXYGEN ENABLE_API_DOXYGEN "LMS API Doxygen documentation")
if(ENABLE_API_DOXYGEN)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/DoxyfileAPI.in ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
    add_custom_target(
        docAPI ALL
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating LMS API documentation with Doxygen"
        VERBATIM)
endif()

if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(
        doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating documentation with Doxygen"
        VERBATIM)
endif()

########################################################################
# CMake Project config files
########################################################################
if(ENABLE_HEADERS AND ENABLE_LIBRARY)
    configure_file(
        ${PROJECT_SOURCE_DIR}/cmake/Modules/limesuitengConfigVersion.in.cmake
        ${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake
        @ONLY)

    install(
        EXPORT limesuiteng
        FILE limesuitengConfig.cmake
        DESTINATION lib${LIB_SUFFIX}/cmake/${PROJECT_NAME})

    install(FILES ${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake DESTINATION lib${LIB_SUFFIX}/cmake/${PROJECT_NAME})
endif(ENABLE_HEADERS AND ENABLE_LIBRARY)

#########################################################################
# summary
#########################################################################
include(FeatureSummary)
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
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/DownloadImages.cmake)

########################################################################
# uninstall target
########################################################################
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE
    @ONLY)

#only add uninstall target if this is the top project
if(${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif()
