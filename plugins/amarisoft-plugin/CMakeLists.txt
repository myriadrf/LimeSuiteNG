project("limesuiteng-amarisoft-plugin")
cmake_minimum_required(VERSION 3.5)
set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release;RelWithDebInfo"
    CACHE STRING INTERNAL FORCE)
set(BUILD_SHARED_LIBS "Build the shared library" ON)
set(CMAKE_BUILD_TYPE "Release")

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_AMARISOFT_PLUGIN "Build LimeSuite integration to Amarisoft" ON "UNIX;ENABLE_LIBRARY" OFF)
add_feature_info(AMARISOFT_PLUGIN ENABLE_AMARISOFT_PLUGIN "LimeSuite Amarisoft integration plugin")

if(NOT ENABLE_AMARISOFT_PLUGIN)
    return()
endif()

#is this processor x86? set variable X86
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*|amd64.*|AMD64.*")
    set(X86 TRUE)
else()
    set(X86 FALSE)
endif()

set(AMARISOFT_PLUGIN_TARGET_NAME trx_limesuite)
add_library(${AMARISOFT_PLUGIN_TARGET_NAME} trx_limesuite.cpp common.cpp)

if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -Wall -Wpedantic)
    target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)

    if(NOT DEFAULT_SIMD_FLAGS)
        set(DEFAULT_SIMD_FLAGS "native")
    endif()
    if("${CMAKE_INSTALL_PREFIX}" STREQUAL "/usr" AND X86)
        set(DEFAULT_SIMD_FLAGS "SSE3")
    endif()

    set(ENABLE_SIMD_FLAGS
        "${DEFAULT_SIMD_FLAGS}"
        CACHE STRING "Set compiler SIMD flags")
    set_property(CACHE ENABLE_SIMD_FLAGS PROPERTY STRINGS none native SSE3 SSE4.2 AVX AVX2)

    if(${ENABLE_SIMD_FLAGS} MATCHES "native")
        target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -march=native)
        if(X86)
            target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -mfpmath=both)
        endif()
        message(STATUS "Enabling native optimizations")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX2")
        target_compile_options(
            ${AMARISOFT_PLUGIN_TARGET_NAME}
            PRIVATE -mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfma -mavx2 -mfpmath=both)
        message(STATUS "Enabling AVX2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "AVX")
        target_compile_options(
            ${AMARISOFT_PLUGIN_TARGET_NAME}
            PRIVATE -mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mfpmath=both)
        message(STATUS "Enabling AVX instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE4.2")
        target_compile_options(
            ${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -mmmx -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mfpmath=both)
        message(STATUS "Enabling SSE4.2 instructions")
    elseif(${ENABLE_SIMD_FLAGS} MATCHES "SSE3")
        target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -mmmx -msse -msse2 -msse3 -mfpmath=both)
        message(STATUS "Enabling SSE3 instructions")
    endif()
endif()

target_link_libraries(${AMARISOFT_PLUGIN_TARGET_NAME} limesuiteng)
target_link_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE "LINKER:-z,defs")

#remove lib prefix, Amarisoft searches for trx_*.so
set_target_properties(${AMARISOFT_PLUGIN_TARGET_NAME} PROPERTIES PREFIX "")
