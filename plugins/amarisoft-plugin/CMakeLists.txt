if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
    cmake_minimum_required(VERSION 3.15)
endif()

project("limesuiteng-amarisoft-plugin")

message(STATUS "##### <${PROJECT_NAME}> #####")

set(CMAKE_CONFIGURATION_TYPES
    "Debug;Release;RelWithDebInfo"
    CACHE STRING INTERNAL FORCE)

#is this processor x86? set variable X86
if(CMAKE_SYSTEM_PROCESSOR MATCHES "i686.*|i386.*|x86.*|amd64.*|AMD64.*")
    set(X86 TRUE)
else()
    set(X86 FALSE)
endif()

set(AMARISOFT_PLUGIN_TARGET_NAME trx_limesuite)
add_library(${AMARISOFT_PLUGIN_TARGET_NAME} SHARED trx_limesuite.cpp)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -Wall -Wpedantic)
    target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
    target_compile_options(${AMARISOFT_PLUGIN_TARGET_NAME} PRIVATE "-z,defs")
endif()

target_link_libraries(${AMARISOFT_PLUGIN_TARGET_NAME} limesuiteng)

#remove lib prefix, Amarisoft searches for trx_*.so
set_target_properties(${AMARISOFT_PLUGIN_TARGET_NAME} PROPERTIES PREFIX "")

message(STATUS "##### </${PROJECT_NAME}> #####")
