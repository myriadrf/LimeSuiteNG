########################################################################
## public headers
########################################################################
file(
    GLOB LIMESUITENG_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/lime/*.h")
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_HEADERS "Enable install headers" ON "TRUE" OFF)
add_feature_info(HEADERS ENABLE_HEADERS "The limesuiteng headers")
if(ENABLE_HEADERS)
    install(FILES ${LIMESUITENG_PUBLIC_HEADERS} DESTINATION include/limesuiteng)
endif()

########################################################################
## lime suite library
########################################################################
set(LIME_SUITE_SOURCES
    StreamComposite.cpp
    CommonFunctions.cpp
    OEMTesting.cpp
    logger/Logger.cpp
    logger/LoggerCString.cpp
    protocols/LMS64CProtocol.cpp
    protocols/TRXLooper.cpp
    protocols/BufferInterleaving.cpp
    protocols/AvgRmsCounter.cpp
    FPGA/FPGA_common.cpp
    FPGA/WriteRegistersBatch.cpp
    threadHelper/threadHelper.cpp
    memory/MemoryPool.cpp
    API/LMS_APIWrapper.cpp
    API/LimePlugin.cpp
    include/limesuiteng/SDRDevice.cpp
    utilities/toString.cpp)

set_source_files_properties(mcu_program/common_src/lms7002m_controls.c PROPERTIES LANGUAGE CXX)

set(LIME_SUITE_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR} include boards protocols lms7002m external threadHelper memory vectorization)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/VersionInfo.in.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp
    @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SystemResources.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

include(FeatureSummary)
include(CMakeDependentOption)

option(ENABLE_LIBRARY "Enable build library" ON)
add_feature_info(LIBRARY ENABLE_LIBRARY "The limesuiteng library")
if(NOT ENABLE_LIBRARY)
    return()
endif(NOT ENABLE_LIBRARY)

add_library(limesuiteng ${LIME_SUITE_SOURCES})
set_target_properties(
    limesuiteng
    PROPERTIES POSITION_INDEPENDENT_CODE TRUE
               SOVERSION ${LIME_SUITE_SOVER}
               VERSION ${LIME_SUITE_LIBVER}
               DEFINE_SYMBOL "LIME_DLL_EXPORTS")

if(BUILD_SHARED_LIBS)
    target_compile_definitions(limesuiteng PUBLIC "LIME_DLL")
endif()

include(CheckAtomic)
if(NOT HAVE_CXX_ATOMICS_WITHOUT_LIB OR NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
    target_link_libraries(limesuiteng PUBLIC atomic)
endif()

find_package(Threads REQUIRED)
target_link_libraries(
    limesuiteng
    PUBLIC Threads::Threads
    PRIVATE $<BUILD_INTERFACE:kissfft> rang $<BUILD_INTERFACE:cpp-feather-ini-parser>)

target_include_directories(
    limesuiteng
    PRIVATE ${LIME_SUITE_INCLUDES}
    INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
              $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
              $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external>
              $<INSTALL_INTERFACE:include>)

if(CMAKE_BUILD_TYPE STREQUAL "Debug"
   AND ENABLE_CODE_COVERAGE
   AND (NOT MSVC))
    include(CodeCoverage)
    append_coverage_compiler_flags_to_target(limesuiteng)

    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
        target_link_options(limesuiteng PUBLIC --coverage)
    endif()
endif()

install(
    TARGETS limesuiteng
    EXPORT limesuitengTarget
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin # .dll file
)

# comms has to be processed before boards, to set up cmake variables needed for board support
add_subdirectory(comms)
add_subdirectory(boards)
add_subdirectory(chips)
add_subdirectory(DSP)
add_subdirectory(examples)
add_subdirectory(utilities)
add_subdirectory(vectorization)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/boards/BoardSupportLoader.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)
target_sources(limesuiteng PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)

########################################################################
# Build pkg config file
########################################################################
if(ENABLE_HEADERS AND ENABLE_LIBRARY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng.pc.in ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc @ONLY)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc DESTINATION lib${LIB_SUFFIX}/pkgconfig)
endif(ENABLE_HEADERS AND ENABLE_LIBRARY)
