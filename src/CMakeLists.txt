set(MAIN_LIBRARY_NAME limesuiteng)

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
    FPGA_common/FPGA_common.cpp
    FPGA_common/WriteRegistersBatch.cpp
    threadHelper/threadHelper.cpp
    EnumToString.cpp
    memory/MemoryPool.cpp
    API/LMS_APIWrapper.cpp
    API/LimePlugin.cpp
    include/limesuiteng/SDRDevice.cpp
    utilities/toString.cpp)

set_source_files_properties(mcu_program/common_src/lms7002m_controls.c PROPERTIES LANGUAGE CXX)

set(LIME_SUITE_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/boards
    ${CMAKE_CURRENT_SOURCE_DIR}/protocols
    ${CMAKE_CURRENT_SOURCE_DIR}/lms7002m
    ${CMAKE_CURRENT_SOURCE_DIR}/FPGA_common
    ${PROJECT_SOURCE_DIR}/external
    ${CMAKE_CURRENT_SOURCE_DIR}/threadHelper
    ${CMAKE_CURRENT_SOURCE_DIR}/memory
    ${CMAKE_CURRENT_SOURCE_DIR}/vectorization)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/VersionInfo.in.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp
    @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/include/limesuiteng/VersionInfo.cpp)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SystemResources.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp @ONLY)
list(APPEND LIME_SUITE_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
list(APPEND LIME_SUITE_LIBRARIES Threads::Threads)

include(CheckAtomic)
if(NOT HAVE_CXX_ATOMICS_WITHOUT_LIB OR NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
    list(APPEND LIME_SUITE_LIBRARIES atomic)
endif()

include(FeatureSummary)
include(CMakeDependentOption)

option(ENABLE_LIBRARY "Enable build library" ON)
add_feature_info(LIBRARY ENABLE_LIBRARY "The limesuiteng library")
if(ENABLE_LIBRARY)
    add_library(${MAIN_LIBRARY_NAME} ${LIME_SUITE_SOURCES})
    target_link_libraries(
        ${MAIN_LIBRARY_NAME}
        PUBLIC ${LIME_SUITE_LIBRARIES}
        PRIVATE $<BUILD_INTERFACE:kissfft> $<BUILD_INTERFACE:samplesConversion> $<BUILD_INTERFACE:cpp-feather-ini-parser>)
    set_property(TARGET ${MAIN_LIBRARY_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES SOVERSION ${LIME_SUITE_SOVER})
    set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES VERSION ${LIME_SUITE_LIBVER})
    set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES DEFINE_SYMBOL "LIME_DLL_EXPORTS")
    target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${LIME_SUITE_INCLUDES})
    target_include_directories(
        ${MAIN_LIBRARY_NAME}
        INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external>
                  $<INSTALL_INTERFACE:include>)

    if(BUILD_SHARED_LIBS)
        target_compile_definitions(${MAIN_LIBRARY_NAME} PUBLIC "LIME_DLL")
    endif()

    if(CMAKE_BUILD_TYPE STREQUAL "Debug"
       AND ENABLE_CODE_COVERAGE
       AND (NOT MSVC))
        include(CodeCoverage)
        append_coverage_compiler_flags_to_target(${MAIN_LIBRARY_NAME})

        if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_link_options(${MAIN_LIBRARY_NAME} PUBLIC --coverage)
        endif()
    endif()

    install(
        TARGETS ${MAIN_LIBRARY_NAME}
        EXPORT limesuiteng
        LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
        ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
        RUNTIME DESTINATION bin # .dll file
    )

    if(MSVC)
        set_target_properties(${MAIN_LIBRARY_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
    endif(MSVC)
endif()

########################################################################
## Supported connections
########################################################################
include(comms/USB/CMakeLists.txt)
include(comms/PCIe/CMakeLists.txt)
include(boards/CMakeLists.txt)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/boards/BoardSupportLoader.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)

target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)

add_subdirectory(chips)
add_subdirectory(DSP)
add_subdirectory(vectorization)

########################################################################
# Build pkg config file
########################################################################
if(ENABLE_HEADERS AND ENABLE_LIBRARY)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/LimeSuite.pc.in ${CMAKE_CURRENT_BINARY_DIR}/LimeSuite.pc @ONLY)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/LimeSuite.pc DESTINATION lib${LIB_SUFFIX}/pkgconfig)
endif(ENABLE_HEADERS AND ENABLE_LIBRARY)

#########################################################################
# utilities
#########################################################################
add_subdirectory(utilities)

#########################################################################
# examples
#########################################################################
add_subdirectory(examples)

#########################################################################
# Lime command line tools
#########################################################################
add_subdirectory(cli)

#########################################################################
# GTest tests
#########################################################################
if(ENABLE_TESTING)
    add_subdirectory(tests)
endif()
