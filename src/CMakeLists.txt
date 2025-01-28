if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
    cmake_minimum_required(VERSION 3.15)
endif()

include(FeatureSummary)
include(CMakeDependentOption)
project(
    "liblimesuiteng"
    VERSION 0.3.0
    DESCRIPTION "Core library for interacting with LimeSDR based devices"
    LANGUAGES C CXX)
checkdebianchangelogversion(${PROJECT_NAME})

add_library(limesuiteng)
# namespaced target for use in plugins build
add_library(limesuiteng::limesuiteng ALIAS limesuiteng)

# the ABI compatibility number should be incremented when the ABI changes
# the format is to use the same major and minor, but to have an incrementing
# number if there are changes within the major.minor release series
set(LIMESUITENG_SOVER "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}-1")

# packagers may specify -DLIMESUITENG_EXTVER="foo" to replace the git hash
if(NOT LIMESUITENG_EXTVER)
    include(GetGitRevisionDescription)
    get_git_head_revision(GITREFSPEC GITHASH)
    if(GITHASH)
        string(SUBSTRING "${GITHASH}" 0 8 GITHASH)
        set(LIMESUITENG_EXTVER "g${GITHASH}")
    else(GITHASH)
        set(LIMESUITENG_EXTVER "unknown")
    endif(GITHASH)
endif()

set_target_properties(
    limesuiteng
    PROPERTIES POSITION_INDEPENDENT_CODE TRUE
               SOVERSION ${LIMESUITENG_SOVER}
               DEFINE_SYMBOL "LIME_DLL_EXPORTS")

if(BUILD_SHARED_LIBS)
    target_compile_definitions(limesuiteng PUBLIC "LIME_DLL")
endif()

# components
# runtime: just the shared library object
# dev: objects, headers, and other development files
# legacyapi: LMS_API wrapper of liblimesuiteng

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

option(INSTALL_DEVELOPMENT "Install limesuiteng headers & development files" ON)
add_feature_info("INSTALL_DEVELOPMENT" INSTALL_DEVELOPMENT "Install limesuiteng headers & development files")
if(INSTALL_DEVELOPMENT)
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc
        COMPONENT dev
        DESTINATION lib${LIB_SUFFIX}/pkgconfig)

    install(
        FILES ${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake
        COMPONENT dev
        DESTINATION lib${LIB_SUFFIX}/cmake/limesuiteng)

    install(
        EXPORT limesuitengTarget
        NAMESPACE limesuiteng::
        FILE limesuitengConfig.cmake
        DESTINATION lib${LIB_SUFFIX}/cmake/limesuiteng
        COMPONENT dev)
endif(INSTALL_DEVELOPMENT)

add_subdirectory(include)

option(ENABLE_NEW_GAIN_BEHAVIOUR "Enable new gain functionality" OFF)
if(ENABLE_NEW_GAIN_BEHAVIOUR)
    target_compile_definitions(limesuiteng PRIVATE NEW_GAIN_BEHAVIOUR)
endif()

########################################################################
# CMake Project config files
########################################################################
include(CMakePackageConfigHelpers)
write_basic_package_version_file(${PROJECT_BINARY_DIR}/limesuitengConfigVersion.cmake COMPATIBILITY SameMinorVersion)

########################################################################
# Build pkg config file
########################################################################
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng.pc.in ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng.pc @ONLY)

install(
    TARGETS limesuiteng
    EXPORT limesuitengTarget
    COMPONENT runtime
    DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin # .dll file
)

########################################################################
## lime suite library
########################################################################
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/SystemResources.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp @ONLY)
target_sources(
    limesuiteng
    PRIVATE CommonFunctions.cpp
            OEMTesting.cpp
            logger/Logger.cpp
            logger/LoggerCString.cpp
            protocols/LMS64CProtocol.cpp
            protocols/LMSBoards.cpp
            FPGA/FPGA_common.cpp
            FPGA/WriteRegistersBatch.cpp
            threadHelper/threadHelper.cpp
            memory/MemoryPool.cpp
            API/LimePlugin.cpp
            include/limesuiteng/SDRDevice.cpp
            utilities/toString.cpp
            ${CMAKE_CURRENT_BINARY_DIR}/SystemResources.cpp)

if(NOT MSVC)
    set_source_files_properties(include/limesuiteng/SDRDevice.cpp PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations)
endif()

set_source_files_properties(mcu_program/common_src/lms7002m_controls.c PROPERTIES LANGUAGE CXX)

target_precompile_headers(limesuiteng PRIVATE ${CPP_STL_CPH})

include(CheckAtomic)
if(NOT HAVE_CXX_ATOMICS_WITHOUT_LIB OR NOT HAVE_CXX_ATOMICS64_WITHOUT_LIB)
    target_link_libraries(limesuiteng PUBLIC atomic)
endif()

find_package(Threads REQUIRED)
target_link_libraries(
    limesuiteng
    PUBLIC Threads::Threads
    PRIVATE $<BUILD_INTERFACE:rang> $<BUILD_INTERFACE:cpp-feather-ini-parser>)

target_include_directories(
    limesuiteng
    PRIVATE threadHelper
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}> # so that GUI build can access internal headers
           $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
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

if(WIN32) # Windows don't have RPATH support, so need to place the DLLs into same directory as binaries
    set_target_properties(limesuiteng PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

# comms has to be processed before boards, to set up cmake variables needed for board support
add_subdirectory(comms)
add_subdirectory(boards)
add_subdirectory(chips)
add_subdirectory(DSP)
add_subdirectory(streaming)

########################################################################
# LMS_API Wrapper lib
# wrapper functions split into separate library to avoid symbols collision
# if both limesuiteng and LimeSuite would be linked to application.
########################################################################
option(BUILD_LEGACYAPI "Build legacy LMS_API" ON)
if(BUILD_LEGACYAPI)
    add_library(limesuiteng-legacyapi API/LMS_APIWrapper.cpp)
    add_library(limesuiteng::limesuiteng-legacyapi ALIAS limesuiteng-legacyapi)
    target_include_directories(
        limesuiteng-legacyapi PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> $<INSTALL_INTERFACE:include>)
    target_link_libraries(limesuiteng-legacyapi PUBLIC limesuiteng)

    install(
        TARGETS limesuiteng-legacyapi
        EXPORT limesuitengTarget
        COMPONENT legacyapi
        DESTINATION lib${LIB_SUFFIX} # .so file
        ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    )
endif()

add_subdirectory(examples)
