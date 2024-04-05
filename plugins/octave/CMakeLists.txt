# LimeSuite Octave plugin compilation
########################################################################
## Feature registration
########################################################################
find_program(OCTAVE_CONFIG_EXECUTABLE NAMES octave-config)
if(NOT OCTAVE_CONFIG_EXECUTABLE)
    message(STATUS "octave-config not found (need liboctave-dev), disabling LimeSuiteOctave")
else()
    execute_process(
        COMMAND ${OCTAVE_CONFIG_EXECUTABLE} -p BINDIR
        OUTPUT_VARIABLE OCTAVE_BIN_PATHS
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    find_program(
        OCTAVE_MKOCTFILE_EXECUTABLE
        HINTS ${OCTAVE_BIN_PATHS}
        NAMES mkoctfile)
endif()

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(
    ENABLE_OCTAVE
    "Enable LimeSuiteNG library plugin for Octave"
    ON
    "ENABLE_LIBRARY;OCTAVE_CONFIG_EXECUTABLE;OCTAVE_MKOCTFILE_EXECUTABLE"
    OFF)
add_feature_info(LimeSuiteOctave ENABLE_OCTAVE "LimeSuiteNG Octave integration")

if(NOT ENABLE_OCTAVE)
    return()
endif()

add_custom_command(
    OUTPUT LimeSuiteNG.oct LimeSuiteNG.o ALL
    COMMAND
        ${OCTAVE_MKOCTFILE_EXECUTABLE} ARGS -I${PROJECT_SOURCE_DIR}/include/limesuiteng ${CMAKE_CURRENT_SOURCE_DIR}/LimeSuiteNG.cc
        -L$<TARGET_FILE_DIR:LimeSuiteNG> -llimesuiteng
    COMMENT "Building LimeSuite Octave plugin"
    DEPENDS limesuiteng)
add_custom_target(LimeSuiteNGOctave ALL DEPENDS LimeSuiteNG.oct)

#install script for preloading Limesuite.oct functions
execute_process(
    COMMAND ${OCTAVE_CONFIG_EXECUTABLE} --m-site-dir
    OUTPUT_VARIABLE OCTAVE_SCRIPTS_PATHS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/LoadLimeSuite.m DESTINATION ${OCTAVE_SCRIPTS_PATHS})

#install dynamic library
execute_process(
    COMMAND ${OCTAVE_CONFIG_EXECUTABLE} --oct-site-dir
    OUTPUT_VARIABLE OCTAVE_LIBS_PATHS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/LimeSuiteNG.oct DESTINATION ${OCTAVE_LIBS_PATHS})
