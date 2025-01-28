include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(BUILD_EXAMPLES "Enable library example programs" ON "BUILD_CORE" OFF)
add_feature_info("BUILD_EXAMPLES" BUILD_EXAMPLES "LimeSuiteNG API examples")
if(NOT BUILD_EXAMPLES)
    return()
endif()

find_package(kissfft CONFIG REQUIRED)

if(UNIX)
    find_package(Gnuplot)
    set_package_properties(Gnuplot PROPERTIES TYPE OPTIONAL)
    if(GNUPLOT_FOUND)
        add_compile_definitions(USE_GNU_PLOT)
    endif()
endif()

set(examplesOutputDir "${CMAKE_BINARY_DIR}/bin/examples")

add_executable(basicRX basicRX.cpp)
set_target_properties(basicRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir})
target_link_libraries(
    basicRX
    PRIVATE limesuiteng
    PRIVATE cli-shared kissfft)

add_executable(dualRXTX dualRXTX.cpp)
set_target_properties(dualRXTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir})
target_link_libraries(dualRXTX PRIVATE limesuiteng)

add_executable(basicTX basicTX.cpp)
set_target_properties(basicTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir})
target_link_libraries(
    basicTX
    PRIVATE limesuiteng
    PRIVATE cli-shared kissfft taywee-args)

add_subdirectory(legacy)
