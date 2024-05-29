include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_EXAMPLES "Enable library example programs" ON "ENABLE_LIBRARY" OFF)
add_feature_info(EXAMPLES ENABLE_EXAMPLES "LimeSuite library API examples")
if(NOT ENABLE_EXAMPLES)
    return()
endif()

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
    PUBLIC ${MAIN_LIBRARY_NAME}
    PRIVATE kissFFT)

add_executable(dualRXTX dualRXTX.cpp)
set_target_properties(dualRXTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir})
target_link_libraries(dualRXTX PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(basicTX basicTX.cpp)
set_target_properties(basicTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir})
target_link_libraries(basicTX PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(legacyBasicRX legacy/basicRX.cpp)
set_target_properties(legacyBasicRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME basicRX)
target_link_libraries(legacyBasicRX PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(legacyBasicTX legacy/basicTX.cpp)
set_target_properties(legacyBasicTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME basicTX)
target_link_libraries(legacyBasicTX PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(legacyDualRXTX legacy/dualRXTX.cpp)
set_target_properties(legacyDualRXTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME dualRXTX)
target_link_libraries(legacyDualRXTX PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(legacyGpio_example legacy/gpio_example.cpp)
set_target_properties(
    legacyGpio_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME gpio_example)
target_link_libraries(legacyGpio_example PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(legacySingleRX legacy/singleRX.cpp)
set_target_properties(legacySingleRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME singleRX)
target_link_libraries(legacySingleRX PUBLIC ${MAIN_LIBRARY_NAME})
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(legacySingleRX PRIVATE -Wno-unused-but-set-variable)
endif()
