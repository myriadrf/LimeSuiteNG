if(NOT BUILD_LEGACYAPI)
    return()
endif()

set(examplesOutputDir "${CMAKE_BINARY_DIR}/bin/examples")

add_executable(legacyBasicRX basicRX.cpp)
set_target_properties(legacyBasicRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME basicRX)
target_link_libraries(legacyBasicRX PRIVATE limesuiteng-legacyapi)

add_executable(legacyBasicTX basicTX.cpp)
set_target_properties(legacyBasicTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME basicTX)
target_link_libraries(legacyBasicTX PRIVATE limesuiteng-legacyapi)

add_executable(legacyDualRXTX dualRXTX.cpp)
set_target_properties(legacyDualRXTX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME dualRXTX)
target_link_libraries(legacyDualRXTX PRIVATE limesuiteng-legacyapi)

add_executable(legacyGpio_example gpio_example.cpp)
set_target_properties(
    legacyGpio_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME gpio_example)
target_link_libraries(legacyGpio_example PRIVATE limesuiteng-legacyapi)

add_executable(legacySingleRX singleRX.cpp)
set_target_properties(legacySingleRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${examplesOutputDir}/legacy RUNTIME_OUTPUT_NAME singleRX)
target_link_libraries(legacySingleRX PRIVATE limesuiteng-legacyapi)
