project(
    "limesuiteng-cli"
    DESCRIPTION "LimeSuiteNG Command Line Interface applications"
    VERSION 1.0.0
    LANGUAGES CXX)

checkdebianchangelogversion(${PROJECT_NAME})

# common functionality among cli tools
add_library(cli-shared STATIC common.cpp)
target_link_libraries(cli-shared PUBLIC limesuiteng taywee-args)

target_precompile_headers(cli-shared PRIVATE ${CPP_STL_PCH} \"args.hxx\")

add_executable(limeDevice limeDevice.cpp)
set_target_properties(limeDevice PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeDevice PRIVATE cli-shared)
target_precompile_headers(limeDevice REUSE_FROM cli-shared)

add_executable(limeSPI limeSPI.cpp)
set_target_properties(limeSPI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeSPI PRIVATE cli-shared)
target_precompile_headers(limeSPI REUSE_FROM cli-shared)

add_executable(limeI2C limeI2C.cpp)
set_target_properties(limeI2C PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeI2C PRIVATE cli-shared)
target_precompile_headers(limeI2C REUSE_FROM cli-shared)

add_executable(limeConfig limeConfig.cpp)
set_target_properties(limeConfig PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeConfig PRIVATE cli-shared)
target_precompile_headers(limeConfig REUSE_FROM cli-shared)

add_executable(limeFLASH limeFLASH.cpp)
set_target_properties(limeFLASH PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeFLASH PRIVATE cli-shared)
target_precompile_headers(limeFLASH REUSE_FROM cli-shared)

add_executable(limeTRX limeTRX.cpp)
set_target_properties(limeTRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeTRX PRIVATE cli-shared nlohmann_json)
target_precompile_headers(limeTRX REUSE_FROM cli-shared)

add_executable(limeOEM limeOEM.cpp)
set_target_properties(limeOEM PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeOEM PRIVATE cli-shared)
target_precompile_headers(limeOEM REUSE_FROM cli-shared)

install(
    TARGETS limeDevice limeSPI limeFLASH limeTRX limeConfig limeOEM limeI2C
    COMPONENT cli
    DESTINATION bin)

add_subdirectory(utilities)
add_subdirectory(iq_monitor)
