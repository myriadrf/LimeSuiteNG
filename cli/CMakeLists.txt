include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_CLI "Enable command line programs" ON "ENABLE_LIBRARY" OFF)
add_feature_info(CLI ENABLE_CLI "LimeSuite command line interface tools")
if(NOT ENABLE_CLI)
    return()
endif()

# common functionality among cli tools
add_library(cli-shared STATIC common.cpp)
target_include_directories(cli-shared PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(cli-shared PUBLIC limesuiteng rang)

add_executable(limeDevice limeDevice.cpp)
set_target_properties(limeDevice PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeDevice PRIVATE cli-shared taywee::args)

add_executable(limeSPI limeSPI.cpp)
set_target_properties(limeSPI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeSPI PRIVATE cli-shared taywee::args)

add_executable(limeConfig limeConfig.cpp)
set_target_properties(limeConfig PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeConfig PRIVATE cli-shared taywee::args)

add_executable(limeFLASH limeFLASH.cpp)
set_target_properties(limeFLASH PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeFLASH PRIVATE cli-shared taywee::args)

add_executable(limeTRX limeTRX.cpp)
set_target_properties(limeTRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeTRX PRIVATE cli-shared kissfft taywee::args)

add_executable(limeOEM limeOEM.cpp)
set_target_properties(limeOEM PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeOEM PRIVATE cli-shared taywee::args)

install(TARGETS limeDevice limeSPI limeFLASH limeTRX limeConfig limeOEM DESTINATION bin)
