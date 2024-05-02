include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_CLI "Enable command line programs" ON "ENABLE_LIBRARY" ON)
add_feature_info(CLI ENABLE_CLI "LimeSuite command line interface tools")
if(NOT ENABLE_CLI)
    return()
endif()

# common functionality among cli tools
add_library(cli-shared STATIC common.cpp)
target_include_directories(cli-shared PUBLIC ${LIME_SUITE_INCLUDES})
target_link_libraries(cli-shared PUBLIC ${MAIN_LIBRARY_NAME})

add_executable(limeDevice limeDevice.cpp)
set_target_properties(limeDevice PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeDevice PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeDevice PRIVATE cli-shared)

add_executable(limeSPI limeSPI.cpp)
set_target_properties(limeSPI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeSPI PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeSPI PRIVATE cli-shared)

add_executable(limeConfig limeConfig.cpp)
set_target_properties(limeConfig PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeConfig PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeConfig PRIVATE cli-shared)

add_executable(limeFLASH limeFLASH.cpp)
set_target_properties(limeFLASH PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeFLASH PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeFLASH PRIVATE cli-shared)

add_executable(limeTRX limeTRX.cpp)
set_target_properties(limeTRX PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeTRX PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeTRX PRIVATE cli-shared kissFFT)

add_executable(limeOEMTest limeOEMTest.cpp)
set_target_properties(limeOEMTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_include_directories(limeOEMTest PRIVATE ${LIME_SUITE_INCLUDES})
target_link_libraries(limeOEMTest PRIVATE cli-shared)

install(TARGETS limeDevice limeSPI limeFLASH limeTRX limeConfig limeOEMTest DESTINATION bin)
