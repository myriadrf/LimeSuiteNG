add_executable(iqmonitor iq_mon.cpp iq_stats.cpp stats.cpp)
set_target_properties(iqmonitor PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(iqmonitor PRIVATE cli-shared)
