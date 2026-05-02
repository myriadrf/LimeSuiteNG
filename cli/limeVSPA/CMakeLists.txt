add_executable(limeVSPA limeVSPA.cpp profiler.cpp)
set_target_properties(limeVSPA PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
target_link_libraries(limeVSPA PRIVATE cli-shared)
target_precompile_headers(limeVSPA REUSE_FROM cli-shared)
