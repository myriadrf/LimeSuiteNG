
# logic to be used by CLI and GUI
add_library(FDQI-design STATIC
	FDQIEconfig.cpp
	FDQIEconfig2.cpp
	FDQIEdesign.cpp
	FDQIspi.cpp
)

target_compile_options(FDQI-design PRIVATE -Wno-old-style-cast)
target_include_directories(FDQI-design PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(FDQI-design
	PUBLIC lime-nrc lime-qfft limesuiteng cpp-feather-ini-parser
)


add_executable(equAPI equAPI.cpp)
target_link_libraries(equAPI PRIVATE cli-shared FDQI-design)
set_target_properties(equAPI PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
