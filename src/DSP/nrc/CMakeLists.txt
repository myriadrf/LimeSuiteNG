add_library(lime-nrc STATIC)

target_compile_options(lime-nrc PRIVATE -Wno-old-style-cast)
set_property(TARGET lime-nrc PROPERTY POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(lime-nrc PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_sources(lime-nrc PRIVATE
	amphgd.cpp
	brent.cpp
	cost.cpp
	dfpmin.cpp
	f1dim.cpp
	linmin.cpp
	mnbrak.cpp
	nrc.cpp
	vector_matrix.cpp
)