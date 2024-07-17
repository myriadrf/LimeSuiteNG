add_library(lime-qfft STATIC)

target_compile_options(lime-qfft PRIVATE -Wno-old-style-cast)
set_property(TARGET lime-qfft PROPERTY POSITION_INDEPENDENT_CODE TRUE)

target_include_directories(lime-qfft PUBLIC ${CMAKE_CURRENT_LIST_DIR})
target_sources(lime-qfft PRIVATE
	qfft.cpp
	fourier.cpp
	fft.cpp
)