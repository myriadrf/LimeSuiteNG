target_sources(limesuiteng PRIVATE samplesConversion.cpp TRXLooper.cpp BufferInterleaving.cpp AvgRmsCounter.cpp)

add_executable(samplesConversionPerfTest main.cpp samplesConversion.cpp)
target_link_libraries(samplesConversionPerfTest limesuiteng)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(samplesConversionPerfTest PUBLIC -Wall -Wpedantic -O3 -march=native)
    #target_compile_options(samplesConversionPerfTest PUBLIC -fopt-info-vec-all)
endif()
