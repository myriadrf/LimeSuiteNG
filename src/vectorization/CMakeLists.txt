if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall -Wpedantic)
    add_compile_options(-O3 -march=native)
    #add_compile_options(-O3 -march=native -fopt-info-vec-all)
endif()

add_library(samplesConversion STATIC samplesConversion.cpp)

add_executable(samplesConversionPerfTest main.cpp)
target_link_libraries(samplesConversionPerfTest samplesConversion)
