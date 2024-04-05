# Compile needed externals

#########################################################################
# GLEW
#########################################################################
set(GLEW_PATH "${PROJECT_SOURCE_DIR}/external/glew")

add_library(GLEW STATIC ${GLEW_PATH}/glew.c)
target_include_directories(GLEW PUBLIC ${GLEW_PATH})

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Ignore warnings about narrowing: static font header
    target_compile_options(GLEW PRIVATE -Wno-address)
endif()

#########################################################################
# kissFFT
#########################################################################
set(kissFFT_PATH "${PROJECT_SOURCE_DIR}/external/kissFFT")

add_library(kissFFT STATIC ${kissFFT_PATH}/kiss_fft.c)
target_include_directories(kissFFT PUBLIC ${kissFFT_PATH})
