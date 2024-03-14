# Compile needed externals

#########################################################################
# GLEW
#########################################################################
set(GLEW_PATH "${PROJECT_SOURCE_DIR}/external/glew")

add_library(GLEW STATIC ${GLEW_PATH}/glew.c)
target_include_directories(GLEW PUBLIC ${GLEW_PATH})

if(CMAKE_COMPILER_IS_GNUCXX)
    # Ignore warnings about narrowing: static font header
    target_compile_options(GLEW PRIVATE -Wno-address)
endif()

