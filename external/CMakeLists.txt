# Compile needed externals

#########################################################################
# GLEW
#########################################################################
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL QUIET)
if(TARGET OpenGL::GL) # GLEW header needs GL/gl.h
    set(GLEW_PATH "${PROJECT_SOURCE_DIR}/external/glew")
    add_library(GLEW STATIC ${GLEW_PATH}/glew.c)
    target_include_directories(GLEW PUBLIC ${GLEW_PATH} ${OPENGL_INCLUDE_DIR})
    target_compile_definitions(GLEW PUBLIC GLEW_STATIC GLEW_NO_GLU)

    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(GLEW PRIVATE -Wno-address)
    endif()

    # compile only when other targets link to it
    set_target_properties(GLEW PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
endif()

#########################################################################
# kissFFT
#########################################################################
set(kissFFT_PATH "${PROJECT_SOURCE_DIR}/external/kissFFT")
set(KISSFFT_OPENMP OFF)
set(KISSFFT_PKGCONFIG OFF)
set(KISSFFT_STATIC OFF)
set(KISSFFT_TEST OFF)
set(KISSFFT_TOOLS OFF)
set(KISSFFT_USE_ALLOCA OFF)
cmake_policy(SET CMP0077 NEW)

add_subdirectory(kissFFT)
