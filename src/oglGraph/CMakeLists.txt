set(oglGraph_src_files dlgMarkers.cpp GLFont.cpp OpenGLGraph.cpp)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if(NOT TARGET OpenGL::GL)
    message(FATAL_ERROR "OpenGL/GL not found!")
endif(NOT TARGET OpenGL::GL)
message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")

add_library(oglGraph STATIC ${oglGraph_src_files})

target_include_directories(
    oglGraph
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${OPENGL_INCLUDE_DIR} # public, because <wx/glcanvas.h> includes GL/gl.h but the include dir is not added by wxWidgets
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Ignore warnings about narrowing: static font header
    target_compile_options(oglGraph PRIVATE -Wno-narrowing -Wno-address)
endif()

if(APPLE)
    set(GL_LIBS "-framework OpenGL" "-framework GLUT")
elseif(UNIX)
    set(GL_LIBS ${OPENGL_LIBRARIES})
endif()

target_link_libraries(oglGraph PUBLIC GLEW ${GL_LIBS} ${wxWidgets_LIBRARIES})
