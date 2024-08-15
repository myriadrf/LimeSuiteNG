set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if(NOT TARGET OpenGL::GL)
    message(FATAL_ERROR "OpenGL/GL not found!")
endif(NOT TARGET OpenGL::GL)
message(STATUS "OPENGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
message(STATUS "OPENGL_LIBRARIES: ${OPENGL_LIBRARIES}")

target_sources(limeGUI PRIVATE dlgMarkers.cpp GLFont.cpp OpenGLGraph.cpp)

target_include_directories(limeGUI PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OPENGL_INCLUDE_DIR})

if(APPLE)
    set(GL_LIBS "-framework OpenGL" "-framework GLUT")
elseif(UNIX)
    set(GL_LIBS ${OPENGL_LIBRARIES})
endif()

target_link_libraries(limeGUI PRIVATE GLEW ${GL_LIBS})
