#########################################################################
# GLEW
#########################################################################
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
if(TARGET OpenGL::GL) # GLEW header needs GL/gl.h
    # OSX has can have glew package, but it's includes fail.
    find_package(GLEW) # use system libs
    if(NOT GLEW_FOUND)
        fetchcontent_declare(
            GLEW_download
            URL https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz
            URL_HASH SHA256=d4fc82893cfb00109578d0a1a2337fb8ca335b3ceccf97b97e5cc7f08e4353e1
            EXCLUDE_FROM_ALL CONFIGURE_COMMAND "" BUILD_COMMAND "")
        fetchcontent_makeavailable(GLEW_download)
        fetchcontent_getproperties(GLEW_download SOURCE_DIR GLEW_PATH)
        add_library(GLEW STATIC ${GLEW_PATH}/src/glew.c)
        target_include_directories(GLEW PUBLIC ${GLEW_PATH}/include ${OPENGL_INCLUDE_DIR})
        target_compile_definitions(GLEW PUBLIC GLEW_STATIC GLEW_NO_GLU)

        if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            target_compile_options(GLEW PRIVATE -Wno-address -Wno-strict-prototypes)
        endif()

        # compile only when other targets link to it
        set_target_properties(GLEW PROPERTIES EXCLUDE_FROM_ALL TRUE EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    endif()
else()
    message(FATAL_ERROR "OpenGL/GL not found!")
endif(TARGET OpenGL::GL)
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
