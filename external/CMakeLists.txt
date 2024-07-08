# Download and compile needed externals

include(FetchContent)

#########################################################################
# GLEW
#########################################################################
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL QUIET)
if(TARGET OpenGL::GL) # GLEW header needs GL/gl.h
    fetchcontent_declare(
        GLEW_download
        URL https://github.com/nigels-com/glew/releases/download/glew-2.2.0/glew-2.2.0.tgz
        URL_HASH SHA256=d4fc82893cfb00109578d0a1a2337fb8ca335b3ceccf97b97e5cc7f08e4353e1
        CONFIGURE_COMMAND "" BUILD_COMMAND "")
    fetchcontent_makeavailable(GLEW_download)
    fetchcontent_getproperties(GLEW_download SOURCE_DIR GLEW_PATH)
    add_library(GLEW STATIC ${GLEW_PATH}/src/glew.c)
    target_include_directories(GLEW PUBLIC ${GLEW_PATH}/include ${OPENGL_INCLUDE_DIR})
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
fetchcontent_declare(
    kissFFT
    GIT_REPOSITORY https://github.com/mborgerding/kissfft.git
    GIT_TAG f5f2a3b2f2cd02bf80639adb12cbeed125bdf420 # latest main at the time of writing this
    EXCLUDE_FROM_ALL # from CMake 3.28+
)

set(BUILD_SHARED_LIBS_ORIG ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
set(KISSFFT_OPENMP
    OFF
    CACHE INTERNAL "")
set(KISSFFT_PKGCONFIG
    OFF
    CACHE INTERNAL "")
set(KISSFFT_STATIC
    ON
    CACHE INTERNAL "")
set(KISSFFT_TEST
    OFF
    CACHE INTERNAL "")
set(KISSFFT_TOOLS
    OFF
    CACHE INTERNAL "")
set(KISSFFT_USE_ALLOCA
    OFF
    CACHE INTERNAL "")
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
# Exclude from all for pre-CMake 3.28
fetchcontent_getproperties(kissFFT)
if(NOT kissfft_POPULATED)
    fetchcontent_populate(kissFFT)
    add_subdirectory(${kissfft_SOURCE_DIR} ${kissfft_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
set_property(TARGET kissfft PROPERTY POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_ORIG})

# Download easily downloadable header only ones

#########################################################################
# args
#########################################################################
fetchcontent_declare(
    args
    GIT_REPOSITORY https://github.com/Taywee/args.git
    GIT_TAG cc2368ca0d8a962862c96c00fe919e1480050f51 # latest main at the time of writing this
    EXCLUDE_FROM_ALL # CMake 3.28+
)
set(ARGS_MAIN_PROJECT
    OFF
    CACHE INTERNAL "")
set(ARGS_BUILD_EXAMPLE
    OFF
    CACHE INTERNAL "")
set(ARGS_BUILD_UNITTESTS
    OFF
    CACHE INTERNAL "")

# Exclude from all for pre-CMake 3.28
fetchcontent_getproperties(args)
if(NOT args_POPULATED)
    fetchcontent_populate(args)
    add_subdirectory(${args_SOURCE_DIR} ${args_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

#########################################################################
# cpp-feather-ini-parser
#########################################################################
fetchcontent_declare(
    cpp-feather-ini-parser
    GIT_REPOSITORY https://github.com/c-ridgway/cpp-feather-ini-parser.git
    GIT_TAG c04a5e504f7b954548fbbd763ac0d4843d4d7c93 # latest main at the time of writing this
    CONFIGURE_COMMAND "" BUILD_COMMAND "")

fetchcontent_makeavailable(cpp-feather-ini-parser)

add_library(cpp-feather-ini-parser INTERFACE)
target_include_directories(cpp-feather-ini-parser INTERFACE ${cpp-feather-ini-parser_SOURCE_DIR})
