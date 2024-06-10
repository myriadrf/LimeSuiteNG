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

# Download and compile

include(FetchContent)

#########################################################################
# kissFFT
#########################################################################
fetchcontent_declare(
    kissFFT
    GIT_REPOSITORY https://github.com/mborgerding/kissfft.git
    GIT_TAG f5f2a3b2f2cd02bf80639adb12cbeed125bdf420 # latest main at the time of writing this
)

set(KISSFFT_OPENMP OFF CACHE INTERNAL "")
set(KISSFFT_PKGCONFIG OFF CACHE INTERNAL "")
set(KISSFFT_STATIC OFF CACHE INTERNAL "")
set(KISSFFT_TEST OFF CACHE INTERNAL "")
set(KISSFFT_TOOLS OFF CACHE INTERNAL "")
set(KISSFFT_USE_ALLOCA OFF CACHE INTERNAL "")
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

fetchcontent_makeavailable(kissFFT)

# Download easily downloadable header only ones

#########################################################################
# args
#########################################################################
fetchcontent_declare(
    args
    GIT_REPOSITORY https://github.com/Taywee/args.git
    GIT_TAG cc2368ca0d8a962862c96c00fe919e1480050f51 # latest main at the time of writing this
)
set(ARGS_MAIN_PROJECT OFF CACHE INTERNAL "")
set(ARGS_BUILD_EXAMPLE OFF CACHE INTERNAL "")
set(ARGS_BUILD_UNITTESTS OFF CACHE INTERNAL "")

fetchcontent_makeavailable(args)

#########################################################################
# cpp-feather-ini-parser
#########################################################################
fetchcontent_declare(
    cpp-feather-ini-parser
    GIT_REPOSITORY https://github.com/c-ridgway/cpp-feather-ini-parser.git
    GIT_TAG c04a5e504f7b954548fbbd763ac0d4843d4d7c93 # latest main at the time of writing this
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
)

fetchcontent_makeavailable(cpp-feather-ini-parser)

add_library(cpp-feather-ini-parser INTERFACE)
target_include_directories(cpp-feather-ini-parser INTERFACE ${cpp-feather-ini-parser_SOURCE_DIR})
