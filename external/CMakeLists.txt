# Download and compile needed externals
include(FetchContent)

#########################################################################
# kissFFT
#########################################################################
# find_package(kissfft)
# kissfft target name varies if it's static or shared library. So just always compile static version from source.
if(TRUE)
    fetchcontent_declare(
        kissFFT
        # GIT_REPOSITORY https://github.com/mborgerding/kissfft.git
        # GIT_TAG f5f2a3b2f2cd02bf80639adb12cbeed125bdf420 # latest main at the time of writing this
        URL ${CMAKE_CURRENT_LIST_DIR}/kissfft.zip EXCLUDE_FROM_ALL # from CMake 3.28+
    )

    set(BUILD_SHARED_LIBS_ORIG ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    set(CMAKE_BUILD_TYPE_ORIG ${CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE "Release") # always build kissFFT in release mode, for better performance.
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
        set_property(TARGET kissfft PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    endif()
    set_property(TARGET kissfft PROPERTY POSITION_INDEPENDENT_CODE TRUE)
    # disable warnings spam
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(kissfft PRIVATE -Wno-strict-prototypes -Wno-newline-eof -Wno-cast-align)
    endif()
    set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_ORIG})
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_ORIG})

    list(APPEND CMAKE_PREFIX_PATH "${kissfft_BINARY_DIR}")
    set(CMAKE_PREFIX_PATH
        ${CMAKE_PREFIX_PATH}
        PARENT_SCOPE)

    add_library(kissfft::kissfft ALIAS kissfft)
    include_directories(${kissfft_SOURCE_DIR})
endif()

#########################################################################
# args
#########################################################################
fetchcontent_declare(
    args
    # GIT_REPOSITORY https://github.com/Taywee/args.git
    # GIT_TAG 30cbfdc69ab65bf9df78b811a9ae19a923ae84c9 # latest main at the time of writing this
    URL ${CMAKE_CURRENT_LIST_DIR}/args.zip EXCLUDE_FROM_ALL # CMake 3.28+
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
    # populating manually, so that only the code would be used, and avoid subproject's CMake installs
    fetchcontent_populate(args)
    add_library(taywee-args INTERFACE)
    target_include_directories(taywee-args INTERFACE ${args_SOURCE_DIR})
    # add_subdirectory(${args_SOURCE_DIR} ${args_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

#########################################################################
# cpp-feather-ini-parser
#########################################################################
fetchcontent_declare(
    cpp-feather-ini-parser
    # GIT_REPOSITORY https://github.com/rjonaitis/cpp-feather-ini-parser
    # GIT_TAG 85a7d3a89dd0e179587a87b8d12ea58a896e7881 # latest main at the time of writing this
    URL ${CMAKE_CURRENT_LIST_DIR}/cpp-feather-ini-parser.zip CONFIGURE_COMMAND "" BUILD_COMMAND "" EXCLUDE_FROM_ALL)

fetchcontent_makeavailable(cpp-feather-ini-parser)

add_library(cpp-feather-ini-parser INTERFACE)
target_sources(cpp-feather-ini-parser INTERFACE ${cpp-feather-ini-parser_SOURCE_DIR}/INI.h)
target_include_directories(cpp-feather-ini-parser INTERFACE ${cpp-feather-ini-parser_SOURCE_DIR})

#########################################################################
# rang // for coloring terminal outputs
#########################################################################
fetchcontent_declare(
    rang
    # GIT_REPOSITORY https://github.com/agauniyal/rang
    # GIT_TAG 22345aa4c468db3bd4a0e64a47722aad3518cc81
    URL ${CMAKE_CURRENT_LIST_DIR}/rang.zip CONFIGURE_COMMAND "" BUILD_COMMAND "" EXCLUDE_FROM_ALL)

# Exclude from all for pre-CMake 3.28
fetchcontent_getproperties(rang)
if(NOT rang_POPULATED)
    fetchcontent_populate(rang)
    add_subdirectory(${rang_SOURCE_DIR} ${rang_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
