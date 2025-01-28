project(
    limesuiteng-embedded
    VERSION 0.1.0
    LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

########################################################################
## public headers
########################################################################
# if(INSTALL_HEADERS)
#     install(
#         DIRECTORY "${PROJECT_SOURCE_DIR}/include/"
#         DESTINATION include
#         COMPONENT HEADERS)
# endif()

add_subdirectory(gfir)
add_subdirectory(lms7002m)
