project(limesuiteng-embedded VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

########################################################################
## public headers
########################################################################
file(
    GLOB_RECURSE LIMESUITENG_EMBEDDED_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng" "${CMAKE_CURRENT_SOURCE_DIR}/include/limesuiteng/*.h")

install(FILES ${LIMESUITENG_EMBEDDED_PUBLIC_HEADERS} DESTINATION include/limesuiteng)

add_subdirectory(lms7002m)