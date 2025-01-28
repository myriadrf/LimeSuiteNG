########################################################################
## public headers
########################################################################

string(TIMESTAMP BUILD_TIMESTAMP "%Y-%m-%d")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/VersionInfo.h.in ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.h @ONLY)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/VersionInfo.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.cpp @ONLY)
target_include_directories(limesuiteng PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

target_sources(
    limesuiteng
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.h ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.cpp)

if(INSTALL_DEVELOPMENT)
    file(
        GLOB LIMESUITENG_PUBLIC_HEADERS
        RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/*.h" "${CMAKE_CURRENT_SOURCE_DIR}/limesuiteng/*.hpp")
    list(APPEND LIMESUITENG_PUBLIC_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/limesuiteng/VersionInfo.h)

    install(
        FILES ${LIMESUITENG_PUBLIC_HEADERS}
        DESTINATION include/limesuiteng
        COMPONENT dev)

    file(
        GLOB LEGACY_PUBLIC_HEADERS
        RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/lime/*.h")

    install(
        FILES ${LEGACY_PUBLIC_HEADERS}
        DESTINATION include/lime
        COMPONENT dev)
endif()
