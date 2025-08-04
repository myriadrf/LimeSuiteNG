# checks if the CMake project version is the same as in debian changelog

function(CheckDebianChangelogVersion PACKAGE_NAME)
    if (NOT PACKAGE_NAME)
        set(PACKAGE_NAME ${PROJECT_NAME})
    endif()

    set(PROJECT_DEBIAN_CHANGELOG "${CMAKE_SOURCE_DIR}/debian/${PACKAGE_NAME}.changelog")
    if(NOT EXISTS ${PROJECT_DEBIAN_CHANGELOG})
        return()
    endif()

    file(READ ${PROJECT_DEBIAN_CHANGELOG} changelog_txt)
    string(REGEX MATCH "([0-9]+)\\.([0-9]+)\\.([0-9]+)" CHANGELOG_MATCH "${changelog_txt}")
    if(NOT CHANGELOG_MATCH)
        message(SEND_ERROR "Failed to extract version number from ${PROJECT_DEBIAN_CHANGELOG}")
    endif(NOT CHANGELOG_MATCH)

    if(NOT PROJECT_VERSION STREQUAL CHANGELOG_MATCH)
        message(SEND_ERROR "Project ${PROJECT_NAME} version ${PROJECT_VERSION} and debian ${PROJECT_DEBIAN_CHANGELOG} version ${CHANGELOG_MATCH} don't match")
    endif()
endfunction()