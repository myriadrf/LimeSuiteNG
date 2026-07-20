# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.

set(CPACK_THREADS "0") # 0=max thread count

set(CPACK_OUTPUT_FILE_PREFIX "${PROJECT_BINARY_DIR}/packages")
set(CPACK_PACKAGE_DIRECTORY ${PROJECT_BINARY_DIR}/packages)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_OUTPUT_CONFIG_FILE "${PROJECT_BINARY_DIR}/packages/CPackConfig.cmake")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "${PROJECT_BINARY_DIR}/packages/CPackSourceConfig.cmake")
set(CPACK_SOURCE_IGNORE_FILES "/build/;/docs/;/\\\\.git/;/*\\\\.gitignore;/*\\\\.lib;/*\\\\.dll")
set(CPACK_IGNORE_FILES "/build/;/docs/;/\\\\.git/")

set(CPACK_DEB_COMPONENT_INSTALL YES)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_ENABLE_COMPONENT_DEPENDS ON) # generate dependencies between packages
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS YES) # needs dpkg-shlibdeps utility
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "Some debian description")
# set(CPACK_DEBIAN_PACKAGE_SECTION "devel")

set(
    CPACK_INSTALL_DEFAULT_DIRECTORY_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

set(CPACK_PACKAGE_DESCRIPTION "Software packages for LimeSDR based devices")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_CONTACT "Lime Microsystems <info@limemicro.com>")
set(CPACK_PACKAGE_VENDOR "Humanity")
set(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/COPYING)

if(WIN32)
    # Windows gets a zip and an NSIS installer (#149), Linux keeps distro packaging
    set(CPACK_GENERATOR "ZIP;NSIS")

    if(CMAKE_GENERATOR_PLATFORM)
        set(WINDOWS_PACKAGE_ARCH ${CMAKE_GENERATOR_PLATFORM})
    else()
        set(WINDOWS_PACKAGE_ARCH ${CMAKE_SYSTEM_PROCESSOR})
    endif()
    set(CPACK_PACKAGE_FILE_NAME "limesuiteng-${PROJECT_VERSION}-windows-${WINDOWS_PACKAGE_ARCH}")

    set(CPACK_PACKAGE_INSTALL_DIRECTORY "LimeSuiteNG")
    set(CPACK_NSIS_PACKAGE_NAME "Lime Suite NG")
    set(CPACK_NSIS_DISPLAY_NAME "Lime Suite NG ${PROJECT_VERSION}")
    set(CPACK_NSIS_URL_INFO_ABOUT "https://myriadrf.org/projects/limesdr/")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    # offer adding the install directory to PATH, so the CLI tools work from any terminal
    set(CPACK_NSIS_MODIFY_PATH ON)
    set(CPACK_PACKAGE_EXECUTABLES "limeGUI" "Lime Suite NG GUI")
endif()
