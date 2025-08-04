if(NOT DEFINED CMAKE_MINIMUM_REQUIRED_VERSION)
    cmake_minimum_required(VERSION 3.15)
endif()

include(FeatureSummary)
include(CMakeDependentOption)

project(
    limepcie
    VERSION 0.1.9
    DESCRIPTION "Linux kernel module for LimeSDR PCIe based devices"
    LANGUAGES C)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
checkdebianchangelogversion(${PROJECT_NAME}-dkms)
include(add_kernel_module)

find_package(LinuxKernelHeaders REQUIRED)

# Get Git changeset
include(GetGitRevisionDescription)
get_git_head_revision(GITREFSPEC GITHASH)
if(GITHASH)
    string(SUBSTRING "${GITHASH}" 0 8 GITHASH)
else(GITHASH)
    set(GITHASH "")
endif(GITHASH)

set(OBJECTS_DIR ${PROJECT_NAME}-${PROJECT_VERSION})
configure_file(version.h.in ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/version.h @ONLY)
configure_file(dkms.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/dkms.conf @ONLY)

find_program(DKMS_EXECUTABLE NAMES dkms)
mark_as_advanced(DKMS_EXECUTABLE)

cmake_dependent_option(LIMEPCIE_DKMS "Use DKMS when installing the kernel module" ON "DKMS_EXECUTABLE" OFF)
add_feature_info("LIMEPCIE_DKMS" LIMEPCIE_DKMS "Use DKMS system to build/install limepcie module")

# sources must include header files with relative path, so they would be copied into system
add_kernel_module(
    NAME
    limepcie
    VERSION
    ${PROJECT_VERSION}
    GITHASH
    ${GITHASH}
    SOURCES
    limepcie.h
    limepcie_core.c
    limeuart.h
    limeuart.c
    litex.h
    boards.h
    bsp/config.h
    bsp/csr.h
    bsp/flags.h
    bsp/mem.h
    bsp/soc.h
    CONFIGURED_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/version.h
    ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/dkms.conf)

option(LIMEPCIE_INSTALL "Install Linux LimePCIe kernel module" ON)
add_feature_info("LIMEPCIE_INSTALL" LIMEPCIE_INSTALL "Install limepcie kernel module to system")
if(NOT LIMEPCIE_INSTALL)
    return()
endif()

option(LIMEPCIE_RELOAD_ON_INSTALL "After install reload kernel driver modules" ON)
add_feature_info("LIMEPCIE_RELOAD_ON_INSTALL" LIMEPCIE_RELOAD_ON_INSTALL "Reload limepcie kernel module after install")
install_kernel_module(NAME limepcie DKMS ${LIMEPCIE_DKMS} RELOAD_MODULE ${LIMEPCIE_RELOAD_ON_INSTALL})
