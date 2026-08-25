include(FeatureSummary)
include(CMakeDependentOption)
project(
    la9310_limesdr
    VERSION 0.2.0
    DESCRIPTION "Linux kernel module for LA9310 based LimeSDR devices"
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

# sources must include header files with relative path, so they would be copied into system
add_kernel_module(
    NAME
    la9310_limesdr
    VERSION
    ${PROJECT_VERSION}
    GITHASH
    ${GITHASH}
    SOURCES
    la9310_softirq.c
    la9310_softirq.h
    la9310_sysfs.c
    la9310_firmware.c
    la9310_limesdr_module.c
    la9310_base.h
    la9310_base.c
    la9310_vspa.h
    la9310_vspa.c
    la9310_pci.h
    la9310_pci.c
    la9310_ioctl.h
    la9310_ioctl.c
    la9310_limesdr_device.h
    la9310_limesdr_device.c
    la9310_character_device.h
    la9310_character_device.c
    la9310_ioctl.h
    common_headers/la9310_pci_def.h
    common_headers/la9310_host_if.h
    CONFIGURED_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/version.h
    ${CMAKE_CURRENT_BINARY_DIR}/${OBJECTS_DIR}/dkms.conf)

add_kernel_module(NAME la9310_uart_module VERSION "1.0.0" SOURCES la9310_uart.c la9310_uart.h la9310_base.h)

option(LA9310_LIMESDR_INSTALL "Install Linux la9310_limesdr kernel module" ON)
add_feature_info("LA9310_LIMESDR_INSTALL" LA9310_LIMESDR_INSTALL "Install la9310_limesdr kernel module to system")
if(NOT LA9310_LIMESDR_INSTALL)
    return()
endif()

cmake_dependent_option(
    LA9310_LIMESDR_DKMS "Use DKMS when installing the kernel module" ON "DKMS_EXECUTABLE;LA9310_LIMESDR_INSTALL" OFF)
add_feature_info("LA9310_LIMESDR_DKMS" LA9310_LIMESDR_DKMS "Use DKMS system to build/install la9310_limesdr module")

option(LA9410_LIMESDR_RELOAD_ON_INSTALL "After install reload kernel driver modules" ON)
add_feature_info("LA9410_LIMESDR_RELOAD_ON_INSTALL" LA9410_LIMESDR_RELOAD_ON_INSTALL "Reload limepcie kernel module after install")
install_kernel_module(NAME la9310_limesdr DKMS ${LA9310_LIMESDR_DKMS} RELOAD_MODULE ${LA9410_LIMESDR_RELOAD_ON_INSTALL})
