cmake_minimum_required(VERSION 3.5)

project(
    litepcie-kernel
    VERSION 0.1.0
    LANGUAGES C)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(MODULE_NAME litepcie)

find_package(LinuxKernelHeaders REQUIRED)

# Get kernel version
execute_process(
    COMMAND uname -r
    OUTPUT_VARIABLE KERNEL_RELEASE
    OUTPUT_STRIP_TRAILING_WHITESPACE)
set(KERNEL_SOURCE_DIR /lib/modules/${KERNEL_RELEASE}/build)

# Get architecture
execute_process(
    COMMAND uname -m
    OUTPUT_VARIABLE ARCH
    OUTPUT_STRIP_TRAILING_WHITESPACE)
# ARCH might be 'aarch64', but the linux lib directories might be named 'arm64'
if(${ARCH} STREQUAL "aarch64" AND NOT EXISTS ${KERNEL_SOURCE_DIR}/arch/${ARCH})
    set(ARCH "arm64")
endif()

# where Kbuild file will be placed
set(KBUILD_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR})

# Generate the Kbuild file through cmake.
file(
    WRITE ${KBUILD_FILE_DIR}/Kbuild
    "ccflags-y = -Wno-declaration-after-statement
    ccflags-y = -I${KBUILD_FILE_DIR}/bsp
    obj-m = litepcie.o")

set(MODULE_OBJECT ${KBUILD_FILE_DIR}/${MODULE_NAME}.ko)
set(KBUILD_CMD
    $(MAKE)
    -C
    ${KERNEL_SOURCE_DIR}
    ARCH=${ARCH}
    # Informs kbuild that an external module is being built.
    # The value given to "M" is the absolute path of the directory where the external module (kbuild file) is located.
    M=${KBUILD_FILE_DIR}
    modules)

set(KBUILD_CLEAN_CMD $(MAKE) -C ${KERNEL_SOURCE_DIR} ARCH=${ARCH} M=${KBUILD_FILE_DIR} clean)

set(KERNEL_SOURCE_RELATIVE_PATHS litepcie.c litepcie.h boards.h bsp/config.h bsp/csr.h bsp/flags.h bsp/mem.h bsp/soc.h)

# Copy all source files into build directory and compile there, as the Kbuild produces artifacts in tree
foreach(SRC_FILENAME ${KERNEL_SOURCE_RELATIVE_PATHS})
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${SRC_FILENAME} ${KBUILD_FILE_DIR}/${SRC_FILENAME} COPYONLY)
endforeach()

add_custom_command(
    OUTPUT ${MODULE_OBJECT}
    COMMAND ${KBUILD_CLEAN_CMD}
    COMMAND ${KBUILD_CMD}
    WORKING_DIRECTORY ${KBUILD_FILE_DIR}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR} # rebuild if anything changes in the source dir
    VERBATIM
    COMMENT "Building Linux kernel module in dir: ${KBUILD_FILE_DIR}")

add_custom_target(litepcie-kernel ALL DEPENDS ${MODULE_OBJECT})

cmake_dependent_option(INSTALL_KERNEL_MODULE "Install Linux LitePCIe kernel module" ON "LITEPCIE_KERNEL_MODULE" OFF)
if(NOT INSTALL_KERNEL_MODULE)
    return()
endif()

# If module is already loaded, unload it
install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/unload.sh ${MODULE_NAME})")
set(KERNEL_MODULE_DESTINATION /lib/modules/${KERNEL_RELEASE}/extra)
install(FILES ${MODULE_OBJECT} DESTINATION ${KERNEL_MODULE_DESTINATION})
# load installed module
install(CODE "execute_process(COMMAND ${CMAKE_CURRENT_LIST_DIR}/install.sh ${MODULE_NAME})")

add_custom_target(
    uninstall-kernel-module
    COMMAND ${CMAKE_CURRENT_LIST_DIR}/uninstall.sh ${MODULE_NAME}
    COMMENT "Uninstalling Linux kernel module ${MODULE_NAME}")
