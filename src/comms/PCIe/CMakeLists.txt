########################################################################
## Support for ConnectionLitePCIe connection
########################################################################
set(THIS_SOURCE_DIR comms/PCIe)

set(COMMS_LIMELITEPCIE_SOURCES
    ${THIS_SOURCE_DIR}/AvgRmsCounter.cpp
    ${THIS_SOURCE_DIR}/LMS64C_FPGA_Over_PCIe.cpp
    ${THIS_SOURCE_DIR}/LMS64C_LMS7002M_Over_PCIe.cpp
    ${THIS_SOURCE_DIR}/LimeLitePCIe.cpp
    ${THIS_SOURCE_DIR}/PCIE_CSR_Pipe.cpp
    ${THIS_SOURCE_DIR}/TRXLooper_PCIE.cpp)

########################################################################
## Feature registration
########################################################################
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_LIMELITEPCIE "Enable Lite PCIe" ON "UNIX;ENABLE_LIBRARY; NOT APPLE" OFF)
add_feature_info(LIMELITEPCIE ENABLE_LIMELITEPCIE "PCIe support")
if(NOT ENABLE_LIMELITEPCIE)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${COMMS_LIMELITEPCIE_SOURCES})
target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR})

# build kernel module only on Linux
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR}/linux-kernel-module)

    cmake_dependent_option(LIMELITEPCIE_KERNEL_MODULE "Build Linux LitePCIe kernel module" ON "UNIX; NOT APPLE" OFF)
    add_feature_info(LIMELITEPCIE_KERNEL_MODULE LIMELITEPCIE_KERNEL_MODULE "Build Linux LitePCIe kernel module")
    if(LIMELITEPCIE_KERNEL_MODULE)
        add_subdirectory(${THIS_SOURCE_DIR}/linux-kernel-module)
    endif()
endif()
