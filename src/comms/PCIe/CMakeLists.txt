include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_LIMEPCIE "Enable Lite PCIe" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info(LIMEPCIE ENABLE_LIMEPCIE "PCIe support")
endif()

if(NOT ENABLE_LIMEPCIE)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(
    limesuiteng PRIVATE LMS64C_FPGA_Over_PCIe.cpp LMS64C_LMS7002M_Over_PCIe.cpp LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)

# build kernel module only on Linux
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    cmake_dependent_option(LIMEPCIE_KERNEL_MODULE "Build Linux LimePCIe kernel module" ON "UNIX; NOT APPLE" OFF)
    add_feature_info(LIMEPCIE_KERNEL_MODULE LIMEPCIE_KERNEL_MODULE "Build Linux LimePCIe kernel module")
    if(LIMEPCIE_KERNEL_MODULE)
        add_subdirectory(linux-kernel-module)
    endif()
endif()
