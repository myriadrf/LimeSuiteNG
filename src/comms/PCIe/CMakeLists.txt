include(FeatureSummary)
include(CMakeDependentOption)

cmake_dependent_option(LIMESUITENG_PCIE "Enable Lite PCIe" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info("LIMESUITENG_PCIE" LIMESUITENG_PCIE "PCIe support")
endif()

if(NOT LIMESUITENG_PCIE)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(
    limesuiteng PRIVATE LMS64C_FPGA_Over_PCIe.cpp LMS64C_LMS7002M_Over_PCIe.cpp LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)

target_include_directories(limesuiteng PRIVATE ${CMAKE_SOURCE_DIR})
