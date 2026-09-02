include(FeatureSummary)
include(CMakeDependentOption)

add_subdirectory(xillybus)

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
target_sources(limesuiteng PRIVATE LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)

# to allow including header from drivers
target_include_directories(limesuiteng PRIVATE ${limesuiteng_SOURCE_DIR})
