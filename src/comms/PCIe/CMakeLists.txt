include(FeatureSummary)
include(CMakeDependentOption)

# to allow including header from drivers
target_include_directories(limesuiteng PRIVATE ${limesuiteng_SOURCE_DIR})

cmake_dependent_option(LIMESUITENG_PCIE "Enable Lite PCIe" ON "UNIX" OFF)
cmake_dependent_option(LIMESUITENG_LA9310 "Enable LA9310 driver" ON "UNIX" OFF)

if(UNIX)
    add_feature_info("LIMESUITENG_PCIE" LIMESUITENG_PCIE "PCIe support")
    add_feature_info("LIMESUITENG_LA9310" LIMESUITENG_LA9310 "LA9310 support")
endif()

if(LIMESUITENG_PCIE)
    target_sources(limesuiteng PRIVATE LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)
endif()

if(LIMESUITENG_LA9310)
    target_sources(limesuiteng PRIVATE LA9310_PCIe.cpp)
endif()
