include(FeatureSummary)
include(CMakeDependentOption)

# to allow including header from drivers
target_include_directories(limesuiteng PRIVATE ${limesuiteng_SOURCE_DIR})

cmake_dependent_option(LIMESUITENG_PCIE "Enable Lite PCIe" ON "UNIX; NOT APPLE" OFF)
cmake_dependent_option(LIMESUITENG_LA9310 "Enable LA9310 driver" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info("LIMESUITENG_PCIE" LIMESUITENG_PCIE "PCIe support")
    add_feature_info("LIMESUITENG_LA9310" LIMESUITENG_LA9310 "LA9310 support")
endif()

if(LIMESUITENG_PCIE)
    target_sources(limesuiteng PRIVATE LimePCIe.cpp LimePCIeDMA.cpp PCIE_CSR_Pipe.cpp)
endif()

if(LIMESUITENG_LA9310)
    target_sources(limesuiteng PRIVATE LA9310_PCIe.cpp)
    target_include_directories(limesuiteng PRIVATE ${CMAKE_SOURCE_DIR}/../la93xx_freertos/common_headers)
endif()
