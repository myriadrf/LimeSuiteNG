include(FeatureSummary)

option(BUILD_DRIVERS_LIMEPCIE "Build limepcie linux kernel module" ON)
add_feature_info("BUILD_DRIVERS_LIMEPCIE" BUILD_DRIVERS_LIMEPCIE "Build limepcie driver")
if(BUILD_DRIVERS_LIMEPCIE)
    add_subdirectory(limepcie)
endif()
