include(FeatureSummary)

option(BUILD_DRIVERS_LIMEPCIE "Build limepcie linux kernel module" ON)
add_feature_info("BUILD_DRIVERS_LIMEPCIE" BUILD_DRIVERS_LIMEPCIE "Build limepcie driver")
if(BUILD_DRIVERS_LIMEPCIE)
    add_subdirectory(limepcie)
endif()

option(BUILD_DRIVERS_LA9310 "Build la9310_limesdr linux kernel module" ON)
add_feature_info("BUILD_DRIVERS_LA9310" BUILD_DRIVERS_LA9310 "Build la9310_limesdr driver")
if(BUILD_DRIVERS_LA9310)
    add_subdirectory(la9310_limesdr)
endif()
