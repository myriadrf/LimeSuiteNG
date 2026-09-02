include(FeatureSummary)
include(CMakeDependentOption)

cmake_dependent_option(LIMESUITENG_XILLYBUS "Enable Xillybus" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info("LIMESUITENG_XILLYBUS" LIMESUITENG_XILLYBUS "Xillybus support")
endif()

if(NOT LIMESUITENG_XILLYBUS)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(limesuiteng PRIVATE Xillybus.cpp XillybusDMAEmulation.cpp)
