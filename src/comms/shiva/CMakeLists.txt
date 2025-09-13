include(FeatureSummary)
include(CMakeDependentOption)

cmake_dependent_option(LIMESUITENG_shiva "Enable LA9310 shiva driver" ON "UNIX; NOT APPLE" OFF)

if(UNIX)
    add_feature_info("LIMESUITENG_shiva" LIMESUITENG_shiva "LA9310 shiva support")
endif()

if(NOT LIMESUITENG_shiva)
    return()
endif()

########################################################################
## Add to library
########################################################################
target_sources(
    limesuiteng PRIVATE shiva_lime.cpp)

# to allow including header from drivers
target_include_directories(limesuiteng PRIVATE ${limesuiteng_SOURCE_DIR})
target_include_directories(limesuiteng PRIVATE ${CMAKE_SOURCE_DIR}/../la93xx_freertos/common_headers)
