########################################################################
## SoapySDR bindings for LimeSuiteNG
########################################################################

find_package(SoapySDR CONFIG)
set_package_properties(SoapySDR PROPERTIES
    TYPE OPTIONAL
    PURPOSE "Adds LimeSuiteNG bindings for SoapySDR"
)

########################################################################
## Feature registration
########################################################################
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_SOAPYSDR "Enable LimeSuiteNG SoapySDR bindings" ON "ENABLE_LIBRARY;SoapySDR_FOUND" OFF)
add_feature_info(SOAPYSDR ENABLE_SOAPYSDR "SoapySDR bindings for LimeSuite")
if (NOT ENABLE_SOAPYSDR)
    return()
endif()

########################################################################
## Build support module
########################################################################
if(CMAKE_COMPILER_IS_GNUCXX)

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

SOAPY_SDR_MODULE_UTIL(
    TARGET LMS7Support
    SOURCES
        Registration.cpp
        Settings.cpp
        Streaming.cpp
    LIBRARIES
        limesuiteng
)
