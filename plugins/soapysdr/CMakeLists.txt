########################################################################
## SoapySDR bindings for LimeSuiteNG
########################################################################
project(
    "limesuiteng-soapysdr"
    VERSION 1.0.0 # soapy_sdr_module_util() picks up project version
    DESCRIPTION "LimeSuiteNG bindings for SoapySDR"
    LANGUAGES C CXX)
message(STATUS "##### <${PROJECT_NAME}> #####")

checkdebianchangelogversion(${PROJECT_NAME})

find_package(SoapySDR CONFIG REQUIRED)
set_package_properties(
    SoapySDR PROPERTIES
    TYPE OPTIONAL
    PURPOSE "Adds LimeSuiteNG bindings for SoapySDR")

########################################################################
## Build support module
########################################################################
# target name must be valid C identifier
soapy_sdr_module_util(TARGET limesuitengSupport SOURCES Registration.cpp Settings.cpp Streaming.cpp LIBRARIES limesuiteng)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Disable warnings for unused parameters
    target_compile_options(limesuitengSupport PRIVATE -Wno-unused-parameter)
endif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

message(STATUS "##### </${PROJECT_NAME}> #####")
