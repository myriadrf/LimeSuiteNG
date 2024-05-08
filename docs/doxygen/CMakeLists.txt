########################################################################
## Config for building doxygen documentation
########################################################################
find_package(Doxygen OPTIONAL_COMPONENTS dot mscgen dia)
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_DOXYGEN "Enable build of Doxygen documentation" ON "DOXYGEN_FOUND" OFF)
add_feature_info(DOXYGEN ENABLE_DOXYGEN "Build Doxygen documentation")
if(ENABLE_DOXYGEN)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/DoxygenConfig.in ${CMAKE_CURRENT_BINARY_DIR}/DoxygenConfig @ONLY)
    add_custom_target(
        doxygen ALL
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/DoxygenConfig
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating Doxygen documentation"
        VERBATIM)
endif()
