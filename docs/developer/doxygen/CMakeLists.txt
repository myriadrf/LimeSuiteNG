########################################################################
## Config for building doxygen documentation
########################################################################
find_package(Doxygen OPTIONAL_COMPONENTS dot mscgen dia)
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(BUILD_DOCS_DOXYGEN "Build Doxygen based documentation" ON "BUILD_DOCS;DOXYGEN_FOUND" OFF)
add_feature_info("BUILD_DOCS_DOXYGEN" BUILD_DOCS_DOXYGEN "Build Doxygen based documentation")
if(BUILD_DOCS_DOXYGEN)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/DoxygenConfig.in ${CMAKE_CURRENT_BINARY_DIR}/DoxygenConfig @ONLY)
    add_custom_target(
        doxygen ALL
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/DoxygenConfig
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating Doxygen documentation"
        VERBATIM)
endif()
