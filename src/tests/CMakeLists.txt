set(LIME_TEST_SUITE_NAME limesuiteng-test)

set(LIME_TEST_SUITE_SOURCES
    boards/LMS7002M_SDRDevice_Fixture.cpp
    # TODO: # parsers/CoefficientFileParserTest.cpp
    protocols/LMS64CProtocol/FirmwareToDescriptorTest.cpp
    protocols/BufferInterleavingTest.cpp)

if(ENABLE_LIMEPCIE)
    list(
        APPEND
        LIME_TEST_SUITE_SOURCES
        boards/LimeSDR_X3/LimeSDR_X3Test.cpp
        boards/LimeSDR_X3/SlaveSelectShimTest.cpp
        boards/LimeSDR_XTRX/LimeSDR_XTRXTest.cpp
        comms/PCIe/PCIE_CSR_PipeTest.cpp)
endif()

if(ENABLE_USB_FX3)
    list(APPEND LIME_TEST_SUITE_SOURCES boards/LimeSDR/LimeSDRTest.cpp boards/LimeSDR/USB_CSR_Pipe_SDRTest.cpp)
endif()

if(ENABLE_USB_FTDI)
    list(APPEND LIME_TEST_SUITE_SOURCES boards/LimeSDR_Mini/LimeSDR_MiniTest.cpp boards/LimeSDR_Mini/USB_CSR_Pipe_MiniTest.cpp)
endif()

add_executable(${LIME_TEST_SUITE_NAME} ${LIME_TEST_SUITE_SOURCES})

target_include_directories(${LIME_TEST_SUITE_NAME} PRIVATE ${LIME_SUITE_INCLUDES} tests)
target_link_libraries(${LIME_TEST_SUITE_NAME} PUBLIC ${MAIN_LIBRARY_NAME} GTest::gtest_main GTest::gmock_main)

if(CMAKE_BINARY_DIR)
    set_target_properties(${LIME_TEST_SUITE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()

include(GoogleTest)
gtest_discover_tests(${LIME_TEST_SUITE_NAME})

if(CMAKE_BUILD_TYPE STREQUAL "Debug"
   AND ENABLE_CODE_COVERAGE
   AND (NOT MSVC))
    include(CodeCoverage)
    setup_target_for_coverage_lcov(
        NAME ${LIME_TEST_SUITE_NAME}_coverage EXECUTABLE ${LIME_TEST_SUITE_NAME} EXCLUDE "/usr/*" "build/*" "external/*" "tests/*")
    target_link_libraries(${LIME_TEST_SUITE_NAME} PUBLIC gcov)
endif()
