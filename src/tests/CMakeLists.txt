set(LIME_TEST_SUITE_NAME limesuiteng-test)

set(LIME_TEST_SUITE_SOURCES
    boards/LMS7002M_SDRDevice_Fixture.cpp
    LimeSuite_wrapper/streaming.cpp
    streaming/streaming.cpp
    # TODO: # parsers/CoefficientFileParserTest.cpp
    protocols/LMS64CProtocol/CustomParameterReadTest.cpp
    protocols/LMS64CProtocol/CustomParameterWriteTest.cpp
    protocols/LMS64CProtocol/DeviceResetTest.cpp
    protocols/LMS64CProtocol/FirmwareToDescriptorTest.cpp
    protocols/LMS64CProtocol/FPGA_SPITest.cpp
    protocols/LMS64CProtocol/GetFirmwareInfoTest.cpp
    protocols/LMS64CProtocol/GPIODirReadTest.cpp
    protocols/LMS64CProtocol/GPIODirWriteTest.cpp
    protocols/LMS64CProtocol/GPIOReadTest.cpp
    protocols/LMS64CProtocol/GPIOWriteTest.cpp
    protocols/LMS64CProtocol/LMS7002M_SPITest.cpp
    protocols/BufferInterleavingTest.cpp)

if(ENABLE_LITE_PCIE)
    list(
        APPEND
        LIME_TEST_SUITE_SOURCES
        boards/LimeSDR_X3/LimeSDR_X3Test.cpp
        boards/LimeSDR_X3/SlaveSelectShimTest.cpp
        boards/LimeSDR_XTRX/LimeSDR_XTRXTest.cpp)
endif()

if(ENABLE_USB_FX3)
    list(APPEND LIME_TEST_SUITE_SOURCES boards/LimeSDR/LimeSDRTest.cpp)
endif()

if(ENABLE_USB_FTDI)
    list(APPEND LIME_TEST_SUITE_SOURCES boards/LimeSDR_Mini/LimeSDR_MiniTest.cpp)
endif()

if(BUILD_SHARED_LIBS)
    # Build the tests in and use a wrapper to execute the tests.
    target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LIME_TEST_SUITE_SOURCES} TestEntry.cpp)
    target_link_libraries(${MAIN_LIBRARY_NAME} PUBLIC $<BUILD_INTERFACE:GTest::gtest> $<BUILD_INTERFACE:GTest::gmock>)

    add_executable(${LIME_TEST_SUITE_NAME} main.cpp)
else()
    add_executable(${LIME_TEST_SUITE_NAME} ${LIME_TEST_SUITE_SOURCES})

    target_include_directories(${LIME_TEST_SUITE_NAME} PRIVATE ${LIME_SUITE_INCLUDES} tests)
    target_link_libraries(${LIME_TEST_SUITE_NAME} PUBLIC GTest::gtest_main GTest::gmock_main)
endif()

target_link_libraries(${LIME_TEST_SUITE_NAME} PUBLIC ${MAIN_LIBRARY_NAME})

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
