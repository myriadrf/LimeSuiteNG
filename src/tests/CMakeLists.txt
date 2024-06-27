set(LIME_TEST_SUITE_NAME limesuiteng-test)

set(LIME_TEST_SUITE_SOURCES
    boards/LMS7002M_SDRDevice_Fixture.cpp
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

add_executable(${LIME_TEST_SUITE_NAME} ${LIME_TEST_SUITE_SOURCES})

target_include_directories(${LIME_TEST_SUITE_NAME} PRIVATE ${LIME_SUITE_INCLUDES} tests)
# Has to be in this specific order otherwise shared libs don't find the tests
target_link_libraries(
    ${LIME_TEST_SUITE_NAME} PUBLIC ${MAIN_LIBRARY_NAME} GTest::gtest GTest::gmock_main GTest::gtest_main GTest::gmock)

if(BUILD_SHARED_LIBS)
    target_compile_definitions(${LIME_TEST_SUITE_NAME} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
endif()

if(CMAKE_BINARY_DIR)
    set_target_properties(${LIME_TEST_SUITE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()

add_subdirectory(streaming)
add_subdirectory(LimeSuite_wrapper)

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
