########################################################################
## wx widgets dependency
########################################################################
set(wxWidgets_USE_DEBUG OFF)
set(wxWidgets_USE_UNICODE ON)
set(wxWidgets_USE_UNIVERSAL OFF)
set(wxWidgets_USE_STATIC ON)
find_package(wxWidgets COMPONENTS richtext adv core base gl)
include(FetchContent)
if(NOT wxWidgets_FOUND AND WIN32)
    cmake_policy(SET CMP0135 NEW)
    message(STATUS "Downloading wxWidgets headers.")
    set(wxRoot ${PROJECT_BINARY_DIR}/_deps/wxWidgets)
    if(CMAKE_CL_64)
        fetchcontent_declare(
            wxWidgets
            URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.4/wxMSW-3.2.4_vc14x_x64_Dev.7z
            URL_HASH SHA1=19cf62ade3c3fc55813ad9a2f1a49220574cb54a
            SOURCE_DIR ${wxRoot} EXCLUDE_FROM_ALL)
    else()
        fetchcontent_declare(
            wxWidgets
            URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.4/wxMSW-3.2.4_vc14x_Dev.7z
            URL_HASH SHA1=29647b24e5120b62e588827c163f0fe660936002
            SOURCE_DIR ${wxRoot} EXCLUDE_FROM_ALL)
    endif()

    fetchcontent_declare(
        wxWidgetsHeaders
        URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.4/wxWidgets-3.2.4-headers.7z
        URL_HASH SHA1=86022483a0bfe81edf5aee38de07a0800f0a0602
        SOURCE_DIR ${wxRoot}/include EXCLUDE_FROM_ALL)

    fetchcontent_makeavailable(wxWidgets wxWidgetsHeaders)

    # find downloaded libraries
    set(wxWidgets_ROOT_DIR ${wxRoot})
    find_package(wxWidgets COMPONENTS richtext adv core base gl)
endif()

message(STATUS "wxWidgets_DEFINITIONS: ${wxWidgets_DEFINITIONS}")
message(STATUS "wxWidgets_INCLUDE_DIRS: ${wxWidgets_INCLUDE_DIRS}")
message(STATUS "wxWidgets_LIBRARIES: ${wxWidgets_LIBRARIES}")

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_GUI "Enable build limeGUI" ON "ENABLE_LIBRARY;wxWidgets_FOUND" OFF)
add_feature_info(GUI ENABLE_GUI "GUI Application for LimeSuite")
if(NOT ENABLE_GUI)
    return()
endif()

set(GUI_SOURCES
    limeGUI.cpp
    limeGUIFrame.cpp
    LMS_Programing/LMS_Programing_wxgui.cpp
    utility/pnlMiniLog.cpp
    utility/SPI_wxgui.cpp
    utility/dlgFullMessageLog.cpp
    utility/utilities_gui.cpp
    dlgAbout_view.cpp
    dlgAbout.cpp
    numericSlider/numericSlider.cpp
    fftviewer_wxgui/fftviewer_wxgui.cpp
    fftviewer_wxgui/fftviewer_frFFTviewer.cpp
    lms7suiteEvents/lms7suiteEvents.cpp
    windowFunction.cpp
    CommsToDevice.cpp
    DeviceConnectionPanel.cpp
    events.cpp
    SDRConfiguration_view.cpp
    parsers/CoefficientFileParser.cpp)

include(${wxWidgets_USE_FILE})
add_subdirectory(oglGraph)

set(BINARY_OUTPUT_DIR "${CMAKE_BINARY_DIR}/bin")
set(GUI_EXECUTABLE_NAME limeGUI)

add_executable(${GUI_EXECUTABLE_NAME} ${GUI_SOURCES} resources/resource.rc)
set_target_properties(${GUI_EXECUTABLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${BINARY_OUTPUT_DIR})
target_compile_definitions(${GUI_EXECUTABLE_NAME} PRIVATE WIN32_LEAN_AND_MEAN)
target_include_directories(
    ${GUI_EXECUTABLE_NAME}
    PRIVATE ${LIME_SUITE_INCLUDES}
            ${CMAKE_CURRENT_SOURCE_DIR}
            lms7002_wxgui
            utilities_gui
            FPGAcontrols_wxgui
            numericSlider
            fftviewer_wxgui
            lms7suiteEvents
            boards_wxgui
            GUI)

add_subdirectory(boards)
add_subdirectory(chips)

target_link_libraries(
    ${GUI_EXECUTABLE_NAME}
    PUBLIC limesuiteng oglGraph ${wxWidgets_LIBRARIES}
    PRIVATE kissFFT)

if(MSVC)
    set_property(
        TARGET ${GUI_EXECUTABLE_NAME}
        APPEND
        PROPERTY LINK_FLAGS /SUBSYSTEM:WINDOWS)
endif(MSVC)

install(TARGETS ${GUI_EXECUTABLE_NAME} DESTINATION bin)
