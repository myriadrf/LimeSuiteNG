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

    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        fetchcontent_declare(
            wxWidgets
            URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.5/wxMSW-3.2.5_vc14x_x64_Dev.7z
            URL_HASH SHA1=f5941fea2d563531401edd32f85b195be01155e9
            SOURCE_DIR ${wxRoot} EXCLUDE_FROM_ALL)
    else()
        fetchcontent_declare(
            wxWidgets
            URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.5/wxMSW-3.2.5_vc14x_Dev.7z
            URL_HASH SHA1=b5c526e3249edf8a9731e3ff36022794637668c3
            SOURCE_DIR ${wxRoot} EXCLUDE_FROM_ALL)
    endif()

    if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(wxWidgetsReleaseDLLs wxWidgetsRelease)

        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            fetchcontent_declare(
                ${wxWidgetsReleaseDLLs}
                URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.5/wxMSW-3.2.5_vc14x_x64_ReleaseDLL.7z
                URL_HASH SHA1=531a679127365546022f53224806e57a5b5584ae
                SOURCE_DIR ${wxRoot}/lib/vc14x_x64_dll/ReleaseDLLs EXCLUDE_FROM_ALL)
        else()
            fetchcontent_declare(
                ${wxWidgetsReleaseDLLs}
                URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.5/wxMSW-3.2.5_vc14x_ReleaseDLL.7z
                URL_HASH SHA1=49029e164b653d41c72584f30b6a7ac5c4eac560
                SOURCE_DIR ${wxRoot}/lib/vc14x_dll/ReleaseDLLs EXCLUDE_FROM_ALL)
        endif()
    endif()

    fetchcontent_declare(
        wxWidgetsHeaders
        URL https://github.com/wxWidgets/wxWidgets/releases/download/v3.2.5/wxWidgets-3.2.5-headers.7z
        URL_HASH SHA1=dd01f12371f4f320dfdc4d2e53b4514eb6f296e7
        SOURCE_DIR ${wxRoot}/include EXCLUDE_FROM_ALL)

    fetchcontent_makeavailable(wxWidgets wxWidgetsHeaders ${wxWidgetsReleaseDLLs})

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

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(x64_postfix "_x64")
    endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

    get_cmake_property(IS_MULTI_CONF GENERATOR_IS_MULTI_CONFIG)
    if(${IS_MULTI_CONF})
        set(additional_folder "/$<CONFIG>")
    endif()

    set(debug_flag "$<$<CONFIG:Debug>:d>")
    set(release_dir "$<$<NOT:$<CONFIG:Debug>>:/ReleaseDLLs/lib/vc14x${x64_postfix}_dll>")

    add_custom_command(
        TARGET ${GUI_EXECUTABLE_NAME}
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${wxWidgets_LIB_DIR}${release_dir}/wxmsw32u${debug_flag}_gl_vc14x${x64_postfix}.dll
            ${wxWidgets_LIB_DIR}${release_dir}/wxmsw32u${debug_flag}_core_vc14x${x64_postfix}.dll
            ${wxWidgets_LIB_DIR}${release_dir}/wxbase32u${debug_flag}_vc14x${x64_postfix}.dll
            ${BINARY_OUTPUT_DIR}${additional_folder})
endif(WIN32)

install(TARGETS ${GUI_EXECUTABLE_NAME} DESTINATION bin)
