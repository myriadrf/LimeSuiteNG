if(WIN32)
    find_package(CyAPI)

    if (NOT CYAPI_FOUND) # if locally installed FX3SDK was not found, download alternative
        message(STATUS "Downloading FX3SDK to: " ${PROJECT_BINARY_DIR}/_deps/fx3sdk)
        include(FetchContent)
        cmake_policy(SET CMP0135 NEW)
        FetchContent_Declare(
          cyapi
          URL "https://downloads.myriadrf.org/project/limesuite/appveyor/FX3SDK.zip"
          URL_HASH SHA1=f24fd11fcac1099312fe6fb534145169dc35a75a
          SOURCE_DIR ${PROJECT_BINARY_DIR}/_deps/fx3sdk
          EXCLUDE_FROM_ALL
        )
        FetchContent_MakeAvailable(cyapi)
        # repeat CyAPI search with the downloaded path
        set(FX3_SDK_PATH ${PROJECT_BINARY_DIR}/_deps/fx3sdk)
        find_package(CyAPI)
    endif()

    set_package_properties(
        CyAPI PROPERTIES
        TYPE RECOMMENDED
        PURPOSE "Adds USB communication support for Cypress FX3 chip")
    set(FX3_FOUND ${CYAPI_FOUND})
    set(FX3_INCLUDE_DIRS ${CYAPI_INCLUDE_DIRS})
    set(FX3_LIBRARIES ${CYAPI_LIBRARIES})
    if(FX3_FOUND)
        # message(STATUS "FX3_INCLUDE_DIRS: ${FX3_INCLUDE_DIRS}")
        # message(STATUS "FX3_LIBRARIES: ${FX3_LIBRARIES}")
    else()
        message(WARNING "CYAPI SDK not found. Omitting Cypress FX3 USB support")
    endif()
    target_link_libraries(${MAIN_LIBRARY_NAME} PRIVATE ${FX3_LIBRARIES})
    target_link_options(${MAIN_LIBRARY_NAME} PRIVATE "legacy_stdio_definitions.lib")
    target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE $<BUILD_INTERFACE:${FX3_INCLUDE_DIRS}> $<INSTALL_INTERFACE:include>)
elseif(UNIX)
    find_package(libusb-1.0)
    set_package_properties(
        libusb-1.0 PROPERTIES
        TYPE RECOMMENDED
        PURPOSE "Adds USB communication support for Cypress FX3 chip")
    set(FX3_FOUND ${LIBUSB_1_FOUND})
    if(NOT ${FX3_FOUND})
        message(WARNING "libusb-1.0 libraries not found. Omitting Cypress FX3 USB support")
    endif()

    target_link_libraries(${MAIN_LIBRARY_NAME} PUBLIC ${LIBUSB_1_LIBRARIES})

    if(LIBUSB_1_INCLUDE_DIR)
        target_include_directories(
            ${MAIN_LIBRARY_NAME} PUBLIC $<BUILD_INTERFACE:${LIBUSB_1_INCLUDE_DIRS}> $<INSTALL_INTERFACE:include>)
    endif()
endif()

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_USB_FX3 "Enable FX3" ON "FX3_FOUND" OFF)
add_feature_info(USB_FX3 ENABLE_USB_FX3 "USB support for Cypress FX3")
if(NOT ENABLE_USB_FX3)
    return()
endif()

set(CONNECTION_FX3_SOURCES ${CMAKE_CURRENT_LIST_DIR}/FX3.cpp ${CMAKE_CURRENT_LIST_DIR}/USBTransferContext_FX3.cpp)

target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR})
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CONNECTION_FX3_SOURCES})
