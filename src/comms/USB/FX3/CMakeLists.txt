if(WIN32)
    find_package(CyAPI)

    if(NOT CYAPI_FOUND) # if locally installed FX3SDK was not found, download alternative
        message(STATUS "Downloading FX3SDK to: " ${PROJECT_BINARY_DIR}/_deps/fx3sdk)
        include(FetchContent)
        cmake_policy(SET CMP0135 NEW)
        fetchcontent_declare(
            cyapi
            URL "https://downloads.myriadrf.org/project/limesuite/appveyor/FX3SDK.zip"
            URL_HASH SHA1=f24fd11fcac1099312fe6fb534145169dc35a75a
            SOURCE_DIR ${PROJECT_BINARY_DIR}/_deps/fx3sdk EXCLUDE_FROM_ALL)
        fetchcontent_makeavailable(cyapi)
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
    target_link_options(limesuiteng PRIVATE "legacy_stdio_definitions.lib")
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

    set(FX3_INCLUDE_DIRS ${LIBUSB_1_INCLUDE_DIRS})
    set(FX3_LIBRARIES ${LIBUSB_1_LIBRARIES})
endif()

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_USB_FX3 "Enable FX3" ON "FX3_FOUND" OFF)
add_feature_info(USB_FX3 ENABLE_USB_FX3 "USB support for Cypress FX3")
if(NOT ENABLE_USB_FX3)
    return()
endif()

target_sources(limesuiteng PRIVATE FX3.cpp)
target_include_directories(limesuiteng PRIVATE ${FX3_INCLUDE_DIRS})
target_link_libraries(limesuiteng PRIVATE ${FX3_LIBRARIES})
