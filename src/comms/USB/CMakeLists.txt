########################################################################
## Support for USB STREAM connection
########################################################################
set(THIS_SOURCE_DIR comms/USB)

include(${THIS_SOURCE_DIR}/FT601/CMakeLists.txt)
include(${THIS_SOURCE_DIR}/FX3/CMakeLists.txt)

if(NOT ENABLE_USB_FX3 AND NOT ENABLE_USB_FTDI)
    return()
endif()

set(COMMS_USB_SOURCES
    ${THIS_SOURCE_DIR}/GlobalHotplugEvents.cpp
    ${THIS_SOURCE_DIR}/LMS64C_FPGA_Over_USB.cpp
    ${THIS_SOURCE_DIR}/LMS64C_LMS7002M_Over_USB.cpp
    ${THIS_SOURCE_DIR}/LMS64C_ADF4002_Over_USB.cpp
    ${THIS_SOURCE_DIR}/USBDMAEmulation.cpp)

########################################################################
## Add to library
########################################################################
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${COMMS_USB_SOURCES})

if(UNIX)
    target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR}/USBGeneric.cpp)
else()
    target_sources(
        ${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR}/WindowsDeviceHotplug.cpp ${THIS_SOURCE_DIR}/WindowsGlobalHotplug.cpp)
    target_link_options(${MAIN_LIBRARY_NAME} PUBLIC "cfgmgr32.lib")
endif()
