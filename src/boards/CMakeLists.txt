target_sources(
    ${MAIN_LIBRARY_NAME}
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/DeviceRegistry.cpp
            ${CMAKE_CURRENT_LIST_DIR}/DeviceHandle.cpp
            ${CMAKE_CURRENT_LIST_DIR}/LMS7002M_SDRDevice.cpp)

if(ENABLE_LITE_PCIE)
    target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/DeviceFactoryPCIe.cpp)
    include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_X3/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_XTRX/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/MMX8/CMakeLists.txt)
    include(${CMAKE_CURRENT_LIST_DIR}/external/XSDR/CMakeLists.txt)
endif()

# if(ENABLE_USB_FX3 OR ENABLE_USB_FTDI)
#     target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/USBEntry.cpp)
# endif()

if(ENABLE_USB_FX3)
    include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR/CMakeLists.txt)
    target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/DeviceFactoryFX3.cpp)
endif()

if(ENABLE_USB_FTDI)
    include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_Mini/CMakeLists.txt)
    target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/DeviceFactoryFTDI.cpp)
endif()
