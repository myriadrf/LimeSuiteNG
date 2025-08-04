configure_file(BoardSupportLoader.in.cpp ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)
target_sources(limesuiteng PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/BoardSupportLoader.cpp)

target_sources(limesuiteng PRIVATE DeviceRegistry.cpp DeviceHandle.cpp LMS7002M_SDRDevice.cpp)

if(LIMESUITENG_PCIE)
    target_sources(limesuiteng PRIVATE DeviceFactoryPCIe.cpp)
    add_subdirectory(LimeSDR_X3)
    add_subdirectory(LimeSDR_XTRX)
    add_subdirectory(MMX8)
endif()

if(LIMESUITENG_USB_FX3)
    add_subdirectory(LimeSDR)
    target_sources(limesuiteng PRIVATE DeviceFactoryFX3.cpp)
endif()

if(LIMESUITENG_USB_FTDI)
    add_subdirectory(LimeSDR_Mini)
    target_sources(limesuiteng PRIVATE DeviceFactoryFTDI.cpp)
endif()
