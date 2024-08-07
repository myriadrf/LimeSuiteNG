target_sources(limesuiteng PRIVATE DeviceRegistry.cpp DeviceHandle.cpp LMS7002M_SDRDevice.cpp)

if(NOT DEFINED ENABLE_LIMEPCIE
   OR NOT DEFINED ENABLE_USB_FX3
   OR NOT DEFINED ENABLE_USB_FTDI)
    message(FATAL_ERROR "Comms cmake has to be processed before boards")
endif()

if(ENABLE_LIMEPCIE)
    target_sources(limesuiteng PRIVATE DeviceFactoryPCIe.cpp)
    add_subdirectory(LimeSDR_X3)
    add_subdirectory(LimeSDR_XTRX)
    add_subdirectory(MMX8)
    add_subdirectory(external/XSDR)
endif()

if(ENABLE_USB_FX3)
    add_subdirectory(LimeSDR)
    target_sources(limesuiteng PRIVATE DeviceFactoryFX3.cpp)
endif()

if(ENABLE_USB_FTDI)
    add_subdirectory(LimeSDR_Mini)
    target_sources(limesuiteng PRIVATE DeviceFactoryFTDI.cpp)
endif()
