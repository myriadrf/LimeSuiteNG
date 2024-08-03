if(NOT ENABLE_USB_FX3 AND NOT ENABLE_USB_FTDI)
    return()
endif()

add_subdirectory(FT601)
add_subdirectory(FX3)

target_sources(
    limesuiteng PRIVATE LMS64C_FPGA_Over_USB.cpp LMS64C_LMS7002M_Over_USB.cpp LMS64C_ADF4002_Over_USB.cpp USBDMAEmulation.cpp)

if(UNIX)
    target_sources(limesuiteng PRIVATE UnixUsb.cpp)
endif()
