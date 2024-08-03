if(UNIX)
    find_package(libusb-1.0)
    set_package_properties(
        libusb-1.0 PROPERTIES
        TYPE RECOMMENDED
        PURPOSE "Adds USB communication support")

    if(${LIBUSB_1_FOUND})
        target_sources(limesuiteng PRIVATE UnixUsb.cpp)
        target_include_directories(limesuiteng PRIVATE ${LIBUSB_1_INCLUDE_DIRS})
        target_link_libraries(limesuiteng PUBLIC ${LIBUSB_1_LIBRARIES})
    else()
        message(WARNING "libusb-1.0 libraries not found. Omitting USB devices support")
    endif()
endif()

add_subdirectory(FT601)
add_subdirectory(FX3)

target_sources(
    limesuiteng PRIVATE LMS64C_FPGA_Over_USB.cpp LMS64C_LMS7002M_Over_USB.cpp LMS64C_ADF4002_Over_USB.cpp USBDMAEmulation.cpp)
