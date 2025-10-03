if(UNIX OR MINGW)
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
endif(UNIX OR MINGW)

add_subdirectory(FT601)
add_subdirectory(FX3)

target_sources(limesuiteng PRIVATE USBDMAEmulation.cpp IUSB.cpp)
