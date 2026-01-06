if(WIN32 AND NOT MINGW)
    file(ARCHIVE_EXTRACT INPUT ${CMAKE_SOURCE_DIR}/external/WU_FTD3XXLib.zip DESTINATION ${FETCHCONTENT_BASE_DIR}/)
    set(FTDI_FOUND TRUE)
    set(FTDI_INCLUDE_DIRS ${FETCHCONTENT_BASE_DIR}/WU_FTD3XXLib/Lib)

    #library directory based on x64 vs x86
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(ARCH x64)
    else()
        set(ARCH x86)
    endif()

    #default is to use static libs
    option(FTD3XX_STATIC "Use static libs for FTD3XX" TRUE)

    #when using static libs specify FTD3XX_STATIC
    set(FTD3XX_LIB_DIR ${FETCHCONTENT_BASE_DIR}/WU_FTD3XXLib/Lib)
    if(FTD3XX_STATIC)
        set(FTDI_LIBRARIES ${FTD3XX_LIB_DIR}/Static/${ARCH}/FTD3XXWU.lib setupapi)
        target_compile_definitions(limesuiteng PRIVATE FTD3XX_STATIC)
    else()
        #when using dynamic libs install the dll to bin/
        set(FTDI_LIBRARIES ${FTD3XX_LIB_DIR}/Dynamic/${ARCH}/FTD3XXWU.lib setupapi)
        install(FILES ${FTD3XX_LIB_DIR}/Dynamic/${ARCH}/FTD3XXWU.dll DESTINATION bin)
    endif()

    message(STATUS "FTDI_INCLUDE_DIRS: ${FTDI_INCLUDE_DIRS}")
    message(STATUS "FTDI_LIBRARIES: ${FTDI_LIBRARIES}")
    if(MSVC)
        target_link_libraries(limesuiteng PRIVATE ${FTDI_LIBRARIES} legacy_stdio_definitions.lib)
    else()
        target_link_libraries(limesuiteng PRIVATE ${FTDI_LIBRARIES})
    endif()
    target_include_directories(limesuiteng PRIVATE ${FTDI_INCLUDE_DIRS})
endif(WIN32 AND NOT MINGW)

include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(LIMESUITENG_USB_FTDI "Enable FT601" ON "FTDI_FOUND OR LIBUSB_1_FOUND" OFF)
add_feature_info("LIMESUITENG_USB_FTDI" LIMESUITENG_USB_FTDI "USB support for FTDI")
if(NOT LIMESUITENG_USB_FTDI)
    return()
endif()

target_sources(limesuiteng PRIVATE FT601.cpp FTDI_SerialPort.cpp)
