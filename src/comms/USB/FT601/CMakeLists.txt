if(WIN32 AND NOT MINGW)
    #select which FTDI D3XX driver library to build against
    set(LIMESUITENG_FTDI_DRIVER "WU_FTD3XX" CACHE STRING "FTDI D3XX driver library to use (FTD3XX or WU_FTD3XX)")
    set_property(CACHE LIMESUITENG_FTDI_DRIVER PROPERTY STRINGS FTD3XX WU_FTD3XX)

    #default is to use static libs
    option(FTD3XX_STATIC "Use static libs for FTD3XX" TRUE)

    if(LIMESUITENG_FTDI_DRIVER STREQUAL "FTD3XX")
        file(ARCHIVE_EXTRACT INPUT ${CMAKE_SOURCE_DIR}/external/FTD3XXLibrary.zip DESTINATION ${FETCHCONTENT_BASE_DIR}/)
        set(FTDI_INCLUDE_DIRS ${FETCHCONTENT_BASE_DIR}/FTD3XXLibrary)

        #library directory based on x64 vs x86
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(FTD3XX_LIB_DIR ${FTDI_INCLUDE_DIRS}/x64)
        else()
            set(FTD3XX_LIB_DIR ${FTDI_INCLUDE_DIRS}/Win32)
        endif()

        set(FTD3XX_STATIC_LIB ${FTD3XX_LIB_DIR}/Static_Lib/FTD3XX.lib)
        set(FTD3XX_DYNAMIC_LIB ${FTD3XX_LIB_DIR}/DLL/FTD3XX.lib)
        set(FTD3XX_DLL ${FTD3XX_LIB_DIR}/DLL/FTD3XX.dll)
    elseif(LIMESUITENG_FTDI_DRIVER STREQUAL "WU_FTD3XX")
        file(ARCHIVE_EXTRACT INPUT ${CMAKE_SOURCE_DIR}/external/WU_FTD3XXLib.zip DESTINATION ${FETCHCONTENT_BASE_DIR}/)
        set(FTDI_INCLUDE_DIRS ${FETCHCONTENT_BASE_DIR}/WU_FTD3XXLib/Lib)

        #library directory based on x64 vs x86
        if(CMAKE_SIZEOF_VOID_P EQUAL 8)
            set(ARCH x64)
        else()
            set(ARCH x86)
        endif()

        set(FTD3XX_LIB_DIR ${FTDI_INCLUDE_DIRS})
        set(FTD3XX_STATIC_LIB ${FTD3XX_LIB_DIR}/Static/${ARCH}/FTD3XXWU.lib)
        set(FTD3XX_DYNAMIC_LIB ${FTD3XX_LIB_DIR}/Dynamic/${ARCH}/FTD3XXWU.lib)
        set(FTD3XX_DLL ${FTD3XX_LIB_DIR}/Dynamic/${ARCH}/FTD3XXWU.dll)
    else()
        message(FATAL_ERROR "Invalid LIMESUITENG_FTDI_DRIVER value '${LIMESUITENG_FTDI_DRIVER}', must be FTD3XX or WU_FTD3XX")
    endif()
    set(FTDI_FOUND TRUE)
    message(STATUS "FTDI driver library: ${LIMESUITENG_FTDI_DRIVER}")

    #when using static libs specify FTD3XX_STATIC
    if(FTD3XX_STATIC)
        set(FTDI_LIBRARIES ${FTD3XX_STATIC_LIB} setupapi)
        target_compile_definitions(limesuiteng PRIVATE FTD3XX_STATIC)
    else()
        #when using dynamic libs install the dll to bin/
        set(FTDI_LIBRARIES ${FTD3XX_DYNAMIC_LIB} setupapi)
        install(FILES ${FTD3XX_DLL} DESTINATION bin)
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
