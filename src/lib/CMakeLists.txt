########################################################################
## LimeSuiteNG public C API wrappers, one file per wrapped class
########################################################################

target_sources(
    limesuiteng
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/device_registry_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/gpio_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/rfstream_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sdrdescriptor_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sdrdevice_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/spi_c.cpp
            ${CMAKE_CURRENT_LIST_DIR}/versioninfo_c.cpp)
