########################################################################
## LimeSuiteNG public C API wrappers, one file per wrapped class
########################################################################

target_sources(
    limesuiteng
    PRIVATE ${CMAKE_CURRENT_LIST_DIR}/device_registry.cpp
            ${CMAKE_CURRENT_LIST_DIR}/gpio.cpp
            ${CMAKE_CURRENT_LIST_DIR}/rfstream.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sdrdescriptor.cpp
            ${CMAKE_CURRENT_LIST_DIR}/sdrdevice.cpp
            ${CMAKE_CURRENT_LIST_DIR}/spi.cpp
            ${CMAKE_CURRENT_LIST_DIR}/versioninfo.cpp)
