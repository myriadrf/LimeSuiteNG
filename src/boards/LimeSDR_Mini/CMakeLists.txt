########################################################################
## Support for LimeSDR Mini v1 board
########################################################################
set(THIS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/boards/LimeSDR_Mini)

set(LIMESDR_MINI_SOURCES
    ${THIS_SOURCE_DIR}/LimeSDR_Mini.cpp
    ${THIS_SOURCE_DIR}/FPGA_Mini.cpp
    ${THIS_SOURCE_DIR}/USB_CSR_Pipe_Mini.cpp
)

########################################################################
## Add to library
########################################################################
target_include_directories(${MAIN_LIBRARY_NAME} PUBLIC ${THIS_SOURCE_DIR})
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LIMESDR_MINI_SOURCES})
