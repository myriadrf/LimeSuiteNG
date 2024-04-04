########################################################################
## Support for LimeSDR board
########################################################################
set(THIS_SOURCE_DIR boards/LimeSDR)

set(LIMESDR_SOURCES
    ${THIS_SOURCE_DIR}/LimeSDR.cpp
    ${THIS_SOURCE_DIR}/USB_CSR_Pipe_SDR.cpp
)

########################################################################
## Add to library
########################################################################
target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR})
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LIMESDR_SOURCES})
target_link_libraries(${MAIN_LIBRARY_NAME} PRIVATE ${LIMESDR_LIBRARIES})
