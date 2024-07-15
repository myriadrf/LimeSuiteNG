########################################################################
## Support for LimeSDR board
########################################################################
set(THIS_SOURCE_DIR boards/LimeSDR)

set(LIMESDR_SOURCES ${THIS_SOURCE_DIR}/LimeSDR.cpp ${THIS_SOURCE_DIR}/USB_CSR_Pipe_SDR.cpp)

########################################################################
## Add to library
########################################################################
target_include_directories(limesuiteng PRIVATE ${THIS_SOURCE_DIR})
target_sources(limesuiteng PRIVATE ${LIMESDR_SOURCES})
target_link_libraries(limesuiteng PRIVATE ${LIMESDR_LIBRARIES})
