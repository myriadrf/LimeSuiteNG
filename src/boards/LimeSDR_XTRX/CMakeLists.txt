########################################################################
## Support for LimeSDR_XTRX board
########################################################################

set(THIS_SOURCE_DIR boards/LimeSDR_XTRX)

set(LIMESDR_XTRX_SOURCES ${THIS_SOURCE_DIR}/LimeSDR_XTRX.cpp ${THIS_SOURCE_DIR}/FPGA_XTRX.cpp)

########################################################################
## Add to library
########################################################################
target_include_directories(limesuiteng PRIVATE ${THIS_SOURCE_DIR})
target_sources(limesuiteng PRIVATE ${LIMESDR_XTRX_SOURCES})
