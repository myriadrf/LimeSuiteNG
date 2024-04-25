########################################################################
## Support for LimeSDR_X3 board
########################################################################

set(THIS_SOURCE_DIR boards/LimeSDR_X3)

set(LIMESDR_X3_SOURCES ${THIS_SOURCE_DIR}/FPGA_X3.cpp ${THIS_SOURCE_DIR}/LimeSDR_X3.cpp ${THIS_SOURCE_DIR}/SlaveSelectShim.cpp)

########################################################################
## Add to library
########################################################################
target_include_directories(${MAIN_LIBRARY_NAME} PRIVATE ${THIS_SOURCE_DIR})
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LIMESDR_X3_SOURCES})
