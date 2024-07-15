########################################################################
## Support for LimeMM-X8 board
########################################################################

set(THIS_SOURCE_DIR boards/MMX8)

set(MM_X8_SOURCES
    ${THIS_SOURCE_DIR}/LMS64C_ADF_Over_PCIe_MMX8.cpp
    ${THIS_SOURCE_DIR}/LMS64C_FPGA_Over_PCIe_MMX8.cpp
    ${THIS_SOURCE_DIR}/LMS64C_LMS7002M_Over_PCIe_MMX8.cpp
    ${THIS_SOURCE_DIR}/MM_X8.cpp)

########################################################################
## Add to library
########################################################################
target_include_directories(limesuiteng PRIVATE ${THIS_SOURCE_DIR})
target_sources(limesuiteng PRIVATE ${MM_X8_SOURCES})
