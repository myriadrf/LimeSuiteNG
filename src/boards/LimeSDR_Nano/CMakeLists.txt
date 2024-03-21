########################################################################
## Support for LimeSDR Nano board
########################################################################

set(THIS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/boards/LimeSDR_Nano)

set(LIMESDR_NANO_SOURCES
    ${THIS_SOURCE_DIR}/LimeSDR_Nano.cpp
)

########################################################################
## Add to library
########################################################################
target_include_directories(${MAIN_LIBRARY_NAME} PUBLIC ${THIS_SOURCE_DIR})
target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LIMESDR_NANO_SOURCES})
