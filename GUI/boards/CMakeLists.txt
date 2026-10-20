target_sources(
    limeGUI
    PRIVATE pnlBoardControls.cpp
            pnlGPIO.cpp
            pnlX3.cpp
            pnlX8.cpp
            pnlXTRX.cpp
            pnluLimeSDR.cpp
            pnlLimeSDR.cpp
            pnlGPIO_Interface.cpp
            pnlsSDR.cpp)

if (UNIX)
    target_sources(limeGUI PRIVATE pnlMicro.cpp)
endif()
