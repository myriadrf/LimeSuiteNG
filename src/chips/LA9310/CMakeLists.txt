set(LA9310_SOURCES
    LA9310.cpp
    vspa/VSPA_mailbox.cpp
    # vspa/VSPA_iqplayer.cpp
    PHYTimer.cpp
    I2C.cpp
    CSR.cpp
)

add_subdirectory(firmware)
add_subdirectory(vspa)

target_sources(limesuiteng PRIVATE ${LA9310_SOURCES})
