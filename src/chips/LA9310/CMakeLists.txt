set(LA9310_SOURCES
    LA9310.cpp
    VSPA_mailbox.cpp
    VSPA_iqplayer.cpp
    PHYTimer.cpp
    I2C.cpp
    CSR.cpp
    VSPA_DMA.cpp
    dma_table.cpp
)

add_subdirectory(vspa)

target_sources(limesuiteng PRIVATE ${LA9310_SOURCES})
