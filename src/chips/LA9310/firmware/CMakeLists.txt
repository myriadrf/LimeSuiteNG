set(LA9310_SOURCES
    IQStreamer_DMA.cpp
    IQStreamer.cpp
    LA9310_FW_Impl.cpp
)

target_sources(limesuiteng PRIVATE ${LA9310_SOURCES})
