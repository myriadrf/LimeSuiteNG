set(VSPA_SOURCES
    VSPA_Trace.cpp
    vspa_loader.c
    vspa_recovery.c
)

target_sources(limesuiteng PRIVATE ${VSPA_SOURCES})
