set(LMS7002M_SOURCES
    LMS7002M_RegistersMap.cpp
    LMS7002M.cpp
    LMS7002MCSR_Data.cpp
    LMS7002MCSR_To_LMS7002MCSR_Data.cpp
    validation.cpp
    RxTxCalibrations.cpp
    BaseCalibrations.cpp
    filtersCalibration.cpp
    gainCalibrations.cpp
    mcu_dc_iq_calibration.cpp
    MCU_BD.cpp
    MCU_File.cpp
    spi.cpp
    calibrations.cpp
    controls.cpp)

target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LMS7002M_SOURCES})
