set(LMS7002M_SOURCES
    LMS7002M_RegistersMap.cpp
    LMS7002M.cpp
    LMS7002MCSR_Data.cpp
    LMS7002MCSR_To_LMS7002MCSR_Data.cpp
    LMS7002M_validation.cpp
    LMS7002M_RxTxCalibrations.cpp
    LMS7002M_BaseCalibrations.cpp
    LMS7002M_filtersCalibration.cpp
    LMS7002M_gainCalibrations.cpp
    mcu_dc_iq_calibration.cpp
    MCU_BD.cpp
    MCU_File.cpp
    spi.cpp
    lms7002m_calibrations.cpp
    lms7002m_controls.cpp)

target_sources(${MAIN_LIBRARY_NAME} PRIVATE ${LMS7002M_SOURCES})
