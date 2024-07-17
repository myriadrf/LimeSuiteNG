set(LMS7002M_SOURCES
    LMS7002M_RegistersMap.cpp
    LMS7002M.cpp
    LMS7002MCSR_Data.cpp
    LMS7002MCSR_To_LMS7002MCSR_Data.cpp
    validation.cpp
    filtersCalibration.cpp
    gainCalibrations.cpp
    mcu_dc_iq_calibration.cpp
    MCU_BD.cpp
    MCU_File.cpp)

target_sources(limesuiteng PRIVATE ${LMS7002M_SOURCES})
target_link_libraries(limesuiteng PRIVATE $<BUILD_INTERFACE:lms7002m>)
target_link_libraries(limesuiteng PRIVATE $<TARGET_OBJECTS:lms_gfir>)

target_include_directories(limesuiteng PRIVATE ${PROJECT_SOURCE_DIR}/embedded/include)