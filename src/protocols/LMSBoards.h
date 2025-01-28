/**
@file	LMSBoards.h
@author Lime Microsystems
@brief	enumerations of available LMS7002M hardware
*/
#pragma once

#include <array>
#include <string>
#include "limesuiteng/config.h"

namespace lime {

/// @brief The list of possible devices.
enum eLMS_DEV {
    LMS_DEV_UNKNOWN = 0,
    LMS_DEV_EVB6 = 1,
    LMS_DEV_DIGIGREEN = 2,
    LMS_DEV_DIGIRED = 3, //2x USB3, LMS6002,
    LMS_DEV_EVB7 = 4,
    LMS_DEV_ZIPPER = 5, //MyRiad bridge to FMC, HSMC bridge
    LMS_DEV_SOCKETBOARD = 6,
    LMS_DEV_EVB7V2 = 7,
    LMS_DEV_STREAM = 8, //Altera Cyclone IV, USB3, 2x 128 MB RAM, RFDIO, FMC
    LMS_DEV_NOVENA = 9, //Freescale iMX6 CPU
    LMS_DEV_DATASPARK = 10, //Altera Cyclone V, 2x 256 MB RAM, 2x FMC (HPC, LPC), USB3
    LMS_DEV_RFSPARK = 11, //LMS7002 EVB
    LMS_DEV_LMS6002USB = 12, //LM6002-USB (USB stick: FX3, FPGA, LMS6002, RaspberryPi con)
    LMS_DEV_RFESPARK = 13, //LMS7002 EVB
    LMS_DEV_LIMESDR = 14, //LimeSDR-USB, 32bit FX3, 2xRAM, LMS7
    LMS_DEV_LIMESDR_PCIE = 15,
    LMS_DEV_LIMESDR_QPCIE = 16, //2x LMS, 14 bit ADC and DAC
    LMS_DEV_LIMESDRMINI = 17, //FTDI + MAX10 + LMS
    LMS_DEV_USTREAM = 18, //with expansion boards (uMyriad)
    LMS_DEV_LIMESDR_SONY_PA = 19, //stand alone board with Sony PAs, tuners
    LMS_DEV_LIMESDR_USB_SP = 20,
    LMS_DEV_LMS7002M_ULTIMATE_EVB = 21,
    LMS_DEV_LIMENET_MICRO = 22, //Raspberry Pi CM3(L), Ethernet, MAX10, LMS7002,
    LMS_DEV_LIMESDR_CORE_SDR = 23, //LMS7002, Intel Cyclone 4, RAM, GNSS
    LMS_DEV_LIMESDR_CORE_HE = 24, //PA board
    LMS_DEV_LIMESDRMINI_V2 = 25, //FTDI + ECP5 + LMS
    LMS_DEV_LIMESDR_X3 = 26, // 3xLMS
    LMS_DEV_LIMESDR_XTRX = 27, // XTRX
    LMS_DEV_LIMESDR_MMX8 = 28,
    LMS_DEV_LIMESDR_MICRO = 29,
    LMS_DEV_EXTERNAL_XSDR = 30,

    LMS_DEV_COUNT
};

LIME_API const std::string_view GetDeviceName(const eLMS_DEV device);

/// @brief The list of possible expansion boards.
enum eEXP_BOARD {
    EXP_BOARD_UNKNOWN,
    EXP_BOARD_UNSUPPORTED,
    EXP_BOARD_NO,
    EXP_BOARD_MYRIAD1,
    EXP_BOARD_MYRIAD2,
    EXP_BOARD_MYRIAD_NOVENA,
    EXP_BOARD_HPM1000,
    EXP_BOARD_MYRIAD7,
    EXP_BOARD_HPM7,
    EXP_BOARD_MYRIAD7_NOVENA,

    EXP_BOARD_COUNT
};

const std::string_view GetExpansionBoardName(const eEXP_BOARD board);

} // namespace lime
