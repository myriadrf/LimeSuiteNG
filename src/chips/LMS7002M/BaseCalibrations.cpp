#include "limesuiteng/LMS7002M.h"
#include "LMS7002M_RegistersMap.h"
#include "LMS7002MCSR_Data.h"
#include <thread>
#include <chrono>
#include "limesuiteng/Logger.h"

using namespace std;

using namespace lime;
using namespace lime::LMS7002MCSR_Data;

OpStatus LMS7002M::CalibrateInternalADC(int clkDiv)
{
    if (Get_SPI_Reg_bits(MASK) == 0)
        return ReportError(OpStatus::NotSupported, "Operation not supported"s);

    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "Device not connected"s);

    const uint16_t biasMux = Get_SPI_Reg_bits(MUX_BIAS_OUT);
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, 1);

    SPI_write(0x0600, 0x0F01);
    SPI_write(0x0602, 0x2000);
    SPI_write(0x0603, 0x0000);
    Modify_SPI_Reg_bits(RSSI_PD, 0);
    Modify_SPI_Reg_bits(RSSI_RSSIMODE, 1);
    Modify_SPI_Reg_bits(DAC_CLKDIV, clkDiv);
    Modify_SPI_Reg_bits(RSSI_BIAS, 8);
    Modify_SPI_Reg_bits(RSSI_DAC_VAL, 170);

    uint8_t bias = Get_SPI_Reg_bits(RSSI_BIAS);
    uint16_t regValue = SPI_read(0x0601, true);
    while (((regValue >> 5) & 0x1) != 1)
    {
        if (bias > 31)
            return ReportError(OpStatus::Error, "Temperature internal ADC calibration failed"s);
        ++bias;
        Modify_SPI_Reg_bits(RSSI_BIAS, bias);
        regValue = SPI_read(0x0601, true);
    }
    Modify_SPI_Reg_bits(RSSI_PD, 0);
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, biasMux);
    Modify_SPI_Reg_bits(RSSI_RSSIMODE, 0);
    return OpStatus::Success;
}

OpStatus LMS7002M::CalibrateRP_BIAS()
{
    if (Get_SPI_Reg_bits(MASK) == 0)
        return ReportError(OpStatus::NotSupported, "Operation not supported"s);

    if (!controlPort)
        return ReportError(OpStatus::IOFailure, "Device not connected"s);

    CalibrateInternalADC(32);
    Modify_SPI_Reg_bits(RSSI_PD, 0);
    Modify_SPI_Reg_bits(RSSI_RSSIMODE, 0);

    const uint16_t biasMux = Get_SPI_Reg_bits(MUX_BIAS_OUT);
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, 1);
    this_thread::sleep_for(chrono::microseconds(250));
    uint16_t reg606 = SPI_read(0x0606, true);
    uint16_t Vref = (reg606 >> 8) & 0xFF;
    uint16_t Vptat = reg606 & 0xFF;

    if (Vref > Vptat)
    {
        uint16_t rpCalib = Get_SPI_Reg_bits(RP_CALIB_BIAS, true);
        while (Vref > Vptat)
        {
            --rpCalib;
            Modify_SPI_Reg_bits(RP_CALIB_BIAS, rpCalib);
            reg606 = SPI_read(0x0606, true);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    if (Vref < Vptat)
    {
        uint16_t rpCalib = Get_SPI_Reg_bits(RP_CALIB_BIAS, true);
        while (Vref < Vptat)
        {
            ++rpCalib;
            Modify_SPI_Reg_bits(RP_CALIB_BIAS, rpCalib);
            reg606 = SPI_read(0x0606, true);
            Vref = (reg606 >> 8) & 0xFF;
            Vptat = reg606 & 0xFF;
        }
    }
    Modify_SPI_Reg_bits(MUX_BIAS_OUT, biasMux);
    return OpStatus::Success;
}
