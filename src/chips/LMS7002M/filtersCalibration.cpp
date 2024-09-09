/**
@file	filtersCalibration.cpp
@author Lime Microsystems (www.limemicro.com)
@brief	Implementation of LMS7002M transceiver filters calibration algorithms
*/

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "LMS7002M_RegistersMap.h"
#include "LMS7002MCSR_Data.h"
#include <cmath>
#include <iostream>
#include <cassert>
#include "MCU_BD.h"
#include "mcu_programs.h"

#ifdef _MSC_VER
    #include <ciso646>
#endif
using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

//rx lpf range limits
static const float_type RxLPF_RF_LimitLow = 1.4e6;
static const float_type RxLPF_RF_LimitHigh = 130e6;

LMS7002M_RegistersMap* LMS7002M::BackupRegisterMap(void)
{
    //BackupAllRegisters(); return NULL;
    auto backup = new LMS7002M_RegistersMap();
    Channel chBck = GetActiveChannel();
    SetActiveChannel(Channel::ChA);
    *backup = *mRegistersMap;
    SetActiveChannel(chBck);
    return backup;
}

void LMS7002M::RestoreRegisterMap(LMS7002M_RegistersMap* backup)
{
    //RestoreAllRegisters(); return;
    Channel chBck = GetActiveChannel();

    for (int ch = 0; ch < 2; ch++)
    {
        //determine addresses that have been changed
        //and restore backup to the main register map
        std::vector<uint16_t> restoreAddrs, restoreData;
        for (const uint16_t addr : mRegistersMap->GetUsedAddresses(ch))
        {
            uint16_t original = backup->GetValue(ch, addr);
            uint16_t current = mRegistersMap->GetValue(ch, addr);
            mRegistersMap->SetValue(ch, addr, original);

            if (ch == 1 and addr < 0x0100)
                continue;
            if (original == current)
                continue;
            restoreAddrs.push_back(addr);
            restoreData.push_back(original);
        }

        //bulk write the original register values from backup
        SetActiveChannel((ch == 0) ? Channel::ChA : Channel::ChB);
        SPI_write_batch(restoreAddrs.data(), restoreData.data(), restoreData.size(), true);
    }

    //cleanup
    delete backup;
    backup = nullptr;
    SetActiveChannel(chBck);
}

OpStatus LMS7002M::TuneRxFilter(float_type rx_lpf_freq_RF)
{
    int status;
    if (RxLPF_RF_LimitLow > rx_lpf_freq_RF || rx_lpf_freq_RF > RxLPF_RF_LimitHigh)
        return ReportError(OpStatus::OutOfRange,
            "RxLPF frequency out of range, available range from %g to %g MHz",
            RxLPF_RF_LimitLow / 1e6,
            RxLPF_RF_LimitHigh / 1e6);

    uint8_t g_tia = Get_SPI_Reg_bits(LMS7002MCSR::G_TIA_RFE);
    if (g_tia == 1 && rx_lpf_freq_RF < 4e6)
    {
        rx_lpf_freq_RF = 4e6;
        lime::warning("Rx LPF min bandwidth is 4MHz when TIA gain is set to -12 dB"s);
    }

    if (mcuControl->ReadMCUProgramID() != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
    {
        OpStatus status = mcuControl->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
        if (status != OpStatus::Success)
            return ReportError(status, "Tune Rx Filter: failed to program MCU"s);
    }

    //set reference clock parameter inside MCU
    long refClk = GetReferenceClk_SX(TRXDir::Rx);
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_REF_CLK, refClk);
    lime::debug("MCU Ref. clock: %g MHz", refClk / 1e6);
    //set bandwidth for MCU to read from register, value is integer stored in MHz
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_BW, rx_lpf_freq_RF);
    mcuControl->RunProcedure(5);

    status = mcuControl->WaitForMCU(1000);
    if (status != MCU_BD::MCU_NO_ERROR)
    {
        lime::error("Tune Rx Filter: MCU error %i (%s)", status, MCU_BD::MCUStatusMessage(status).c_str());
        return OpStatus::Error;
    }
    //sync registers to cache
    std::vector<uint16_t> regsToSync = { 0x0112, 0x0117, 0x011A, 0x0116, 0x0118, 0x0114, 0x0019, 0x0115 };
    for (const auto addr : regsToSync)
        SPI_read(addr, true);

    return OpStatus::Success;
}
