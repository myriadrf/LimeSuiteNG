#include "limesuiteng/LMS7002M.h"
#include "LMS7002MCSR_Data.h"
#include <assert.h>
#include "MCU_BD.h"
#include "mcu_programs.h"
#include <chrono>
#include <thread>
#include "limesuiteng/Logger.h"
#include "LMSBoards.h"

#ifndef NDEBUG
    #define LMS_VERBOSE_OUTPUT
#endif

using namespace std;
using namespace lime;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

// class BoardLoopbackStore
// {
// public:
//     BoardLoopbackStore(lime::IConnection* port) : port(port)
//     {
//         if(port && port->ReadRegister(LoopbackCtrAddr, mLoopbackState) != 0)
//             lime::ReportError(-1, "Failed to read Loopback controls"s);
//     }
//     ~BoardLoopbackStore()
//     {
//         if(port && port->WriteRegister(LoopbackCtrAddr, mLoopbackState) != 0)
//             lime::ReportError(-1, "Failed to restore Loopback controls"s);
//     }
// private:
//     lime::IConnection* port;
//     static const uint32_t LoopbackCtrAddr = 0x0017;
//     int mLoopbackState;
// };

static uint8_t GetExtLoopPair(lime::LMS7002M& ctr, bool calibratingTx)
{
    uint8_t loopPair = 0;
    // TODO;
    // lime::IConnection* port = ctr.GetConnection();
    // if(!port)
    //     return 0;

    // auto devName = port->GetDeviceInfo().deviceName;
    // uint8_t activeLNA = ctr.Get_SPI_Reg_bits(SEL_PATH_RFE);
    // uint8_t activeBand = (ctr.Get_SPI_Reg_bits(SEL_BAND2_TRF) << 1 | ctr.Get_SPI_Reg_bits(SEL_BAND1_TRF))-1;

    // if(devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDR))
    //     loopPair = 1 << 2 | 0x1; // band2 -> LNAH
    // else if(devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDRMINI))
    //     loopPair = activeBand << 2 | activeLNA;
    // else if (devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDRMINI_V2))
    //     loopPair = activeBand << 2 | activeLNA;
    return loopPair;
}

/*!
 * Convert the 12-bit twos compliment register into a signed integer
 */
static inline int16_t signextIqCorr(const uint16_t regVal)
{
    int16_t signedPhase = static_cast<int16_t>(regVal << 4);
    return signedPhase >> 4;
}

const double TrxCalib_RF_LimitLow = 2.5e6;
const double TrxCalib_RF_LimitHigh = 120e6;

/*
static int SetExtLoopback(IConnection* port, uint8_t ch, bool enable, bool tx)
{
    //enable external loopback switches
    const uint32_t LoopbackCtrAddr = 0x0017;
    uint32_t value = 0;
    int status;
    status = port->ReadRegister(LoopbackCtrAddr, value);
    if(status != 0)
        return -1;
    auto devName = port->GetDeviceInfo().deviceName;

    if(devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDR))
    {
        const uint16_t mask = 0x7;
        const uint8_t shiftCount = (ch==2 ? 4 : 0);
        value &= ~(mask << shiftCount);
        value |= enable << shiftCount;   //EN_Loopback
        value |= enable << (shiftCount+1); //EN_Attenuator
        value |= !enable << (shiftCount+2); //EN_Shunt
    }
    else if (devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDRMINI) || devName == lime::GetDeviceName(lime::LMS_DEV_LIMESDRMINI_V2))
    {
        //EN_Shunt
        value &= ~(1 << 2);
        value |= !enable << 2;

        if(tx)
        {
            uint32_t wr = 0x0103 << 16;
            uint32_t path;
            port->ReadLMS7002MSPI(&wr, &path, 1);
            path >>= 10;
            path &= 0x3;
            if (path==1)
            {
                value &= ~(1<<13);
                value |= 1<<12;

                value &= ~(1<<8); //LNA_H
                value |= 1<<9;
            }
            else if (path==2)
            {
                value &= ~(1<<12);
                value |= 1<<13;

                value &= ~(1<<9); //LNA_W
                value |= 1<<8;
            }
            //value |= enable << 1; //EN_Attenuator
        }
        else
        {
            uint32_t wr = 0x010D << 16;
            uint32_t path;
            port->ReadLMS7002MSPI(&wr, &path, 1);
            path &= ~0x0180;
            path >>= 7;
            if (path==1)
            {
                value &= ~(1<<8);
                value |= 1<<9;
            }
            else if (path==3)
            {
                value &= ~(1<<9);
                value |= 1<<8;
            }
        }
    }
    status = port->WriteRegister(LoopbackCtrAddr, value);
    if(status != 0)
        return ReportError(status, "Failed to enable external loopback"s);
    return status;
}
*/

uint16_t LMS7002M::GetRSSIDelayCounter()
{
    const uint16_t sampleCount = (2 << 7) << Get_SPI_Reg_bits(AGC_AVG_RXTSP);
    uint8_t decimation = Get_SPI_Reg_bits(HBD_OVR_RXTSP);
    if (decimation < 6)
        decimation = (2 << decimation);
    else
        decimation = 1; //bypass

    const float waitTime = sampleCount / ((GetReferenceClk_TSP(TRXDir::Rx) / 2) / decimation);
    return (0xFFFF) - static_cast<uint16_t>(waitTime * GetReferenceClk_SX(TRXDir::Rx) / 12);
}

/** @brief Flips the CAPTURE bit and returns digital RSSI value
*/
uint32_t LMS7002M::GetRSSI()
{
    const int waitTime = 1000000.0 * (0xFFFF - GetRSSIDelayCounter()) * 12 / GetReferenceClk_SX(TRXDir::Rx);
    std::this_thread::sleep_for(std::chrono::microseconds(waitTime));
    Modify_SPI_Reg_bits(CAPTURE, 0);
    Modify_SPI_Reg_bits(CAPTURE, 1);
    const uint32_t rssi = SPI_read(0x040F);
    return (rssi << 2 | (SPI_read(0x040E) & 0x3));
}

int16_t LMS7002M::ReadAnalogDC(const uint16_t addr)
{
    const uint16_t mask = addr < 0x05C7 ? 0x03FF : 0x003F;
    uint16_t value;
    int16_t result;
    SPI_write(addr, 0);
    SPI_write(addr, 0x4000);
    value = SPI_read(addr);
    SPI_write(addr, value & ~0xC000);
    result = (value & mask);
    if (value & (mask + 1))
        result *= -1;
    return result;
}

OpStatus LMS7002M::CalibrateTx(float_type bandwidth_Hz, bool useExtLoopback)
{
    if (TrxCalib_RF_LimitLow > bandwidth_Hz)
    {
        lime::warning(
            "Calibrating Tx for %g MHz (requested %g MHz [out of range])", TrxCalib_RF_LimitLow / 1e6, bandwidth_Hz / 1e6);
        bandwidth_Hz = TrxCalib_RF_LimitLow;
    }
    else if (bandwidth_Hz > TrxCalib_RF_LimitHigh)
    {
        lime::warning(
            "Calibrating Tx for %g MHz (requested %g MHz [out of range])", TrxCalib_RF_LimitHigh / 1e6, bandwidth_Hz / 1e6);
        bandwidth_Hz = TrxCalib_RF_LimitHigh;
    }
    if (controlPort == nullptr)
        return ReportError(OpStatus::InvalidValue, "Tx Calibration: Device not connected"s);
    auto beginTime = std::chrono::high_resolution_clock::now();
    int status;
    uint8_t ch = static_cast<uint8_t>(Get_SPI_Reg_bits(MAC));
    if (ch == 0 || ch == 3)
        return ReportError(OpStatus::InvalidValue, "Tx Calibration: Incorrect channel selection MAC %i", ch);

    //caching variables
    double txFreq = GetFrequencySX(TRXDir::Tx);
    uint8_t channel = ch == 1 ? 0 : 1;
    int band = Get_SPI_Reg_bits(SEL_BAND1_TRF) ? 0 : 1;

    int dccorri(0), dccorrq(0), gcorri(0), gcorrq(0), phaseOffset(0);
    lime::debug("Tx calibration using MCU %s loopback", useExtLoopback ? "EXTERNAL" : "INTERNAL");
    lime::debug("Tx ch.%s @ %4g MHz, BW: %g MHz, RF output: %s, Gain: %i",
        channel ? "B" : "A",
        txFreq / 1e6,
        bandwidth_Hz / 1e6,
        band ? "BAND2" : "BAND1",
        Get_SPI_Reg_bits(CG_IAMP_TBB));

    uint8_t mcuID = mcuControl->ReadMCUProgramID();
    lime::debug(
        "Current MCU firmware: %i, %s", mcuID, mcuID == MCU_ID_CALIBRATIONS_SINGLE_IMAGE ? "DC/IQ calibration full" : "unknown");
    if (mcuID != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
    {
        lime::debug("Uploading DC/IQ calibration firmware"s);
        OpStatus status = mcuControl->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
        if (status != OpStatus::Success)
            return status;
    }

    //set reference clock parameter inside MCU
    long refClk = GetReferenceClk_SX(TRXDir::Rx);
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_REF_CLK, refClk);
    lime::debug("MCU Ref. clock: %g MHz", refClk / 1e6);
    //Tx Rx separation bandwidth while calibrating
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_BW, bandwidth_Hz);

    {
        //BoardLoopbackStore onBoardLoopbackRestoration(GetConnection());
        if (useExtLoopback)
        {
            // TODO:
            // status = SetExtLoopback(controlPort, ch, true, true);
            // if(status != 0)
            //     return ReportError(OpStatus::InvalidValue, "Tx Calibration: Failed to enable external loopback"s);
            uint8_t loopPair = GetExtLoopPair(*this, true);
            mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_EXT_LOOPBACK_PAIR, loopPair);
        }
        mcuControl->RunProcedure(useExtLoopback ? MCU_FUNCTION_CALIBRATE_TX_EXTLOOPB : MCU_FUNCTION_CALIBRATE_TX);
        status = mcuControl->WaitForMCU(1000);
        if (status != MCU_BD::MCU_NO_ERROR)
            return ReportError(
                OpStatus::InvalidValue, "Tx Calibration: MCU error %i (%s)", status, MCU_BD::MCUStatusMessage(status).c_str());
    }

    //sync registers to cache
    const std::vector<uint16_t> regsToSync = { 0x0208, 0x05C0 };
    for (const auto addr : regsToSync)
        this->SPI_read(addr, true);

    //need to read back calibration results
    dccorri = ReadAnalogDC(channel ? DC_TXBI.address : DC_TXAI.address);
    dccorrq = ReadAnalogDC(channel ? DC_TXBQ.address : DC_TXAQ.address);
    gcorri = Get_SPI_Reg_bits(GCORRI_TXTSP, true);
    gcorrq = Get_SPI_Reg_bits(GCORRQ_TXTSP, true);
    phaseOffset = signextIqCorr(Get_SPI_Reg_bits(IQCORR_TXTSP, true));

    lime::info("Tx calibration finished"s);
    lime::debug("Tx | DC  | GAIN | PHASE"s);
    lime::debug("---+-----+------+------"s);
    lime::debug("I: | %3i | %4i | %i", dccorri, gcorri, phaseOffset);
    lime::debug("Q: | %3i | %4i |", dccorrq, gcorrq);
    int32_t duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - beginTime).count();
    lime::debug("Duration: %i ms", duration);
    return OpStatus::Success;
}

OpStatus LMS7002M::CalibrateRx(float_type bandwidth_Hz, bool useExtLoopback)
{
    if (TrxCalib_RF_LimitLow > bandwidth_Hz)
    {
        lime::warning(
            "Calibrating Rx for %g MHz (requested %g MHz [out of range])", TrxCalib_RF_LimitLow / 1e6, bandwidth_Hz / 1e6);
        bandwidth_Hz = TrxCalib_RF_LimitLow;
    }
    else if (bandwidth_Hz > TrxCalib_RF_LimitHigh)
    {
        lime::warning(
            "Calibrating Rx for %g MHz (requested %g MHz [out of range])", TrxCalib_RF_LimitHigh / 1e6, bandwidth_Hz / 1e6);
        bandwidth_Hz = TrxCalib_RF_LimitHigh;
    }
    if (controlPort == nullptr)
        return ReportError(OpStatus::IOFailure, "Rx Calibration: Device not connected"s);
#ifdef LMS_VERBOSE_OUTPUT
    auto beginTime = std::chrono::high_resolution_clock::now();
#endif

    uint8_t ch = static_cast<uint8_t>(Get_SPI_Reg_bits(MAC));
    if (ch == 0 || ch == 3)
        return ReportError(OpStatus::InvalidValue, "Rx Calibration: Incorrect channel selection MAC %i", ch);
    uint8_t channel = ch == 1 ? 0 : 1;
    uint8_t lna = static_cast<uint8_t>(Get_SPI_Reg_bits(SEL_PATH_RFE));
    double rxFreq = GetFrequencySX(TRXDir::Rx);

    const std::string_view lnaName = [lna]() {
        switch (lna)
        {
        case 0:
            return "none"sv;
        case 1:
            return "LNAH"sv;
        case 2:
            return "LNAL"sv;
        case 3:
            return "LNAW"sv;
        default:
            return "none"sv;
        }
    }();
    lime::debug("Rx calibration using %s loopback", (useExtLoopback ? "EXTERNAL" : "INTERNAL"));
    lime::debug("Rx ch.%s @ %4g MHz, BW: %g MHz, RF input: %s, PGA: %i, LNA: %i, TIA: %i",
        ch == static_cast<uint8_t>(Channel::ChA) ? "A" : "B",
        rxFreq / 1e6,
        bandwidth_Hz / 1e6,
        lnaName.data(),
        Get_SPI_Reg_bits(G_PGA_RBB),
        Get_SPI_Reg_bits(G_LNA_RFE),
        Get_SPI_Reg_bits(G_TIA_RFE));

    int dcoffi(0), dcoffq(0), gcorri(0), gcorrq(0), phaseOffset(0);
    //check if MCU has correct firmware
    uint8_t mcuID = mcuControl->ReadMCUProgramID();
    lime::debug(
        "Current MCU firmware: %i, %s", mcuID, mcuID == MCU_ID_CALIBRATIONS_SINGLE_IMAGE ? "DC/IQ calibration full" : "unknown");
    if (mcuID != MCU_ID_CALIBRATIONS_SINGLE_IMAGE)
    {
        lime::debug("Uploading DC/IQ calibration firmware"s);
        OpStatus status = mcuControl->Program_MCU(mcu_program_lms7_dc_iq_calibration_bin, MCU_BD::MCU_PROG_MODE::SRAM);
        if (status != OpStatus::Success)
            return status;
    }

    //set reference clock parameter inside MCU
    long refClk = GetReferenceClk_SX(TRXDir::Rx);
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_REF_CLK, refClk);
    lime::debug("MCU Ref. clock: %g MHz", refClk / 1e6);
    //Tx Rx separation bandwidth while calibrating
    mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_BW, bandwidth_Hz);

    {
        //BoardLoopbackStore onBoardLoopbackRestoration(GetConnection());
        if (useExtLoopback)
        {
            // TODO:
            // status = SetExtLoopback(controlPort, ch, true, false);
            // if(status != 0)
            //     return ReportError(OpStatus::InvalidValue, "Rx Calibration: Failed to enable external loopback"s);
            uint8_t loopPair = GetExtLoopPair(*this, false);
            mcuControl->SetParameter(MCU_BD::MCU_Parameter::MCU_EXT_LOOPBACK_PAIR, loopPair);
        }

        mcuControl->RunProcedure(useExtLoopback ? MCU_FUNCTION_CALIBRATE_RX_EXTLOOPB : MCU_FUNCTION_CALIBRATE_RX);
        int status = mcuControl->WaitForMCU(1000);
        if (status != MCU_BD::MCU_NO_ERROR)
            return ReportError(
                OpStatus::InvalidValue, "Rx calibration: MCU error %i (%s)", status, MCU_BD::MCUStatusMessage(status).c_str());
    }

    //sync registers to cache
    const std::vector<uint16_t> regsToSync = { 0x040C, 0x05C0 };
    for (const auto addr : regsToSync)
        this->SPI_read(addr, true);

    //read back for cache input and print
    dcoffi = ReadAnalogDC(channel ? DC_RXBI.address : DC_RXAI.address);
    dcoffq = ReadAnalogDC(channel ? DC_RXBQ.address : DC_RXAQ.address);
    gcorri = Get_SPI_Reg_bits(GCORRI_RXTSP, true);
    gcorrq = Get_SPI_Reg_bits(GCORRQ_RXTSP, true);
    phaseOffset = signextIqCorr(Get_SPI_Reg_bits(IQCORR_RXTSP, true));

    lime::info("Rx calibration finished"s);
    lime::debug("RX | DC  | GAIN | PHASE"s);
    lime::debug("---+-----+------+------"s);
    lime::debug("I: | %3i | %4i | %i", dcoffi, gcorri, phaseOffset);
    lime::debug("Q: | %3i | %4i |", dcoffq, gcorrq);
#ifdef LMS_VERBOSE_OUTPUT
    int32_t duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - beginTime).count();
    lime::debug("Duration: %i ms", duration);
#endif //LMS_VERBOSE_OUTPUT
    return OpStatus::Success;
}

OpStatus LMS7002M::LoadDC_REG_IQ(TRXDir dir, int16_t I, int16_t Q)
{
    if (dir == TRXDir::Tx)
    {
        Modify_SPI_Reg_bits(DC_REG_TXTSP, I);
        Modify_SPI_Reg_bits(TSGDCLDI_TXTSP, 0);
        Modify_SPI_Reg_bits(TSGDCLDI_TXTSP, 1);
        Modify_SPI_Reg_bits(TSGDCLDI_TXTSP, 0);
        Modify_SPI_Reg_bits(DC_REG_TXTSP, Q);
        Modify_SPI_Reg_bits(TSGDCLDQ_TXTSP, 0);
        Modify_SPI_Reg_bits(TSGDCLDQ_TXTSP, 1);
        Modify_SPI_Reg_bits(TSGDCLDQ_TXTSP, 0);
    }
    else
    {
        Modify_SPI_Reg_bits(DC_REG_RXTSP, I);
        Modify_SPI_Reg_bits(TSGDCLDI_RXTSP, 0);
        Modify_SPI_Reg_bits(TSGDCLDI_RXTSP, 1);
        Modify_SPI_Reg_bits(TSGDCLDI_RXTSP, 0);
        Modify_SPI_Reg_bits(DC_REG_RXTSP, Q);
        Modify_SPI_Reg_bits(TSGDCLDQ_RXTSP, 0);
        Modify_SPI_Reg_bits(TSGDCLDQ_RXTSP, 1);
        Modify_SPI_Reg_bits(TSGDCLDQ_RXTSP, 0);
    }
    return OpStatus::Success;
}
