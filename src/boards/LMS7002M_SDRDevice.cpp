#include "LMS7002M_SDRDevice.h"

#include "FPGA_common.h"
#include "limesuiteng/LMS7002M.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "chips/LMS7002M/MCU_BD.h"
#include "LMSBoards.h"
#include "limesuiteng/Logger.h"
#include "TRXLooper.h"
#include "utilities/toString.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>

using namespace std::literals::string_literals;

namespace lime {
using namespace lime::LMS7002MCSR_Data;

#ifdef NEW_GAIN_BEHAVIOUR
constexpr static int MAXIMUM_GAIN_VALUE = 62; // Gain table size
// clang-format off
// LNA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> LNATable = {
    0,  0,  0,  1,  1,  1,  2,  2,  2,  3,  3,  3,  4,  4,  4,  5,
    5,  5,  6,  6,  6,  7,  7,  7,  8,  9,  10, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 11, 11, 11, 12, 13, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14
};
// PGA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> PGATable = {
    0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,
    1,  2,  0,  1,  2,  0,  1,  2,  0,  0,  0,  0,  1,  2,  3,  4,
    5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};
// clang-format on
#else
constexpr static int MAXIMUM_GAIN_VALUE = 74;
// clang-format off
// LNA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> LNATable = {
    0,  0,  0,  1,  1,  1,  2,  2,  2,  3,  3,  3,  4,  4,  4,  5,
    5,  5,  6,  6,  6,  7,  7,  7,  8,  9,  10, 11, 11, 11, 11, 11,
    11, 11, 11, 11, 11, 11, 11, 11, 12, 13, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14
};
// PGA table
constexpr static std::array<unsigned int, MAXIMUM_GAIN_VALUE> PGATable = {
    0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,  1,  2,  0,
    1,  2,  0,  1,  2,  0,  1,  2,  0,  0,  0,  0,  1,  2,  3,  4,
    5,  6,  7,  8,  9,  10, 11, 12, 12, 12, 12, 4,  5,  6,  7,  8,
    9,  10, 11, 12, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31
};
// clang-format on
#endif

LMS7002M_SDRDevice::LMS7002M_SDRDevice()
    : mCallback_logMessage(nullptr)
{
}

LMS7002M_SDRDevice::~LMS7002M_SDRDevice()
{
}

OpStatus LMS7002M_SDRDevice::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    if (moduleIndex >= mLMSChips.size())
        return OpStatus::InvalidValue;
    return mLMSChips.at(moduleIndex)->EnableChannel(trx, channel % 2, enable);
}

void LMS7002M_SDRDevice::SetMessageLogCallback(LogCallbackType callback)
{
    mCallback_logMessage = callback;
    for (auto& looper : mStreamers)
        looper->SetMessageLogCallback(mCallback_logMessage);
}

const SDRDescriptor& LMS7002M_SDRDevice::GetDescriptor() const
{
    return mDeviceDescriptor;
}

OpStatus LMS7002M_SDRDevice::Reset()
{
    OpStatus status;
    for (auto& iter : mLMSChips)
    {
        status = iter->ResetChip();
        if (status != OpStatus::Success)
            return status;
    }
    return OpStatus::Success;
}

OpStatus LMS7002M_SDRDevice::GetGPSLock(GPS_Lock* status)
{
    uint16_t regValue = mFPGA->ReadRegister(0x114);
    status->glonass = static_cast<GPS_Lock::LockStatus>((regValue >> 0) & 0x3);
    status->gps = static_cast<GPS_Lock::LockStatus>((regValue >> 4) & 0x3);
    status->beidou = static_cast<GPS_Lock::LockStatus>((regValue >> 8) & 0x3);
    status->galileo = static_cast<GPS_Lock::LockStatus>((regValue >> 12) & 0x3);
    // TODO: not all boards have GPS
    return OpStatus::Success;
}

double LMS7002M_SDRDevice::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate)
{
    if (moduleIndex >= mLMSChips.size())
    {
        ReportError(OpStatus::OutOfRange, "GetSample rate invalid module index (%i)", moduleIndex);
        return 0;
    }
    double sampleRate = mLMSChips[moduleIndex]->GetSampleRate(trx, LMS7002M::Channel::ChA);
    if (rf_samplerate)
    {
        int oversample_control = mLMSChips[moduleIndex]->Get_SPI_Reg_bits(trx == TRXDir::Rx ? HBD_OVR_RXTSP : HBI_OVR_TXTSP);
        if (oversample_control != 7)
            *rf_samplerate = sampleRate * (2 << oversample_control);
        else
            *rf_samplerate = sampleRate;
    }
    return sampleRate;
}

double LMS7002M_SDRDevice::GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    lime::LMS7002M* lms = mLMSChips.at(moduleIndex).get();
    if (trx == TRXDir::Rx)
    {
        LMS7002M::ChannelScope scope(lms, LMS7002M::Channel::ChSXT);
        if (lms->Get_SPI_Reg_bits(PD_LOCH_T2RBUF) == 0) // TDD mode, return SXT LO
            trx = TRXDir::Tx;
    }
    return lms->GetFrequencySX(trx);
}

OpStatus LMS7002M_SDRDevice::SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency)
{
    lime::LMS7002M* lms = mLMSChips.at(moduleIndex).get();
    int64_t oppositeDirLO = lms->GetFrequencySX(trx == TRXDir::Rx ? TRXDir::Tx : TRXDir::Rx);
    OpStatus status = lms->SetFrequencySX(trx, frequency);
    // Readback of LO frequency might not exactly match what was requested, so compare with some margin
    bool useTDD = (abs(oppositeDirLO - frequency) <= 20);
    lms->EnableSXTDD(useTDD);
    return status;
}

double LMS7002M_SDRDevice::GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    double phaseOffset = 0.0;
    return GetFrequency(moduleIndex, trx, channel) - GetNCOFrequency(moduleIndex, trx, channel, 0, phaseOffset);
}

double LMS7002M_SDRDevice::GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset)
{
    auto& lms = mLMSChips.at(moduleIndex);

    lms->SetActiveChannel(channel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    double freq = lms->GetNCOFrequency(trx, index, true);

    bool down = lms->Get_SPI_Reg_bits(trx == TRXDir::Tx ? CMIX_SC_TXTSP : CMIX_SC_RXTSP);
    if (!(trx == TRXDir::Tx) && (lms->Get_SPI_Reg_bits(MASK) == 0))
    {
        down = !down;
    }

    uint16_t value = lms->SPI_read(trx == TRXDir::Tx ? 0x0241 : 0x0441);
    phaseOffset = 360.0 * value / 65536.0;

    return down ? -freq : freq;
}

OpStatus LMS7002M_SDRDevice::SetNCOFrequency(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset)
{
    if (index > 15)
        return ReportError(OpStatus::OutOfRange, "%s NCO%i index invalid", ToString(trx).c_str(), index);

    auto& lms = mLMSChips.at(moduleIndex);

    lms->SetActiveChannel(channel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);

    bool enable = frequency != 0;
    bool tx = trx == TRXDir::Tx;

    if ((lms->Modify_SPI_Reg_bits(tx ? CMIX_BYP_TXTSP : CMIX_BYP_RXTSP, !enable) != OpStatus::Success) ||
        (lms->Modify_SPI_Reg_bits(tx ? CMIX_GAIN_TXTSP : CMIX_GAIN_RXTSP, enable) != OpStatus::Success))
    {
        return ReportError(OpStatus::Error, "Failed to set %s NCO%i frequency", ToString(trx).c_str(), index);
    }

    OpStatus status = lms->SetNCOFrequency(trx, index, std::fabs(frequency));
    if (status != OpStatus::Success)
        return status;

    if (enable)
    {
        bool down = frequency < 0;
        if ((!tx) && (lms->Get_SPI_Reg_bits(MASK) == 0))
        {
            down = !down;
        }

        if ((lms->Modify_SPI_Reg_bits(tx ? SEL_TX : SEL_RX, index) != OpStatus::Success) ||
            (lms->Modify_SPI_Reg_bits(tx ? MODE_TX : MODE_RX, 0) != OpStatus::Success) ||
            (lms->Modify_SPI_Reg_bits(tx ? CMIX_SC_TXTSP : CMIX_SC_RXTSP, down) != OpStatus::Success))
        {
            return ReportError(OpStatus::Error, "Failed to set %s NCO%i frequency", ToString(trx).c_str(), index);
        }
    }

    if (phaseOffset != -1.0)
        return lms->SetNCOPhaseOffsetForMode0(trx, phaseOffset);
    return OpStatus::Success;
}

int LMS7002M_SDRDevice::GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    const auto& selParameter = trx == TRXDir::Tx ? SEL_TX : SEL_RX;
    return GetParameter(moduleIndex, channel, selParameter.address, selParameter.msb, selParameter.lsb);
}

OpStatus LMS7002M_SDRDevice::SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv)
{
    auto& cmixBypassParameter = trx == TRXDir::Tx ? CMIX_BYP_TXTSP : CMIX_BYP_RXTSP;
    auto& cmixGainParameter = trx == TRXDir::Tx ? CMIX_GAIN_TXTSP : CMIX_GAIN_RXTSP;
    auto& selectionParameter = trx == TRXDir::Tx ? SEL_TX : SEL_RX;
    auto& cmixSelectionParameter = trx == TRXDir::Tx ? CMIX_SC_TXTSP : CMIX_SC_RXTSP;

    if (OpStatus status = SetParameter(
            moduleIndex, channel, cmixBypassParameter.address, cmixBypassParameter.msb, cmixBypassParameter.lsb, index < 0 ? 1 : 0);
        status != OpStatus::Success)
        return status;
    if (OpStatus status = SetParameter(
            moduleIndex, channel, cmixGainParameter.address, cmixGainParameter.msb, cmixGainParameter.lsb, index < 0 ? 0 : 1);
        status != OpStatus::Success)
        return status;

    if (index >= 16)
    {
        lime::error("Invalid NCO index value."s);
        return OpStatus::OutOfRange;
    }

    if (OpStatus status =
            SetParameter(moduleIndex, channel, selectionParameter.address, selectionParameter.msb, selectionParameter.lsb, index);
        status != OpStatus::Success)
        return status;

    if (OpStatus status = SetParameter(
            moduleIndex, channel, cmixSelectionParameter.address, cmixSelectionParameter.msb, cmixSelectionParameter.lsb, downconv);
        status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

double LMS7002M_SDRDevice::GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return lowPassFilterCache[trx][channel]; // Default initializes to 0
}

OpStatus LMS7002M_SDRDevice::SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf)
{
    auto& lms = mLMSChips.at(moduleIndex);

    LMS7002M::Channel ch = channel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB;

    lms->SetActiveChannel(ch);

    const auto& bw_range = mDeviceDescriptor.rfSOC.at(moduleIndex).lowPassFilterRange.at(trx);
    bool tx = trx == TRXDir::Tx;

    if (lpf < 0)
    {
        lpf = lowPassFilterCache[trx][channel]; // Default initializes to 0
    }

    double newLPF = std::clamp(lpf, bw_range.min, bw_range.max);

    if (newLPF != lpf)
    {
        lime::warning("%cXLPF set to %.3f MHz (requested %0.3f MHz [out of range])", tx ? 'T' : 'R', newLPF / 1e6, lpf / 1e6);
    }

    lpf = newLPF;
    lowPassFilterCache[trx][channel] = lpf;

    OpStatus status = OpStatus::Success;
    if (tx)
    {
        int gain = lms->GetTBBIAMP_dB(ch);
        status = lms->SetTxLPF(lpf);
        lms->SetTBBIAMP_dB(gain, ch);
    }
    else
    {
        status = lms->SetRxLPF(lpf);
    }

    if (status != OpStatus::Success)
        return status;

    lime::info(ToString(trx) + " LPF configured"s);
    return OpStatus::Success;
}

uint8_t LMS7002M_SDRDevice::GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    auto& lms = mLMSChips.at(moduleIndex);

    if (trx == TRXDir::Tx)
    {
        return lms->GetBandTRF();
    }
    else
    {
        return static_cast<uint8_t>(lms->GetPathRFE());
    }
}

OpStatus LMS7002M_SDRDevice::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    if (path >= mDeviceDescriptor.rfSOC.at(moduleIndex).pathNames.at(trx).size())
        lime::error("Out of bounds antenna path");

    auto& lms = mLMSChips.at(moduleIndex);

    return lms->SetPath(trx, channel % 2, path);
}

OpStatus LMS7002M_SDRDevice::Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->SetActiveChannel(static_cast<LMS7002M::Channel>((channel % 2) + 1));
    OpStatus ret;
    auto reg20 = lms->SPI_read(0x20);
    lms->SPI_write(0x20, reg20 | (20 << (channel % 2)));
    if (trx == TRXDir::Tx)
    {
        ret = lms->CalibrateTx(bandwidth, false);
    }
    else
    {
        ret = lms->CalibrateRx(bandwidth, false);
    }
    lms->SPI_write(0x20, reg20);
    return ret;
}

OpStatus LMS7002M_SDRDevice::ConfigureGFIR(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings)
{
    auto& lms = mLMSChips.at(moduleIndex);
    LMS7002M::Channel enumChannel = channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;

    return lms->SetGFIRFilter(trx, enumChannel, settings.enabled, settings.bandwidth);
}

OpStatus LMS7002M_SDRDevice::SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value)
{
    auto& device = mLMSChips.at(moduleIndex);
    LMS7002M::Channel enumChannel = channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;

    switch (gain)
    {
    case eGainTypes::LNA:
        return device->SetRFELNA_dB(value, enumChannel);
    case eGainTypes::LoopbackLNA:
        return device->SetRFELoopbackLNA_dB(value, enumChannel);
    case eGainTypes::PGA:
        return device->SetRBBPGA_dB(value, enumChannel);
    case eGainTypes::TIA:
        return device->SetRFETIA_dB(value, enumChannel);
    case eGainTypes::PAD:
        return device->SetTRFPAD_dB(value, enumChannel);
    case eGainTypes::IAMP:
        return device->SetTBBIAMP_dB(value, enumChannel);
    case eGainTypes::LoopbackPAD:
        return device->SetTRFLoopbackPAD_dB(value, enumChannel);
    case eGainTypes::PA:
        // TODO: implement
        return OpStatus::Error;
    case eGainTypes::UNKNOWN:
    default:
        if (TRXDir::Tx == direction)
            return SetGenericTxGain(*device, enumChannel, value);
        else
            return SetGenericRxGain(*device, enumChannel, value);
    }
}

OpStatus LMS7002M_SDRDevice::SetGenericTxGain(lime::LMS7002M& chip, LMS7002M::Channel channel, double value)
{
    if (chip.SetTRFPAD_dB(value, channel) != OpStatus::Success)
        return OpStatus::Error;

#ifdef NEW_GAIN_BEHAVIOUR
    if (value <= 0)
    {
        return chip.Modify_SPI_Reg_bits(LMS7002MCSR::CG_IAMP_TBB, 1);
    }

    if (chip.GetTBBIAMP_dB(channel) < 0.0)
    {
        return chip.CalibrateTxGain(0, nullptr);
    }
#else
    value -= chip.GetTRFPAD_dB(channel);
    if (chip.SetTBBIAMP_dB(value, channel) != OpStatus::Success)
    {
        return OpStatus::Error;
    }
#endif
    return OpStatus::Success;
}

OpStatus LMS7002M_SDRDevice::SetGenericRxGain(lime::LMS7002M& chip, LMS7002M::Channel channel, double value)
{
    value = std::clamp(static_cast<int>(value + 12), 0, MAXIMUM_GAIN_VALUE - 1);

    unsigned int lna = LNATable.at(std::lround(value));
    unsigned int pga = PGATable.at(std::lround(value));

    unsigned int tia = 0;
#ifdef NEW_GAIN_BEHAVIOUR
    if (value > 0)
    {
        tia = 1;
    }
#else
    // TIA table
    if (value > 51)
    {
        tia = 2;
    }
    else if (value > 42)
    {
        tia = 1;
    }
#endif
    int rcc_ctl_pga_rbb = (430 * (pow(0.65, pga / 10.0)) - 110.35) / 20.4516 + 16; // From data sheet

    // TODO: optimize into single write batch
    if ((chip.Modify_SPI_Reg_bits(LMS7002MCSR::G_LNA_RFE, lna + 1) != OpStatus::Success) ||
        (chip.Modify_SPI_Reg_bits(LMS7002MCSR::G_TIA_RFE, tia + 1) != OpStatus::Success) ||
        (chip.Modify_SPI_Reg_bits(LMS7002MCSR::G_PGA_RBB, pga) != OpStatus::Success) ||
        (chip.Modify_SPI_Reg_bits(LMS7002MCSR::RCC_CTL_PGA_RBB, rcc_ctl_pga_rbb) != OpStatus::Success))
    {
        return OpStatus::IOFailure;
    }

    return OpStatus::Success;
}

OpStatus LMS7002M_SDRDevice::GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value)
{
    auto& device = mLMSChips.at(moduleIndex);
    LMS7002M::Channel enumChannel = channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;

    switch (gain)
    {
    case eGainTypes::LNA:
        value = device->GetRFELNA_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::LoopbackLNA:
        value = device->GetRFELoopbackLNA_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::PGA:
        value = device->GetRBBPGA_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::TIA:
        value = device->GetRFETIA_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::PAD:
        value = device->GetTRFPAD_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::IAMP:
        value = device->GetTBBIAMP_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::LoopbackPAD:
        value = device->GetTRFLoopbackPAD_dB(enumChannel);
        return OpStatus::Success;
    case eGainTypes::PA:
        // TODO: implement
        return OpStatus::Error;
    case eGainTypes::UNKNOWN:
    default:
#ifdef NEW_GAIN_BEHAVIOUR
        if (TRXDir::Tx == direction)
        {
            value = device->GetTRFPAD_dB(enumChannel);
        }
        else
        {
            value = device->GetRFELNA_dB(enumChannel) + device->GetRBBPGA_dB(enumChannel);
        }
#else
        if (TRXDir::Tx == direction)
        {
            value = device->GetTRFPAD_dB(enumChannel) + device->GetTBBIAMP_dB(enumChannel);
        }
        else
        {
            value = device->GetRFELNA_dB(enumChannel) + device->GetRFETIA_dB(enumChannel) + device->GetRBBPGA_dB(enumChannel);
        }
#endif
        return OpStatus::Success;
    }
}

bool LMS7002M_SDRDevice::GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (trx == TRXDir::Rx)
    {
        auto& lms = mLMSChips.at(moduleIndex);
        return lms->Get_SPI_Reg_bits(LMS7002MCSR::DC_BYP_RXTSP, channel) == 0;
    }

    return false;
}

OpStatus LMS7002M_SDRDevice::SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic)
{
    if (trx == TRXDir::Tx)
        return OpStatus::NotSupported;

    auto& lms = mLMSChips.at(moduleIndex);
    return lms->Modify_SPI_Reg_bits(LMS7002MCSR::DC_BYP_RXTSP, isAutomatic == 0, channel);
}

complex64f_t LMS7002M_SDRDevice::GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    double I = 0.0;
    double Q = 0.0;

    auto& lms = mLMSChips.at(moduleIndex);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, (channel % 2) + 1);
    lms->GetDCOffset(trx, I, Q);
    return complex64f_t(I, Q);
}

OpStatus LMS7002M_SDRDevice::SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, (channel % 2) + 1);
    return lms->SetDCOffset(trx, offset.real(), offset.imag());
}

complex64f_t LMS7002M_SDRDevice::GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, (channel % 2) + 1);

    double phase = 0.0, gainI = 0.0, gainQ = 0.0;
    lms->GetIQBalance(trx, phase, gainI, gainQ);
    auto balance = (gainI / gainQ) * std::polar(1.0, phase);
    return { balance.real(), balance.imag() };
}

OpStatus LMS7002M_SDRDevice::SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance)
{
    std::complex<double> bal{ balance.real(), balance.imag() };
    double gain = std::abs(bal);

    double gainI = 1.0;
    if (gain < 1.0)
    {
        gainI = gain;
    }

    double gainQ = 1.0;
    if (gain > 1.0)
    {
        gainQ = 1.0 / gain;
    }

    auto& lms = mLMSChips.at(moduleIndex);
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, (channel % 2) + 1);
    return lms->SetIQBalance(trx, std::arg(bal), gainI, gainQ);
}

bool LMS7002M_SDRDevice::GetCGENLocked(uint8_t moduleIndex)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->GetCGENLocked();
}

double LMS7002M_SDRDevice::GetTemperature(uint8_t moduleIndex)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->GetTemperature();
}

bool LMS7002M_SDRDevice::GetSXLocked(uint8_t moduleIndex, TRXDir trx)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->GetSXLocked(trx);
}

unsigned int LMS7002M_SDRDevice::ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA)
{
    if (useFPGA)
    {
        return ReadFPGARegister(address);
    }

    auto& lms = mLMSChips.at(moduleIndex);
    return lms->SPI_read(address);
}

OpStatus LMS7002M_SDRDevice::WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA)
{
    if (useFPGA)
        return WriteFPGARegister(address, value);

    return mLMSChips.at(moduleIndex)->SPI_write(address, value);
}

OpStatus LMS7002M_SDRDevice::LoadConfig(uint8_t moduleIndex, const std::string& filename)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->LoadConfig(filename);
}

OpStatus LMS7002M_SDRDevice::SaveConfig(uint8_t moduleIndex, const std::string& filename)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->SaveConfig(filename);
}

uint16_t LMS7002M_SDRDevice::GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->SetActiveChannel(channel % 2 == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);

    try
    {
        uint16_t val = lms->Get_SPI_Reg_bits(lms->GetParam(parameterKey));
        return val;
    } catch (...)
    {
        throw std::runtime_error("failure getting key: "s + parameterKey);
    }
}

OpStatus LMS7002M_SDRDevice::SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->SetActiveChannel(channel % 2 == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    return lms->Modify_SPI_Reg_bits(lms->GetParam(parameterKey), value);
}

uint16_t LMS7002M_SDRDevice::GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->SetActiveChannel(channel % 2 == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);

    try
    {
        uint16_t val = lms->Get_SPI_Reg_bits(address, msb, lsb);
        return val;
    } catch (...)
    {
        // TODO: fix return
        throw std::runtime_error("failure setting parameter: "s + std::to_string(address));
    }
}

OpStatus LMS7002M_SDRDevice::SetParameter(
    uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    auto& lms = mLMSChips.at(moduleIndex);
    lms->SetActiveChannel(channel % 2 == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    return lms->Modify_SPI_Reg_bits(address, msb, lsb, value);
}

OpStatus LMS7002M_SDRDevice::Synchronize(bool toChip)
{
    OpStatus status = OpStatus::Success;
    for (auto& iter : mLMSChips)
    {
        status = toChip ? iter->UploadAll() : iter->DownloadAll();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

void LMS7002M_SDRDevice::EnableCache(bool enable)
{
    for (auto& iter : mLMSChips)
        iter->EnableValuesCache(enable);
    if (mFPGA)
        mFPGA->EnableValuesCache(enable);
}

void* LMS7002M_SDRDevice::GetInternalChip(uint32_t index)
{
    if (index >= mLMSChips.size())
        throw std::logic_error("Invalid chip index"s);
    return mLMSChips.at(index).get();
}

uint64_t LMS7002M_SDRDevice::GetHardwareTimestamp(uint8_t moduleIndex)
{
    return mStreamers.at(moduleIndex)->GetHardwareTimestamp();
}

OpStatus LMS7002M_SDRDevice::SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now)
{
    // TODO: return status
    mStreamers.at(moduleIndex)->SetHardwareTimestamp(now);
    return OpStatus::Success;
}

OpStatus LMS7002M_SDRDevice::SetTestSignal(uint8_t moduleIndex,
    TRXDir direction,
    uint8_t channel,
    ChannelConfig::Direction::TestSignal signalConfiguration,
    int16_t dc_i,
    int16_t dc_q)
{
    auto& lms = mLMSChips.at(moduleIndex);

    bool div4 = signalConfiguration.divide == ChannelConfig::Direction::TestSignal::Divide::Div4;
    bool fullscale = signalConfiguration.scale == ChannelConfig::Direction::TestSignal::Scale::Full;

    switch (direction)
    {
    case TRXDir::Rx:
        if (lms->Modify_SPI_Reg_bits(LMS7002MCSR::INSEL_RXTSP, signalConfiguration.enabled, true) != OpStatus::Success)
            return ReportError(OpStatus::IOFailure, "Failed to set Rx test signal."s);

        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGFCW_RXTSP, div4 ? 2 : 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGFC_RXTSP, fullscale ? 1 : 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGMODE_RXTSP, signalConfiguration.dcMode);
        break;
    case TRXDir::Tx:
        if (lms->Modify_SPI_Reg_bits(LMS7002MCSR::INSEL_TXTSP, signalConfiguration.enabled, true) != OpStatus::Success)
            return ReportError(OpStatus::IOFailure, "Failed to set Tx test signal."s);

        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGFCW_TXTSP, div4 ? 2 : 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGFC_TXTSP, fullscale ? 1 : 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::TSGMODE_TXTSP, signalConfiguration.dcMode);
        break;
    }

    if (signalConfiguration.dcMode)
        return lms->LoadDC_REG_IQ(direction, dc_i, dc_q);

    return OpStatus::Success;
}

ChannelConfig::Direction::TestSignal LMS7002M_SDRDevice::GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel)
{
    auto& lms = mLMSChips.at(moduleIndex);
    ChannelConfig::Direction::TestSignal signalConfiguration;

    switch (direction)
    {
    case TRXDir::Tx:
        if (lms->Get_SPI_Reg_bits(LMS7002MCSR::INSEL_TXTSP) == 0)
        {
            return signalConfiguration;
        }
        signalConfiguration.enabled = true;

        if (lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGMODE_TXTSP) != 0)
        {
            signalConfiguration.dcMode = true;
            return signalConfiguration;
        }

        signalConfiguration.divide =
            static_cast<ChannelConfig::Direction::TestSignal::Divide>(lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGFCW_TXTSP));
        signalConfiguration.scale =
            static_cast<ChannelConfig::Direction::TestSignal::Scale>(lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGFC_TXTSP));

        return signalConfiguration;
    case TRXDir::Rx:
        if (lms->Get_SPI_Reg_bits(LMS7002MCSR::INSEL_RXTSP) == 0)
        {
            return signalConfiguration;
        }
        signalConfiguration.enabled = true;

        if (lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGMODE_RXTSP) != 0)
        {
            signalConfiguration.dcMode = true;
            return signalConfiguration;
        }

        signalConfiguration.divide =
            static_cast<ChannelConfig::Direction::TestSignal::Divide>(lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGFCW_RXTSP));
        signalConfiguration.scale =
            static_cast<ChannelConfig::Direction::TestSignal::Scale>(lms->Get_SPI_Reg_bits(LMS7002MCSR::TSGFC_RXTSP));

        return signalConfiguration;
    }

    throw std::runtime_error("Failed to get test mode"s);
}

std::vector<double> LMS7002M_SDRDevice::GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID)
{
    auto& lms = mLMSChips.at(moduleIndex);

    const uint8_t count = gfirID == 2 ? 120 : 40;
    std::vector<double> coefficientBuffer(count);

    lms->GetGFIRCoefficients(trx, gfirID, coefficientBuffer.data(), count);

    return coefficientBuffer;
}

OpStatus LMS7002M_SDRDevice::SetGFIRCoefficients(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->SetGFIRCoefficients(trx, gfirID, coefficients.data(), coefficients.size());
}

OpStatus LMS7002M_SDRDevice::SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled)
{
    auto& lms = mLMSChips.at(moduleIndex);

    if (gfirID > 2)
        return ReportError(OpStatus::OutOfRange, "Failed to set GFIR filter, invalid filter index %i.", gfirID);

    std::vector<std::reference_wrapper<const CSRegister>> txGfirBypasses = { GFIR1_BYP_TXTSP, GFIR2_BYP_TXTSP, GFIR3_BYP_TXTSP };
    std::vector<std::reference_wrapper<const CSRegister>> rxGfirBypasses = { GFIR1_BYP_RXTSP, GFIR2_BYP_RXTSP, GFIR3_BYP_RXTSP };

    lms->SetActiveChannel(static_cast<LMS7002M::Channel>((channel % 2) + 1));
    OpStatus status;
    if (trx == TRXDir::Tx)
        status = lms->Modify_SPI_Reg_bits(txGfirBypasses[gfirID], enabled == false);
    else
        status = lms->Modify_SPI_Reg_bits(rxGfirBypasses[gfirID], enabled == false);
    return status;
}

OpStatus LMS7002M_SDRDevice::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    if (moduleIndex > mStreamers.size())
        return OpStatus::InvalidValue;

    if (mStreamers.at(moduleIndex)->IsStreamRunning())
        return OpStatus::Busy;

    return mStreamers.at(moduleIndex)->Setup(config);
}

void LMS7002M_SDRDevice::StreamStart(uint8_t moduleIndex)
{
    mStreamers.at(moduleIndex)->Start();
}

void LMS7002M_SDRDevice::StreamStop(uint8_t moduleIndex)
{
    assert(moduleIndex < mStreamers.size());
    mStreamers.at(moduleIndex)->Stop();
}

void LMS7002M_SDRDevice::StreamDestroy(uint8_t moduleIndex)
{
    assert(moduleIndex < mStreamers.size());
    mStreamers.at(moduleIndex)->Teardown();
}

uint32_t LMS7002M_SDRDevice::StreamRx(uint8_t moduleIndex, complex32f_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamRx(dest, count, meta);
}

uint32_t LMS7002M_SDRDevice::StreamRx(uint8_t moduleIndex, complex16_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamRx(dest, count, meta);
}

uint32_t LMS7002M_SDRDevice::StreamRx(uint8_t moduleIndex, complex12_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamRx(dest, count, meta);
}

uint32_t LMS7002M_SDRDevice::StreamTx(
    uint8_t moduleIndex, const complex32f_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamTx(samples, count, meta);
}

uint32_t LMS7002M_SDRDevice::StreamTx(
    uint8_t moduleIndex, const complex16_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamTx(samples, count, meta);
}

uint32_t LMS7002M_SDRDevice::StreamTx(
    uint8_t moduleIndex, const complex12_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mStreamers.at(moduleIndex)->StreamTx(samples, count, meta);
}

void LMS7002M_SDRDevice::StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx)
{
    auto& trx = mStreamers.at(moduleIndex);
    if (rx != nullptr)
        *rx = trx->GetStats(TRXDir::Rx);

    if (tx != nullptr)
        *tx = trx->GetStats(TRXDir::Tx);
}

OpStatus LMS7002M_SDRDevice::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    return OpStatus::NotImplemented;
}

RFSOCDescriptor LMS7002M_SDRDevice::GetDefaultLMS7002MDescriptor()
{
    RFSOCDescriptor soc;
    // LMS#1
    soc.name = "LMS7002M"s;
    soc.channelCount = 2;
    soc.pathNames[TRXDir::Rx] = { "None"s, "LNAH"s, "LNAL"s, "LNAW"s, "LB1"s, "LB2"s };
    soc.pathNames[TRXDir::Tx] = { "None"s, "Band1"s, "Band2"s };

    soc.samplingRateRange = { 100e3, 61.44e6, 0 };
    soc.frequencyRange = { 100e3, 3.8e9, 0 };

    soc.lowPassFilterRange[TRXDir::Rx] = { 1.4001e6, 130e6 };
    soc.lowPassFilterRange[TRXDir::Tx] = { 5e6, 130e6 };

    soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 2e9, 2.6e9 };
    soc.antennaRange[TRXDir::Rx]["LNAL"s] = { 700e6, 900e6 };
    soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 700e6, 2.6e9 };
    soc.antennaRange[TRXDir::Rx]["LB1"s] = soc.antennaRange[TRXDir::Rx]["LNAL"s];
    soc.antennaRange[TRXDir::Rx]["LB2"s] = soc.antennaRange[TRXDir::Rx]["LNAW"s];
    soc.antennaRange[TRXDir::Tx]["Band1"s] = { 30e6, 1.9e9 };
    soc.antennaRange[TRXDir::Tx]["Band2"s] = { 2e9, 2.6e9 };

    SetGainInformationInDescriptor(soc);
    return soc;
}

OpStatus LMS7002M_SDRDevice::UpdateFPGAInterfaceFrequency(LMS7002M& soc, FPGA& fpga, uint8_t chipIndex)
{
    double fpgaTxPLL = soc.GetReferenceClk_TSP(TRXDir::Tx);
    int interp = soc.Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    if (interp != 7)
    {
        uint8_t siso = soc.Get_SPI_Reg_bits(LML1_SISODDR);
        fpgaTxPLL /= std::pow(2, interp + siso);
    }
    double fpgaRxPLL = soc.GetReferenceClk_TSP(TRXDir::Rx);
    int dec = soc.Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    if (dec != 7)
    {
        uint8_t siso = soc.Get_SPI_Reg_bits(LML2_SISODDR);
        fpgaRxPLL /= std::pow(2, dec + siso);
    }

    OpStatus status = fpga.SetInterfaceFreq(fpgaTxPLL, fpgaRxPLL, chipIndex);
    if (status != OpStatus::Success)
        return status;
    soc.ResetLogicRegisters();
    return OpStatus::Success;
}

int LMS7002M_SDRDevice::ReadFPGARegister(uint32_t address)
{
    return mFPGA->ReadRegister(address);
}

OpStatus LMS7002M_SDRDevice::WriteFPGARegister(uint32_t address, uint32_t value)
{
    return mFPGA->WriteRegister(address, value);
}

void LMS7002M_SDRDevice::SetGainInformationInDescriptor(RFSOCDescriptor& descriptor)
{
    descriptor.gainValues[TRXDir::Rx][eGainTypes::LNA] = { { 1, 0 },
        { 2, 3 },
        { 3, 6 },
        { 4, 9 },
        { 5, 12 },
        { 6, 15 },
        { 7, 18 },
        { 8, 21 },
        { 9, 24 },
        { 10, 25 },
        { 11, 26 },
        { 12, 27 },
        { 13, 28 },
        { 14, 29 },
        { 15, 30 } };
    descriptor.gainValues[TRXDir::Rx][eGainTypes::TIA] = { { 1, 0 }, { 2, 9 }, { 3, 12 } };

    std::vector<GainValue> PGAParameter(32);
    for (uint8_t i = 0; i < PGAParameter.size(); ++i)
    {
        PGAParameter[i] = { i, static_cast<float>(i - 12) };
    }
    descriptor.gainValues[TRXDir::Rx][eGainTypes::PGA] = PGAParameter;

    std::vector<GainValue> IAMPParameter(63);
    for (uint8_t i = 1; i <= IAMPParameter.size(); ++i)
    {
        IAMPParameter[i - 1] = { i, static_cast<float>(i) };
    }
    descriptor.gainValues[TRXDir::Tx][eGainTypes::IAMP] = IAMPParameter;

    std::vector<GainValue> PADParameter(31);
    for (uint8_t i = 0; i < PADParameter.size(); ++i)
    {
        PADParameter[i] = { i, static_cast<float>(i) };
    }
    descriptor.gainValues[TRXDir::Tx][eGainTypes::PAD] = PADParameter;

    descriptor.gains[TRXDir::Rx] = {
        eGainTypes::LNA,
        eGainTypes::PGA,
        eGainTypes::TIA,
    };

    descriptor.gains[TRXDir::Tx] = {
        eGainTypes::PAD,
        eGainTypes::IAMP,
    };

    descriptor.gainRange[TRXDir::Rx][eGainTypes::LNA] = Range(0, 30);
    descriptor.gainRange[TRXDir::Rx][eGainTypes::LoopbackLNA] = Range(0, 40);
    descriptor.gainRange[TRXDir::Rx][eGainTypes::TIA] = Range(0, 12);
    descriptor.gainRange[TRXDir::Rx][eGainTypes::PGA] = Range(-12, 19);
    descriptor.gainRange[TRXDir::Tx][eGainTypes::PAD] = Range(0, 52);
    descriptor.gainRange[TRXDir::Tx][eGainTypes::LoopbackPAD] = Range(-4.3, 0);
    descriptor.gainRange[TRXDir::Tx][eGainTypes::IAMP] = Range(-12, 12);

#ifdef NEW_GAIN_BEHAVIOUR
    descriptor.gainRange[TRXDir::Rx][eGainTypes::UNKNOWN] = Range(-12, 49);
    descriptor.gainRange[TRXDir::Tx][eGainTypes::UNKNOWN] = Range(0, 52);
#else
    descriptor.gainRange[TRXDir::Rx][eGainTypes::UNKNOWN] = Range(-12, 61);
    descriptor.gainRange[TRXDir::Tx][eGainTypes::UNKNOWN] = Range(-12, 64);
#endif
}

OpStatus LMS7002M_SDRDevice::LMS7002LOConfigure(LMS7002M& chip, const SDRConfig& cfg)
{
    OpStatus status = OpStatus::Success;
    if (cfg.referenceClockFreq != 0)
    {
        status = chip.SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, cfg.referenceClockFreq);
        if (status != OpStatus::Success)
            return status;
    }

    const bool tddMode = cfg.channel[0].rx.centerFrequency == cfg.channel[0].tx.centerFrequency;
    {
        // TODO: verify if every FPGA gateware has this
        // configure FPGA to do TDD switching
        uint16_t reg000A = mFPGA->ReadRegister(0x000A);
        reg000A &= ~(1 << 11);
        if (tddMode)
            reg000A |= (1 << 11);
        mFPGA->WriteRegister(0x000A, reg000A);
    }
    // Rx PLL is not used in TDD mode
    if (cfg.channel[0].rx.centerFrequency > 0)
    {
        status = chip.SetFrequencySX(TRXDir::Rx, cfg.channel[0].rx.centerFrequency);
        if (status != OpStatus::Success)
            return status;
    }
    if (cfg.channel[0].tx.centerFrequency > 0)
    {
        status = chip.SetFrequencySX(TRXDir::Tx, cfg.channel[0].tx.centerFrequency);
        if (status != OpStatus::Success)
            return status;
    }
    chip.EnableSXTDD(tddMode);
    return status;
}

OpStatus LMS7002M_SDRDevice::LMS7002ChannelConfigure(LMS7002M& chip, const ChannelConfig& config, uint8_t channelIndex)
{
    OpStatus status;
    const ChannelConfig& ch = config;
    chip.SetActiveChannel((channelIndex & 1) ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA);

    chip.EnableChannel(TRXDir::Rx, channelIndex, ch.rx.enabled);
    chip.SetPathRFE(static_cast<LMS7002M::PathRFE>(ch.rx.path));
    if (static_cast<LMS7002M::PathRFE>(ch.rx.path) == LMS7002M::PathRFE::LB1 ||
        static_cast<LMS7002M::PathRFE>(ch.rx.path) == LMS7002M::PathRFE::LB2)
    {
        // TODO: confirm which should be used for loopback
        if (ch.rx.lpf > 0)
            chip.Modify_SPI_Reg_bits(INPUT_CTL_PGA_RBB, 3); // baseband loopback
        else
            chip.Modify_SPI_Reg_bits(INPUT_CTL_PGA_RBB, 2); // LPF bypass
    }

    chip.EnableChannel(TRXDir::Tx, channelIndex, ch.tx.enabled);
    chip.SetBandTRF(ch.tx.path);

    for (const auto& gain : ch.rx.gain)
    {
        SetGain(0, TRXDir::Rx, channelIndex, gain.first, gain.second);
    }

    for (const auto& gain : ch.tx.gain)
    {
        SetGain(0, TRXDir::Tx, channelIndex, gain.first, gain.second);
    }

    status = chip.SetRxLPF(ch.rx.lpf);
    if (status != OpStatus::Success)
        return status;
    status = chip.SetTxLPF(ch.tx.lpf);
    if (status != OpStatus::Success)
        return status;
    // TODO: set GFIR filters...
    return status;
}

OpStatus LMS7002M_SDRDevice::LMS7002ChannelCalibration(LMS7002M& chip, const ChannelConfig& config, uint8_t channelIndex)
{
    int i = channelIndex;
    auto enumChannel = i == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB;
    chip.SetActiveChannel(enumChannel);
    const ChannelConfig& ch = config;

    // TODO: Don't configure GFIR when external ADC/DAC is used
    if (ch.rx.enabled && chip.SetGFIRFilter(TRXDir::Rx, enumChannel, ch.rx.gfir.enabled, ch.rx.gfir.bandwidth) != OpStatus::Success)
        return lime::ReportError(OpStatus::Error, "Rx ch%i GFIR config failed", i);
    if (ch.tx.enabled && chip.SetGFIRFilter(TRXDir::Tx, enumChannel, ch.tx.gfir.enabled, ch.tx.gfir.bandwidth) != OpStatus::Success)
        return lime::ReportError(OpStatus::Error, "Tx ch%i GFIR config failed", i);

    OpStatus rxStatus = OpStatus::Success;
    if (ch.rx.calibrate && ch.rx.enabled)
    {
        rxStatus = chip.CalibrateRx(ch.rx.sampleRate);
    }

    OpStatus txStatus = OpStatus::Success;
    if (ch.tx.calibrate && ch.tx.enabled)
    {
        txStatus = chip.CalibrateTx(ch.tx.sampleRate);
    }

    if (rxStatus != OpStatus::Success || txStatus != OpStatus::Success)
        return OpStatus::Error;
    return OpStatus::Success;
}

OpStatus LMS7002M_SDRDevice::LMS7002TestSignalConfigure(LMS7002M& chip, const ChannelConfig& config, uint8_t channelIndex)
{
    const ChannelConfig& ch = config;
    chip.Modify_SPI_Reg_bits(INSEL_RXTSP, ch.rx.testSignal.enabled ? 1 : 0);
    if (ch.rx.testSignal.enabled)
    {
        const ChannelConfig::Direction::TestSignal& signal = ch.rx.testSignal;
        bool fullscale = signal.scale == ChannelConfig::Direction::TestSignal::Scale::Full;
        bool div4 = signal.divide == ChannelConfig::Direction::TestSignal::Divide::Div4;
        chip.Modify_SPI_Reg_bits(TSGFC_RXTSP, fullscale ? 1 : 0);
        chip.Modify_SPI_Reg_bits(TSGFCW_RXTSP, div4 ? 2 : 1);
        chip.Modify_SPI_Reg_bits(TSGMODE_RXTSP, signal.dcMode ? 1 : 0);
        chip.SPI_write(0x040C, 0x01FF); // DC.. bypasss
        // TSGMODE_RXTSP change resets DC values
        return chip.LoadDC_REG_IQ(TRXDir::Rx, signal.dcValue.real(), signal.dcValue.imag());
    }

    chip.Modify_SPI_Reg_bits(INSEL_TXTSP, ch.tx.testSignal.enabled ? 1 : 0);
    if (ch.tx.testSignal.enabled)
    {
        const ChannelConfig::Direction::TestSignal& signal = ch.tx.testSignal;
        bool fullscale = signal.scale == ChannelConfig::Direction::TestSignal::Scale::Full;
        bool div4 = signal.divide == ChannelConfig::Direction::TestSignal::Divide::Div4;
        chip.Modify_SPI_Reg_bits(TSGFC_TXTSP, fullscale ? 1 : 0);
        chip.Modify_SPI_Reg_bits(TSGFCW_TXTSP, div4 ? 2 : 1);
        chip.Modify_SPI_Reg_bits(TSGMODE_TXTSP, signal.dcMode ? 1 : 0);
        chip.SPI_write(0x040C, 0x01FF); // DC.. bypasss
        // TSGMODE_TXTSP change resets DC values
        return chip.LoadDC_REG_IQ(TRXDir::Tx, signal.dcValue.real(), signal.dcValue.imag());
    }
    return OpStatus::Success;
}

} // namespace lime
