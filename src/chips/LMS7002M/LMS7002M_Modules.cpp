#include "LMS7002M_Modules.h"

#include "DCCorrector.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/LMS7002MCSR.h"

#include <cassert>
#include <cmath>

namespace lime {

// LMS7002M_RFPath

LMS7002M_RFPath::LMS7002M_RFPath(LMS7002M* chip, TRXDir dir, uint8_t channel)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
{
    assert(chip);
}

std::vector<IRFPath::PathDescriptor> LMS7002M_RFPath::GetPaths() const
{
    // Same names and recommended ranges as the LMS7002M RFSOCDescriptor in LMS7002M_SDRDevice
    if (mDir == TRXDir::Tx)
        return {
            { "None", Range<double>(0, 0, 0) },
            { "Band1", Range<double>(2e9, 2.6e9, 0) },
            { "Band2", Range<double>(30e6, 1.9e9, 0) },
        };
    return {
        { "None", Range<double>(0, 0, 0) },
        { "LNAH", Range<double>(2e9, 2.6e9, 0) },
        { "LNAL", Range<double>(700e6, 900e6, 0) },
        { "LNAW", Range<double>(700e6, 2.6e9, 0) },
        { "LB1", Range<double>(0, 0, 0) },
        { "LB2", Range<double>(0, 0, 0) },
    };
}

// GetPaths() list positions are the hardware values:
// Rx - LMS7002M::PathRFE, Tx - TRF band number (0 - none).
OpStatus LMS7002M_RFPath::SetPath(const std::string& name)
{
    const std::vector<PathDescriptor> paths = GetPaths();
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        if (paths[i].name == name)
            return mChip->SetPath(mDir, mChannel, i);
    }
    return OpStatus::InvalidValue;
}

std::string LMS7002M_RFPath::GetPath()
{
    std::size_t index;
    {
        LMS7002M::ChannelScope scope(mChip, mChannel);
        index = mDir == TRXDir::Tx ? mChip->GetBandTRF() : static_cast<std::size_t>(mChip->GetPathRFE());
    }
    const std::vector<PathDescriptor> paths = GetPaths();
    return index < paths.size() ? paths[index].name : std::string();
}

// LMS7002M_Sensor

LMS7002M_Sensor::LMS7002M_Sensor(std::string name, Units units, ReadFunction read)
    : mName(std::move(name))
    , mUnits(units)
    , mRead(std::move(read))
{
    assert(mRead);
}

// LMS7002M_GainStage

LMS7002M_GainStage::LMS7002M_GainStage(
    LMS7002M* chip, TRXDir dir, uint8_t channel, eGainTypes type, std::string name, Units units, Range<double> range)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
    , mName(std::move(name))
    , mUnits(units)
    , mRange(range)
    , mType(type)
{
    assert(chip);
}

static LMS7002M::Channel ToChipChannel(uint8_t channel)
{
    return channel > 0 ? LMS7002M::Channel::ChB : LMS7002M::Channel::ChA;
}

OpStatus LMS7002M_GainStage::SetValue(double value)
{
    const LMS7002M::Channel ch = ToChipChannel(mChannel);
    switch (mType)
    {
    case eGainTypes::LNA:
        return mChip->SetRFELNA_dB(value, ch);
    case eGainTypes::LoopbackLNA:
        return mChip->SetRFELoopbackLNA_dB(value, ch);
    case eGainTypes::TIA:
        return mChip->SetRFETIA_dB(value, ch);
    case eGainTypes::PGA:
        return mChip->SetRBBPGA_dB(value, ch);
    case eGainTypes::IAMP:
        return mChip->SetTBBIAMP_dB(value, ch);
    case eGainTypes::PAD:
        return mChip->SetTRFPAD_dB(value, ch);
    case eGainTypes::LoopbackPAD:
        return mChip->SetTRFLoopbackPAD_dB(value, ch);
    case eGainTypes::GENERIC:
        return mDir == TRXDir::Rx ? mChip->SetGenericRxGain_dB(value, ch) : mChip->SetTRFPAD_dB(value, ch);
    default:
        return OpStatus::NotImplemented;
    }
}

OpStatus LMS7002M_GainStage::GetValue(double& value)
{
    const LMS7002M::Channel ch = ToChipChannel(mChannel);
    switch (mType)
    {
    case eGainTypes::LNA:
        value = mChip->GetRFELNA_dB(ch);
        break;
    case eGainTypes::LoopbackLNA:
        value = mChip->GetRFELoopbackLNA_dB(ch);
        break;
    case eGainTypes::TIA:
        value = mChip->GetRFETIA_dB(ch);
        break;
    case eGainTypes::PGA:
        value = mChip->GetRBBPGA_dB(ch);
        break;
    case eGainTypes::IAMP:
        value = mChip->GetTBBIAMP_dB(ch);
        break;
    case eGainTypes::PAD:
        value = mChip->GetTRFPAD_dB(ch);
        break;
    case eGainTypes::LoopbackPAD:
        value = mChip->GetTRFLoopbackPAD_dB(ch);
        break;
    case eGainTypes::GENERIC:
        value = mDir == TRXDir::Rx ? mChip->GetGenericRxGain_dB(ch) : mChip->GetTRFPAD_dB(ch);
        break;
    default:
        return OpStatus::NotImplemented;
    }
    return OpStatus::Success;
}

// LMS7002M_LPF

LMS7002M_LPF::LMS7002M_LPF(LMS7002M* chip, TRXDir dir, uint8_t channel)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
    , mBandwidth_Hz(0)
{
    assert(chip);
}

OpStatus LMS7002M_LPF::SetBandwidth(double bandwidth_Hz)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    const OpStatus status = mDir == TRXDir::Tx ? mChip->SetTxLPF(bandwidth_Hz) : mChip->SetRxLPF(bandwidth_Hz);
    if (status == OpStatus::Success)
        mBandwidth_Hz = bandwidth_Hz;
    return status;
}

double LMS7002M_LPF::GetBandwidth()
{
    return mBandwidth_Hz;
}

Range<double> LMS7002M_LPF::GetBandwidthRange() const
{
    if (mDir == TRXDir::Tx)
        return Range<double>(5e6, 130e6, 0);
    return Range<double>(1.4001e6, 130e6, 0);
}

// LMS7002M_NCO

LMS7002M_NCO::LMS7002M_NCO(LMS7002M* chip, TRXDir dir, uint8_t channel)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
{
    assert(chip);
}

OpStatus LMS7002M_NCO::Enable(bool enable)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Modify_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::CMIX_BYP_TXTSP : LMS7002MCSR::CMIX_BYP_RXTSP, !enable);
}

bool LMS7002M_NCO::IsEnabled()
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Get_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::CMIX_BYP_TXTSP : LMS7002MCSR::CMIX_BYP_RXTSP) == 0;
}

OpStatus LMS7002M_NCO::SelectPreset(uint8_t index)
{
    if (index >= GetPresetCount())
        return OpStatus::OutOfRange;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Modify_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::SEL_TX : LMS7002MCSR::SEL_RX, index);
}

uint8_t LMS7002M_NCO::GetSelectedPreset()
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Get_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::SEL_TX : LMS7002MCSR::SEL_RX);
}

Range<double> LMS7002M_NCO::GetFrequencyRange()
{
    return Range<double>(0, mChip->GetReferenceClk_TSP(mDir) / 2, 0);
}

OpStatus LMS7002M_NCO::SetFrequency(uint8_t index, double frequency_Hz)
{
    if (index >= GetPresetCount())
        return OpStatus::OutOfRange;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->SetNCOFrequency(mDir, index, frequency_Hz);
}

double LMS7002M_NCO::GetFrequency(uint8_t index)
{
    if (index >= GetPresetCount())
        return 0;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->GetNCOFrequency(mDir, index);
}

OpStatus LMS7002M_NCO::SetPhaseOffset(uint8_t index, double phase_deg)
{
    if (index >= GetPresetCount())
        return OpStatus::OutOfRange;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->SetNCOPhaseOffset(mDir, index, phase_deg);
}

double LMS7002M_NCO::GetPhaseOffset(uint8_t index)
{
    if (index >= GetPresetCount())
        return 0;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->GetNCOPhases(mDir).at(index);
}

// LMS7002M_GFIR

static LMS7002MCSR GFIRBypassRegister(TRXDir dir, uint8_t index)
{
    static constexpr LMS7002MCSR tx[] = {
        LMS7002MCSR::GFIR1_BYP_TXTSP, LMS7002MCSR::GFIR2_BYP_TXTSP, LMS7002MCSR::GFIR3_BYP_TXTSP
    };
    static constexpr LMS7002MCSR rx[] = {
        LMS7002MCSR::GFIR1_BYP_RXTSP, LMS7002MCSR::GFIR2_BYP_RXTSP, LMS7002MCSR::GFIR3_BYP_RXTSP
    };
    return dir == TRXDir::Tx ? tx[index] : rx[index];
}

LMS7002M_GFIR::LMS7002M_GFIR(LMS7002M* chip, TRXDir dir, uint8_t channel, uint8_t index)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
    , mIndex(index)
{
    assert(chip);
    assert(index < 3);
}

OpStatus LMS7002M_GFIR::Enable(bool enable)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Modify_SPI_Reg_bits(GFIRBypassRegister(mDir, mIndex), !enable);
}

bool LMS7002M_GFIR::IsEnabled()
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->Get_SPI_Reg_bits(GFIRBypassRegister(mDir, mIndex)) == 0;
}

std::size_t LMS7002M_GFIR::GetMaxCoefficientCount()
{
    // Same limits as lms7002m_set_gfir_coefficients(): bankCount * min(oversample, 8)
    LMS7002M::ChannelScope scope(mChip, mChannel);
    const uint16_t ovr = mChip->Get_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::HBI_OVR_TXTSP : LMS7002MCSR::HBD_OVR_RXTSP);
    const std::size_t oversample = (ovr != 7) ? (2 << ovr) : 1; // 7 is bypass, otherwise 2**(ovr+1)
    const std::size_t bankCount = mIndex < 2 ? 5 : 15;
    const std::size_t bankLength = oversample > 8 ? 8 : oversample;
    return bankCount * bankLength;
}

OpStatus LMS7002M_GFIR::SetCoefficients(std::span<const double> coefficients)
{
    if (coefficients.size() > GetMaxCoefficientCount())
        return OpStatus::OutOfRange;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->SetGFIRCoefficients(mDir, mIndex, coefficients.data(), coefficients.size());
}

OpStatus LMS7002M_GFIR::GetCoefficients(std::vector<double>& coefficients)
{
    // Coefficient registers count, the readback order matches what was set only if the max count was used.
    coefficients.resize(mIndex < 2 ? 40 : 120);
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->GetGFIRCoefficients(mDir, mIndex, coefficients.data(), coefficients.size());
}

// LMS7002M_LOMixer

LMS7002M_LOMixer::LMS7002M_LOMixer(LMS7002M* chip, TRXDir dir)
    : mChip(chip)
    , mDir(dir)
{
    assert(chip);
}

Range<double> LMS7002M_LOMixer::GetFrequencyRange() const
{
    // VCO range 3.8 GHz - 7.714 GHz, divided by 2**(DIV_LOCH+1), DIV_LOCH 0..6
    return Range<double>(3800e6 / 128, 7714e6 / 2, 0);
}

OpStatus LMS7002M_LOMixer::SetFrequency(double frequency_Hz)
{
    return mChip->SetFrequencySX(mDir, frequency_Hz);
}

double LMS7002M_LOMixer::GetFrequency()
{
    return mChip->GetFrequencySX(mDir);
}

bool LMS7002M_LOMixer::IsLocked()
{
    return mChip->GetSXLocked(mDir);
}

// LMS7002M_Oversampler

LMS7002M_Oversampler::LMS7002M_Oversampler(LMS7002M* chip, TRXDir dir)
    : mChip(chip)
    , mDir(dir)
{
    assert(chip);
}

static uint8_t OversampleToOVR(uint32_t oversample_pow2)
{
    return oversample_pow2 == 0 ? 7 : oversample_pow2 - 1; // 7 is bypass, otherwise oversample = 2**(ovr+1)
}

static uint32_t OVRToOversample(uint16_t ovr)
{
    return ovr == 7 ? 0 : ovr + 1;
}

OpStatus LMS7002M_Oversampler::SetOversample(uint32_t oversample_pow2)
{
    if (oversample_pow2 > 5)
        return OpStatus::OutOfRange;

    // HBI/HBD are per channel registers, but the TSP clock divider configured from them is common, so set both channels.
    LMS7002M::ChannelScope scope(mChip, LMS7002M::Channel::ChAB);
    uint8_t hbi = mChip->Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    uint8_t hbd = mChip->Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    if (mDir == TRXDir::Tx)
        hbi = OversampleToOVR(oversample_pow2);
    else
        hbd = OversampleToOVR(oversample_pow2);
    return mChip->SetInterfaceFrequency(mChip->GetFrequencyCGEN(), hbi, hbd);
}

uint32_t LMS7002M_Oversampler::GetOversample()
{
    LMS7002M::ChannelScope scope(mChip, LMS7002M::Channel::ChA);
    return OVRToOversample(mChip->Get_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::HBI_OVR_TXTSP : LMS7002MCSR::HBD_OVR_RXTSP));
}

// LMS7002M_QEC

LMS7002M_QEC::LMS7002M_QEC(LMS7002M* chip, TRXDir dir, uint8_t channel)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
{
    assert(chip);
}

static void GainImbalanceToGains(float iq_gain_imb, double& gainI, double& gainQ)
{
    // GCORR can only attenuate, so attenuate the branch that has to be reduced, keep the other at unity.
    const double ratio = std::pow(10.0, std::abs(iq_gain_imb) / 20.0);
    gainI = iq_gain_imb > 0 ? 1.0 / ratio : 1.0;
    gainQ = iq_gain_imb < 0 ? 1.0 / ratio : 1.0;
}

OpStatus LMS7002M_QEC::SetImbalance(float iq_gain_imb, float phase_imb_deg)
{
    double gainI, gainQ;
    GainImbalanceToGains(iq_gain_imb, gainI, gainQ);
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return mChip->SetIQBalance(mDir, phase_imb_deg * M_PI / 180.0, gainI, gainQ);
}

OpStatus LMS7002M_QEC::SetPhaseCorrection(float phase_imb_deg)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    double phase, gainI, gainQ;
    OpStatus status = mChip->GetIQBalance(mDir, phase, gainI, gainQ);
    if (status != OpStatus::Success)
        return status;
    return mChip->SetIQBalance(mDir, phase_imb_deg * M_PI / 180.0, gainI, gainQ);
}

OpStatus LMS7002M_QEC::SetGainCorrection(float iq_gain_imb)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    double phase, gainI, gainQ;
    OpStatus status = mChip->GetIQBalance(mDir, phase, gainI, gainQ);
    if (status != OpStatus::Success)
        return status;
    GainImbalanceToGains(iq_gain_imb, gainI, gainQ);
    return mChip->SetIQBalance(mDir, phase, gainI, gainQ);
}

lime::Range<float> LMS7002M_QEC::GetGainRange()
{
    // GCORR has 11 bit resolution
    return lime::Range<float>(-3.0f, 3.0f, 0);
}

lime::Range<float> LMS7002M_QEC::GetPhaseRange()
{
    // IQCORR is 12 bit signed, mapped to +-90 degrees.
    return lime::Range<float>(-90.0f, 90.0f, 180.0f / 2048);
}

// LMS7002M_ToneGenerator

LMS7002M_ToneGenerator::LMS7002M_ToneGenerator(LMS7002M* chip, TRXDir dir, uint8_t channel)
    : mChip(chip)
    , mDir(dir)
    , mChannel(channel)
    , mAmplitude(1.0)
    , mFrequencyWeight(1.0 / 8)
{
    assert(chip);
}

OpStatus LMS7002M_ToneGenerator::Enabled(bool enabled)
{
    LMS7002M::ChannelScope scope(mChip, mChannel);
    if (enabled)
    {
        OpStatus status = Apply();
        if (status != OpStatus::Success)
            return status;
    }
    // INSEL: 0 - LML samples, 1 - test signal generator
    return mChip->Modify_SPI_Reg_bits(mDir == TRXDir::Tx ? LMS7002MCSR::INSEL_TXTSP : LMS7002MCSR::INSEL_RXTSP, enabled);
}

OpStatus LMS7002M_ToneGenerator::SetAmplitude(double amplitude)
{
    if (amplitude < -1.0 || amplitude > 1.0)
        return OpStatus::OutOfRange;
    mAmplitude = amplitude;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return Apply();
}

OpStatus LMS7002M_ToneGenerator::SetFrequency(double freq_weight)
{
    const double magnitude = std::abs(freq_weight);
    if (magnitude != 0 && magnitude != 1.0 / 8 && magnitude != 1.0 / 4)
        return OpStatus::InvalidValue;
    mFrequencyWeight = freq_weight;
    LMS7002M::ChannelScope scope(mChip, mChannel);
    return Apply();
}

OpStatus LMS7002M_ToneGenerator::Apply()
{
    const bool isTx = mDir == TRXDir::Tx;
    const bool dcMode = mFrequencyWeight == 0;

    // TSGMODE: 0 - NCO, 1 - DC source
    OpStatus status = mChip->Modify_SPI_Reg_bits(isTx ? LMS7002MCSR::TSGMODE_TXTSP : LMS7002MCSR::TSGMODE_RXTSP, dcMode);
    if (status != OpStatus::Success)
        return status;

    if (dcMode)
    {
        const int16_t value = std::lrint(mAmplitude * 32767);
        return mChip->LoadDC_REG_IQ(mDir, value, value);
    }

    // TSGFCW: 1 - TSP clk/8, 2 - TSP clk/4
    status = mChip->Modify_SPI_Reg_bits(
        isTx ? LMS7002MCSR::TSGFCW_TXTSP : LMS7002MCSR::TSGFCW_RXTSP, std::abs(mFrequencyWeight) == 1.0 / 4 ? 2 : 1);
    if (status != OpStatus::Success)
        return status;
    // TSGSWAPIQ: swapping I and Q mirrors the tone to the negative frequency
    status = mChip->Modify_SPI_Reg_bits(isTx ? LMS7002MCSR::TSGSWAPIQ_TXTSP : LMS7002MCSR::TSGSWAPIQ_RXTSP, mFrequencyWeight < 0);
    if (status != OpStatus::Success)
        return status;
    // TSGFC: 0 - -6 dB, 1 - full scale
    return mChip->Modify_SPI_Reg_bits(isTx ? LMS7002MCSR::TSGFC_TXTSP : LMS7002MCSR::TSGFC_RXTSP, std::abs(mAmplitude) > 0.75);
}

// LMS7002M_RFChannel

LMS7002M_RFChannel::LMS7002M_RFChannel(
    LMS7002M* chip, TRXDir dir, uint8_t index, std::shared_ptr<ILOMixer> loMixer, std::shared_ptr<IOversampler> oversampler)
    : mDir(dir)
    , mIndex(index)
    , mRFPath(std::make_unique<LMS7002M_RFPath>(chip, dir, index))
    , mLPF(std::make_unique<LMS7002M_LPF>(chip, dir, index))
    , mNCO(std::make_unique<LMS7002M_NCO>(chip, dir, index))
    , mDC(std::make_unique<LMS7002M_DC>(chip->mC_impl, dir, index))
    , mQEC(std::make_unique<LMS7002M_QEC>(chip, dir, index))
    , mToneGenerator(std::make_unique<LMS7002M_ToneGenerator>(chip, dir, index))
    , mLOMixer(std::move(loMixer))
    , mOversampler(std::move(oversampler))
{
    for (uint8_t i = 0; i < GFIR_COUNT; ++i)
        mGFIR[i] = std::make_unique<LMS7002M_GFIR>(chip, dir, index, i);

    // Step 0 where the hardware steps are not evenly spaced.
    using Units = IGainStage::Units;
    auto addStage = [&](eGainTypes type, const char* name, Units units, Range<double> range) {
        mGainStages.push_back(std::make_unique<LMS7002M_GainStage>(chip, dir, index, type, name, units, range));
    };
    if (dir == TRXDir::Rx)
    {
        addStage(eGainTypes::LNA, "LNA", Units::dB, Range<double>(0, 30, 0));
        addStage(eGainTypes::LoopbackLNA, "LoopbackLNA", Units::dB, Range<double>(0, 40, 0));
        addStage(eGainTypes::TIA, "TIA", Units::dB, Range<double>(0, 12, 0));
        addStage(eGainTypes::PGA, "PGA", Units::dB, Range<double>(-12, 19, 1));
        // Overall gain, distributed between LNA, TIA and PGA by LMS7002M::SetGenericRxGain_dB()
        addStage(eGainTypes::GENERIC, "Generic", Units::dB, Range<double>(-12, 50, 1));
    }
    else
    {
        // CG_IAMP_TBB register code
        addStage(eGainTypes::IAMP, "IAMP", Units::Raw, Range<double>(1, 63, 1));
        addStage(eGainTypes::PAD, "PAD", Units::dB, Range<double>(0, 52, 0));
        addStage(eGainTypes::LoopbackPAD, "LoopbackPAD", Units::dB, Range<double>(-4.3, 0, 0));
        // Overall gain, set with the PAD like SDRDevice::SetGain() with eGainTypes::GENERIC
        addStage(eGainTypes::GENERIC, "Generic", Units::dB, Range<double>(0, 52, 0));
    }
}

LMS7002M_RFChannel::~LMS7002M_RFChannel() = default;

IGainStage* LMS7002M_RFChannel::GetGainStage(uint8_t index)
{
    if (index >= mGainStages.size())
        return nullptr;
    return mGainStages[index].get();
}

IGFIR* LMS7002M_RFChannel::GetGFIR(uint8_t index)
{
    if (index >= GFIR_COUNT)
        return nullptr;
    return mGFIR[index].get();
}

IDCCorrector* LMS7002M_RFChannel::GetDCCorrector()
{
    return mDC.get();
}

// LMS7002M IRFSOC implementation

static std::size_t DirIndex(TRXDir dir)
{
    return dir == TRXDir::Tx ? 1 : 0;
}

uint8_t LMS7002M::GetChannelCount(TRXDir dir) const
{
    return CHANNEL_COUNT;
}

IRFChannel* LMS7002M::GetChannel(TRXDir dir, uint8_t index)
{
    if (index >= CHANNEL_COUNT)
        return nullptr;
    return mRFChannels[DirIndex(dir)][index].get();
}

uint8_t LMS7002M::GetSensorCount() const
{
    return static_cast<uint8_t>(mSensors.size());
}

ISensor* LMS7002M::GetSensor(uint8_t index)
{
    if (index >= mSensors.size())
        return nullptr;
    return mSensors[index].get();
}

void LMS7002M::CreateModules()
{
    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
    {
        auto loMixer = std::make_shared<LMS7002M_LOMixer>(this, dir);
        auto oversampler = std::make_shared<LMS7002M_Oversampler>(this, dir);
        for (uint8_t ch = 0; ch < CHANNEL_COUNT; ++ch)
            mRFChannels[DirIndex(dir)][ch] = std::make_unique<LMS7002M_RFChannel>(this, dir, ch, loMixer, oversampler);
    }

    // The underlying LMS7002M getters do not report errors, so these reads always succeed.
    using Units = ISensor::Units;
    mSensors.push_back(std::make_unique<LMS7002M_Sensor>("Temperature", Units::Celsius, [this](double& value) {
        value = GetTemperature();
        return OpStatus::Success;
    }));
    mSensors.push_back(std::make_unique<LMS7002M_Sensor>("CGEN PLL locked", Units::Boolean, [this](double& value) {
        value = GetCGENLocked() ? 1.0 : 0.0;
        return OpStatus::Success;
    }));
    for (TRXDir dir : { TRXDir::Rx, TRXDir::Tx })
    {
        const char* name = dir == TRXDir::Rx ? "RxPLL locked" : "TxPLL locked";
        mSensors.push_back(std::make_unique<LMS7002M_Sensor>(name, Units::Boolean, [this, dir](double& value) {
            value = GetSXLocked(dir) ? 1.0 : 0.0;
            return OpStatus::Success;
        }));
    }
}

} // namespace lime
