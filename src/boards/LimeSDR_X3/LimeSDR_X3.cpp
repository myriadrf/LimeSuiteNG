#include "LimeSDR_X3.h"

#include <fcntl.h>
#include <sstream>

#include "limesuiteng/Logger.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "limesuiteng/LMS7002M.h"
#include "FPGA/FPGA_common.h"
#include "FPGA_X3.h"
#include "protocols/LMS64CProtocol.h"
#include "DSP/CFR/CrestFactorReduction.h"
#include "DeviceTreeNode.h"
#include "limesuiteng/SDRDescriptor.h"
#include "CommonFunctions.h"
#include "SlaveSelectShim.h"
#include "utilities/toString.h"
#include "streaming/TRXLooper.h"

#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "chips/LMS7002M/MCU_BD.h"

#include <cmath>

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

namespace lime {
namespace limesdrx3 {

// X3 board specific subdevice ids
static const uint8_t SPI_LMS7002M_1 = 0;
static const uint8_t SPI_LMS7002M_2 = 1;
static const uint8_t SPI_LMS7002M_3 = 2;
static const uint8_t SPI_FPGA = 3;

static CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static CustomParameter cp_temperature = { "Board Temperature"s, 1, 0, 65535, true };

static CustomParameter cp_lms1_tx1dac = { "LMS1 TX1DAC"s, 2, 0, 65535, false };
static CustomParameter cp_lms1_tx2dac = { "LMS1 TX2DAC"s, 3, 0, 65535, false };

static const std::vector<std::pair<uint16_t, uint16_t>> lms1defaultsOverride = {
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x0525 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x218C },
    { 0x008C, 0x267B },
    { 0x00A6, 0x000F },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0108, 0x218C },
    { 0x0109, 0x57C1 },
    { 0x010A, 0x154C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x011A },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x000C },
    { 0x0113, 0x01C1 },
    { 0x0114, 0x01F0 },
    { 0x0115, 0x000D },
    { 0x0118, 0x418C },
    { 0x0119, 0x528C },
    { 0x011A, 0x3001 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0984 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3638 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB },
};

static const std::vector<std::pair<uint16_t, uint16_t>> lms2and3defaultsOverride = {
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0082, 0x803E }, // Power down AFE ADCs/DACs
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x0525 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x218C },
    { 0x008C, 0x267B },
    { 0x00A6, 0x000F },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0108, 0x218C },
    { 0x0109, 0x57C1 },
    { 0x010A, 0xD54C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x011A },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x000C },
    { 0x0113, 0x01C1 },
    { 0x0114, 0x01F0 },
    { 0x0115, 0x000D },
    { 0x0118, 0x418C },
    { 0x0119, 0xD28C },
    { 0x011A, 0x3001 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0984 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3638 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB },
};

} // namespace limesdrx3

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_X3::LMS1_UpdateFPGAInterface(void* userData)
{
    constexpr int chipIndex = 0;
    assert(userData != nullptr);
    LimeSDR_X3* pthis = static_cast<LimeSDR_X3*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(chipIndex), *pthis->mFPGA, chipIndex);
}

/// @brief Constructs a new LimeSDR_X3 object
///
/// @param spiLMS7002M The communications port to the LMS7002M chips.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param trxStreams The communications ports to send and receive sample data.
/// @param control The serial port of the device for retrieving device firmware information.
LimeSDR_X3::LimeSDR_X3(std::shared_ptr<IComms> spiLMS7002M,
    std::shared_ptr<IComms> spiFPGA,
    std::vector<std::shared_ptr<LimePCIe>> trxStreams,
    std::shared_ptr<ISerialPort> control)
    : LMS7002M_SDRDevice()
    , mTRXStreamPorts(trxStreams)
    , mfpgaPort(spiFPGA)
    , mConfigInProgress(false)
{
    mStreamers.resize(3);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes
    SDRDescriptor& desc = mDeviceDescriptor;

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*control, fw);
    LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    mLMS7002Mcomms[0] = std::make_shared<SlaveSelectShim>(spiLMS7002M, limesdrx3::SPI_LMS7002M_1);

    mFPGA = std::make_unique<lime::FPGA_X3>(spiFPGA, mLMS7002Mcomms[0]);
    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, desc);

    desc.spiSlaveIds = {
        { "LMS7002M_1"s, limesdrx3::SPI_LMS7002M_1 },
        { "LMS7002M_2"s, limesdrx3::SPI_LMS7002M_2 },
        { "LMS7002M_3"s, limesdrx3::SPI_LMS7002M_3 },
        { "FPGA"s, limesdrx3::SPI_FPGA },
    };

    const std::unordered_map<std::string, Region> eepromMap = { { "VCTCXO_DAC"s, { 16, 2 } } };

    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);
    desc.memoryDevices[ToString(eMemoryDevice::EEPROM)] = std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM, eepromMap);

    desc.customParameters.push_back(limesdrx3::cp_vctcxo_dac);
    desc.customParameters.push_back(limesdrx3::cp_temperature);

    mEqualizer = std::make_unique<CrestFactorReduction>(std::make_shared<SlaveSelectShim>(spiFPGA, limesdrx3::SPI_FPGA));
    mClockGeneratorCDCM = std::make_unique<CDCM_Dev>(spiFPGA, CDCM2_BASE_ADDR);
    // TODO: read back cdcm values or mClockGeneratorCDCM->Reset(30.72e6, 25e6);

    // LMS#1
    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.name = "LMS 1"s;
        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> lms1 = std::make_unique<LMS7002M>(mLMS7002Mcomms[0]);
        lms1->ModifyRegistersDefaults(limesdrx3::lms1defaultsOverride);
        lms1->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);
        mLMSChips.push_back(std::move(lms1));
    }

    // LMS#2
    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.name = "LMS 2"s;
        soc.pathNames[TRXDir::Rx] = { "None"s, "TDD"s, "FDD"s, "Calibration(LMS3)"s };
        soc.pathNames[TRXDir::Tx] = { "None"s, "TDD"s, "FDD"s };

        desc.rfSOC.push_back(soc);
        mLMS7002Mcomms[1] = std::make_shared<SlaveSelectShim>(spiLMS7002M, limesdrx3::SPI_LMS7002M_2);
        std::unique_ptr<LMS7002M> lms2 = std::make_unique<LMS7002M>(mLMS7002Mcomms[1]);
        lms2->ModifyRegistersDefaults(limesdrx3::lms2and3defaultsOverride);
        mLMSChips.push_back(std::move(lms2));
    }

    // LMS#3
    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.name = "LMS 3"s;
        soc.pathNames[TRXDir::Rx] = { "None"s, "LNAH"s, "Calibration(LMS2)"s };
        soc.pathNames[TRXDir::Tx] = { "None"s, "Band1"s };
        desc.rfSOC.push_back(soc);
        mLMS7002Mcomms[2] = std::make_shared<SlaveSelectShim>(spiLMS7002M, limesdrx3::SPI_LMS7002M_3);
        std::unique_ptr<LMS7002M> lms3 = std::make_unique<LMS7002M>(mLMS7002Mcomms[2]);
        lms3->ModifyRegistersDefaults(limesdrx3::lms2and3defaultsOverride);
        mLMSChips.push_back(std::move(lms3));
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_X3, mFPGA.get());
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>("LMS_1"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>("LMS_2"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(1).get()));
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>("LMS_3"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(2).get()));
    desc.socTree = std::make_shared<DeviceTreeNode>("X3"s, eDeviceTreeNodeClass::SDRDevice, this);
    desc.socTree->children.push_back(fpgaNode);

    desc.socTree->children.push_back(
        std::make_shared<DeviceTreeNode>("CDCM6208"s, eDeviceTreeNodeClass::CDCM6208, mClockGeneratorCDCM.get()));
}

LimeSDR_X3::~LimeSDR_X3()
{
}

// Setup default register values specifically for onboard LMS1 chip
OpStatus LimeSDR_X3::InitLMS1(bool skipTune)
{
    LMS1_PA_Enable(0, false);
    LMS1_PA_Enable(1, false);

    double dacVal = 65535;
    const std::vector<CustomParameterIO> params{ { limesdrx3::cp_lms1_tx1dac.id, dacVal, ""s },
        { limesdrx3::cp_lms1_tx2dac.id, dacVal, ""s } };
    CustomParameterWrite(params);

    OpStatus status;
    auto& lms = mLMSChips.at(0);
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = lms->SetFrequencySX(TRXDir::Tx, lms->GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = lms->SetFrequencySX(TRXDir::Rx, lms->GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    // if (SetRate(10e6,2)!=0)
    //     return -1;
    return OpStatus::Success;
}

static void EnableChannelLMS2(LMS7002M& chip, TRXDir dir, const uint8_t channel, const bool enable)
{
    LMS7002M::ChannelScope scope(&chip, channel);

    const bool isTx = dir == TRXDir::Tx;
    //--- LML ---
    if (channel == 0)
    {
        if (isTx)
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::TXEN_A, enable ? 1 : 0);
        else
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::RXEN_A, enable ? 1 : 0);
    }
    else
    {
        if (isTx)
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::TXEN_B, enable ? 1 : 0);
        else
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::RXEN_B, enable ? 1 : 0);
    }

    //--- ADC/DAC ---
    chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_AFE, 1);
    chip.Modify_SPI_Reg_bits(isTx ? PD_TX_AFE1 : PD_RX_AFE1, 1);
    chip.Modify_SPI_Reg_bits(isTx ? PD_TX_AFE2 : PD_RX_AFE2, 1);

    //int disabledChannels = (chip.Get_SPI_Reg_bits(PD_AFE.address,4,1)&0xF);//check if all channels are disabled
    //chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_AFE,disabledChannels==0xF ? 0 : 1);
    //chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_AFE, disabledChannels==0xF ? 1 : 0);

    //--- digital --- not used for LMS2
    if (isTx)
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_TXTSP, 0);
    }
    else
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_RXTSP, 0);
        // chip.Modify_SPI_Reg_bits(LMS7002MCSR::AGC_MODE_RXTSP, 2); //bypass
        // chip.Modify_SPI_Reg_bits(LMS7002MCSR::AGC_BYP_RXTSP, 1);
        //chip.SPI_write(0x040C, 0x01FF) // bypass all RxTSP
    }

    //--- baseband ---
    if (isTx)
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_TBB, 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_TBB, enable ? 1 : 0);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_LPFIAMP_TBB, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::TSTIN_TBB, 3); // switch to external DAC
    }
    else
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_RBB, 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_RBB, enable ? 1 : 0);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_PGA_RBB, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::OSW_PGA_RBB, 1); // switch external ADC
    }

    //--- frontend ---
    if (isTx)
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_TRF, 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_TRF, enable ? 1 : 0);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_TLOBUF_TRF, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_TXPAD_TRF, enable ? 0 : 1);
    }
    else
    {
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_RFE, 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G_RFE, enable ? 1 : 0);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_MXLOBUF_RFE, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_QGEN_RFE, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_TIA_RFE, enable ? 0 : 1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::PD_LNA_RFE, enable ? 0 : 1);
    }

    //--- synthesizers ---
    if (isTx)
    {
        chip.SetActiveChannel(LMS7002M::Channel::ChSXT);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_SXRSXT, 1);
        //chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G, (disabledChannels&3) == 3?0:1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G, 1);
        if (channel == 1) //enable LO to channel B
        {
            chip.SetActiveChannel(LMS7002M::Channel::ChA);
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_NEXTTX_TRF, enable ? 1 : 0);
        }
    }
    else
    {
        chip.SetActiveChannel(LMS7002M::Channel::ChSXR);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_DIR_SXRSXT, 1);
        //chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G, (disabledChannels&0xC)==0xC?0:1);
        chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_G, 1);
        if (channel == 1) //enable LO to channel B
        {
            chip.SetActiveChannel(LMS7002M::Channel::ChA);
            chip.Modify_SPI_Reg_bits(LMS7002MCSR::EN_NEXTRX_RFE, enable ? 1 : 0);
        }
    }
}

// Setup default register values specifically for onboard LMS2 chip
OpStatus LimeSDR_X3::InitLMS2(bool skipTune)
{
    LMS2_PA_LNA_Enable(0, false, false);
    LMS2_PA_LNA_Enable(1, false, false);

    struct regVal {
        uint16_t adr;
        uint16_t val;
    };

    OpStatus status;
    auto& lms = mLMSChips.at(1);
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = lms->SetFrequencySX(TRXDir::Tx, lms->GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = lms->SetFrequencySX(TRXDir::Rx, lms->GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    // if (SetRate(10e6,2)!=0)
    //     return -1;
    return OpStatus::Success;
}

// TODO: Setup default register values specifically for onboard LMS3 chip
OpStatus LimeSDR_X3::InitLMS3(bool skipTune)
{
    auto& lms = mLMSChips.at(2);
    OpStatus status;
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = lms->SetFrequencySX(TRXDir::Tx, lms->GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = lms->SetFrequencySX(TRXDir::Rx, lms->GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

OpStatus LimeSDR_X3::ConfigureLMS1(const SDRConfig& cfg)
{
    const int socIndex = 0;
    auto& chip = mLMSChips.at(socIndex);
    mConfigInProgress = true;

    // Turn off PAs before configuring chip to avoid unexpectedly strong signal input
    LMS1_PA_Enable(0, false);
    LMS1_PA_Enable(1, false);

    // config validation complete, now do the actual configuration
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS1(skipTune);
    }
    chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0); // enabled DAC is required for FPGA to work

    OpStatus status = LMS7002M_Configure(*chip, cfg);
    mConfigInProgress = false;

    if (status != OpStatus::Success)
        return status;

    // enabled ADC/DAC is required for FPGA to work
    chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
    chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);

    double sampleRate{ 0 };
    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }
    if (rxUsed)
        sampleRate = cfg.channel[0].rx.sampleRate;
    else if (txUsed)
        sampleRate = cfg.channel[0].tx.sampleRate;

    if (sampleRate > 0)
        LMS7002M_SDRDevice::LMS7002M_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);

    for (int c = 0; c < 2; ++c)
    {
        SetLMSPath(TRXDir::Tx, cfg.channel[c].tx, c, socIndex);
        SetLMSPath(TRXDir::Rx, cfg.channel[c].rx, c, socIndex);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    LMS1_UpdateFPGAInterface(this);

    for (int c = 0; c < 2; ++c)
    {
        const ChannelConfig::Direction& tx_channel_cfg = cfg.channel[c].tx;
        // set PA gains
        auto gainIter = tx_channel_cfg.gain.find(eGainTypes::PA);
        if (gainIter != tx_channel_cfg.gain.end())
        {
            int32_t paramId = 2 + c;
            const std::string units = ""s;
            double dac = gainIter->second;
            CustomParameterWrite({ { paramId, dac, units } });
        }

        // Turn on needed PAs
        LMS1_PA_Enable(c, tx_channel_cfg.enabled);
    }

    return OpStatus::Success;
}

OpStatus LimeSDR_X3::ConfigureLMS2(const SDRConfig& cfg)
{
    const int socIndex = 1;
    auto& chip = mLMSChips.at(socIndex);
    mConfigInProgress = true;

    // Turn off PAs before configuring chip to avoid unexpectedly strong signal input
    LMS2_PA_LNA_Enable(0, false, false);
    LMS2_PA_LNA_Enable(1, false, false);

    // config validation complete, now do the actual configuration
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS2(skipTune);
    }
    // chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0); // enabled DAC is required for FPGA to work

    OpStatus status = LMS7002M_Configure(*chip, cfg);
    mConfigInProgress = false;

    if (status != OpStatus::Success)
        return status;

    double sampleRate{ 0 };
    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;

        // LMS2 uses external ADC/DAC
        EnableChannelLMS2(*chip, TRXDir::Rx, i, ch.rx.enabled);
        EnableChannelLMS2(*chip, TRXDir::Tx, i, ch.tx.enabled);
    }
    if (rxUsed)
        sampleRate = cfg.channel[0].rx.sampleRate;
    else if (txUsed)
        sampleRate = cfg.channel[0].tx.sampleRate;

    if (sampleRate > 0)
    {
        CrestFactorReduction::Config eqCfg;
        for (int i = 0; i < 2; ++i)
        {
            eqCfg.bypassRxEqualizer[i] = true;
            eqCfg.bypassTxEqualizer[i] = true;
            eqCfg.cfr[i].bypass = true;
            eqCfg.cfr[i].sleep = true;
            eqCfg.cfr[i].bypassGain = true;
            eqCfg.cfr[i].interpolation = cfg.channel[0].tx.oversample;
            eqCfg.fir[i].sleep = true;
            eqCfg.fir[i].bypass = true;
        }
        mEqualizer->Configure(eqCfg);
        LMS2_SetSampleRate(sampleRate, cfg.channel[0].tx.oversample);
    }

    for (int c = 0; c < 2; ++c)
    {
        auto enumChannel = c == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB;
        chip->SetActiveChannel(enumChannel);
        SetLMSPath(TRXDir::Tx, cfg.channel[c].tx, c, socIndex);
        SetLMSPath(TRXDir::Rx, cfg.channel[c].rx, c, socIndex);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    for (int c = 0; c < 2; ++c)
    {
        // Turn on needed PAs
        LMS2_PA_LNA_Enable(c, cfg.channel[c].tx.enabled, cfg.channel[c].rx.enabled);
    }

    return OpStatus::Success;
}

OpStatus LimeSDR_X3::ConfigureLMS3(const SDRConfig& cfg)
{
    const int socIndex = 2;
    auto& chip = mLMSChips.at(socIndex);
    mConfigInProgress = true;

    // config validation complete, now do the actual configuration
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS3(skipTune);
    }
    // chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0); // enabled DAC is required for FPGA to work

    OpStatus status = LMS7002M_Configure(*chip, cfg);
    mConfigInProgress = false;

    if (status != OpStatus::Success)
        return status;

    // // enabled ADC/DAC is required for FPGA to work
    // chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
    // chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);

    double sampleRate{ 0 };
    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }
    if (rxUsed)
        sampleRate = cfg.channel[0].rx.sampleRate;
    else if (txUsed)
        sampleRate = cfg.channel[0].tx.sampleRate;

    if (sampleRate > 0)
        LMS3_SetSampleRate_ExternalDAC(cfg.channel[0].rx.sampleRate, cfg.channel[1].rx.sampleRate);

    for (int c = 0; c < 2; ++c)
    {
        SetLMSPath(TRXDir::Tx, cfg.channel[c].tx, c, socIndex);
        SetLMSPath(TRXDir::Rx, cfg.channel[c].rx, c, socIndex);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    return OpStatus::Success;
}

OpStatus LimeSDR_X3::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    std::vector<std::string> errors;
    OpStatus status;
    switch (socIndex)
    {
    case 0:
        status = ConfigureLMS1(cfg);
        break;
    case 1:
        status = ConfigureLMS2(cfg);
        break;
    case 2:
        status = ConfigureLMS3(cfg);
        break;
    default:
        return ReportError(OpStatus::InvalidValue, "Invalid chip index");
    }
    return status;
}

void LimeSDR_X3::SetLMSPath(const TRXDir dir, const ChannelConfig::Direction& trx, const int ch, const uint8_t socIndex)
{
    switch (socIndex)
    {
    case 0:
        LMS1SetPath(dir, ch, trx.path);
        break;
    case 1:
        uint8_t path;

        if (trx.enabled)
            path = trx.path;
        else
            path = (dir == TRXDir::Rx) ? static_cast<uint8_t>(ePathLMS2_Rx::NONE) : static_cast<uint8_t>(ePathLMS2_Tx::NONE);

        LMS2SetPath(dir, ch, path);
        break;
    case 2:
        LMS3SetPath(dir, ch, trx.path);
        break;
    default:
        break;
    }
}

OpStatus LimeSDR_X3::Init()
{
    // Clock generator must be the first thing to be configured, otherwise
    // some FPGA registers might not work
    OpStatus status = mClockGeneratorCDCM->Reset(30.72e6, 25e6) == 0 ? OpStatus::Success : OpStatus::Error;
    if (status != OpStatus::Success)
        return ReportError(status, "Failed to reset CDCM clock generator");

    const bool clockLocked = mClockGeneratorCDCM->IsLocked();
    if (!clockLocked)
        return ReportError(status, "CDCM clock PLL not locked");

    struct regVal {
        uint16_t adr;
        uint16_t val;
    };

    const std::vector<regVal> mFPGAInitVals = {
        { 0x00D1, 0x3357 }, // RF Switches
        //{0x00D2, 0x003C} // PA controls
    };

    for (auto i : mFPGAInitVals)
        mFPGA->WriteRegister(i.adr, i.val);

    const bool skipTune = true;
    InitLMS1(skipTune);
    InitLMS2(skipTune);
    InitLMS3(skipTune);
    return OpStatus::Success;
}

OpStatus LimeSDR_X3::Reset()
{
    OpStatus status = OpStatus::Success;
    for (uint32_t i = 0; i < mLMSChips.size(); ++i)
    {
        status = mLMS7002Mcomms[i]->ResetDevice();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

double LimeSDR_X3::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate)
{
    switch (moduleIndex)
    {
    case 1:
        if (trx == TRXDir::Rx)
        {
            double rate = mClockGeneratorCDCM->GetFrequency(CDCM_Y4); // Rx Ch. A
            if (rf_samplerate)
                *rf_samplerate = rate;
            return rate;
        }
        else
        {
            const int oversample = mEqualizer->GetOversample();
            double rate = mClockGeneratorCDCM->GetFrequency(CDCM_Y0Y1) / oversample; // Tx Ch. A&B
            if (rf_samplerate)
                *rf_samplerate = rate * oversample;
            return rate;
        }
    case 2:
        if (trx == TRXDir::Rx) // LMS3 Rx uses external ADC
        {
            double rate = mClockGeneratorCDCM->GetFrequency(CDCM_Y6); // Rx Ch. A
            if (rf_samplerate)
                *rf_samplerate = rate;
            return rate;
        }
        else // LMS3 Tx uses internal ADC
        {
            return LMS7002M_SDRDevice::GetSampleRate(moduleIndex, TRXDir::Tx, channel, rf_samplerate);
        }
    default:
        return LMS7002M_SDRDevice::GetSampleRate(moduleIndex, trx, channel, rf_samplerate);
    }
}

OpStatus LimeSDR_X3::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    if (moduleIndex == 0 && sampleRate > 0)
    {
        return LMS7002M_SetSampleRate(sampleRate, oversample, oversample);
    }
    else if (moduleIndex == 1 && sampleRate > 0)
    {
        CrestFactorReduction::Config eqCfg;
        for (int i = 0; i < 2; ++i)
        {
            eqCfg.bypassRxEqualizer[i] = true;
            eqCfg.bypassTxEqualizer[i] = true;
            eqCfg.cfr[i].bypass = true;
            eqCfg.cfr[i].sleep = true;
            eqCfg.cfr[i].bypassGain = true;
            eqCfg.cfr[i].interpolation = oversample;
            eqCfg.fir[i].sleep = true;
            eqCfg.fir[i].bypass = true;
        }
        mEqualizer->Configure(eqCfg);
        LMS2_SetSampleRate(sampleRate, oversample);
    }
    else if (moduleIndex == 2 && sampleRate > 0)
    {
        LMS3_SetSampleRate_ExternalDAC(sampleRate, sampleRate);
    }
    return OpStatus::Success;
}

double LimeSDR_X3::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_X3::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_X3::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limesdrx3::SPI_LMS7002M_1:
        return mLMS7002Mcomms[0]->SPI(MOSI, MISO, count);
    case limesdrx3::SPI_LMS7002M_2:
        return mLMS7002Mcomms[1]->SPI(MOSI, MISO, count);
    case limesdrx3::SPI_LMS7002M_3:
        return mLMS7002Mcomms[2]->SPI(MOSI, MISO, count);
    case limesdrx3::SPI_FPGA:
        return mfpgaPort->SPI(MOSI, MISO, count);
    default:
        throw std::logic_error("invalid SPI chip select"s);
    }
}

void LimeSDR_X3::LMS1_PA_Enable(uint8_t chan, bool enabled)
{
    uint16_t pa_addr = 0x00D2;
    uint16_t pa_val = mFPGA->ReadRegister(pa_addr);

    const int bitMask = 1 << (5 - chan);
    if (enabled)
        pa_val |= bitMask; // PA on
    else
        pa_val &= ~bitMask; // chan 0 = 5; chan 1 = 4
    mFPGA->WriteRegister(pa_addr, pa_val);
}

void LimeSDR_X3::LMS1SetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    const bool tx = dir == TRXDir::Tx;
    uint16_t sw_addr = 0x00D1;
    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);

    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    if (tx)
    {
        switch (ePathLMS1_Tx(pathId))
        {
        case ePathLMS1_Tx::NONE: // RF switch don't need to change. Still set value to be deterministic.
        case ePathLMS1_Tx::BAND1:
            sw_val |= 1 << (13 - chan); // chan 0 = 13; chan 1 = 12
            break;
        case ePathLMS1_Tx::BAND2:
            sw_val &= ~(1 << (13 - chan));
            break;
        default:
            lime::error("Invalid LMS1 Tx path"s);
            return;
        }
        mFPGA->WriteRegister(sw_addr, sw_val);
        lms->SetBandTRF(pathId);
    }
    else
    {
        switch (ePathLMS1_Rx(pathId))
        {
        case ePathLMS1_Rx::LNAW:
            lime::warning("LNAW has no connection to RF ports"s);
        case ePathLMS1_Rx::NONE:
            break;
        case ePathLMS1_Rx::LNAH:
            sw_val |= 1 << (11 - chan);
            break;
        case ePathLMS1_Rx::LNAL:
            sw_val &= ~(1UL << (11 - chan));
            break;
        }
        mFPGA->WriteRegister(sw_addr, sw_val);
        lms->SetPathRFE(lime::LMS7002M::PathRFE(pathId));
    }
}

void LimeSDR_X3::LMS2_PA_LNA_Enable(uint8_t chan, bool PAenabled, bool LNAenabled)
{
    uint16_t pa_addr = 0x00D2;
    struct RegPA {
        RegPA(uint16_t value)
        {
            lms1PA[0] = value & (1 << 5);
            lms1PA[1] = value & (1 << 4);
            lms2PA[0] = value & (1 << 3);
            lms2PA[1] = value & (1 << 2);
            lms2LNA[0] = !(value & (1 << 1)); // 1=LNA is powered down
            lms2LNA[1] = !(value & (1 << 0));
        }
        uint16_t Value()
        {
            uint16_t value = 0;
            value |= lms1PA[0] << 5;
            value |= lms1PA[1] << 4;
            value |= lms2PA[0] << 3;
            value |= lms2PA[1] << 2;
            value |= (!lms2LNA[0]) << 1;
            value |= (!lms2LNA[1]) << 0;
            return value;
        }
        bool lms1PA[2]{};
        bool lms2PA[2]{};
        bool lms2LNA[2]{};
    };

    RegPA pa(mFPGA->ReadRegister(pa_addr));

    pa.lms2PA[chan] = PAenabled;
    pa.lms2LNA[chan] = LNAenabled;

    mFPGA->WriteRegister(pa_addr, pa.Value());
}

void LimeSDR_X3::LMS2SetPath(TRXDir dir, uint8_t chan, uint8_t path)
{
    const bool tx = dir == TRXDir::Tx;
    uint16_t sw_addr = 0x00D1;
    /*struct RegSW
    {
        RegSW(uint16_t value)
        {
            lms1txBand[0] = value & (1<<13);
            lms1txBand[1] = value & (1<<12);
            lms1rxPath[1] = value & (1<<11);
            lms1rxPath[1] = value & (1<<10);
        }

        bool lms1txBand[2];
        bool lms1rxPath[2];
        bool lms2tx
    }*/

    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);

    uint16_t shift = chan == 0 ? 0 : 2;
    if (path == 0)
    {
    }
    else if (tx && ePathLMS2_Tx(path) == ePathLMS2_Tx::TDD) // TDD_TX
    {
        if (chan == 0)
            sw_val &= ~(1 << 7); // TRX1T to RSFW_TRX1
        else
            sw_val |= 1 << 9; // TRX2T to RSFW_TRX2
        sw_val |= 1 << (6 + shift); // TRX1 or TRX2 to J8 or J10
        sw_val &= ~(1 << (2 + shift)); // RX1C or RX2C to RX1IN or RX2IN (LNA)
        sw_val |= 1 << (3 + shift); // RX1IN or RX2IN to RFSW_TRX1 or RFSW_TRX2
    }
    else if (!tx && ePathLMS2_Rx(path) == ePathLMS2_Rx::TDD) // TDD_RX
    {
        if (chan == 0)
            sw_val |= 1 << 7; // TRX1T to ground
        else
            sw_val &= ~(1 << 9); // TRX2T to ground
        sw_val &= ~(1 << (6 + shift)); // TRX1 or TRX2 to J8 or J10
        sw_val &= ~(1 << (2 + shift)); // RX1C or RX2C to RX1IN or RX2IN (LNA)
        sw_val |= 1 << (3 + shift); // RX1IN or RX2IN to RFSW_TRX1 or RFSW_TRX1
    }
    else if (ePathLMS2_Rx(path) == ePathLMS2_Rx::FDD || ePathLMS2_Tx(path) == ePathLMS2_Tx::FDD) // FDD
    {
        if (chan == 0)
            sw_val &= ~(1 << 7); // TRX1T to RSFW_TRX1
        else
            sw_val |= 1 << 9; // TRX2T to RSFW_TRX2
        sw_val |= 1 << (6 + shift); // TRX1 or TRX2 to J8 or J10
        sw_val &= ~(1 << (2 + shift)); // RX1C or RX2C to RX1IN or RX2IN (LNA)
        sw_val &= ~(1 << (3 + shift)); // RX1IN or RX2In to J9 or  J11
    }
    else if (!tx && ePathLMS2_Rx(path) == ePathLMS2_Rx::CALIBRATION) // Cal
    {
        if (chan == 0)
            sw_val |= 1 << 7; // TRX1T to ground
        else
            sw_val &= ~(1 << 9); // TRX2T to ground
        sw_val |= 1 << (6 + shift); // TRX1 or TRX2 to J8 or J10
        sw_val |= 1 << (2 + shift); // RX1C or RX2C to LMS3 TX1_1 or TX2_1
        sw_val |= 1 << (3 + shift); // RX1IN or RX2IN to RFSW_TRX1 or RFSW_TRX1
    }

    mFPGA->WriteRegister(sw_addr, sw_val);
    auto& lms = mLMSChips.at(1);
    lms->SetBandTRF(1); // LMS2 uses only BAND1
    lms->SetPathRFE(LMS7002M::PathRFE::LNAH); // LMS2 only uses LNAH
}

void LimeSDR_X3::LMS3SetPath(TRXDir dir, uint8_t chan, uint8_t path)
{
    uint16_t sw_addr = 0x00D1;
    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);
    auto& lms = mLMSChips.at(0);

    if (dir == TRXDir::Tx)
        lms->SetBandTRF(path);
    else
    {
        if (path == static_cast<uint8_t>(ePathLMS3_Rx::NONE) || path > 2)
        {
            lms->SetPathRFE(lime::LMS7002M::PathRFE(path));
            return;
        }
        else if (path == static_cast<uint8_t>(ePathLMS3_Rx::LNAH))
            sw_val &= ~(1 << (chan - 4));
        else if (path == 2) // Calibration path
            sw_val |= 1 << (chan - 4);

        mFPGA->WriteRegister(sw_addr, sw_val);
        lms->SetPathRFE(lime::LMS7002M::PathRFE(ePathLMS3_Rx::LNAH));
    }
}

void LimeSDR_X3::LMS2_SetSampleRate(double f_Hz, uint8_t oversample)
{
    assert(mClockGeneratorCDCM);
    double txClock = f_Hz;

    // Oversample is available only to Tx for LMS#2
    oversample = std::min<uint8_t>(oversample, 2);
    if (oversample == 2 || oversample == 0) // 0 is "auto", use max oversample
        txClock *= 2;

    mEqualizer->SetOversample(oversample);

    if (mClockGeneratorCDCM->SetFrequency(CDCM_Y0Y1, txClock, false) != 0) // Tx Ch. A&B
        throw std::runtime_error("Failed to configure CDCM_Y0Y1"s);
    if (mClockGeneratorCDCM->SetFrequency(CDCM_Y4, f_Hz, false) != 0) // Rx Ch. A
        throw std::runtime_error("Failed to configure CDCM_Y4"s);
    if (mClockGeneratorCDCM->SetFrequency(CDCM_Y5, f_Hz, true) != 0) // Rx Ch. B
        throw std::runtime_error("Failed to configure CDCM_Y5"s);

    if (!mClockGeneratorCDCM->IsLocked())
        throw std::runtime_error("CDCM is not locked"s);
}

void LimeSDR_X3::LMS3_SetSampleRate_ExternalDAC(double chA_Hz, double chB_Hz)
{
    if (mClockGeneratorCDCM->SetFrequency(CDCM_Y6, chA_Hz, false) != 0) // Rx Ch. A
        throw std::runtime_error("Failed to configure CDCM_Y4"s);
    if (mClockGeneratorCDCM->SetFrequency(CDCM_Y7, chB_Hz, true) != 0) // Rx Ch. B
        throw std::runtime_error("Failed to configure CDCM_Y5"s);
    if (!mClockGeneratorCDCM->IsLocked())
        throw std::runtime_error("CDCM is not locked"s);
}

OpStatus LimeSDR_X3::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR_X3::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterRead(parameters);
}

OpStatus LimeSDR_X3::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    int progMode;
    LMS64CProtocol::ALTERA_FPGA_GW_WR_targets target = LMS64CProtocol::ALTERA_FPGA_GW_WR_targets::FPGA;

    switch (device)
    {
    case eMemoryDevice::FPGA_RAM:
        progMode = 0;
        break;
    case eMemoryDevice::FPGA_FLASH:
        progMode = 1;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    return mfpgaPort->ProgramWrite(data, length, progMode, static_cast<int>(target), callback);
}

OpStatus LimeSDR_X3::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }

    return mfpgaPort->MemoryWrite(region.address, data, region.size);
}

OpStatus LimeSDR_X3::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }

    return mfpgaPort->MemoryRead(region.address, data, region.size);
}

OpStatus LimeSDR_X3::UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count)
{
    // TODO: return TRXLooper::UploadTxWaveform(mFPGA, txdma, config, moduleIndex, samples, count);
    return OpStatus::Error;
}

std::unique_ptr<lime::RFStream> LimeSDR_X3::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (moduleIndex >= mTRXStreamPorts.size())
    {
        lime::warning("X3 RF%i data stream is not available", 0);
        return std::unique_ptr<RFStream>(nullptr);
    }

    std::shared_ptr<LimePCIe> trxPort{ mTRXStreamPorts.at(moduleIndex) };
    auto rxdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::HostToDevice);

    std::unique_ptr<TRXLooper> streamer =
        std::make_unique<TRXLooper>(rxdma, txdma, mFPGA.get(), mLMSChips.at(moduleIndex).get(), moduleIndex);
    if (!streamer)
        return streamer;

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);
    OpStatus status = streamer->Setup(config);
    if (status != OpStatus::Success)
        return std::unique_ptr<RFStream>(nullptr);
    return streamer;
}

} //namespace lime
