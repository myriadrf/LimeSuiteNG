#include "LimeSDR_XTRX.h"

#include <fcntl.h>
#include <cmath>
#include <chrono>

#include "limesuiteng/Logger.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "FPGA/FPGA_common.h"
#include "FPGA_XTRX.h"
#include "LMS64CProtocol.h"
#include "CommonFunctions.h"
#include "utilities/toString.h"
#include "OEMTesting.h"

#include "DeviceTreeNode.h"
#include "comms/IComms.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "streaming/TRXLooper.h"

#include "limesuiteng/LMS7002M.h"

using namespace std;

namespace lime {
using namespace LMS7002MCSR_Data;

using namespace std::literals::string_literals;

// XTRX board specific devices ids and data
static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter cp_temperature = { "Board Temperature"s, 1, 0, 65535, true };

// Fairwaves XTRX rev.5 requires specific LDO configuration to work properly
static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_fairwaves_xtrx_rev5 = {
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x008B, 0x218C },
    { 0x00A6, 0x000F },
    { 0x011C, 0x8941 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3638 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    // LDOs
    { 0x0092, 0x0D15 },
    { 0x0093, 0x01B1 },
    { 0x00A6, 0x000F },
    // XBUF
    { 0x0085, 0x0019 },
};

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_limesdr_xtrx = {
    { 0x0020, 0xFFFD },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x0081, 0x0001 },
    { 0x0086, 0x4101 },
    { 0x0089, 0x1040 },
    { 0x008B, 0x2198 },
    { 0x009B, 0x8C65 },
    { 0x009E, 0x8C65 },
    { 0x00A0, 0x658C },
    { 0x00A6, 0x000F },
    { 0x0100, 0x7409 },
    { 0x0101, 0x1800 },
    { 0x0103, 0x0A50 },
    { 0x0105, 0x0011 },
    { 0x0108, 0x410C },
    { 0x010A, 0x1FFF },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x009F },
    { 0x010F, 0x3042 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x2106 },
    { 0x0113, 0x01C1 },
    { 0x0114, 0x01B0 },
    { 0x0117, 0x2044 },
    { 0x0119, 0x528C },
    { 0x011A, 0x3001 },
    { 0x011C, 0x8141 },
    { 0x011F, 0x3602 },
    { 0x0120, 0x35FF },
    { 0x0121, 0x37F8 },
    { 0x0122, 0x0654 },
    { 0x0124, 0x001F },
    { 0x0208, 0x017B },
    { 0x0400, 0x8081 },
    { 0x0405, 0x0303 },
    { 0x0406, 0x0303 },
    { 0x0407, 0x0303 },
    { 0x040A, 0x2000 },
    { 0x040C, 0x01FF },
};

static inline void ValidateChannel(uint8_t channel)
{
    if (channel > 2)
        throw std::logic_error("invalid channel index"s);
}

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_XTRX::LMS1_UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeSDR_XTRX* pthis = static_cast<LimeSDR_XTRX*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

/// @brief Constructs a new LimeSDR_XTRX object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
LimeSDR_XTRX::LimeSDR_XTRX(std::shared_ptr<IComms> spiRFsoc,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<LimePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LMS7002M_SDRDevice()
    , lms7002mPort(spiRFsoc)
    , fpgaPort(spiFPGA)
    , mStreamPort(sampleStream)
    , mSerialPort(control)
    , mConfigInProgress(false)
{
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_XTRX);

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw);
    LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    desc.spiSlaveIds = { { "LMS7002M"s, SPI_LMS7002M }, { "FPGA"s, SPI_FPGA } };

    const std::unordered_map<std::string, Region> flashMap = { { "VCTCXO_DAC"s, { 0x01FF0000, 2 } } };
    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] =
        std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH, flashMap);
    desc.customParameters = { cp_vctcxo_dac, cp_temperature };

    mFPGA = std::make_unique<lime::FPGA_XTRX>(spiFPGA, spiRFsoc);
    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, desc);

    const bool isFairwavesRev5 = gw.hardwareVersion == 0;

    // Initial XTRX gateware supported only 32bit DMA, it worked fine on x86 with the PCIe driver
    // limiting the address mask to 32bit, but some systems require at least 35bits,
    // like Raspberry Pi, or other Arm systems. If host requires more than 32bit DMA mask
    // the driver starts using 64bit mask, in that case it's a matter of luck if the system
    // provided DMA addresses will be in 32bit zone, and could work, otherwise, data will be
    // seen as transferred, but the values will be undefined.
    // LimeSDR XTRX gateware added 64bit DMA support in 1.13
    // Fairwaves XTRX Rev 5 gateware added 64bit DMA support in 1.3
    if (gw.version == 1 && ((isFairwavesRev5 && gw.revision < 4) || (!isFairwavesRev5 && gw.revision < 13)))
    {
        lime::warning("Current XTRX gateware does not support 64bit DMA addressing. "
                      "RF data streaming might not work. "
                      "Please update gateware."s);
    }

    const bool isGoldGatewareActive = static_cast<uint16_t>(gw.version) == 0xDEAD && static_cast<uint16_t>(gw.revision) == 0xDEAD;
    if (isGoldGatewareActive)
        lime::warning("XTRX FPGA is running backup 'gold' image, 'user' image might be corrupted, and need reflashing");

    // LimeSDR XTRX gateware revision 1.13 introduced "dual boot" images
    if ((!isFairwavesRev5 && gw.version >= 1 && gw.revision >= 13) || isGoldGatewareActive)
    {
        desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_GOLD_IMAGE)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_GOLD_IMAGE);
        desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_USER_IMAGE)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_USER_IMAGE);
    }

    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Rx]["LNAL"s] = { 0.3e9, 2.2e9 };
        soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 0.7e9, 2.6e9 };
        soc.antennaRange[TRXDir::Rx]["LB1"s] = soc.antennaRange[TRXDir::Rx]["LNAL"s];
        soc.antennaRange[TRXDir::Rx]["LB2"s] = soc.antennaRange[TRXDir::Rx]["LNAW"s];
        soc.antennaRange[TRXDir::Tx]["Band1"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Tx]["Band2"s] = { 0.03e9, 1.9e9 };

        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(spiRFsoc);

        if (isFairwavesRev5)
            chip->ModifyRegistersDefaults(lms7002defaultsOverrides_fairwaves_xtrx_rev5);
        else // LimeSDR XTRX
            chip->ModifyRegistersDefaults(lms7002defaultsOverrides_limesdr_xtrx);
        chip->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }
    {
        mStreamers.reserve(mLMSChips.size());
        if (mStreamPort.get() != nullptr)
        {
            std::shared_ptr<LimePCIe> trxPort{ mStreamPort };
            auto rxdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::DeviceToHost);
            auto txdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::HostToDevice);

            std::unique_ptr<TRXLooper> streamer = std::make_unique<TRXLooper>(rxdma, txdma, mFPGA.get(), mLMSChips.at(0).get(), 0);
            mStreamers.push_back(std::move(streamer));
        }
        else
            lime::warning("XTRX RF data stream is not available");
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_XTRX, mFPGA.get());
    fpgaNode->children.push_back(
        std::make_shared<DeviceTreeNode>("LMS7002M"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    desc.socTree = std::make_shared<DeviceTreeNode>("XTRX"s, eDeviceTreeNodeClass::SDRDevice, this);
    desc.socTree->children.push_back(fpgaNode);
}

static OpStatus InitLMS1(LMS7002M& lms, bool skipTune = false)
{
    OpStatus status;
    status = lms.ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = lms.SetFrequencySX(TRXDir::Tx, lms.GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = lms.SetFrequencySX(TRXDir::Rx, lms.GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    // if (SetRate(10e6,2)!=0)
    //     return -1;
    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    std::vector<std::string> errors;
    bool isValidConfig = LMS7002M_Validate(cfg, errors);

    if (!isValidConfig)
    {
        std::stringstream ss;
        for (const auto& err : errors)
            ss << err << std::endl;
        return ReportError(OpStatus::Error, "LimeSDR_XTRX config: "s + ss.str());
    }

    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }

    try
    {
        mConfigInProgress = true;
        auto& chip = mLMSChips.at(socIndex);
        if (!cfg.skipDefaults)
        {
            const bool skipTune = true;
            InitLMS1(*chip, skipTune);
        }

        LMS7002LOConfigure(*chip, cfg);
        for (int i = 0; i < 2; ++i)
        {
            LMS7002ChannelConfigure(*chip, cfg.channel[i], i);
            LMS7002TestSignalConfigure(*chip, cfg.channel[i], i);
        }

        // enabled ADC/DAC is required for FPGA to work
        chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
        chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);
        chip->SetActiveChannel(LMS7002M::Channel::ChA);

        double sampleRate{ 0 };
        if (rxUsed)
            sampleRate = cfg.channel[0].rx.sampleRate;
        else if (txUsed)
            sampleRate = cfg.channel[0].tx.sampleRate;

        if (sampleRate > 0)
            LMS1_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);

        for (int i = 0; i < 2; ++i)
        {
            const ChannelConfig& ch = cfg.channel[i];
            LMS1SetPath(false, i, ch.rx.path);
            LMS1SetPath(true, i, ch.tx.path);
            LMS7002ChannelCalibration(*chip, ch, i);
        }
        chip->SetActiveChannel(LMS7002M::Channel::ChA);

        // Workaround: Toggle LimeLights transmit port to flush residual value from data interface
        uint16_t txMux = chip->Get_SPI_Reg_bits(LMS7002MCSR::TX_MUX);
        chip->Modify_SPI_Reg_bits(LMS7002MCSR::TX_MUX, 2);
        chip->Modify_SPI_Reg_bits(LMS7002MCSR::TX_MUX, txMux);

        mConfigInProgress = false;
        if (sampleRate > 0)
            return LMS1_UpdateFPGAInterface(this);
    } //try
    catch (std::logic_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_XTRX config: "s + e.what());
    } catch (std::runtime_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_XTRX config: "s + e.what());
    }
    return OpStatus::Success;
}

OpStatus LimeSDR_XTRX::Init()
{
    struct regVal {
        uint16_t adr;
        uint16_t val;
    };

    const std::vector<regVal> mFPGAInitVals = {
        { 0x00D1, 0x3357 }, // RF Switches
    };

    for (auto i : mFPGAInitVals)
        mFPGA->WriteRegister(i.adr, i.val);

    // uint8_t paramId = 2;
    // double dacVal = 65535;
    // CustomParameterWrite(&paramId,&dacVal,1,"");
    // paramId = 3;
    // CustomParameterWrite(&paramId,&dacVal,1,"");

    OpStatus status = LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    if (status != OpStatus::Success)
        return status;

    const bool skipTune = true;
    return InitLMS1(*mLMSChips.at(0), skipTune);
}

OpStatus LimeSDR_XTRX::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return LMS1_SetSampleRate(sampleRate, oversample, oversample);
}

double LimeSDR_XTRX::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    ValidateChannel(channel);
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_XTRX::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    ValidateChannel(channel);
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_XTRX::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case SPI_LMS7002M:
        return lms7002mPort->SPI(MOSI, MISO, count);
    case SPI_FPGA:
        return fpgaPort->SPI(MOSI, MISO, count);
    default:
        throw std::logic_error("invalid SPI chip select"s);
    }
}

OpStatus LimeSDR_XTRX::LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation)
{
    if (f_Hz <= 61.44e6)
    {
        if (rxDecimation == 1)
            rxDecimation = 2;
        if (txInterpolation == 1)
            txInterpolation = 2;
    }
    else // sample rate above 61.44MHz is supported only in SISO mode, and no oversampling
    {
        rxDecimation = 1;
        txInterpolation = 1;
    }

    if (rxDecimation != 0 && txInterpolation / rxDecimation > 4)
        throw std::logic_error(
            strFormat("TxInterpolation(%i)/RxDecimation(%i) should not be more than 4", txInterpolation, rxDecimation));
    uint8_t oversample = rxDecimation;
    const bool bypass = ((oversample == 1 || oversample == 0) && f_Hz > 61.44e6);
    uint8_t hbd_ovr = 7; // decimation ratio is 2^(1+hbd_ovr), HBD_OVR_RXTSP=7 - bypass
    uint8_t hbi_ovr = 7; // interpolation ratio is 2^(1+hbi_ovr), HBI_OVR_TXTSP=7 - bypass
    double cgenFreq = f_Hz * 4; // AI AQ BI BQ
    if (!bypass)
    {
        if (oversample == 0)
        {
            const int n = lime::LMS7002M::CGEN_MAX_FREQ / (cgenFreq);
            oversample = (n >= 32) ? 32 : (n >= 16) ? 16 : (n >= 8) ? 8 : (n >= 4) ? 4 : 2;
        }

        hbd_ovr = 4;
        if (oversample <= 16)
        {
            const int decTbl[] = { 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
            hbd_ovr = decTbl[oversample];
            rxDecimation = pow(2, hbd_ovr + 1);
        }
        cgenFreq *= 2 << hbd_ovr;
        rxDecimation = 2 << hbd_ovr;

        if (txInterpolation == 0)
        {
            //int txMultiplier = std::log2(lime::LMS7002M::CGEN_MAX_FREQ / cgenFreq);
            txInterpolation = rxDecimation; // << txMultiplier;
        }

        if (txInterpolation >= rxDecimation)
        {
            hbi_ovr = hbd_ovr + std::log2(txInterpolation / rxDecimation);
            txInterpolation = pow(2, hbi_ovr + 1);
        }
        else
            return lime::ReportError(
                OpStatus::NotSupported, "Rx decimation(2^%i) > Tx interpolation(2^%i) currently not supported", hbd_ovr, hbi_ovr);
    }
    lime::info("Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: 2^%i, Interp: 2^%i",
        f_Hz / 1e6,
        cgenFreq / 1e6,
        1 + hbd_ovr,
        1 + hbi_ovr);
    auto& mLMSChip = mLMSChips.at(0);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::EN_ADCCLKH_CLKGN, 0);
    if (rxDecimation != 0)
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::CLKH_OV_CLKL_CGEN, 2 - std::log2(txInterpolation / rxDecimation));
    else
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::CLKH_OV_CLKL_CGEN, 2);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 2);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP, hbd_ovr);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP, hbi_ovr);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP, hbd_ovr);
    mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP, hbi_ovr);

    if (f_Hz > 61.44e6)
    {
        // LimeLight & Pad
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::DIQ2_DS, 1);
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR, 1);
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_SISODDR, 1);
        // CDS
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXALML, 0);
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXALML, 1);
        // LDO
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp1, 0);
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp2, 0);
        mLMSChip->Modify_SPI_Reg_bits(LMS7002MCSR::RDIV_DIGIp2, 140);
    }

    return mLMSChip->SetInterfaceFrequency(cgenFreq, hbi_ovr, hbd_ovr);
}

enum // TODO: replace
{
    LMS_PATH_NONE = 0, ///<No active path (RX or TX)
    LMS_PATH_LNAH = 1, ///<RX LNA_H port
    LMS_PATH_LNAL = 2, ///<RX LNA_L port
    LMS_PATH_LNAW = 3, ///<RX LNA_W port
    LMS_PATH_TX1 = 1, ///<TX port 1
    LMS_PATH_TX2 = 2, ///<TX port 2
    LMS_PATH_AUTO = 255, ///<Automatically select port (if supported)
};

void LimeSDR_XTRX::LMS1SetPath(bool tx, uint8_t chan, uint8_t pathId)
{
    uint16_t sw_addr = 0x000A;
    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);
    auto& lms = mLMSChips.at(0);

    // Set active channel for configuring TRF and RFE registers to match the
    // requested channel index.
    const LMS7002M::Channel old_channel = lms->GetActiveChannel();
    lms->SetActiveChannel(chan == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);

    if (tx)
    {
        uint8_t path;
        switch (ePathLMS1_Tx(pathId))
        {
        case ePathLMS1_Tx::NONE:
            path = LMS_PATH_NONE;
            break;
        case ePathLMS1_Tx::BAND1:
            path = LMS_PATH_TX1;
            break;
        case ePathLMS1_Tx::BAND2:
            path = LMS_PATH_TX2;
            break;
        default:
            throw std::logic_error("Invalid LMS1 Tx path"s);
        }
        sw_val &= ~(1 << 4);
        if (path == LMS_PATH_TX1)
            sw_val |= 1 << 4;
        else if (path == LMS_PATH_TX2)
            sw_val &= ~(1 << 4);
        lms->SetBandTRF(path);
    }
    else
    {
        lime::LMS7002M::PathRFE path{ pathId };
        // first configure chip path or loopback
        lms->SetPathRFE(lime::LMS7002M::PathRFE(path));

        // configure rf switches ignoring loopback values
        if (path == LMS7002M::PathRFE::LB1)
            path = LMS7002M::PathRFE::LNAL;
        else if (path == LMS7002M::PathRFE::LB2)
            path = LMS7002M::PathRFE::LNAW;

        sw_val &= ~(0x3 << 2);
        if (path == LMS7002M::PathRFE::LNAW)
            sw_val &= ~(0x3 << 2);
        else if (path == LMS7002M::PathRFE::LNAH)
            sw_val |= 2 << 2;
        else if (path == LMS7002M::PathRFE::LNAL)
            sw_val |= 1 << 2;
    }
    // RF switch controls are toggled for both channels, use channel 0 as the deciding source.
    if (chan == 0)
        mFPGA->WriteRegister(sw_addr, sw_val);

    lms->SetActiveChannel(old_channel);
}

OpStatus LimeSDR_XTRX::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return fpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR_XTRX::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return fpgaPort->CustomParameterRead(parameters);
}

OpStatus LimeSDR_XTRX::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    int progMode;
    LMS64CProtocol::ProgramWriteTarget target = LMS64CProtocol::ProgramWriteTarget::FPGA;

    switch (device)
    {
    case eMemoryDevice::FPGA_RAM:
        progMode = 0;
        break;
    case eMemoryDevice::FPGA_FLASH:
        progMode = 1;
        break;
    case eMemoryDevice::GATEWARE_GOLD_IMAGE:
        progMode = 3;
        break;
    case eMemoryDevice::GATEWARE_USER_IMAGE:
        progMode = 4;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    return fpgaPort->ProgramWrite(data, length, progMode, static_cast<int>(target), callback);
}

OpStatus LimeSDR_XTRX::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this)
        return OpStatus::InvalidValue;
    return fpgaPort->MemoryWrite(region.address, data, region.size);
}

OpStatus LimeSDR_XTRX::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this)
        return OpStatus::InvalidValue;
    return fpgaPort->MemoryRead(region.address, data, region.size);
}

// Calls test start/end callbacks
class TestScope
{
  public:
    TestScope(OEMTestReporter& reporter, OEMTestData& data)
        : reporter(reporter)
        , data(data)
    {
        reporter.OnStart(data, data.title);
    }
    ~TestScope()
    {
        if (data.output.empty())
            data.output = ToString(data.status);

        if (data.status == OpStatus::Success)
            reporter.OnSuccess(data);
        else
            reporter.OnFail(data, data.output);
    }

  private:
    OEMTestReporter& reporter;
    OEMTestData& data;
};

OEMTestData LimeSDR_XTRX::PCIeClockTest(OEMTestReporter& reporter)
{
    OEMTestData test("PCIe Ref Clk");
    OpStatus& status = test.status;
    TestScope testScopeReporter(reporter, test);

    status = mFPGA->OEMTestSetup(FPGA::TestID::HostReferenceClock, chrono::milliseconds(1000));
    if (status != OpStatus::Success)
    {
        test.output = ToString(status);
        return test;
    }

    uint32_t addr[] = { 0x69, 0x69, 0x69 };
    uint32_t vals[3];

    status = fpgaPort->SPI(addr, vals, 3);
    if (status != OpStatus::Success)
    {
        test.output = ToString(status);
        return test;
    }

    // Test passes is values are not identical
    const bool pass = !(vals[0] == vals[1] && vals[1] == vals[2]);
    // reporter.OnStepUpdate(
    //     test, "results: " + std::to_string(vals[0]) + "; " + std::to_string(vals[1]) + "; " + std::to_string(vals[2]));

    if (pass)
    {
        status = OpStatus::Success;
        test.output = "PASS"s;
    }
    else
    {
        status = OpStatus::Error;
        test.output = "FAIL"s;
    }
    return test;
}

OEMTestData LimeSDR_XTRX::GNSSTest(OEMTestReporter& reporter)
{
    OEMTestData test("GNSS");
    TestScope testScopeReporter(reporter, test);

    test.status = mFPGA->OEMTestSetup(FPGA::TestID::GNSS, chrono::milliseconds(1000));
    return test;
}

class CustomParameterStash
{
  public:
    CustomParameterStash(SDRDevice* dev, const std::vector<CustomParameterIO>& args)
        : device(dev)
        , stash(args)
    {
        assert(dev);
        device->CustomParameterRead(stash);
    }
    ~CustomParameterStash() { device->CustomParameterWrite(stash); }

  private:
    SDRDevice* device;
    std::vector<CustomParameterIO> stash;
};

OEMTestData LimeSDR_XTRX::VCTCXOTest(OEMTestReporter& reporter)
{
    OEMTestData test("VCTCXO");
    OpStatus& status = test.status;

    test.measurements.reserve(2);
    test.measurements.push_back({ "VCTCXO(min)", "no data" });
    OEMTestData::Measurement& vctcxoMin = test.measurements.back();
    test.measurements.push_back({ "VCTCXO(max)", "no data" });
    OEMTestData::Measurement& vctcxoMax = test.measurements.back();

    TestScope testScopeReporter(reporter, test);

    std::vector<CustomParameterIO> params{ { cp_vctcxo_dac.id, 0, "" } };

    // Store current value, and restore it on return
    CustomParameterStash vctcxoStash(this, params);

    params[0].value = cp_vctcxo_dac.minValue;
    status = CustomParameterWrite(params);
    if (status != OpStatus::Success)
        return test;

    status = mFPGA->OEMTestSetup(FPGA::TestID::VCTCXO, chrono::milliseconds(1000));
    if (status != OpStatus::Success)
        return test;

    uint32_t addr[] = { 0x72, 0x73 };
    uint32_t vals[2];
    status = mFPGA->ReadRegisters(addr, vals, 2);
    if (status != OpStatus::Success)
        return test;

    const uint32_t vctcxo_min_count = vals[0] + (vals[1] << 16);
    params[0].value = cp_vctcxo_dac.maxValue;
    status = CustomParameterWrite(params);
    if (status != OpStatus::Success)
        return test;

    status = mFPGA->OEMTestSetup(FPGA::TestID::VCTCXO, chrono::milliseconds(1000));
    if (status != OpStatus::Success)
        return test;

    status = mFPGA->ReadRegisters(addr, vals, 2);
    if (status != OpStatus::Success)
        return test;

    const uint32_t vctcxo_max_count = vals[0] + (vals[1] << 16);
    vctcxoMin.value = std::to_string(vctcxo_min_count);
    vctcxoMax.value = std::to_string(vctcxo_max_count);

    const std::string str = "counts min(" + std::to_string(vctcxo_min_count) + ") max(" + std::to_string(vctcxo_max_count) + ")";
    // reporter.OnStepUpdate(test, str);

    const bool fail = (vctcxo_min_count + 25 > vctcxo_max_count) || (vctcxo_min_count + 35 < vctcxo_max_count);
    if (fail)
    {
        status = OpStatus::Error;
        test.output = "FAIL:"s + str;
    }
    else
        status = OpStatus::Success;

    return test;
}

OEMTestData LimeSDR_XTRX::LMS7002_Test(OEMTestReporter& reporter)
{
    OEMTestData test("LMS7002M");
    OpStatus& status = test.status;
    TestScope testScopeReporter(reporter, test);

    auto& lmsControl = mLMSChips.at(0);
    try
    {
        lmsControl->SPI_write(0xA6, 0x0001);
        lmsControl->SPI_write(0x92, 0xFFFF);
        lmsControl->SPI_write(0x93, 0x03FF);
    } catch (...)
    {
        test.output = "SPI failed"s;
        return test;
    }

    status = lmsControl->RegistersTest();
    if (status != OpStatus::Success)
        return test;

    LMS64CProtocol::DeviceReset(*mSerialPort, 0);

    lmsControl->SPI_write(0x0020, 0xFFFD);
    int val = lmsControl->SPI_read(0x20, true, &status);
    if (status != OpStatus::Success)
        return test;

    if (val != 0xFFFD)
    {
        char str[64];
        std::snprintf(str, sizeof(str), "  Reg 0x20: Write value 0xFFFD, Read value 0x%04X", val);
        reporter.OnStepUpdate(test, str);
        test.output = "Register value mismatch"s;
        test.status = OpStatus::Error;
        return test;
    }

    LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    val = lmsControl->SPI_read(0x20, true, &status);
    if (status != OpStatus::Success)
        return test;

    if (val != 0xFFFF)
    {
        char str[64];
        std::snprintf(str, sizeof(str), "  Reg 0x20: value after reset 0x0%4X", val);
        reporter.OnStepUpdate(test, str);
        test.output = "External Reset line test FAILED"s;
        test.status = OpStatus::Error;
        reporter.OnStepUpdate(test, test.output);
        return test;
    }
    return test;
}

static int AntennaNameToIndex(const std::vector<std::string>& antennaNames, const std::string& name)
{
    if (name.empty())
        return -1;

    for (size_t j = 0; j < antennaNames.size(); ++j)
    {
        if (antennaNames[j] == name)
            return j;
    }
    return -1;
}

OEMTestData LimeSDR_XTRX::ConfigureAndMeasure(OEMTestReporter& reporter,
    uint8_t channelIndex,
    double LOFreq,
    const std::string& txAntenna,
    int txGain,
    const std::string& rxAntenna,
    int rxGain,
    double expect_dBFS,
    double allowed_deviation_dBFS)
{
    const std::string title = ("Ch"s + char('A' + channelIndex)) + " "s + std::to_string(uint64_t(LOFreq / 1e6)) + "MHz "s +
                              txAntenna + "->"s + rxAntenna;
    OEMTestData test(title);
    OpStatus& status = test.status;
    TestScope testScopeReporter(reporter, test);

    SDRConfig config;
    config.channel[0].rx.enabled = false;
    config.channel[0].tx.enabled = false;
    config.channel[0].tx.sampleRate = config.channel[0].rx.sampleRate = 61.44e6;
    config.channel[0].tx.testSignal = ChannelConfig::Direction::TestSignal{ true, true }; // Test signal: DC
    config.channel[0].tx.testSignal.dcValue = complex16_t(0x7000, 0x7000);
    config.channel[0].tx.gain[eGainTypes::GENERIC] = txGain;

    const double tx_lo_offset = 5e6;
    config.channel[0].rx.centerFrequency = LOFreq;
    config.channel[0].tx.centerFrequency = LOFreq + tx_lo_offset;
    config.channel[0].rx.path = AntennaNameToIndex(mDeviceDescriptor.rfSOC.at(0).pathNames.at(TRXDir::Rx), rxAntenna);
    config.channel[0].rx.gain[eGainTypes::GENERIC] = rxGain;

    // If RX H is chosen, use TX 1; else use TX 2
    config.channel[0].tx.path = AntennaNameToIndex(mDeviceDescriptor.rfSOC.at(0).pathNames.at(TRXDir::Tx), txAntenna);

    // same config for both channels
    config.channel[1] = config.channel[0];

    // enable only the channel being tested
    config.channel[channelIndex].rx.enabled = true;
    config.channel[channelIndex].tx.enabled = true;

    status = Configure(config, 0);
    if (status != OpStatus::Success)
    {
        test.output = "Config failed"s;
        return test;
    }

    const int rfSOCIndex = 0;
    const int fftSize = 8192;
    std::vector<complex32f_t> samples;
    status = CaptureRxSamples(*this, rfSOCIndex, channelIndex, samples, fftSize);
    if (status != OpStatus::Success)
        return test;

    double signalPeak_dBFS;
    double samplerateWeight;
    CalculateSignalPeak(samples, signalPeak_dBFS, samplerateWeight);

    const double peakFrequency = samplerateWeight * config.channel[0].rx.sampleRate;

    const bool freqIsGood = IsWithinTolerance(peakFrequency, tx_lo_offset, 50e3);
    const bool amplitudeIsGood = IsWithinTolerance(signalPeak_dBFS, expect_dBFS, allowed_deviation_dBFS);

    if (freqIsGood && amplitudeIsGood)
    {
        char str[128];
        snprintf(str, sizeof(str), "PASS: %.2f@%.3fMHz", signalPeak_dBFS, peakFrequency / 1e6);
        test.output = std::string(str);
        test.status = OpStatus::Success;
    }
    else
    {
        char str[128];
        snprintf(str,
            sizeof(str),
            "FAIL: %.2f@%.3fMHz (expected:%.2f±%g@%.3fMHz)",
            signalPeak_dBFS,
            peakFrequency / 1e6,
            expect_dBFS,
            allowed_deviation_dBFS,
            tx_lo_offset / 1e6);
        test.output = std::string(str);
        test.status = OpStatus::Error;
    }
    return test;
}

OEMTestData LimeSDR_XTRX::RFTest(OEMTestReporter& reporter)
{
    OEMTestData test("RF");
    OpStatus& status = test.status;
    TestScope testScopeReporter(reporter, test);
    //reporter.OnStepUpdate(test, "Note: The test should be run with loop connected between RF ports");

    status = Init();
    if (status != OpStatus::Success)
    {
        test.output = "Failed to initialize device"s;
        return test;
    }

    struct Inputs {
        uint8_t channelIndex;
        double centerFrequency;
        std::string txAntenna;
        int txGain;
        std::string rxAntenna;
        int rxGain;
        double expect_dBFS;
    };

    std::vector<Inputs> inputs;
    // TODO: Update gains and measure expected values
    // Rx gain range [-12: 61]
    // Tx gain range [-12; 64]

    // channel A
    inputs.push_back({ 0, 500e6, "Band2", 0, "LNAL", 0, -8 });
    inputs.push_back({ 0, 900e6, "Band2", 0, "LNAL", 0, -8 });
    inputs.push_back({ 0, 1900e6, "Band2", 0, "LNAL", 0, -8 });

    inputs.push_back({ 0, 900e6, "Band2", 0, "LNAW", 0, -8 });
    inputs.push_back({ 0, 1900e6, "Band2", 0, "LNAW", 0, -8 });
    inputs.push_back({ 0, 2700e6, "Band2", 0, "LNAW", 0, -8 });

    inputs.push_back({ 0, 3300e6, "Band1", 0, "LNAH", 0, -8 });

    // channel B
    inputs.push_back({ 1, 500e6, "Band2", 0, "LNAL", 0, -8 });
    inputs.push_back({ 1, 900e6, "Band2", 0, "LNAL", 0, -8 });
    inputs.push_back({ 1, 1900e6, "Band2", 0, "LNAL", 0, -8 });

    inputs.push_back({ 1, 900e6, "Band2", 0, "LNAW", 0, -8 });
    inputs.push_back({ 1, 1900e6, "Band2", 0, "LNAW", 0, -8 });
    inputs.push_back({ 1, 2700e6, "Band2", 0, "LNAW", 0, -8 });

    inputs.push_back({ 1, 3300e6, "Band1", 0, "LNAH", 0, -8 });

    const double dbfs_deviation = 6.0;
    test.status = OpStatus::Success;
    for (const auto& row : inputs)
    {
        OEMTestData data = ConfigureAndMeasure(reporter,
            row.channelIndex,
            row.centerFrequency,
            row.txAntenna,
            row.txGain,
            row.rxAntenna,
            row.rxGain,
            row.expect_dBFS,
            dbfs_deviation);
        test.measurements.push_back({ data.title, data.output });

        if (data.status != OpStatus::Success)
        {
            test.output = "unexpected measurements"s;
            test.status = data.status;
        }
    }
    return test;
}

LimeSDR_XTRX::TestData::TestData()
{
    memset(this, 0, sizeof(TestData));
}

OpStatus LimeSDR_XTRX::OEMTest(OEMTestReporter* reporter)
{
    OEMTestData test("LimeSDR-XTRX OEM Test");
    TestScope testScopeReporter(*reporter, test);
    test.status = OpStatus::Success;

    std::vector<OEMTestData> tests;
    tests.push_back(PCIeClockTest(*reporter));
    tests.push_back(VCTCXOTest(*reporter));
    tests.push_back(GNSSTest(*reporter));
    tests.push_back(LMS7002_Test(*reporter));
    tests.push_back(RFTest(*reporter));

    for (const auto& result : tests)
    {
        reporter->ReportColumn(result.title, result.output);
        for (const auto& data : result.measurements)
            reporter->ReportColumn(data.title, data.value);

        if (result.status != OpStatus::Success)
        {
            test.status = OpStatus::Error;
            test.output = "some tests have failed"s;
        }
    }
    return test.status;
}

OpStatus LimeSDR_XTRX::WriteSerialNumber(uint64_t serialNumber)
{
    std::vector<uint8_t> bytes(sizeof(serialNumber));
    for (size_t i = 0; i < sizeof(serialNumber); ++i)
        bytes[i] = serialNumber >> (8 * i);
    OpStatus status = LMS64CProtocol::WriteSerialNumber(*mSerialPort, bytes);

    if (status == OpStatus::Success)
        mDeviceDescriptor.serialNumber = serialNumber;
    return status;
}

OpStatus LimeSDR_XTRX::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;
    LMS1SetPath(trx == TRXDir::Tx, channel, path);
    return OpStatus::Success;
}

} //namespace lime
