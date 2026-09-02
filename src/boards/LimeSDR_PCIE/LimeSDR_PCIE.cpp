#include "LimeSDR_PCIE.h"

#include "limesuiteng/Logger.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/ToString.h"

#include <cmath>
#include <unistd.h>
#include <fcntl.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "chips/ADF4002/ADF4002.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "comms/PCIe/xillybus/Xillybus.h"
#include "comms/PCIe/xillybus/XillybusDMAEmulation.h"

#include "protocols/LMS64C/LMS64C_ADF4002_SPI.h"

#include "comms/SPI/ISPI.h"
#include "protocols/LMS64C/CSR.h"
#include "FPGA/FPGA_common.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"

#include "CommonFunctions.h"
#include "DeviceTreeNode.h"
#include "OEMTesting.h"

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

namespace lime {

namespace limesdrpcie {
// XTRX board specific devices ids and data
static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter cp_temperature = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_LimeSDR_PCIE = {
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x0081, 0x0001 },
    { 0x009B, 0x8C65 },
    { 0x009E, 0x8C65 },
    { 0x00A0, 0x658C },
    { 0x00A6, 0x000F },
    { 0x0100, 0x7408 },
    { 0x0101, 0x1800 }, // F_TXPAD_TRF=0
    { 0x0103, 0x0A50 },
    { 0x0108, 0x410C },
    { 0x010F, 0x3042 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x2106 },
    { 0x0114, 0x01B0 },
    { 0x0119, 0x528C },
    { 0x011C, 0x8141 },
    { 0x011F, 0x3602 }, // SX_DITHER_EN=1
    { 0x0120, 0x29DC },
    { 0x0121, 0x37F8 },
    { 0x0122, 0x0FFF },
    { 0x0123, 0x200F },
    { 0x0124, 0x001F },
    // { 0x0208, 0x017B },
    // { 0x040C, 0x01FF },
};

} // namespace limesdrpcie

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_PCIE::LMS1_UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeSDR_PCIE* pthis = static_cast<LimeSDR_PCIE*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

/// @brief Constructs a new LimeSDR_PCIE object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
LimeSDR_PCIE::LimeSDR_PCIE(std::shared_ptr<ISPI> spiRFsoc,
    std::shared_ptr<ISPI> spiFPGA,
    std::shared_ptr<Xillybus> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LMS7002M_SDRDevice()
    , lms7002mPort(spiRFsoc)
    , fpgaPort(spiFPGA)
    , mStreamPort(sampleStream)
    , mSerialPort(control)
    , mADF(std::make_unique<ADF4002>())
    , mConfigInProgress(false)
    , mSubDeviceIndex(0)
{
    mStreamers.resize(1);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_PCIE);

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw, 0);
    LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    desc.spiSlaveIds = { { "LMS7002M"s, limesdrpcie::SPI_LMS7002M }, { "FPGA"s, limesdrpcie::SPI_FPGA } };

    // const std::unordered_map<std::string, Region> flashMap = { { "VCTCXO_DAC"s, { 0x01FF0000, 2 } } };
    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);

    desc.customParameters = { limesdrpcie::cp_vctcxo_dac, limesdrpcie::cp_temperature };

    mFPGA = std::make_unique<lime::FPGA>(spiFPGA, spiRFsoc);

    void SetFeatures(const GatewareFeatures& flags);
    GatewareFeatures gw_config;
    gw_config.databusWidth = 4096; // Tx must provide the entire 4096B packet
    gw_config.hasConfigurableStreamPacketSize = false; // Tx must provide the entire 4096B packet
    mFPGA->SetFeatures(gw_config);

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, desc);

    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Rx]["LNAL"s] = { 0.3e9, 2.2e9 };
        soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 0.7e9, 2.6e9 };
        // TODO: separate loopbacks from antennas
        // soc.antennaRange[TRXDir::Rx]["LB1"s] = soc.antennaRange[TRXDir::Rx]["LNAL"s];
        // soc.antennaRange[TRXDir::Rx]["LB2"s] = soc.antennaRange[TRXDir::Rx]["LNAW"s];
        soc.antennaRange[TRXDir::Tx]["Band1"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Tx]["Band2"s] = { 0.03e9, 1.9e9 };

        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(spiRFsoc);

        chip->ModifyRegistersDefaults(limesdrpcie::lms7002defaultsOverrides_LimeSDR_PCIE);
        chip->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>(mFPGA.get(), "FPGA_XTRX"s, "FPGA"s);
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>(mLMSChips.at(0).get(), "LMS7002M"s));
    desc.socTree = std::make_shared<DeviceTreeNode>(this, "SDRDevice"s, "LimeSDR-PCIE"s);
    desc.socTree->children.push_back(fpgaNode);

    auto ADFComms = std::make_shared<LMS64C_ADF4002_SPI>(mSerialPort, 0);
    mADF->Initialize(ADFComms, 30.72e6);
    desc.socTree->children.push_back(std::make_shared<DeviceTreeNode>(mADF.get(), "ADF4002"s));
}

void LimeSDR_PCIE::SetSubDeviceIndex(uint32_t index)
{
    mSubDeviceIndex = index;
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

OpStatus LimeSDR_PCIE::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    auto& chip = mLMSChips.at(0);

    mConfigInProgress = true;
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        OpStatus status = InitLMS1(*chip, skipTune);
        if (status != OpStatus::Success)
        {
            mConfigInProgress = false;
            return status;
        }
    }

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
    }
    if (rxUsed)
        sampleRate = cfg.channel[0].rx.sampleRate;
    else if (txUsed)
        sampleRate = cfg.channel[0].tx.sampleRate;

    if (sampleRate > 0)
    {
        status = LMS1_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);
        if (status != OpStatus::Success)
            return status;
    }

    for (int c = 0; c < 2; ++c)
    {
        LMSSetPath(TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMSSetPath(TRXDir::Rx, c, cfg.channel[c].rx.path);
        SetNCOFrequency(0, TRXDir::Tx, c, 0, cfg.channel[c].tx.NCOoffset, 0);
        SetNCOFrequency(0, TRXDir::Rx, c, 0, cfg.channel[c].rx.NCOoffset, 0);
        status = LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
        if (status != OpStatus::Success)
            return status;
    }

    if (sampleRate > 0)
        return LMS1_UpdateFPGAInterface(this);

    return OpStatus::Success;
}

OpStatus LimeSDR_PCIE::Init()
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

    // Stop streaming just in case it was left enabled by crashed application.
    mFPGA->StopStreaming();

    // uint8_t paramId = 2;
    // double dacVal = 65535;
    // CustomParameterWrite(&paramId,&dacVal,1,"");
    // paramId = 3;
    // CustomParameterWrite(&paramId,&dacVal,1,"");

    OpStatus status = LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    // XTRX on X8 board don't have Reset command, returns Unknown
    if (status != OpStatus::Success && status != OpStatus::NotImplemented)
        return status;

    const bool skipTune = true;
    return InitLMS1(*mLMSChips.at(0), skipTune);
}

OpStatus LimeSDR_PCIE::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return LMS1_SetSampleRate(sampleRate, oversample, oversample);
}

double LimeSDR_PCIE::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_PCIE::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_PCIE::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limesdrpcie::SPI_LMS7002M:
        return lms7002mPort->Transact(MOSI, MISO, count);
    case limesdrpcie::SPI_FPGA:
        return fpgaPort->Transact(MOSI, MISO, count);
    default:
        throw std::logic_error("invalid SPI chip select"s);
    }
}

OpStatus LimeSDR_PCIE::LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation)
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

    if (f_Hz > 61.44e6)
    {
        auto& lms = mLMSChips.at(0);
        // LimeLight & Pad
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::DIQ2_DS, 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR, 1);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::LML2_SISODDR, 1);
        // CDS
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::CDSN_RXALML, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::CDS_RXALML, 1);
        // LDO
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp1, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::PD_LDO_DIGIp2, 0);
        lms->Modify_SPI_Reg_bits(LMS7002MCSR::RDIV_DIGIp2, 140);
    }

    return LMS7002M_SDRDevice::LMS7002M_SetSampleRate(f_Hz, rxDecimation, txInterpolation);
}

void LimeSDR_PCIE::LMSSetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    uint16_t sw_addr = 0x000A;
    uint16_t sw_val = mFPGA->ReadRegister(sw_addr);

    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    if (dir == TRXDir::Tx)
    {
        switch (ePathLMS1_Tx(pathId))
        {
        case ePathLMS1_Tx::NONE: // RF switch don't need to change. Still set value to be deterministic.
        case ePathLMS1_Tx::BAND1:
            sw_val |= 1 << 4;
            break;
        case ePathLMS1_Tx::BAND2:
            sw_val &= ~(1 << 4);
            break;
        default:
            lime::error("Invalid Tx RF path"s);
        }
        lms->SetBandTRF(pathId);
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
    // TODO: if MIMO use channel 0 as deciding factor, otherwise use active channel
    // RF switch controls are toggled for both channels, use channel 0 as the deciding source.
    if (chan == 0)
        mFPGA->WriteRegister(sw_addr, sw_val);
}

OpStatus LimeSDR_PCIE::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterWrite(*mSerialPort, parameters, mSubDeviceIndex);
}

OpStatus LimeSDR_PCIE::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterRead(*mSerialPort, parameters, mSubDeviceIndex);
}

OpStatus LimeSDR_PCIE::UploadMemory(
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
    case eMemoryDevice::GATEWARE_GOLD_IMAGE:
        progMode = 3;
        break;
    case eMemoryDevice::GATEWARE_USER_IMAGE:
        progMode = 4;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    return LMS64CProtocol::FirmwareWrite(*mSerialPort, data, length, progMode, target, callback, mSubDeviceIndex);
}

OpStatus LimeSDR_PCIE::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryWrite(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, mSubDeviceIndex);
}

OpStatus LimeSDR_PCIE::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryRead(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, mSubDeviceIndex);
}

OpStatus LimeSDR_PCIE::WriteSerialNumber(uint64_t serialNumber)
{
    std::vector<uint8_t> bytes(sizeof(serialNumber));
    for (size_t i = 0; i < sizeof(serialNumber); ++i)
        bytes[i] = serialNumber >> (8 * i);
    OpStatus status = LMS64CProtocol::WriteSerialNumber(*mSerialPort, bytes);

    if (status == OpStatus::Success)
        mDeviceDescriptor.serialNumber = serialNumber;
    return status;
}

OpStatus LimeSDR_PCIE::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;
    LMSSetPath(trx, channel, path);
    return OpStatus::Success;
}

std::unique_ptr<lime::RFStream> LimeSDR_PCIE::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (mStreamPort.get() == nullptr)
    {
        lime::warning("LimeSDR_PCIE RF data stream is not available");
        return std::unique_ptr<RFStream>(nullptr);
    }

    auto rxdma = std::make_shared<XillybusDMAEmulation>(mStreamPort, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<XillybusDMAEmulation>(mStreamPort, DataTransferDirection::HostToDevice);

    std::unique_ptr<TRXLooper> streamer = std::make_unique<TRXLooper>(rxdma, txdma, mFPGA.get(), mLMSChips.at(0).get(), 0);
    if (!streamer)
        return streamer;

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);
    OpStatus status = streamer->Setup(config);
    if (status != OpStatus::Success)
        return std::unique_ptr<RFStream>(nullptr);
    return streamer;
}

ICSR* LimeSDR_PCIE::getICSR()
{
    return new LMS64C_CSR(mSerialPort);
}

} //namespace lime
