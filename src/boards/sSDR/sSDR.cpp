#include "sSDR.h"

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

#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "chips/LMS8001/LMS8001.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "comms/SPI/ISPI.h"
#include "FPGA/FPGA_common.h"
#include "boards/LimeSDR_XTRX/FPGA_XTRX.h"
#include "protocols/LMS64CProtocol.h"
#include "protocols/LMS64C/SPI.h"
#include "streaming/TRXLooper.h"

#include "CommonFunctions.h"
#include "DeviceTreeNode.h"

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

namespace lime {

namespace ssdr {
static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;
static const uint8_t SPI_LMS8001 = 2;

static const CustomParameter cp_temperature = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_sSDR = {
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

} // namespace ssdr

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus sSDR::UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    sSDR* pthis = static_cast<sSDR*>(userData);
    // don't care about cgen changes while doing Config(), to avoid unnecessary fpga updates
    if (pthis->mConfigInProgress)
        return OpStatus::Success;
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

/// @brief Constructs a new sSDR object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
sSDR::sSDR(std::shared_ptr<ISPI> spiRFsoc,
    std::shared_ptr<ISPI> spiFPGA,
    std::shared_ptr<LimePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LMS7002M_SDRDevice()
    , lms7002mPort(spiRFsoc)
    , fpgaPort(spiFPGA)
    , mStreamPort(sampleStream)
    , mSerialPort(control)
    , mConfigInProgress(false)
    , mSubDeviceIndex(0)
{
    mStreamers.resize(1);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_EXTERNAL_SSDR);

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw, 0);
    LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    desc.spiSlaveIds = { { "LMS7002M"s, ssdr::SPI_LMS7002M }, { "FPGA"s, ssdr::SPI_FPGA }, { "LMS8001"s, ssdr::SPI_LMS8001 } };

    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);

    desc.customParameters = { ssdr::cp_temperature };

    mFPGA = std::make_unique<lime::FPGA_XTRX>(spiFPGA, spiRFsoc);
    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, desc);

    const bool isGoldGatewareActive = static_cast<uint16_t>(gw.version) == 0xDEAD && static_cast<uint16_t>(gw.revision) == 0xDEAD;
    if (isGoldGatewareActive)
        lime::warning("sSDR FPGA is running backup 'gold' image, 'user' image might be corrupted, and need reflashing");

    desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_GOLD_IMAGE)] =
        std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_GOLD_IMAGE);
    desc.memoryDevices[ToString(eMemoryDevice::GATEWARE_USER_IMAGE)] =
        std::make_shared<DataStorage>(this, eMemoryDevice::GATEWARE_USER_IMAGE);

    {
        RFSOCDescriptor soc = GetDefaultLMS7002MDescriptor();
        soc.antennaRange[TRXDir::Rx]["LNAH"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Rx]["LNAL"s] = { 0.3e9, 2.2e9 };
        soc.antennaRange[TRXDir::Rx]["LNAW"s] = { 0.7e9, 2.6e9 };

        soc.antennaRange[TRXDir::Tx]["Band1"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Tx]["Band2"s] = { 0.03e9, 1.9e9 };

        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(spiRFsoc);
        chip->ModifyRegistersDefaults(ssdr::lms7002defaultsOverrides_sSDR);

        chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }
    {
        const int spi_bus_index = 0;
        const int chipSelect = 1;
        lms8spi = std::make_shared<LMS64C_SPI>(mSerialPort,
            LMS64CProtocol::Command::PERIPHSPI_TRNSF,
            LMS64CProtocol::Command::PERIPHSPI_TRNSF,
            spi_bus_index,
            chipSelect);
        rfsoc_lms8001 = std::make_unique<LMS8001>(lms8spi);
    }

    auto fpgaNode = std::make_shared<DeviceTreeNode>(mFPGA.get(), "FPGA_XTRX"s, "FPGA"s);
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>(mLMSChips.at(0).get(), "LMS7002M"s));
    fpgaNode->children.push_back(std::make_shared<DeviceTreeNode>(rfsoc_lms8001.get(), "LMS8001"s));
    desc.socTree = std::make_shared<DeviceTreeNode>(this, "SDRDevice"s, desc.name);
    desc.socTree->children.push_back(fpgaNode);
}

static OpStatus InitLMS7002M(LMS7002M& rfsoc, bool skipTune = false)
{
    OpStatus status;
    status = rfsoc.ResetChip();
    if (status != OpStatus::Success)
        return status;

    if (skipTune)
        return OpStatus::Success;

    status = rfsoc.SetFrequencySX(TRXDir::Tx, rfsoc.GetFrequencySX(TRXDir::Tx));
    if (status != OpStatus::Success)
        return status;

    status = rfsoc.SetFrequencySX(TRXDir::Rx, rfsoc.GetFrequencySX(TRXDir::Rx));
    if (status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

OpStatus sSDR::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    auto& chip = mLMSChips.at(0);

    mConfigInProgress = true;
    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS7002M(*chip, skipTune);
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
        LMS7002M_SetSampleRate(sampleRate, cfg.channel[0].rx.oversample, cfg.channel[0].tx.oversample);

    for (int c = 0; c < 2; ++c)
    {
        LMSSetPath(TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMSSetPath(TRXDir::Rx, c, cfg.channel[c].rx.path);
        SetNCOFrequency(0, TRXDir::Tx, c, 0, cfg.channel[c].tx.NCOoffset, 0);
        SetNCOFrequency(0, TRXDir::Rx, c, 0, cfg.channel[c].rx.NCOoffset, 0);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    if (sampleRate > 0)
        return UpdateFPGAInterface(this);

    return OpStatus::Success;
}

OpStatus sSDR::Init()
{
    struct regVal {
        uint16_t adr;
        uint16_t val;
    };

    const std::vector<regVal> mFPGAInitVals = {
        { 0x000A, 0x0000 }, // RF Switches
    };

    for (auto i : mFPGAInitVals)
        mFPGA->WriteRegister(i.adr, i.val);

    // Stop streaming just in case it was left enabled by crashed application.
    mFPGA->StopStreaming();

    OpStatus status = LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    if (status != OpStatus::Success && status != OpStatus::NotImplemented)
        return status;

    const bool skipTune = true;
    return InitLMS7002M(*mLMSChips.at(0), skipTune);
}

OpStatus sSDR::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return LMS7002M_SetSampleRate(sampleRate, oversample, oversample);
}

double sSDR::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus sSDR::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus sSDR::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case ssdr::SPI_LMS7002M:
        return lms7002mPort->Transact(MOSI, MISO, count);
    case ssdr::SPI_FPGA:
        return fpgaPort->Transact(MOSI, MISO, count);
    case ssdr::SPI_LMS8001:
        return lms8spi->Transact(MOSI, MISO, count);
    default:
        throw std::logic_error("invalid SPI chip select"s);
    }
}

OpStatus sSDR::LMS7002M_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation)
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

void sSDR::LMSSetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    // uint16_t sw_addr = 0x000A;
    // uint16_t sw_val = mFPGA->ReadRegister(sw_addr);

    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    if (dir == TRXDir::Tx)
    {
        // TODO: Rx HF/LF switch
        lms->SetBandTRF(pathId);
    }
    else
    {
        // TODO: Rx HF/LF switch

        lime::LMS7002M::PathRFE path{ pathId };
        // first configure chip path or loopback
        lms->SetPathRFE(lime::LMS7002M::PathRFE(path));
    }
    // TODO: if MIMO use channel 0 as deciding factor, otherwise use active channel
    // RF switch controls are toggled for both channels, use channel 0 as the deciding source.
    // if (chan == 0)
    //     mFPGA->WriteRegister(sw_addr, sw_val);
}

OpStatus sSDR::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterWrite(*mSerialPort, parameters, mSubDeviceIndex);
}

OpStatus sSDR::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterRead(*mSerialPort, parameters, mSubDeviceIndex);
}

OpStatus sSDR::UploadMemory(
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

OpStatus sSDR::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryWrite(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, mSubDeviceIndex);
}

OpStatus sSDR::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryRead(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, mSubDeviceIndex);
}

OpStatus sSDR::WriteSerialNumber(uint64_t serialNumber)
{
    std::vector<uint8_t> bytes(sizeof(serialNumber));
    for (size_t i = 0; i < sizeof(serialNumber); ++i)
        bytes[i] = serialNumber >> (8 * i);
    OpStatus status = LMS64CProtocol::WriteSerialNumber(*mSerialPort, bytes);

    if (status == OpStatus::Success)
        mDeviceDescriptor.serialNumber = serialNumber;
    return status;
}

OpStatus sSDR::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;
    LMSSetPath(trx, channel, path);
    return OpStatus::Success;
}

std::unique_ptr<lime::RFStream> sSDR::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (mStreamPort.get() == nullptr)
    {
        lime::warning("sSDR RF data stream is not available");
        return std::unique_ptr<RFStream>(nullptr);
    }

    std::shared_ptr<LimePCIe> trxPort{ mStreamPort };
    auto rxdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<LimePCIeDMA>(trxPort, DataTransferDirection::HostToDevice);

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

} //namespace lime
