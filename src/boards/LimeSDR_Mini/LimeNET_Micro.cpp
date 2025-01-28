#include "LimeNET_Micro.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

#include "chips/Si5351C/Si5351C.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "comms/IComms.h"
#include "comms/ISerialPort.h"
#include "comms/USB/FT601/FT601.h"
#include "comms/USB/IUSB.h"
#include "comms/USB/USBDMAEmulation.h"
#include "comms/SPI_utilities.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"

#include "FPGA_Mini.h"
#include "DeviceTreeNode.h"

#include <cassert>
#include <memory>
#include <set>
#include <stdexcept>
#include <cmath>

using namespace lime::LMS64CProtocol;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

// LimeNET Micro functionally is very similar to LimeSDR-Mini, could technically reuse it's code.

namespace lime {
namespace limenetmicro {

static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static const CustomParameter CP_VCTCXO_DAC = { "VCTCXO DAC (runtime)"s, 0, 0, 255, false };
static const CustomParameter CP_TEMPERATURE = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides = { //
    { 0x0022, 0x0FFF },
    { 0x0023, 0x5550 },
    { 0x002B, 0x0038 },
    { 0x002C, 0x0000 },
    { 0x002D, 0x0641 },
    { 0x0086, 0x4101 },
    { 0x0087, 0x5555 },
    { 0x0088, 0x03F0 },
    { 0x0089, 0x1078 },
    { 0x008B, 0x2100 },
    { 0x008C, 0x267B },
    { 0x00A1, 0x656A },
    { 0x00A6, 0x0009 },
    { 0x00A7, 0x8A8A },
    { 0x00A9, 0x8000 },
    { 0x00AC, 0x2000 },
    { 0x0105, 0x0011 },
    { 0x0108, 0x218C },
    { 0x0109, 0x6100 },
    { 0x010A, 0x1F4C },
    { 0x010B, 0x0001 },
    { 0x010C, 0x8865 },
    { 0x010E, 0x0000 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x942E },
    { 0x0113, 0x03C2 },
    { 0x0114, 0x00D0 },
    { 0x0117, 0x1230 },
    { 0x0119, 0x18D2 },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0740 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x3650 },
    { 0x0123, 0x000F },
    { 0x0200, 0x00E1 },
    { 0x0208, 0x017B },
    { 0x020B, 0x4000 },
    { 0x020C, 0x8000 },
    { 0x0400, 0x8081 },
    { 0x0404, 0x0006 },
    { 0x040B, 0x1020 },
    { 0x040C, 0x00FB }
};

} // namespace limenetmicro

/// @brief Constructs a new LimeNET_Micro object
/// @param spiLMS The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param streamPort The communications port to send and receive sample data.
/// @param commsPort The communications port for direct communications with the device.
LimeNET_Micro::LimeNET_Micro(std::shared_ptr<IComms> spiLMS,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<IUSB> streamPort,
    std::shared_ptr<ISerialPort> commsPort)
    : mStreamPort(streamPort)
    , mSerialPort(commsPort)
    , mlms7002mPort(spiLMS)
    , mfpgaPort(spiFPGA)
{
    mStreamers.resize(1);
    SDRDescriptor& descriptor = mDeviceDescriptor;

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw);
    LMS64CProtocol::FirmwareToDescriptor(fw, descriptor);

    mFPGA = std::make_unique<FPGA_Mini>(spiFPGA, spiLMS);
    double refClk = 30.72e6;

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, descriptor);

    descriptor.customParameters.push_back(limenetmicro::CP_VCTCXO_DAC);
    descriptor.customParameters.push_back(limenetmicro::CP_TEMPERATURE);

    {
        RFSOCDescriptor soc{ GetDefaultLMS7002MDescriptor() };
        // override specific capabilities
        soc.channelCount = 1;

        soc.pathNames[TRXDir::Rx] = { "NONE"s, "LNAH"s, "LNAL"s, "LNAW_NC"s }; // LNAW is not connected
        soc.samplingRateRange = { 100e3, 30.72e6, 0 };
        soc.frequencyRange = { 10e6, 3.5e9, 0 };
        descriptor.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(mlms7002mPort);
        chip->ModifyRegistersDefaults(limenetmicro::lms7002defaultsOverrides);

        chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    descriptor.spiSlaveIds = { { "LMS7002M"s, limenetmicro::SPI_LMS7002M }, { "FPGA"s, limenetmicro::SPI_FPGA } };

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_MINI, mFPGA.get());
    fpgaNode->children.push_back(
        std::make_shared<DeviceTreeNode>("LMS7002"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    descriptor.socTree = std::make_shared<DeviceTreeNode>("LimeNET-Micro"s, eDeviceTreeNodeClass::SDRDevice, this);
    descriptor.socTree->children.push_back(fpgaNode);
}

LimeNET_Micro::~LimeNET_Micro()
{
}

OpStatus LimeNET_Micro::Configure(const SDRConfig& cfg, uint8_t moduleIndex = 0)
{
    auto& chip = mLMSChips.at(0);

    mConfigInProgress = true;
    if (!cfg.skipDefaults)
    {
        Init();
    }
    // enabled ADC/DAC is required for FPGA to work
    chip->Modify_SPI_Reg_bits(PD_RX_AFE1, 0);
    chip->Modify_SPI_Reg_bits(PD_TX_AFE1, 0);

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
        status = SetSampleRate(0, TRXDir::Rx, 0, sampleRate, cfg.channel[0].rx.oversample);
        if (status != OpStatus::Success)
            return status;
    }

    for (int c = 0; c < 2; ++c)
    {
        SetAntenna(0, TRXDir::Tx, c, cfg.channel[c].tx.path);
        SetAntenna(0, TRXDir::Rx, c, cfg.channel[c].rx.path);
        LMS7002ChannelCalibration(*chip, cfg.channel[c], c);
    }

    if (sampleRate > 0)
    {
        status = UpdateFPGAInterface(this);
        if (status != OpStatus::Success)
            return lime::ReportError(OpStatus::Error, "Failed to update FPGA interface frequency."s);
    }
    return OpStatus::Success;
}

OpStatus LimeNET_Micro::Init()
{
    auto& lms = mLMSChips.at(0);
    OpStatus status;
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    return OpStatus::Success;
}

OpStatus LimeNET_Micro::Reset()
{
    return LMS64CProtocol::DeviceReset(*mSerialPort, 0);
}

double LimeNET_Micro::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mLMSChips[0]->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeNET_Micro::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return mLMSChips[0]->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeNET_Micro::Synchronize(bool toChip)
{
    if (toChip)
    {
        OpStatus status = mLMSChips[0]->UploadAll();
        if (status == OpStatus::Success)
        {
            //ret = SetFPGAInterfaceFreq(-1, -1, -1000, -1000); // TODO: implement
        }
        return status;
    }
    else
        return mLMSChips[0]->DownloadAll();
}

OpStatus LimeNET_Micro::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limenetmicro::SPI_LMS7002M:
        return mlms7002mPort->SPI(0, MOSI, MISO, count);
    case limenetmicro::SPI_FPGA:
        return mfpgaPort->SPI(MOSI, MISO, count);
    default:
        throw std::logic_error("LimeNET_Micro SPI invalid SPI chip select"s);
    }
}

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeNET_Micro::UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeNET_Micro* pthis = static_cast<LimeNET_Micro*>(userData);
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

double LimeNET_Micro::GetTemperature(uint8_t moduleIndex)
{
    return LMS7002M_SDRDevice::GetTemperature(moduleIndex);
}

OpStatus LimeNET_Micro::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML1_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML2_SISODDR, 1);
    const bool bypass = ((oversample == 1 || oversample == 0) && sampleRate > 61.44e6);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CDSN_RXALML, !bypass);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::EN_ADCCLKH_CLKGN, 0);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CLKH_OV_CLKL_CGEN, 2);

    return LMS7002M_SDRDevice::LMS7002M_SetSampleRate(sampleRate, oversample, oversample);
}

OpStatus LimeNET_Micro::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;

    const uint32_t addr = 0xC4;
    uint32_t value;

    OpStatus ret = mFPGA->ReadRegisters(&addr, &value, 1);
    buffer[0] = value;

    if (bufLength > 1)
        buffer[1] = (value >> 8);

    return ret;
}

OpStatus LimeNET_Micro::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;

    const uint32_t addr = 0xC2;
    uint32_t value;

    OpStatus ret = mFPGA->ReadRegisters(&addr, &value, 1);
    buffer[0] = value;

    if (bufLength > 1)
    {
        buffer[1] = (value >> 8);
    }

    return ret;
}

OpStatus LimeNET_Micro::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;

    const uint32_t addr = 0xC4;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeNET_Micro::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
        return OpStatus::InvalidValue;

    const uint32_t addr = 0xC6;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeNET_Micro::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeNET_Micro::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterRead(parameters);
}

void LimeNET_Micro::SetSerialNumber(const std::string& number)
{
    uint64_t sn = 0;
    sscanf(number.c_str(), "%16lX", &sn);
    mDeviceDescriptor.serialNumber = sn;
}

OpStatus LimeNET_Micro::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;

    if (channel == 0)
        return SetRFSwitch(trx, path);
    else
        return status;
}

OpStatus LimeNET_Micro::SetRFSwitch(TRXDir dir, uint8_t path)
{
    const int reg3 = mFPGA->ReadRegister(3);
    const int bom_ver = reg3 >> 4;
    const int hw_ver = reg3 & 0xF;

    if (dir == TRXDir::Rx)
    {
        switch (path)
        {
        case 3: // LNAW
            lime::warning("LNAW has no connection to RF ports");
            break;
        case 2: // LNAL
        {
            if (hw_ver >= 3)
            {
                constexpr Register rxRFswitch(0x0017, 10, 8);
                return ModifyRegister(mfpgaPort.get(), rxRFswitch, 5);
            }
            else
            {
                constexpr Register rxRFswitch(0x0017, 9, 8);
                return ModifyRegister(mfpgaPort.get(), rxRFswitch, 1);
            }
            break;
        }
        case 1: // LNAH
        {
            if (hw_ver >= 3)
            {
                constexpr Register rxRFswitch(0x0017, 10, 8);
                return ModifyRegister(mfpgaPort.get(), rxRFswitch, 6);
            }
            else
            {
                constexpr Register rxRFswitch(0x0017, 9, 8);
                uint8_t value = bom_ver == 0 ? 1 : 2;
                return ModifyRegister(mfpgaPort.get(), rxRFswitch, value);
            }
            break;
        }
        default:
            break; // not connected
        }
    }
    else
    {
        switch (path)
        {
        case 2: // BAND2
        {
            if (hw_ver >= 3)
            {
                uint16_t value = mFPGA->ReadRegister(0x17);
                value &= ~(0x7001);
                return mFPGA->WriteRegister(0x17, value | 0x6000);
            }
            else
            {
                Register txRFswitch(0x0017, 13, 12);
                const uint16_t value = bom_ver == 0 ? 1 : 2;
                return ModifyRegister(mfpgaPort.get(), txRFswitch, value);
            }
        }
        break;
        case 1: // BAND1
        {
            if (hw_ver >= 3)
            {
                uint16_t value = mFPGA->ReadRegister(0x17);
                value &= ~(0x7001);
                return mFPGA->WriteRegister(0x17, value | 0x5000);
            }
            else
            {
                Register txRFswitch(0x0017, 13, 12);
                return ModifyRegister(mfpgaPort.get(), txRFswitch, 1);
            }
        }
        break;
        default: // not connected
            break;
        }
    }
    return OpStatus::Success;
}

std::unique_ptr<lime::RFStream> LimeNET_Micro::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    constexpr uint8_t rxBulkEndpoint = 0x83;
    constexpr uint8_t txBulkEndpoint = 0x03;
    auto rxdma = std::make_shared<USBDMAEmulation>(mStreamPort, rxBulkEndpoint, DataTransferDirection::DeviceToHost);
    auto txdma = std::make_shared<USBDMAEmulation>(mStreamPort, txBulkEndpoint, DataTransferDirection::HostToDevice);

    std::unique_ptr<TRXLooper> streamer = std::make_unique<TRXLooper>(
        std::static_pointer_cast<IDMA>(rxdma), std::static_pointer_cast<IDMA>(txdma), mFPGA.get(), mLMSChips.at(0).get(), 0);
    if (!streamer)
        return streamer;

    if (mCallback_logMessage)
        streamer->SetMessageLogCallback(mCallback_logMessage);
    OpStatus status = streamer->Setup(config);
    if (status != OpStatus::Success)
        return std::unique_ptr<RFStream>(nullptr);
    return streamer;
}

} // namespace lime
