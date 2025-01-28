#include "LimeSDR_Mini.h"

#include <cassert>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"

#include "comms/IComms.h"
#include "chips/Si5351C/Si5351C.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "comms/ISerialPort.h"
#include "comms/SPI_utilities.h"
#include "comms/USB/IUSB.h"
#include "comms/USB/FT601/FT601.h"
#include "comms/USB/USBDMAEmulation.h"

#include "protocols/LMSBoards.h"
#include "protocols/LMS64CProtocol.h"

#include "utilities/toString.h"

#include "DeviceTreeNode.h"
#include "streaming/TRXLooper.h"

#include "FPGA_Mini.h"

using namespace lime::LMS64CProtocol;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

namespace lime {
namespace limesdrmini {

// hardware versions reported by gateware
// v1.0 - 0
// v1.1 - 0
// v1.2 - 0
// v1.3 - 3
// v2.0 - 3
// v2.1 - 4
// v2.2 - 5
// v2.3 - 6
// v2.4 - 7

static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static const CustomParameter CP_VCTCXO_DAC_v1 = { "VCTCXO DAC (runtime)"s, 0, 0, 255, false };
static const CustomParameter CP_VCTCXO_DAC_v2 = { "VCTCXO DAC (runtime)"s, 0, 0, 1023, false };

static const CustomParameter CP_TEMPERATURE = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_1v0 = { //
    { 0x0020, 0xFFD5 },
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
    { 0x0092, 0xFFFF },
    { 0x0093, 0x03FF },
    { 0x00A1, 0x656A },
    { 0x00A6, 0x0001 },
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
    { 0x0119, 0x18CC },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0740 },
    { 0x0120, 0xE6C0 },
    { 0x0121, 0x8650 },
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

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_1v2 = { //
    { 0x0020, 0xFFD5 },
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
    { 0x0119, 0x18CC },
    { 0x011C, 0x8941 },
    { 0x011D, 0x0000 },
    { 0x011E, 0x0740 },
    { 0x0120, 0xC5C0 },
    { 0x0121, 0x8650 },
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

} // namespace limesdrmini

/// @brief Constructs a new LimeSDR_Mini object
/// @param spiLMS The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param streamPort The communications port to send and receive sample data.
/// @param commsPort The communications port for direct communications with the device.
LimeSDR_Mini::LimeSDR_Mini(std::shared_ptr<IComms> spiLMS,
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
    double refClk = mFPGA->DetectRefClk();

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, descriptor);

    if (descriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI))
    {
        descriptor.customParameters.push_back(limesdrmini::CP_VCTCXO_DAC_v1);
    }
    else if (descriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI_V2))
    {
        descriptor.customParameters.push_back(limesdrmini::CP_VCTCXO_DAC_v2);
        descriptor.customParameters.push_back(limesdrmini::CP_TEMPERATURE);
    }

    {
        RFSOCDescriptor soc{ GetDefaultLMS7002MDescriptor() };
        // override specific capabilities
        soc.channelCount = 1;

        soc.pathNames[TRXDir::Rx] = { "NONE"s, "LNAH"s, "LNAL_NC"s, "LNAW"s }; // LNAL is not connected
        soc.samplingRateRange = { 100e3, 30.72e6, 0 };
        soc.frequencyRange = { 10e6, 3.5e9, 0 };
        descriptor.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(mlms7002mPort);
        if (gw.hardwareVersion >= 2)
            chip->ModifyRegistersDefaults(limesdrmini::lms7002defaultsOverrides_1v2);
        else
            chip->ModifyRegistersDefaults(limesdrmini::lms7002defaultsOverrides_1v0);
        chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    descriptor.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);

    descriptor.spiSlaveIds = { { "LMS7002M"s, limesdrmini::SPI_LMS7002M }, { "FPGA"s, limesdrmini::SPI_FPGA } };

    auto fpgaNode = std::make_shared<DeviceTreeNode>("FPGA"s, eDeviceTreeNodeClass::FPGA_MINI, mFPGA.get());
    fpgaNode->children.push_back(
        std::make_shared<DeviceTreeNode>("LMS7002"s, eDeviceTreeNodeClass::LMS7002M, mLMSChips.at(0).get()));
    descriptor.socTree = std::make_shared<DeviceTreeNode>("LimeSDR-Mini"s, eDeviceTreeNodeClass::SDRDevice, this);
    descriptor.socTree->children.push_back(fpgaNode);
}

LimeSDR_Mini::~LimeSDR_Mini()
{
}

OpStatus LimeSDR_Mini::Configure(const SDRConfig& cfg, uint8_t moduleIndex = 0)
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

OpStatus LimeSDR_Mini::Init()
{
    auto& lms = mLMSChips.at(0);
    OpStatus status = LMS64CProtocol::DeviceReset(*mSerialPort, 0);
    if (status != OpStatus::Success)
        return status;

    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

    // TODO: replace with static register values
    // Do TxLPF configuration to set CG_IAMP_TBB to reasonable value, as they are related.
    // Otherwise if TxLPF is not configured, or CG_IAMP_TBB is not set explicitly to match it.
    // it can result in inconsistent Tx gain results.
    lms->SetActiveChannel(LMS7002M::Channel::ChA);
    lms->SetTxLPF(20e6);
    lms->SetRxLPF(20e6);

    return OpStatus::Success;
}

OpStatus LimeSDR_Mini::Reset()
{
    return LMS64CProtocol::DeviceReset(*mSerialPort, 0);
}

double LimeSDR_Mini::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mLMSChips[0]->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_Mini::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return mLMSChips[0]->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

OpStatus LimeSDR_Mini::Synchronize(bool toChip)
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

OpStatus LimeSDR_Mini::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    switch (chipSelect)
    {
    case limesdrmini::SPI_LMS7002M:
        return mlms7002mPort->SPI(0, MOSI, MISO, count);
    case limesdrmini::SPI_FPGA:
        return mfpgaPort->SPI(MOSI, MISO, count);
    default:
        throw std::logic_error("LimeSDR_Mini SPI invalid SPI chip select"s);
    }
}

// Callback for updating FPGA's interface clocks when LMS7002M CGEN is manually modified
OpStatus LimeSDR_Mini::UpdateFPGAInterface(void* userData)
{
    assert(userData != nullptr);
    LimeSDR_Mini* pthis = static_cast<LimeSDR_Mini*>(userData);
    return UpdateFPGAInterfaceFrequency(*pthis->mLMSChips.at(0), *pthis->mFPGA, 0);
}

double LimeSDR_Mini::GetTemperature(uint8_t moduleIndex)
{
    if (mDeviceDescriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI))
    {
        throw std::logic_error("LimeSDR-Mini v1 doesn't have a temperature sensor"s);
    }

    return LMS7002M_SDRDevice::GetTemperature(moduleIndex);
}

OpStatus LimeSDR_Mini::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML1_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML2_SISODDR, 1);
    const bool bypass = ((oversample == 1 || oversample == 0) && sampleRate > 61.44e6);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CDSN_RXALML, !bypass);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::EN_ADCCLKH_CLKGN, 0);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CLKH_OV_CLKL_CGEN, 2);

    return LMS7002M_SDRDevice::LMS7002M_SetSampleRate(sampleRate, oversample, oversample);
}

OpStatus LimeSDR_Mini::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC4;
    uint32_t value;

    OpStatus ret = mFPGA->ReadRegisters(&addr, &value, 1);
    buffer[0] = value;

    if (bufLength > 1)
    {
        buffer[1] = (value >> 8);
    }

    return ret;
}

OpStatus LimeSDR_Mini::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

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

OpStatus LimeSDR_Mini::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC4;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeSDR_Mini::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    if (!buffer || bufLength == 0)
    {
        return OpStatus::InvalidValue;
    }

    const uint32_t addr = 0xC6;
    const uint32_t value = (bufLength == 1) ? buffer[0] : buffer[0] | (buffer[1] << 8);

    return mFPGA->WriteRegisters(&addr, &value, 1);
}

OpStatus LimeSDR_Mini::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterWrite(parameters);
}

OpStatus LimeSDR_Mini::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return mfpgaPort->CustomParameterRead(parameters);
}

OpStatus LimeSDR_Mini::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    int progMode;
    LMS64CProtocol::ALTERA_FPGA_GW_WR_targets target = LMS64CProtocol::ALTERA_FPGA_GW_WR_targets::FPGA;

    switch (device)
    {
    case eMemoryDevice::FPGA_FLASH:
        progMode = 1;
        break;
    default:
        return OpStatus::InvalidValue;
    }

    const char* data_src = data;

    // LimeSDR-Mini v1.X, needs data modification
    std::vector<char> v1_buffer;

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    if (gw.boardID == LMS_DEV_LIMESDRMINI && gw.hardwareVersion <= 3) // LimeSDR-Mini v1.X
    {
        if (gw.version != 0)
        {
            // boot from flash
            mfpgaPort->ProgramWrite(nullptr, 0, 2, static_cast<int>(target), nullptr);
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }

        const int sizeUFM = 0x8000;
        const int sizeCFM0 = 0x42000;
        const int startUFM = 0x1000;
        const int startCFM0 = 0x4B000;

        if (length != startCFM0 + sizeCFM0)
            return ReportError(OpStatus::Error, "LimeSDR_Mini: Invalid image file");

        v1_buffer.resize(sizeUFM + sizeCFM0);
        memcpy(v1_buffer.data(), data + startUFM, sizeUFM);
        memcpy(v1_buffer.data() + sizeUFM, data + startCFM0, sizeCFM0);

        data_src = v1_buffer.data();
    }

    OpStatus status = mfpgaPort->ProgramWrite(data_src, length, progMode, static_cast<int>(target), callback);
    if (status != OpStatus::Success)
        return status;

    progMode = 2; // boot from FLASH
    return mfpgaPort->ProgramWrite(nullptr, 0, progMode, static_cast<int>(target), nullptr);
}

void LimeSDR_Mini::SetSerialNumber(const std::string& number)
{

    uint64_t sn = 0;
    sscanf(number.c_str(), "%16lX", &sn);
    mDeviceDescriptor.serialNumber = sn;
}

OpStatus LimeSDR_Mini::SetRFSwitch(TRXDir dir, uint8_t path)
{
    if (dir == TRXDir::Rx)
    {
        Register rxRFswitch(0x0017, 9, 8);
        uint8_t value = 0;
        switch (path)
        {
        case 3: // LNAW
            value = 0x2;
            break;
        case 2: // LNAL
            lime::warning("LNAL has no connection to RF ports");
            break;
        case 1: // LNAH
            value = 0x1;
            break;
        default:
            value = 0;
            break; // not connected
        }
        return ModifyRegister(mfpgaPort.get(), rxRFswitch, value);
    }
    else
    {
        Register txRFswitch(0x0017, 13, 12);
        uint8_t value = 0;
        switch (path)
        {
        case 2: // BAND2
            value = 0x2;
            break;
        case 1: // BAND1
            value = 0x1;
            break;
        default: // not connected
            value = 0;
            break;
        }
        return ModifyRegister(mfpgaPort.get(), txRFswitch, value);
    }
}

OpStatus LimeSDR_Mini::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;

    if (channel == 0)
        return SetRFSwitch(trx, path);
    else
        return status;
}

std::unique_ptr<lime::RFStream> LimeSDR_Mini::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
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