#include "LimeSDR_Mini.h"

#include "comms/USB/IUSB.h"
#include "LMSBoards.h"
#include "limesuiteng/LMS7002M.h"
#include "chips/Si5351C/Si5351C.h"
#include "LMS64CProtocol.h"
#include "limesuiteng/Logger.h"
#include "FPGA_Mini.h"
#include "comms/USB/USBDMAEmulation.h"
#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "protocols/LMS64CProtocol.h"
#include "DeviceTreeNode.h"
#include "comms/IComms.h"
#include "ISerialPort.h"
#include "comms/USB/FT601/FT601.h"
#include "comms/SPI_utilities.h"
#include "streaming/TRXLooper.h"

#include <assert.h>
#include <memory>
#include <set>
#include <stdexcept>
#include <cmath>

using namespace lime;
using namespace lime::LMS64CProtocol;
using namespace lime::LMS7002MCSR_Data;
using namespace std::literals::string_literals;

static const uint8_t SPI_LMS7002M = 0;
static const uint8_t SPI_FPGA = 1;

static const CustomParameter CP_VCTCXO_DAC = { "VCTCXO DAC (runtime)"s, 0, 0, 255, false };
static const CustomParameter CP_TEMPERATURE = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_1v0 = { //
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
    { 0x0119, 0x18D2 },
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
    SDRDescriptor& descriptor = mDeviceDescriptor;

    LMS64CProtocol::FirmwareInfo fw{};
    LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw);
    LMS64CProtocol::FirmwareToDescriptor(fw, descriptor);

    mFPGA = std::make_unique<FPGA_Mini>(spiFPGA, spiLMS);
    double refClk = mFPGA->DetectRefClk();

    FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    FPGA::GatewareToDescriptor(gw, descriptor);

    descriptor.customParameters.push_back(CP_VCTCXO_DAC);
    if (descriptor.name == GetDeviceName(LMS_DEV_LIMESDRMINI_V2))
    {
        descriptor.customParameters.push_back(CP_TEMPERATURE);
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
            chip->ModifyRegistersDefaults(lms7002defaultsOverrides_1v2);
        else
            chip->ModifyRegistersDefaults(lms7002defaultsOverrides_1v0);
        chip->SetOnCGENChangeCallback(UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        mLMSChips.push_back(std::move(chip));
    }
    {
        mStreamers.reserve(mLMSChips.size());
        constexpr uint8_t rxBulkEndpoint = 0x83;
        constexpr uint8_t txBulkEndpoint = 0x03;
        auto rxdma = std::make_shared<USBDMAEmulation>(mStreamPort, rxBulkEndpoint, DataTransferDirection::DeviceToHost);
        auto txdma = std::make_shared<USBDMAEmulation>(mStreamPort, txBulkEndpoint, DataTransferDirection::HostToDevice);

        mStreamers.push_back(std::make_unique<TRXLooper>(
            std::static_pointer_cast<IDMA>(rxdma), std::static_pointer_cast<IDMA>(txdma), mFPGA.get(), mLMSChips.at(0).get(), 0));
    }

    descriptor.spiSlaveIds = { { "LMS7002M"s, SPI_LMS7002M }, { "FPGA"s, SPI_FPGA } };

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
    OpStatus status = OpStatus::Success;
    std::vector<std::string> errors;
    bool isValidConfig = LMS7002M_Validate(cfg, errors, 1);

    if (!isValidConfig)
    {
        std::stringstream ss;

        for (const auto& err : errors)
        {
            ss << err << std::endl;
        }

        return lime::ReportError(OpStatus::Error, "LimeSDR-Mini: "s + ss.str());
    }

    bool rxUsed = false;
    bool txUsed = false;
    for (int i = 0; i < 2; ++i)
    {
        const ChannelConfig& ch = cfg.channel[i];
        rxUsed |= ch.rx.enabled;
        txUsed |= ch.tx.enabled;
    }

    // config validation complete, now do the actual configuration
    try
    {
        mConfigInProgress = true;
        auto& chip = mLMSChips.at(0);
        if (!cfg.skipDefaults)
        {
            status = Init();
            if (status != OpStatus::Success)
                return status;
        }

        status = LMS7002LOConfigure(*chip, cfg);
        if (status != OpStatus::Success)
            return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: LO configuration failed."s);
        for (int i = 0; i < 2; ++i)
        {
            status = LMS7002ChannelConfigure(*chip, cfg.channel[i], i);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: channel%i configuration failed.", i);
            LMS7002TestSignalConfigure(*chip, cfg.channel[i], i);

            if (i == 0) // only channel A is connected to RF ports
            {
                SetRFSwitch(TRXDir::Rx, cfg.channel[0].rx.path);
                SetRFSwitch(TRXDir::Tx, cfg.channel[0].tx.path);
            }
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
        {
            status = SetSampleRate(0, TRXDir::Rx, 0, sampleRate, cfg.channel[0].rx.oversample);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: failed to set sampling rate."s);
        }

        for (int i = 0; i < 2; ++i)
        {
            const ChannelConfig& ch = cfg.channel[i];
            LMS7002ChannelCalibration(*chip, ch, i);
            // TODO: should report calibration failure, but configuration can
            // still work after failed calibration.
        }
        chip->SetActiveChannel(LMS7002M::Channel::ChA);

        mConfigInProgress = false;
        if (sampleRate > 0)
        {
            status = UpdateFPGAInterface(this);
            if (status != OpStatus::Success)
                return lime::ReportError(OpStatus::Error, "LimeSDR_Mini: failed to update FPGA interface frequency."s);
        }
    } //try
    catch (std::logic_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_Mini config: "s + e.what());
    } catch (std::runtime_error& e)
    {
        return ReportError(OpStatus::Error, "LimeSDR_Mini config: "s + e.what());
    }
    return OpStatus::Success;
}

OpStatus LimeSDR_Mini::Init()
{
    auto& lms = mLMSChips.at(0);
    OpStatus status;
    status = lms->ResetChip();
    if (status != OpStatus::Success)
        return status;

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
    case SPI_LMS7002M:
        return mlms7002mPort->SPI(0, MOSI, MISO, count);
    case SPI_FPGA:
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
    const bool bypass = (oversample == 1);
    uint8_t decimation = 7; // HBD_OVR_RXTSP=7 - bypass
    uint8_t interpolation = 7; // HBI_OVR_TXTSP=7 - bypass
    double cgenFreq = sampleRate * 4; // AI AQ BI BQ
    // TODO:
    // for (uint8_t i = 0; i < GetNumChannels(false) ;i++)
    // {
    //     if (rx_channels[i].cF_offset_nco != 0.0 || tx_channels[i].cF_offset_nco != 0.0)
    //     {
    //         bypass = false;
    //         break;
    //     }
    // }

    if (bypass)
    {
        lime::info("Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: bypass, Interp: bypass", sampleRate / 1e6, cgenFreq / 1e6);
    }
    else
    {
        if (oversample == 0)
        {
            const int n = lime::LMS7002M::CGEN_MAX_FREQ / (cgenFreq);
            oversample = (n >= 32) ? 32 : (n >= 16) ? 16 : (n >= 8) ? 8 : (n >= 4) ? 4 : 2;
        }

        decimation = 4;
        if (oversample <= 16)
        {
            constexpr std::array<int, 17> decimationTable{ 0, 0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3 };
            decimation = decimationTable.at(oversample);
        }
        interpolation = decimation;
        cgenFreq *= 2 << decimation;

        lime::info("Sampling rate set(%.3f MHz): CGEN:%.3f MHz, Decim: 2^%i, Interp: 2^%i",
            sampleRate / 1e6,
            cgenFreq / 1e6,
            decimation + 1,
            interpolation + 1); // dec/inter ratio is 2^(value+1)
    }

    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML1_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::LML2_SISODDR, 1);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CDSN_RXALML, !bypass);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::EN_ADCCLKH_CLKGN, 0);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::CLKH_OV_CLKL_CGEN, 2);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 2);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::HBD_OVR_RXTSP, decimation);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::HBI_OVR_TXTSP, interpolation);
    mLMSChips.at(moduleIndex)->Modify_SPI_Reg_bits(LMS7002MCSR_Data::MAC, 1);
    if (bypass)
    {
        return mLMSChips.at(moduleIndex)->SetInterfaceFrequency(sampleRate * 4, 7, 7);
    }
    else
    {
        return mLMSChips.at(moduleIndex)->SetInterfaceFrequency(cgenFreq, interpolation, decimation);
    }
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
