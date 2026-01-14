#include "LimeSDR_Micro.h"

#include <cmath>
#include <sstream>
#include <string>
#include <iostream>
#include <fstream>

#include "limesuiteng/Logger.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/ToString.h"

#include "boards/LimeSDR_Micro/LimeSDR_Micro.h"
#include "chips/LMS7002M/validation.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/LimePCIeDMA.h"
#include "comms/IDMA.h"
#include "DeviceTreeNode.h"

#include "protocols/LMS64CProtocol.h"
#include "protocols/LMS64C/SPI.h"
#include "streaming/TRXLooper.h"

#ifdef __unix__
    // Linux headers
    #include <fcntl.h> // Contains file controls like O_RDWR
    #include <errno.h> // Error integer and strerror() function
    #include <termios.h> // Contains POSIX terminal control definitions
    #include <unistd.h> // write(), read(), close()
#endif

#include "chips/LMS7002M/validation.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "chips/LA9310/LA9310.h"
#include "comms/I2Cbus.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"

#include "CommonFunctions.h"
#include "DeviceTreeNode.h"

#include "LA9310_TRX.h"
#include "comms/PCIe/LA9310_PCIe.h"

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

namespace lime {

namespace limesdrmicro {
// XTRX board specific devices ids and data
static const uint8_t SPI_LMS7002M = 0;

static const CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter cp_board_temperature = { "Board Temperature"s, 1, 0, 65535, true };

static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_LimeSDR_Micro = {};

static OpStatus ReadFileIntoVector(const std::string& filepath, std::vector<char>& data)
{
    std::ifstream inputFile;
    inputFile.open(filepath, std::ifstream::in | std::ifstream::binary);
    if (!inputFile)
        return OpStatus::FileNotFound;
    inputFile.seekg(0, std::ios_base::end);
    auto cnt = inputFile.tellg();
    inputFile.seekg(0, std::ios_base::beg);
    data.resize(cnt);
    inputFile.read(data.data(), cnt);
    inputFile.close();
    return OpStatus::Success;
}

} // namespace limesdrmicro

/// @brief Constructs a new LimeSDR_Micro object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
LimeSDR_Micro::LimeSDR_Micro(std::shared_ptr<ISPI> spiRFsoc,
    std::shared_ptr<LimePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    std::shared_ptr<LA9310_PCIe> streamingPort,
    //std::shared_ptr<I2C_bus> i2c_bus,
    double refClk)
    : LMS7002M_SDRDevice()
    , lmsSPI(spiRFsoc)
    , mSerialPort(control)
    , mStreamingPort(streamingPort)
    , la9310(std::make_shared<LA9310>(streamingPort))
    // , mI2C(i2c_bus)
    , mConfigInProgress(false)
{
    mStreamers.resize(1);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_MICRO);

    if (la9310->IsM4CoreProgrammed())
    {
        LMS64CProtocol::FirmwareInfo fw{};
        LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw, 0);
        LMS64CProtocol::FirmwareToDescriptor(fw, desc);
    }

    desc.spiSlaveIds = { { "LMS7002M"s, limesdrmicro::SPI_LMS7002M } };
    desc.i2cBusIds = { { "LA9310"s, 0 } };

    desc.customParameters = { limesdrmicro::cp_vctcxo_dac, limesdrmicro::cp_board_temperature };

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

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(lmsSPI);

        chip->ModifyRegistersDefaults(limesdrmicro::lms7002defaultsOverrides_LimeSDR_Micro);
        // chip->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    // const std::unordered_map<std::string, Region> flashMap = { { "VCTCXO_DAC"s, { 0x01FF0000, 2 } } };
    desc.memoryDevices[ToString(eMemoryDevice::ARM_M4)] = std::make_shared<DataStorage>(this, eMemoryDevice::ARM_M4);
    desc.memoryDevices[ToString(eMemoryDevice::VSPA)] = std::make_shared<DataStorage>(this, eMemoryDevice::VSPA);
    desc.memoryDevices[ToString(eMemoryDevice::EEPROM)] = std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM);
    {
        const std::unordered_map<std::string, Region> eepromMap = { { "XO_DAC"s, { 0xFFFE, 2 } } };
        desc.memoryDevices[ToString(eMemoryDevice::EEPROM)] =
            std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM, std::move(eepromMap));
    }

    desc.socTree = std::make_shared<DeviceTreeNode>(this, "SDRDevice", "LimeSDR-Micro"s);
    desc.socTree->children.push_back(std::make_shared<DeviceTreeNode>(la9310.get(), "LA9310"s));
    desc.socTree->children.push_back(std::make_shared<DeviceTreeNode>(mLMSChips.at(0).get(), "LMS7002M"s));
}

LimeSDR_Micro::~LimeSDR_Micro()
{
}

static OpStatus InitLMS7002M(LMS7002M& lms, bool skipTune = false)
{
    return OpStatus::Success;
}

static OpStatus SetLA9310SamplingRate(std::shared_ptr<LA9310> la9310, const SDRConfig& cfg)
{
    double maxDACrequest = 0;
    double maxADCrequest = 0;
    // find highest requested sampling rate
    for (size_t i = 0; i < cfg.channel.size(); ++i)
    {
        if (cfg.channel[i].rx.enabled)
        {
            double dacRate = cfg.channel[i].rx.sampleRate * std::max(1u, uint(cfg.channel[i].rx.oversample));
            if (dacRate > maxDACrequest)
                maxDACrequest = dacRate;
        }
        if (cfg.channel[i].tx.enabled)
        {
            double adcRate = cfg.channel[i].tx.sampleRate * std::max(1u, uint(cfg.channel[i].tx.oversample));
            if (adcRate > maxADCrequest)
                maxADCrequest = adcRate;
        }
    }

    double systemClock = std::max(maxADCrequest, maxDACrequest);
    uint8_t adc_divider_mask = 0;
    uint8_t dac_divider_mask = 0;

    if (systemClock <= 80e6)
    {
        // run higher system clock rate with half sampling rate
        systemClock *= 2;
        adc_divider_mask = 0xF;
        dac_divider_mask = 0x1;
    }

    if (systemClock < 1.25e6) // minimal system clock that can be generated by LMS7002M CGEN
        return ReportError(OpStatus::OutOfRange, "Cannot satisfy requested sample rate");

    OpStatus status = la9310->SetSystemClock(systemClock, adc_divider_mask, dac_divider_mask);
    if (status != OpStatus::Success)
        return ReportError(status, "Failed to set LA9310 system clock");

    return OpStatus::Success;
}

OpStatus LimeSDR_Micro::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    OpStatus status;
    if (!la9310->IsM4CoreProgrammed())
    {
        std::vector<char> firmware;
        const std::string m4firmware_path = "/lib/firmware/la9310.bin";
        status = limesdrmicro::ReadFileIntoVector(m4firmware_path, firmware);
        if (status != OpStatus::Success)
            return status;
        lime::info("Programming LimeSDR-Micro " + m4firmware_path);
        const std::string vspafirmware_path = "/lib/firmware/apm-iqplayer.eld";
        status = mStreamingPort->LoadArmM4Firmware(firmware.data(), firmware.size());
        if (status != OpStatus::Success)
            return ReportError(status, "Failed to program LimeSDR-Micro LA9310 Arm M4 firmware");
        status = limesdrmicro::ReadFileIntoVector(vspafirmware_path, firmware);
        if (status != OpStatus::Success)
            return status;
        lime::info("Programming LimeSDR-Micro VSPA " + vspafirmware_path);
        status = mStreamingPort->LoadVSPAFirmware(firmware.data(), firmware.size());
        if (status != OpStatus::Success)
            return ReportError(status, "Failed to program LimeSDR-Micro LA9310 VSPA firmware");
    }

    auto& rfsoc = mLMSChips.at(0);

    status = SetLA9310SamplingRate(la9310, cfg);
    if (status != OpStatus::Success)
        return status;

    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS7002M(*rfsoc, skipTune);
    }

    status = LMS7002M_Configure(*rfsoc, cfg);
    if (status != OpStatus::Success)
        return status;

    for (int c = 0; c < 2; ++c)
    {
        LMSSetPath(TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMSSetPath(TRXDir::Rx, c, cfg.channel[c].rx.path);
        // LMS7002ChannelCalibration(*rfsoc, cfg.channel[c], c);
    }
    if ((cfg.channel[0].rx.calibrate & CalibrationFlag::DCIQ) && cfg.channel[0].rx.enabled)
    {
        status = CalibrateRx();
        if (status != OpStatus::Success)
            return status;
    }
    if ((cfg.channel[0].tx.calibrate & CalibrationFlag::DCIQ) && cfg.channel[0].tx.enabled)
    {
        status = CalibrateTx();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

const SDRDescriptor& LimeSDR_Micro::GetDescriptor() const
{
    return mDeviceDescriptor;
}

OpStatus LimeSDR_Micro::Init()
{
    const bool skipTune = true;
    return InitLMS7002M(*mLMSChips.at(0), skipTune);
}

OpStatus LimeSDR_Micro::Reset()
{
    return OpStatus::NotImplemented;
}

double LimeSDR_Micro::GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency)
{
    return OpStatus::NotImplemented;
}

double LimeSDR_Micro::GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetNCOFrequency(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset)
{
    return OpStatus::NotImplemented;
}

int LimeSDR_Micro::GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv)
{
    return OpStatus::NotImplemented;
}

double LimeSDR_Micro::GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return OpStatus::NotImplemented;
}

double LimeSDR_Micro::GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf)
{
    return OpStatus::NotImplemented;
}

uint8_t LimeSDR_Micro::GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

double LimeSDR_Micro::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->GetClockFreq(static_cast<LMS7002M::ClockID>(clk_id));
}

OpStatus LimeSDR_Micro::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    auto& chip = mLMSChips.at(channel / 2);
    return chip->SetClockFreq(static_cast<LMS7002M::ClockID>(clk_id), freq);
}

// OpStatus LimeSDR_Micro::SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value)
// {
//     return OpStatus::NotImplemented;
// }

// OpStatus LimeSDR_Micro::GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value)
// {
//     return OpStatus::NotImplemented;
// }

bool LimeSDR_Micro::GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return false;
}

OpStatus LimeSDR_Micro::SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic)
{
    return OpStatus::NotImplemented;
}

complex64f_t LimeSDR_Micro::GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return complex64f_t();
}

OpStatus LimeSDR_Micro::SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset)
{
    return OpStatus::NotImplemented;
}

complex64f_t LimeSDR_Micro::GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return complex64f_t();
}

OpStatus LimeSDR_Micro::SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance)
{
    return OpStatus::NotImplemented;
}

bool LimeSDR_Micro::GetCGENLocked(uint8_t moduleIndex)
{
    return false;
}

double LimeSDR_Micro::GetTemperature(uint8_t moduleIndex)
{
    return 0;
}

bool LimeSDR_Micro::GetSXLocked(uint8_t moduleIndex, TRXDir trx)
{
    return false;
}

unsigned int LimeSDR_Micro::ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA)
{
    return 0;
}

OpStatus LimeSDR_Micro::WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::LoadConfig(uint8_t moduleIndex, const std::string& filename)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::SaveConfig(uint8_t moduleIndex, const std::string& filename)
{
    return OpStatus::NotImplemented;
}

uint16_t LimeSDR_Micro::GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value)
{
    return OpStatus::NotImplemented;
}

uint16_t LimeSDR_Micro::GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetParameter(
    uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::Synchronize(bool toChip)
{
    return OpStatus::NotImplemented;
}

void LimeSDR_Micro::EnableCache(bool enable)
{
}

OpStatus LimeSDR_Micro::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth)
{
    return trx == TRXDir::Rx ? CalibrateRx() : CalibrateTx();
}

OpStatus LimeSDR_Micro::ConfigureGFIR(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings)
{
    return OpStatus::NotImplemented;
}

std::vector<double> LimeSDR_Micro::GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID)
{
    return std::vector<double>();
}

OpStatus LimeSDR_Micro::SetGFIRCoefficients(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled)
{
    return OpStatus::NotImplemented;
}

uint64_t LimeSDR_Micro::GetHardwareTimestamp(uint8_t moduleIndex)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::SetTestSignal(uint8_t moduleIndex,
    TRXDir direction,
    uint8_t channel,
    ChannelConfig::Direction::TestSignal signalConfiguration,
    int16_t dc_i,
    int16_t dc_q)
{
    return OpStatus::NotImplemented;
}

void LimeSDR_Micro::StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx)
{
}

ChannelConfig::Direction::TestSignal LimeSDR_Micro::GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel)
{
    return ChannelConfig::Direction::TestSignal();
}

OpStatus LimeSDR_Micro::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return lmsSPI->Transact(MOSI, MISO, count);
}

void LimeSDR_Micro::SetMessageLogCallback(LogCallbackType callback)
{
    mCallback_logMessage = callback;
}

OpStatus LimeSDR_Micro::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterWrite(*mSerialPort, parameters, 0);
}

OpStatus LimeSDR_Micro::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return LMS64CProtocol::CustomParameterRead(*mSerialPort, parameters, 0);
}

void LimeSDR_Micro::LMSSetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    const uint8_t i2c_expander_address = 0x20;
    uint8_t value = 0;
    I2CRead(0, i2c_expander_address, 0x19, 1, &value, 1);
    if (dir == TRXDir::Tx)
    {
        lms->SetBandTRF(pathId);
        value &= ~(1 << 1); // clear TX_SW
        if (pathId == 1)
            value |= (1 << 1); // set TX_SW, Band1
        else
            value |= (0 << 1); // set TX_SW, Band2
        I2CWrite(0, i2c_expander_address, 0x19, 1, &value, 1);
    }
    else
    {
        lime::LMS7002M::PathRFE path{ pathId };
        // first configure chip path or loopback
        lms->SetPathRFE(lime::LMS7002M::PathRFE(path));
        value &= ~(0x5); // clear RX_SW2, RX_SW3
        uint8_t rxsw2 = 0;
        uint8_t rxsw3 = 0;
        if (path == LMS7002M::PathRFE::NONE)
        {
            rxsw2 = 1;
            rxsw3 = 1;
        }
        else if (path == LMS7002M::PathRFE::LNAH)
        {
            rxsw2 = 1;
            rxsw3 = 0;
        }
        else if (path == LMS7002M::PathRFE::LNAL)
        {
            rxsw2 = 0;
            rxsw3 = 1;
        }
        else if (path == LMS7002M::PathRFE::LNAW)
        {
            rxsw2 = 0;
            rxsw3 = 0;
        }
        value |= (rxsw2 << 0) | (rxsw3 << 2); // set TX_SW, Band2
        I2CWrite(0, i2c_expander_address, 0x19, 1, &value, 1);
    }
}

OpStatus LimeSDR_Micro::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    OpStatus status;

    switch (device)
    {
    case eMemoryDevice::ARM_M4: {
        status = mStreamingPort->LoadArmM4Firmware(data, length);
        if (status != OpStatus::Success)
            return status;

        // Update firmware information after firmware change
        LMS64CProtocol::FirmwareInfo fw{};
        LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw, 0);
        LMS64CProtocol::FirmwareToDescriptor(fw, mDeviceDescriptor);
        return status;
    }
    case eMemoryDevice::VSPA:
        // Make sure the VCPU is stopped
        status = la9310->vspa.ResetVCPU();
        if (status != OpStatus::Success)
            return status;
        // (Re-)programm the firmware
        return mStreamingPort->LoadVSPAFirmware(data, length);
    default:
        return OpStatus::NotImplemented;
    }
}

OpStatus LimeSDR_Micro::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryWrite(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, 0);
}

OpStatus LimeSDR_Micro::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr || storage->ownerDevice != this || storage->memoryDeviceType != eMemoryDevice::EEPROM)
    {
        return OpStatus::Error;
    }
    return LMS64CProtocol::MemoryRead(
        *mSerialPort, LMS64CProtocol::MEMORY_WR_targets::EEPROM, region.address, data, region.size, 0);
}

OpStatus LimeSDR_Micro::UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::GetGPSLock(GPS_Lock* status)
{
    return OpStatus::NotImplemented;
}

// GPIO_Interface* LimeSDR_Micro::GetGPIOControls()
// {
//     return mGPIO.get();
// }

OpStatus LimeSDR_Micro::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LimeSDR_Micro::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    OpStatus status = LMS7002M_SDRDevice::SetAntenna(moduleIndex, trx, channel, path);
    if (status != OpStatus::Success)
        return status;
    LMSSetPath(trx, channel, path);
    return OpStatus::Success;
}

std::unique_ptr<lime::RFStream> LimeSDR_Micro::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    auto stream = std::make_unique<LA9310_TRX>(la9310);
    StreamConfig config_mod = config;
    if (config.hintSampleRate <= 0)
        config_mod.hintSampleRate = GetSampleRate(0, TRXDir::Rx, 0);

    if (mCallback_logMessage)
        stream->SetMessageLogCallback(mCallback_logMessage);
    stream->Setup(config_mod);
    return stream;
}

OpStatus LimeSDR_Micro::I2CWrite(
    uint32_t bus, uint32_t soc, uint32_t offset, uint8_t offset_len, const uint8_t* data, uint32_t length)
{
    return LMS64CProtocol::I2C_Write(*mSerialPort, soc, offset, offset_len, data, length);
}

OpStatus LimeSDR_Micro::I2CRead(uint32_t bus, uint32_t soc, uint32_t offset, uint8_t offset_len, uint8_t* data, uint32_t length)
{
    return LMS64CProtocol::I2C_Read(*mSerialPort, soc, offset, offset_len, data, length);
}

OpStatus LimeSDR_Micro::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    return OpStatus::Error;
}

void LimeSDR_Micro::StreamStart(uint8_t moduleIndex)
{
}
void LimeSDR_Micro::StreamStop(uint8_t moduleIndex)
{
}
void LimeSDR_Micro::StreamDestroy(uint8_t moduleIndex)
{
}

double LimeSDR_Micro::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate)
{
    // sampling rate is dictated by LMS7002M MCLK1, and LA9310 ADC/DAC clock divider
    auto& rfsoc = mLMSChips.at(0);
    const double cgenFrequency = rfsoc->GetFrequencyCGEN();
    const int CLKL_divider = (1 << rfsoc->Get_SPI_Reg_bits(LMS7002MCSR::CLKH_OV_CLKL_CGEN, true));
    const double MCLK1_frequency = cgenFrequency / CLKL_divider;

    uint8_t adcRate, dacRate;
    la9310->GetADCDACRates(&adcRate, &dacRate);

    double rate = MCLK1_frequency;
    // ADC, DAC clock dividers by 2
    if (trx == TRXDir::Tx)
        rate /= (1 << dacRate);
    else
        rate /= (1 << ((dacRate >> channel) & 1));

    // TODO: take into account firmware applied decimation/interpolation
    if (rf_samplerate)
        *rf_samplerate = rate;
    return rate;
}

} //namespace lime
