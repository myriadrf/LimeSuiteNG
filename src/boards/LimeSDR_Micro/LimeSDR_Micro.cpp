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
#include "boards/LimeSDR_Micro/M4.h"
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
#include "chips/LA9310/VSPA_iqplayer.h"
#include "chips/LA9310/CSR.h"
#include "comms/I2Cbus.h"
#include "protocols/LMS64CProtocol.h"
#include "streaming/TRXLooper.h"
#include "interface/IDCCorrector.h"
#include "interface/IQuadratureErrorCorrector.h"

#include "CommonFunctions.h"
#include "DeviceTreeNode.h"

#include "LA9310_TRX.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "OEMTesting.h"

using namespace std::literals::string_literals;
using namespace lime::LMS7002MCSR_Data;

using std::max;

namespace lime {

namespace limesdrmicro {

static const char* vspa_firmware_name_default = "limesdr-micro-iqplayer.eld";
static const char* m4_firmware_name_default = "limesdr-micro-m4.bin";
// XTRX board specific devices ids and data
static const uint8_t SPI_LMS7002M = 0;

static const CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static const CustomParameter cp_board_temperature = { "Board Temperature"s, 1, 0, 65535, true };

// Can't use hardware reset of LMS7002M as that would affect CGEN output, which is necessary for device to function.
// Just do sofware writes of all necessary defaults
static const std::vector<std::pair<uint16_t, uint16_t>> lms7002defaultsOverrides_LimeSDR_Micro = {
    { 0x0021, 0x0E9F },
    { 0x002A, 0x0086 },
    { 0x002B, 0x001C },
    { 0x002C, 0xFFFF },
    { 0x002E, 0x0000 },
    { 0x0081, 0x0000 },
    { 0x0082, 0x800F },
    { 0x0084, 0x0400 },
    { 0x0085, 0x0001 },
    { 0x0086, 0x4101 },
    { 0x008A, 0x0514 },
    { 0x008C, 0x267B },
    { 0x0092, 0x0001 },
    { 0x0093, 0x0000 },
    { 0x0094, 0x0000 },
    { 0x0095, 0x0000 },
    { 0x0096, 0x0000 },
    { 0x0097, 0x0000 },
    { 0x0098, 0x0000 },
    { 0x0099, 0x6565 },
    { 0x009A, 0x658C },
    { 0x009B, 0x6565 },
    { 0x009C, 0x658C },
    { 0x009D, 0x6565 },
    { 0x009E, 0x658C },
    { 0x009F, 0x658C },
    { 0x00A0, 0x6565 },
    { 0x00A1, 0x6565 },
    { 0x00A2, 0x6565 },
    { 0x00A3, 0x6565 },
    { 0x00A4, 0x6565 },
    { 0x00A5, 0x6565 },
    { 0x00A6, 0x000F },
    { 0x00A7, 0x6565 },
    { 0x00A8, 0x0000 },
    { 0x0100, 0x3409 },
    { 0x0101, 0x7FFE },
    { 0x0102, 0x3180 },
    { 0x0103, 0x0A12 },
    { 0x0104, 0x0088 },
    { 0x0105, 0x0007 },
    { 0x0106, 0x318C },
    { 0x0107, 0x318C },
    { 0x0108, 0x210C },
    { 0x0109, 0x61C1 },
    { 0x010A, 0xD54C },
    { 0x010B, 0x0000 },
    { 0x010C, 0x8865 },
    { 0x010D, 0x011A },
    { 0x010E, 0x2040 },
    { 0x010F, 0x3142 },
    { 0x0110, 0x2B14 },
    { 0x0111, 0x0000 },
    { 0x0112, 0x000C },
    { 0x0113, 0x0041 },
    { 0x0114, 0x008D },
    { 0x0115, 0x000D },
    { 0x0116, 0x8180 },
    { 0x0117, 0x280C },
    { 0x0118, 0x418C },
    { 0x0119, 0xD28C },
    { 0x011A, 0x2E02 },
    { 0x011C, 0x8101 },
    { 0x011D, 0x0400 },
    { 0x011E, 0x0780 },
    { 0x011F, 0x3640 },
    { 0x0120, 0x29DC },
    { 0x0121, 0x3190 },
    { 0x0122, 0x0514 },
    { 0x0123, 0x200F },
    { 0x0124, 0x0000 },
    { 0x0125, 0x9400 },
    { 0x0126, 0x12FF },
    { 0x0200, 0x0080 },
    { 0x0400, 0x0080 },
    { 0x0440, 0x0020 },
    { 0x05C0, 0x00FF },
    { 0x05CB, 0x1F0F },
    { 0x05CC, 0x0000 },
    { 0x0600, 0x0F01 },
    { 0x0602, 0x2000 },
    { 0x0603, 0x0000 },
    { 0x0640, 0x00A1 },
    { 0x0641, 0x2040 },
};

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
    , la9310(std::make_shared<LimeSDR_Micro_M4>(streamingPort))
    // , mI2C(i2c_bus)
    , mConfigInProgress(false)
{
    mStreamers.resize(1);
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_MICRO);

    OpStatus status = OpStatus::Success;
    if (!la9310->IsM4CoreProgrammed())
    {
        std::vector<char> firmware;
        const std::string m4firmware_path = "/lib/firmware/"s + limesdrmicro::m4_firmware_name_default;
        status = limesdrmicro::ReadFileIntoVector(m4firmware_path, firmware);
        if (status == OpStatus::Success)
        {
            lime::info("Programming LimeSDR-Micro " + m4firmware_path);
            status = mStreamingPort->LoadArmM4Firmware(firmware.data(), firmware.size());
            if (status != OpStatus::Success)
                lime::error("Failed to program LimeSDR-Micro LA9310 Arm M4 firmware");
        }
        else
            lime::error("M4 firmware file not found: "s + m4firmware_path);

        // also load VSPA just in case it would be needed, it can be reprogrammed later
        // M4 must be successfully programmed to work with VSPA
        if (status == OpStatus::Success && !la9310->vspa.IsFirmwareLoaded())
        {
            std::vector<char> firmware;
            const std::string vspafirmware_path = "/lib/firmware/"s + limesdrmicro::vspa_firmware_name_default;
            OpStatus status = limesdrmicro::ReadFileIntoVector(vspafirmware_path, firmware);
            if (status == OpStatus::Success)
            {
                lime::info("Programming LimeSDR-Micro VSPA " + vspafirmware_path);
                status = la9310->LoadVSPAFirmware(firmware);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                if (status != OpStatus::Success)
                    lime::error("Failed to program LimeSDR-Micro LA9310 VSPA firmware");
            }
            else
                lime::error("VSPA File not found: "s + vspafirmware_path);
        }
    }
    if (status == OpStatus::Success)
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
        soc.antennaRange[TRXDir::Tx]["Band1"s] = { 3.3e9, 3.8e9 };
        soc.antennaRange[TRXDir::Tx]["Band2"s] = { 0.03e9, 1.9e9 };

        soc.samplingRateRange = { 7e6, 160e6, 1 }; // going below 7MHz without decimation introduces noise

        desc.rfSOC.push_back(soc);

        std::unique_ptr<LMS7002M> chip = std::make_unique<LMS7002M>(lmsSPI);

        chip->ModifyRegistersDefaults(limesdrmicro::lms7002defaultsOverrides_LimeSDR_Micro);
        // chip->SetOnCGENChangeCallback(LMS1_UpdateFPGAInterface, this);

        uint32_t boardRefClk = la9310->GetReferenceClock();
        if (boardRefClk > 0)
        {
            lime::info("LimeSDR Micro refclk: %i", boardRefClk);
            refClk = boardRefClk;
        }
        chip->SetReferenceClk_SX(TRXDir::Rx, refClk);
        chip->SetClockFreq(LMS7002M::ClockID::CLK_REFERENCE, refClk);
        mLMSChips.push_back(std::move(chip));
    }

    desc.memoryDevices[ToString(eMemoryDevice::ARM_M4)] = std::make_shared<DataStorage>(this, eMemoryDevice::ARM_M4);
    desc.memoryDevices[ToString(eMemoryDevice::VSPA)] = std::make_shared<DataStorage>(this, eMemoryDevice::VSPA);
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
    std::vector<uint16_t> addrs;
    addrs.reserve(limesdrmicro::lms7002defaultsOverrides_LimeSDR_Micro.size() + 2);
    std::vector<uint16_t> values;
    values.reserve(limesdrmicro::lms7002defaultsOverrides_LimeSDR_Micro.size() + 2);

    addrs.push_back(0x0020);
    values.push_back(0xFFFF); // enable simultaneous A&B write

    for (const auto& val : limesdrmicro::lms7002defaultsOverrides_LimeSDR_Micro)
    {
        addrs.push_back(val.first);
        values.push_back(val.second);
    }
    addrs.push_back(0x0020);
    values.push_back(0xFFFD); // back to A channel only

    OpStatus status = lms.SPI_write_batch(addrs.data(), values.data(), addrs.size(), true);
    return status;
}

static double CalculateCommonSampleRate(const SDRConfig& cfg)
{
    double maxDACrequest = 0;
    double maxADCrequest = 0;
    // find highest requested sampling rate
    for (size_t i = 0; i < cfg.channel.size(); ++i)
    {
        if (cfg.channel[i].rx.enabled)
        {
            double dacRate = cfg.channel[i].rx.sampleRate; // * std::max(1u, uint(cfg.channel[i].rx.oversample));
            if (dacRate > maxDACrequest)
                maxDACrequest = dacRate;
        }
        if (cfg.channel[i].tx.enabled)
        {
            double adcRate = cfg.channel[i].tx.sampleRate; // * std::max(1u, uint(cfg.channel[i].tx.oversample));
            if (adcRate > maxADCrequest)
                maxADCrequest = adcRate;
        }
    }
    return std::max(maxADCrequest, maxDACrequest);
}

static OpStatus SetLA9310SamplingRate(std::shared_ptr<LimeSDR_Micro_M4> la9310, double sampleRate, uint32_t oversample)
{
    constexpr double maxSystemClock = 160e6;
    uint8_t adc_divider_mask = 0;
    uint8_t dac_divider_mask = 0;

    if (sampleRate <= 20e6)
    {
        adc_divider_mask = 0xF;
        dac_divider_mask = 0x1;
        if (oversample > 0)
            oversample = std::min(oversample, 4u);
        else
            oversample = 4;
    }
    else if (sampleRate <= 30e6)
    {
        adc_divider_mask = 0xF;
        dac_divider_mask = 0x1;
        if (oversample > 0)
            oversample = std::min(oversample, 2u);
        else
            oversample = 2;
    }
    else if (sampleRate <= 80e6)
    {
        adc_divider_mask = 0xF;
        dac_divider_mask = 0x1;
        oversample = 1;
    }
    else
    {
        adc_divider_mask = 0;
        dac_divider_mask = 0;
        oversample = 1;
    }

    double systemClock = sampleRate * oversample * ((adc_divider_mask | dac_divider_mask) ? 2 : 1);
    if (systemClock > maxSystemClock)
    {
        return ReportError(OpStatus::OutOfRange, "Requested LA9310 system clock (%f) > %f", systemClock, maxSystemClock);
    }

    if (systemClock <= 0)
        return OpStatus::InvalidValue;

    if (systemClock < 15e6) // even though clock can be lower, digital artifacts start to appear in data
        return ReportError(OpStatus::OutOfRange, "Requested LA9310 system clock (%f) < 15e6", systemClock);

    if (systemClock < 1.25e6) // minimal system clock that can be generated by LMS7002M CGEN
        return ReportError(OpStatus::OutOfRange, "LMS7002M Cannot satisfy requested clock rate");

    OpStatus status = la9310->SetSystemClock(systemClock, adc_divider_mask, dac_divider_mask);
    if (status != OpStatus::Success)
        return ReportError(status, "Failed to set LA9310 system clock");

    la9310->phytimer.SetReferenceClock(systemClock);
    if (la9310->vspa.SetInterpolation(oversample) != OpStatus::Success)
    {
        lime::error("Failed to set interpolation.");
        return OpStatus::Error;
    }
    for (int i = 0; i < 4; ++i)
    {
        const auto vspa_ch = la9310->vspa.api_channel_remap(i);
        if (la9310->vspa.SetDecimation(vspa_ch, oversample) != OpStatus::Success)
        {
            lime::error("Failed to set decimation.");
            return OpStatus::Error;
        }
    }
    return OpStatus::Success;
}

OpStatus LimeSDR_Micro::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    OpStatus status;
    auto& rfsoc = mLMSChips.at(0);

    if (cfg.referenceClockFreq > 0)
    {
        // uint32_t currentRefClk = la9310->GetReferenceClock();
        bool external = cfg.referenceClockSource != 0;
        status = la9310->SetReferenceClock(cfg.referenceClockFreq, external);
        if (status != OpStatus::Success)
            return ReportError(status, "Failed to set reference clock");
    }

    if (!cfg.skipDefaults)
    {
        const bool skipTune = true;
        InitLMS7002M(*rfsoc, skipTune);
    }

    status = LMS7002LOConfigure(*rfsoc, cfg);
    if (status != OpStatus::Success)
        return status;

    for (int i = 0; i < 2; ++i)
    {
        status = LMS7002ChannelConfigure(*rfsoc, cfg.channel[i], i);
        if (status != OpStatus::Success)
            return status;
    }

    for (int c = 0; c < 2; ++c)
    {
        LMSSetPath(TRXDir::Tx, c, cfg.channel[c].tx.path);
        LMSSetPath(TRXDir::Rx, c, cfg.channel[c].rx.path);
    }

    double sampleRate = CalculateCommonSampleRate(cfg);
    if (sampleRate > 0)
    {
        status = SetLA9310SamplingRate(la9310, sampleRate, cfg.channel[0].rx.oversample);
        if (status != OpStatus::Success)
            return status;
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

double LimeSDR_Micro::GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset)
{
    // LMS7002M digital NCO is not usable
    // TODO: implement NCO in VSPA
    return 0;
}

OpStatus LimeSDR_Micro::SetNCOFrequency(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset)
{
    return OpStatus::NotSupported;
}

int LimeSDR_Micro::GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv)
{
    return OpStatus::NotSupported;
}

double LimeSDR_Micro::GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return 0;
}

OpStatus LimeSDR_Micro::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    return SetLA9310SamplingRate(la9310, sampleRate, oversample);
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
    if (trx == TRXDir::Tx)
        return la9310->vspa.GetTxDCCorrector()->SetDCOffset(offset);
    else
        return la9310->vspa.GetRxDCCorrector()->SetDCOffset(offset);
}

complex64f_t LimeSDR_Micro::GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    return complex64f_t();
}

OpStatus LimeSDR_Micro::SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance)
{
    double ratio = balance.real() / (balance.imag() == 0 ? 1 : balance.imag());
    if (trx == TRXDir::Tx)
        return la9310->vspa.GetTxQEC()->SetGainCorrection(ratio);
    else
        return la9310->vspa.GetRxQEC()->SetGainCorrection(ratio);
}

unsigned int LimeSDR_Micro::ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA)
{
    auto& lms = mLMSChips.at(moduleIndex);
    return lms->SPI_read(address);
}

OpStatus LimeSDR_Micro::WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA)
{
    return mLMSChips.at(moduleIndex)->SPI_write(address, value);
}

OpStatus LimeSDR_Micro::Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth)
{
    mLMSChips.at(0)->SetActiveChannel(static_cast<LMS7002M::Channel>((channel % 2) + 1));
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

OpStatus LimeSDR_Micro::LMSSetPath(TRXDir dir, uint8_t chan, uint8_t pathId)
{
    OpStatus status = OpStatus::Success;
    auto& lms = mLMSChips.at(0);
    LMS7002M::ChannelScope scope(lms.get(), chan);

    constexpr uint8_t i2c_expander_address = 0x20;
    constexpr uint8_t gpiob_addr = 0x13;
    uint8_t value = 0;
    I2CRead(0, i2c_expander_address, gpiob_addr, 1, &value, 1);
    if (dir == TRXDir::Tx)
    {
        status = lms->SetBandTRF(pathId);
        if (status != OpStatus::Success)
            return status;

        if (chan != 0)
            return status;

        int ver = GetDescriptor().hardwareVersion == "0" ? 0 : 1;
        if (ver == 0)
        {
            value &= ~(1 << 1); // clear TX_SW
            if (pathId == 1)
                value |= (1 << 1); // set TX_SW, Band1
            else
                value |= (0 << 1); // set TX_SW, Band2
        }
        else
        {
            auto timer = la9310->phytimer.GetTimerControl(15);
            auto triggerValue = pathId == 1 ? PHYTimerControl::TriggerLogic::ForceOne : PHYTimerControl::TriggerLogic::ForceZero;
            timer.TriggerDirectly(triggerValue);
        }
    }
    else
    {
        lime::LMS7002M::PathRFE path{ pathId };
        // first configure chip path or loopback
        status = lms->SetPathRFE(lime::LMS7002M::PathRFE(path));
        if (status != OpStatus::Success)
            return status;

        if (chan != 0)
            return status;

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
    }
    I2CWrite(0, i2c_expander_address, gpiob_addr, 1, &value, 1);
    return status;
}

OpStatus LimeSDR_Micro::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    OpStatus status;

    switch (device)
    {
    case eMemoryDevice::ARM_M4: {
        // Check if firmware is already running
        if (la9310->CheckFirmwareAlive())
        {
            lime::info("LA9310 Firmware is already running, enter reload mode\n");
            status = la9310->EnterFirmwareReloadMode();
            if (status != OpStatus::Success)
                return status;
        }
        else
            lime::info("LA9310 Firmware is not running\n");

        status = la9310->LoadFirmware(std::span<const char, std::dynamic_extent>(data, length));
        if (status != OpStatus::Success)
            return status;

        // Update firmware information after firmware change
        LMS64CProtocol::FirmwareInfo fw{};
        LMS64CProtocol::GetFirmwareInfo(*mSerialPort, fw, 0);
        LMS64CProtocol::FirmwareToDescriptor(fw, mDeviceDescriptor);
        return status;
    }
    case eMemoryDevice::VSPA:
    {
        // Make sure the VCPU is stopped
        status = la9310->vspa.ResetVCPU();
        if (status != OpStatus::Success)
            return status;
        // (Re-)programm the firmware
        auto fwdata = std::span<const char, std::dynamic_extent>(data, length);
        return la9310->LoadVSPAFirmware(fwdata);
    }
    case eMemoryDevice::EEPROM:
        if (length > 65532) // Don't overwrite stored board parameters
            return OpStatus::OutOfRange;
        return la9310->eeprom.Write(0x0, data, length);
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
    return LMSSetPath(trx, channel, path);
}

std::unique_ptr<lime::RFStream> LimeSDR_Micro::StreamCreate(const StreamConfig& config, uint8_t moduleIndex)
{
    if (!la9310->vspa.IsFirmwareLoaded())
    {
        std::vector<char> firmware;
        const std::string vspafirmware_path = "/lib/firmware/"s + limesdrmicro::vspa_firmware_name_default;
        OpStatus status = limesdrmicro::ReadFileIntoVector(vspafirmware_path, firmware);
        if (status != OpStatus::Success)
        {
            lime::error("File not found: "s + vspafirmware_path);
            return nullptr;
        }
        lime::info("Programming LimeSDR-Micro VSPA " + vspafirmware_path);
        status = la9310->LoadVSPAFirmware(firmware);
        if (status != OpStatus::Success)
            return nullptr;
    }
    if (la9310->vspa.Initialize() != OpStatus::Success)
        return nullptr;

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
    const double MCLK1_frequency = cgenFrequency / 4;

    uint8_t adcRate, dacRate;
    la9310->GetADCDACRates(&adcRate, &dacRate);
    const auto vspa_ch = la9310->vspa.api_channel_remap(channel);

    double rate = MCLK1_frequency;
    // ADC, DAC clock dividers by 2
    if (trx == TRXDir::Tx)
        rate /= (1 << dacRate);
    else
        rate /= (1 << ((adcRate >> vspa_ch) & 1));

    if (rf_samplerate)
        *rf_samplerate = rate;

    int dec = la9310->vspa.GetDecimation(vspa_ch);
    if (dec > 0)
    {
        rate /= dec;
    }

    return rate;
}

ICSR* LimeSDR_Micro::getICSR()
{
    return new LA9310_CSR(mStreamingPort);
}

OpStatus LimeSDR_Micro::RunRxTestConfig(OEMTestReporter& reporter,
    TestData::RFData* results,
    const std::string& name,
    int channelIndex,
    double LOFreq,
    int gain,
    int rxPath,
    double expectCh_dBFS)
{
    SDRConfig config;
    config.channel[0].tx.sampleRate = config.channel[0].rx.sampleRate = 40e6;
    config.channel[0].rx.enabled = true;
    config.channel[0].tx.enabled = false;
    // config.channel[0].tx.testSignal = ChannelConfig::Direction::TestSignal{ true, true }; // Test signal: DC
    // config.channel[0].tx.testSignal.dcValue = complex16_t(0x7000, 0x7000);
    // config.channel[0].tx.gain[eGainTypes::PAD] = 52;
    // config.channel[0].tx.gain[eGainTypes::IAMP] = -18;

    const double tx_offset = 2e6;
    config.channel[0].rx.centerFrequency = LOFreq;
    config.channel[0].tx.centerFrequency = LOFreq + tx_offset;
    config.channel[0].rx.path = rxPath;
    config.channel[0].rx.gain[eGainTypes::GENERIC] = gain;

    // If RX H is chosen, use TX 1; else use TX 2
    // config.channel[0].tx.path = rxPath == 1 ? 1 : 2;

    // same config for both channels
    config.channel[1] = config.channel[0];

    bool configPass = false;
    bool chAPass = false;
    bool chBPass = false;

    OpStatus status = Configure(config, 0);
    configPass = status == OpStatus::Success;

    RFTestInput args;
    args.rfTestTolerance_dB = 4;
    args.rfTestTolerance_Hz = 50e3;
    args.sampleRate = config.channel[0].rx.sampleRate;
    args.expectedPeakval_dBFS = expectCh_dBFS;
    args.expectedPeakFrequency = tx_offset;
    args.moduleIndex = 0;

    args.testName = name;
    args.channelIndex = channelIndex;

    RFTestOutput output{};
    if (configPass)
        chAPass = RunRFTest(*this, args, &reporter, &output) == OpStatus::Success;

    results[channelIndex].frequency = output.frequency;
    results[channelIndex].amplitude = output.amplitude_dBFS;
    results[channelIndex].passed = chAPass;

    bool pass = configPass && chAPass; // && chBPass;
    return pass ? OpStatus::Success : OpStatus::Error;
}

OpStatus LimeSDR_Micro::RunTxTestConfig(OEMTestReporter& reporter,
    TestData::RFData* results,
    const std::string& name,
    int channelIndex,
    double LOFreq,
    int gain,
    int txPath,
    double expectedPeakval_dBFS)
{
    OEMTestData test(name);

    reporter.OnStepUpdate(test, name);
    SDRConfig config;
    double sampleRate = 40e6;
    config.channel[0].tx.sampleRate = config.channel[0].rx.sampleRate = sampleRate;
    config.channel[0].rx.enabled = true;
    config.channel[0].tx.enabled = true;
    // config.channel[0].tx.testSignal = ChannelConfig::Direction::TestSignal{ true, true }; // Test signal: DC
    // config.channel[0].tx.testSignal.dcValue = complex16_t(0x7000, 0x7000);
    // config.channel[0].tx.gain[eGainTypes::PAD] = 52;
    // config.channel[0].tx.gain[eGainTypes::IAMP] = -18;

    config.channel[0].rx.centerFrequency = LOFreq;
    config.channel[0].tx.centerFrequency = LOFreq;

    config.channel[0].rx.path = 3;
    config.channel[0].rx.gain[eGainTypes::GENERIC] = 30;

    config.channel[0].tx.path = txPath;
    config.channel[0].tx.gain[eGainTypes::GENERIC] = gain;

    // same config for both channels
    config.channel[1] = config.channel[0];

    config.channel[1].tx.enabled = false;

    bool configPass = false;

    OpStatus status = Configure(config, 0);
    configPass = status == OpStatus::Success;

    status = la9310->vspa.GenerateTxTone(false, 8192);
    if (status != OpStatus::Success)
        printf("Failed to stop Tx tone\n");

    status = la9310->vspa.GenerateTxTone(true, 8192);
    if (status != OpStatus::Success)
        printf("Failed to set Tx tone\n");

    char ctemp[512];
    sprintf(
        ctemp, "Measure Tx signal strength. Expected %0.2f dBm @ %.3f MHz", expectedPeakval_dBFS, (LOFreq + sampleRate / 8) / 1e6);
    reporter.WaitForUserAction(test, ctemp);

    results[txPath - 1].frequency = (LOFreq + sampleRate / 8);
    std::string input = reporter.UserInputValue(test);

    double amplitude = std::stod(input);
    results[txPath - 1].amplitude = amplitude;
    results[txPath - 1].passed = results[channelIndex].amplitude > expectedPeakval_dBFS;

    status = la9310->vspa.GenerateTxTone(false, 8192);
    if (status != OpStatus::Success)
        printf("Failed to stop Tx tone\n");

    return status;
}

OpStatus LimeSDR_Micro::RFTest(OEMTestReporter& reporter, TestData& results)
{
    OEMTestData test("RF");
    reporter.OnStart(test);
    //reporter.OnStepUpdate(test, "Note: The test should be run with loop connected between RF ports");
    reporter.OnStepUpdate(test, "->Configure LMS");

    if (Init() != OpStatus::Success)
    {
        test.passed = false;
        reporter.OnFail(test, "Failed to initialize device");
        return OpStatus::Error;
    }
    reporter.OnStepUpdate(test, "->Init Done");
    std::vector<OpStatus> statuses(3);

    int rxgain = 30;
    reporter.OnStepUpdate(test, "Set signal generator to 1202 MHz, -60 dBm output");
    reporter.WaitForUserAction(test, "Connect Signal generator to Rx A");
    statuses.push_back(RunRxTestConfig(reporter, results.lnah, "LNA_H A", 0, 1200e6, rxgain, 1, -32.0));
    statuses.push_back(RunRxTestConfig(reporter, results.lnal, "LNA_L A", 0, 1200e6, rxgain, 2, -38.0));
    statuses.push_back(RunRxTestConfig(reporter, results.lnaw, "LNA_W A", 0, 1200e6, rxgain, 3, -37.0));

    reporter.WaitForUserAction(test, "Connect Signal generator to Rx B");
    statuses.push_back(RunRxTestConfig(reporter, results.lnah, "LNA_H B", 1, 1200e6, rxgain, 1, -35.0));
    statuses.push_back(RunRxTestConfig(reporter, results.lnal, "LNA_L B", 1, 1200e6, rxgain, 2, -39.0));
    statuses.push_back(RunRxTestConfig(reporter, results.lnaw, "LNA_W B", 1, 1200e6, rxgain, 3, -39.0));

    reporter.WaitForUserAction(test, "Set spectrum analyzer to 2100 MHz");
    int txgain = 52;
    statuses.push_back(RunTxTestConfig(reporter, results.band, "TxA Band1", 0, 2100e6, txgain, 1, -18.0));
    statuses.push_back(RunTxTestConfig(reporter, results.band, "TxA Band2", 0, 2100e6, txgain, 2, -18.0));

    for (OpStatus s : statuses)
    {
        if (s != OpStatus::Success)
        {
            reporter.OnFail(test);
            return OpStatus::Error;
        }
    }
    test.passed = true;
    reporter.OnSuccess(test);
    return OpStatus::Success;
}

OpStatus LimeSDR_Micro::OEMTest(OEMTestReporter* reporter)
{
    TestData results;
    OEMTestData test("LimeSDR-Micro OEM Test");

    uint16_t dac_value = 0xFFFF;
    OpStatus status = la9310->eeprom.Read(0xFFFE, &dac_value, 2);
    if (dac_value == 0xFFFF) // has never been set
    {
        uint16_t default_dac = 0xA1B8;
        status = la9310->eeprom.Write(0xFFFE, &default_dac, 2);
    }

    reporter->OnStart(test);
    bool pass = true;
    const bool rfPassed = RFTest(*reporter, results) == OpStatus::Success;
    pass &= rfPassed;

    reporter->ReportColumn("RF", rfPassed ? "PASS" : "FAIL");
    reporter->ReportColumn("LNA_H A", std::to_string(results.lnah[0].amplitude));
    reporter->ReportColumn("LNA_L A", std::to_string(results.lnal[0].amplitude));
    reporter->ReportColumn("LNA_W A", std::to_string(results.lnaw[0].amplitude));
    reporter->ReportColumn("LNA_H B", std::to_string(results.lnah[1].amplitude));
    reporter->ReportColumn("LNA_L B", std::to_string(results.lnal[1].amplitude));
    reporter->ReportColumn("LNA_W B", std::to_string(results.lnaw[1].amplitude));
    reporter->ReportColumn("Tx Band1", std::to_string(results.band[0].amplitude));
    reporter->ReportColumn("Tx Band2", std::to_string(results.band[1].amplitude));

    if (pass)
    {
        reporter->OnSuccess(test);
        return OpStatus::Success;
    }
    else
    {
        reporter->OnFail(test);
        return OpStatus::Error;
    }
}

LimeSDR_Micro::TestData::TestData()
{
    memset(this, 0, sizeof(TestData));
}

OpStatus LimeSDR_Micro::WriteSerialNumber(uint64_t serialNumber)
{
    return la9310->eeprom.Write(0xFFF0, &serialNumber, 8);
}

} //namespace lime
