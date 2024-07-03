#include "MM_X8.h"

#include <fcntl.h>
#include <sstream>

#include "limesuiteng/Logger.h"
#include "LitePCIe.h"
#include "limesuiteng/LMS7002M.h"
#include "chips/LMS7002M/validation.h"
#include "FPGA_common.h"
#include "DSP/Equalizer/Equalizer.h"

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"
#include "DeviceTreeNode.h"
#include "utilities/toString.h"

#include <cmath>

namespace lime {

using namespace std::literals::string_literals;

static const char DEVICE_NUMBER_SEPARATOR_SYMBOL = '@';
static const char PATH_SEPARATOR_SYMBOL = '/';

static CustomParameter cp_vctcxo_dac = { "VCTCXO DAC (volatile)"s, 0, 0, 65535, false };
static double X8ReferenceClock = 30.72e6;

/// @brief Constructs the LimeSDR_MMX8 object.
///
/// @param spiLMS7002M The communications ports to the LMS7002M chips.
/// @param spiFPGA The communications ports to the device's FPGA chips.
/// @param trxStreams The communications ports to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param adfComms The communications port to the device's ADF4002 chip.
LimeSDR_MMX8::LimeSDR_MMX8(std::vector<std::shared_ptr<IComms>>& spiLMS7002M,
    std::vector<std::shared_ptr<IComms>>& spiFPGA,
    std::vector<std::shared_ptr<LitePCIe>> trxStreams,
    std::shared_ptr<ISerialPort> control,
    std::shared_ptr<ISPI> adfComms)
    : mMainFPGAcomms(spiFPGA[8])
    , mTRXStreamPorts(trxStreams)
    , mADF(std::make_unique<ADF4002>())
{
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes

    SDRDescriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_MMX8);

    // LMS64CProtocol::FirmwareInfo fw;
    // LMS64CProtocol::GetFirmwareInfo(controlPipe, fw);
    // LMS64CProtocol::FirmwareToDescriptor(fw, desc);

    // mFPGA = new lime::FPGA_X3(spiFPGA, SPI_LMS7002M_1);
    // mFPGA->SetConnection(&mFPGAcomms);
    // FPGA::GatewareInfo gw = mFPGA->GetGatewareInfo();
    // FPGA::GatewareToDescriptor(gw, desc);

    desc.socTree = std::make_shared<DeviceTreeNode>("X8"s, eDeviceTreeNodeClass::SDRDevice, this);

    // TODO: read-back board's reference clock
    mADF->Initialize(adfComms, 30.72e6);
    desc.socTree->children.push_back(std::make_shared<DeviceTreeNode>("ADF4002"s, eDeviceTreeNodeClass::ADF4002, mADF.get()));

    mSubDevices.reserve(8);
    desc.spiSlaveIds["FPGA"s] = 0;

    const std::unordered_map<std::string, Region> eepromMap = { { "VCTCXO_DAC"s, { 16, 2 } } };

    desc.memoryDevices[ToString(eMemoryDevice::FPGA_FLASH)] = std::make_shared<DataStorage>(this, eMemoryDevice::FPGA_FLASH);
    desc.memoryDevices[ToString(eMemoryDevice::EEPROM)] = std::make_shared<DataStorage>(this, eMemoryDevice::EEPROM, eepromMap);

    desc.customParameters.push_back(cp_vctcxo_dac);
    for (size_t i = 0; i < 8; ++i)
    {
        std::unique_ptr<LimeSDR_XTRX> xtrx =
            std::make_unique<LimeSDR_XTRX>(spiLMS7002M[i], spiFPGA[i], trxStreams[i], control, X8ReferenceClock);
        const SDRDescriptor& subdeviceDescriptor = xtrx->GetDescriptor();

        for (const auto& soc : subdeviceDescriptor.rfSOC)
        {
            RFSOCDescriptor temp = soc;
            temp.name = soc.name + DEVICE_NUMBER_SEPARATOR_SYMBOL + std::to_string(i + 1);
            desc.rfSOC.push_back(temp);
        }

        for (const auto& slaveId : subdeviceDescriptor.spiSlaveIds)
        {
            const std::string slaveName = slaveId.first + DEVICE_NUMBER_SEPARATOR_SYMBOL + std::to_string(i + 1);
            desc.spiSlaveIds[slaveName] = (i + 1) << 8 | slaveId.second;
            chipSelectToDevice[desc.spiSlaveIds[slaveName]] = xtrx.get();
        }

        for (const auto& memoryDevice : subdeviceDescriptor.memoryDevices)
        {
            const std::string indexName = subdeviceDescriptor.name + DEVICE_NUMBER_SEPARATOR_SYMBOL + std::to_string(i + 1) +
                                          PATH_SEPARATOR_SYMBOL + memoryDevice.first;

            desc.memoryDevices[indexName] = memoryDevice.second;
        }

        for (const auto& customParameter : subdeviceDescriptor.customParameters)
        {
            CustomParameter parameter = customParameter;
            parameter.id |= (i + 1) << 8;
            parameter.name = customParameter.name + DEVICE_NUMBER_SEPARATOR_SYMBOL + std::to_string(i + 1);
            desc.customParameters.push_back(parameter);
            customParameterToDevice[parameter.id] = xtrx.get();
        }

        mSubDevices.push_back(std::move(xtrx));

        const std::string treeName = subdeviceDescriptor.socTree->name + "#"s + std::to_string(i + 1);
        subdeviceDescriptor.socTree->name = treeName;
        desc.socTree->children.push_back(subdeviceDescriptor.socTree);
    }
}

LimeSDR_MMX8::~LimeSDR_MMX8()
{
}

const SDRDescriptor& LimeSDR_MMX8::GetDescriptor() const
{
    return mDeviceDescriptor;
}

OpStatus LimeSDR_MMX8::Configure(const SDRConfig& cfg, uint8_t socIndex)
{
    return mSubDevices[socIndex]->Configure(cfg, 0);
}

OpStatus LimeSDR_MMX8::Init()
{
    OpStatus status = OpStatus::Success;
    for (size_t i = 0; i < mSubDevices.size(); ++i)
    {
        // TODO: check if the XTRX board slot is populated
        status = mSubDevices[i]->Init();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus LimeSDR_MMX8::Reset()
{
    OpStatus status = OpStatus::Success;
    for (uint32_t i = 0; i < mSubDevices.size(); ++i)
    {
        status = mSubDevices[i]->Reset();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus LimeSDR_MMX8::GetGPSLock(GPS_Lock* status)
{
    // TODO: implement
    return OpStatus::NotImplemented;
}

// TODO: clean up all the functions to use the exact same device selection code (maybe even extract it out into a function)
double LimeSDR_MMX8::GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetFrequency(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetFrequency(0, trx, channel, frequency);
}

double LimeSDR_MMX8::GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetNCOFrequency(0, trx, channel, index, phaseOffset);
}

OpStatus LimeSDR_MMX8::SetNCOFrequency(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetNCOFrequency(0, trx, channel, index, frequency, phaseOffset);
}

int LimeSDR_MMX8::GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetNCOIndex(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetNCOIndex(0, trx, channel, index, downconv);
}

double LimeSDR_MMX8::GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetNCOOffset(0, trx, channel);
}

double LimeSDR_MMX8::GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetSampleRate(0, trx, channel, rf_samplerate);
}

OpStatus LimeSDR_MMX8::SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetSampleRate(0, trx, channel, sampleRate, oversample);
}

double LimeSDR_MMX8::GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetLowPassFilter(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetLowPassFilter(0, trx, channel, lpf);
}

uint8_t LimeSDR_MMX8::GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetAntenna(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetAntenna(0, trx, channel, path);
}

double LimeSDR_MMX8::GetClockFreq(uint8_t clk_id, uint8_t channel)
{
    return mSubDevices[channel / 2]->GetClockFreq(clk_id, channel & 0x1);
}

OpStatus LimeSDR_MMX8::SetClockFreq(uint8_t clk_id, double freq, uint8_t channel)
{
    return mSubDevices[channel / 2]->SetClockFreq(clk_id, freq, channel & 1);
}

OpStatus LimeSDR_MMX8::SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value)
{
    auto& device = mSubDevices.at(moduleIndex);
    return device->SetGain(0, direction, channel, gain, value);
}

OpStatus LimeSDR_MMX8::GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value)
{
    auto& device = mSubDevices.at(moduleIndex);
    return device->GetGain(0, direction, channel, gain, value);
}

bool LimeSDR_MMX8::GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetDCOffsetMode(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetDCOffsetMode(0, trx, channel, isAutomatic);
}

complex64f_t LimeSDR_MMX8::GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetDCOffset(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetDCOffset(0, trx, channel, offset);
}

complex64f_t LimeSDR_MMX8::GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetIQBalance(0, trx, channel);
}

OpStatus LimeSDR_MMX8::SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetIQBalance(0, trx, channel, balance);
}

bool LimeSDR_MMX8::GetCGENLocked(uint8_t moduleIndex)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetCGENLocked(0);
}

double LimeSDR_MMX8::GetTemperature(uint8_t moduleIndex)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetTemperature(0);
}

bool LimeSDR_MMX8::GetSXLocked(uint8_t moduleIndex, TRXDir trx)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetSXLocked(0, trx);
}

unsigned int LimeSDR_MMX8::ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->ReadRegister(0, address, useFPGA);
}

OpStatus LimeSDR_MMX8::WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->WriteRegister(0, address, value, useFPGA);
}

OpStatus LimeSDR_MMX8::LoadConfig(uint8_t moduleIndex, const std::string& filename)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->LoadConfig(0, filename);
}

OpStatus LimeSDR_MMX8::SaveConfig(uint8_t moduleIndex, const std::string& filename)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SaveConfig(0, filename);
}

uint16_t LimeSDR_MMX8::GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetParameter(0, channel, parameterKey);
}

OpStatus LimeSDR_MMX8::SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetParameter(0, channel, parameterKey, value);
}

uint16_t LimeSDR_MMX8::GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetParameter(0, channel, address, msb, lsb);
}

OpStatus LimeSDR_MMX8::SetParameter(
    uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetParameter(0, channel, address, msb, lsb, value);
}

OpStatus LimeSDR_MMX8::Synchronize(bool toChip)
{
    OpStatus status = OpStatus::Success;
    for (auto& d : mSubDevices)
    {
        status = d->Synchronize(toChip);
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

void LimeSDR_MMX8::EnableCache(bool enable)
{
    for (auto& d : mSubDevices)
        d->EnableCache(enable);
}

OpStatus LimeSDR_MMX8::EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->EnableChannel(0, trx, channel, enable);
}

OpStatus LimeSDR_MMX8::Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->Calibrate(0, trx, channel, bandwidth);
}

OpStatus LimeSDR_MMX8::ConfigureGFIR(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->ConfigureGFIR(0, trx, channel, settings);
}

std::vector<double> LimeSDR_MMX8::GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetGFIRCoefficients(0, trx, channel, gfirID);
}

OpStatus LimeSDR_MMX8::SetGFIRCoefficients(
    uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetGFIRCoefficients(0, trx, channel, gfirID, coefficients);
}

OpStatus LimeSDR_MMX8::SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetGFIR(0, trx, channel, gfirID, enabled);
}

uint64_t LimeSDR_MMX8::GetHardwareTimestamp(uint8_t moduleIndex)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetHardwareTimestamp(0);
}

OpStatus LimeSDR_MMX8::SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetHardwareTimestamp(0, now);
}

OpStatus LimeSDR_MMX8::SetTestSignal(uint8_t moduleIndex,
    TRXDir direction,
    uint8_t channel,
    ChannelConfig::Direction::TestSignal signalConfiguration,
    int16_t dc_i,
    int16_t dc_q)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->SetTestSignal(0, direction, channel, signalConfiguration, dc_i, dc_q);
}

ChannelConfig::Direction::TestSignal LimeSDR_MMX8::GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel)
{
    if (moduleIndex >= 8)
    {
        moduleIndex = 0;
    }

    return mSubDevices[moduleIndex]->GetTestSignal(0, direction, channel);
}

OpStatus LimeSDR_MMX8::StreamSetup(const StreamConfig& config, uint8_t moduleIndex)
{
    OpStatus ret = mSubDevices[moduleIndex]->StreamSetup(config, 0);
    if (ret != OpStatus::Success)
        return ret;
    // X8 board has two stage stream start.
    // start stream for expected subdevices, they will wait for secondary enable from main fpga register
    mSubDevices[moduleIndex]->StreamStart(0);
    return ret;
}

void LimeSDR_MMX8::StreamStart(uint8_t moduleIndex)
{
    std::vector<uint8_t> index;
    index.push_back(moduleIndex);
    StreamStart(index);
}

void LimeSDR_MMX8::StreamStart(const std::vector<uint8_t> moduleIndexes)
{
    FPGA tempFPGA(mMainFPGAcomms, nullptr);
    int interface_ctrl_000A = tempFPGA.ReadRegister(0x000A);
    uint16_t mask = 0;
    for (uint8_t moduleIndex : moduleIndexes)
    {
        mask |= (1 << (2 * moduleIndex));
    }
    tempFPGA.WriteRegister(0x000A, interface_ctrl_000A & ~mask);
    tempFPGA.WriteRegister(0x000A, interface_ctrl_000A | mask);
}

void LimeSDR_MMX8::StreamStop(uint8_t moduleIndex)
{
    std::vector<uint8_t> index;
    index.push_back(moduleIndex);
    StreamStop(index);
}

void LimeSDR_MMX8::StreamStop(const std::vector<uint8_t> moduleIndexes)
{
    FPGA tempFPGA(mMainFPGAcomms, nullptr);
    int interface_ctrl_000A = tempFPGA.ReadRegister(0x000A);
    uint16_t mask = 0;
    for (uint8_t moduleIndex : moduleIndexes)
    {
        mask |= (1 << (2 * moduleIndex));
    }
    tempFPGA.WriteRegister(0x000A, interface_ctrl_000A & ~mask);
    for (uint8_t moduleIndex : moduleIndexes)
        mSubDevices[moduleIndex]->StreamStop(0);
}

void LimeSDR_MMX8::StreamDestroy(uint8_t moduleIndex)
{
    mSubDevices.at(moduleIndex)->StreamDestroy(0);
}

uint32_t LimeSDR_MMX8::StreamRx(uint8_t moduleIndex, lime::complex32f_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamRx(0, dest, count, meta);
}

uint32_t LimeSDR_MMX8::StreamRx(uint8_t moduleIndex, lime::complex16_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamRx(0, dest, count, meta);
}

uint32_t LimeSDR_MMX8::StreamRx(uint8_t moduleIndex, lime::complex12_t* const* dest, uint32_t count, StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamRx(0, dest, count, meta);
}

uint32_t LimeSDR_MMX8::StreamTx(
    uint8_t moduleIndex, const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamTx(0, samples, count, meta);
}

uint32_t LimeSDR_MMX8::StreamTx(
    uint8_t moduleIndex, const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamTx(0, samples, count, meta);
}

uint32_t LimeSDR_MMX8::StreamTx(
    uint8_t moduleIndex, const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return mSubDevices[moduleIndex]->StreamTx(0, samples, count, meta);
}

void LimeSDR_MMX8::StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx)
{
    mSubDevices[moduleIndex]->StreamStatus(0, rx, tx);
}

OpStatus LimeSDR_MMX8::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    if (chipSelect == 0)
    {
        return mMainFPGAcomms->SPI(MOSI, MISO, count);
    }

    SDRDevice* dev = chipSelectToDevice.at(chipSelect);
    if (!dev)
    {
        throw std::logic_error("invalid SPI chip select"s);
    }

    uint32_t subSelect = chipSelect & 0xFF;
    return dev->SPI(subSelect, MOSI, MISO, count);
}

void LimeSDR_MMX8::SetMessageLogCallback(LogCallbackType callback)
{
    for (size_t i = 0; i < mSubDevices.size(); ++i)
        mSubDevices[i]->SetMessageLogCallback(callback);
}

void* LimeSDR_MMX8::GetInternalChip(uint32_t index)
{
    return mSubDevices[index % mSubDevices.size()]->GetInternalChip(0);
}

OpStatus LimeSDR_MMX8::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    OpStatus status = OpStatus::Success;

    for (const CustomParameterIO& param : parameters)
    {
        int subModuleIndex = (param.id >> 8) - 1;
        int id = param.id & 0xFF;

        std::vector<CustomParameterIO> parameter{ { id, param.value, param.units } };

        if (subModuleIndex >= 0)
            status = mSubDevices[subModuleIndex]->CustomParameterWrite(parameter);
        else
            status = mMainFPGAcomms->CustomParameterWrite(parameter);

        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus LimeSDR_MMX8::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    OpStatus status = OpStatus::Success;

    for (CustomParameterIO& param : parameters)
    {
        int subModuleIndex = (param.id >> 8) - 1;
        int id = param.id & 0xFF;

        std::vector<CustomParameterIO> parameter{ { id, param.value, param.units } };

        if (subModuleIndex >= 0)
            status = mSubDevices[subModuleIndex]->CustomParameterRead(parameter);
        else
            status = mMainFPGAcomms->CustomParameterRead(parameter);

        if (status != OpStatus::Success)
            return status;

        param.value = parameter[0].value;
        param.units = parameter[0].units;
    }

    return status;
}

OpStatus LimeSDR_MMX8::UploadMemory(
    eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback)
{
    if (device == eMemoryDevice::FPGA_FLASH && moduleIndex == 0)
    {
        int progMode = 1;
        LMS64CProtocol::ProgramWriteTarget target;
        target = LMS64CProtocol::ProgramWriteTarget::FPGA;
        return mMainFPGAcomms->ProgramWrite(data, length, progMode, static_cast<int>(target), callback);
    }

    auto& dev = mSubDevices.at(moduleIndex);
    if (!dev)
        return ReportError(OpStatus::InvalidValue, "Invalid id select"s);

    return dev->UploadMemory(device, 0, data, length, callback);
}

OpStatus LimeSDR_MMX8::MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data)
{
    if (storage == nullptr)
        return ReportError(OpStatus::InvalidValue, "Invalid storage"s);

    if (storage->ownerDevice == this)
        return mMainFPGAcomms->MemoryWrite(region.address, data, region.size);

    SDRDevice* dev = storage->ownerDevice;
    if (dev == nullptr)
        return ReportError(OpStatus::InvalidValue, "Storage has no owner"s);

    return dev->MemoryWrite(storage, region, data);
}

OpStatus LimeSDR_MMX8::MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data)
{
    if (storage == nullptr)
        return ReportError(OpStatus::InvalidValue, "Invalid storage"s);

    if (storage->ownerDevice == this)
        return mMainFPGAcomms->MemoryRead(region.address, data, region.size);

    SDRDevice* dev = storage->ownerDevice;
    if (dev == nullptr)
        return ReportError(OpStatus::InvalidValue, "Storage has no owner"s);

    return dev->MemoryRead(storage, region, data);
}

OpStatus LimeSDR_MMX8::UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count)
{
    return mSubDevices[moduleIndex]->UploadTxWaveform(config, 0, samples, count);
}

} //namespace lime
