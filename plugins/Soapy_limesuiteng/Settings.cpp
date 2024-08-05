/**
@file   Settings.cpp
@brief  Soapy SDR + IConnection config settings.
@author Lime Microsystems (www.limemicro.com)
*/

#include "Soapy_limesuiteng.h"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDescriptor.h"
#include "utilities/toString.h"

using namespace lime;

constexpr uint8_t socIndex = 0;

/*******************************************************************
 * Constructor/destructor
 ******************************************************************/
Soapy_limesuiteng::Soapy_limesuiteng(const DeviceHandle& handle, const SoapySDR::Kwargs& args)
    : isStreamRunning(false)
    , sampleRate{ 0.0, 0.0 }
    , oversampling(0) // Auto
{
    SoapySDR::log(SOAPY_SDR_INFO, "Make connection: '" + handle.ToString() + "'");
    //NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
    sdrDevice = DeviceRegistry::makeDevice(handle);

    if (sdrDevice == nullptr)
    {
        throw std::runtime_error("Failed to make connection with '" + handle.Serialize() + "'");
    }

    const auto& descriptor = sdrDevice->GetDescriptor();
    const auto channelCount = descriptor.rfSOC.at(socIndex).channelCount;

    // Quick summary
    SoapySDR::log(SOAPY_SDR_INFO, "Device name: " + descriptor.name);
    SoapySDR::logf(SOAPY_SDR_INFO, "Reference: %g MHz", sdrDevice->GetClockFreq(0, 0) / 1e6);

    sdrDevice->Init();

    // Enable all channels
    for (uint8_t channel = 0; channel < channelCount; channel++)
    {
        sdrDevice->EnableChannel(socIndex, TRXDir::Rx, channel, true);
        sdrDevice->EnableChannel(socIndex, TRXDir::Tx, channel, true);
    }

    // Disable use of value cache automatically
    // Or specify args[enableCache] == 1 to enable
    bool cacheEnable = false;
    if (args.count("cacheCalibrations"))
    {
        SoapySDR::log(SOAPY_SDR_INFO, "'cacheCalibrations' setting is deprecated use 'enableCache' instead");
        if (std::stoi(args.at("cacheCalibrations")))
        {
            cacheEnable = true;
        }
    }
    else
    {
        cacheEnable = args.count("enableCache") && std::stoi(args.at("enableCache")) != 0;
    }

    SoapySDR::logf(SOAPY_SDR_INFO, "LMS7002M register cache: %s", cacheEnable ? "Enabled" : "Disabled");
    sdrDevice->EnableCache(cacheEnable);

    settingsCache.at(SOAPY_SDR_RX).resize(channelCount);
    settingsCache.at(SOAPY_SDR_TX).resize(channelCount);
}

Soapy_limesuiteng::~Soapy_limesuiteng(void)
{
    // Power down all channels
    for (uint8_t channel = 0; channel < sdrDevice->GetDescriptor().rfSOC.at(socIndex).channelCount; channel++)
    {
        sdrDevice->EnableChannel(socIndex, TRXDir::Rx, channel, false);
        sdrDevice->EnableChannel(socIndex, TRXDir::Tx, channel, false);
    }

    DeviceRegistry::freeDevice(sdrDevice);
}

/*******************************************************************
 * Identification API
 ******************************************************************/
std::string Soapy_limesuiteng::getDriverKey(void) const
{
    return "limesuiteng";
}

std::string Soapy_limesuiteng::getHardwareKey(void) const
{
    return sdrDevice->GetDescriptor().name;
}

SoapySDR::Kwargs Soapy_limesuiteng::getHardwareInfo(void) const
{
    const auto& descriptor = sdrDevice->GetDescriptor();

    SoapySDR::Kwargs info;
    if (descriptor.expansionName != "UNSUPPORTED")
    {
        info["expansionName"] = descriptor.expansionName;
    }

    info["firmwareVersion"] = descriptor.firmwareVersion;
    info["hardwareVersion"] = descriptor.hardwareVersion;
    info["protocolVersion"] = descriptor.protocolVersion;
    info["gatewareVersion"] = descriptor.gatewareVersion;

    if (descriptor.serialNumber != -1UL)
    {
        std::stringstream ss;
        ss << "0x" << std::hex << descriptor.serialNumber;
        info["boardSerialNumber"] = ss.str();
    }

    return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t Soapy_limesuiteng::getNumChannels([[maybe_unused]] const int direction) const
{
    return sdrDevice->GetDescriptor().rfSOC.at(socIndex).channelCount;
}

bool Soapy_limesuiteng::getFullDuplex([[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    return true;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> Soapy_limesuiteng::listAntennas(const int direction, [[maybe_unused]] const size_t channel) const
{
    return sdrDevice->GetDescriptor().rfSOC.at(socIndex).pathNames.at(direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx);
}

void Soapy_limesuiteng::setAntenna(const int direction, const size_t channel, const std::string& name)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Soapy_limesuiteng::setAntenna(%s, %ld, %s)", ToString(dir).c_str(), channel, name.c_str());

    std::vector<std::string> nameList = sdrDevice->GetDescriptor().rfSOC.at(socIndex).pathNames.at(dir);

    for (std::size_t path = 0; path < nameList.size(); path++)
    {
        if (nameList[path] == name)
        {
            sdrDevice->SetAntenna(socIndex, dir, channel, path);

            // TODO:
            // sdrDevice->Calibrate(0, dir, channel, sdrDevice->GetFrequency(0, dir, channel));
            return;
        }
    }

    throw std::runtime_error("Soapy_limesuiteng::setAntenna(TX, " + name + ") - unknown antenna name");
}

std::string Soapy_limesuiteng::getAntenna(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;

    uint8_t path = sdrDevice->GetAntenna(socIndex, dir, channel);

    const auto& descriptor = sdrDevice->GetDescriptor().rfSOC.at(socIndex);
    const std::vector<std::string>& nameList = descriptor.pathNames.at(dir);

    return path < nameList.size() ? nameList.at(path) : "";
}

/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

bool Soapy_limesuiteng::hasDCOffsetMode(const int direction, [[maybe_unused]] const size_t channel) const
{
    return (direction == SOAPY_SDR_RX);
}

void Soapy_limesuiteng::setDCOffsetMode(const int direction, const size_t channel, const bool automatic)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    sdrDevice->SetDCOffsetMode(socIndex, direction == SOAPY_SDR_RX ? ::TRXDir::Rx : TRXDir::Tx, channel, automatic);
}

bool Soapy_limesuiteng::getDCOffsetMode(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return sdrDevice->GetDCOffsetMode(socIndex, direction == SOAPY_SDR_RX ? ::TRXDir::Rx : TRXDir::Tx, channel);
}

bool Soapy_limesuiteng::hasDCOffset([[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    return true;
}

void Soapy_limesuiteng::setDCOffset(const int direction, const size_t channel, const std::complex<double>& offset)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;
    complex64f_t complex{ offset.real(), offset.imag() };
    sdrDevice->SetDCOffset(socIndex, lmsDir, channel, complex);
}

std::complex<double> Soapy_limesuiteng::getDCOffset(const int direction, const size_t channel) const
{
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto complex = sdrDevice->GetDCOffset(socIndex, lmsDir, channel);
    return { complex.real(), complex.imag() };
}

bool Soapy_limesuiteng::hasIQBalance([[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    return true;
}

void Soapy_limesuiteng::setIQBalance(const int direction, const size_t channel, const std::complex<double>& balance)
{
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    complex64f_t complex{ balance.real(), balance.imag() };
    sdrDevice->SetIQBalance(socIndex, lmsDir, channel, complex);
}

std::complex<double> Soapy_limesuiteng::getIQBalance(const int direction, const size_t channel) const
{
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto complex = sdrDevice->GetIQBalance(socIndex, lmsDir, channel);
    return { complex.real(), complex.imag() };
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> Soapy_limesuiteng::listGains(const int direction, [[maybe_unused]] const size_t channel) const
{
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;
    const auto& gainEnums = sdrDevice->GetDescriptor().rfSOC.at(socIndex).gains.at(dir);
    std::vector<std::string> gains;

    for (const auto& gain : gainEnums)
    {
        gains.push_back(ToString(gain));
    }

    return gains;
}

void Soapy_limesuiteng::setGain(const int direction, const size_t channel, const double value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;
    SoapySDR::logf(SOAPY_SDR_DEBUG, "Soapy_limesuiteng::setGain(%s, %ld, %g dB)", ToString(dir).c_str(), channel, value);

    sdrDevice->SetGain(socIndex, dir, channel, eGainTypes::UNKNOWN, value);

    SoapySDR::logf(SOAPY_SDR_DEBUG, "Actual %s[%ld] gain %g dB", ToString(dir).c_str(), channel, getGain(direction, channel));
}

double Soapy_limesuiteng::getGain(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;

    double gain = 0;
    OpStatus returnValue = sdrDevice->GetGain(socIndex, dir, channel, eGainTypes::UNKNOWN, gain);
    if (returnValue != OpStatus::Success)
    {
        throw std::runtime_error(
            "Soapy_limesuiteng::getGain(" + ToString(dir) + ", " + std::to_string(channel) + ") - failed to get gain");
    }

    return gain;
}

void Soapy_limesuiteng::setGain(const int direction, const size_t channel, const std::string& name, const double value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;
    SoapySDR::logf(
        SOAPY_SDR_DEBUG, "Soapy_limesuiteng::setGain(%s, %ld, %s, %g dB)", ToString(dir).c_str(), channel, name.c_str(), value);

    eGainTypes gainType = ToEnumClass<eGainTypes>(name);

    sdrDevice->SetGain(socIndex, dir, channel, gainType, value);

    SoapySDR::logf(SOAPY_SDR_DEBUG,
        "Actual %s%s[%ld] gain %g dB",
        ToString(dir).c_str(),
        name.c_str(),
        channel,
        getGain(direction, channel, name));
}

double Soapy_limesuiteng::getGain(const int direction, const size_t channel, const std::string& name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;
    eGainTypes gainType = ToEnumClass<eGainTypes>(name);

    double gain = 0;
    OpStatus returnValue = sdrDevice->GetGain(socIndex, dir, channel, gainType, gain);
    if (returnValue != OpStatus::Success)
    {
        throw std::runtime_error(
            "Soapy_limesuiteng::getGain(" + ToString(dir) + ", " + std::to_string(channel) + ") - failed to get gain");
    }

    return gain;
}

SoapySDR::Range Soapy_limesuiteng::getGainRange(const int direction, [[maybe_unused]] const size_t channel) const
{
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;

    auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).gainRange.at(dir).at(eGainTypes::UNKNOWN);

    return SoapySDR::Range(range.min, range.max, range.step);
}

SoapySDR::Range Soapy_limesuiteng::getGainRange(
    const int direction, [[maybe_unused]] const size_t channel, const std::string& name) const
{
    TRXDir dir = direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx;
    eGainTypes gainType = ToEnumClass<eGainTypes>(name);

    auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).gainRange.at(dir).at(gainType);

    return SoapySDR::Range(range.min, range.max, range.step);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/
SoapySDR::ArgInfoList Soapy_limesuiteng::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    return SoapySDR::Device::getFrequencyArgsInfo(direction, channel);
}

void Soapy_limesuiteng::setFrequency(int direction, size_t channel, double frequency, [[maybe_unused]] const SoapySDR::Kwargs& args)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;

    OpStatus status = sdrDevice->SetFrequency(socIndex, dir, channel, frequency);
    if (status != OpStatus::Success)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR,
            "setFrequency(%s, %ld, %g MHz) Failed with message %s",
            ToString(dir).c_str(),
            channel,
            frequency / 1e6,
            lime::GetLastErrorMessageCString());
        throw std::runtime_error("Soapy_limesuiteng::setFrequency() failed");
    }
}

void Soapy_limesuiteng::setFrequency(const int direction,
    const size_t channel,
    const std::string& name,
    const double frequency,
    [[maybe_unused]] const SoapySDR::Kwargs& args)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;
    SoapySDR::logf(SOAPY_SDR_DEBUG,
        "Soapy_limesuiteng::setFrequency(%s, %ld, %s, %g MHz)",
        ToString(dir).c_str(),
        channel,
        name.c_str(),
        frequency / 1e6);
    if (name == "RF")
    {
        OpStatus status = sdrDevice->SetFrequency(socIndex, dir, channel, frequency);
        if (status != OpStatus::Success)
        {
            SoapySDR::logf(
                SOAPY_SDR_ERROR, "setFrequency(%s, %ld, RF, %g MHz) Failed", ToString(dir).c_str(), channel, frequency / 1e6);
            throw std::runtime_error("Soapy_limesuiteng::setFrequency(RF) failed");
        }
    }

    if (name == "BB")
    {
        sdrDevice->SetNCOFrequency(socIndex, dir, channel, 0, dir == TRXDir::Tx ? frequency : -frequency);
        return;
    }

    throw std::runtime_error("Soapy_limesuiteng::setFrequency(" + name + ") unknown name");
}

double Soapy_limesuiteng::getFrequency(const int direction, const size_t channel, const std::string& name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const TRXDir dir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;
    if (name == "RF")
        return sdrDevice->GetFrequency(socIndex, dir, channel);

    if (name == "BB")
    {
        double phaseOffset = 0.0;
        return sdrDevice->GetNCOFrequency(socIndex, dir, channel, 0, phaseOffset);
    }

    throw std::runtime_error("Soapy_limesuiteng::getFrequency(" + name + ") unknown name");
}

double Soapy_limesuiteng::getFrequency(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    return sdrDevice->GetFrequency(socIndex, direction == SOAPY_SDR_RX ? TRXDir::Rx : TRXDir::Tx, channel);
}

std::vector<std::string> Soapy_limesuiteng::listFrequencies(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    std::vector<std::string> opts;
    opts.push_back("RF");
    opts.push_back("BB");
    return opts;
}

SoapySDR::RangeList Soapy_limesuiteng::getFrequencyRange(const int direction, const size_t channel, const std::string& name) const
{
    SoapySDR::RangeList ranges;
    if (name == "RF")
    {
        auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).frequencyRange;
        ranges.push_back(SoapySDR::Range(range.min, range.max, range.step));
    }
    if (name == "BB")
    {
        std::unique_lock<std::recursive_mutex> lock(_accessMutex);
        const auto lmsDir = (direction == SOAPY_SDR_TX) ? 5 : 4;
        const double dspRate = sdrDevice->GetClockFreq(lmsDir, channel);
        ranges.push_back(SoapySDR::Range(-dspRate / 2, dspRate / 2));
    }
    return ranges;
}

SoapySDR::RangeList Soapy_limesuiteng::getFrequencyRange(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    SoapySDR::RangeList ranges;
    auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).frequencyRange;
    ranges.push_back(SoapySDR::Range(range.min, range.max, range.step));
    return ranges;
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void Soapy_limesuiteng::setSampleRate(const int direction, const size_t channel, const double rate)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;

    if (isStreamRunning)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR,
            "setSampleRate(%s, %ld, %g MHz) setting the sample rate while the stream is running is not allowed.",
            ToString(dir).c_str(),
            channel,
            rate / 1e6);
        throw std::runtime_error(
            "Soapy_limesuiteng::setSampleRate(): setting the sample rate while the stream is running is not allowed.");
    }

    SoapySDR::logf(SOAPY_SDR_DEBUG, "setSampleRate(%s, %ld, %g MHz)", ToString(dir).c_str(), channel, rate / 1e6);

    OpStatus status = sdrDevice->SetSampleRate(socIndex, dir, channel, rate, oversampling);
    if (status != OpStatus::Success)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "setSampleRate(%s, %ld, %g MHz) Failed", ToString(dir).c_str(), channel, rate / 1e6);
        throw std::runtime_error("Soapy_limesuiteng::setSampleRate() failed with message " + lime::GetLastErrorMessage());
    }

    sampleRate[static_cast<bool>(direction)] = rate;
    return;
}

double Soapy_limesuiteng::getSampleRate(const int direction, [[maybe_unused]] const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    return sdrDevice->GetSampleRate(socIndex, direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx, 0);
}

SoapySDR::RangeList Soapy_limesuiteng::getSampleRateRange(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).samplingRateRange;

    return { SoapySDR::Range(range.min, range.max, range.step) };
}

/*******************************************************************
 * Bandwidth API
 ******************************************************************/

void Soapy_limesuiteng::setBandwidth(const int direction, const size_t channel, const double bw)
{
    if (bw == 0.0)
    {
        return; // Special ignore value
    }

    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;

    SoapySDR::logf(SOAPY_SDR_DEBUG, "Soapy_limesuiteng::setBandwidth(%s, %ld, %g MHz)", ToString(dir).c_str(), channel, bw / 1e6);

    OpStatus status = sdrDevice->SetLowPassFilter(socIndex, dir, channel, bw);
    if (status != OpStatus::Success)
    {
        SoapySDR::logf(
            SOAPY_SDR_ERROR, "Soapy_limesuiteng::setBandwidth(%s, %ld, %g MHz) Failed", ToString(dir).c_str(), channel, bw / 1e6);
        throw std::runtime_error("setBandwidth() failed");
    }
}

double Soapy_limesuiteng::getBandwidth(const int direction, const size_t channel) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    return sdrDevice->GetLowPassFilter(socIndex, direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx, channel);
}

SoapySDR::RangeList Soapy_limesuiteng::getBandwidthRange(const int direction, [[maybe_unused]] const size_t channel) const
{
    SoapySDR::RangeList bws;

    TRXDir dir = direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx;
    auto range = sdrDevice->GetDescriptor().rfSOC.at(socIndex).lowPassFilterRange.at(dir);
    bws.push_back(SoapySDR::Range(range.min, range.max));

    return bws;
}

/*******************************************************************
 * Clocking API
 ******************************************************************/

double Soapy_limesuiteng::getMasterClockRate(void) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return sdrDevice->GetClockFreq(3, 0);
}

/*******************************************************************
 * Time API
 ******************************************************************/

bool Soapy_limesuiteng::hasHardwareTime(const std::string& what) const
{
    // Assume hardware time when no argument is specified.
    // Some boards may not ever support hw time, so TODO.
    return what.empty();
}

long long Soapy_limesuiteng::getHardwareTime(const std::string& what) const
{
    if (!what.empty())
    {
        throw std::invalid_argument("Soapy_limesuiteng::getHardwareTime(" + what + ") unknown argument");
    }

    if (sampleRate[SOAPY_SDR_RX] == 0)
    {
        throw std::runtime_error("Soapy_limesuiteng::getHardwareTime() sample rate unset");
    }
    auto ticks = sdrDevice->GetHardwareTimestamp(socIndex);
    return SoapySDR::ticksToTimeNs(ticks, sampleRate[SOAPY_SDR_RX]);
}

void Soapy_limesuiteng::setHardwareTime(const long long timeNs, const std::string& what)
{
    if (!what.empty())
    {
        throw std::invalid_argument("Soapy_limesuiteng::setHardwareTime(" + what + ") unknown argument");
    }

    if (sampleRate[SOAPY_SDR_RX] == 0)
    {
        throw std::runtime_error("Soapy_limesuiteng::setHardwareTime() sample rate unset");
    }
    auto ticks = SoapySDR::timeNsToTicks(timeNs, sampleRate[SOAPY_SDR_RX]);
    sdrDevice->SetHardwareTimestamp(socIndex, ticks);
}

/*******************************************************************
 * Sensor API
 ******************************************************************/

std::vector<std::string> Soapy_limesuiteng::listSensors(void) const
{
    // TODO: update
    std::vector<std::string> sensors;
    sensors.push_back("clock_locked");
    sensors.push_back("lms7_temp");
    return sensors;
}

SoapySDR::ArgInfo Soapy_limesuiteng::getSensorInfo(const std::string& name) const
{
    SoapySDR::ArgInfo info;
    if (name == "clock_locked")
    {
        info.key = "clock_locked";
        info.name = "Clock Locked";
        info.type = SoapySDR::ArgInfo::BOOL;
        info.value = "false";
        info.description = "CGEN clock is locked, good VCO selection.";
    }
    else if (name == "lms7_temp")
    {
        info.key = "lms7_temp";
        info.name = "LMS7 Temperature";
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.value = "0.0";
        info.units = "C";
        info.description = "The temperature of the LMS7002M in degrees C.";
    }
    return info;
}

std::string Soapy_limesuiteng::readSensor(const std::string& name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    if (name == "clock_locked")
    {
        return sdrDevice->GetCGENLocked(socIndex) ? "true" : "false";
    }
    if (name == "lms7_temp")
    {
        return std::to_string(sdrDevice->GetTemperature(socIndex));
    }

    throw std::runtime_error("Soapy_limesuiteng::readSensor(" + name + ") - unknown sensor name");
}

std::vector<std::string> Soapy_limesuiteng::listSensors(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    std::vector<std::string> sensors;
    sensors.push_back("lo_locked");
    return sensors;
}

SoapySDR::ArgInfo Soapy_limesuiteng::getSensorInfo(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel, const std::string& name) const
{
    SoapySDR::ArgInfo info;
    if (name == "lo_locked")
    {
        info.key = "lo_locked";
        info.name = "LO Locked";
        info.type = SoapySDR::ArgInfo::BOOL;
        info.value = "false";
        info.description = "LO synthesizer is locked, good VCO selection.";
    }
    return info;
}

std::string Soapy_limesuiteng::readSensor(const int direction, [[maybe_unused]] const size_t channel, const std::string& name) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const auto lmsDir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;

    if (name == "lo_locked")
    {
        return sdrDevice->GetSXLocked(socIndex, lmsDir) ? "true" : "false";
    }

    throw std::runtime_error("Soapy_limesuiteng::readSensor(" + name + ") - unknown sensor name");
}

/*******************************************************************
 * Register API
 ******************************************************************/

std::vector<std::string> Soapy_limesuiteng::listRegisterInterfaces(void) const
{
    std::vector<std::string> ifaces;
    ifaces.push_back("BBIC");
    for (std::size_t i = 0; i < sdrDevice->GetDescriptor().rfSOC.at(socIndex).channelCount / 2; i++)
    {
        ifaces.push_back("RFIC" + std::to_string(i));
    }
    return ifaces;
}

void Soapy_limesuiteng::writeRegister(const std::string& name, const unsigned addr, const unsigned value)
{
    if (name == "BBIC")
    {
        return writeRegister(addr, value);
    }

    if ("RFIC" != name.substr(0, 4))
    {
        throw std::runtime_error("Soapy_limesuiteng::readRegister(" + name + ") unknown interface");
    }

    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    uint8_t index = std::stoi(name.substr(4));

    try
    {
        sdrDevice->WriteRegister(index, addr, value);
    } catch (...)
    {
        throw std::runtime_error("Soapy_limesuiteng::WriteRegister(" + name + ", " + std::to_string(addr) + ") FAIL");
    }

    return;
}

unsigned Soapy_limesuiteng::readRegister(const std::string& name, const unsigned addr) const
{
    if (name == "BBIC")
    {
        return readRegister(addr);
    }

    if ("RFIC" != name.substr(0, 4))
    {
        throw std::runtime_error("Soapy_limesuiteng::readRegister(" + name + ") unknown interface");
    }

    uint8_t index = std::stoi(name.substr(4));

    std::unique_lock<std::recursive_mutex> lock(_accessMutex);

    return sdrDevice->ReadRegister(index, addr);
}

void Soapy_limesuiteng::writeRegister(const unsigned addr, const unsigned value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    sdrDevice->WriteRegister(0, addr, value, true);
}

unsigned Soapy_limesuiteng::readRegister(const unsigned addr) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    return sdrDevice->ReadRegister(0, addr, true);
}

/*******************************************************************
 * Settings API
 ******************************************************************/
SoapySDR::ArgInfoList Soapy_limesuiteng::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList infos;

    {
        SoapySDR::ArgInfo info;
        info.key = "SAVE_CONFIG";
        info.type = SoapySDR::ArgInfo::STRING;
        info.description = "Save LMS settings to file";
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "LOAD_CONFIG";
        info.type = SoapySDR::ArgInfo::STRING;
        info.description = "Load LMS settings from file";
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "OVERSAMPLING";
        info.type = SoapySDR::ArgInfo::INT;
        info.description = "oversampling ratio (0 - auto)";
        info.options = { "0", "1", "2", "4", "8", "16", "32" };
        infos.push_back(info);
    }

    return infos;
}

void Soapy_limesuiteng::writeSetting(const std::string& key, const std::string& value)
{
    const size_t channelCount = sdrDevice->GetDescriptor().rfSOC.at(socIndex).channelCount;
    if (key == "RXTSP_CONST")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, "TSP_CONST", value);
        }
    }

    else if (key == "TXTSP_CONST")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_TX, channel, "TSP_CONST", value);
        }
    }

    else if (key == "CALIBRATE_TX")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_TX, channel, "CALIBRATE_TX", value);
        }
    }

    else if (key == "CALIBRATE_RX")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, "CALIBRATE_RX", value);
        }
    }

    else if (key == "ENABLE_RX_GFIR_LPF")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, "ENABLE_GFIR_LPF", value);
        }
    }

    else if (key == "ENABLE_TX_GFIR_LPF")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_TX, channel, "ENABLE_GFIR_LPF", value);
        }
    }

    else if (key == "DISABLE_RX_GFIR_LPF")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, "DISABLE_GFIR_LPF", value);
        }
    }

    else if (key == "DISABLE_TX_GFIR_LPF")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_TX, channel, "DISABLE_GFIR_LPF", value);
        }
    }

    else if (key == "RXTSG_NCO")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, "TSG_NCO", value);
        }
    }

    else if (key == "TXTSG_NCO")
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_TX, channel, "TSG_NCO", value);
        }
    }

    else if (key == "SAVE_CONFIG")
    {
        std::unique_lock<std::recursive_mutex> lock(_accessMutex);
        sdrDevice->SaveConfig(0, value);
    }

    else if (key == "LOAD_CONFIG")
    {
        std::unique_lock<std::recursive_mutex> lock(_accessMutex);
        sdrDevice->LoadConfig(0, value);
    }

    else if (key == "OVERSAMPLING")
    {
        oversampling = std::stoi(value);
        if (sampleRate[SOAPY_SDR_RX] > 0)
        {
            setSampleRate(SOAPY_SDR_RX, 0, sampleRate[SOAPY_SDR_RX]);
        }
        if (sampleRate[SOAPY_SDR_TX] > 0)
        {
            setSampleRate(SOAPY_SDR_TX, 0, sampleRate[SOAPY_SDR_TX]);
        }
    }

    else
    {
        for (std::size_t channel = 0; channel < channelCount; channel++)
        {
            writeSetting(SOAPY_SDR_RX, channel, key, value);
        }
    }
}

SoapySDR::ArgInfoList Soapy_limesuiteng::getSettingInfo(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    SoapySDR::ArgInfoList infos;

    {
        SoapySDR::ArgInfo info;
        info.key = "TSP_CONST";
        info.value = "16383";
        info.type = SoapySDR::ArgInfo::INT;
        info.description = "Digital DC test signal level in LMS7002M TSP chain.";
        info.range = SoapySDR::Range(0, (1 << 15) - 1);
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "CALIBRATE";
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.description = " DC/IQ calibration bandwidth";
        info.range = SoapySDR::Range(2.5e6, 120e6);
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "ENABLE_GFIR_LPF";
        info.type = SoapySDR::ArgInfo::FLOAT;
        info.description = "LPF bandwidth (must be set after sample rate)";
        infos.push_back(info);
    }

    {
        SoapySDR::ArgInfo info;
        info.key = "TSG_NCO";
        info.value = "4";
        info.description = "Enable NCO test signal";
        info.type = SoapySDR::ArgInfo::INT;
        info.options = { "-1", "4", "8" };
        info.optionNames = { "OFF", "SR/4", "SR/8" };
        infos.push_back(info);
    }
    return infos;
}

void Soapy_limesuiteng::writeSetting(const int direction, const size_t channel, const std::string& key, const std::string& value)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    const TRXDir dir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir::Rx;

    if (key == "TSP_CONST")
    {
        const auto ampl = std::stoi(value);
        sdrDevice->SetTestSignal(socIndex, dir, channel, { true, true }, ampl, ampl);
        settingsCache.at(direction).at(channel).DCTestAmplitude = ampl;
    }

    else if (key == "CALIBRATE_TX" or (dir == TRXDir::Tx and key == "CALIBRATE"))
    {
        double bw = std::stof(value);
        SoapySDR::logf(SOAPY_SDR_INFO, "Calibrate Tx %f", bw);
        sdrDevice->Calibrate(socIndex, TRXDir::Tx, channel, bw);

        settingsCache.at(direction).at(channel).calibrationBandwidth = bw;
    }

    else if (key == "CALIBRATE_RX" or (dir == TRXDir::Rx and key == "CALIBRATE"))
    {
        double bw = std::stof(value);
        SoapySDR::logf(SOAPY_SDR_INFO, "CalibrateRx %f", bw);
        sdrDevice->Calibrate(socIndex, TRXDir::Rx, channel, bw);
        settingsCache.at(direction).at(channel).calibrationBandwidth = bw;
    }

    else if (key == "ENABLE_GFIR_LPF")
    {
        double bw = std::stof(value);
        SoapySDR::logf(SOAPY_SDR_INFO, "Configure GFIR LPF %f", bw);
        sdrDevice->ConfigureGFIR(socIndex, dir, channel, { true, bw });
        settingsCache.at(direction).at(channel).GFIRBandwidth = bw;
    }

    else if (key == "DISABLE_GFIR_LPF")
    {
        SoapySDR::logf(SOAPY_SDR_INFO, "Disable GFIR LPF");
        sdrDevice->ConfigureGFIR(socIndex, dir, channel, { false, 0.0 });
        settingsCache.at(direction).at(channel).GFIRBandwidth = -1;
    }

    else if (key == "TSG_NCO")
    {
        auto select = std::stoi(value);
        if (select == -1)
        {
            // Requested to disable the TSG
            sdrDevice->SetTestSignal(socIndex, dir, channel, { false });
        }
        else if (select == 4)
        {
            sdrDevice->SetTestSignal(socIndex,
                dir,
                channel,
                { true,
                    false,
                    ChannelConfig::Direction::TestSignal::Divide::Div4,
                    ChannelConfig::Direction::TestSignal::Scale::Full });
        }
        else if (select == 8)
        {
            sdrDevice->SetTestSignal(socIndex,
                dir,
                channel,
                { true,
                    false,
                    ChannelConfig::Direction::TestSignal::Divide::Div8,
                    ChannelConfig::Direction::TestSignal::Scale::Full });
        }
        else
        {
            throw std::runtime_error("Invalid TSG_NCO option: " + value);
        }
    }
    else
    {
        uint16_t val = std::stoi(value);

        sdrDevice->SetParameter(socIndex, channel, key, val);
    }
}

std::string Soapy_limesuiteng::readSetting(const std::string& key) const
{
    if (key == "SAVE_CONFIG" || key == "LOAD_CONFIG")
    {
        return "";
    }

    if (key == "OVERSAMPLING")
    {
        return std::to_string(oversampling);
    }

    return readSetting(SOAPY_SDR_TX, 0, key);
}

std::string Soapy_limesuiteng::readSetting(const int direction, const size_t channel, const std::string& key) const
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    if (key == "TSG_NCO")
    {
        auto testConfig = sdrDevice->GetTestSignal(socIndex, direction == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx, channel);

        if (testConfig.dcMode || !testConfig.enabled)
        {
            return "-1";
        }

        switch (testConfig.scale)
        {
        case ChannelConfig::Direction::TestSignal::Scale::Half:
            return "-1";
        case ChannelConfig::Direction::TestSignal::Scale::Full:
            switch (testConfig.divide)
            {
            case ChannelConfig::Direction::TestSignal::Divide::Div4:
                return "4";
            case ChannelConfig::Direction::TestSignal::Divide::Div8:
                return "8";
            }
        }
    }
    if (key == "ENABLE_GFIR_LPF")
    {
        return std::to_string(settingsCache.at(direction).at(channel).GFIRBandwidth);
    }
    if (key == "CALIBRATE")
    {
        return std::to_string(settingsCache.at(direction).at(channel).calibrationBandwidth);
    }
    if (key == "TSP_CONST")
    {
        return std::to_string(settingsCache.at(direction).at(channel).DCTestAmplitude);
    }

    uint16_t val = sdrDevice->GetParameter(0, channel, key);
    return std::to_string(val);
}
/******************************************************************
 * GPIO API
 ******************************************************************/

std::vector<std::string> Soapy_limesuiteng::listGPIOBanks(void) const
{
    std::vector<std::string> banks;
    banks.push_back("MAIN"); // Just one associated with the connection
    return banks;
}

void Soapy_limesuiteng::writeGPIO(const std::string&, const unsigned value)
{
    OpStatus r = sdrDevice->GPIOWrite(reinterpret_cast<const uint8_t*>(&value), sizeof(value));
    if (r != OpStatus::Success)
    {
        throw std::runtime_error("Soapy_limesuiteng::writeGPIO() " + GetLastErrorMessage());
    }
}

unsigned Soapy_limesuiteng::readGPIO(const std::string&) const
{
    unsigned buffer(0);
    OpStatus r = sdrDevice->GPIORead(reinterpret_cast<uint8_t*>(&buffer), sizeof(buffer));
    if (r != OpStatus::Success)
    {
        throw std::runtime_error("Soapy_limesuiteng::readGPIO() " + GetLastErrorMessage());
    }
    return buffer;
}

void Soapy_limesuiteng::writeGPIODir(const std::string&, const unsigned dir)
{
    OpStatus r = sdrDevice->GPIODirWrite(reinterpret_cast<const uint8_t*>(&dir), sizeof(dir));
    if (r != OpStatus::Success)
    {
        throw std::runtime_error("Soapy_limesuiteng::writeGPIODir() " + GetLastErrorMessage());
    }
}

unsigned Soapy_limesuiteng::readGPIODir(const std::string&) const
{
    unsigned buffer(0);
    OpStatus r = sdrDevice->GPIODirRead(reinterpret_cast<uint8_t*>(&buffer), sizeof(buffer));
    if (r != OpStatus::Success)
    {
        throw std::runtime_error("Soapy_limesuiteng::readGPIODir() " + GetLastErrorMessage());
    }
    return buffer;
}
