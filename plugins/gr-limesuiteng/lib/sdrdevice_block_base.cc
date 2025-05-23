#include "sdrdevice_block_base.h"

#include "sdrdevice_manager.h"

#include <chrono>
#include <limits>
#include <sstream>
#include <thread>

#include "limesuiteng/Logger.h"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.h"
#include "limesuiteng/SDRConfig.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/ToString.h"
#include "limesuiteng/complex.h"

using namespace lime;

namespace gr {
namespace limesuiteng {

static lime::DataFormat GetDataFormatEnum(const std::string& formatname)
{
    if (formatname == "complex16_t")
        return lime::DataFormat::I16;
    else if (formatname == "complex12_t")
        return lime::DataFormat::I12;
    else if (formatname == "complex32f_t")
        return lime::DataFormat::F32;
    else
        return lime::DataFormat::F32;
}

sdrdevice_block_base::sdrdevice_block_base(lime::TRXDir dir,
                                           const std::string& alias,
                                           const std::string& deviceHandleHint,
                                           uint32_t chipIndex,
                                           const std::vector<int>& channelIndexes,
                                           const std::string& dataFormat,
                                           double sampleRate,
                                           int rf_oversampling,
                                           gr::logger_ptr logger,
                                           gr::logger_ptr debug_baselogger)
    : chipIndex(chipIndex),
      direction(dir),
      autoAntenna(true),
      canWork(false),
      baselogger(logger)
{
    devManager = sdrdevice_manager::GetSingleton();
    assert(devManager);
    devContext = devManager->GetDeviceContextByHandle(deviceHandleHint);
    if (!devContext)
        throw std::runtime_error("sdrdevice_block_base::sdrdevice_block_base(): No "
                                 "valid device with handle");

    // TODO: allow multiple source instances of the same sdr device, if it use different
    // chip indexe
    if (direction == TRXDir::Rx) {
        if (devContext->sourceBlockCounter > 0)
            throw std::runtime_error("More than one Source block "
                                     "assigned to same SDR device");
        ++devContext->sourceBlockCounter;
    } else {
        if (devContext->sinkBlockCounter > 0)
            throw std::runtime_error("More than one Sink block "
                                     "assigned to same SDR device");
        ++devContext->sinkBlockCounter;
    }

    devContext->streamCfg.channels.at(direction).clear();
    for (const auto index : channelIndexes)
        devContext->streamCfg.channels.at(direction).push_back(index);

    devContext->streamCfg.format = GetDataFormatEnum(dataFormat);
    devContext->streamCfg.linkFormat = lime::DataFormat::I16;
    devContext->streamCfg.hintSampleRate = sampleRate;

    lime::SDRConfig& config = devContext->deviceConfig;
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        auto& channel =
            (direction == TRXDir::Tx) ? config.channel[ch].tx : config.channel[ch].rx;
        channel.enabled = true;
        channel.sampleRate = sampleRate;
        channel.oversample = rf_oversampling;
    }
}

sdrdevice_block_base::~sdrdevice_block_base()
{
    GR_LOG_DEBUG(baselogger, __func__);
    ReleaseResources();
}

void sdrdevice_block_base::ReleaseResources()
{
    GR_LOG_DEBUG(baselogger, __func__);
    devContext.reset();
    devManager.reset();
}

bool sdrdevice_block_base::start()
{
    GR_LOG_DEBUG(baselogger, __func__);
    assert(devContext);

    lime::SDRConfig& config = devContext->deviceConfig;

    if (direction == TRXDir::Tx)
        devContext->sinkConfigReady = true;
    else
        devContext->sourceConfigReady = true;

    const bool doConfigure =
        (devContext->sourceBlockCounter ? devContext->sourceConfigReady : true) &&
        (devContext->sinkBlockCounter ? devContext->sinkConfigReady : true);

    if (!doConfigure) {
        canWork = true;
        return true;
    }

    GR_LOG_DEBUG(baselogger, "SDRDevice Configure()");
    config.skipDefaults = !devContext->customBaseConfigFilepath.empty();
    if (config.skipDefaults &&
        devContext->device->LoadConfig(chipIndex, devContext->customBaseConfigFilepath) !=
            OpStatus::Success) {
        const std::string msg = fmt::format("Failed to load configuration file {:s}",
                                            devContext->customBaseConfigFilepath);
        GR_LOG_ERROR(baselogger, msg);
        throw std::runtime_error(msg);
    }

    if (devContext->device->Configure(config, chipIndex) != OpStatus::Success) {
        const std::string msg = fmt::format("Failed to configure device. {:s}",
                                            lime::GetLastErrorMessageCString());
        GR_LOG_ERROR(baselogger, msg);
        throw std::runtime_error(msg);
        return false;
    }

    GR_LOG_DEBUG(baselogger, "RFStream Create()");
    devContext->stream =
        devContext->device->StreamCreate(devContext->streamCfg, chipIndex);
    if (!devContext->stream)
        return false;

    // Start the actual streaming in work(), otherwise multiple devices won't be started
    // at the same time
    OpStatus status = devContext->stream->StageStart();
    GR_LOG_INFO(baselogger, "Device configured and ready.");
    canWork = true;
    return true;
}

void sdrdevice_block_base::StartRFStreaming()
{
    if (devContext->stream->Start() != OpStatus::Success) {
        GR_LOG_ERROR(baselogger, "RFStream Start() Failed");
    } else {
        GR_LOG_INFO(baselogger, "RFStream Start()");
    }
}

bool sdrdevice_block_base::stop()
{
    assert(devContext);
    GR_LOG_DEBUG(baselogger, __func__);
    if (direction == TRXDir::Tx)
        devContext->sinkBlockCounter--;
    else
        devContext->sourceBlockCounter--;

    const bool doStop =
        (devContext->sourceBlockCounter == 0 && devContext->sinkBlockCounter == 0);

    if (doStop && devContext->stream) {
        GR_LOG_DEBUG(baselogger, "RFStream Stop");
        devContext->stream->Stop();
        devContext->stream->Teardown();
        devContext->stream.reset();
    }
    canWork = false;
    // GRC does not call destructor, so cleanup resources on stop() to ensure proper
    // resources destruction order
    ReleaseResources();
    return true;
}

void sdrdevice_block_base::set_config_file(const std::string& file_path)
{
    if (!devContext)
        return;

    GR_LOG_INFO(baselogger, fmt::format("{:s} {:s}", __func__, file_path));
    devContext->customBaseConfigFilepath = file_path;
}

double sdrdevice_block_base::set_lo_frequency(double frequencyHz)
{
    if (!devContext)
        return frequencyHz;

    GR_LOG_INFO(baselogger, fmt::format("{:s} {:f}", __func__, frequencyHz));

    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            devContext->device->SetFrequency(chipIndex, direction, ch, frequencyHz);
        } else {
            if (direction == TRXDir::Tx)
                devContext->deviceConfig.channel[ch].tx.centerFrequency = frequencyHz;
            else
                devContext->deviceConfig.channel[ch].rx.centerFrequency = frequencyHz;
        }

        if (autoAntenna)
            set_antenna("auto");
    }
    return frequencyHz;
}

double sdrdevice_block_base::set_lpf_bandwidth(double bandwidthHz)
{
    if (!devContext)
        return bandwidthHz;

    GR_LOG_INFO(baselogger, fmt::format("{:s} {:f}", __func__, bandwidthHz));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            devContext->device->SetLowPassFilter(chipIndex, direction, ch, bandwidthHz);
        } else {
            if (direction == TRXDir::Tx)
                devContext->deviceConfig.channel[ch].tx.lpf = bandwidthHz;
            else
                devContext->deviceConfig.channel[ch].rx.lpf = bandwidthHz;
        }
    }
    return bandwidthHz;
}

double sdrdevice_block_base::set_gfir_bandwidth(double bandwidthHz)
{
    if (!devContext)
        return bandwidthHz;

    const bool enableFilter = bandwidthHz > 0;
    GR_LOG_INFO(baselogger,
                fmt::format("{:s} {:f} {:s}",
                            __func__,
                            bandwidthHz,
                            enableFilter ? "enabled" : "disabled"));
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            ChannelConfig::Direction::GFIRFilter filter;
            filter.enabled = enableFilter;
            filter.bandwidth = bandwidthHz;
            devContext->device->ConfigureGFIR(chipIndex, direction, ch, filter);
        } else {
            if (direction == TRXDir::Tx) {
                devContext->deviceConfig.channel[ch].tx.gfir.enabled = enableFilter;
                devContext->deviceConfig.channel[ch].tx.gfir.bandwidth = bandwidthHz;
            } else {
                devContext->deviceConfig.channel[ch].rx.gfir.enabled = enableFilter;
                devContext->deviceConfig.channel[ch].rx.gfir.bandwidth = bandwidthHz;
            }
        }
    }
    return bandwidthHz;
}

static std::string
GetAntennaForFrequency(double frequencyHz,
                       const std::vector<std::string>& options,
                       const std::unordered_map<std::string, lime::Range<double>>& ranges)
{
    std::string name = "auto_failed";
    double deviation = std::numeric_limits<double>::max();
    for (const auto& name_range : ranges) {
        const auto& range = name_range.second;

        if (std::find(options.begin(), options.end(), name_range.first) == options.end())
            continue;

        const double mid = range.min + (range.max - range.min) / 2;
        const double d = std::abs(mid - frequencyHz);
        if (d <= deviation) {
            name = name_range.first;
            deviation = d;
        }
    }
    return name;
}

bool sdrdevice_block_base::set_antenna(const std::string& antenna_name)
{
    if (!devContext)
        return false;

    GR_LOG_INFO(baselogger, fmt::format("{:s} {:s}", __func__, antenna_name));
    const auto& antennas =
        devContext->device->GetDescriptor().rfSOC.at(chipIndex).pathNames.at(direction);
    auto antennaFind = antennas.begin();

    autoAntenna = antenna_name.empty() || (antenna_name == "auto");

    if (autoAntenna) {
        double freq;
        int ch = devContext->streamCfg.channels.at(direction).front();
        if (devContext->stream) {
            freq = devContext->device->GetFrequency(chipIndex, direction, ch);
        } else {
            if (direction == TRXDir::Tx)
                freq = devContext->deviceConfig.channel[ch].tx.centerFrequency;
            else
                freq = devContext->deviceConfig.channel[ch].rx.centerFrequency;
        }
        const std::string bestAntenna = GetAntennaForFrequency(
            freq,
            antennas,
            devContext->device->GetDescriptor().rfSOC.at(chipIndex).antennaRange.at(
                direction));
        GR_LOG_INFO(baselogger, fmt::format("auto selected antenna: {:s}", bestAntenna));
        antennaFind = std::find(antennas.begin(), antennas.end(), bestAntenna);
    } else
        antennaFind = std::find(antennas.begin(), antennas.end(), antenna_name);

    if (antennaFind == antennas.end()) {
        std::stringstream ss;
        ss << "Antenna " << antenna_name << " not found. Available:" << std::endl;
        for (const auto& iter : antennas)
            ss << "\t\"" << iter << "\"" << std::endl;
        GR_LOG_ERROR(baselogger, ss.str());
        return false;
    }

    const int antennaIndex = std::distance(antennas.begin(), antennaFind);
    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            devContext->device->SetAntenna(chipIndex, direction, ch, antennaIndex);
        } else {
            if (direction == TRXDir::Tx)
                devContext->deviceConfig.channel[ch].tx.path = antennaIndex;
            else
                devContext->deviceConfig.channel[ch].rx.path = antennaIndex;
        }
    }
    return true;
}

double sdrdevice_block_base::set_gain_generic(double gain_dB)
{
    return set_gain(lime::eGainTypes::GENERIC, gain_dB);
}

double sdrdevice_block_base::set_gain(lime::eGainTypes type, double gain_value)
{
    if (!devContext)
        return gain_value;

    const std::string gainName = lime::ToString(type);

    GR_LOG_INFO(baselogger,
                fmt::format("{:s} {:s} {:f}", __func__, gainName, gain_value));
    const RFSOCDescriptor& desc = devContext->device->GetDescriptor().rfSOC.at(chipIndex);

    lime::Range gain_range = desc.gainRange.at(direction).at(type);
    GR_LOG_INFO(baselogger,
                fmt::format("{:s} {:s} gain range: {:f}:{:f}",
                            __func__,
                            gainName,
                            gain_range.min,
                            gain_range.max));
    if (gain_value > gain_range.max) {
        gain_value = gain_range.max;
        GR_LOG_WARN(baselogger,
                    fmt::format("{:s} clip gain to {:f}", __func__, gain_value));
    } else if (gain_value < gain_range.min) {
        gain_value = gain_range.min;
        GR_LOG_WARN(baselogger,
                    fmt::format("{:s} clip gain to {:f}", __func__, gain_value));
    }

    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            devContext->device->SetGain(chipIndex, direction, ch, type, gain_value);
        } else {
            if (direction == TRXDir::Tx)
                devContext->deviceConfig.channel[ch].tx.gain[type] = gain_value;
            else
                devContext->deviceConfig.channel[ch].rx.gain[type] = gain_value;
        }
    }
    return gain_value;
}

double sdrdevice_block_base::set_nco_frequency(double frequency_offset_Hz)
{
    if (!devContext)
        return frequency_offset_Hz;

    GR_LOG_INFO(baselogger, fmt::format("{:s} {:f}", __func__, frequency_offset_Hz));
    if (!devContext)
        return frequency_offset_Hz;

    for (const int ch : devContext->streamCfg.channels.at(direction)) {
        if (devContext->stream) {
            devContext->device->SetNCOFrequency(
                chipIndex, direction, ch, 0, frequency_offset_Hz);
        } else {
            if (direction == TRXDir::Tx)
                devContext->deviceConfig.channel[ch].tx.NCOoffset = frequency_offset_Hz;
            else
                devContext->deviceConfig.channel[ch].rx.NCOoffset = frequency_offset_Hz;
        }
    }
    return frequency_offset_Hz;
}

} /* namespace limesuiteng */
} /* namespace gr */
