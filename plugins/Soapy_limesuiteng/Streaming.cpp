/**
@file	Streaming.cpp
@brief	Soapy SDR + IConnection streaming interfaces.
@author Lime Microsystems (www.limemicro.com)
*/

#include "Soapy_limesuiteng.h"
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>

#include <algorithm>
#include <iostream>
#include <ratio>
#include <thread>

#include "limesuiteng/Logger.h"

using namespace lime;

/*******************************************************************
 * Stream data structure
 ******************************************************************/
struct IConnectionStream {
    SDRDevice* ownerDevice{};
    int direction{};
    size_t elemSize{};
    size_t elemMTU{};

    // Rx command requests
    bool rxBurstRequest{};
    int flags{};
    long long rxBurstStart_timeNs{};
    size_t rxBurstSamples{};

    StreamConfig streamConfig;
};

/*******************************************************************
 * Stream information
 ******************************************************************/
std::vector<std::string> Soapy_limesuiteng::getStreamFormats(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    return { SOAPY_SDR_CF32, SOAPY_SDR_CS12, SOAPY_SDR_CS16 };
}

std::string Soapy_limesuiteng::getNativeStreamFormat(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel, double& fullScale) const
{
    // LimeSDR native format can be CS16 or CS12, it's selected during StreamSetup
    // but getNativeStreamFormat can be called before StreamSetup, so it's not entirely correct
    fullScale = 32767;
    return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList Soapy_limesuiteng::getStreamArgsInfo(
    [[maybe_unused]] const int direction, [[maybe_unused]] const size_t channel) const
{
    SoapySDR::ArgInfoList argInfos;

    // Link format
    {
        SoapySDR::ArgInfo info;
        info.value = SOAPY_SDR_CS16;
        info.key = "linkFormat";
        info.name = "Link Format";
        info.description = "The format of the samples over the link.";
        info.type = SoapySDR::ArgInfo::STRING;
        info.options.push_back(SOAPY_SDR_CS16);
        info.options.push_back(SOAPY_SDR_CS12);
        info.optionNames.push_back("Complex int16");
        info.optionNames.push_back("Complex int12");
        argInfos.push_back(info);
    }

    // // Skip calibrations
    // {
    //     SoapySDR::ArgInfo info;
    //     info.value = "false";
    //     info.key = "skipCal";
    //     info.name = "Skip Calibration";
    //     info.description = "Skip automatic activation calibration.";
    //     info.type = SoapySDR::ArgInfo::BOOL;
    //     argInfos.push_back(info);
    // }

    // // Align phase of Rx channels
    // {
    //     SoapySDR::ArgInfo info;
    //     info.value = "false";
    //     info.key = "alignPhase";
    //     info.name = "Align phase";
    //     info.description = "Attempt to align phase of Rx channels.";
    //     info.type = SoapySDR::ArgInfo::BOOL;
    //     argInfos.push_back(info);
    // }

    return argInfos;
}

/*******************************************************************
 * Stream config
 ******************************************************************/
SoapySDR::Stream* Soapy_limesuiteng::setupStream(
    const int direction, const std::string& format, const std::vector<size_t>& channels, const SoapySDR::Kwargs& args)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    // Store result into opaque stream object
    auto stream = new IConnectionStream;
    stream->direction = direction;
    stream->elemSize = SoapySDR::formatToSize(format);
    stream->rxBurstRequest = false;

    StreamConfig& config = streamConfig;
    config.bufferSize = 0; // Auto

    TRXDir dir = (direction == SOAPY_SDR_TX) ? TRXDir::Tx : TRXDir ::Rx;

    // Default to channel 0, if none were specified
    const std::vector<size_t>& channelIDs = channels.empty() ? std::vector<size_t>{ 0 } : channels;
    config.channels[dir] = std::vector<uint8_t>(channelIDs.begin(), channelIDs.end());

    if (format == SOAPY_SDR_CF32)
    {
        config.format = DataFormat::F32;
    }
    else if (format == SOAPY_SDR_CS16)
    {
        config.format = DataFormat::I16;
    }
    else if (format == SOAPY_SDR_CS12)
    {
        config.format = DataFormat::I12;
    }
    else
    {
        throw std::runtime_error("Soapy_limesuiteng::setupStream(format=" + format + ") unsupported stream format");
    }

    config.linkFormat = config.format == DataFormat::F32 ? DataFormat::I16 : config.format;

    // Optional link format
    if (args.count("linkFormat"))
    {
        auto linkFormat = args.at("linkFormat");
        if (linkFormat == SOAPY_SDR_CS16)
        {
            config.linkFormat = DataFormat::I16;
        }
        else if (linkFormat == SOAPY_SDR_CS12)
        {
            config.linkFormat = DataFormat::I12;
        }
        else
        {
            throw std::runtime_error("Soapy_limesuiteng::setupStream(linkFormat=" + linkFormat + ") unsupported link format");
        }
    }

    // TODO: reimplement if relevant
    // Optional buffer length if specified (from device args)

    // const auto devArgsBufferLength = _deviceArgs.find(config.isTx ? "txBufferLength" : "rxBufferLength");
    // if (devArgsBufferLength != _deviceArgs.end())
    // {
    //     config.bufferLength = std::stoul(devArgsBufferLength->second);
    // }

    // TODO: reimplement if relevant
    // Optional buffer length if specified (takes precedent)

    // if (args.count("bufferLength") != 0)
    // {
    //     config.bufferLength = std::stoul(args.at("bufferLength"));
    // }

    // TODO: reimplement if relevant
    // Optional packets latency, 1-maximum throughput, 0-lowest latency

    // if (args.count("latency") != 0)
    // {
    //     config.performanceLatency = std::stof(args.at("latency"));
    //     if (config.performanceLatency < 0)
    //     {
    //         config.performanceLatency = 0;
    //     }
    //     else if (config.performanceLatency > 1)
    //     {
    //         config.performanceLatency = 1;
    //     }
    // }

    // Create the stream
    auto returnValue = sdrDevice->StreamSetup(config, 0);
    if (returnValue != OpStatus::Success)
    {
        throw std::runtime_error("Soapy_limesuiteng::setupStream() failed: " + std::string(GetLastErrorMessage()));
    }

    auto samplesPerPacket = config.linkFormat == DataFormat::I16 ? 1020 : 1360;
    // TODO: figure out a way to actually get the correct packet per batch count
    stream->elemMTU = 8 / config.channels.at(dir).size() * samplesPerPacket;

    stream->ownerDevice = sdrDevice;
    stream->streamConfig = config;

    return reinterpret_cast<SoapySDR::Stream*>(stream);
}

void Soapy_limesuiteng::closeStream(SoapySDR::Stream* stream)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);

    const auto& ownerDevice = icstream->ownerDevice;
    ownerDevice->StreamStop(0);
    ownerDevice->StreamDestroy(0);
    delete icstream;
}

size_t Soapy_limesuiteng::getStreamMTU(SoapySDR::Stream* stream) const
{
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);
    return icstream->elemMTU;
}

int Soapy_limesuiteng::activateStream(SoapySDR::Stream* stream, const int flags, const long long timeNs, const size_t numElems)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);
    const auto& ownerDevice = icstream->ownerDevice;

    if (isStreamRunning)
        return 0;

    if (sampleRate[SOAPY_SDR_RX] <= 0.0)
        sampleRate[SOAPY_SDR_RX] = sdrDevice->GetSampleRate(0, TRXDir::Rx, 0);

    if (sampleRate[SOAPY_SDR_RX] <= 0.0)
        throw std::runtime_error("Soapy_limesuiteng::activateStream() - the sample rate has not been configured!");

    // Perform self calibration with current bandwidth settings.
    // This is for the set-it-and-forget-it style of use case
    // where boards are configured, the stream is setup,
    // and the configuration is maintained throughout the run.

    // TODO: Double check
    // while (not _channelsToCal.empty() and not icstream->skipCal)
    // {
    //     bool dir = _channelsToCal.begin()->first;
    //     auto ch = _channelsToCal.begin()->second;
    //     auto bw = mChannels[dir].at(ch).rf_bw > 0 ? mChannels[dir].at(ch).rf_bw : sampleRate[dir];
    //     bw = bw > 2.5e6 ? bw : 2.5e6;
    //     sdrDevice->Calibrate(dir == SOAPY_SDR_TX ? TRXDir::Tx : TRXDir::Rx, ch, bw, 0);
    //     settingsCache.at(direction).at(ch).calibrationBandwidth = bw;
    //     _channelsToCal.erase(_channelsToCal.begin());
    // }
    // Stream requests used with rx
    icstream->flags = flags;
    icstream->rxBurstStart_timeNs = timeNs;
    icstream->rxBurstRequest = (flags & SOAPY_SDR_HAS_TIME) | (flags & SOAPY_SDR_END_BURST);
    icstream->rxBurstSamples = numElems;

    ownerDevice->StreamStart(0);
    isStreamRunning = true;
    return 0;
}

int Soapy_limesuiteng::deactivateStream(
    SoapySDR::Stream* stream, [[maybe_unused]] const int flags, [[maybe_unused]] const long long timeNs)
{
    std::unique_lock<std::recursive_mutex> lock(_accessMutex);
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);
    const auto& ownerDevice = icstream->ownerDevice;
    icstream->rxBurstRequest = false;

    ownerDevice->StreamStop(0);
    isStreamRunning = false;
    return 0;
}

/*******************************************************************
 * Stream API
 ******************************************************************/
int Soapy_limesuiteng::readStream(
    SoapySDR::Stream* stream, void* const* buffs, size_t numElems, int& flags, long long& timeNs, const long timeoutUs)
{
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);

    // Handle the one packet flag by clipping
    if ((flags & SOAPY_SDR_ONE_PACKET) != 0)
        numElems = std::min(numElems, icstream->elemMTU);

    StreamMeta metadata{};
    const uint64_t requestedBurstStart =
        ((icstream->rxBurstRequest) ? SoapySDR::timeNsToTicks(icstream->rxBurstStart_timeNs, sampleRate[SOAPY_SDR_RX]) : 0);

    if (icstream->rxBurstRequest && ((icstream->flags & SOAPY_SDR_HAS_TIME) != 0))
    {
        uint64_t rxNow = 0; // assume burst request is done with streamActivate, so Rx always start from 0
        // rx samples requested to start at specified timestamp, drop all samples before that
        while (rxNow < requestedBurstStart)
        {
            uint32_t samplesToSkip = rxNow + numElems >= requestedBurstStart ? requestedBurstStart - rxNow : numElems;

            int samplesReceived = 0;
            switch (icstream->streamConfig.format)
            {
            case DataFormat::I16:
            case DataFormat::I12:
                samplesReceived =
                    icstream->ownerDevice->StreamRx(0, reinterpret_cast<complex16_t* const*>(buffs), samplesToSkip, &metadata);
                break;
            case DataFormat::F32:
                samplesReceived =
                    icstream->ownerDevice->StreamRx(0, reinterpret_cast<complex32f_t* const*>(buffs), samplesToSkip, &metadata);
                break;
            }
            if (samplesReceived <= 0)
                return SOAPY_SDR_STREAM_ERROR;
            rxNow = metadata.timestamp + samplesReceived;
        }
    }

    if (icstream->rxBurstRequest)
    {
        if (icstream->rxBurstSamples > 0)
            numElems = std::min(numElems, icstream->rxBurstSamples);
        if (icstream->rxBurstSamples == 0 && ((icstream->flags & SOAPY_SDR_END_BURST) != 0))
            return SOAPY_SDR_TIMEOUT; // requested burst samples have been consumed, return TIMEOUT as if no more data is available
    }

    int samplesReceived = 0;
    switch (icstream->streamConfig.format)
    {
    case DataFormat::I16:
    case DataFormat::I12:
        samplesReceived = icstream->ownerDevice->StreamRx(0, reinterpret_cast<complex16_t* const*>(buffs), numElems, &metadata);
        break;
    case DataFormat::F32:
        samplesReceived = icstream->ownerDevice->StreamRx(0, reinterpret_cast<complex32f_t* const*>(buffs), numElems, &metadata);
        break;
    }

    flags = 0;

    if (samplesReceived == 0)
        return SOAPY_SDR_TIMEOUT;

    if (samplesReceived < 0)
        return SOAPY_SDR_STREAM_ERROR;

    if (icstream->rxBurstRequest && ((icstream->flags & SOAPY_SDR_HAS_TIME) != 0))
    {
        icstream->flags &= ~SOAPY_SDR_HAS_TIME; // clear for next read
        if (metadata.timestamp != requestedBurstStart)
        {
            SoapySDR::log(SOAPY_SDR_ERROR,
                "readStream() rx burst overflow, expected tick:" + std::to_string(requestedBurstStart) +
                    ", got: " + std::to_string(metadata.timestamp));
            return SOAPY_SDR_OVERFLOW;
        }
    }

    if (icstream->rxBurstSamples > 0)
    {
        icstream->rxBurstSamples -= samplesReceived;
        if (icstream->rxBurstSamples == 0)
            flags |= SOAPY_SDR_END_BURST;
    }

    // LimeSDR always return Rx timestamp
    flags |= SOAPY_SDR_HAS_TIME;
    timeNs = SoapySDR::ticksToTimeNs(metadata.timestamp, sampleRate[SOAPY_SDR_RX]);

    // Return num read or error code
    return (samplesReceived >= 0) ? samplesReceived : SOAPY_SDR_STREAM_ERROR;
}

int Soapy_limesuiteng::writeStream(SoapySDR::Stream* stream,
    const void* const* buffs,
    const size_t numElems,
    int& flags,
    const long long timeNs,
    [[maybe_unused]] const long timeoutUs)
{
    if ((flags & SOAPY_SDR_HAS_TIME) && (timeNs < 0))
    {
        return SOAPY_SDR_TIME_ERROR;
    }

    auto icstream = reinterpret_cast<IConnectionStream*>(stream);
    const auto& ownerDevice = icstream->ownerDevice;

    // Input metadata
    StreamMeta metadata{};
    metadata.timestamp = SoapySDR::timeNsToTicks(timeNs, sampleRate[SOAPY_SDR_RX]);
    metadata.waitForTimestamp = (flags & SOAPY_SDR_HAS_TIME);
    metadata.flushPartialPacket = (flags & SOAPY_SDR_END_BURST);

    int samplesSent = 0;
    switch (icstream->streamConfig.format)
    {
    case DataFormat::I16:
    case DataFormat::I12:
        samplesSent = ownerDevice->StreamTx(0, reinterpret_cast<const complex16_t* const*>(buffs), numElems, &metadata);
        break;
    case DataFormat::F32:
        samplesSent = ownerDevice->StreamTx(0, reinterpret_cast<const complex32f_t* const*>(buffs), numElems, &metadata);
        break;
    }

    if (samplesSent == 0)
        return SOAPY_SDR_TIMEOUT;

    if (samplesSent < 0)
        return SOAPY_SDR_STREAM_ERROR;

    return samplesSent;
}

int Soapy_limesuiteng::readStreamStatus(
    SoapySDR::Stream* stream, [[maybe_unused]] size_t& chanMask, int& flags, long long& timeNs, const long timeoutUs)
{
    auto icstream = reinterpret_cast<IConnectionStream*>(stream);
    const auto& ownerDevice = icstream->ownerDevice;

    int ret = 0;
    flags = 0;
    StreamStats metadata;
    auto start = std::chrono::high_resolution_clock::now();
    while (1)
    {
        if (icstream->direction == SOAPY_SDR_TX)
        {
            ownerDevice->StreamStatus(0, nullptr, &metadata);
        }
        else
        {
            ownerDevice->StreamStatus(0, &metadata, nullptr);
        }

        if (metadata.loss)
        {
            if (icstream->direction == SOAPY_SDR_TX)
                ret = SOAPY_SDR_UNDERFLOW;
            else
                ret = SOAPY_SDR_OVERFLOW;
        }
        else if (metadata.overrun)
        {
            ret = SOAPY_SDR_OVERFLOW;
        }
        else if (metadata.underrun)
        {
            ret = SOAPY_SDR_UNDERFLOW;
        }

        if (ret)
        {
            break;
        }
        // Check timeout
        std::chrono::duration<double, std::micro> microseconds = std::chrono::high_resolution_clock::now() - start;
        if (microseconds.count() >= timeoutUs)
        {
            return SOAPY_SDR_TIMEOUT;
        }
        // Sleep to avoid high CPU load
        if (timeoutUs >= 1000000)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::microseconds(timeoutUs));
        }
    }

    timeNs = SoapySDR::ticksToTimeNs(metadata.timestamp, sampleRate[SOAPY_SDR_RX]);
    // Output metadata
    flags |= SOAPY_SDR_HAS_TIME;
    return ret;
}
