#include "StreamComposite.h"

#include <algorithm>
#include <cassert>

#include "limesuiteng/types.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/StreamMeta.h"

using namespace std;

namespace lime {

StreamComposite::StreamComposite()
    : warnAboutMisalignment(true)
{
}

StreamComposite::~StreamComposite()
{
    for (auto& substream : mAggregate)
        delete substream;
}

OpStatus StreamComposite::Add(std::unique_ptr<RFStream> stream)
{
    mAggregate.push_back(stream.release());
    return OpStatus::Success;
}

OpStatus StreamComposite::Setup(const StreamConfig& config)
{
    size_t rxTotalCount = 0;
    size_t txTotalCount = 0;
    for (auto& a : mAggregate)
    {
        assert(a);
        StreamConfig subConfig = a->GetConfig();
        rxTotalCount += subConfig.channels.at(TRXDir::Rx).size();
        txTotalCount += subConfig.channels.at(TRXDir::Tx).size();
    }

    if (config.channels.at(TRXDir::Rx).size() > rxTotalCount)
        return ReportError(OpStatus::InvalidValue, "StreamComposite Setup requests too many Rx channels");

    if (config.channels.at(TRXDir::Tx).size() > txTotalCount)
        return ReportError(OpStatus::InvalidValue, "StreamComposite Setup requests too many Tx channels");

    mConfig = config;
    return OpStatus::Success;
}

const StreamConfig& StreamComposite::GetConfig() const
{
    return mConfig;
}

OpStatus StreamComposite::Start()
{
    warnAboutMisalignment = true;
    OpStatus status{ OpStatus::InvalidValue }; // if there are no aggregates to be started
    for (auto& a : mAggregate)
    {
        status = a->Start();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus StreamComposite::StageStart()
{
    OpStatus status{ OpStatus::InvalidValue }; // if there are no aggregates to be started
    for (auto& a : mAggregate)
    {
        status = a->StageStart();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

void StreamComposite::Stop()
{
    for (auto& a : mAggregate)
        a->Stop();
}

void StreamComposite::Teardown()
{
    for (auto& a : mAggregate)
        a->Teardown();
}

template<class T>
uint32_t StreamComposite::StreamRx_T(T* const* samples, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
{
    T* const* dest = samples;
    StreamRxMeta subDeviceMeta[8]{};
    uint32_t samplesGot[8]{};
    uint8_t subDeviceCount = 0;
    uint8_t channelsCount = 0;
    for (auto& a : mAggregate)
    {
        samplesGot[subDeviceCount] = a->Receive(dest, count, &subDeviceMeta[subDeviceCount]);
        const int devChannels = a->GetConfig().channels.at(TRXDir::Rx).size();
        dest += devChannels;
        channelsCount += devChannels;
        ++subDeviceCount;

        // aggregate subdevices might not necessarilly be used
        if (channelsCount >= mConfig.channels[TRXDir::Rx].size())
            break;
    }

    if (meta)
        meta->timestamp = subDeviceMeta[0].timestamp;

    if (!warnAboutMisalignment)
        return count;

    bool misalignedTimestamps{ false };
    for (uint32_t i = 0; i < subDeviceCount; ++i)
    {
        if (samplesGot[i] != count)
        {
            lime::error("StreamComposite: not enough samples");
            return samplesGot[i];
        }
        // if (subDeviceMeta[0].timestamp != subDeviceMeta[0].timestamp)
        // {
        //     misalignedTimestamps = true;
        //     break;
        // }
    }

    if (misalignedTimestamps)
    {
        lime::error("StreamComposite: misaligned timestamps among channels.");
        warnAboutMisalignment = false; // warn once per stream activation to prevent spam
    }
    return count;
}

template<class T>
uint32_t StreamComposite::StreamTx_T(
    const T* const* samples, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds timeout)
{
    const T* const* src = samples;
    uint8_t channelsCount = 0;
    for (auto& a : mAggregate)
    {
        uint32_t ret = a->Transmit(src, count, meta);
        if (ret != count)
            return ret;

        const int devChannels = a->GetConfig().channels.at(TRXDir::Tx).size();
        src += devChannels;
        channelsCount += devChannels;

        // aggregate subdevices might not necessarilly be used
        if (channelsCount >= mConfig.channels[TRXDir::Tx].size())
            break;
    }
    return count;
}

uint32_t StreamComposite::StreamRx(
    lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRx_T(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

uint32_t StreamComposite::StreamRx(
    lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRx_T(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

uint32_t StreamComposite::StreamRx(
    lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRx_T(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

uint32_t StreamComposite::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta ? meta->waitForTimestamp : false;
    if (txmeta.hasTimestamp)
    {
        if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
            txmeta.timestamp = Timespec(meta->timestamp / mConfig.hintSampleRate);
        else
            txmeta.timestamp = Timespec(meta->timestamp >> 32, (meta->timestamp & 0xFFFFFFFF) / 1e9);
    }
    return StreamTx_T(samples, count, &txmeta, timeout);
}

uint32_t StreamComposite::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta ? meta->waitForTimestamp : false;
    if (txmeta.hasTimestamp)
    {
        if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
            txmeta.timestamp = Timespec(meta->timestamp / mConfig.hintSampleRate);
        else
            txmeta.timestamp = Timespec(meta->timestamp >> 32, (meta->timestamp & 0xFFFFFFFF) / 1e9);
    }
    return StreamTx_T(samples, count, &txmeta, timeout);
}

uint32_t StreamComposite::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta ? meta->waitForTimestamp : false;
    if (txmeta.hasTimestamp)
    {
        if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
            txmeta.timestamp = Timespec(meta->timestamp / mConfig.hintSampleRate);
        else
            txmeta.timestamp = Timespec(meta->timestamp >> 32, (meta->timestamp & 0xFFFFFFFF) / 1e9);
    }
    return StreamTx_T(samples, count, &txmeta, timeout);
}

void StreamComposite::StreamStatus(StreamStats* rx, StreamStats* tx)
{
    // TODO: implement
}

uint64_t StreamComposite::GetHardwareTimestamp() const
{
    if (mAggregate.empty())
        return 0;
    return mAggregate.front()->GetHardwareTimestamp();
}

uint32_t StreamComposite::Receive(
    lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
{
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::Receive(
    lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
{
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::Receive(
    lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
{
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::Transmit(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* meta, chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::Transmit(
    const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* meta, chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::Transmit(
    const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* meta, chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
}

} // namespace lime
