#include "StreamComposite.h"

#include <algorithm>
#include <cassert>

#include "limesuiteng/types.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/Logger.h"

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
uint32_t StreamComposite::StreamRx_T(T* const* samples, uint32_t count, StreamMeta* meta, chrono::microseconds timeout)
{
    T* const* dest = samples;
    StreamMeta subDeviceMeta[8]{};
    uint8_t subDeviceCount = 0;
    uint8_t channelsCount = 0;
    for (auto& a : mAggregate)
    {
        uint32_t ret = a->StreamRx(dest, count, &subDeviceMeta[subDeviceCount], timeout);
        if (ret != count)
            return ret;

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
    for (uint32_t i = 1; i < subDeviceCount; ++i)
    {
        if (subDeviceMeta[i].timestamp != subDeviceMeta[0].timestamp)
        {
            misalignedTimestamps = true;
            break;
        }
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
    const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    const T* const* src = samples;
    uint8_t channelsCount = 0;
    for (auto& a : mAggregate)
    {
        uint32_t ret = a->StreamTx(src, count, meta, timeout);
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
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::StreamRx(
    lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::StreamRx(
    lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamRx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
}

uint32_t StreamComposite::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamTx_T(samples, count, meta, timeout);
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

} // namespace lime
