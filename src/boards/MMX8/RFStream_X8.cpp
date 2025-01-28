#include "RFStream_X8.h"

#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "MM_X8.h"

#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/OpStatus.h"

namespace lime {

RFStream_X8::RFStream_X8(LimeSDR_MMX8* parentDevice, std::unique_ptr<lime::RFStream> substream, uint8_t moduleIndex)
    : parentDevice(parentDevice)
    , stream(std::move(substream))
    , moduleIndex(moduleIndex)
    , isActive(false)
{
}

RFStream_X8::~RFStream_X8()
{
    Stop();
}

uint64_t RFStream_X8::GetHardwareTimestamp() const
{
    return stream->GetHardwareTimestamp();
}

OpStatus RFStream_X8::Setup(const StreamConfig& config)
{
    return stream->Setup(config);
}

const StreamConfig& RFStream_X8::GetConfig() const
{
    return stream->GetConfig();
}

OpStatus RFStream_X8::Start()
{
    OpStatus status = StageStart();
    if (status != OpStatus::Success)
        return status;

    parentDevice->StreamsTrigger();
    isActive = true;
    return status;
}

OpStatus RFStream_X8::StageStart()
{
    // XTRX streaming will be activated, threads and DMA will be ready.
    // but will not start until X8 FPGA enables stream.
    OpStatus status = stream->Start();
    if (status != OpStatus::Success)
        return status;
    parentDevice->StreamEnable(moduleIndex, true);
    isActive = true;
    return status;
}

void RFStream_X8::Stop()
{
    if (!isActive)
        return;

    stream->Stop();
    parentDevice->StreamEnable(moduleIndex, false);
    parentDevice->StreamsTrigger();
    isActive = false;
}

void RFStream_X8::Teardown()
{
    Stop();
    stream->Teardown();
}

uint32_t RFStream_X8::StreamRx(
    lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamRx(samples, count, meta, timeout);
}

uint32_t RFStream_X8::StreamRx(
    lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamRx(samples, count, meta, timeout);
}

uint32_t RFStream_X8::StreamRx(
    lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamRx(samples, count, meta, timeout);
}

uint32_t RFStream_X8::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamTx(samples, count, meta, timeout);
}

uint32_t RFStream_X8::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamTx(samples, count, meta, timeout);
}

uint32_t RFStream_X8::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return stream->StreamTx(samples, count, meta, timeout);
}

void RFStream_X8::StreamStatus(StreamStats* rx, StreamStats* tx)
{
    stream->StreamStatus(rx, tx);
}
} // namespace lime
