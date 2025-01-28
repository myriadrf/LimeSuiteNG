#pragma once

#include <chrono>
#include <memory>

#include "limesuiteng/RFStream.h"

namespace lime {

class LimeSDR_MMX8;

class LIME_API RFStream_X8 : public RFStream
{
  public:
    RFStream_X8(LimeSDR_MMX8* parentDevice, std::unique_ptr<lime::RFStream> substream, uint8_t moduleIndex);
    virtual ~RFStream_X8();

    uint64_t GetHardwareTimestamp() const override;

    OpStatus Setup(const StreamConfig& config) override;
    const StreamConfig& GetConfig() const override;

    OpStatus Start() override;
    OpStatus StageStart() override;

    void Stop() override;
    void Teardown() override;

    uint32_t StreamRx(lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    uint32_t StreamRx(lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

  private:
    LimeSDR_MMX8* parentDevice;
    std::unique_ptr<RFStream> stream;
    uint8_t moduleIndex;
    bool isActive;
};

} // namespace lime
