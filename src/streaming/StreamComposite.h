#pragma once

#include <vector>
#include <chrono>
#include <memory>
#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/RFStream.h"

namespace lime {

struct StreamMeta;

/** @brief Class for managing streaming from multiple devices at the same time. */
class LIME_API StreamComposite : public RFStream
{
  public:
    StreamComposite();
    virtual ~StreamComposite();

    /// @brief Adds given stream into streams aggregation.
    /// @param stream Stream interface to be added.
    /// The StreamComposite takes over ownership of the aggregate streams.
    OpStatus Add(std::unique_ptr<RFStream> stream);

    /// @copydoc RFStream::Setup()
    OpStatus Setup(const StreamConfig& config) override;
    const StreamConfig& GetConfig() const override;

    /// @copydoc RFStream::Start()
    OpStatus Start() override;

    /// @copydoc RFStream::StageStart()
    OpStatus StageStart() override;

    /// @copydoc RFStream::Stop()
    void Stop() override;

    /// @copydoc RFStream::Teardown()
    void Teardown() override;

    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamRx()
    uint32_t StreamRx(lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;
    /// @copydoc RFStream::StreamTx()
    uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = RFStream::DEFAULT_TIMEOUT) override;

    /// @copydoc RFStream::StreamStatus()
    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

    uint64_t GetHardwareTimestamp() const override;

  private:
    template<class T> uint32_t StreamRx_T(T* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamTx_T(const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout);

    std::vector<StreamConfig> SplitAggregateStreamSetup(const StreamConfig& cfg);
    std::vector<RFStream*> mAggregate;
    StreamConfig mConfig;
    bool warnAboutMisalignment; // warn only once if channels get desynced
};

} // namespace lime
