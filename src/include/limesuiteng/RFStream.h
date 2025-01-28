#ifndef LIME_RFSTREAM_H
#define LIME_RFSTREAM_H

#include <chrono>

#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/OpStatus.h"

namespace lime {

class SDRDevice;
struct StreamConfig;
struct StreamMeta;
struct StreamStats;

/// @brief Interface for interacting with RF Rx/Tx samples data streaming
class LIME_API RFStream
{
  public:
    static constexpr std::chrono::microseconds DEFAULT_TIMEOUT{ std::chrono::microseconds(1000000) };

    virtual ~RFStream(){};

    /// @brief Gets the hardware timestamp.
    /// @return The most recent timestamp of the hardware in samples count.
    virtual uint64_t GetHardwareTimestamp() const = 0;

    /// @brief Configures data streaming parameters
    /// @param config The configuration to use for setting the streams up.
    /// @return The status code of the operation.
    virtual OpStatus Setup(const StreamConfig& config) = 0;

    /// @brief Returns current stream configuration.
    virtual const StreamConfig& GetConfig() const = 0;

    /// @brief Starts RF data streaming.
    /// If the stream is alrady started, repeated calls to Start() should have no effect.
    virtual OpStatus Start() = 0;

    /// @brief Prepares stream into working state, but the actual hardware stream will be triggered externaly.
    /// StageStart() is to be used when multiple independent RFStreams need to be synchronously started at the same time.
    /// If and which streams can be synchronized depends on hardware configuration.
    /// If multiple independent RFStream of the same device are staged, starting one of them will start all of them.
    /// StagedStart() can be cancelled by Stop()
    virtual OpStatus StageStart() = 0;

    /// @brief Stops RF data streaming and clears internal buffers.
    virtual void Stop() = 0;

    /// @brief Frees internal resources.
    /// After Teardown(), stream needs to be Setup() before it can be started.
    virtual void Teardown() = 0;

    /// @brief Receives RF samples data.
    /// @param samples The buffer to put the received samples in.
    /// @param count The number of samples to receive into the buffer.
    /// @param meta The metadata of the packets of the stream.
    /// @return The amount of samples received.
    virtual uint32_t StreamRx(lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc RFStream::StreamRx()
    virtual uint32_t StreamRx(lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc RFStream::StreamRx()
    virtual uint32_t StreamRx(lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;

    /// @brief Transmits RF samples data.
    /// @param samples The buffer of the samples to transmit.
    /// @param count The number of samples to transmit.
    /// @param meta The metadata of the packets of the stream.
    /// @return The amount of samples transmitted.
    virtual uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc RFStream::StreamTx()
    virtual uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;
    /// @copydoc RFStream::StreamRx()
    virtual uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout = DEFAULT_TIMEOUT) = 0;

    /// @brief Retrieves the current stream statistics.
    /// @param rx The pointer (or nullptr if not needed) to store the receive statistics to.
    /// @param tx The pointer (or nullptr if not needed) to store the transmit statistics to.
    virtual void StreamStatus(StreamStats* rx, StreamStats* tx) = 0;
};

} // namespace lime
#endif
