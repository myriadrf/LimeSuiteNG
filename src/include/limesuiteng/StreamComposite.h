#pragma once

#include <vector>
#include <memory>
#include "limesuiteng/config.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/SDRDevice.h"

namespace lime {

struct StreamConfig;
struct StreamMeta;

/** @brief Structure for holding information about the aggregate stream. */
struct LIME_API StreamAggregate {
    SDRDevice* device; ///< The device the stream is coming from.
    std::vector<int32_t> channels; ///< The channels the device is streaming with.
    int32_t streamIndex; ///< The index of the stream.
};

/** @brief Class for managing streaming from multiple devices at the same time. */
class LIME_API StreamComposite
{
  public:
    StreamComposite() = delete;

    /// @brief Constructs the StreamComposite object.
    /// @param aggregate The list of streams to aggregate into one stream.
    StreamComposite(const std::vector<StreamAggregate>& aggregate);

    /// @brief Sets up the streams with the given configuration.
    /// @param config The configuration to set up the streams with.
    /// @return The status of the operation.
    OpStatus StreamSetup(const StreamConfig& config);

    /// @brief Starts all of the aggregated streams.
    void StreamStart();

    /// @brief Ends all of the aggregated streams.
    void StreamStop();

    /// @copydoc TRXLooper::StreamRx()
    /// @tparam T The type of streams to send.
    template<class T> uint32_t StreamRx(T** samples, uint32_t count, StreamMeta* meta);

    /// @copydoc TRXLooper::StreamTx()
    /// @tparam T The type of streams to receive.
    template<class T> uint32_t StreamTx(const T* const* samples, uint32_t count, const StreamMeta* meta);

  private:
    std::vector<StreamConfig> SplitAggregateStreamSetup(const StreamConfig& cfg);
    std::vector<StreamAggregate> mAggregate;
    std::vector<StreamAggregate> mActiveAggregates;
};

} // namespace lime
