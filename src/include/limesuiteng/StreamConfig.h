#ifndef LIMESUITENG_SDRSTREAMCONFIG_H
#define LIMESUITENG_SDRSTREAMCONFIG_H

#include "limesuiteng/config.h"
#include "limesuiteng/types.h"

namespace lime {

/// @brief Structure for holding the statistics of a stream
struct StreamStats {
    /// @brief Structure for storing the first in first out queue statistics
    struct FIFOStats {
        std::size_t totalCount; ///< The total amount of samples that can be in the FIFO queue.
        std::size_t usedCount; ///< The amount of samples that is currently in the FIFO queue.

        /// @brief Gets the ratio of the amount of FIFO filled up.
        /// @return The amount of FIFO filled up (0 - completely empty, 1 - completely full).
        constexpr float ratio() const { return static_cast<float>(usedCount) / totalCount; }
    };

    StreamStats() { std::memset(this, 0, sizeof(StreamStats)); }
    uint64_t timestamp; ///< The current timestamp of the stream.
    int64_t bytesTransferred; ///< The total amount of bytes transferred.
    int64_t packets; ///< The total amount of packets transferred.
    FIFOStats FIFO; ///< The status of the FIFO queue.
    float dataRate_Bps; ///< The current data transmission rate.
    uint32_t overrun; ///< The amount of packets overrun.
    uint32_t underrun; ///< The amount of packets underrun.
    uint32_t loss; ///< The amount of packets that are lost.
    uint32_t late; ///< The amount of packets that arrived late for transmitting and were dropped.
};

/// @brief Configuration settings for a stream.
struct LIME_API StreamConfig {
    /// @brief Extra configuration settings for a stream.
    struct Extras {
        /// @brief The settings structure for a packet transmission.
        struct PacketTransmission {
            PacketTransmission();

            uint16_t samplesInPacket; ///< The amount of samples to transfer in a single packet.
            uint32_t packetsInBatch; ///< The amount of packets to send in a single transfer.
        };

        Extras();
        bool usePoll; ///< Whether to use a polling strategy for PCIe devices.

        PacketTransmission rx; ///< Configuration of the receive transfer direction.
        PacketTransmission tx; ///< Configuration of the transmit transfer direction.

        bool negateQ; ///< Whether to negate the Q element before sending the data or not.
        bool waitPPS; ///< Start sampling from next following PPS.
    };

    /// @brief The definition of the function that gets called whenever a stream status changes.
    typedef bool (*StatusCallbackFunc)(bool isTx, const StreamStats* stats, void* userData);

    StreamConfig();

    std::unordered_map<TRXDir, std::vector<uint8_t>> channels; ///< The channels to set up for the stream.

    DataFormat format; ///< Samples format used for Read/Write functions
    DataFormat linkFormat; ///< Samples format used in transport layer Host<->FPGA

    /// @brief Memory size to allocate for each channel buffering.
    /// Default: 0 - allow to decide internally.
    uint32_t bufferSize;

    /// Optional: expected sampling rate for data transfer optimizations (in Hz).
    /// Default: 0 - decide internally.
    float hintSampleRate;
    bool alignPhase; ///< Attempt to do phases alignment between paired channels

    StatusCallbackFunc statusCallback; ///< Function to call on a status change.
    void* userData; ///<  Data that will be supplied to statusCallback
    // TODO: callback for drops and errors

    Extras extraConfig; ///< Extra stream configuration settings.
};

/// @brief The metadata of a stream packet.
struct StreamMeta {
    /**
     * Timestamp is a value of HW counter with a tick based on sample rate.
     * In RX: time when the first sample in the returned buffer was received.
     * In TX: time when the first sample in the submitted buffer should be send.
     */
    uint64_t timestamp;

    /**
     * In RX: not used/ignored.
     * In TX: wait for the specified HW timestamp before broadcasting data over the air.
     */
    bool waitForTimestamp;

    /**
     * In RX: not used/ignored.
     * In TX: send samples to HW even if packet is not completely filled (end TX burst).
     */
    bool flushPartialPacket;
};

} // namespace lime
#endif
