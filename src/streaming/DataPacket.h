#ifndef LIME_DATAPACKET_H
#define LIME_DATAPACKET_H

#include "BufferInterleaving.h"

#include <cassert>
#include <cstddef>
#include <cstring>

namespace lime {

/// @brief The structure of an incoming samples packet
struct FPGA_RxDataPacket {
    FPGA_RxDataPacket()
    {
        assert(sizeof(FPGA_RxDataPacket) - sizeof(data) == 16);
        std::memset(this, 0, sizeof(FPGA_RxDataPacket));
    }

    /// @brief Checks if a Tx packet was dropped
    /// @return Whether a Tx packet was dropped or not
    inline bool txWasDropped() const { return header0 & (1 << 3); }

    /// @brief Gets the fill amount of the FIFO queue.
    /// @return The amount of FIFO buffer filled. Range: [0:1] with 0 being empty.
    inline float RxFIFOFill() const { return (header0 & 0x7) * 0.125; }

    /// @brief Gets the size of the payload in this packet.
    /// @return The size of the payload.
    inline uint16_t GetPayloadSize() const
    {
        uint16_t payloadSize = (payloadSizeMSB << 8) | payloadSizeLSB;
        assert(payloadSize <= sizeof(data));
        return payloadSize;
    }

    // Order matters
    uint8_t header0; ///< @copydoc StreamHeader::header0
    // Payload size specifies how many bytes are valid samples data, 0 - full packet is valid
    uint8_t payloadSizeLSB; ///< @copydoc StreamHeader::payloadSizeLSB
    uint8_t payloadSizeMSB; ///< @copydoc StreamHeader::payloadSizeMSB
    uint8_t reserved[5]; ///< @copydoc StreamHeader::reserved

    // Should be unsigned, but that's prone to underflow during arithmetic and would choke FPGA, packets would not be sent
    int64_t counter; ///< @copydoc StreamHeader::counter

    uint8_t data[4080]; ///< The samples data of the packet.
};

/// @brief The structure of an outgoing samples packet
struct FPGA_TxDataPacket {
    FPGA_TxDataPacket()
    {
        assert(sizeof(FPGA_TxDataPacket) - sizeof(data) == 16);
        std::memset(this, 0, sizeof(FPGA_TxDataPacket));
    }

    /// @copydoc StreamHeader::ignoreTimestamp()
    inline void ignoreTimestamp(bool enabled)
    {
        constexpr uint8_t mask = 1 << 4;
        header0 &= ~mask; //clear ignore timestamp
        header0 |= enabled ? mask : 0; //ignore timestamp
    }

    /// @copydoc StreamHeader::getIgnoreTimestamp()
    inline bool getIgnoreTimestamp() const
    {
        constexpr uint8_t mask = 1 << 4;
        return header0 & mask; //ignore timestamp
    }

    /// @copydoc StreamHeader::Clear()
    inline void ClearHeader() { std::memset(this, 0, 16); }

    /// @copydoc StreamHeader::SetPayloadSize()
    inline void SetPayloadSize(uint16_t size)
    {
        payloadSizeLSB = size & 0xFF;
        payloadSizeMSB = (size >> 8) & 0xFF;
    }

    /// @copydoc StreamHeader::GetPayloadSize()
    inline uint16_t GetPayloadSize() const { return (payloadSizeMSB << 8) | payloadSizeLSB; }

    // Order matters
    uint8_t header0; ///< @copydoc StreamHeader::header0
    // Payload size specifies how many bytes are valid samples data, 0 - full packet is valid
    uint8_t payloadSizeLSB; ///< @copydoc StreamHeader::payloadSizeLSB
    uint8_t payloadSizeMSB; ///< @copydoc StreamHeader::payloadSizeMSB
    uint8_t reserved[5]; ///< @copydoc StreamHeader::reserved

    // Should be unsigned, but that's prone to underflow during arithmetic and would choke FPGA, packets would not be sent
    int64_t counter; ///< @copydoc StreamHeader::counter
    uint8_t data[4080]; ///< The samples data of the packet.
};

static_assert(sizeof(FPGA_TxDataPacket) == 4096);
static_assert(sizeof(FPGA_TxDataPacket) == sizeof(FPGA_RxDataPacket));

/** @brief Data structure used for interacting with the header of sample stream packets. */
struct StreamHeader {
    StreamHeader() { Clear(); }

    /// @brief Sets whether to ignore the timestamp on this packet or not
    /// @param enabled Whether to ignore the timestamp.
    inline void ignoreTimestamp(bool enabled)
    {
        constexpr uint8_t mask = 1 << 4;
        header0 &= ~mask; //clear ignore timestamp
        header0 |= enabled ? mask : 0; //ignore timestamp
    }

    /// @brief Gets whether the ignore timestamp flag is set
    /// @return The current state of the timestamp flag
    inline bool getIgnoreTimestamp() const
    {
        constexpr uint8_t mask = 1 << 4;
        return header0 & mask; //ignore timestamp
    }

    /// @brief Clears all the data in the stream header
    inline void Clear() { std::memset(this, 0, sizeof(StreamHeader)); }

    /// @brief Sets the size of the payload
    /// @param size The new size of the payload
    inline void SetPayloadSize(uint16_t size)
    {
        payloadSizeLSB = size & 0xFF;
        payloadSizeMSB = (size >> 8) & 0xFF;
    }

    /// @brief Gets the current size of the payload
    /// @return The current size of the payload
    inline uint16_t GetPayloadSize() const { return (payloadSizeMSB << 8) | payloadSizeLSB; }

    // Order matters
    uint8_t header0; ///< The flags byte of the packet.
    // Payload size specifies how many bytes are valid samples data, 0 - full packet is valid (4080 bytes payload)
    uint8_t payloadSizeLSB; ///< The least significant byte of the payload size.
    uint8_t payloadSizeMSB; ///< The most significant byte of the payload size.
    uint8_t reserved[5]; ///< Currently unused bytes.

    // Should be unsigned, but that's prone to underflow during arithmetic and would choke FPGA, packets would not be sent
    int64_t counter; ///< The amount of samples sent in total.
};

static_assert(sizeof(StreamHeader) == 16);
static_assert(sizeof(FPGA_RxDataPacket::data) == sizeof(FPGA_RxDataPacket) - sizeof(StreamHeader));
static_assert(sizeof(FPGA_TxDataPacket::data) == sizeof(FPGA_TxDataPacket) - sizeof(StreamHeader));

} // namespace lime

#endif // LIME_DATAPACKET_H
