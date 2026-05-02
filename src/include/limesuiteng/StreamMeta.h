#ifndef LIMESUITENG_STREAMMETA_H
#define LIMESUITENG_STREAMMETA_H

/**
@file limesuiteng/StreamMeta.h
@author Lime Microsystems
@brief Defines structures for Stream TX and RX meta data collection
*/

#include "limesuiteng/Timespec.h"

namespace lime {

/// @brief Transmit stream meta data storage class.
class StreamTxMeta
{
  public:
    /// @brief Transmit stream flag options.
    enum Flags {
        EndOfBurst = (1 << 0),
        StartOfBurst = (1 << 1),
    };
    lime::Timespec timestamp; ///< Latest transmit stream timestamp.
    bool hasTimestamp; ///< Timestamp availability status in transmit stream.

    uint32_t flags; ///< Transmit stream flags status.
};

/// @brief Receive stream meta data storage class.
class StreamRxMeta
{
  public:
    lime::Timespec timestamp; ///< Latest receive stream timestamp.
    bool hasTimestamp; ///< Timestamp availability status in receive stream.
};

} // namespace lime

#endif