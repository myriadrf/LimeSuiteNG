#ifndef LIME_BUFFER_INTERLEAVING_H
#define LIME_BUFFER_INTERLEAVING_H

#include "limesuiteng/types.h"

namespace lime {

/// @brief Structure defining how to convert the samples data.
struct DataConversion {
    DataFormat srcFormat; ///< The format to convert from.
    DataFormat destFormat; ///< The format to convert to.
    uint8_t channelCount; ///< The amount of channels the data has.
};

int Deinterleave(void* const* dest, const uint8_t* buffer, uint32_t length, const DataConversion& fmt);
int Interleave(uint8_t* dest, const void* const* src, uint32_t count, const DataConversion& fmt);

} // namespace lime

#endif