#ifndef LIME_BUFFER_INTERLEAVING_H
#define LIME_BUFFER_INTERLEAVING_H

#include "limesuiteng/types.h"

namespace lime {

struct DataConversion {
    DataFormat srcFormat;
    DataFormat destFormat;
    uint8_t channelCount;
};

int Deinterleave(void* const* dest, const std::byte* buffer, uint32_t length, const DataConversion& fmt);
int Interleave(std::byte* dest, const void* const* src, uint32_t count, const DataConversion& fmt);

} // namespace lime

#endif