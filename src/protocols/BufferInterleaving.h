#ifndef LIME_BUFFER_INTERLEAVING_H
#define LIME_BUFFER_INTERLEAVING_H

#include "limesuiteng/config.h"
#include "limesuiteng/types.h"

namespace lime {

struct DataConversion {
    DataFormat srcFormat;
    DataFormat destFormat;
    uint8_t channelCount;
};

int LIME_API Deinterleave(void* const* dest, const uint8_t* buffer, uint32_t length, const DataConversion& fmt);
int LIME_API Interleave(uint8_t* dest, const void* const* src, uint32_t count, const DataConversion& fmt);

} // namespace lime

#endif