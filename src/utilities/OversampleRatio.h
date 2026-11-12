#ifndef LIME_OVERSAMPLE_RATIO_H
#define LIME_OVERSAMPLE_RATIO_H

#include <cstdint>

namespace lime {

/// @brief Converts an integer oversampling ratio to the exponent of the largest power of 2 not greater than it.
/// @param ratio The requested oversampling ratio, must be at least 1.
/// @return The exponent n = floor(log2(ratio)).
inline uint32_t OversampleRatioToPow2(uint32_t ratio)
{
    uint32_t exponent = 0;
    while (ratio > 1)
    {
        ratio >>= 1;
        ++exponent;
    }
    return exponent;
}

} // namespace lime

#endif // LIME_OVERSAMPLE_RATIO_H
