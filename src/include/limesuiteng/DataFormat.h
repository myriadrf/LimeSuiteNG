#ifndef LIMESUITENG_DATAFORMAT_H
#define LIMESUITENG_DATAFORMAT_H

#include <cstdint>

namespace lime {

/// @brief Enumerator describing the data formats.
enum class DataFormat : uint8_t {
    I16, ///< 16-bit integers.
    I12, ///< 12-bit integers. Stored as int16_t, but the expected range is [-2048;2047]
    F32, ///< 32-bit floating-point.
};

} // namespace lime
#endif //LIMESUITENG_DATAFORMAT_H
