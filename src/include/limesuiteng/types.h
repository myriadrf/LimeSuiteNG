#ifndef LIMESUITENG_TYPES_H
#define LIMESUITENG_TYPES_H

#include "limesuiteng/config.h"
#include <cstdint>

namespace lime {

/// @brief The direction of the transmission
enum class TRXDir : bool { Rx, Tx };

/// @brief Enumerator describing the data formats.
enum class DataFormat : uint8_t {
    I16, ///< 16-bit integers.
    I12, ///< 12-bit integers. Stored as int16_t, but the expected range is [-2048;2047]
    F32, ///< 32-bit floating-point.
};

/// @brief Structure describing the range possible.
struct Range {
    /// @brief Constructs the range structure,
    /// @param min The minimum value of the range (default 0.0)
    /// @param max The maximum value of the range (default 0.0)
    /// @param step The step of the range (default 0.0 - no step)
    constexpr Range(double min = 0.0, double max = 0.0, double step = 0.0)
        : min(min)
        , max(max)
        , step(step){};
    double min; ///< The minimum value of the range
    double max; ///< The maximum value of the range
    double step; ///< The step of the range (or 0.0 for any step)
};

} // namespace lime

#endif
