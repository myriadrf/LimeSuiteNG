#pragma once

#if __cplusplus < 201402L
    #define CPP14constexpr
#else
    #define CPP14constexpr constexpr
#endif

namespace lime {

/// @brief The direction of the transmission
enum class TRXDir : bool { Rx, Tx };

const char* ToCString(TRXDir dir);

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
