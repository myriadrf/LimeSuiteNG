#pragma once

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

/// @brief PPS disciplined oscillator: tunes the board's reference oscillator to an external PPS signal
///
/// The regulation loop counts clock ticks over 1s, 10s and 100s windows of PPS pulses
/// and adjusts the oscillator's trim DAC until the counts are within tolerance of their targets.
class LIME_API PPSDO
{
  public:
    virtual ~PPSDO(){};

    struct Config {
        /// Frequency of the clock counted between PPS pulses, in Hz.
        /// 0 lets the device use its own, devices that measure an internal clock known to them ignore it.
        double clockFrequency{ 0 };
        /// Frequency error tolerated by each measurement window, in ppm. 0 selects the device default, if it has one.
        double tolerancePPM{ 0 };
    };
    /// @brief Configures the regulation loop, does not change whether it is enabled.
    virtual OpStatus Configure(const Config& config) = 0;

    /// @brief Starts or stops disciplining the oscillator.
    virtual OpStatus Enable(bool enable) = 0;

    enum class TuneState { Idle, CoarseTune, FineTune, Unknown };

    struct Status {
        uint32_t dacValue{ 0 }; ///< Oscillator trim DAC value.
        // Error of the last completed measurement window, in clock ticks.
        int32_t error1s{ 0 };
        int32_t error10s{ 0 };
        int32_t error100s{ 0 };

        TuneState state{ TuneState::Unknown };
        uint8_t accuracy{ 0 }; ///< Regulation accuracy, from 0 (lowest) to 3 (1s, 10s and 100s windows are all within tolerance).
        bool ppsActive{ false }; ///< PPS pulses are being received.
        bool enabled{ false };
    };
    virtual OpStatus GetStatus(Status& status) = 0;
};

} // namespace lime
