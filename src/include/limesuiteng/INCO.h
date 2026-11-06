#ifndef LIMESUITENG_INCO_H
#define LIMESUITENG_INCO_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

#include <cstdint>

namespace lime {

/// @brief Numerically controlled oscillator (digital mixer) of a single RF SoC channel.
///
/// The NCO holds a table of frequency/phase presets, one of which is active at a time.
class INCO
{
  public:
    virtual ~INCO() = default;

    /// @return The direction of the signal path this NCO is in.
    virtual TRXDir GetDirection() const = 0;

    /// @brief Enables the NCO, or bypasses it when disabled.
    virtual OpStatus Enable(bool enable) = 0;
    virtual TriState IsEnabled() = 0;

    /// @return The number of frequency/phase presets.
    virtual uint8_t GetPresetCount() const = 0;
    /// @brief Selects which preset is used for mixing.
    virtual OpStatus SelectPreset(uint8_t index) = 0;
    virtual uint8_t GetSelectedPreset() = 0;

    /// @return The frequency range a preset can be set to, depends on the current sample rate.
    virtual Range<double> GetFrequencyRange() = 0;
    virtual OpStatus SetFrequency(uint8_t index, double frequency_Hz) = 0;
    virtual double GetFrequency(uint8_t index) = 0;

    virtual OpStatus SetPhaseOffset(uint8_t index, double phase_deg) = 0;
    virtual double GetPhaseOffset(uint8_t index) = 0;
};

} // namespace lime

#endif
