#ifndef LIMESUITENG_ILOMIXER_H
#define LIMESUITENG_ILOMIXER_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

namespace lime {

/// @brief Local oscillator and RF mixer, converting between RF and baseband.
///
/// A local oscillator can be shared by several channels of an RF SoC.
class ILOMixer
{
  public:
    virtual ~ILOMixer() = default;

    /// @return The direction of the signal path this mixer is in.
    virtual TRXDir GetDirection() const = 0;

    /// @return The frequency range the local oscillator can be tuned to.
    virtual Range<double> GetFrequencyRange() const = 0;
    virtual OpStatus SetFrequency(double frequency_Hz) = 0;
    virtual double GetFrequency() = 0;
    /// @return Whether the local oscillator's PLL is locked.
    virtual bool IsLocked() = 0;
};

} // namespace lime

#endif
