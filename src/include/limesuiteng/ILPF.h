#ifndef LIMESUITENG_ILPF_H
#define LIMESUITENG_ILPF_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

namespace lime {

/// @brief Analog low pass filter of a single RF SoC channel.
class ILPF
{
  public:
    virtual ~ILPF() = default;

    /// @return The direction of the signal path this filter is in.
    virtual TRXDir GetDirection() const = 0;

    /// @brief Configures the filter for the given RF bandwidth.
    /// @param bandwidth_Hz The RF bandwidth in Hz.
    virtual OpStatus SetBandwidth(double bandwidth_Hz) = 0;
    /// @return The RF bandwidth in Hz, 0 if it is not known.
    virtual double GetBandwidth() = 0;
    /// @return The RF bandwidth range the filter can be configured to.
    virtual Range<double> GetBandwidthRange() const = 0;
};

} // namespace lime

#endif
