#ifndef LIMESUITENG_IOVERSAMPLER_H
#define LIMESUITENG_IOVERSAMPLER_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/types.h"

namespace lime {

class IOversampler
{
  public:
    virtual ~IOversampler(){};

    /// @brief Sets the oversampling ratio.
    /// @param ratio The oversampling ratio, 1 means no oversampling (bypass), 0 is invalid.
    /// Devices that support only power of 2 ratios round the ratio down to a power of 2.
    /// Use GetOversample() to read back the ratio actually applied.
    /// @return OpStatus::OutOfRange if ratio is greater than GetMaxOversample().
    virtual OpStatus SetOversample(uint32_t ratio) = 0;
    /// @return the current oversampling ratio, 1 means no oversampling
    virtual uint32_t GetOversample() = 0;
    /// @return the largest oversampling ratio the device supports
    virtual uint32_t GetMaxOversample() = 0;
};

} // namespace lime

#endif // LIMESUITENG_IOVERSAMPLER_H
