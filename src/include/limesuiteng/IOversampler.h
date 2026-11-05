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

    virtual OpStatus SetOversample(uint32_t oversample_pow2) = 0;
    /// @return the oversampling factor as a power of 2
    virtual uint32_t GetOversample() = 0;
};

} // namespace lime

#endif // LIMESUITENG_IOVERSAMPLER_H
