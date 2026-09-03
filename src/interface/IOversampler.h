#ifndef LIME_IOVERSAMPLER_H
#define LIME_IOVERSAMPLER_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/types.h"

namespace lime {

class IOversampler
{
  public:
    virtual ~IOversampler(){};

    virtual OpStatus SetOversample(uint32_t oversample_pow2) = 0;
    virtual uint32_t GetOversample() = 0;
};

} // namespace lime

#endif // LIME_IOVERSAMPLER_H
