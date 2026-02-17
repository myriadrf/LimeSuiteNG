#ifndef LIME_IDCCORRECTOR_H
#define LIME_IDCCORRECTOR_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/types.h"

namespace lime {

class IDCCorrector
{
  public:
    enum class Type { Analog, Digital };

    virtual ~IDCCorrector(){};
    virtual OpStatus Enabled(bool enable) = 0;

    virtual OpStatus SetDCOffset(complex16_t offset) = 0;
    virtual OpStatus SetDCI(int16_t offset) = 0;
    virtual OpStatus SetDCQ(int16_t offset) = 0;
    virtual complex16_t GetDCOffset() = 0;

    virtual lime::Range<float> GetRange() = 0;
    virtual Type GetType() const = 0;
};

} // namespace lime

#endif