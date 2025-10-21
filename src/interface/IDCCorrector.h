#ifndef LIME_IDCCORRECTOR_H
#define LIME_IDCCORRECTOR_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/complex.h"

namespace lime {

class IDCCorrector
{
  public:
    virtual ~IDCCorrector(){};

    virtual OpStatus SetDCOffset(complex16_t offset) = 0;
    virtual OpStatus SetDCI(int16_t offset) = 0;
    virtual OpStatus SetDCQ(int16_t offset) = 0;
    virtual complex16_t GetDCOffset() = 0;
};

} // namespace lime

#endif