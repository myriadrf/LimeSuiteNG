#ifndef LIME_ITONE_GENERATOR_H
#define LIME_ITONE_GENERATOR_H

#include "limesuiteng/OpStatus.h"

namespace lime {

class IToneGenerator
{
  public:
    virtual ~IToneGenerator(){};

    virtual OpStatus Enabled(bool enabled);

    // @brief Generate specific frequency tone
    virtual OpStatus GenerateSpecificFrequency(double freqHz, double amplitude) = 0;

    // @brief Generate tone
    virtual OpStatus GenerateRelativeFrequency(double freqHz, double amplitude) = 0;
};

} // namespace lime

#endif