#ifndef LIMESUITENG_ITONEGENERATOR_H
#define LIMESUITENG_ITONEGENERATOR_H

#include "limesuiteng/OpStatus.h"

namespace lime {

class IToneGenerator
{
  public:
    virtual ~IToneGenerator(){};

    virtual OpStatus Enabled(bool enabled) = 0;

    // @brief Generate specific frequency tone
    // virtual OpStatus GenerateSpecificFrequency(double freqHz, double amplitude) = 0;

    // @brief Generate tone
    // virtual OpStatus GenerateRelativeFrequency(double freqHz, double amplitude) = 0;
    virtual OpStatus SetAmplitude(double amplitude) = 0;
    virtual OpStatus SetFrequency(double freq_weight) = 0;
};

} // namespace lime

#endif